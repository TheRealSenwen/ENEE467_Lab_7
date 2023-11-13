FROM osrf/ros:noetic-desktop-full

# Set default shell
SHELL ["/bin/bash", "-c"]

ARG USER=user
ARG UID=1000
ARG GID=1000
ARG PW=user@123

RUN groupadd -g ${GID} -o ${USER}
RUN useradd --system --create-home --home-dir /home/${USER} --shell /bin/bash --uid ${UID} -g ${GID} --groups sudo,video ${USER} && \ 
    echo "${USER}:${PW}" | chpasswd && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

ENV QT_X11_NO_MITSHM=1 \
    USER=${USER} \
    LANG=en_US.UTF-8 \
    HOME=/home/${USER} \
    XDG_RUNTIME_DIR=/run/user/${UID} \
    TZ=America/New_York 


USER ${USER}
WORKDIR ${HOME}
# custom Bash prompt
RUN { echo && echo "PS1='\[\e]0;\u \w\a\]\[\033[01;32m\]\u\[\033[00m\] \[\033[01;34m\]\w\[\033[00m\] \\\$ '" ; } >> .bashrc

RUN sudo mkdir -p -m 0700 /run/user/${UID} && \
    sudo chown ${USER}:${USER} /run/user/${UID}

# Basic setup
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends --allow-unauthenticated \
    software-properties-common \
    build-essential \
    bash-completion \
    g++ \
    git \
    ca-certificates \
    make \
    automake \
    autoconf \
    libtool \
    pkg-config \
    python \
    libxext-dev \
    libx11-dev \
    doxygen \
    tmux \
    tzdata \
    xclip \
    mc \
    curl \
    iproute2 \
    iputils-ping \
    cmake \
    x11proto-gl-dev && \
    sudo rm -rf /var/lib/apt/lists/*

# Setup tmux config
ADD --chown=${USER}:${USER} https://raw.githubusercontent.com/kanishkaganguly/dotfiles/master/tmux/.tmux.bash.conf $HOME/.tmux.conf

# Set datetime and timezone correctly
RUN sudo ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo '$TZ' | sudo tee -a /etc/timezone

# Install ROS packages
RUN sudo apt-get update && sudo apt-get install -y \
    python3-catkin-tools \
    ros-noetic-moveit-visual-tools \
    ros-noetic-moveit-* \
    ros-noetic-usb-cam \
    ros-noetic-fiducial-msgs \
    ros-noetic-aruco-detect \
    # ros-melodic-gripper-action-controller \
    libeigen3-dev && \
    sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

# Setup UR Drivers
RUN source /opt/ros/noetic/setup.bash && \
	mkdir -p $HOME/ros_ur_driver/src && \
    cd ~/ros_ur_driver && \
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver && \
    git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git src/universal_robot && \
    sudo apt update -qq && \
    rosdep update --include-eol-distros && \
    rosdep install --from-paths src --ignore-src -y && \
    catkin build && \
    sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

# Setup ROS workspace directory
RUN source /opt/ros/noetic/setup.bash && \
	mkdir -p $HOME/catkin_ws/src && \
    catkin init --workspace $HOME/catkin_ws/

# Set up working directory and bashrc
WORKDIR ${HOME}/catkin_ws/
RUN echo 'source /opt/ros/noetic/setup.bash' >> $HOME/.bashrc && \
	echo 'source $HOME/ros_ur_driver/devel/setup.bash' >> $HOME/.bashrc && \
	echo 'source $HOME/catkin_ws/devel/setup.bash' >> $HOME/.bashrc
    
CMD /bin/bash
