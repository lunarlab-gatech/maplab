# Start from ROS Melodic base (Ubuntu 18.04 / Bionic)
FROM ros:melodic-ros-base-bionic
ENV DEBIAN_FRONTEND=noninteractive
# Set workspace path after defining USERNAME
ARG USERNAME=cam
ARG USER_UID=1000
ARG USER_GID=1000
ARG CATKIN_WS=/home/${USERNAME}/maplab_ws

# Create a non-root user with matching UID/GID
RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} && \
    usermod -aG sudo ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Add a custom .bashrc with warm colors and fixed hostname
RUN echo 'export PS1="\\[\\e[1;31m\\]\\u@maplab_container\\[\\e[0m\\]:\\[\\e[1;33m\\]\\w\\[\\e[0m\\]\\$ "' > /home/${USERNAME}/.bashrc && \
    chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.bashrc

# Set default user
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Core build tools + dependencies
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    build-essential \
    git \
    curl \
    wget \
    python-pip \
    python-setuptools \
    python-catkin-tools \
    python-git \
    python-termcolor \
    python-wstool \
    python-autopep8 \
    pylint \
    clang-format-6.0 \
    autotools-dev \
    ccache \
    doxygen \
    dh-autoreconf \
    liblapack-dev \
    libblas-dev \
    libgtest-dev \
    libreadline-dev \
    libssh2-1-dev \
    libatlas3-base \
    libv4l-dev \
    && sudo rm -rf /var/lib/apt/lists/* \
    && sudo pip install requests

# Initialize rosdep
RUN sudo rosdep update

# Source ROS on container start
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# Enable ccache
RUN echo 'export PATH="/usr/lib/ccache:$PATH"' >> ~/.bashrc \
    && echo 'ccache --max-size=20G' >> ~/.bashrc

# Set the User environment variables
RUN echo "export USER=${USERNAME}" >> ~/.bashrc

# Create catkin workspace
RUN sudo mkdir -p ${CATKIN_WS}/src && \
    sudo chown -R ${USERNAME}:${USERNAME} ${CATKIN_WS}

# Initialize catkin workspace (as user, not sudo)
RUN cd ${CATKIN_WS} && \
    catkin init && \
    catkin config --merge-devel && \
    catkin config --extend /opt/ros/melodic && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

# More verbose build output
RUN cd ${CATKIN_WS} && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS=-fdiagnostics-color

# Force GitHub SSH -> HTTPS (for submodules)
RUN git config --global url."https://github.com/".insteadOf git@github.com:

# Clone maplab with submodules
RUN cd ${CATKIN_WS}/src && \
    git clone --recursive https://github.com/ethz-asl/maplab.git -b develop

# Setup linter hooks (optional, if contributing)
RUN cd ${CATKIN_WS}/src/maplab && \
    ./dependencies/internal/linter/init-git-hooks.py || true

# Setup other building dependencies
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    libeigen3-dev \
    libyaml-cpp-dev \
    ros-melodic-eigen-conversions \
    ros-melodic-tf \
    ros-melodic-tf-conversions \
    ros-melodic-rviz \
    qt5-default \
    qttools5-dev \
    qttools5-dev-tools \
    libqt5core5a \
    libqt5gui5 \
    libqt5widgets5 \
    ros-melodic-pcl-conversions \
    ros-melodic-pcl-ros \
    && sudo rm -rf /var/lib/apt/lists/*

# STOPPING POINT:
# Next step would be: (change the number based on your device's computation power)
RUN cd ${CATKIN_WS} && catkin build maplab -j1 -l1