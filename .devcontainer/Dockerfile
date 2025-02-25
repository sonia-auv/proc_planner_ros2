FROM ros:humble-ros-base

ARG USERNAME=sonia
ARG UID=1000
ARG GID=$UID

# Install some dependencies packages
RUN apt update -q && apt upgrade -q -y \
    && apt install -y software-properties-common \ 
    python3-pip \
    python3-flake8-docstrings \
    python3-pip \
    python3-pytest-cov \
    ros-dev-tools \
    clang-format \ 
    clang-tidy

# Create and switch to user
RUN groupadd -g $GID $USERNAME \
    && useradd -lm -u $UID -g $USERNAME -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USERNAME

# Create workspace so that user own this directory
RUN mkdir -p /home/$USERNAME/ros2_sonia_ws/src
WORKDIR /home/$USERNAME/ros2_sonia_ws/src/
RUN git clone -b develop https://github.com/sonia-auv/sonia_common_ros2.git
WORKDIR /home/$USERNAME/ros2_sonia_ws/src/hardware_interface_manager

# Copy configuration files
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /home/'$USERNAME'/ros2_sonia_ws/install/setup.bash' >> /home/$USERNAME/.bashrc

# Setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
