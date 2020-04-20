# Use ROS Melodic Docker distribution (Ubuntu 18.04 = Bionic)
FROM ros:melodic

# Install all necessary libraries/packages
# Start with update
RUN apt update && \
  apt install -y \
    python-catkin-tools \
    ros-melodic-joy \
    ros-melodic-tf2-ros \
    ros-melodic-tf2-eigen \
    libeigen3-dev \
    ros-melodic-gazebo-msgs \
    && \
  rm -rf /var/lib/apt/lists/*

# Add user user
RUN useradd -ms /bin/bash user -G video
# Temporary user settings to help build Docker image quicker
RUN  echo user:kdf | chpasswd && \
  adduser user sudo
USER user

# Make the ros workspace
RUN mkdir -p /home/user/catkin_ws/src

WORKDIR /home/user/catkin_ws/

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

# nvidia-docker hooks
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}
