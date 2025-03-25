################################################################################
# There are three places where your actions are needed in this Dockerfile.
# These are marked with the label `ACTIONS_NEEDED`.
# After you are done with those:
#
# Pull the robetarme noetic base image with
# `docker login registry.gitlab.com`
# `docker pull registry.gitlab.com/certh-iti-robotics-lab/robetarme/robetarme/ros-noetic:base`
#
# Then build with
# `docker compose build`,
#
# and run with
# `docker compose up -d`.
#
# Then gain access to the container with
# `docker attach ros_noetic_xy_development_container`,
# ** where the string after "attach" is the one specified in the `docker-compose.yml` file under
# services/ros_noetic_xy_development/container_name.
#
# Then simply launch your ROS noetic packages.
################################################################################

FROM registry.gitlab.com/certh-iti-robotics-lab/robetarme/robetarme/ros2-humble:base
ARG USER=robetarme_user

# ACTIONS_NEEDED
#-------------------------------------------------------------------------------
# Install your dependencies here

### Add a few essential tools and catkin tools
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    git \
    python3 \
    openssh-client \
    net-tools \
    build-essential \
    cmake \
    wget \
    bash-completion \
    silversearcher-ag \
    apt-transport-https \
    less \
    alsa-utils \
    libcgal-dev \
    python3-pip \
    python-is-python3 \
    python3-tk

#-------------------------------------------------------------------------------
# Setup the project using automatic script
# Use a secret for the SSH key
COPY ./scripts/setupControlLasaEnv.sh /run/setupControlLasaEnv.sh
RUN chmod +x /run/setupControlLasaEnv.sh
RUN ./run/setupControlLasaEnv.sh

RUN apt-get remove --purge python3-matplotlib -y
### Add python libraries
RUN pip install --upgrade pip
RUN pip install \
    transforms3d \
    statsmodels \
    opencv-python \
    'numpy<2' \
    pandas \
    pyvista \
    opencv-contrib-python \
    numpy-quaternion \
    shapely \
    pybullet \
    trimesh \
    networkx \
    "pybind11[global]"

RUN pip install --ignore-installed open3d

### Add ros library
RUN apt update --fix-missing && apt upgrade -y && apt install -y \
    ros-${ROS_DISTRO}-ros-core \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    python3-pcl

# Install moveit tools
RUN apt update --fix-missing && apt upgrade -y && apt install -y \
    python3-vcstool \
    ros-${ROS_DISTRO}-rqt-joint-trajectory-controller \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros
    
# Install NLopt library for trac-ik
RUN apt update --fix-missing && apt upgrade -y && apt install -y \
    libnlopt-dev \
    libnlopt-cxx-dev

RUN sudo apt clean && sudo apt autoremove -y


# Install GTest library
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    libgtest-dev

# Update package list and upgrade
RUN apt-get update --fix-missing && apt-get upgrade -y && apt-get clean


#-------------------------------------------------------------------------------


# ------------------------------------------------------------------------------
# Place ROS packages into /home/${USER}/ros1_ws workspace and catkin build
# ------------------------------------------------------------------------------
# ACTIONS_NEEDED
# COPY your ROS packages from the host into the container's
# /home/${USER}/ros1_ws/src/. THIS INCLUDES CUSTOM INTERFACES.
# These packages must reside on the host alongside this Dockerfile,
# e.g. if this Dockerfile is at /path/Dockerfile then you should
# `cd /path/`
# `git clone https://github.com/ros/ros_tutorials.git`
# where you should replace the `ros_tutorials` repository with yours.
# This applies to any other directory you want access to in the container.
# The following two lines are indicative and should be replaced by you.
# CAUTION: leave trailing slashes as-is
#-------------------------------------------------------------------------------


# # ------------------------------------------------------------------------------
# # catkin build ROS noetic packages
# RUN cd /home/${USER}/catkin_ws                                              && \
#     source /opt/ros/noetic/setup.bash                                       && \
#     colcon build || true                                                    && \
#     source /home/${USER}/catkin_ws/devel/setup.bash
# # ------------------------------------------------------------------------------


#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
# Do not modify below this line
#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

# ------------------------------------------------------------------------------
# Bind mount/shared directory between container and host
# https://docs.docker.com/storage/bind-mounts/
RUN mkdir /home/${USER}/shared
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# Make ${USER} a homeowner
RUN chown -R robetarme_user /home/${USER}
# ------------------------------------------------------------------------------

COPY src/ src/

# Build the workspace and source the devel/setup.bash
WORKDIR /home/${USER}/ros2_ws
RUN rosdep update
RUN rosdep install --from-paths /home/${USER}/ros2_ws/src --ignore-src -r -y

RUN cd /home/${USER}/ros2_ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build && \
    source install/setup.bash
#-------------------------------------------------------------------------------
# ACTIONS_NEEDED
# Your attention is required at file
# docker-compose.yml (:services/volumes).
#-------------------------------------------------------------------------------
# `fix-perms.sh` fixes permissions for volumes when the host user uid does not
# coincide with the container user uid
#-------------------------------------------------------------------------------
COPY fix-perms.sh /fix-perms.sh
COPY entrypointd.sh /entrypoint.sh
RUN chmod +x /fix-perms.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
#-------------------------------------------------------------------------------
RUN echo "cd /home/${USER}/ros2_ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash&& \
    colcon build && \
    source install/setup.bash"  >> ${HOME}/.bashrc ;
# RUN echo "source devel/setup.bash" >> ~/.bashrc
# RUN cho "ros2 launch wp5_tasks main_taske.py" >> ~/.bashrc
# USER ${USER}

CMD ["/bin/bash"]