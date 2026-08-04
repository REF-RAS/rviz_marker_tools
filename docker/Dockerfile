# Copyright 2024 - Andrew Kwok Fai LUI, Robotics and Autonomous System Group,
# Research Engineering Facility, RI
# the Queensland University of Technology

ARG ROS_DISTRO=noetic
FROM osrf/ros:${ROS_DISTRO}-desktop-full
LABEL author "Andrew Lui <luia2@qut.edu.au>"
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# install tools
RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y \
sudo \
vim \
git \
net-tools \
iputils-ping \
locales \
wget \
curl \
dbus \
htop \
build-essential \
cmake \
gdb  \
lsb-release \
openssh-server \
ca-certificates

# install python packages
RUN apt-get update && apt-get install -y software-properties-common
RUN add-apt-repository ppa:deadsnakes/ppa
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
# python3.12 \
python3-pip \
python3-venv \
python3-rosdep

# alternatively add flag --break-system-packages
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install --only-upgrade python3-pip

# link python3.11 to python
RUN ln -s /usr/bin/python3 /usr/bin/python

# clean the apt cache
RUN apt-get clean
RUN rm -rf /var/lib/apt/lists/*

# set locales
RUN locale-gen en_US.UTF-8  
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8  
ENV LANGUAGE en_US:en  
ENV LC_ALL en_US.UTF-8

# input argument from the build command
ARG HOST_UID
ARG HOST_GID
ARG USER=ubuntu
ARG HOME=/workspace

# add the user and group using the input UID and GID: enable files and folders generated are consistent with that in the host computer
RUN echo "${USER} ${HOST_UID} ${HOST_GID} "
RUN if [ -n "$HOST_GID" ] && [ -n "$HOST_UID" ]; then \
        if [ "$USER" = "root" ]; then \
            groupmod -g ${HOST_GID} root; \
            usermod -g root -d ${HOME} root; \
        else \
            groupadd -g ${HOST_GID} ${USER};  \
            useradd -u ${HOST_UID} -g ${USER} -d ${HOME} -m ${USER}; \
        fi \
    else \
        if [ "$USER" = "root" ]; then \
            groupmod root; \
            usermod -g root -d ${HOME} root; \
        else \
            groupadd ${USER};  \
            useradd -g ${USER} -d ${HOME} -m ${USER}; \
        fi \
    fi


# add the USER to the sudoers 
RUN echo "${USER} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# create a working directory: for mounting source code and data volumes
RUN mkdir -p ${HOME}/.cache ${HOME}/.local
RUN chown -R ${USER}:${USER} ${HOME}
WORKDIR ${HOME} 

# install the python modules specified
# remove permanantly the lock for systen managed package bypass PEP 668 restriction
# RUN rm /usr/lib/python*/EXTERNALLY-MANAGED  

# pull the official pre-compiled uv binary
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/
# set environment variable for uv inside docker

# change to user for the rest of the operations
USER ${USER}

# complete rosdep installation
RUN rosdep update && rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths .

# install python modules using uv
ENV UV_SYSTEM_PYTHON=1
ENV UV_COMPILE_BYTECODE=1
COPY assets/requirements.txt /tmp/requirements.txt
COPY assets/requirements_dev.txt /tmp/requirements_dev.txt
# RUN pip install -r /workspace/requirements.txt --no-warn-script-location --no-cache-dir 
RUN sudo uv pip install -r /tmp/requirements.txt --system
RUN sudo uv pip install -r /tmp/requirements_dev.txt --system
RUN sudo rm /tmp/requirements*.txt

# create catkin_ws
ENV CATKIN_WS=${HOME}/catkin_ws
RUN mkdir -p ${CATKIN_WS}/src
RUN chown -R ${USER}:${USER} ${CATKIN_WS}/src

# modify the login script
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# copy and define entry script
ARG ENTRYPOINT_FILE=/usr/local/bin/ros_entrypoint
COPY assets/entrypoint_setup.sh ${ENTRYPOINT_FILE}
RUN sudo chmod +x ${ENTRYPOINT_FILE}
ENTRYPOINT ["/usr/local/bin/ros_entrypoint"]
CMD ["bash"]

