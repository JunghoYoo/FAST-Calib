FROM osrf/ros:noetic-desktop-full

# revised on Feb 1st 2026

##############################################################################
# RealSense SDK and ROS wrapper version 
# Release note Check https://github.com/realsenseai/realsense-ros/releases/tag/2.3.2
#2.3.2 = latest stable for ROS Noetic as of Feb 1st 2026
ENV LIBREALSENSE_VERSION=2.50.0
ENV REALSENSEROS_VERSION=2.3.2
##############################################################################

# Fix for X11 shared memory issues in Docker
ENV QT_X11_NO_MITSHM=1

RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Dependencies we use, catkin tools is very good build system
# https://github.com/ethz-asl/kalibr/wiki/installation
RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
	apt-get install -y \
	git wget curl autoconf automake nano \
	python3-dev python3-pip python3-scipy python3-matplotlib \
	ipython3 python3-wxgtk4.0 python3-tk python3-igraph python3-pyx \
	libeigen3-dev libboost-all-dev libsuitesparse-dev \
	doxygen \
	libopencv-dev \
	libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev \
	python3-catkin-tools python3-osrf-pycommon \
	software-properties-common

# Open3D is a Python package (not in apt); install via pip
RUN pip3 install --no-cache-dir open3d

# Install dependencies for building librealsense from source
RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
	apt-get install -y \
	libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev \
	libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
	ros-noetic-ddynamic-reconfigure

# Build and install Intel RealSense SDK from source
RUN cd /tmp && \
	git clone https://github.com/IntelRealSense/librealsense.git && \
	cd librealsense && \
	git checkout v${LIBREALSENSE_VERSION} && \
	mkdir build && cd build && \
	cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false && \
	make -j$(nproc) && \
	make install && \
	ldconfig && \
	rm -rf /tmp/librealsense

# Install original Livox SDK
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git /tmp/Livox-SDK && \
    cd /tmp/Livox-SDK && mkdir -p build && cd build && \
    cmake .. -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/Livox-SDK

# Create the workspace and build kalibr in it
ENV WORKSPACE=/catkin_ws

RUN mkdir -p $WORKSPACE/src && \
	cd $WORKSPACE && \
	catkin init && \
	catkin config --extend /opt/ros/noetic && \
	catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

# Copy kalibr source code into workspace
#ADD . $WORKSPACE/src/kalibr
RUN cd $WORKSPACE/src && \
	git clone https://github.com/ethz-asl/kalibr.git

# Clone RealSense ROS wrapper (version matched with SDK)
RUN cd $WORKSPACE/src && \
	git clone https://github.com/IntelRealSense/realsense-ros.git && \
	cd realsense-ros && \
	git checkout ${REALSENSEROS_VERSION}

# Clone allan_variance_ros package for IMU analysis
RUN cd $WORKSPACE/src && \
	git clone https://github.com/ori-drs/allan_variance_ros.git && \
	cd allan_variance_ros 

# Clone Livox ROS driver for Livox LiDAR support
RUN cd $WORKSPACE/src && \
    git clone https://github.com/Livox-SDK/livox_ros_driver.git 

# Clone Fast-Calib package for lidar-camera calibration
RUN cd $WORKSPACE/src && \
	git clone https://github.com/hku-mars/FAST-Calib.git

RUN	cd $WORKSPACE &&\
	catkin build -j$(nproc)

# When a user runs a command we will run this code before theirs
# This will allow for using the manual focal length if it fails to init
# https://github.com/ethz-asl/kalibr/pull/346
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 && cd /catkin_ws && /bin/bash"]

