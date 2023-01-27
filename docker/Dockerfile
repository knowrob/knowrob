FROM ubuntu:20.04

ARG HOST_IP=172.17.0.1
ARG ROS_SOURCES="http://packages.ros.org/ros/ubuntu"

ARG USE_APT_CACHE=0
ARG LOCAL_BUILD=0
ENV DEBIAN_FRONTEND=noninteractive

# Use apt-cacher container
RUN if [ "x$USE_APT_CACHE" = "x0" ] ; then \
    echo "no apt-cacher is used." ; else \
    echo 'Acquire::http { Proxy "http://172.17.42.1:3142"; };' >> /etc/apt/apt.conf.d/01proxy \
    echo 'Acquire::http { Proxy "http://172.17.0.1:3142"; };' >> /etc/apt/apt.conf.d/02proxy ; \
    fi

RUN apt-get -qq update && \
    apt-get -qq -y install wget apt-utils gnupg tzdata
RUN apt-get update && apt-get install -y locales
RUN sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
    dpkg-reconfigure --frontend=noninteractive locales && \
    update-locale LANG=en_US.UTF-8
ENV LANG en_US.UTF-8 
    
# Install ROS
RUN sh -c 'echo "deb ${ROS_SOURCES} focal main" > /etc/apt/sources.list.d/ros-latest.list'
#RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN sh -c 'wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc && apt-key add ros.asc' && \
    apt-get -qq -y update && \
    apt-get -qq -y install ros-noetic-desktop \
                           libjson-glib-dev python3-yaml python3-catkin-pkg python3-rospkg \
                           emacs ros-noetic-catkin git \
                           ros-noetic-rosbridge-suite \
                           ros-noetic-cv-bridge \
                           software-properties-common \
                           mongodb-clients libmongoc-dev  python3-pip\
                           ros-noetic-rosauth mencoder lame libavcodec-extra \
                           texlive-latex-base imagemagick python3-rdflib  && \
    apt-add-repository -y ppa:swi-prolog/stable && \
    apt-get -qq -y update && \
    apt-get -qq -y install  swi-prolog && \
    apt-get -qq -y autoremove  && \
    apt-get -qq -y clean  && \
    rm -rf /var/lib/apt/lists/*  && \
    rm -rf /tmp/*
RUN pip install -U rosdep future
# ROS environment setup
RUN cp /opt/ros/noetic/setup.sh /etc/profile.d/ros_noetic.sh && rosdep init

# Create user 'ros'
RUN useradd -m -d /home/ros -p ros ros && \
    adduser ros sudo && \
    chsh -s /bin/bash ros
ENV HOME /home/ros
WORKDIR /home/ros
# Switch to the new user 'ros'
USER ros
RUN mkdir /home/ros/src && \
    chown -R ros:ros /home/ros && \
    rosdep update

# set pre-build variables: only packages in /opt/ros
ENV PATH /home/ros/devel/bin:/opt/ros/noetic/bin:.:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games

ENV PATH=/home/ros/devel/bin:/opt/ros/noetic/bin:.:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games
ENV ROS_PACKAGE_PATH=/home/ros/src:/opt/ros/noetic/share:/opt/ros/noetic/stacks
ENV CMAKE_PREFIX_PATH=/home/ros/noetic/catkin_ws/devel:/opt/ros/noetic
ENV PKG_CONFIG_PATH=/home/ros/devel/lib/pkgconfig:/opt/ros/noetic/lib/pkgconfig
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_WORKSPACE=/home/ros
ENV ROS_IP=127.0.0.1
ENV SWI_HOME_DIR=/usr/lib/swi-prolog
ENV PYTHONPATH=/home/ros/devel/lib/python2.7/dist-packages:/opt/ros/noetic/lib/python2.7/dist-packages
ENV LD_LIBRARY_PATH=/home/ros/devel/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/python2.7/dist-packages

# Forward ports: webserver + rosbridge
EXPOSE 1111 9090

# Initialize the catkin workspace
USER ros
WORKDIR /home/ros
RUN . /opt/ros/noetic/setup.sh && catkin_make

WORKDIR /home/ros/src
RUN if [ "x$LOCAL_BUILD" = "x0" ] ; then \
    rm -rf /home/ros/src/* && \
    git clone https://github.com/knowrob/knowrob.git && \
    git clone https://github.com/knowrob/rosprolog.git && \
    git clone https://github.com/code-iai/iai_maps.git && \
    git clone https://github.com/code-iai/iai_common_msgs.git && \
    git clone https://github.com/RobotWebTools/tf2_web_republisher.git ; \
    fi

WORKDIR /home/ros
# Build the catkin workspace
RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY run_knowrob.sh /run_knowrob.sh
COPY run_knowrob_local.sh /run_knowrob_local.sh

ENTRYPOINT ["/run_knowrob.sh"]