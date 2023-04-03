FROM ubuntu:18.04
MAINTAINER Daniel Beßler, danielb@uni-bremen.de

ARG HOST_IP=172.17.0.1
ARG ROS_SOURCES="http://packages.ros.org/ros/ubuntu"

ARG USE_APT_CACHE=0
ARG LOCAL_BUILD=0

# Use apt-cacher container
RUN if [ "x$USE_APT_CACHE" = "x0" ] ; then \
    echo "no apt-cacher is used." ; else \
    echo 'Acquire::http { Proxy "http://172.17.42.1:3142"; };' >> /etc/apt/apt.conf.d/01proxy \
    echo 'Acquire::http { Proxy "http://172.17.0.1:3142"; };' >> /etc/apt/apt.conf.d/02proxy ; \
    fi

RUN apt-get -qq update && \
    DEBIAN_FRONTEND=noninteractive apt-get -qq -y install wget apt-utils gnupg tzdata
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y locales
RUN sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
    dpkg-reconfigure --frontend=noninteractive locales && \
    update-locale LANG=en_US.UTF-8
ENV LANG en_US.UTF-8 
    
# Install ROS
RUN sh -c 'echo "deb ${ROS_SOURCES} bionic main" > /etc/apt/sources.list.d/ros-latest.list'
#RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN sh -c 'wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.value && apt-value add ros.value' && \
    apt-get -qq -y update && \
    apt-get -qq -y install ros-melodic-desktop \
                           libjson-glib-dev python-yaml python-catkin-pkg python-rospkg \
                           emacs ros-melodic-catkin git \
                           ros-melodic-rosbridge-suite \
                           ros-melodic-cv-bridge \
                           software-properties-common \
                           mongodb-clients libmongoc-dev  python-pip\
                           ros-melodic-rosauth mencoder lame libavcodec-extra \
                           texlive-latex-base imagemagick python-rdflib  && \
    apt-add-repository -y ppa:swi-prolog/stable && \
    apt-get -qq -y update && \
    apt-get -qq -y install  swi-prolog && \
    apt-get -qq -y autoremove  && \
    apt-get -qq -y clean  && \
    rm -rf /var/lib/apt/lists/*  && \
    rm -rf /tmp/*
RUN pip install -U rosdep future
# ROS environment setup
RUN cp /opt/ros/melodic/setup.sh /etc/profile.d/ros_melodic.sh && rosdep init

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

RUN echo "source /opt/ros/melodic/setup.bash" >> /home/ros/.bashrc && \
    echo "source /home/ros/.bashrc" >> /home/ros/.bash_profile

# set pre-build variables: only packages in /opt/ros
ENV PATH /home/ros/devel/bin:/opt/ros/melodic/bin:.:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games

ENV PATH=/home/ros/devel/bin:/opt/ros/melodic/bin:.:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games
ENV ROS_PACKAGE_PATH=/home/ros/src:/opt/ros/melodic/share:/opt/ros/melodic/stacks
ENV CMAKE_PREFIX_PATH=/home/ros/melodic/catkin_ws/devel:/opt/ros/melodic
ENV PKG_CONFIG_PATH=/home/ros/devel/lib/pkgconfig:/opt/ros/melodic/lib/pkgconfig
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_WORKSPACE=/home/ros
ENV ROS_IP=127.0.0.1
ENV SWI_HOME_DIR=/usr/lib/swi-prolog
ENV PYTHONPATH=/home/ros/devel/lib/python2.7/dist-packages:/opt/ros/melodic/lib/python2.7/dist-packages
ENV LD_LIBRARY_PATH=/home/ros/devel/lib:/opt/ros/melodic/lib:/opt/ros/melodic/lib/python2.7/dist-packages

# Forward ports: webserver + rosbridge
EXPOSE 1111 9090

# Initialize the catkin workspace
USER ros
WORKDIR /home/ros/src
RUN /usr/bin/python /opt/ros/melodic/bin/catkin_init_workspace

RUN if [ "x$LOCAL_BUILD" = "x0" ] ; then \
    rm -rf /home/ros/src/* && \
    git clone https://github.com/knowrob/knowrob.git && \
    git clone https://github.com/knowrob/rosprolog.git && \
    git clone https://github.com/code-iai/iai_maps.git && \
    git clone https://github.com/code-iai/iai_common_msgs.git && \
    git clone https://github.com/code-iai/iai_cad_tools.git && \
    git clone https://github.com/RobotWebTools/mjpeg_server.git && \
    git clone https://github.com/RobotWebTools/tf2_web_republisher.git ; \
    fi

WORKDIR /home/ros
# Build the catkin workspace
RUN /opt/ros/melodic/bin/catkin_make

COPY run_knowrob.sh /run_knowrob.sh
COPY run_knowrob_local.sh /run_knowrob_local.sh

ENTRYPOINT ["/run_knowrob.sh"]
