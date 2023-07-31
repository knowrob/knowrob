FROM knowrob:noetic-base
MAINTAINER Sascha Jongebloed, jongebloed@uni-bremen.de

ENV SWI_HOME_DIR=/usr/lib/swi-prolog
ENV LD_LIBRARY_PATH=/usr/lib/swi-prolog/lib/x86_64-linux:$LD_LIBRARY_PATH

RUN apt update
Run apt install -y gdb g++ clang cmake make libeigen3-dev libspdlog-dev libraptor2-dev mongodb-clients libmongoc-1.0-0 libmongoc-dev libfmt-dev software-properties-common python3-catkin-pkg

RUN apt-add-repository ppa:swi-prolog/stable
RUN apt update
RUN apt install -y swi-prolog* 

COPY run_knowrob.sh /run_knowrob.sh
COPY run_knowrob_local.sh /run_knowrob_local.sh

RUN ls /usr/lib/swi-prolog/lib/x86_64-linux
RUN echo "a"

RUN echo $LD_LIBRARY_PATH

ENTRYPOINT ["/run_knowrob.sh"]
