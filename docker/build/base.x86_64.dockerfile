FROM ubuntu:18.04

RUN mkdir /Downloads

COPY installers /Downloads/installers

# install the necessary libs
RUN apt-get update && apt-get install -y ruby ruby-dev 
RUN apt-get install -y libpoco-dev uuid-dev libncurses5-dev python3-dev python3-pip libeigen3-dev
RUN apt-get install -y wget cmake curl libcurl4-openssl-dev git
RUN apt-get install -y openssh-server

# cmake 
WORKDIR /Downloads
COPY cmake-3.16.0.tar.gz /Downloads
RUN tar -zxvf cmake-3.16.0.tar.gz
WORKDIR /Downloads/cmake-3.16.0
RUN ./bootstrap
RUN make -j16
RUN make install

#absl
RUN bash /Downloads/installers/install_absl.sh

# third party
WORKDIR /home/baojiali/Downloads
ADD third_party_civpilot.zip /home/baojiali/Downloads/
ADD scripts /home/baojiali/Downloads/scripts
RUN unzip -d third_party third_party_civpilot.zip
RUN bash ./scripts/install.sh

# proj
RUN bash /Downloads/installers/install_proj.sh

WORKDIR /

