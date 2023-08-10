FROM ubuntu:18.04

RUN mkdir /Downloads
# install the necessary libs
RUN apt-get update && apt-get install -y ruby ruby-dev 
RUN apt-get install -y libpoco-dev uuid-dev libncurses5-dev python3-dev python3-pip libeigen3-dev
RUN apt-get install -y wget cmake curl libcurl4-openssl-dev git


# cmake 
WORKDIR /Downloads
COPY cmake-3.16.0.tar.gz /Downloads
RUN tar -zxvf cmake-3.16.0.tar.gz
WORKDIR /Downloads/cmake-3.16.0
RUN ./bootstrap
RUN make -j16
RUN make install

# absl
WORKDIR /Downloads
RUN wget https://apollo-system.cdn.bcebos.com/archive/6.0/20200225.2.tar.gz
RUN tar -xzvf 20200225.2.tar.gz
WORKDIR /Downloads/abseil-cpp-20200225.2
RUN sed -i '23i set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")' CMakeLists.txt
RUN cmake -DBUILD_SHARED_LIBS=ON -L CMakeLists.txt && make -j$(nproc)
RUN make install

# third party
WORKDIR /home/baojiali/Downloads
ADD third_party_civpilot.zip /home/baojiali/Downloads/
ADD scripts /home/baojiali/Downloads/scripts
RUN unzip -d third_party third_party_civpilot.zip
RUN bash ./scripts/install.sh

# proj
WORKDIR /Downloads
RUN apt-get install -y libproj-dev sqlite3 libtiff-dev
RUN apt-get install -y libgtest-dev
RUN wget https://download.osgeo.org/proj/proj-9.0.1.tar.gz
RUN tar -xzvf proj-9.0.1.tar.gz
WORKDIR /Downloads/proj-9.0.1
RUN mkdir build
WORKDIR /Downloads/proj-9.0.1/build
RUN cmake ..
RUN cmake --build . -- -j$(nproc)
RUN make install

WORKDIR /

