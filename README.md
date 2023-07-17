# Apollo(v8.0.0) CyberRT

## #1 Env

1. > dependence

```shell
sudo apt update
sudo apt install -y libpoco-dev uuid-dev libncurses5-dev python3-dev python3-pip libeigen3-dev
python3 -m pip install protobuf==3.14.0
```

2. > absl
```shell
    wget https://apollo-system.cdn.bcebos.com/archive/6.0/20200225.2.tar.gz
    tar -xzvf 20200225.2.tar.gz
```

add the following cmake command in CMakeLists.txt
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
```shell
    cmake -DBUILD_SHARED_LIBS=ON -L CMakeLists.txt && make
    sudo make install
```

3. > proj
```shell
    sudo apt-get install libproj-dev
    sudo apt install sqlite3
```
Download projxxxx.tar.gz from from https://proj.org/download.html
```shell
    tar -xzvf projxxx.tar.gz
    cd proj-9.0.1
    mkdir build
    cd build
    cmake ..
    cmake --build .
	sudo make install
```

4. > OpenCV
```shell
    sudo apt-get install libpng-dev
    sudo apt-get install libjpeg-dev
    sudo apt-get install libopenexr-dev
    sudo apt-get install libtiff-dev
    sudo apt-get install libwebp-dev    

    https://docs.opencv.org/4.x/d2/de6/tutorial_py_setup_in_ubuntu.html
    git clone https://github.com/opencv/opencv.git

    mkdir build
    cd build
    cmake ../
    make -j16
    sudo make install
```
5. > VTK VTK 8.2.0
```shell
    https://vtk.org/download/
    cmake-gui (select qt related, then press "configure","generate")
    cd build
    make
    sudo make install 
    dependencies
    sudo apt-get install qttools5-dev
    sudo apt install libxt-dev
```
6. > QT 5.12
```shell
    refer to https://doc.qt.io/archives/qt-5.12/linux-building.html
    https://download.qt.io/archive/qt/5.12/5.12.12/
    download qt-everywhere-src-5.12.12.tar.xz 
    xz -d qt-everywhere-src-5.12.12.tar.xz
    tar -xvf qt-everywhere-src-5.12.12.tar
    cd qt-everywhere-src-5.12.12
    ./configure
    make
    make install
    sudo apt install libqt5x11extras5-dev
    sudo apt install libqt5serialport5
    sudo apt install libqt5serialport5-dev

    Download qt-opensource-linux-x64-5.12.12.run
    Run it and install only Qt-Creator

    sudo apt-get install xserver-xorg
```
## #2 Build

1. clone

```shell
git clone git@github.com:foxbao/civpilot8.git
cd civpilot8
git checkout -b dev origin/dev
```

2. build third party

> install

```shell
./scripts/install.sh
```

> export path

```shell
source install/setup.bash
```

3. build cyber

```shell
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## #3 Examples

1. pub/sub

> talker

```shell
source setup.bash
./cyber/examples/cyber_example_talker
```
> listener

```shell
source setup.bash
./cyber/examples/cyber_example_listener
```

2. component

```shell
source setup.bash
cyber_launch start share/examples/common.launch
./cyber/examples/common_component_example/channel_prediction_writer
./cyber/examples/common_component_example/channel_test_writer
```

## #4 Tools

1. channel

> list

```shell
source setup.bash
cyber_channel list

# The number of channels is:  1
# /apollo/test
```

> echo
```shell
source setup.bash
cyber_channel echo /apollo/test
```
![example](docs/cyber_echo.png)

> more ...

```shell
Commands:
	cyber_channel list	list active channels
	cyber_channel info	print information about active channel
	cyber_channel echo	print messages to screen
	cyber_channel hz	display publishing rate of channel
	cyber_channel bw	display bandwidth used by channel
	cyber_channel type	print channel type
```

2. node

```shell
Commands:
	cyber_node list 	List active nodes.
	cyber_node info 	Print node info.
```

3. service

```shell
Commands:
	cyber_service list	list active services
	cyber_service info	print information about active service
```

4. launch

```shell
cyber_launch start share/examples/common.launch
```

5. monitor

```shell
cyber_monitor
```

6. recorder

```shell
Commands:
  	cyber_recorder info	Show information of an exist record.
	cyber_recorder play	Play an exist record.
	cyber_recorder record	Record same topic.
	cyber_recorder split	Split an exist record.
	cyber_recorder recover	Recover an exist record.
```

## #5 Package

```shell
cmake -DCMAKE_INSTALL_PREFIX=/you/install/path ..
make
make package
sudo dpkg -i package/*.deb
```
