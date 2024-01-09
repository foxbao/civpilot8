# V2I Docker Image Build Process
- [V2I Docker Image Build Process](#v2i-docker-image-build-process)
  - [Build preparation](#build-preparation)
  - [build the light Docker](#build-the-light-docker)
  - [Start the docker](#start-the-docker)
  - [Get into docker](#get-into-docker)
  - [build the Full Docker](#build-the-full-docker)

## Build preparation
The easiest way is to build the docker and then build and run the program in container.
Make sure that all these files are available in docker/build because they will be used in the building of docker
Two files might be missing from github. You can download from Baidu Pan  
cmake-3.16.0.tar.gz, link：https://pan.baidu.com/s/1mrFneIqKtS_6vmeZb7jJBA , passwd：1234  
third_party_civpilot.zip, link: https://pan.baidu.com/s/1racV6nXaHjWpa3d7RAci-Q, code:1234  

```shell
├── base.x86_64.dockerfile
├── build_docker.sh
├── cmake-3.16.0.tar.gz
├── README.md
├── scripts
│   ├── apollo.bashrc
│   ├── dev_start.sh
│   ├── docker_base.sh
│   ├── FastRTPS_1.5.0.patch
│   └── install.sh
└── third_party_civpilot.zip
```


## build the light Docker
we can build a light docker without visualization function, so without OpenCV, QT or VTK
```shell
cd docker/build
sudo docker build -f base.x86_64.dockerfile -t civ:civauto .
```
if you want to build a docker with visualization
```shell
cd docker/build
sudo docker build  -f full.x86_64.dockerfile -t civ:civauto .
```

## Start the docker
Go to the folder of civpilot8, and start the docker. The folder civpilot8 will be projected into the docker
```shell
cd civpilot8
sudo docker run --rm -i -d -v `pwd`:/home/baojiali/Downloads/civpilot8 --name civauto civ:civauto
```
If you want to mount a device
```shell
sudo docker run --rm -i -d -v `pwd`:/home/baojiali/Downloads/civpilot8 --device=/dev/ttyUSB0:/dev/ttyUSB0 --name civauto civ:civauto
```

## Get into docker
```shell
sudo docker exec -it civauto /bin/bash
```

## build the Full Docker
```shell
bash ./build_full_docker.sh
```
## build code
```shell
cd civpilot8
source ../install/setup.bash
mkdir build
cd build
cmake ..
make -j16
```
