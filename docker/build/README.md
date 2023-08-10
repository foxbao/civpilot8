# Apollo Docker Image Build Process

## Table of Contents



## Build preparation
The easiest way is to build the docker and then build and run the program in container.
Make sure that all these files are available in docker/build because they will be used in the building of docker
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
docker build  -f base.x86_64.dockerfile -t civ:civauto .
```

## Start the docker
Go to the folder of civpilot8, and start the docker. The folder civpilot8 will be projected into the docker
```shell
docker run --rm -i -d -v `pwd`:/home/baojiali/Downloads/civpilot8 --name civauto civ:civauto
```
If you want to mount a device
```shell
docker run --rm -i -d -v `pwd`:/home/baojiali/Downloads/civpilot8 --device=/dev/ttyUSB0:/dev/ttyUSB0 --name civauto civ:civauto
```

## Get into docker
```shell
docker exec -it civauto /bin/bash
```

## build the Full Docker
```shell
bash ./build_full_docker.sh
```
