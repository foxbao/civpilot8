# Apollo Docker Image Build Process

## build the light Docker
we can build a light docker without visualization function, so without OpenCV, QT or VTK
```shell
bash ./build_docker.sh
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
