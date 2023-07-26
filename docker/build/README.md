# Apollo Docker Image Build Process

## build the light Docker
we can build a light docker without visualization function, so without OpenCV, QT or VTK
```shell
bash ./build_docker.sh
```

## Start the docker
```shell
docker run --rm -i -d -v `pwd`/../../:/home/baojiali/civpilot8 --name civauto baojiali:test
```

## Get into docker

## build the Full Docker
```shell
bash ./build_full_docker.sh
```
