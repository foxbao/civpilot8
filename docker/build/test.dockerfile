FROM ubuntu
RUN apt-get update && apt-get install -y ruby ruby-dev 
RUN apt-get install -y libpoco-dev uuid-dev libncurses5-dev python3-dev python3-pip libeigen3-dev
