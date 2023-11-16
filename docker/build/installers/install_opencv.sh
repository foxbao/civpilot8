cd /Downloads
apt-get install -y libpng-dev libjpeg-dev libopenexr-dev libtiff-dev libwebp-dev    
git clone https://github.com/opencv/opencv.git
cd opencv
mkdir build
cd build
cmake ../
make -j16
make install