# proj
cd /Downloads
apt-get install -y libproj-dev sqlite3 libtiff-dev
apt-get install -y libgtest-dev
wget https://download.osgeo.org/proj/proj-9.0.1.tar.gz
tar -xzvf proj-9.0.1.tar.gz
cd /Downloads/proj-9.0.1
mkdir build
cd /Downloads/proj-9.0.1/build
cmake ..
cmake --build . -- -j$(nproc)
make install