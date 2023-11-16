# absl
cd /Downloads
wget https://apollo-system.cdn.bcebos.com/archive/6.0/20200225.2.tar.gz
tar -xzvf 20200225.2.tar.gz
cd /Downloads/abseil-cpp-20200225.2
sed -i '23i set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")' CMakeLists.txt
cmake -DBUILD_SHARED_LIBS=ON -L CMakeLists.txt && make -j$(nproc)
make install