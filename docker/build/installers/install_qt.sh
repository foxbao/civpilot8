cd /Downloads
wget https://download.qt.io/archive/qt/5.12/5.12.12/single/qt-everywhere-src-5.12.12.tar.xz
tar -xf qt-everywhere-src-5.12.12.tar.xz
cd qt-everywhere-src-5.12.12
./configure
make -j16
make install
sudo apt install libqt5x11extras5-dev
sudo apt install libqt5serialport5
sudo apt install libqt5serialport5-dev