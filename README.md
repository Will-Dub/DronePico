To install OpenOCD for all rp2040 model:
1. git clone https://github.com/raspberrypi/openocd.git --recursive --branch rp2040 --depth=1
2. cd openocd
3. ./bootstrap
4. ./configure --enable-sysfsgpio --enable-bcm2835gpio
5. make
6. sudo make install