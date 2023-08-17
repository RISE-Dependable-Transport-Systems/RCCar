# RCCar
Implementation of an autonomous rc car based on WayWise.

Main components:
- rc car base with brushless motor (almost anything works but, e.g., [Traxxas](https://traxxas.com/products/showroom) can be recomended)
- Raspberry Pi (or similar)
- [VESC motor controller](https://trampaboards.com/vesc--c-1434.html)
- [u-blox F9R GNSS receiver](https://www.sparkfun.com/products/16475) (alternatively F9P plus BNO055 or VESC-internal IMU are supported)

Optional components:
- Luxonis AI stereo camera, e.g., [OAK D](https://docs.luxonis.com/projects/hardware/en/latest/pages/BW1098OAK.html) (Lite, Pro, PoE, ...)
- [Pozyx](https://www.pozyx.io/) Creator Kit for UWB indoor positioning (rudimentary support)

![IMG_2485 (3)](https://user-images.githubusercontent.com/2404625/202223980-23ae9371-e6f8-4109-9016-bb176be81f4f.jpg)

![image](https://user-images.githubusercontent.com/2404625/202226896-c18d3567-b714-4700-89af-7c20cadf11c0.png)

MAVSDK commit 926b067 or newer is require for building, which will probably become MAVSDK 2.0. For the time being you need to build MAVSDK yourself or use the *.deb (Ubuntu 20.04/22.04) provided in the main folder of this repository.

## Installing Prerequisites (on Ubuntu 20.04/22.04) & Building
    sudo apt install git build-essential cmake libqt5serialport5-dev 
    git clone --recurse-submodules git@github.com:RISE-Dependable-Transport-Systems/RCCar.git
    
    # In case you do not want to build MAVSDK (main branch) yourself:
    sudo dpkg -i RCCar/libmavsdk-dev_*_amd64.deb && sudo ln -s /usr/local/lib/libmavsdk.so.1.4.0 /usr/local/lib/libmavsdk.so.1
    
    mkdir build && cd build
    cmake ..
    make -j4
