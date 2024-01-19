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

MAVSDK 2.0 or newer is required and pre-built releases can be found at https://github.com/mavlink/MAVSDK/releases. To instead build MAVSDK from source, simple [scripts can be found in the WayWise repository](https://github.com/RISE-Dependable-Transport-Systems/WayWise/tree/main/tools/build_MAVSDK).

## Installing Prerequisites (on Ubuntu 20.04/22.04) & Building
    # Installing MAVSDK
    sudo dpkg -i libmavsdk-dev*.deb

    sudo apt install git build-essential cmake libqt5serialport5-dev libgpiod-dev
    git clone --recurse-submodules git@github.com:RISE-Dependable-Transport-Systems/RCCar.git
    cd RCCar
    mkdir build && cd build
    cmake ..
    make -j4
    
## Funded by
<img src="https://user-images.githubusercontent.com/2404625/202213271-a4006999-49d5-4e61-9f3d-867a469238d1.png" width="120" height="81" align="left" alt="EU logo" />
This project has received funding from the European Union’s Horizon 2020 and Horizon Europe research and innovation programmes, and the Digital Europe programme under grant agreement nº 814975, nº 101095835, 101069573 and nº 101100622. The results reflect only the authors' view and the Agency is not responsible
for any use that may be made of the information it contains.
