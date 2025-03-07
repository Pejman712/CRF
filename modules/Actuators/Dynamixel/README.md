# Dynamixel module {#dynamixel_modules}

To new Dynamixel unpacking, it is mandatory to setup the ID (1 by default) and Baud-Rate (default settings depends of the model).
To carry it out, it is necessary to use the Dynamixel Wizard 2.

## Dynamixel Wizard2
### Installation
1. Download the package file: 
    http://www.robotis.com/service/download.php?no=1671
2. Enter the following command to change the permission:
```ruby
    sudo chmod 775 DynamixelWizard2Setup_x64
```
3. Run the install program:
```ruby
    ./DynamixelWizard2Setup_x64
```

Maybe you will need permission to `/dev` folder:
```ruby
    sudo usermod -a -G dialout $USER
    sudo reboot
```

To use the dynamixel connected in the port `/dev/ttyUSB0` you have to add your uer to dialout group:
```bash
    sudo usermod -a -G dialout $USER
```


To use the module DynamixelSDK needs to be installed.

## GitHub repository
### Installation
1. Clone the repository from:
    https://github.com/ROBOTIS-GIT/DynamixelSDK

2. Enter on the specific folder for your programming language/build/#your system#/
```bash
    cd ~/DynamixelSDK/c++/build/linux64
```
3. Build de package:
```bash
    make
    sudo make install
```

4. Add DynamixelSDK to your project CMakeList.
```ruby
    set(MODULE_LIBS dxl_x64_cpp)
```
