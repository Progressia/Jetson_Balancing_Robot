# JETSON BALANCING ROBOT

A two-wheeled balancing robot inspired by the [*SimpleFOC Balancing Robot*](https://github.com/simplefoc/Arduino-FOC-balancer.git)

ToDo: Real photo -- comming soon

Modular two wheeled balancing robot based on gimbal BLDC motors and [*Simple**FOC**Shields*](https://simplefoc.com/). NVIDIA Jetson Nano is use  to allow for control motion on base computer vision algorithms and AI models. Application for remote control of robot was build on flutter framework. 3D models with STL files are in CAD folder.

## YouTube video demo

ToDo: Video -- comming soon

## Readme structure
- [Mechanical components](#mechanical-components)
    - [3D printed parts](#3d-printed-parts)
    - [Hardware parts](#hardware-parts)
- [Electrical components](#electrical-components)
    - [Structure](#structure)
    - [BLDC motor](#bldc-motor)
    - [Position sensors](#position-sensors)
    - [BLDC driver](#bldc-driver)
    - [IMU module](#imu-module)
    - [WiFi module](#wifi-module)
    - [Microcontroller](#microcontroller)
- [Software](#software)
    - [STM32 Code](#stm32-code)
    - [NVIDIA Jetson Nano Code](#nvidia-jetson-nano-code)
    - [Remote control app](#remote-control-app)
    - [RP2040 LCD UART](#rp2040-lcd-uart)

--------------------------

## Mechanical components

### 3D printed parts

Robot has 4 3d printed parts. You can find them in the `CAD` directory. They are (***reasonably precise***):
- Base frame (`base_frame.stl`)
- Cube frame (`cube_frame.stl`) 
- Plate for electronics (`plate.stl`)
- Wheel hub (`wheel_hub.stl`)
- Camera and LCD hub (`camera_lcd_hub.stl`)
- RP2040 LCD adapter (`rp2040_hub.stl`)
- Distance sensor hub (`dist_sens_hub.stl`)

Parts was printed from ABS filament.

>Beware: Assembly holes match the motor and electronics I was using, so please check the dimensions of your parts holes before printing

### Hardware parts

In addition to the printed parts, you will need two RC wheels (HEX17) and a handful of nuts and bolts

Component | Description
---- | ----
<img src="/home/dell/PROGRESSIA/robots/Jetson_Balancing_Robot/images/wheels.png" height="125px"> | Wheels Louise B-Rocket hex 17mm 1:8 Buggy (2 pcs)
<img src="/home/dell/PROGRESSIA/robots/Jetson_Balancing_Robot/images/screws.jpg" height="100px"> | Screws M3x20 (4 pcs)
<img src="/home/dell/PROGRESSIA/robots/Jetson_Balancing_Robot/images/nutsM3.png" height="110px"> | Nuts M3 
<img src="/home/dell/PROGRESSIA/robots/Jetson_Balancing_Robot/images/screwsM10.png" height="85px"> | Screws M10x25 (2 pcs)
<img src="/home/dell/PROGRESSIA/robots/Jetson_Balancing_Robot/images/nutsM10.png" height="95px">  | Nuts M10 (2 pcs)

--------------------------

## Electrical components

### Structure
<img src="/home/dell/PROGRESSIA/robots/Jetson_Balancing_Robot/images/scheme.png" height="537px">
<p style="text-align: center;">System diagram</p>

- Connection pinout

STM32 | SimpleFOC #1
---- | ----
5V | VCC
GND | GND

STM32 | SimpleFOC #2
---- | ----
5V | VCC
GND | GND

// (6, 5, 10, 8)
#define MOT1_A 6
#define MOT1_B 5
#define MOT1_C 10
#define MOT1_EN 8

// (9, 3, 11, 7)
#define MOT2_A 9
#define MOT2_B 3
#define MOT2_C 11
#define MOT2_EN 7

STM32 | IMU (MPU6050)
---- | ----
5V | VCC
PB8 (I2C1_SCL) | SCL
PB9 (I2C1_SDA) | SDA
GND | GND

STM32 | Magnetic Encoder #Left (AS5048A)
---- | ----
5V | VCC (Green)
PA10 (CS) | CS (White)
PB13 (SPI2_SCK) | CLK (Red)
PB15 (SPI2_MOSI) | MOSI (Black)
PB14 (SPI2_MISO) | MISO (Yellow)
GND | GND (Purple)

STM32 | Magnetic Encoder #Right (AS5048A)
---- | ----
5V | VCC (Green)
PB5 (CS) | CS (White)
PB13 (SPI2_SCK) | CLK (Red)
PB15 (SPI2_MOSI) | MOSI (Black)
PB14 (SPI2_MISO) | MISO (Yellow)
GND | GND (Purple)

STM32 | NVIDIA Jetson Nano 4GB
---- | ----
5V (optional) | 5V
PC11 (UART4_RX) | TXD (8-Yellow)
PC10 (UART4_TX) | RXD (10-Orange)
GND | GND (6)

NVIDIA Jetson Nano | RP2040 LCD
---- | ----
USB | USB-C
### BLDC motor

The BLDC motors I have used are the iPower GM4108. You can use any other gimbal motors of this class with minor modifications - repositioning the mounting holes in 3D printed parts.

Component | Description
---- | ----
<img src="/home/dell/PROGRESSIA/robots/Jetson_Balancing_Robot/images/motor.jpg" height="110px"> | iPower Motor GM4108H-120T

### Position sensors
In my case, I used two magnetic sensors (attached to the motors). You can use any type of position sensor that is supported by the [*Simple**FOC**Shields library*](https://simplefoc.com/).

Component | Description
---- | ----
<img src="/home/dell/PROGRESSIA/robots/Jetson_Balancing_Robot/images/motor_sensor.jpg" height="130px"> | Magnetic sensor AS5048 (attached to the motors)

### BLDC driver
For this project, I used [*Simple**FOC**Shields v2.0.4*](https://simplefoc.com/), which enables the use of the FOC algorithm with BLDC motors.

Component | Description
---- | ----
<img src="/home/dell/PROGRESSIA/robots/Jetson_Balancing_Robot/images/simplefocShield.jpg" height="130px"> | [SimpleFOC Shield v2.0.4](https://simplefoc.com/)

### IMU module
I used the IMU MPU6050 with integrated DMP.

Component | Description
---- | ----
<img src="/home/dell/PROGRESSIA/robots/Jetson_Balancing_Robot/images/MPU6050-Module.jpg" height="150px"> | IMU MPU6050

### Embedded computing board & Microcontroller
Based on the [*SimpleFOC Balancing Robot*](https://github.com/simplefoc/Arduino-FOC-balancer.git), I decided to extend the robot and added mainly an NVIDIA Jetson Nano for using computer vision algorithms and an AI model.
I used the ESP8266 WiFi server for testing and initial calibration of the robot.

Component | Description
---- | ----
<img src="/home/dell/PROGRESSIA/robots/Jetson_Balancing_Robot/images/nvidiaJetsonNano.jpeg" height="140px"> | Nvidia Jetson Nano B01
<img src="/home/dell/PROGRESSIA/robots/Jetson_Balancing_Robot/images/stm32_nucleo.jpeg" height="105px"> | STM32 F446 Nucleo
<img src="/home/dell/PROGRESSIA/robots/Jetson_Balancing_Robot/images/esp8266.jpg" height="80px"> | ESP8266
<img src="/home/dell/PROGRESSIA/robots/Jetson_Balancing_Robot/images/lcd.png" height="140px"> | [Waveshare RP2040 LCD IPS 1,28'' 240 x 240 px](https://www.waveshare.com/wiki/RP2040-LCD-1.28)

### Others
I added a camera, display and distance sensors. You can use any NVIDIA Jetson Nano compatible camera.

Component | Description
---- | ----
<img src="https://www.waveshare.com/w/fkbk/swtumb.php?f=IMX219-83-Stereo-Camera-1.jpg&width=300" height="110px"> | [IMX219-83 Stereo Camera](https://www.waveshare.com/wiki/IMX219-83_Stereo_Camera)
<img src="/home/dell/PROGRESSIA/robots/Jetson_Balancing_Robot/images/SEN0307.png" height="120px"> | [URM09 Analog Ultrasonic Sensor](https://www.dfrobot.com/product-1862.html) 2pcs
---| [Battery LiFePO4 9.9V 2500 mAh](https://btobattery.com/)

--------------------------

## Software

### STM32 Code
The code for the STM32 is actually almost the same as for the example [*SimpleFOC Balancing Robot*](https://github.com/simplefoc/Arduino-FOC-balancer.git) with a few modifications. 

For more details, see section [stm32_BLDC_Controller](/home/dell/PROGRESSIA/robots/Jetson_Balancing_Robot/stm32_BLDC_Controller)

### ESP Code
In the initial phase, the ESP8266 was used as a WiFi server to control the robot and make an initial calibration of the controller settings. 

For more details, see section [esp8266_Basic_Server](/home/dell/PROGRESSIA/robots/Jetson_Balancing_Robot/esp8266_Basic_Server)

### NVIDIA Jetson Nano Code
As mentioned earlier. NVIDIA Jetson Nano is used to implement motion algorithms and implement AI models. As an example, it demonstrates how to get a remote view from a camera mounted on a robot and stream it to a smartphone. 

For more details, see section [nvidiaJetson_app](/home/dell/PROGRESSIA/robots/Jetson_Balancing_Robot/nvidiaJetson_app)

### Robot script
Script in Python with using object-oriented programming.

### Remote control app
I made the app to handle the robot's functions, parameter settings and camera view on the Flutter framework. You can compile the app on Android, iOS(not tested), Linux, Windows (not tested) and others. 

For more details, see section [flutter_remote_controller](/home/dell/PROGRESSIA/robots/Jetson_Balancing_Robot/flutter_remote_controller)

### RP2040 LCD UART
The application, on the RP2040, displays information from the NVIDIA Jetson platform via the UART

For more details, see section [rp2040_lcd_uart](/home/dell/PROGRESSIA/robots/Jetson_Balancing_Robot/rp2040_lcd_uart)

