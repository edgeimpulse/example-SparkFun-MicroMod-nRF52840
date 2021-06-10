# example-SparkFun-MicroMod-nRF52840

This example has been designed for this workshop hosted by SparkFun: [AI Workshop: Meet Your New Fitness Trainer](https://learn.sparkfun.com/events/204_).

This tutorial is using:

**Hardware:**

- [SparkFun MicroMod Machine Learning Carrier Board](https://www.sparkfun.com/products/16400)
![micromod ML](assets/micromod-ml.png) 
- [Nordic NRF52840 MicroMod Processor Board](https://www.sparkfun.com/products/16984)
![nrf-processor](assets/nrf-processor.png)

**Tools and Softwares:**

- [Edge Impulse Studio](https://studio.edgeimpulse.com) and [Edge Impulse CLI](https://docs.edgeimpulse.com/docs/cli-installation)
- [Arduino IDE](https://www.arduino.cc/en/software)

## Installing the dependencies

### Install Arduino IDE

Download the lastest Arduino version: [https://www.arduino.cc/en/main/software](https://www.arduino.cc/en/main/software)

![dl arduino ide](assets/dl-arduino-ide.png)

We have been testing with Arduino 1.8.13. It should work also with the current latest Arduino IDE version (1.8.15). However, we do not recommend to use the Arduino IDE 2.0 beta at the moment as it has some breaking changes.

### Adding the MicroMod nRF52840 Board to Arduino IDE

A complete and official guide is provided in the following link to setup Arduino IDE to run with the nRF52840 Processor board: [https://learn.sparkfun.com/tutorials/micromod-nrf52840-processor-hookup-guide/arduino-software-setup](https://learn.sparkfun.com/tutorials/micromod-nrf52840-processor-hookup-guide/arduino-software-setup).

*Note that recently, Arduino has deprecated the Arduino Mbed OS Boards to provided standalone package for each boards:*

![board-manager-mbded-1.3.1](assets/board-manager-mbded-1.3.1.png)

In order to be able to compile and run this project on the MicroMod nRF52840 Processor, you will need to install the version `1.3.1` of this deprecated package. Sparkfun is working on a fix, the official guide should be update once done.


## Collecting data



## Train your Machine Learning Model



## Run your inference on the target


