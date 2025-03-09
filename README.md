# STM-32 BATTERY MANAGEMENT SYSTEM (BMS)

A comprehensive Battery Mangement System built using the STM32F446RET6 MCU.

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Hardware Design](#hardware-design)
- [Firmware](#firmware)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
- [Usage](#usage)
- [Acknowledgements](#acknowledgements)

 ## Overview
This project entails the development of a Battery Management System (BMS) utilizing the STM32F446RE. The initiative began in order to gain proficiency with STM32CubeIDE and KiCAD, resulting in two versions:
1. STM32 Nucleo F446RE Board Setup: Implemented on the MCU board with the components on a breadboard. Used intially to make sure my design is functional.
2. Custom PCB Design: A more compact and streamlined version based on the same STM32F446RET6 MCU. Worked on this after verification of design on the Nucleo board.
Both versions adhere to a similar schematic, with the primary difference being the use of a voltage sensor in the original board setup implementation, which was replaced by a voltage divider in the PCB design in order to save cost.

## Features
+ Real Time Battery Monitoring: Continuous tracking of battery voltage, current and temperature.
+ Overcharge, Overdischarge and Overheat Protection: Safeguards to prevent battery damage due to extreme charge levels.
+ Modular Design: PCB design entails voltage, current and temperature specs for a 18650 Li-ion battery but can be easily scaled and integrated for various battery configurations.

## Hardware Design
The hardware comprises the ST MCU at the heart, accompanied by necessary sensors and protection circuits. The PCB design was crafted using KiCAD, emphasizing a compact layout while maintaining signal integrity. A complete list of the exact components used can be found in the Bill of Materials excel file in the PCB design folder.
Schematic Diagram:

<img width="712" alt="image" src="https://github.com/user-attachments/assets/6041a2f6-3a65-4674-be4f-bb3ce416be32" />

## Firmware
The firmware is developed in C using STM32CubeIDE, with the core functionalities being:
+ ADC Configuration: For sampling voltage, current and temperature data.
+ I2C Communication: Enables data transmission to the LED for easy visualization of data.
+ Control Algorithms: Manage protection mechanisms. 

## Getting Started
How to set up the project...

### Prerequisites
+ Hardware:
  - STM32F446RET6 MCU or any board with that MCU such as the Nucleo board
  - Associated sensors and peripheral components (listed in the BOM in the PCB folder)
+ Software:
  - STM32CubeIDE
  - KiCAD

### Installation
1. Clone the Repository:
```bash
git clone https://github.com/r4hulrr/stm-32-bms.git
```
2. Open the Firmware Project:
   + Launch STM32CubeIDE
   + Navigate to FILE->OPEN PROJECTS FROM FILE SYSTEM->FIRMWARE
3. Build and Flash:
   + Connect your STM32 board via USB.
   + Build the project and flash the firmware using the IDE's tools. 
## Usage
Upon successful flashing, the BMS will commence real-time monitoring. Data can be observed through the LCD screen.

## Acknowledgements
This project includes the I2C driver for LCD display from [@eziya](https://github.com/eziya)


