# STM 32 BMS

This project is based on the STM32F446RET6 MCU. I started this project in order to familiarize myself with STM32CubeIDE and learn KiCAD. There are two versions to this:
1. STM 32 Nucleo F446RE Board setup - this was setup on the MCU board using components arranged on the breadboard 
2. STM32F446RET6 based PCB design - more space efficient and streamlined design, as the initial design was not compact

Both follow a similar schematic based on the below design.

<img width="712" alt="image" src="https://github.com/user-attachments/assets/6041a2f6-3a65-4674-be4f-bb3ce416be32" />

The main difference is that the board setup implementation uses a voltage sensor (which has a built in voltage divider with the same resistor values as schematic) placed in the same location, instead of a voltage divider circuit. 

