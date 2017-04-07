# Prototype RF Driver for STM Sub-1 GHz RF Expansion Boards based on the SPSGRF-868 Module for STM32 Nucleo #

Currently supported boards:
 * [X-NUCLEO-IDS01A4](http://www.st.com/content/st_com/en/products/ecosystems/stm32-open-development-environment/stm32-nucleo-expansion-boards/stm32-ode-connect-hw/x-nucleo-ids01a4.html)

**Note**, in order to use expansion board `X-NUCLEO-IDS01A4` in mbed you need to perform the following HW modifications on the board:
 * **Un**mount resistor `R4`
 * **Mount** resistor `R7`
 
Furthermore, on some Nucleo development boards (e.g. the [NUCLEO_F429ZI](https://developer.mbed.org/platforms/ST-Nucleo-F429ZI/)), in order to be able to use Ethernet together with these Sub-1 GHz RF expansion boards, you need to compile this driver with macro `SPIRIT1_SPI_MOSI=PB_5` defined, while the development board typically requires some HW modification as e.g. described [here](https://github.com/ARMmbed/sal-nanostack-driver-stm32-eth)! 

This driver can be used together with the 6LoWPAN stack (*a.k.a.* Nanostack).
