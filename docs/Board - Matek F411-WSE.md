# Board - [MATEKSYS F411-WSE](http://www.mateksys.com/?portfolio=f411-wse)

![Matek F411 Wing](http://www.mateksys.com/wp-content/uploads/2019/06/F411-WSE_1-1500x600.jpg)

## Specification:

* STM32F411 CPU
* OSD
* BMP280 barometer
* Integrated PDB for 2 motors
* 2 UART ports
* 6 servos
* no SD card slot
* 3 BECs

## Details

* [Full specification](http://www.mateksys.com/?portfolio=f411-wse)
* SBUS pad has a built-in inverter and is connected to UART2 RX

## Available TARGETS

* `MATEKF411SE` Stock target. LED control and have SS1 on ST1 pad, SS2 on TX2 pad.
* `MATEKF411SE_PINIO` Adds USER 2 PINIO support on the LED pad.
* `MATEKF411SE_FD_SFTSRL1` Adds full duplex SS1 by putting the RX on the LED pad.
* `MATEKF411SE_SS2_CH6` SS2 moved to Ch6 pad. This keeps UART 2 as a full UART (for example, for use with Crossfire) and SS2 support.
