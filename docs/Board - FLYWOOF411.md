# Board - FLYWOOF411

## Flywoo 411 Flight controllers
Most of the Flywoo FC do not have I2C, BARO or both so may not suitable for iNav

Flywoo Goku F411 V1 does not have BARO or I2C pads:
https://cdn.shopify.com/s/files/1/0010/7410/2324/files/0e7d0153c93500f48670455f119034da.jpg?v=1575563256

Flywoo Goku F411 V2 does have I2C pads. Some boards may have a BARO. This board is suitable for iNav but but may require external BARO and Compass
https://cdn.shopifycdn.net/s/files/1/0010/7410/2324/products/20201126012224_2048x2048.png?v=1606326840


## NOXE F4
![Banggood](https://img.staticbg.com/thumb/view/oaupload/banggood/images/A5/01/79d28a2c-ef4b-4e4f-bab6-14edf66bbb23.jpg)
![Banggood](https://img.staticbg.com/images/oaupload/banggood/images/2E/C5/c14a1e86-4e58-4bc8-85de-8e344cb382b9.jpg)
![Banggood](https://img.staticbg.com/images/oaupload/banggood/images/42/F7/45a68ade-9be1-4fff-afec-bbdd45f0331d.jpg)

*Note:* This board used to be sold as a 'NOX F4' but is now an updated version similar to a Flywoo F411

## Banggood Specification:
* Model: Upgrade Betaflight F4 Noxe V1
* Version: Acro Version / Deluxe Version
* Acro Version: Without  Barometer and Blackbox
* Deluxe Version: With Barometer and Blackbox
* CPU: STM32F411C
* MPU: MPU6000
* Input Voltage: Support 2-6S Lipo Input
* Built-In Betaflight OSD
* Built-in 5V @ 2.5A BEC & 8V @ 3A BEC
* 3.3V
* 4.5V powered by USB
* DShot, Proshot ESC
* Support Spektrum 1024 /2048 , SBUS, IBUS, PPM 
* Built in flash for blackbox 16MB
* Support WS2812 LED strip
* Size: 27x27mm
* Mounting Hole: 20x20mm , M2.5
* Weight: 4.3g
* DSM / IBUS/SBUS Receiver, choose UART RX2
* PPM Receiver, don't need choose UART Port


## Where to buy:
* [Banggood](https://inavflight.com/shop/s/bg/1310419)

## Available TARGETS

* `FLYWOOF411` Warning - Serial 2 TX is shared with soft serial for use as S.Port or VTX control
* `FLYWOOF411GV2` Flywoo F411 Goku V2 with softserial full-duplex (TX = M5, RX = M6)
