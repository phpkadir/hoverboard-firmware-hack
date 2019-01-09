# Hoverboard-Firmware-Hack

[![Build Status](https://travis-ci.com/BobbyIndustries/hoverboard-firmware-hack.svg?branch=master)](https://travis-ci.com/BobbyIndustries/hoverboard-firmware-hack)

### Under heavy development

This firmware is much better than the old one. tested up to 40A / 60V, no dead board so far :)
But The Firmware can't do inpossible Stuff pleace stay at 12s (under 50V) because the boards use condensators up to 50V higher voltages damage them.


This repo contains open source firmware for generic Hoverboard Mainboards.
The firmware you can find here allows you to use your Hoverboard Hardware (like the Mainboard, Motors and Battery) for cool projects like driving armchairs, person-tracking transportation robots and every other application you can imagine that requires controlling the Motors.

If you want an overview of what you can do with this firmware, here is a ~40min video of a talk about this project:
https://media.ccc.de/v/gpn18-95-howto-moving-objects

---

#### Hardware
![otter](https://raw.githubusercontent.com/NiklasFauth/hoverboard-firmware-hack/master/pinout.png)

The original Hardware supports two 4-pin cables that originally were connected to the two sensor boards. They break out GND, 12/15V and USART2&3 of the Hoverboard mainboard.
Both USART2 & 3 can be used for UART and I2C, PA2&3 can be used as 12bit ADCs.

The reverse-engineered schematics of the mainboard can be found here:
http://vocke.tv/lib/exe/fetch.php?media=20150722_hoverboard_sch.pdf

---

#### Flashing
To build the firmware, just type "make". Make sure you have specified your gcc-arm-none-eabi binary location in the Makefile ("PREFIX = ..."). Right to the STM32, there is a debugging header with GND, 3V3, SWDIO and SWCLK. Connect GND, SWDIO and SWCLK to your SWD programmer, like the ST-Link found on many STM devboards.

Make sure you hold the powerbutton or connect a jumper to the power button pins while flashing the firmware, as the STM might release the power latch and switches itself off during flashing. Battery > 36V have to be connected while flashing.

To flash the STM32, use the ST-Flash utility (https://github.com/texane/stlink).

If you never flashed your mainboard before, the STM is probably locked. To unlock the flash, use the following OpenOCD command:

```
make unlock
```
if it fails use
```
make fallback_unlock
```
if it doest work now use *Windows utility*

Then you can simply flash the firmware:
```
make flash
```

---
#### Troubleshooting
First, check that power is connected and voltage is >36V while flashing.
If the board draws more than 100mA in idle, it's probably broken.

If the motors do something, but don't rotate smooth and quietly, try to use an alternative phase mapping. Usually, color-correct mapping (blue to blue, green to green, yellow to yellow) works fine. However, some hoverboards have a different layout then others, and this might be the reason your motor isn't spinning.

Nunchuck not working: Use the right one of the 2 types of nunchucks. Use i2c pullups.

Nunchuck or PPM working bad: The i2c bus and PPM signal are very sensitive to emv distortions of the motor controller. They get stronger the faster you are. Keep cables short, use shielded cable, use ferrits, stabalize voltage in nunchuck or reviever, add i2c pullups. To many errors leads to very high accelerations which triggers the protection board within the battery to shut everything down.

Most robust way for input is to use the ADC and potis. It works well even on 1m unshielded cable. Solder ~100k Ohm resistors between ADC-inputs and gnd directly on the mainboard. Use potis as pullups to 3.3V.

---


#### Examples

We want support many projects like a longboard. our firmware will support variable sync with i2c and an i2c display PPM-Sum signal and ADC input
It will get new fieldweakoning algorythm and a timing algorythm
