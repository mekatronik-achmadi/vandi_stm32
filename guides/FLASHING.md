# Program Flashing Guide

## Table of Contents
- [SWD](https://github.com/mekatronik-achmadi/vandi_stm32/blob/main/guides/FLASHING.md#swd)
  - [Requirements](https://github.com/mekatronik-achmadi/vandi_stm32/blob/main/guides/FLASHING.md#requirements)
  - [Pinout](https://github.com/mekatronik-achmadi/vandi_stm32/blob/main/guides/FLASHING.md#pinout)
  - [Connect](https://github.com/mekatronik-achmadi/vandi_stm32/blob/main/guides/FLASHING.md#connect)
  - [Erasing](https://github.com/mekatronik-achmadi/vandi_stm32/blob/main/guides/FLASHING.md#erasing)
  - [Writing](https://github.com/mekatronik-achmadi/vandi_stm32/blob/main/guides/FLASHING.md#writing)
- [Bootloader]
  - [Requirements]
  - [Pinout]
  - [Connect]
  - [Erasing]
  - [Writing]

-------------------------------------------------------------------

### SWD

SWD (Serial Wire Debug) is a serial communication protocol used for program and debug STM32 chip.
Compared to JTAG (Joint-Test Action) protocol, SWD is subset of it, providing simpler way.

#### Requirements

To perform SWD flashing, you need some of these things:
- ST-Link device
  - For Nucleo/Discovery boards, it's already equipped with a ST-Link.
  - For Custom board, you can buy standalone ST-Link device like [this](https://www.aliexpress.com/item/32502417987.html).
  - There is some fake/clone ST-Link like [this](https://www.aliexpress.com/item/1005002072117117.html). Use this only if you really dont have other options.
- ST-Link driver. See installation guide
- ST-Link interface. They are two kinds available:
  - Open Source kind. For Linux/BSD, this is recommended. See yourself [here](https://github.com/stlink-org/stlink).
  - Official kind. Recommended for Windows. See installation guide. This guide will focus in this kind.

-------------------------------------------------------------------

#### Pinout

| ST-Link | STM32 | Notes |
|:-------:|:-----:|:-----:|
| VDD | VDD | STLink not providing 3v3 |
| SWCLK | PA14 | Avoid using for GPIO |
| GND | GND | |
| SWDIO | PA13 | Avoid using for GPIO |

-------------------------------------------------------------------

#### Connect

**Warning**: As STM32 chip will be put under reset, check all other connected device which will behave uncontrolably.
Devices like electric motors or it's drivers should turn it off first before put STM32 under reset.

Open _STM32 ST-Link Utility_, then _Target_ -> _Settings_,

From there, make sure:
- _Port_ is set to _SWD 
- _Mode_ is set to _Connect Under Reset_
- _Reset Mode_ is set to _Hardware Reset_

![images](images/st-link0.png?raw=true)

Click _OK_ and ST-Link Utility will show the Flash Memory map contents.

![images](images/st-link1.png?raw=true)

**Notes**: It is possible to connect STM32 without putting the chip under reset.
But it will not give STLink full control of STM32.

-------------------------------------------------------------------

#### Erasing

Erasing or especially Mass/Full Erasing is way to make sure STM32 chip will not leave any left-over value in it's flash memory.
With this, you can expect STM32 will work as intended.
Futhermore, if you use STM32 Flash as EEPROM, this will reset value off all it's address.

To erase chip, go to _Target_ -> _Erase Chip_, then click _OK_ on confirmation dialog.

STLink Utility then will show all program memory section (0x8000000 and above) become their default value (0xFFFFFFFF).

![images](images/st-link2.png?raw=true)

-------------------------------------------------------------------

#### Writing

First you open compilation result file, either Intel Hex (.hex) or Raw Binary (.bin).
Go to _File_ -> _Open file_, then navigate to intended file.

Now STLink Utility will show memory value map of that file in a tab on the right of Devices Memory map tab.

![images](images/st-link3.png?raw=true)

Now, to start program, go to _Target_ -> _Program & Verify_, then Click _Start_ to write to STM32 chip

![images](images/st-link4.png?raw=true)

After that, make sure you see "Verification...OK" on STLink Utility log.

![images](images/st-link5.png?raw=true)

Now you can disconnect the chip from _Target_ -> _Disconnect_

-------------------------------------------------------------------
