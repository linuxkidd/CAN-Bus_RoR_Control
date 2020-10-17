# CAN-Bus Roll-off-Roof (and Flap) Astronomy Controller

(This document is a work in progress)

## Scope:
- This project provides a fully functioning [ASCOM Alpaca](https://ascom-standards.org/api/#/Dome%20Specific%20Methods/get_dome__device_number__shutterstatus) compliant roll-off-roof control mechanism, with or without Flap control.
- While not an overly advanced project, it does have many parts which work together, thus appears quite complex at first glance.
- Basic Linux, Arduino and miscellaneous electronics experience will make assembly much easier.

## Basic overview:
- The Roof ( and Flap if you use one ) is controlled by an Arduino Nano + 2 Relay board
    - One relay is used to enable/disable movement
    - The other relay is used to control direction of movement of the roof/flap ( open/close )
- The Roof / Flap controllers communication occurs via [CAN-Bus](https://en.wikipedia.org/wiki/CAN_bus).
    - CAN-Bus has been used in industrial scenarios since the early 80's, and in Automobiles since ~2004.
    - It is a differential signal physical protocol, meaning it is very resilient to noise introduced by electro-magnetic interference
    - CAN-Bus is thus, fairly robust -- and can be made fault tolerant at more expense.
- The controlling computer in this project is a Raspberry Pi with a CAN-Bus card added on.
    - Technically, any linux system running Python3 can be used, but that is outside the scope of this guide.


## NOTE:
- I use DC motors and thus, a [Variable Speed, Reversible, DC Motor Controller](https://www.amazon.com/gp/product/B073S3P1FY)
- You may opt for additional, higher power handling Relays to add into an AC motor control system

## Parts lists:

### Roof / Flap controller:
- [Arduino Nano](https://www.amazon.com/gp/product/B0775XQXRB) - or similar
- [Dual Relay Board](https://www.amazon.com/gp/product/B07DYSYRFR) - or similar
- [MCP2515 CAN-Bus Board](https://www.amazon.com/gp/product/B01IV3ZSKO) - or similar
- [Limit Switches](https://www.amazon.com/TWTADE-ME-8108-Adjustable-arduino-Momentary/dp/B07NL12NL5)
- [Nano Screw Terminal Adapter](https://www.amazon.com/gp/product/B0788MLRLK) - not requried, but makes assembly easier for non-solder skilled persons

### Control Computer:
- [Raspberry Pi 3B+](https://www.amazon.com/ELEMENT-Element14-Raspberry-Pi-Motherboard/dp/B07BDR5PDW)
    - Yes, there are more powerful ( Pi 4 ) systems, but this one is plenty powerful and less power hungry
- [PiCAN2 + RPi Enclosure](https://copperhilltech.com/plastic-enclosure-for-pican2-and-raspberry-pi-2-3/)
    - Two options for the RPi CAN-Bus board, one with power supply built in, one without:
        - [PiCAN2 CAN-Bus Hat - no Power Supply](https://copperhilltech.com/pican-2-can-bus-interface-for-raspberry-pi/)
        - [PiCAN2 CAN-Bus Hat + WITH Power Supply](https://copperhilltech.com/pican2-can-interface-for-raspberry-pi-with-smps/)
    - The power supply version allows you to power your Roof/Flap control boards and the RPi all off the same 12v battery power.
    - If you'd rather power your Raspberry pi from a different power source, feel free to buy the unit without the Power Supply on-board.
- [High Endurance micro-SD card](https://www.amazon.com/dp/B084CJLNM4)

### Miscellaneous bits:
- 4 Conductor Wire ( +12v, Ground, CAN-High, CAN-Low )
- 4 Position connectors for easy connect/disconnect of low power connections

