# FuckIO
An opensource fucking machine, that is far superiour to anything on the market.
![Rendering](/images/CAD-Screen.png)

## Specs
* Easy to built just with a 3D-printer, screwdrivers and easy-to-source parts.
* Completely 3D-printable with easy to print parts and almost no supports. Also avoiding tight tolerances to make this accessable as possible. 
* Simple, yet verastile mechanics.
* Based on a linear position drive, not a crank mechanism.
* Double 24V PWM output to drive external systems. Could be lube pumps, vibrators, etc.

## Mechanics
The mechanics is a direct derivative of an other open source project called OSSM. You can also find some informations about this build on their Discord.
https://github.com/KinkyMakers/OSSM-hardware

The motor mount is designed for a NEMA23 servo or stepper motor. Any motor fitting to this standart should work. 

### BOM
* iHSV57-30-18-36-... integrated servo motor from JMC
* 24V Power Brick Mean Well GST280A24-C6P 
* 40cm MGN12H linear rail
* 4x 605-2RS Z2 bearings
* Micro Switch
* GT2 20 Tooth Pulley for 8mm Axis and a 10mm belt
* GT2 belt with 10mm width
* Various M3 and M5 screws and nuts
* Arduino Nano33 IoT

## Electronics
The electronics board is used to interface an external stepper or servo controller. I can't drive a motor directly. It is based around an Arduino Nano33 IoT to enable BLE or WiFi support. It sends 5V DIR, PUL, ENA signals and expects FAULT signal and a homing switch. Two MOSFETs with freewheeling diodes allow PWM control of external systems running of the 24V supply.

## Firmware
The firmware uses a self developed stepper library. It uses the SAMD21 timer perihperials to generate the PUL signal. That way it is possible to create a much higher step rate with no processor loading. Popular Arduino libraries like AccelStepper are not up to the task.

__Do not use this firmware! It is heavly beeing worked on. It will destroy your hardware!__

## Control Software

