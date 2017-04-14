# SmartSupply
SmartSupply is a digital battery powered lab powersupply. It is Arduino compatible and can be controlled via PC over USB.


Specifications:
 * 0 - 1A,  steps of 1 mA  (10 bit DAC)
 * 0 - 20V, steps of 20 mV (10 bit DAC) (true 0V operation)
 * Voltage measurement: 20 mV resolution (10 bit ADC)
 * Current measurement: 
      -     < 40mA:  10uA resolution (ina219)
 			< 80mA:  20uA resolution (ina219)
 			< 160mA: 40uA resolution (ina219)
			< 320mA: 80uA resolution (ina219)
			> 320mA: 1mA  resolution (10 bit ADC)

Features: 
 * Constant voltage and constant current modes
 * Uses a low noise linear regulator, preceded by a tracking preregulator to minimize power dissipation
 * Aluminium case end panel used as heatsink 
 * Use of handsolderable components to keep the project accessible 
 * Powered by ATMEGA328P, programmed with Arduino IDE
 * PC communication via Java application over micro usb
 * Powered by 2 protected 18650 Lithium Ion cells
 * 18 mm spaced banana plugs for compatibility with BNC adapters

![SmartSupply](https://github.com/ThomasVDD/SmartSupply/blob/master/Pictures/Front.jpg)
![SmartSupply](https://github.com/ThomasVDD/SmartSupply/blob/master/Pictures/Inside.jpg)
![SmartSupply](https://github.com/ThomasVDD/SmartSupply/blob/master/Pictures/Back.jpg)
![SmartSupply](https://github.com/ThomasVDD/SmartSupply/blob/master/Pictures/PC.jpg)


This project would never have been possible without the help of some people, so a shootout seems appropriate.

First of all, a huge thanks to Johan Pattyn for producing the prototypes of this project. 
Also Cedric Busschots and Hans Ingelberts deserve credit for the help with troubleshooting.
And of course, a special thanks to David L. Jones for releasing his schematics under an open source license and sharing all his knowledge.

