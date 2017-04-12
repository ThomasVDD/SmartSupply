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

![SmartSupply](https://github.com/ThomasVDD/SmartSupply/blob/master/Pictures/SmartSupply.png)
