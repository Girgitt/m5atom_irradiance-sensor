# m5atom_irradiance-sensor
A tiny project to test vlem7700 sensor as a solar irradiance meter. 
![image](https://github.com/user-attachments/assets/47ed2ab9-3031-4166-9c36-3ba79692ffc2)
_AI render illustrating the concept_ 

Connect the sensor to the standard I2C pins on the M5Atom:
- SDA: GPIO 25
- SCL: GPIO 21
- Power: 3V3 and GND

Compile the project in VS Code using PlatformIO and upload it to the device.
Use the serial monitor to verify operation. If you see the message "VEML7700 not found", try swapping the SDA and SCL pins, as it likely indicates an I2C initialization issue.

A properly working setup outputs:

1. Serial output example:
```
Lux: 570958 | W/m² (1–999): 453 | W/m² raw: 453
```
1. A 5×5 dot matrix bargraph display showing irradiance as three stacked bars:
- Red: hundreds
- Yellow: tens
- Green (2) and White (1): units
  
limited capacity of the 5x5 dot matrix display supports showing value only up to 999.
 
```verbatim
example for number 525 (5*red, 2*yellow, 2*green+1*white):
rr___
rrr__
y____
y____
ggw__

example for number 108 (1*red, 4*green):
r____
_____
_____
_____
gggg_
```
