# arduino-fan-controller
Project using Arduino Leonardo + Maxim 6615 + Elegoo 2.8" TFT 

## License
TFT display part of the code is based on Elegoo derived software, see terms and conditions on their website. All other parts including I2C, MAX6615 driver, are under MIT license.

## Goals
* Have finer control over fan speed than off the shelf solutions.
* Support stopping fans when temperature is below threshold and restart fans when above threshold.
* Support external temperature sensors to allow exact placement next to temperature sensitive equipment.
* Support 2 fans with low noise characteristics. Noise dBA was priortized over CFM.
* Basic display showing fan speed and current temperature.

## Hardware
The hardware chosen for this project build was
* Arduino Leonardo
* MAX6615 Dual-Channel Temperature Monitors and Fan-Speed Controllers with Thermistor Inputs
* Elegoo 2.8" TFT with touchscreen and SD card
* be quiet! Silent Wings 3 140mm PWM, BL067
* NTC Thermistor 10K ohm
* Various resistors and caps (see circuit diagram)

## Circuit Design
The design is entirely based on the reference design from Maxim datasheet for MAX6615, page 18. See link: https://datasheets.maximintegrated.com/en/ds/MAX6615-MAX6616.pdf. The only modification required was adding 4.7K pull-up resistors for the TACH1 and TACH2 signals. The transistors on PWM were excluded since fans supported PWM pin directly.

## Hardware Setup
The Elegoo TFT display plugs directly to the Arduino Leonardo header and sample code from Elegoo works out of the box when using the Arduino Uno sample code. A separate PCB was used for the MAX6615 circuit, with wires added between Arduino and PCB for I2C signals and 5V power. Due to pin conflict on Leonardo between I2C and TFT data pins 0 and 1, HW and SW modifications are needed for both to work concurrently. The Elegoo driver pin mapping was modified to move D3/SCL and D2/SDA pins to D1/TX and D0/RX pins respectively. On the Leonardo, serial is handled directly through USB so D1/TX and D0/RX pins are free. Header socket was removed for D3-D0 pins and wires added to connect D0-D1 to TFT.
![Alt text](doc/pcb_prototype.JPG?raw=true "PCB sample")
![Alt text](doc/leo_wiring.JPG?raw=true "Leonardo wire out")
![Alt text](doc/tft_wiring.JPG?raw=true "Leonardo Connector modification")

## Fan Speed Control
Initially the plan was to use the MAX6615 automatic fan control to regulate the fan speed, but after experimentation it seems the 0% duty cycle does not work when below the low temperature threshold (irregardless of the MIN duty setting in the configuration register), it would always set to the minimum duty cycle. So instead I used the manual control and used a lookup table to associate fan speed with temperature. In my use case, the device would be used inside multimedia cabinet which meant my target temperature range was around 20-40 degrees C (68-104F). Below 20C the fan will turn off to reduce noise in cool conditions. Above 40C fan will run at 100%. In theory any PC PWM case fan (i.e. 4-pin) should work, but I didn't test it. The fan speed lookup table is defined in variable fan_speed_map[], and the behavior looks like this:
![Alt text](doc/fan_graph.png?raw=true "Fan Percent Graph")

## Display
The TFT display shows Fan A and B RPM, percentage of speed, and temperature. Bar graphs are used to visually represent fan speed from 0% (no bar) to 100% (full vertical height). During error conditions the text and background will show yellow/red color to highlight error messages.
![Alt text](doc/display_ref.JPG?raw=true "Display")

## Wish List
* Adding audio buzzer when fan failure is detected.
* Having accurate fan tachometer readings (MAX6615 only updates once every minute)
* Extend support for more fans by adding more MAX6615 circuits on different I2C address
* Faster refresh rate. Currently runs at around 1 sec due to display drawing. Without display (I2C only) update would reach around 20-50 ms
