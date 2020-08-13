<!-- Please do not change this logo with link -->
[![MCHP](images/microchip.png)](https://www.microchip.com)

# Realistic Heartbeat

This example outputs a PWM signal to an LED, where the duty cycle of the PWM signal is dynamically increased or decreased to change the brightness of the LED in order to mimic a heartbeat. The LED pulsates 100% independently of the CPU after the initial setup with a configurable Beats Per Minute (BPM). This is done by using three of the core independent peripherals of the ATtiny817: TCB, TCD and CCL. After the initial setup, the CPU is turned off by using the idle sleep mode, while the LED continues to pulsate.

This example is programmed using START drivers, i.e. not bare metal. Another bare metal version of the example is also available.

If the example is run on the ATtiny817 Xplained Pro Development Board, the LED0 will mimic the heartbeat. If just the ATtiny817 is being used, connect the anode (+) of an LED to VCC and cathode (-) to PB4 to see the same result. The example should also work with other devices in the tinyAVR-1 Series, but might require reconfiguring the output pin. 

Included in the example is a function for changing the BPM and pulse length during run-time. The main file includes a detailed explanation of how the example works. Look at the set_heartbeat_BPM()-function in order to see how it is implemented in practice.


## Related Documentation

- [ATtiny817 Device Page](https://www.microchip.com/wwwproducts/en/ATtiny817)


## Software Used

- [Atmel Studio](https://www.microchip.com/mplab/avr-support/atmel-studio-7) 7.0.2397 or later
- [ATtiny DFP](http://packs.download.atmel.com/) 1.6.316 or later
- AVR/GNU C Compiler (Built-in compiler) 5.4.0 or later


## Hardware Used

- [ATtiny817 Xplained Pro](https://www.microchip.com/DevelopmentTools/ProductDetails/attiny817-xpro)
- Micro-USB cable (Type-A/Micro-B)



## Operation

1. Connect the ATtiny817 Xplained Pro board to the PC using the USB cable.

2. Download the zip file or clone the example to get the source code.

3. Open the .atsln file in Atmel Studio.

4. Build the solution and program the ATtiny817. 

5. Observe that the LED0 pulsates like a heartbeat on the board.



## Conclusion
This example has now shown how to use a PWM signal to increase and decrease the brightness of the LED in order to mimic a heartbeat.
