Introduction
============

This example outputs a PWM signal to an LED, where the duty cycle of the PWM signal is dynamically
increased or decreased to change the brightness of the LED in order to mimic a heartbeat.
The LED pulsates 100% independently of the CPU after the initial setup with a configurable Beats Per Minute (BPM).
This is done by using three of the core independent peripherals of the ATtiny817: TCB, TCD and CCL.
After the initial setup, the CPU is turned off by using the idle sleep mode, while the LED continues to pulsate.

This example is programmed bare metal, i.e. no START drivers. 
Another version of the example with START drivers is also available.

If the example is run on the ATtiny817 Xplained Pro Development Board, the LED0 will mimic the heartbeat.
If just the ATtiny817 is being used, connect the anode (+) of an LED to VCC and cathode (-) to PB4 to see the
same result. The example should also work with other devices in the tinyAVR-1 Series, but might require reconfiguring
the output pin. Included in the example is a function for changing the BPM and pulse length during run-time.
The main file includes a detailed explanation of how the example works. 
Look at the set_heartbeat_BPM()-function in order to see how it is implemented in practice.

Supported Boards
------------------------

- `ATtiny817 Xplained Pro <http://www.microchip.com/wwwproducts/en/ATtiny817>`_

Running the demo
----------------

1. Press Download Pack and save the .atzip file
2. Import .atzip file into Atmel Studio 7, File->Import->Atmel Start Project.
3. Configure the output method.
4. Build the project and program the ATtiny817 Xplained Pro.
5. Observe that the LED0 pulsates like a heartbeat on the board.