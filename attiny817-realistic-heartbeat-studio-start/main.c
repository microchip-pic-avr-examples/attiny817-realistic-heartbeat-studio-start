/*
 * -------------------------------------------------------------------------------------------------------------------------
 *                                    REALISTIC HEARTBEAT LED EXAMPLE (START DRIVERS)
 * -------------------------------------------------------------------------------------------------------------------------
 * This example outputs a PWM signal to an LED, where the duty cycle of the PWM signal is dynamically increased or
 * decreased to change the brightness of the LED in order to mimic a heartbeat. The LED pulsates 100% independently of
 * the CPU after the initial setup with a configurable Beats Per Minute (BPM). This is done by using three of the core
 * independent peripherals of the ATtiny817: TCB, TCD and CCL. After the initial setup, the CPU is turned off by using
 * the idle sleepmode, while the LED continues to pulsate.
 *
 * This example is programmed using START drivers, i.e. not bare metal.
 * Another bare metal version of the example is also available.
 *
 * If the example is run on the ATtiny817 Xplained Pro Development Board, the LED0 will mimic the heartbeat. If just the
 * ATtiny817 is being used, connect the anode (+) pin of an LED to VCC and cathode (-) to PB4 to see the same result.
 * The example should also work with other devices in the tinyAVR-1 Series, but might require reconfiguring the output
 * pin. Included in the example is a function for changing the RPM and pulse length during run-time. Below is a detailed
 * explanation of how the example works. Look at the set_heartbeat_BPM()-function in order to see how it is implemented
 * in practice.
 *
 * -------------------------------------------------------------------------------------------------------------------------
 *                                          	DETAILED EXAMPLE EXPLANATION
 * -------------------------------------------------------------------------------------------------------------------------
 *
 * Two Timer/Counters are used to generate the heartbeat pulse output to the LED: TCB and TCD.
 * The TCB generates *one* PWM signal: TCB waveform output (TCB_WO) with a frequency (f_TCB) and duty cycle (TCB_WO_dc).
 * The TCD generates *two* PWM signals: waveform output A (TCD_WOA) and waveform output B (TCD_WOB), which have the
 * *same* frequency (f_TCD), where f_TCD > f_TCB. However, TCD_WOA and TCD_WOB has a different duty cycle (TCD_WOA_dc
 * and TCD_WOB_dc) and phase. The phase shift is represented by the percentage of the period between the falling edge of
 * TCD_WOA and the rising edge of TCD_WOB and hereby called phase_dc. One period of each PWM signal is illustrated below
 * (where __-- indicates a transition from low to high output and --__ vice versa):
 *
 * 						        |  TCB_WO_dc   |
 * 						TCB_WO  ----------------_____________________________________________ f_TCB
 * 						TCD_WOA ___________-----------__________________________________      f_TCD
 * 						TCD_WOB ___________________________________---------------------      f_TCD
 * 						                   |TCD_WOA_dc|  phase_dc  |     TCD_WOB_dc     |
 *
 *
 * By using the Configurable Custom Logic (CCL) peripheral, a Look Up Table (LUT) is configured to continuously do an
 * AND operation between either TCB_WO and TCD_WOA or TCB_WO and TCD_WOB. In boolean algebra terms:
 *
 *      									 OUTPUT = TCB_WO * (TCD_WOA ^ TCD_WOB).
 *
 * 						TCB_WO  -----------------____________________________________________ f_TCB
 * 						TCD_WOA ___________-----------__________________________________      f_TCD
 * 						TCD_WOB ___________________________________---------------------      f_TCD
 * 						OUTPUT  ___________------____________________________________________   -
 * 						                   |O_dc|
 *
 * Since f_TCD > f_TCB, the TCB_WO signal will drift relative to the TCD_WOA and TCD_WOB signals, changing the overlap.
 * In addition, the duty cycles are set such that TCD_WOA_dc < phase_dc < TCB_WO_dc < TCD_WOB_dc. This means that the
 * duty cycle of the OUTPUT signal (O_dc) will gradually increase and decrease. More specifically, when TCB_WO start to
 * overlap with TCD_WOA, O_dc will go from 0 and increase to TCD_WOA_dc, then start to decrease again. Just before the
 * duty cycle becomes zero again, TCB_WO will overlap with TCD_WOB, increasing O_dc until it is equal to TCB_WO_dc.
 * Since TCD_WOB_dc > TCB_WO_dc, O_dc will continue to be equal to TCB_WO_dc for a while. Eventually, the drifting of
 * the signals relative to each other results in a decrease of O_dc until it becomes zero for a long while. The cycle
 * will then repeat. An attempt to illustrate this is shown below:
 *
 * 	    TCB_WO  ---_______---_______---_______---_______---_______---_______---_______---_______---_______---_______
 * 	    TCD_WOA ___-________-________-________-________-________-________-________-________-________-________-______
 * 	    TCD_WOB ______----_____----_____----_____----_____----_____----_____----_____----_____----_____----_____----
 * 	    OUTPUT  ____________-________-________-___________-________--_______---_______---_______--________-_________
 *
 * As OUTPUT is used to turn on/off the LED, the result is that the LED will be turned off, then increase in light
 * intensity with a short pulse, then a higher peak intensity with a bigger pulse shortly after - mimicking a heartbeat.
 * The pulses will manifest themselves with a frequency (in Beats Per Minute (BPM)) given by:
 *
 *      										BPM = 60*(f_TCD - f_TCB)
 *
 * Changing the pulse width of the heartbeat is done by increasing or decreasing all the duty cycles described above
 * such that the ratio between the duty cycles remains the same. Finally, the LED0 on the ATtiny817 Xplained Pro
 * development board is low side driven, i.e. the anode (+) is connected to VCC and the cathode (-) is connected to an
 * IO pin. Thus, in order to turn on the LED we need to pull the IO pin low. So, in order to make the LED pulsate like a
 * heartbeat, the OUTPUT signal has to be inverted before it is propagated to the IO pin. This is achieved by either
 * inverting the boolean expression in the LUT in the CCL [!(TCB_WO * (TCD_WOA ^ TCD_WOB))] or inverting the IO pin
 * output by setting the INVEN bit in the PINCTRL register of the PORT peripheral.
 *
 *              LED0 Light
 *               Intensity
 *                 ^
 *                 |               ____                                    ____
 *                 |              /     \                                 /     \
 *                 |     __      /       \                       __      /       \
 *                 |   /    \   /         \                    /    \   /         \
 *                 |  /      \_/           \                  /      \_/           \
 *                 | /                      \________________/                      \________________
 *                 |----------------------------------------------------------------------------------> Time
 *                  <---------------BPM period--------------> <-----Pulse Lenght----->
 *
 * -------------------------------------------------------------------------------------------------------------------------
 *
 * -------------------------------------------------------------------------------------------------------------------------
 */

/*
 * Constants for calculating values to set a new BPM,
 * must be changed if clock frequency or Timer/Counter
 * clock source divisions are changed.
 */
#define OSC_20M 20000000
#define F_CPU OSC_20M / 32
#define F_TCB_CLK F_CPU / 2
#define F_TCD_CLK OSC_20M / 4

#define BPM 60
#define BPM_PULSE_LEN_PCT 40

#include <atmel_start.h>
#include <avr/sleep.h>
#include <math.h>

float set_heartbeat_BPM(uint8_t ideal_BPM, uint8_t pulse_length_pct);
void  set_TCB_CCMP(uint8_t low_byte, uint8_t high_byte);

int main(void)
{
	/*
	 * Initializes MCU, drivers and middleware in addition to initializing the BPM example with
	 * BPM = 54 and pulse length = 40%
	 */
	atmel_start_init();

	/* Only need to be called if changing the default BPM period and pulse_length is desirable */
	set_heartbeat_BPM(BPM, BPM_PULSE_LEN_PCT);

	while (1) {
		/* Start Idle Sleepmode (turn off CPU) */
		sleep_mode();
	}
}

/*
 * This function sets the BPM of the realistic heartbeat LED by finding the closest match
 * to the ideal_BPM input parameter given the desired pulse_length_pct (%) input and the limitations
 * on the resolution of the two Timer/Counters used with their current clock input (see the
 * F_CPU, F_TCB_DIV and F_TCD_DIV macros).
 *
 * Valid input for ideal_BPM: 18-255, pulse_length_pct: 10-100.
 * The function returns the new BPM for comparison with the ideal_BPM input.
 *
 * If changing BPM/pulse_length during runtime is not required, then this function can be modified and
 * run on a computer to return/print the register values you need for a specific BPM + pulse_length and
 * then use these values to set the registers directly when initializing the MCU to save resources.
 */
float set_heartbeat_BPM(uint8_t ideal_BPM, uint8_t pulse_length_pct)
{

	float ideal_BPM_freq = (float)ideal_BPM / 60;
	float pulse_length   = (float)pulse_length_pct / 100;

	/*
	 * Since BPM = 60*(f_TCD - f_TCB), we can test all reasonable/possible values
	 * for f_TCB, then calculate what the ideal_f_TCD should be to get the correct BPM. By comparing the
	 * ideal_f_TCD to the closest possible f_TCD for all the values of f_TCB we can find the parameters
	 * which results in the smallest error between the ideal_BPM and the new BPM.
	 * TCB_top + 1 decides the frequency of TCB (f_TCB) and TCD_top + 1 decides the frequency of TCD (f_TCD).
	 */

	float   best_error   = 255; // Arbitrary high value to avoid checking a "first iteration flag" each loop
	uint8_t best_TCB_top = 0;

	for (uint8_t TCB_top = 255; TCB_top > 65; TCB_top--) {
		float    f_TCB       = (float)F_TCB_CLK / ((float)TCB_top + 1);
		float    ideal_f_TCD = f_TCB + ideal_BPM_freq;
		uint16_t TCD_top     = floor((F_TCD_CLK / ideal_f_TCD) - 1);
		float    f_TCD       = F_TCD_CLK / ((float)TCD_top + 1);

		float error = fabs(f_TCD - ideal_f_TCD); // fabs() returns absolute value of a float
		if (error < best_error) {
			best_error   = error;
			best_TCB_top = TCB_top;
		}
	}

	/*
	 * Recalculate values above based on best_TCB_top to reduce the amount of store instructions
	 * each time the current error is lower than the best error in the loop above.
	 */

	float    f_TCB       = (float)F_TCB_CLK / ((float)best_TCB_top + 1);
	float    ideal_f_TCD = f_TCB + ideal_BPM_freq;
	uint16_t TCD_top     = floor((F_TCD_CLK / ideal_f_TCD) - 1);
	float    f_TCD       = F_TCD_CLK / ((float)TCD_top + 1);

	/*
	 * TCD is running in One Ramp Mode which means that the counter will count up to the value of TCD0.CMPBCLR
	 * before resetting. In other words the value of TCD0.CMPBCLR decides the TCD period and thus the f_TCD.
	 * TCD0.CMPASET decides the "dead time" for waveform output A (WOA) (which counter value results in setting WOA
	 * high), and TCD0.CMPACLR decides the "on time" for WOA (at which counter value to set WOA low). Likewise, the
	 * TCD0.CMPBSET decide the "dead time" for WOB and TCD0.CMPBCLR decide the "on time" for WOB. TCD One Ramp Mode
	 * output looks like this:
	 *
	 * TCD_WOA ______________------------_______________________________
	 * TCD_WOB ________________________________________-----------------
	 *         dead_time_A  | on_time_A | dead_time_B |    on_time_B   |
	 *                      ^           ^             ^                ^
	 *                   CMPASET     CMPACLR       CMPBSET          CMPBCLR
	 * counter value--------------------------------------------------->
	 *
	 * Since we want a specific f_TCD, we have first calculated that CMPBCLR = TCD_top,
	 * then we work backwards to achieve the correct "on time" and "dead time" for both TCD_WOA and TCD_WOB.
	 * This is done by first using the desired pulse_length to calculate a duty cycle for on_time_A,
	 * dead_time_B and on_time_B in addition to the on_time of the TCB_WO.
	 */

	float TCD_on_time_B   = pulse_length / 2;
	float TCD_on_time_A   = pulse_length * 0.2;
	float TCD_dead_time_B = pulse_length - TCD_on_time_A - TCD_on_time_B;
	// TCD_dead_time_A = 1 - pulse_length, but it is not needed for further calculation
	float TCB_on_time = pulse_length * 0.4;

	/* When to set PWM output B low, depending on the desired f_TCD*/
	TCD0.CMPBCLR = TCD_top;

	/* When to set PWM output B high, depending on the desired duty cycle*/
	TCD0.CMPBSET = TCD0.CMPBCLR - floor((TCD0.CMPBCLR + 1) * TCD_on_time_B);

	/* When to set PWM output A low, depending on the desired duty cycle*/
	TCD0.CMPACLR = TCD0.CMPBSET - floor((TCD0.CMPBCLR + 1) * TCD_dead_time_B);

	/* When to set PWM output A high, depending on the desired duty cycle*/
	TCD0.CMPASET = TCD0.CMPACLR - floor((TCD0.CMPBCLR + 1) * TCD_on_time_A);

	/* Restart TCD for changes to take effect if the TCD is already running*/
	TCD0.CTRLE |= TCD_RESTART_bm;

	set_TCB_CCMP(best_TCB_top, (uint8_t)floor((best_TCB_top + 1) * TCB_on_time));

	return 60 * (f_TCD - f_TCB); // New BPM value
}

/*
 * Change the value of the compare/capture registers in TCB when already running in 8-bit PWM mode according
 * to the recommendations of the ATtiny817 datasheet, since the transition may output invalid values.
 */
void set_TCB_CCMP(uint8_t low_byte, uint8_t high_byte)
{

	/* 1. Disable the peripheral */
	TCB0.CTRLA &= ~TCB_ENABLE_bm;

	/* 2. Write compare/capture register */

	/* On 16 bit registers (such as TCB0.CNT and TCB0.CCMP) you have to first write the low byte then the high byte */

	/* When to set PWM output on TCB low */
	TCB0.CCMPL = low_byte;

	/* When to set PWM output on TCB high */
	TCB0.CCMPH = high_byte;

	/* 3. Reset the counting register */
	TCB0.CNTL = 0x0;
	TCB0.CNTH = 0x0;

	/* 4. Re-enable the peripheral */
	TCB0.CTRLA |= TCB_ENABLE_bm;
}
