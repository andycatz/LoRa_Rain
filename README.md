# LoRa_Rain
Version 8, 18th September 2021.
PIC18F46K22 LoRa Rain Sensor (Transmitter)
Uses Microchip XC8 compiler.
Transmits when a rain tip occurs or every 2 minutes if there is no rainfall.
 * Keeps a count of the total tips which is transmitted (32 bit unsigned integer).
 * The counter is not reset unless the power is removed.
 * 
 * Sleep current consumption is 12µA with standard PIC18F46K22.
 * Could reduce to 1µA using PIC18LF46K22.
 * Runs on 3V battery.  I recommend two C size Alkaline cells which should last a good couple of years.
 * 
 * AN0 reads battery voltage through a resistor divider (30k/10k) with 1.024V internal reference
 * AN1 reads local temperature through 10k NTC and 10k resistor as a divider from 3.3V
 * RB2 (INT1) is rain tip input.
 
 Default frequency is 866.5MHz, this is easily changed in the code
 The LoRa sync word is 0x55.
 
 Program flow:
 Configure the I/O
 Set up the A to D converter
 Wait 5ms for things to settle
 Read the battery voltage
 Read the local temperature
 Transmit the data using LoRa if the battery is ok
 otherwise flash the on board LED 3 times
 Disable the onboard PIC peripherals
 Go to sleep
 When awakened, go back to the beginning of the program flow.  Update the counter if the interrupt pin woke the PIC.
 
 At any time, an interrupt may occur.  This will wake up the PIC.  If it sees that the
 interrupt is caused by the interrupt pin (that the rain gauge reed switch is connected to)
 then the 32-bit counter is incremented.
 The PIC also wakes up when the watchdog timer times out (about 2 minutes).
 For either of these the program flow will go back to the beginning of the program flow.
