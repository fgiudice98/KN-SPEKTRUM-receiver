# KN-SPEKTRUM-receiver
SPEKTRUM receiver using digispark and NRF24L01

## Wiring
P0 is the SPEKTRUM2048 output
![Wiring](wiring.png?raw=true "Wiring diagram")

## Important notice
Make sure to:
1. Install the `Modified DigisparkSoftSerial` library
1. Check the baudrate using `digispark_baud` sketch and some type of frequency measuring device such as:
 * Oscilloscope (best)
 * Another Arduino with a good frequency counter sketch
1. Compile using `"Digispark (16MHz - No USB)"`