/*
	Written by Steve Elmer

	This program demonstrates how to connect the EVAWGIB EV-ER007 barcode scanner to the Uno, trigger scanning, and read the scanned sequence.
	It uses a pullup button to send the trigger signal once per press.  Each successful scan is then forwarded to Serial.println for display on the PC.

  EV-ER007 Barcode Scanner from EVAWGIB  (EV-ER008 is available with continuous scanning)
		TTL interface, 9600 Baud, no check bit, 8 bits, 1 stop bit

		Pin 1 - no connection      Pin 2 - VCC (+3.3V)
		Pin 3 - GND                Pin 4 - RxD
		Pin 5 - TxD                Pin 6 - D- signal
		Pin 7 - D+ signal          Pin 8 - no connection
		Pin 9 - Beep               Pin 10 - LED (success)
		Pin 11 - Wake              Pin 12 - Trigger (weak pull-up wake/trigger)

		1D Decode: Codebar, Code 11, Code 39/Code 93, UPC/EAN, Code 128/EAN128, Interleaved 2 of 5, Matrix 2 of 5, MSI Code, Industrial 2 of 5, GS1 Databar(RSS)
		2d Decode: QR code, Data Matrix, PDF417

	To connect the EV-ER007 to the Uno, an adapter board is required to convert FFC to wire receptacles.  I used the Uxcell converter board, which is $7.49 on Amazon

		Uxcell FFC FPC 12 pin 0.5mm/1.0mm Pitch DIP 2.0mm PCB Converter Board Couple Extedn Adapter (FPC-12P)
		https://smile.amazon.com/uxcell-Converter-Couple-Extend-Adapter/dp/B07RVD1J1K/ref=sr_1_4

		0.5mm pitch on one side of board, 1.0mm on the other side - use the 0.5mm side for EV-ER007
		both sides connected to the PCB through-hole DIP adapters over their own traces
		Connect two 6-pin receptacles to the through-holes and solder them in place

		flip up the black keeper, install FFC cable blue side up, then close the black keeper
		connect other end of cable to the EV-ER007, blue side up relative to the socket on the scanner
		The EV-ER007 has a keeper opposite the mouth of the socket but it also flips up to release and down to close
		Pin numbers are labeled on the converter board and match the EV-ER007 pinouts documented above.
	
	Connect EV-ER007 Trigger to any digital OUTPUT on the Uno.
	Connect EV-ER007 TxD to Uno Rx.  Uno only supports interrupts on ports 2 and 3, so use one of those for Rx
	Connect EV-ER007 GND to Uno GND
	Connect EV-ER007 VCC to Uno 3.3v

	EV-ER007 Wake is not needed
	EV-ER007 RxD is not needed (EV-ER007 use of serial input is unknown)
	EV-ER007 TTL does not use D+/D- (that's for USB)
	EV-ER007 LED and EV-ER007 Beep are optional. 

	Passing -1 to SoftwareSerial works so no need to allocate a pin for Tx when it's not going to be used.  
	Do not use SoftwareSerial inverted signaling, the EV-ER007 is standard.

	Trigger the EV-ER007 by sending HIGH, waiting 1ms, and then returning to LOW.  Failing to wait 1ms causes poor behavior.  (Even 800us was not 100% reliable.)

	Once triggered, the scanner looks for a barcode for about half a second.
	If a barcode is found then the scanner's LED & light then go out, the barcode is sent over Pin 4, and Pin 10 goes HIGH briefly.
	After timeout is reached with no scan, the scaner's LED & light go out and no result is sent.

	I was not able to use Pin 9 to drive the LED.  More experimentation is required in order to drive a sound generator.

	Function buttonPressed debounces a pullup button and is reliable.  Debounce delay of 5ms arrived at experimentally, smaller values exhibited
	erroneous presses.  A larger value may be used if you get more than one TRUE per button press.

	Function readBarcode reliably reads the whole barcode sequence when called.  A full scan can be read in less than 1ms, so no need to deal with multi-loop() reads.

	Function serialPrintln can be used for convenient debugging e.g. serialPrintln("Time = %u", millis());

	Spurious results may happen immediately after initiating serial communications with the Uno.  The very first scanned sequence may be corrupted.  
	Killing and restarting the modem application on the PC seems to trigger a scan without a button press.

	https://learn.sparkfun.com/tutorials/2d-barcode-scanner-breakout-hookup-guide/all#de2120-arduino-library looks like the same module, check out the
	barcode configurations.  Look at reading area and motion sensor mode.
		https://github.com/sparkfun/SparkFun_DE2120_Arduino_Library/tree/main/barcodes
*/

#define rx 2
#define sw 4
#define tr 5

#include <SoftwareSerial.h>
SoftwareSerial scan = SoftwareSerial(rx, -1, false);

// Max 500 characters in final message
void serialPrintln(char *fmt, ...) {
	va_list args;
	char buf[500];
	va_start(args, fmt);
	vsprintf(buf, fmt, args);
	va_end(args);
	Serial.println(buf);
}

// Exactly one pullup button debounced - be sure to use INPUT_PULLUP in setup()
// Returns true when the button is pressed
int buttonPressed(int pin) {
	static int lastReading = HIGH;  // initialized at program start, scoped to only this function
	static unsigned long t = 0;     // initialized at program start, scoped to only this function

	int reading = digitalRead(pin);
	if (reading != lastReading) {
	  if (t == 0) {
			t = millis();
		} else if (millis() - t > 5) {
			lastReading = reading;
			t = 0;
			return reading == LOW;
		}
	}
	return false;
}

// 1ms minimum delay is *required* for the scanner to reliably be triggered
void triggerBarcode() {
	digitalWrite(tr, HIGH);
	delay(1);
	digitalWrite(tr, LOW);
}

// Reliably read the entire sequence by checking available() again after each read() loop
// No overflow checking, make sure buf is big enough for your data!
// Returns pointer to sequence or NULL if no sequence is available
char *readBarcode(char *buf) {
	int scanned;
	int num = 0;
	while (scanned = scan.available()) {
	  while (scanned--) {
			buf[num++] = scan.read();
		}
		delay(1);
	}
	if (num > 0) {
		buf[num] = '\0';
		return buf;
	}
	return NULL;
}

void setup() {
	pinMode(tr, OUTPUT);
	pinMode(rx, INPUT);
	pinMode(sw, INPUT_PULLUP);

  Serial.begin(9600);
	scan.begin(9600);
}

void loop() {
	char buf[100];
  if (buttonPressed(sw)) triggerBarcode();
  if (readBarcode(buf)) Serial.println(buf);
}
