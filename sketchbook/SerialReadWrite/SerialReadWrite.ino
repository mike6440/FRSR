// #include <SoftwareSerial.h>
//SoftwareSerial mySerial(8, 9); // RX, TX

char buf[200];
unsigned long msec;
char a;
byte i;


int chmax=6;


void setup() {
	// Open serial communications and wait for port to open:
	Serial.begin(9600);
	while (!Serial) {
		; // wait for serial port to connect. Needed for native USB port only
	}
	Serial.println("5 sec wait for entry.");
	Serial.setTimeout(5000);
	Serial.println("Goodnight moon!");

	// set the data rate for the SoftwareSerial port
// 	mySerial.begin(9600);
//	mySerial.println("Hello, world..");
}

void loop() { // run over and over
	//mySerial.print("#01");mySerial.write(13);	// command to 4017
	Serial.println("Enter something: ");		// echo to monitor
	// RECEIVE DATA
	i = Serial.readBytesUntil(13, buf, chmax);
 	buf[i] = '\0';
 	if ( i > 0 ) {
 		Serial.print("i = "); Serial.println(i);
 		Serial.print("You entered..."); Serial.println(buf);
 	}
// 	Serial.println("2 You entered ..");
// 	delay(2000);				// delay to next sample
}
