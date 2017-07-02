//161213--We modify the rosr arduino program for the frsr circuit board.

#define PROGRAMNAME "frsr_main"
#define VERSION		"7"
#define EDITDATE	"170208"
#define	EEPROM_ID  6
//v2 - add adc stuff in action
//v3 - Nsweep in eeprom
/*v4 - Nchans=7, int adcmv[NCHANS*NSWEEP] shoehorn in full sweep.
//		changed ADS1115_CONVERSIONDELAY to 1 to speed up adc for sweep
// 		Change the i2c clock to 400KHz
//  	conversion time for 250 samples is 2.364 sec.
*/
/*v5 - The ads1115 conversion does not work,
//		changed ADS1115_CONVERSIONDELAY back to 8 for reliable operation
//		This version focuses on the heater and on the motor control.
 √ -- motor and shadowband stop at nadir
 √ -- power mfr head and check ouput.
 √ -- menu sweep and minimum detect
 √ -- sweep minumum and globals
 √-- shadowratio and sweep decision
 √-- block averaging
 √-- output packet
*/
/*v6
 √-- Add inverting op amps to mimic the frsr-v2
*/ /*v7
	Switch to 12bit adc ads1015.  ADS1015_a0 CONVERSIONDELAY = 1
	fix thermistor 1 & 2 in mfr head
 	tilt sensor
 */

//NOTE ====== INCLUDES
#include <string.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

//Added for the ADS
///Users/rmr/Documents/Arduino/libraries/Adafruit_ADS1X15/Adafruit_ADS1015.h
//To achieve higher speed change
// #define ADS1115_CONVERSIONDELAY         (8) to (2)

#include <Wire.h>
#include <Adafruit_ADS1015.h>
Adafruit_ADS1015 ads0;  	// Construct an ads1015 at the default address: 0x48
Adafruit_ADS1015 ads1(0x49);	// construct an ads1115 at address 0x49
Adafruit_ADS1015 ads2(0x4A);	// ad2, u14, construct an ads1115 at address 0x4A

//NOTE ====== DIGITAL PIN ASSIGNMENTS
#define RAIN	4			// RAIN DETECTOR -- Drives the LED
#define NADIR	5		 // INPUT Nadir Switch 1
#define MOTOR	9		  // shadowband motor on/off
#define HEATER	10		   // mfr heater on/off
#define LED11	24		// Green Scan Drum motor
#define LED12	26		// Red shutter motor
#define LED21	28		// green BB2 heater on
#define LED22	30		// red BB1 heat on
#define LED31	32		// green -- continuous on	power
#define LED32	13		// red -- heartbeat. red 5REF
#define REFSW	22		// BB thermistor reference voltage
#define	SHADOWLIMIT	10
#define SHADOWCHANNEL 3	// filter 2 500 nm
#define	NBLKS 	11		// number of avg'd block per side of shadow
#define	NDATAFILE 370  	// size of data storage area
//23bins * 2char/bin * 7chans + 14chars/globals + ~20 header = 361

//NOTE ==== ANALOG ASSIGNMENTS
// three 4-chan 16 bit adc, adc#=(unsigned)(x/4), chan#=x%4;
#define ADCMAX  12		 // 12 normally. maximum number of ADC channels
#define	NSWEEP  250
#define NCHANS  7
// 250*71750 samples in 3 sec  583 samples/sec (583 Hz)
#define T1adc  0		// MFR Temp 1
#define T2adc  1		// MFR T2
#define mfr1  2		// MFR CHAN 1 (filter 1)
#define NRF  3		// 5REF/2
#define mfr2  4		// MFR CHAN 2 (filter 2)
#define mfr3  5		// MFR CHAN 3 (filter 3)
#define mfr4  6		// MFR CHAN 4 (filter 4)
#define mfr5  7		// MFR CHAN 5 (filter 5)
#define mfr6  8		// MFR CHAN 6 (filter 6)
#define NVin  9		// VIN/4
#define Arain  10	// Rain / 4
#define mfr7  11	// MFR CHAN 7 (filter 7)

//NOTE  MISC CONSTANTS
#define OK  1
#define NOTOK  0
#define MISSING  -999
#define POS  1
#define NEG  -1
#define CCW  1			// forward direction
#define CW  -1			// reverse direction
#define STOP  0			// motor stop
#define ON  1
#define OFF  0
#define OPEN  1
#define CLOSED  0
#define SPACE  ' '
#define ZERO  '0'
// test menu
#define TEST  -1
#define RUN  1
// TEMPERATURE
#define WARMUPMILLISECS  50
#define ADCREADWAIT  100
#define TMAX  70
#define TMIN  -5
// SPECTRON TILT
#define C1  0.0129
#define C2  0
#define C3  -0.0000000003
#define P1  1 // use -1 for a reversed direction
#define P0  0
#define R1  1 // use -1 for a reversed direction
#define R0  0


//=========================================================================
//NOTE ====== GLOBAL VARIABLES
//=========================================================================
// system time
unsigned long msstart;	//set at start, milliseconds
unsigned long menustart;

// user i/o
char	RunMode;

// READING THE ADC
byte Chan[]={2,4,5,6,7,8,11};
int16_t adcmv[NCHANS*NSWEEP];			// All filter outputs
unsigned int Imin[NCHANS];
double	Smin[NCHANS];
double	G[NCHANS][2];	//sweep globals
double	Smean[NCHANS], Sstd[NCHANS];
double	Shadow[NCHANS];
byte	Nsz[NBLKS]={5,5,5,5,5,10,10,10,20,20,30};
int		sweepBlk[NBLKS*2+1][NCHANS];
char	datafile[NDATAFILE];
unsigned int 	df_index;

// MOTOR
byte MotorState;

unsigned long SampMicroSecs;
float	Sweeptime;

// ADC16 limits for temperature cal. volts
double AdcThreshold[2] = {0, 6000};

// TILT
double pitav = 0;
double pitsd = 0;
double rolav = 0;
double rolsd = 0;
int npit = 0;
int nrol = 0;

// OTHER TEMPS
double	T1 = 0; // MFR temp T1
double	T2 = 0; // MFR temp T2

// RAIN
unsigned long  millisec_rain;	// the clock count to open.	 0 if open
char	RainState;
unsigned long secs_lapsed;


//NOTE ====== EEPROM
// Scan angles: abb1, abb2, asky, asea, a1, a2
const float default_rain_threshold = .090;
const unsigned long default_rainsecs = 600;
const unsigned default_Nsweep = NSWEEP;
const float default_pitch_correct = 0; //v29
const float default_roll_correct = 0;  //v29
const char default_mfrheat = OFF;
const double	default_Rref[2] = {
  10000, 10000
}; // T1, T2
const byte	default_shadowlimit = SHADOWLIMIT;
const byte	default_shadowchannel = SHADOWCHANNEL;

struct eeprom {
  byte id, FrsrState;
  byte shadowlimit, shadowchannel;
  unsigned long Nsweep;	 // total samps over the hemisphere.
  float rain_threshold;
  float pitch_correct, roll_correct; 
  unsigned long rainsecs;
  char mfrheat; // on or off
  double Rref[2];
};
struct eeprom ee;
int eeSize;

int istart; // set to 1 at startup so we can go to a menu.
unsigned int checksum;



//NOTE ====== FUNCTIONS
// i/o
void		Action(char*);
void		MFRHeater(int);
void		CheckRain(double *, unsigned long *);
unsigned int checksum_nmea(char *);
unsigned long ElapsedTime (char *);
void		EepromDefault(void);
void		EepromStore(void);
void		EepromRead(void);
void		EepromPrint(void);
char *		floatToString(char *, double, byte, byte);
void		GetAdc (int16_t *, byte chan );   // ads adc
double 	GetAdcVolts(byte ch);	 // ads adc
unsigned int GetThermCoefs(unsigned int, double *coef );   // ads adc
long		Hex2Dec(char *);
void		MeanStdev(double *, double *, int, double );
void 		NadirShutdown(void);
void 		NadirTime(void);
void		PrintProgramID(void);
void		PrintBytes(char *);
void		ReadTilt(double*, double*);
void		ReadSweep(unsigned int *, int , unsigned long );
unsigned	Ref(int);
double		ShadowRatio(byte chan);
int			sign (float);
double		SteinhartHart(double beta[], double r);
unsigned int SweepMinimum(byte chan);
double 		SweepNoise(byte chan, byte, double *mnx, double *stdx); // run after SweepMinimum
void 		SweepGlobals(byte chan, byte nsum, double *g1, double *g2);
void		SweepPack(void);
double		ThermistorTemp(byte ,double Rref);
void FillAdc(void);

//============================================================================
void setup() {
  // USER Serial setup
  Serial.begin(115200);
//	 ENCODER serial setup
//	 Serial1.begin(9600);	 // serial
  // TILT
  Serial2.begin(19200);	  // serial
//	 KT15
//	 Serial3.begin(9600);	 // serial kt15

  // sign on
  PrintProgramID();

  // Set system clock
  msstart = millis();

  // 5REF START OFF
  Ref(OFF);

  // MISC
  pinMode(LED11, OUTPUT);
  pinMode(LED12, OUTPUT);
  pinMode(LED21, OUTPUT);
  pinMode(LED22, OUTPUT);
  pinMode(LED31, OUTPUT);  // HBT
  pinMode(LED32, OUTPUT);
  pinMode(RAIN, INPUT); // LED OUTPUT

  // NADIR SWITCHES
  pinMode(NADIR, INPUT);		   // shadowband nadir switch

  // set mem for eeprom
  eeSize = sizeof(struct eeprom);
  //  Serial.print("Check EEPROM.  ");
  EepromRead();
  if (ee.id != EEPROM_ID ) {
		Serial.println("ID NO  match => use default.");
		EepromDefault();
  }


  // ADS1115 ADC
	//Serial.println("Getting single-ended readings from AIN0..3");
	//Serial.println("ADC Range: +/- 6.144V (1 bit = .15mV)");
  ads0.begin();
  ads0.setGain(GAIN_ONE);	  // 1x gain   +/- 4.096V
  //  ads0.setGain(GAIN_ONE);	  // 1x gain   +/- 4.096V  1 bit = 2mV
  //  ads0.setGain(GAIN_TWO);	  // 2x gain   +/- 2.048V  1 bit = 1mV
  //  ads0.setGain(GAIN_FOUR);	  // 4x gain   +/- 1.024V  1 bit = 0.5mV
  //  ads0.setGain(GAIN_EIGHT);	  // 8x gain   +/- 0.512V  1 bit = 0.25mV
  //  ads0.setGain(GAIN_SIXTEEN); // 16x gain  +/- 0.256V
  ads1.begin();
  ads1.setGain(GAIN_ONE);	  // 1x gain   +/- 4.096V
  ads2.begin();
  ads2.setGain(GAIN_ONE);	  // 1x gain   +/- 4.096V

// 	Chan = {mfr1, mfr2, mfr3, mfr4, mfr5, mfr6, mfr7};
  istart = 1;
  
  // speed up adc - see https://forums.adafruit.com/viewtopic.php?f=19&t=59304
  //TWBR = ((F_CPU /400000l) - 16) / 2; // Change the i2c clock to 400KHz
  
  //Shadowband Parked
  if(MotorState==ON){
  	NadirShutdown();
  }
}

//=============================================================================
void loop() {
  int	  i = 0;
  unsigned int	  b = 0;
  char	  buf[20];
  double ddum, fdum;


  Serial.setTimeout(1000); // 1 sec wait for a key entry
  //test Serial.println("Enter 'T' or 't' to go directly to test mode.");
  i = Serial.readBytes(buf, 1); // normal operation -- wait for key entry
  if ( i > 0 || istart == 1) {
		if ( buf[0] == 't' || buf[0] == 'T' || istart == 1) {
			menustart = millis();
			istart = 0; // normal operation
			RunMode = TEST;
			Serial.setTimeout(10000); // 10 sec wait for a key entry
			//Serial.println("TEST mode.");
			while ( RunMode == TEST ) {
				if ( millis() - menustart > 600000) {
					break;
				}
				// prompt
				Serial.print("> ");
				// Wait for a command
				i = Serial.readBytesUntil(13, buf, 11);
				buf[i] = '\0';
				if ( i > 0 ) {
					// G - go to main loop
					if ( buf[0] == 'G' || buf[0] == 'g' ) {
						RunMode = RUN;
						Serial.println("Begin Operation...");
					}
					else {
						Serial.println("");
						Action(buf);
						menustart = millis();
					}
				}
			}
		}
	}


  // Elapsed time, days, since startup
  ElapsedTime(buf);

  // Wait for Nadir switch
  // Wait for horizon
	// READ ADC
	// TILT 1
	// SAMPLE OVER THE HORIZON
	// TILT 2
	// FIND SHADOW
	// MAKE OUT STRING
	// XMIT OUT STRING
	
  //RAIN STATE
  // CHECKSUM AND END
  Serial.println("Diagnostic.");
  delay(1000);

}
// =============== END OF LOOP =======================================


//*****************************************************************
void	Action(char *cmd)
{
  //  Read message and take action.
  // Create an output packet in out_buffer and send;
  // input:
  // in_buffer = pointer to a message string
  // in_fromlist = char string, e.g. "121115"
  //  meaning the message came from 12 in the route 15->11->12
  char str[50];
  char eok = 0;
  double fdum, ddum;
  byte i,ib,ic;
  unsigned long Ldum;
  unsigned long sweeptime,calctime;
  unsigned int isamp,ix,iy,iz;
  int n, n1;
  int16_t i16;

  // TAKE ACTION AND PREPARE AN OUTPUT MESSAGE IN out_message
  if (cmd[0] == '?') {
  PrintProgramID();
  Serial.println("------- EEPROM -----------------------------------");
  Serial.println("E       -- show eeprom ");
  Serial.println("ESiii   -- shadowlimit");
  Serial.println("ERfff.f -- Rain threshold volts");
  Serial.println("Ejff.f  -- Pitch correct deg         EJff.f  -- Roll correct deg");
  Serial.println("ENiii   -- Nsweep sweep");
  Serial.println("");
  Serial.println("------- FUNCTIONS -----------------------------------");
  Serial.println("an -- ADC Chan n              A  -- ADC all hemisphere");
  Serial.println("B  -- shadowband motor ON     b  -- shadowband OFF");
  Serial.println("C  -- 5REF ON                 c  -- 5REF OFF");
  Serial.println("H  -- MFR heater ON           h  -- shadowband OFF");
  Serial.println("j1 -- Thermister 1            j2 -- Thermister 2");
  Serial.println("n  -- Nadir switch            N  -- nadir loop");
  Serial.println("p  -- pitch/roll single       P  -- pitch/roll loop");
  Serial.println("r  -- Rain check single       R  -- Rain check loop");
  Serial.println("t  -- Read system clock       T  -- Set default f.p. day");
  Serial.println("sn -- LED test:  n=1 Heartbeat   2 Drum   3 Shutter/Rain");
  Serial.println("v/V -- Program version");
  Serial.println("g/G -- Continue sampling.");
  }
	
  // READ ADC CHANNEL
  else if (cmd[0] == 'a' && strlen(cmd) > 1) {
		if(strlen(cmd)==2){ib = cmd[1] - 48;}
		else{ ib=10*(cmd[1]-48)+(cmd[2]-48); }
		if (ib < 0 || ib > ADCMAX-1) {
			Serial.println("Error");
		} else {
			Serial.print("Chan "); Serial.println(ib);delay(10);
			while ( !Serial.available()	 ) {
// 				ddum=GetAdcVolts(ib);
				GetAdc(&i16,ib);
				Serial.println(i16);
				delay(100);
			}
		}
  }
  
  // ADC LOOP
  //Read all NCHAN channels 250 times as fast as possible.
  else if (cmd[0] == 'A') {
		//Loop 
		Sweeptime=3e6;
		SampMicroSecs=(unsigned long) (Sweeptime / (double)ee.Nsweep);
		Serial.print("SampMicroSecs=");Serial.print(SampMicroSecs,DEC); Serial.println(" microsecs");
		// Mark start time
		sweeptime=millis();
		// head temp
		T1 = ThermistorTemp(T1adc, ee.Rref[0]);
		T2 = ThermistorTemp(T2adc, ee.Rref[0]);
		// sweep
		FillAdc(); //test
// 		isamp=0;
// 		for(iy=0; iy<ee.Nsweep; iy++){
// 			for(ib=0; ib<NCHANS; ib++){
// 				GetAdc( (adcmv+isamp), Chan[ib] );
// 				isamp++;
// 			}
// 		}
		// elapsed time
		calctime = millis();
		sweeptime=calctime - sweeptime;
		// sweep calculations, chan=0,1,...,6
		for(ib=0;ib<NCHANS;ib++){
			Imin[ib]=SweepMinimum(ib);
			SweepGlobals(ib,10,&ddum,&fdum);
			G[ib][0]=ddum;
			G[ib][1]=fdum;
			SweepNoise(ib, 10, (Smean+ib), (Sstd+ib) );
			Shadow[ib]=ShadowRatio(ib);
		}
		//block average
		if(Shadow[ee.shadowchannel] >= ee.shadowlimit){
			SweepPack();
		}
		PackBlock();
		//computation   time
		calctime=millis()-calctime;
		// PRINT OUT SWEEP
		Serial.println("");
		isamp=0;
		for(iy=0; iy<ee.Nsweep; iy++){
			Serial.print(iy,DEC); Serial.print(" "); // sample count
			for(iz=0; iz<NCHANS; iz++){
				Serial.print(adcmv[isamp],DEC); Serial.print(" ");
				//Serial.print(isamp); Serial.print(" ");
				isamp++;
			}
			Serial.println("");
		}
		// PRINT OUT STATS
		Serial.print("Sample time: "); Serial.print(sweeptime,DEC); Serial.println(" msecs");		
		Serial.print("Comptime time: "); Serial.print(calctime,DEC); Serial.println(" msecs");
		Serial.print("T1=");Serial.print(T1,2);Serial.print("   T2=");Serial.println(T2,2);
		Serial.print("Shadow channel: ");Serial.println(ee.shadowchannel);
		for(ib=0; ib<NCHANS; ib++){
			ix= Imin[ib]*NCHANS + ib;
			Serial.print(ib);Serial.print("  ");Serial.print(Imin[ib]);
			Serial.print("  ");Serial.println(adcmv[ix]);
		}
		Serial.print("Global 1:");
		for(ib=0; ib<NCHANS; ib++){
			Serial.print("  ");Serial.print(G[ib][0],2);
		}
		Serial.println("");
		Serial.print("Global 2:");
		for(ib=0; ib<NCHANS; ib++){
			Serial.print("  ");Serial.print(G[ib][1],2);
		}
		Serial.println("");
		Serial.print("Sweep mean:");
		for(ib=0; ib<NCHANS; ib++){
			Serial.print("  ");Serial.print(Smean[ib],2);
		}
		Serial.println("");
		Serial.print("Sweep stdev:");
		for(ib=0; ib<NCHANS; ib++){
			Serial.print("  ");Serial.print(Sstd[ib],2);
		}
		Serial.println("");
		Serial.print("Shadowratio:");
		for(ib=0; ib<NCHANS; ib++){
			Serial.print("  ");Serial.print(Shadow[ib],2);
		}
		Serial.println("");
		Serial.print("Limit ");Serial.print(ee.shadowlimit);
		  Serial.print("  Shadow ");Serial.println(Shadow[ee.shadowchannel]);
		//Packet
		if(Shadow[ee.shadowchannel] >= ee.shadowlimit){
			Serial.println("Packed Bins");
			for(ic=0; ic<NCHANS; ic++){
				//Serial.print(ic); Serial.print(" ");
				for(ib=0; ib<2*NBLKS+1; ib++){
					Serial.print(" "); Serial.print(sweepBlk[ib][ic]);
				}
				Serial.println("");
			}
		} else{
			Serial.println("No Shadow");
		}
		//packet
		Serial.print(datafile);
			// TERMINATE DATA FILE STRING
		checksum = checksum_nmea(datafile+1);
    Serial.print("*"); 
    Serial.println(checksum,HEX); 
  }
	
  // MFR HEATER ON
  else if (cmd[0] == 'H') {
	ix = cmd[1] - 48;
	Serial.print("MFR HEATER ON");
	MFRHeater(ON);
  }
  // MFR HEATER OFF
  else if (cmd[0] == 'h') {
	Serial.println("MFR HEATER OFF");
	MFRHeater(OFF);
  }

  // SHADOWBAND MOTOR ON
  else if (cmd[0] == 'B') {
	Serial.print("SHADOWBAND MOTOR ON");
	Shadowband(ON);
  }
  // MFR HEATER OFF
  else if (cmd[0] == 'b') {
	Serial.println("SHADOWBAND MOTOR OFF");
	// Wait for the nadir to go from 1 to 0
	NadirShutdown();
  }
  
  // 5REF	ON/OFF
  else if (cmd[0] == 'C' ) {
	Serial.println("5REF ON");
	Ref(ON);
  }
  else if (cmd[0] == 'c' ) {
	Serial.println("5REF OFF");
	Ref(OFF);
  }

	// THERMISTER
	else if (cmd[0] == 'j' || cmd[0] == 'J') {
		//T1
		fdum=ThermistorTemp(0, ee.Rref[0]);
		Serial.print("T1=");Serial.println(fdum,2);
		//T2
		fdum=ThermistorTemp(1, ee.Rref[0]);
		Serial.print("T2=");Serial.println(fdum,2);
	}
  
  // NADIR SWITCH
  else if (cmd[0] == 'n') {
		Serial.println("0=CLOSED   1=OPEN");
		while ( !Serial.available()	 ) {
	  	Serial.println(digitalRead(NADIR));
	  	delay(20);
		}
  }
  else if (cmd[0] == 'N') {
	NadirTime();
  }
  // TIME
  else if (cmd[0] == 't') {
	Serial.println("Elapsed time as dd.hhmmss---");
	while (1) {
	  Ldum = ElapsedTime(str);
	  Serial.print("Millisecs elapsed: "); Serial.print(Ldum);
	  Serial.print("   "); Serial.println(str);
	  if ( Serial.available()) {
		break;
	  }
	  delay(2000);

	}
  }

  // EEPROM
  else if ( cmd[0] == 'E' || cmd[0] == 'e') {
	if ( strlen(cmd) <= 1 ) {
	  EepromPrint();
	}
	else if (strlen(cmd) > 2) {
	  eok = 0;
	  if ( cmd[1] == 'j' ) { 
		ddum = atof(cmd + 2);
		Serial.print("PITCH CORRECTION = ");
		Serial.println(ddum, 5);
		ee.pitch_correct = ddum;
		eok = 1;
	  }
	  else if ( cmd[1] == 'J' ) { 
		ddum = atof(cmd + 2);
		Serial.print("ROLL CORRECTION = ");
		Serial.println(ddum, 3);
		ee.roll_correct = ddum;
		eok = 1;
	  }
	  else if ( cmd[1] == 'R' ) {
		ddum = atof(cmd + 2);
		Serial.print("SET RAIN THRESHOLD = ");
		Serial.println(ddum, 2);
		ee.rain_threshold = ddum;
		eok = 1;
	  }
	  else if ( cmd[1] == 'N' ) {
		Serial.print("Nsweeps: ");
		ee.Nsweep = atoi(cmd + 2);
		Serial.println(ee.Nsweep);
	  }
	  else if ( cmd[1] == 's' ) {
		Serial.print("Shadow Limit: ");
		ee.shadowlimit = atoi(cmd + 2);
		Serial.println(ee.shadowlimit);
	  }
	  else if ( cmd[1] == 'S' ) {
		Serial.print("Shadow Channel: ");
		ee.shadowchannel = atoi(cmd + 2);
		Serial.println(ee.shadowchannel);
	  }
	  else if ( cmd[1] == 'E' ) {
		Serial.print("SET MODE: ");
		ix = atoi(cmd + 2);
		if (ix == 0) {
		  Serial.println("RUN");
		}
		else if (ix == 1) {
		  Serial.println("CALIBRATION");
		}
		else {
		  Serial.println("UNDEFINED, NO CHANGE");
		}
		eok = 1;
	  }
	  else if ( cmd[1] == 'r' ) {
		ix = atoi(cmd + 2);
		Serial.print("SET RAIN DELAY SECS = ");
		Serial.println(ix, DEC);
		ee.rainsecs = ix;
		eok = 1;
	  }
	}
  }


  // PITCH/ROLL
  else if (cmd[0] == 'p') {
	ReadTilt(&ddum, &fdum);
	Serial.print("pitch = ");
	Serial.print(ddum, 2);
	Serial.print("	  roll = ");
	Serial.println(fdum, 2);
  }
  else if (cmd[0] == 'P') {
	Serial.println(" N	PITCH  ROLL");
	ix = 0;
	while (! Serial.available()) {
	  // index
	  Serial.print(ix, DEC);
	  Serial.print("  ");
	  ReadTilt(&ddum, &fdum);
	  Serial.print(ddum, 2);
	  Serial.print("  ");
	  Serial.println(fdum, 2);
	  delay(1000);
	  ix++;
	}
  }

  // RAIN
  else if (cmd[0] == 'r') {
	CheckRain( &ddum, &Ldum );
	Serial.print("Rain volts = ");
	Serial.print(ddum, 2);
	Serial.print("	 Sec to open = ");
	Serial.print(Ldum);
	Serial.print("	 State = ");
	Serial.println(RainState, DEC);
  }
  else if (cmd[0] == 'R') {
	Serial.println(" N	STATUS	 VOLTS	 SECS");
	ix = 0;
	while (! Serial.available()) {
	  // index
	  Serial.print(ix, DEC);
	  Serial.print("  ");
	  CheckRain( &ddum, &Ldum );
	  Serial.print(RainState, DEC);
	  Serial.print("   ");
	  Serial.print(ddum, 2);
	  Serial.print("  ");
	  Serial.println( Ldum );
	  delay(1000);
	  ix++;
	}
  }
  else if (cmd[0] == 's' && strlen(cmd) > 1) {
	ix = cmd[1] - 48;
	n = 1;
	if (ix == 1) {
	  Serial.println("HEARTBEAT LED FLASH");
	  n1 = LED31;
	}
	else if (ix == 2) {
	  Serial.println("DRUM LED FLASH");
	  n1 = LED11;
	}
	else if (ix == 3) {
	  Serial.println("SHUTTER LED FLASH");
	  n1 = LED12;
	}
	else {
	  Serial.println("HEARTBEAT LED FLASH");
	  n1 = LED31;
	}
	// flash led
	while (! Serial.available()) {
	  digitalWrite(n1, HIGH);
	  delay(500);
	  digitalWrite(n1, LOW);
	  delay(500);
	}
  }
  // VERSION
  else if (cmd[0] == 'v' || cmd[0] == 'V') {
	PrintProgramID();
  }
  // RETURN TO RUN
  else if (cmd[0] == 'g' || cmd[0] == 'G') {
	RunMode == RUN;
  }
  // DEFAULT
  else {
	Serial.print(cmd);
	Serial.println(" not recognized.");
  }
  return;
}

//======================================================================================
void CheckRain(double *v, unsigned long *rainsecs)
// Output
//		v = analog (volts)
//		rainsecs = seconds until shutter opens
// RETURN 1 or 0 for rain mode and shutter control.
// Global in
//	ee.rain_threshold;
//	ee.rainsecs;
// Global out
//	 unsigned long	millisec_rain;	// clock count for opening
//	 RainState
{
  int a;

  *v=GetAdcVolts(Arain);
  *v *= 4;
  //Serial.print("Chan ");	Serial.print(Arain,DEC); Serial.print("	 ");  Serial.println(*v,4);

  // RAIN
  if ( *v > ee.rain_threshold ) {
	RainState = 1;
	millisec_rain = millis();
	*rainsecs = ee.rainsecs;
  }
  // NO RAIN
  else {
	// SHUTTER IS CLOSED
	if ( RainState == 1 ) {
	  secs_lapsed = ( millis() - millisec_rain ) / 1000;
	  // TIME TO OPEN
	  if ( secs_lapsed > ee.rainsecs ) {
		RainState = 0;
		*rainsecs = 0;
	  }
	  // DRYING TIME
	  else {
		RainState = 1;
		*rainsecs = ee.rainsecs - secs_lapsed;
	  }
	}
	else {
	  *rainsecs = 0;
	  RainState = 0;
	}
  }
  return;
}
//***************************************************************************
unsigned int checksum_nmea(char *strPtr) {
  // code --  http://www.mculabs.com/snippets/nmea_checksum.html
  // validate -- http://www.hhhh.org/wiml/proj/nmeaxor.html
  int p;
  char c;
  byte chksum;

  c = strPtr[0]; // get first chr
  chksum = c;
  p = 1;
  while ( c != 0x00 ) {
	c = strPtr[p]; // get next chr
	if ( c != 0x00 ) {
	  chksum = chksum ^ c;
	}
	p++;
  }
  return chksum;
}
//========================================================================
float DiffAngle(float a2, float a1) {
  // Compute the smallest angle arc between the a2 and a1.
  float arc;
  arc = a2 - a1;
  if ( abs(arc) > 180 ) {
	if ( sign(arc) > 0 ) arc -= 360;
	else arc += 360;
  }
  return arc;
}

//*******************************************************************
void EepromDefault() {
  int i;
  Serial.println("Initialize eeprom...");
  ee.id = EEPROM_ID;
  ee.shadowlimit = default_shadowlimit;
  ee.rain_threshold = default_rain_threshold;
  ee.rainsecs = default_rainsecs;
  for (i = 0; i < 2; i++) {
	ee.Rref[i] = default_Rref[i];
  } 
  ee.mfrheat = default_mfrheat;
  ee.pitch_correct = default_pitch_correct;
  ee.roll_correct = default_roll_correct;
  ee.Nsweep = default_Nsweep;
  ee.FrsrState = ON;
  EepromStore();
  EepromRead();
  EepromPrint();
}

//=============================================================================
void EepromStore()
//Determines the size of the structure and stores it entirely
//in eeprom space
{
  Serial.println("StoreUee...");
  int i;
  // pointer to struct ee
  //struct eeprom* ptr = &ee;
  byte* a = &(ee.id);

  // for each byte in the eeprom structure
  for (i = 0; i < eeSize; i++) {
	EEPROM.write(i, *a );	// store this byte
	a++;
  }
  return;
}

//=============================================================================
void EepromRead()
{
  int i;
  // pointer to struct ee
  byte* a = &(ee.id);
  // for each byte in the eeprom structure
  for (i = 0; i < eeSize; i++)
  {
	*a = EEPROM.read(i);  // get the byte
	a++;
  }
  return;
}

//===============================================================================
void EepromPrint()
{
//	 byte id; 
//	 float rain_threshold;
//	 float pitch_correct, roll_correct; 
//	 unsigned long rainsecs;
//	 char mfrheat; // on or off
//	 double Rref[2];
// 
	int i;
	Serial.println("EepromPrint: ");
	Serial.print("  ID = ");  Serial.println(ee.id);
	Serial.print("  s Shadow limit	= "); Serial.println(ee.shadowlimit);
	Serial.print("  S Shadow channel	= "); Serial.println(ee.shadowchannel);
	Serial.print("  R Rain threshold	= ");Serial.print(ee.rain_threshold, 2);Serial.println(" volts");
	Serial.print("  r Rain shutter delay	= ");Serial.print(ee.rainsecs);Serial.println(" secs");
	Serial.print("  j pitch correct = "); Serial.println(ee.pitch_correct, 1);  //v29
	Serial.print("  J roll correct correct = ");	Serial.println(ee.roll_correct, 1);	 //v29
	Serial.print("  MFR heater configuration = ");	Serial.println(ee.mfrheat, DEC);
	Serial.print(" Sweep samples = ");Serial.println(ee.Nsweep, DEC);
	Serial.print(" Rref = ");Serial.print(ee.Rref[0],1); Serial.print("  "); Serial.println(ee.Rref[1],1);
	Serial.print(" FrsrState	= "); Serial.println(ee.FrsrState);
	

	return;
}
/***************************************************************************************/
unsigned long ElapsedTime (char *ddhhmmss) {

  unsigned long Ld, Lh, Lm, Ls, ms;
  char ch[3];
  ddhhmmss[0] = '\0';

  // Elapsed time, days, since startup
  ms = millis() - msstart;

  //days
  Ld = ms / 86400000;		  // number of days
  if (Ld >= 0 && Ld < 100) {
	ch[2] = '\0'; ch[1] = Ld % 10 + 48; ch[0] = Ld / 10 + 48;
  } else {
	strcpy(ch, "xx");
  }
  strcat(ddhhmmss, ch);
  strcat(ddhhmmss, ".");
  //	 Serial.print("days = ");  Serial.print(Ld);
  //	 Serial.print("	  string:");  Serial.print(ch);
  //	 Serial.print("	  ddhhmmss:");	Serial.println(ddhhmmss);

  //hours
  Lh = (ms - Ld * 86400000) / 3600000;	  // number hours
  if (Lh >= 0 && Lh < 100) {
	ch[2] = '\0'; ch[1] = Lh % 10 + 48; ch[0] = Lh / 10 + 48;
  } else {
	strcpy(ch, "xx");
  }
  strcat(ddhhmmss, ch);
  //	 Serial.print("hours = ");	Serial.print(Lh);
  //	 Serial.print("	 string = ");  Serial.print(ch);
  //	 Serial.print("	 ddhhmmss:");  Serial.println(ddhhmmss);

  //min
  Lm = (ms - Ld * 86400000 - Lh * 3600000) / 60000;
  if (Lm >= 0 && Lm < 100) {
	ch[2] = '\0'; ch[1] = Lm % 10 + 48; ch[0] = Lm / 10 + 48;
  } else {
	strcpy(ch, "xx");
  }
  strcat(ddhhmmss, ch);
  //	 Serial.print("mins = ");  Serial.print(Lm);
  //	 Serial.print("	 string = ");  Serial.print(ch);
  //	 Serial.print("	 ddhhmmss:");  Serial.println(ddhhmmss);

  //sec
  Ls = (ms - Ld * 86400000 - Lh * 3600000 - Lm * 60000) / 1000;
  if (Ls >= 0 && Ls < 100) {
	ch[2] = '\0'; ch[1] = Ls % 10 + 48; ch[0] = Ls / 10 + 48;
  } else {
	strcpy(ch, "xx");
  }
  strcat(ddhhmmss, ch);
  //	 Serial.print("secs = ");  Serial.print(Ls);
  //	 Serial.print("	 string = ");  Serial.print(ch);
  //	 Serial.print("	 ddhhmmss:");  Serial.println(ddhhmmss);

  return ms;
}
//*******************************************************************
char * floatToString(char * outstr, double val, byte precision, byte widthp)
/********
 * Convert a double prec variable to a string of characters.
 * Example	 floatToString(s, 123.456, 2, 8) returns
 * s = [' ',' ','1','2','3','.','4','6','\0'] for 8 characters and a NULL
 *********/
{
  // http://forum.arduino.cc/index.php/topic,37391.0.html

  char temp[16];
  byte i;

  // ROUNDING
  double roundingFactor = 0.5;
  unsigned long mult = 1;
  for (i = 0; i < precision; i++)
  { //			 *
	roundingFactor /= 10.0;		// .5, .05, .005, ...
	mult *= 10;					// 1,  10,	100, ...
  }


  // OUTSTRING
  temp[0] = '\0';
  outstr[0] = '\0';

  // NEGATIVE NUMBERS
  if (val < 0.0) {
	strcpy(outstr, "-\0");
	val = -val;
  }

  // 123.461
  val += roundingFactor;
  strcat(outstr, ltoa(long(val), temp, 10));   //prints the int part

  if ( precision > 0) {
	strcat(outstr, ".\0"); // print the decimal point

	unsigned long frac;
	unsigned long mult = 1;
	byte padding = precision - 1;
	while (precision--)
	  mult *= 10;

	if (val >= 0)
	  frac = (val - long(val)) * mult;
	else
	  frac = (long(val) - val ) * mult;

	unsigned long frac1 = frac;
	while (frac1 /= 10)
	  padding--;

	while (padding--)
	  strcat(outstr, "0\0");

	strcat(outstr, ltoa(frac, temp, 10));
  }
  //Serial.print("test2 ");Serial.println(outstr);

  // generate space padding
  if ((widthp != 0) && (widthp >= strlen(outstr))) {
	byte J = 0;
	J = widthp - strlen(outstr);

	for (i = 0; i < J; i++) {
	  temp[i] = ' ';
	}

	temp[i++] = '\0';
	strcat(temp, outstr);
	strcpy(outstr, temp);
  }

  return outstr;
}

//======================================================================================
void GetAdc (int16_t *iv, byte chan) {
  //GetAdc returns a single read of 16 bit adc channel chan.
  int16_t adc;
  if (chan >= 0 && chan <= 3){
		adc = ads0.readADC_SingleEnded(chan);
	}
  else if (chan >= 4 && chan <= 7)
		adc = ads1.readADC_SingleEnded(chan - 4);
  else if (chan >= 8 && chan <= 11)
		adc = ads2.readADC_SingleEnded(chan - 8);
  else adc = 0;
	*iv = (int16_t) ((float)adc / 8);
	
  return;
}


//======================================================================================
double GetAdcVolts(byte ch) {
  //	  okflag = GetAdcVolts(ch, *vmean){
  // GetAdcVolts takes nsap (=10) readings of the ADC channel ch. Tosses out the min and max
  // voltages and takes the mean of the remainder.

  int16_t v, vmn, vmx;
  double vsum, vmean;
  int16_t vi[10];
  unsigned int i, imx, imn, nav;
  nav = 10;
  vsum = 0;
  vmx = -4096;
  vmn = 4096;
  imx = imn = 0;
  //  10 samples
  for (i = 0; i < nav; i++) {
		GetAdc( &v, ch);
		vi[i]=v;		// in millivolts
		vsum += (double)v ;
		//max and min
		if (v < vmn) {
	  	imn = i;
	  	vmn = v;
		}
		if (v > vmx) {
			imx = i;
			vmx = v;
		}
  }
  vmean = (vsum - (double)vi[imx] - (double)vi[imn]) / (double)(nav - 2);
  return vmean;
}
//============================================================
unsigned int GetThermCoefs(unsigned int nt, double c[] ) {
  //   revision, add 9,10,11,12
  // Give the correct SHH coefs for the thermnumber
  // See 04.20.18.10../Tcal3_1605_thermistor_cal_rosr3/Tcal1605_report.key.pdf
  // See BB_Therm_Master_List spreadsheet we need to revise this for each rosr

  double tcal[13][3] = {  //v28	 !! 13 rows, add to this for new calibrations.
	//0 standard ysi
	{ 1.025579e-03,	  2.397338e-04,	  1.542038e-07 },
	//1 Therm #1, Rosr1 T11
	{ 1.0108115e-03, 2.4212099e-04,	  1.4525424e-07 },
	//2 Therm #2, ROSR1, T12
	{ 1.0138029e-03, 2.4156995e-04,	  1.4628056e-07 },
	//3 Therm #3, ROSR1, T21
	{ 1.0101740e-03, 2.4208389e-04,	  1.4485814e-07 },
	//4 Therm #4, ROSR1, T22
	{ 1.0137647e-03, 2.4161708e-04,	  1.4619775e-07 },
	//5 Therm #5, ROSR2, T11
	{ 1.0073532E-03, 2.41655931E-04, 1.56252214E-07 },
	//6 Therm #8, ROSR2, T12
	{ 1.00647228E-03, 2.42265935E-04, 1.50546312E-07 },
	//7 Therm #9, ROSR2, T21
	{ 1.01935214E-03, 2.40428900E-04,	1.56322696E-07 },
	//8 Therm #10, ROSR2, T22
	{ 1.02013510E-03, 2.40304832E-04, 1.55160255E-07  },
	//9 Therm-12, ROSR3, T11  //v28
	{ 1.00171527e-03, 2.42997797e-04, 1.46913550e-07 },
	//10 Therm-13. ROSR3, T12  //v28
	{ 1.01743969e-03, 2.40737206e-04, 1.52367550e-07 },
	//11 Therm-15, ROSR3, T21  //v28
	{ 9.92861282e-04, 2.44664838e-04, 1.37715245e-07 },
	//12 Therm-16, ROSR3, T22  //v28
	{ 1.00677444e-03, 2.42252224e-04, 1.49693186e-07 }
  };

  if (nt < 0 || nt > 13) {
	Serial.print("Error in GetThermCoefs() -- bad thermnumber=");
	Serial.println(nt, DEC);
	c[0] = c[1] = c[2] = 0;
	return 0;
  }
  else {
	c[0] = tcal[nt][0];
	c[1] = tcal[nt][1];
	c[2] = tcal[nt][2];
	return 1;
  }
}
//==========================================================================
long Hex2Dec(char *e)
// Convert a 3-char hex string to a decimal number
{
  unsigned int j;
  byte i;

  //  Serial.print(e[0],DEC); Serial.print("  ");
  if (e[0] >= 48 && e[0] <= 57) i = e[0] - 48;
  else if (e[0] >= 65 && e[0] <= 70) i = e[0] - 55;
  else return MISSING;
  //  else {Serial.println(" BAD"); return MISSING;}
  j = i * 256;

  //  Serial.print(e[1],DEC); Serial.print("  ");
  if (e[1] >= 8 && e[1] <= 57) i = e[1] - 48;
  else if (e[1] >= 65 && e[1] <= 70) i = e[1] - 55;
  else return MISSING;
  //  else {Serial.println(" BAD"); return MISSING;}
  j = j + i * 16;

  //  Serial.println(e[2],DEC);
  if (e[2] >= 48 && e[2] <= 57) i = e[2] - 48;
  else if (e[2] >= 65 && e[2] <= 70) i = e[2] - 55;
  //  else {Serial.println(" BAD"); return MISSING;}
  else return MISSING;
  j = j + i;

  return j;
}
//====================================================================================================
void	MeanStdev(double *sum, double *sum2, int N, double missing)
//	Compute mean and standard deviation from
//	the count, the sum and the sum of squares.
{
  if ( N <= 2 ) {
	*sum = missing;
	*sum2 = missing;
  }
  else {
	*sum /= (double)N;		// mean value
	*sum2 = *sum2 / (double)N - (*sum * *sum); // sumsq/N - mean^2
	*sum2 = *sum2 * (double)N / (double)(N - 1); // (N/N-1) correction
	if ( *sum2 < 0 ) *sum2 = 0;
	else *sum2 = sqrt(*sum2);
  }
  return;
}
//==================================================================
void MFRHeater(int k)
/***************
 * input:
 * k = ON or OFF
	**********/
{
	if (k == ON) {
		digitalWrite(HEATER, HIGH);
		digitalWrite(LED22, HIGH);
		//Serial.println("MFR heater on.");
	}
	if (k == OFF) {
		digitalWrite(HEATER, LOW);
		digitalWrite(LED22, LOW);
		//Serial.println("MFR heater OFF.");
	}
  return;
}
//======================================================================
void NadirShutdown(void)
{
	unsigned long t0;
	Serial.print("NadirShutdown: MotorState="); Serial.println(MotorState);
	//wait for nadir==1
	t0=millis();
	while(digitalRead(NADIR) == 0 ){
		if(millis()-t0>10000){
			Serial.println("NadirShutdown timeout.");
			Shadowband(OFF);
			return;
		}
		delay(10);
	}
	while(digitalRead(NADIR) == 1 ){
		if(millis()-t0>10000){
			Serial.println("NadirShutdown timeout.");
			Shadowband(OFF);
			return;
		}
		delay(10);
	}
	Shadowband(OFF);
}
//======================================================================
void NadirTime(void)
{
	unsigned long t0;
	Shadowband(ON);
	while(1){
		// wait for nadir start 	
		t0=millis();
		while(digitalRead(NADIR) == 0 );
		Serial.print("1  ");
		//now wait for nadir end
		while(digitalRead(NADIR) == 1 );
		Serial.println( millis()-t0 );
		if ( Serial.available() ) break;
	}
	Shadowband(OFF);
}
//==================================================================
void PrintBytes(char *str) {
  int i, ic;
  i = strlen(str);
  Serial.print("PrintBytes: ");
  for (ic = 0; ic < i; ic++) {
	Serial.print(str[ic], HEX);
	Serial.print(", ");
  }
  Serial.println("");
}
//==================================================================
void PrintProgramID(void)
{
  Serial.println("");
  Serial.print("PROGRAM:");
  Serial.print(PROGRAMNAME);
  Serial.print("   VERSION:");
  Serial.print(VERSION);
  Serial.print("  EDIT:");
  Serial.print(EDITDATE);
  Serial.println("");
}
//============================================================================
void		ReadTilt(double *pitch, double *roll)
{
  unsigned long microstart; // define input wait time
  char e[4], chrin;
  byte i, count;
  double ddum;

  Serial2.setTimeout(100);

  // Clear buffer
  //  while( Serial2.available() ) Serial2.read();

  // READ IN PITCH, TRY SEVERAL TIMES
  //	count=0;
  //  Serial.print("pitch ");
  //	while( count < 3 )	  {
  //	  count++;
  Serial2.write(66);
  delay(10); //v6
  Serial2.readBytesUntil('\n', e, 4);
  e[3] = '\0';
  //	  Serial.print(e[0],DEC);Serial.print("	 ");
  //	  Serial.print(e[1],DEC);Serial.print("	 ");
  //	  Serial.println(e[2],DEC);
  //  }
  ddum = (double) Hex2Dec(e);
  if (ddum != MISSING && ddum >= 0 && ddum <= 4097) {
	ddum -= 2048;
	if (ddum < -1660) ddum = -1660;
	if (ddum > 1660) ddum = 1660;
	*pitch = P1 * (C1 * ddum + C2 * ddum * ddum + C3 * ddum * ddum * ddum) + P0 + ee.pitch_correct;	 //v29
  }
  else *pitch = MISSING;
  //  Serial.print("Pitch = "); Serial.println(*pitch,1);


  // READ IN ROLL, TRY SEVERAL TIMES
  // Clear buffer
  //  while( Serial2.available() ) Serial2.read();
  //	count=0;
  //  Serial.print("roll ");
  //	while( count < 3 )	  {
  //	  count++;
  Serial2.write(67);
  delay(10); //v6
  Serial2.readBytesUntil('\n', e, 4);
  e[3] = '\0';
  //	  Serial.print(e[0],DEC);Serial.print("	 ");
  //	  Serial.print(e[1],DEC);Serial.print("	 ");
  //	  Serial.println(e[2],DEC);
  //  }
  ddum = (double) Hex2Dec(e);
  if (ddum != MISSING && ddum >= 0 && ddum <= 4097) {
	ddum -= 2048;
	if (ddum < -1660) ddum = -1660;
	if (ddum > 1660) ddum = 1660;
	*roll = R1 * (C1 * ddum + C2 * ddum * ddum + C3 * ddum * ddum * ddum) + R0 + ee.roll_correct; //v29
  }
  else *roll = MISSING;
  //  Serial.print("Roll = "); Serial.println(*roll,1);
  return;
}
//==================================================================
unsigned	Ref(int K) {
  if (K == ON) {
	digitalWrite(REFSW, LOW);
	digitalWrite(LED32, HIGH);
	return OK;
  }
  else if (K == OFF) {
	digitalWrite(REFSW, HIGH);
	digitalWrite(LED32, LOW);
	return OK;
  }
  return NOTOK;
}
//==================================================================
void Shadowband(int k)
/***************
 * input:
 * k = ON or OFF
	**********/
{
	if (k == ON) {
		digitalWrite(MOTOR, HIGH);
		digitalWrite(LED22, HIGH);
		//Serial.println("Shadowband motor on.");
		MotorState=ON;
	}
	if (k == OFF) {
		digitalWrite(MOTOR, LOW);
		digitalWrite(LED22, LOW);
		//Serial.println("Shadowband motor OFF.");
		MotorState=OFF;
	}
  return;
}

//============================================================================
int sign (float input) {
  return ((input < 0.0) ? NEG : POS);
}
//==========================================================================
double ThermistorTemp(byte nadc, double Rref){
/* nadc = 0 or 1
Rref usually = 10000
*/
	double vx, v;
	double r;
	double t, t2;
	int16_t i16;
	double beta[]={ 1.025579e-03,   2.397338e-04,   1.542038e-07};
	
	//T1
	v=GetAdcVolts(nadc);	
	//check bad input
	if(v<AdcThreshold[0]) return 0;
	if(v>AdcThreshold[1]) return 50;
	vx = 2.045*GetAdcVolts(3); // Vref
	//Serial.print("vref="); Serial.println(vx,1);
	
	r = Rref * (v / (vx-v));
	//Serial.print("test rtherm="); Serial.println(r,2);
	if(r<1000||r>30000){
		Serial.print("Error. rtherm="); Serial.println(r,2);
		return 0;
	}
	vx = log(r);
//	Serial.print("log(r)="); Serial.println(vx,3);
// 	Serial.print("beta0="); Serial.print(beta[0],8);
// 	Serial.print(";	beta1="); Serial.print(beta[1],8);
// 	Serial.print(";	beta2="); Serial.println(beta[2],10);
	//fitted curve
	t2 = beta[0] + beta[1] * vx + beta[2] * vx * vx * vx;
	t = 1 / t2 - (double)273.15;
	// from FRSR v3 code (PRP2)
// #define A1 -3.4868183734e-7
// #define A2 -8.9323216739e-6
// #define A3 -3.308255931317e-4
// #define A4 7.92320546465e-4
// 	d = log(1/rt);
// 	d = A1 * d * d * d + A2 * d * d + A3 * d + A4;
// 	T1 = 1.0 / d - 273.15;

// 	Serial.print("T="); Serial.println(t,3);

	return t;
}
//===================================================================
unsigned int SweepMinimum(byte chan)
/*
int16_t adcmv[NCHANS*NSWEEP] is the full array of adc reads.
byte chan is the selected adc channel, 0-6.
adcmv[chan] is the first point. adcmv[chan+NCHANS] is the second point, ...
*/
{
	unsigned int imin=0;
	int16_t adcmin = 32767;
	unsigned int ix,ip;
	
	for(ix=0;ix<NSWEEP;ix++){		//0,1,2,...249
		ip=chan + ix*NCHANS;	//[0,7,14...1743] [1,8,15...1744]...[6,13,...1749] 
		if(adcmv[ip]<adcmin){
			imin=ix;
			adcmin=adcmv[ip];
		}
	}
	Smin[chan]=adcmin;
	return imin;
}
//===================================================================
double SweepNoise(byte chan, byte nsum, double *mnx, double *stdx)
/*
byte chan - is the selected adc channel, 0-6.
Imin[chan] - is the index for the minimum for the chan timeseries.
int16_t adcmv[NCHANS*NSWEEP] - is the full array of adc reads.
int16_t adcmv[chan] is the first point. adcmv[chan+NCHANS] is the second point, ...
int16_t adcmv[chan+imin*NCHANS] - is the minimum value.
*/
{
	unsigned int ix,ip;
	unsigned int isum=0;
	double x,xs,x1,xs1;

	// LEFT SIDE
	x = 0; xs=0;isum=0;
	for(ix=0; ix<nsum; ix++){
		ip = chan + ix*NCHANS;	//[0,7,14...1743] [1,8,15...1744]...[6,13,...1749] 
		x += (double)adcmv[ip];
		xs += (double)adcmv[ip]*(double)adcmv[ip];
		isum++;
	}
	MeanStdev(&x, &xs, isum, 0); 


	// RIGHT SIDE
	x1=0; xs1=0;isum=0;
	for(ix=NSWEEP-nsum; ix<NSWEEP; ix++){
		ip = chan + ix*NCHANS;	//[0,7,14...1743] [1,8,15...1744]...[6,13,...1749] 
			x1 += (double)adcmv[ip];
			xs1 += (double)adcmv[ip]*(double)adcmv[ip];
			isum++;
	}
	MeanStdev(&x1, &xs1, isum, 0); 
	*mnx=(x+x1)/2;
	*stdx = (xs+xs1)/2; // approximate is stdevs are small.
	return *stdx;
}
//===================================================================
void SweepGlobals(byte chan, byte nsum, double *g1, double *g2)
/*
int16_t adcmv[NCHANS*NSWEEP] is the full array of adc reads.
double *G is the end point at each end.
byte chan channel (0,1,...6)
byte nsum - typically = 10
*/
{
	unsigned int ix,ip;
	
	// LEFT SIDE
	*g1 = 0;
	for(ix=0; ix<nsum; ix++){
		ip = chan + ix*NCHANS;	//[0,7,14...1743] [1,8,15...1744]...[6,13,...1749] 
		*g1 += (double)adcmv[ip];
	}
	*g1 /= (double) nsum;
	// RIGHT SIDE
	*g2 = 0;
	for(ix=NSWEEP-nsum; ix<NSWEEP; ix++){
		ip = chan + ix*NCHANS;	//[0,7,14...1743] [1,8,15...1744]...[6,13,...1749] 
		*g2 += (double) adcmv[ip];
	}
	*g2 /= (double) nsum;
}
//===========================================================================
double	ShadowRatio(byte chan)
{
	double shadowratio;
	if(Sstd[chan]<=0){return 0;}
	
	shadowratio = (Smean[chan] - Smin[chan]) / Sstd[chan];
	return max(shadowratio,0);
}
//===========================================================================
void	SweepPack(void)
/* If the desired channel, Shadow > ShadowThreshold, block avg 23 floats

*/
{
	byte ic, ib, ibk ;
	int i,i1,i2;
	byte npts;
	double sum;
	
	// channels
	for(ic=0; ic<NCHANS; ic++){		// [0,1,...249]
		// blocks
		for(ib = 0; ib < 2 * NBLKS + 1; ib++) sweepBlk[ib][ic] = 0;
		// RIGHT SIDE
		i2 = Imin[ic];
		for(ibk=0; ibk<NBLKS; ibk++){  // 0,1,...,10
			// INDEX IN THE BLOCK ARRAY
			ib = NBLKS + ibk + 1; // 12,...22
			// SUMMATION LIMITS
			i1 = i2+1; 							// just to right of the shadow 
			i2 = i1 + Nsz[ibk] - 1; 			// Nsz points in mean
			if( i2 >= NSWEEP) i2 = NSWEEP-1;  	// do not overrun the array
			sum = 0;  npts=0;					// initialize
			for(i=i1; i<=i2; i++){
				sum += (double)adcmv[i*NCHANS+ic];
				npts++;
			}
			sweepBlk[ib][ic] = (int) round(sum / (double)npts);
			if( i2 == NSWEEP-1 ) break;
		}
		// LEFT SIDE BLOCK AVGS
		i1 = Imin[ic];
		for(ibk=0; ibk<NBLKS; ibk++){  // 0,1,...,10
			// INDEX IN THE BLOCK ARRAY
			ib = NBLKS - ibk - 1; // 10,9,...,0
			// SUMMATION LIMITS -- from shadow to left
			i2 = i1 - 1;
			i1 = i2 - Nsz[ibk] + 1;
			//Serial.print(i1,DEC);Serial.print(" ");Serial.println(i2,DEC);
			if( i1 < 0 ) i1 = 0;
			sum = 0;  npts=0;
			for(i=i1; i<=i2; i++){
				sum += (double)adcmv[i*NCHANS+ic];
				npts++;
			}
			sweepBlk[ib][ic] = (int) round(sum / (double) npts);
			if( i1 ==  0 ) break;
		}
		// MINIMUM
		/**********************************
		MINIMUM VALUES AT THE MIN INDEX
		***********************************/
		sweepBlk[NBLKS][ic] = Smin[ic];  // single min at block 11
	}
	return;
}

void PackBlock( void )
/**************************************************
Pack the sweep by sweep data into a character block
A description of the lines.
STANDBY (Low mode)
 headtemp, adc[chan0], adc[chan1],...adc[chan6],
OPERATE MODE
 headtemp, shadowratio*10, threshold*10, globalleft[chan0]...globalleft[chan6],
   globalright[chan0],...globalright[chan6], sweepblocks[chan0], sweepblocks{chan1],...
   sweepblock[chan6].
***************************************************/
{
	unsigned long	ichar;
	int	ib, ic;
	Serial.println("PackBlock");	
	df_index = 0;
	strcpy(datafile,"$FSR03,\0");		
	df_index+=7;
// 	datafile[df_index]='$';df_index++;
// 	datafile[df_index]='F';df_index++;
// 	datafile[df_index]='S';df_index++;
// 	datafile[df_index]='R';df_index++;
// 	datafile[df_index]='0';df_index++;
// 	datafile[df_index]='3';df_index++;
// 	datafile[df_index]=',';df_index++;
	if(ee.FrsrState == ON){
		//Place a marker to signify daytime record
		datafile[df_index] = 'H';
	} else {
		//Place a marker to signify OFF
		datafile[df_index] = 'L';
	}
	df_index++;
	datafile[df_index]=',';df_index++;
	
	// Head Temperature - 2 bytes -20.0 to 40.0 and more
	ichar = (unsigned int) ((T1 + 20) * 10 + 0.5);
	PsuedoAscii( ichar, 2);
	ichar = (unsigned int) ((T2 + 20) * 10 + 0.5);
	PsuedoAscii( ichar, 2);
	datafile[df_index]=',';df_index++;
// 	// MODE INFORMATION	
	if(ee.FrsrState == ON)
	{
		//shadowratio 2 bytes
		ichar = Shadow[ee.shadowchannel]*10.0;  // from 0 to maybe 400
		PsuedoAscii( ichar, 2);
		
		// Threshold shadow ratio - 2 bytes
		ichar = ee.shadowlimit*10;   // limit=[0-127] -> [0-1270]
		PsuedoAscii( ichar, 2);
		datafile[df_index]=',';df_index++;
		
		// G1 - 2 bytes each
		for(ic=0; ic<NCHANS; ic++) {
			PsuedoAscii( G[ic][0], 2);
		}
	datafile[df_index]=',';df_index++;
		// G2
		for(ic=0; ic<NCHANS; ic++) {
			PsuedoAscii( G[ic][1], 2);
		}
	datafile[df_index]=',';df_index++;
		
		// SWEEPS BIN DATA 
		if(Shadow[ee.shadowchannel] >= ee.shadowlimit){
			for(ic=0; ic<NCHANS; ic++){
				for(ib=0; ib< NBLKS*2+1; ib++) {
					PsuedoAscii(sweepBlk[ib][ic], 2);
				}
				datafile[df_index]=',';df_index++;
			}
		}
	// LOW MODE ============== 
	} else {
		// GLOBAL VALUES - 2 bytes
		//in the low mode the channels are sampled one time.
		for(ic=0; ic<NCHANS; ic++) {
			PsuedoAscii( adcmv[ic], 2);
		}
	}	
	datafile[df_index] = '\0';	
	return;
}
//=================================================================
void PsuedoAscii(unsigned long ul, unsigned int nc)
/************************************
df_index is the pointer to the datafile[df_index] string.
c1 = x % 64 +48; LSB first
c2 = int(x/64) % 64 + 48; MSB
example:  x = 1055
c1 = 79 => O
c2 = 64 => @
ASCII output = @O
*************************************/
{
	unsigned int  n;
	unsigned long u1;
	// clip to the defined range
	u1 = pow((double)64, (double)nc) - 1;  //if nc=2, u1=4095
	if( ul > u1 ) ul = u1;								// clip to max value
	if( ul < 0 ) ul = 0;									// lower clip to zero
	
	// pseudo ascii conversion
	Serial.print("ul=");Serial.println(ul,DEC);
	n=1;
	u1 = ul;
	while( n < nc ) {												//if nc=2, 0,1
		datafile[df_index] = u1 % 64 + 48;	// put the character into the datafile
// 		Serial.print("dfindex=");Serial.println(df_index);
// 		Serial.print("df=");Serial.println(datafile[df_index],DEC);
		df_index++;							// increment the pointer
		u1 = u1/64;							// divide and convert to integer
		n++;								// increment the character number
	}
	// Last character
	datafile[df_index] = u1 % 64 + 48;
// 	Serial.print("dfindex=");Serial.println(df_index);
// 	Serial.print("df=");Serial.println(datafile[df_index],DEC);
	df_index++;
	return;
}
//=================================================================
void FillAdc(void){
	int ix,ic,ip;
	randomSeed(analogRead(0));
	for(ic=0;ic<NCHANS;ic++){
		for(ix=0;ix<NSWEEP;ix++){
			ip=ic+ix*NCHANS;
			if(ix<100)adcmv[ip]=3850+random(150);
			else if(ix>=100 && ix<110) adcmv[ip]=1850+random(150);
			else if(ix>=110 && ix<120) adcmv[ip]=850+random(15);
			else if(ix>=120 && ix<130) adcmv[ip]=1850+random(150);
			else adcmv[ip]=3750+random(150);
		}
	}
}
