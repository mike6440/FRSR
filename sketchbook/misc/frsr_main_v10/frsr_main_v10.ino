//NOTE:
// to do
// 1. fix mfr head t1 & t2
// 2. heater circuit
// 3. 


#define PROGRAMNAME "frsr_main"
#define VERSION		"10"
#define EDITDATE	"20170402T072516Z"
#define	EEPROM_ID  7
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
 √-- Switch to 12bit adc ads1015.  ADS1015_a0 CONVERSIONDELAY = 1
*/ /*v8
 √-- tilt sensor
*/ /* v9
 √-- GPS input
v10 -- Ship ready
*/
//NOTE   INCLUDES
#include <string.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

#include <Wire.h>
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads0(0x48);	// ad0, u13, construct an ads1115 at address 0x48

//NOTE   DEFINES
#define RAIN	4			// RAIN DETECTOR -- Drives the LED
#define NADIR	5		 // INPUT Nadir Switch 1
#define MOTOR	9		  // shadowband motor on/off
#define HEATER	10		   // mfr heater on/off
#define LED11	24		// Green Scan Drum motor
#define LED12	26		// Red shutter motor
#define LED21	28		// green BB2 heater on
#define	NBLKS 	11		// number of avg'd block per side of shadow
#define LED22	30		// red BB1 heat on
#define LED31	32		// green -- continuous on	power
#define LED32	13		// red -- heartbeat. red 5REF
#define REFSW	22		// BB thermistor reference voltage
#define	NOutStr 500  	// size of data storage area
//23bins * 2char/bin * 7chans + 14chars/globals + ~20 header = 361
#define		DONE	3	//gps packet
#define		LENGPS	100	//max expected GPSstring
#define		RMCCOMMAS 11 // sum of commas in the GPRMC packet
// ANALOG ASSIGNMENTS
// three 4-chan 16 bit adc, adc#=(unsigned)(x/4), chan#=x%4;
#define ADCMAX  12		 // 12 normally. maximum number of ADC channels
#define	NSWEEP  250
#define NCHANS  7
// 250*71750 samples in 3 sec  583 samples/sec (583 Hz)
// 16bit adc channels
#define T1adc  0		// MFR Temp 1
#define T2adc  1		// MFR T2

#define NVin  8		// VIN/4
#define Arain 7		// Rain / 4

#define LOWMODECYCLE_usecs	6500000		// sample delay in low mode

// MISC CONSTANTS
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
#define TEST -1
#define LOW 0
#define TRANSITION  2
#define HIGH 1
// SPECTRON TILT
#define C1  0.0129
#define C2  0
#define C3  -0.0000000003
#define P1  1 // use -1 for a reversed direction
#define P0  0
#define R1  1 // use -1 for a reversed direction
#define R0  0

// EEPROM DEFAULT
#define default_FrsrState 1  // 1 => ON
#define default_rain_threshold .090
#define default_rainsecs 600
#define default_pitch_correct 0 //v29
#define default_roll_correct 0  //v29
#define default_mfrheat OFF //OFF
#define default_NadirDisable 0   //flag
#define default_Rref1 10000
#define default_Rref2 10000
#define	SHADOWLIMIT	10
#define SHADOWCHANNEL 3	// filter 2 500 nm
#define NADIRWAITTIME 8000000 	// MICROSECS wait for a nadir
#define default_nadirdelay	1						// millisecs adjust for true bottom
#define default_MfrSetTemp 40
#define default_Low 20		// adc10 at low limit
#define default_TransitionTime 600  // 10 min = 600 sec

//==============================================================
//NOTE GLOBAL VARIABLES
//===============================================================

unsigned long	udum;	// misc use

// GPS special
//char	OutStr[300];
//unsigned int df_index;
char 	bufgps[LENGPS];		// GPS input string
byte	offsets[13];	// indexes of commas and the *
int 	byteGPS;		// GPS processing
int		ctr1, ctr2;		// counters for GPS processing
char 	gpsid[7] = "$GPRMC";

// i/o
char	  		userbuf[20];

// system time
unsigned long msstart;	//set at start, milliseconds
unsigned long menustart;

char	RunMode;
unsigned long modetime;  //millisec setting for mode change

// ADC10
byte ichan;
byte Chan[]={0,1,2,3,4,5,6};
int16_t adcmv[NCHANS*NSWEEP];			// All filter outputs
double	adcx;							// has the value for the mfr head level test

// SWEEP CALCULATIONS
unsigned int Imin[NCHANS];
double	Smin[NCHANS];
double	G[NCHANS][2];	//sweep globals
double	Smean[NCHANS], Sstd[NCHANS];
double	Shadow[NCHANS];
byte	Nsz[NBLKS]={5,5,5,5,5,10,10,10,20,20,30};
int		sweepBlk[NBLKS*2+1][NCHANS];

// OUTPUT STRING
char	OutStr[NOutStr];
unsigned int 	df_index;

// motor
char			MotorPwrFlag;
unsigned long	CycleUsecs; // microsecs between nadirs
unsigned long	NadirTimeUsecs; //Exact time in microsecs at the last nadir
unsigned long 	SampUsecs;
unsigned long 	horiz1usec, horiz2usec;
unsigned long 	t0usecs;
unsigned long 	sweeptime,calctime;

// 	HEATER
char HeaterFlag;
float TempHead;

// ADC16 limits for temperature cal. volts
double AdcThreshold[2] = {0, 6000};
// Temperature limits
double TempLim[2] = {-20,80};

// TILT
double pitch1,pitch2,roll1,roll2;

// MFR TEMPS
double	T1 = 0; // MFR temp T1
double	T2 = 1; // MFR temp T2

// RAIN
unsigned long  millisec_rain;	// the clock count to open.	 0 if open
char	RainState;
unsigned long secs_lapsed;


struct eeprom {
  byte id;
  byte shadowlimit, shadowchannel;
  char FrsrState;
  char NadirDisable;
  float MfrSetTemp;
  float rain_threshold;
  float pitch_correct, roll_correct; 
  unsigned long rainsecs;
  char mfrheat; // on or off
  double Rref[2];
  unsigned long nadirdelay;
  int16_t Low;  //adc counts
  unsigned int TransitionTime; // secs
};
struct eeprom ee;
int eeSize;

int istart; // set to 1 at startup so we can go to a menu.
unsigned int checksum;


//NOTE ====== FUNCTIONS
// i/o
void		Action(char*);
void		MFRHeater(int);
void 		CheckMode(void);
void		CheckRain(double *, unsigned long *);
unsigned int checksum_nmea(char *);
unsigned long ElapsedTime (char *);
void		EepromDefault(void);
void		EepromStore(void);
void		EepromRead(void);
void		EepromPrint(void);
char *		floatToString(char *, double, byte, byte);
void		GetAdc16 (int16_t *, byte chan );   // ads adc
void		GetAdc10 (int16_t *, byte chan );   // mega onboard adc
double 		GetAdc16Volts(byte ch);	 // ads adc
double 		GetAdc10Volts(byte ch);	 // ads adc
double		GetMfrTemp(void);
unsigned int GetThermCoefs(unsigned int, double *coef );   // ads adc
char 		HeaterCheck(void);
long		Hex2Dec(char *);
void		MeanStdev(double *, double *, int, double );
void		MotorOff(void);
void		MotorOn(void);
int 		MoveBuf( char *buf1, char *buf2, int i1, int i2, int numchars);
char		NadirCheck(void);	// Time (millisecs) of nadir
// void 		NadirShutdown(void);
byte 		nmea_validateChecksum(char *strPtr);
void 		PackBlock0( void );
void 		PackBlockH( void );
void		PrintProgramID(void);
void 		PrintBuf( char *buff, int istart, int numchars);
void		PrintBytes(char *);
void 		PsuedoAscii(unsigned long, unsigned int); // fill OutStr[]
byte 		ReadGps(void);
void		ReadTilt(double*, double*);
void		ReadSweep(unsigned int *, int , unsigned long );
unsigned	Ref(int);
void		SendPacket(void);
void		SetMode(char);
double		ShadowRatio(byte chan);
void		SweepAnalysis(void);
int			sign (float);
double		SteinhartHart(double beta[], double r);
unsigned int SweepMinimum(byte chan);
double 		SweepNoise(byte chan, byte, double *mnx, double *stdx); // run after SweepMinimum
void 		SweepGlobals(byte chan, byte nsum, double *g1, double *g2);
void		SweepPack(void);
double		ThermistorTemp(byte ,double Rref);
void 		FillAdcSweep(void);

//============================================================================
void setup() {
	// USER Serial setup
	Serial.begin(115200);
	// TILT
	Serial2.begin(19200);	  // serial
	// GPS
	Serial3.begin(9600);

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
	CycleUsecs=6500000;
	t0usecs=0;

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

	// gps
	Serial3.begin(9600);
	Serial3.setTimeout(200);


	istart = 1;

	userbuf[0]='t'; userbuf[1]='\0'; // begin in menu
}

//=============================================================================
void loop() {
	int	  			i = 0;
	unsigned int 	b = 0;
	double 			ddum, fdum;

	if(Serial.available()>0 || istart == 1){
		userbuf[0] = Serial.read();
		if ( userbuf[0] == 't' || userbuf[0] == 'T' || istart == 1) {
			// heater off
			MFRHeater(OFF);
			// park the shadowband
			if(MotorPwrFlag==ON) NadirShutdown();
			menustart = millis();
			istart = 0; // normal operation
			RunMode = TEST;
			Serial.setTimeout(10000); // 10 sec wait for a key entry
			//Serial.println("TEST mode.");
			while ( RunMode == TEST ) {
				// 10 minute menu 
				//!! menu timeout if ( millis() - menustart > 600000) break;
				// prompt
				Serial.print("> ");
				// Wait for a command
				i = Serial.readBytesUntil(13, userbuf, 11);
				userbuf[i] = '\0';
				if ( i > 0 ) {
					// G - go to main loop
					if ( userbuf[0] == 'G' || userbuf[0] == 'g' ) {
						Serial.println("Operation...");
						istart=1; CheckMode(); istart=0;  // avoids transition mode at startup
						
					} else {
						Serial.println("");
						Action(userbuf);
						menustart = millis();
					}
				}
			}
		}
	}
	GetMfrTemp();
	// ======== HIGH ==============================================================
	if( (RunMode == HIGH || RunMode == TRANSITION) ) {
		//
		MotorOn();
		
		/**************************
		For the first cycle, the arm will go all the way around one
		sweep to sync on the start of the nadir switch on sector.
		On other cycles, the switch will be open after the serial out
		and the sync will take place when the arm hits the
		nadir position.  digitalRead(NADIR) == 0
		***************************/
		if( NadirCheck() == OK ) { 		//NadirTimeUsecs & CycleUsecs
			Serial.print("N ");Serial.print((float)CycleUsecs/1e6,2);
			SampUsecs=(unsigned long) (CycleUsecs / NSWEEP / 2);
			horiz1usec=NadirTimeUsecs+CycleUsecs/4;
			horiz2usec=horiz1usec + CycleUsecs/2;
			ReadGps();
			/*************
			Horizon 1
			**************/
			while( micros() < horiz1usec ); // wait for the time
			
			/**************
			Read tilt at horiz 1
			***************/
			Serial.print(" P");
			ReadTilt(&pitch1, &roll1);
			
			/**************
			Turn off the heater to avoid noise issues
			***************/
			Serial.print("h");
			MFRHeater(OFF); 
			
			/*************
			Read a full block.
			**************/
			Serial.print("S");
			FillAdcSweep(); 
			
			/******************
			Horizon 2 - if necc wait for the correct time
			*******************/
			while( micros() < horiz2usec ); //wait for horiz 2
			
			/**************
			Read tilt at horiz 2
			***************/
			Serial.print(" P");
			ReadTilt(&pitch2, &roll2);
			
			/**************
			head temp and heater on if needed
			***************/
			if( HeaterCheck() ) { Serial.print("H "); MFRHeater(ON); }
			else { Serial.print("h "); MFRHeater(OFF); }
			
			/*********************
			process data
			**********************/
			// sweep calculations, chan=0,1,...,6
			SweepAnalysis(); // minimums, globals, noise, shadow
		
			
			/*****************************
			 PACK DATA TO SEND
			******************************/
			PackBlock0();
			Serial.print("  ");Serial.print(Shadow[ee.shadowchannel-1],1);
			Serial.print("/");Serial.print(ee.shadowlimit,1);
			if(Shadow[ee.shadowchannel-1] >= ee.shadowlimit){
				SweepPack();
				PackBlockH();
			}
			
			/*****************************
			Send out the data file
			******************************/
			SendPacket();
			
			/*****************************
			MODE CHECK -- LOW, TRANSITION, HIGH
			SET NEXT CYCLE START TIME
			*******************************/
			CheckMode();	// check for high,low,transition modes
			if(RunMode == LOW) {
				NadirShutdown();
			}
		} else {
			/**********************************
			NADIR SWITCH FAILS
			Revert to low mode sampling
			***********************************/
			RunMode = LOW;
			MotorOff();
		}
	} else { // RunMode == LOW 
		/**************
		head temp and heater
		***************/
		if( HeaterCheck() ) MFRHeater(ON);
		else MFRHeater(OFF);
		
		CycleUsecs=LOWMODECYCLE_usecs;
		NadirTimeUsecs=micros();
		horiz1usec=NadirTimeUsecs+CycleUsecs/4;
		horiz2usec=horiz1usec+CycleUsecs/2;
		ReadGps();
		
		// horizon 1 time
		while( micros() < horiz1usec ); // wait for the time
		// Global 1
		for(ichan=0;ichan<NCHANS;ichan++){
			G[ichan][0]=GetAdc10Volts(Chan[ichan]);
			//Serial.print(G[ichan][0],1);Serial.print(", ");
		}
		// Tilt 1
		Serial.print(" p");
		ReadTilt(&pitch1, &roll1);

		// horizon 2 time
		while( micros() < horiz2usec ); // wait for the time

		// Tilt 2
		Serial.print(" p ");
		ReadTilt(&pitch2, &roll2);
		// Global 2
		for(ichan=0;ichan<NCHANS;ichan++){
			G[ichan][1]=GetAdc10Volts(Chan[ichan]);
			//Serial.print(G[ichan][1],1);Serial.print(" ");
		}

		/*****************************
		 PACK DATA TO SEND
		******************************/
		PackBlock0();
		
		/*****************************
		Send out the data file
		******************************/
		SendPacket();
		
		// MODE CHECK
		CheckMode();
		/*********************
		At this point the system is in LOW mode. If the RunMode comes back HIGH/TRANSITION
		the motor must be started and the system synched.
		**********************/
		if(RunMode==HIGH){
			MotorOn();
			NadirTimeUsecs = micros();  // assume the band is at nadir
		}
		// LOW mode, wait for default time.
		else while( micros() - NadirTimeUsecs < CycleUsecs );
	}
}


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
	Serial.println("ETff.f   -- MFR Set Temp");
	Serial.println("EFn      -- Motor ON/OFF, n = 1/0");
	Serial.println("");
	Serial.println("------- FUNCTIONS -----------------------------------");
	Serial.println("a -- ADC10 Chan 0-8           A  -- ADC16 Chan n(0-3)");
	Serial.println("b -- Hemisphere sample        B  -- ");
	Serial.println("C  -- shadowband motor ON     c  -- shadowband OFF");
	Serial.println("D  -- 5REF ON                 d  -- 5REF OFF");
	Serial.println("k  -- GPS");
	Serial.println("H  -- MFR heater ON           h  -- shadowband OFF");
	Serial.println("j  -- Thermister 1 & 2        J  -- MFR Temperature");
	Serial.println("n  -- Nadir switch            N  -- nadir loop microsecs");
	Serial.println("p  -- pitch/roll");
	Serial.println("r  -- Rain check single       R  -- Rain check loop");
	Serial.println("t  -- Read system clock       T  -- Set default f.p. day");
	Serial.println("u -- Test floatToString");
	Serial.println("v/V -- Program version");
	Serial.println("g/G -- Continue sampling.");
	}
	   
  // READ ADC10 CHANNEL
	else if (cmd[0] == 'a') {
		Serial.println("a0    a1    a2    a3    a4    a5    a6    a7    a8");
		while ( !Serial.available()	 ) {
			for(ib=0;ib<9;ib++){
				ddum=GetAdc10Volts(ib);
				Serial.print(ddum,1); Serial.print("   ");
			}
			Serial.println("");
			delay(1000);
		}
	}
	// READ ADC16 CHANNEL
	else if (cmd[0] == 'A' && strlen(cmd) > 1) {
		if(strlen(cmd)==2){ib = cmd[1] - 48;}
		else{ ib=10*(cmd[1]-48)+(cmd[2]-48); }
		if (ib < 0 || ib > 6) {
			Serial.println("Error");
		} else {
			Serial.print("Chan "); Serial.println(ib);delay(10);
			while ( !Serial.available()	 ) {
				GetAdc16(&i16,ib);
				Serial.print(ib);Serial.print("  ");Serial.println(i16);
				delay(500);
			}
		}
	}
	// ADC LOOP
	else if (cmd[0] == 'b') {
		iz=0; if(cmd[1] == 'w') iz=1;
		//Loop 
		MotorOn(); // MotorPwrFlag = ON
		NadirCheck(); Serial.println(CycleUsecs);
		NadirCheck();
		Serial.print("N ");Serial.print(CycleUsecs);
		ReadGps();
		SampUsecs=(unsigned long) (CycleUsecs / NSWEEP / 2);
		horiz1usec=NadirTimeUsecs+CycleUsecs/4;
		horiz2usec=horiz1usec+CycleUsecs/2;
		
		// head temp
		T1 = ThermistorTemp(T1adc, ee.Rref[0]);
		T2 = ThermistorTemp(T2adc, ee.Rref[1]);
		GetMfrTemp();

		// Wait for horizon 1
		while( micros() < horiz1usec ); // wait for the time
		Serial.print("  horiz1 ");Serial.print(micros()-NadirTimeUsecs);
		
		// pitch, roll
		ReadTilt(&pitch1, &roll1);
		
		// sweep
		FillAdcSweep(); 
		
		// Wait for horizon 2
		while( micros() < horiz2usec ); // wait for the time
		Serial.print("  horiz2 ");Serial.print(micros()-NadirTimeUsecs);
		
		// pitch, roll
		ReadTilt(&pitch2, &roll2);
		
		// elapsed time
		calctime = micros();
		sweeptime=calctime - sweeptime;
		
		// sweep calculations, chan=0,1,...,6
		SweepAnalysis(); // minimums, globals, noise, shadow
		
		//block average
		PackBlock0();
		if(Shadow[ee.shadowchannel-1] >= ee.shadowlimit){
			SweepPack();
			PackBlockH();
		}
		Serial.print("  p ");Serial.println(micros()-NadirTimeUsecs);
		//computation   time
		calctime=micros()-calctime;
		
		// wait for Nadir
		NadirShutdown(); 
		
		// PRINT OUT SWEEP
		if(iz == 1){
			isamp=0;
			for(iy=0; iy<NSWEEP; iy++){
				Serial.print(iy,DEC); Serial.print(" "); // sample count
				for(iz=0; iz<NCHANS; iz++){
					Serial.print(adcmv[isamp],DEC); Serial.print(" ");
					//Serial.print(isamp); Serial.print(" ");
					isamp++;
				}
				Serial.println("");
			}
		}
		// PRINT OUT STATS
//char * floatToString(char * outstr, double val, byte precision, byte widthp)
/********
 * Convert a double prec variable to a string of characters.
 * Example	 floatToString(s, 123.456, 2, 8) returns
 * s = [' ',' ','1','2','3','.','4','6','\0'] for 8 characters and a NULL
 *********/
 		floatToString(userbuf,T1,1,4);
 		Serial.print("T1 ");PrintBuf( userbuf, 0, 4);
 		floatToString(userbuf,T2,1,4);
 		Serial.print("   T2 ");PrintBuf( userbuf, 0, 4);Serial.println("");
		Serial.println(bufgps);
		Serial.print("Sample time: "); Serial.print(sweeptime,DEC); Serial.println(" msecs");		
		Serial.print("Comptime time: "); Serial.print(calctime,DEC); Serial.println(" msecs");
		Serial.print("pitch1=");Serial.print(pitch1,2);Serial.print("   pitch2=");Serial.println(pitch2,2);
		Serial.print("roll1=");Serial.print(roll1,2);Serial.print("   roll2=");Serial.println(roll2,2);
		Serial.print("Shadow channel: ");Serial.println(ee.shadowchannel);
		for(ib=0; ib<NCHANS; ib++){
			ix= Imin[ib]*NCHANS + ib;
			Serial.print(ib);Serial.print("  ");Serial.print(Imin[ib]);
			Serial.print("  ");Serial.println(adcmv[ix]);
		}
		Serial.print("Global 1:");
		for(ib=0; ib<NCHANS; ib++){
			Serial.print("  ");Serial.print(G[ib][0],1);
		}
		Serial.println("");
		Serial.print("Global 2:");
		for(ib=0; ib<NCHANS; ib++){
			Serial.print("  ");Serial.print(G[ib][1],1);
		}
		Serial.println("");
		Serial.print("Sweep mean:");
		for(ib=0; ib<NCHANS; ib++){
			Serial.print("  ");Serial.print(Smean[ib],1);
		}
		Serial.println("");
		Serial.print("Sweep stdev:");
		for(ib=0; ib<NCHANS; ib++){
			Serial.print("  ");Serial.print(Sstd[ib],1);
		}
		Serial.println("");
		Serial.print("Shadowratio:");
		for(ib=0; ib<NCHANS; ib++){
			Serial.print("  ");Serial.print(Shadow[ib],2);
		}
		Serial.println("");
		Serial.print("Limit ");Serial.print(ee.shadowlimit);
		  Serial.print("  Shadow ");Serial.println(Shadow[ee.shadowchannel-1]);
		  
		//Packet
		if(Shadow[ee.shadowchannel-1] >= ee.shadowlimit){
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
		SendPacket();
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
	else if (cmd[0] == 'C') {
		MotorOn();
	}
	// MFR HEATER OFF
	else if (cmd[0] == 'c') {
		// Wait for the nadir to go from 1 to 0
		NadirShutdown();
	}
	// 5REF	ON/OFF
	else if (cmd[0] == 'D' ) {
		Serial.println("5REF ON");
		Ref(ON);
	}
	else if (cmd[0] == 'd' ) {
		Serial.println("5REF OFF");
		Ref(OFF);
	}
	// GPS
	else if (cmd[0] == 'k') {
		while ( !Serial.available()	 ) {
			byte j;
			unsigned long t1,t2;
			df_index=0;	
			Ldum=millis();
			ReadGps();
			Ldum=millis()-Ldum;
			Serial.print(Ldum);Serial.print("   ");
			Serial.println(bufgps);
			delay(4000);
		}
	}	
	// THERMISTER
	else if (cmd[0] == 'j') {
		while ( !Serial.available()	 ) {
			//T1
			fdum=ThermistorTemp(0, ee.Rref[0]);
			Serial.print(fdum,2);
			//T2
			fdum=ThermistorTemp(1, ee.Rref[1]);
			Serial.print("  ");Serial.println(fdum,2);
			delay(500);
		}
	}
	else if (cmd[0] == 'J') {
		GetMfrTemp();
		Serial.print("Tmfr=");Serial.println(TempHead,2);
	}  
	// NADIR SWITCH
	else if (cmd[0] == 'n') {
		MotorOn();
		Serial.println("0=CLOSED   1=OPEN");
		while ( !Serial.available()	 ) {
			Serial.println(digitalRead(NADIR));
			delay(20);
		}
		NadirShutdown();
	}
	else if (cmd[0] == 'N') {
		MotorOn();
		while(1){
			NadirCheck();
			Serial.println(CycleUsecs);
			if ( Serial.available() ) break;
		}
		NadirShutdown();
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
	else if (cmd[0] == 'u') {
		Serial.println("floatToString(userbuf, 123.456, 2, 6);  ");
		floatToString(userbuf, 123.456, 2, 6);
		Serial.println(userbuf);
		Serial.print("strlen=");Serial.println( strlen(userbuf));
		Serial.println("Move to OutStr[10]");ix=MoveBuf(userbuf,OutStr,0,10,6);
		Serial.print("strlen OutStr[10]=");Serial.println( strlen(OutStr[10]));
		Serial.print("MoveBuf index=");Serial.println( ix );
		Serial.print("OutStr:");PrintBuf(OutStr,10,6);Serial.println("");
	}
	// EEPROM
	else if ( cmd[0] == 'E' || cmd[0] == 'e') {
		if ( strlen(cmd) <= 1 ) {
			EepromPrint();
		}
		else if (strlen(cmd) > 2) {
			eok = 0;
			if ( cmd[1] == 'F' ) {
				Serial.print("FRSR State: ");
				ee.FrsrState = atoi(cmd + 2);
				Serial.println(ee.FrsrState);
				eok = 1;
			}
			else if ( cmd[1] == 'j' ) { 
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
				Serial.print("Delay to Nadir (millisecs): ");
				ee.nadirdelay = atoi(cmd + 2);
				Serial.println(ee.nadirdelay);
				eok = 1;
			}
			else if ( cmd[1] == 'D' ) {
				Serial.print("TransitionTime (secs): ");
				ee.TransitionTime = atoi(cmd + 2);
				Serial.println(ee.TransitionTime);
				eok = 1;
			}
			else if ( cmd[1] == 's' ) {
				Serial.print("Shadow Limit: ");
				ee.shadowlimit = atoi(cmd + 2);
				Serial.println(ee.shadowlimit);
				eok = 1;
			}
			else if ( cmd[1] == 'S' ) {
				Serial.print("Shadow Channel: ");
				ee.shadowchannel = atoi(cmd + 2);
				Serial.println(ee.shadowchannel);
				eok = 1;
			}
			else if ( cmd[1] == 'L' ) {
				Serial.print("Low threshold: ");
				ee.Low = atoi(cmd + 2);
				Serial.println(ee.Low);
				eok = 1;
			}
			else if ( cmd[1] == 'T' ) {
				ddum = atof(cmd + 2);
				Serial.print("MFR SET TEMP = ");
				Serial.println(ddum, 2);
				ee.MfrSetTemp = ddum;
				eok = 1;
			}
			else if ( cmd[1] == 'E' ) {
				Serial.print("SET MODE 0=L, 1=H: ");
				ix = atoi(cmd + 2);
				if (ix == 0) {
					Serial.println("LOW");
				}
				else if (ix == 1) {
					Serial.println("HIGH");
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
			if ( eok == 1 ) {
			EepromStore();
			EepromPrint();
			}
		}
	}


	// PITCH/ROLL
	else if (cmd[0] == 'p') {
		Serial.println(" N  PITCH  ROLL");
		ix = 0;
		while (! Serial.available()) {
			// index
			Serial.print(ix, DEC);
			Serial.print("   ");
			ReadTilt(&ddum, &fdum);
			Serial.print(ddum, 2);
			Serial.print("    ");
			Serial.println(fdum, 2);
			delay(1000);
			ix++;
		}
	}
	// RAIN
	else if (cmd[0] == 'r') {
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
		RunMode == HIGH;
	}
	// DEFAULT
	else {
		Serial.print(cmd);
		Serial.println(" not recognized.");
	}
	return;
}
//======================================================================================
void CheckMode(void)
/**********************************
Check the PSP reading.
The low cutoff set by ee.Low (in millivolts)
rmr 170305
**********************************/
{
	/***************
	Read the adc counts from the selected channel
	****************/	
	adcx=GetAdc10Volts(ee.shadowchannel-1);
	Serial.print(ee.shadowchannel-1);
	Serial.print("/"); Serial.print(adcx,1);
	Serial.print("/"); Serial.println(ee.Low);
	// SET TO LOW MODE IF FORCED BY FRSRSTATE VARIABLE
	if( ee.FrsrState == 0 ) {
		SetMode(LOW);
	}
	// Low or Transition -->> High
	else if( (int)adcx >= ee.Low ) {
		if( RunMode != HIGH ){
			SetMode(HIGH);
		}
	}
	// BELOW THRESHOLD
	else{
// 		Serial.print(" cm "); Serial.print(millis());
// 		Serial.print("  "); Serial.print(modetime);
		// start up
		if(istart==1) SetMode(LOW);
		// Transition -->> Low
		else if( RunMode == TRANSITION && millis() > modetime ){ 
			SetMode(LOW);
		} 
		// High -->> Transition
		else if( RunMode == HIGH ) {
			modetime = millis() + (unsigned long) ee.TransitionTime*1000;
			SetMode(TRANSITION);
		}
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

  *v=GetAdc10Volts(Arain);  //adc counts. volts = adc*0.004906
  
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
//*******************************************************************
void EepromDefault() {
  int i;
  Serial.println("Initialize eeprom...");
  ee.id = EEPROM_ID;
  ee.FrsrState = default_FrsrState;
  ee.shadowlimit = SHADOWLIMIT;
  ee.shadowchannel = SHADOWCHANNEL;
  ee.rain_threshold = default_rain_threshold;
  ee.rainsecs = default_rainsecs;
  ee.Rref[0]=default_Rref1;
  ee.Rref[1]=default_Rref2;
  ee.mfrheat = OFF;
  ee.pitch_correct = default_pitch_correct;
  ee.roll_correct = default_roll_correct;
  ee.nadirdelay = default_nadirdelay;
  ee.FrsrState = ON;
  ee.MfrSetTemp = default_MfrSetTemp;
  ee.Low = default_Low;
  ee.TransitionTime = default_TransitionTime;
  ee.NadirDisable = OFF;
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
  // address of eeprom structure
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
  // pointer to structure ee
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
	int i;
	Serial.println("EepromPrint: ");
	Serial.print("  ID = ");  Serial.println(ee.id);
	Serial.print("  F FrsrState = "); Serial.println(ee.FrsrState);
	Serial.print("  s Shadow limit	= "); Serial.println(ee.shadowlimit);
	Serial.print("  S Shadow channel	= "); Serial.println(ee.shadowchannel);
	Serial.print("  R Rain threshold	= ");Serial.print(ee.rain_threshold, 2);Serial.println(" volts");
	Serial.print("  r Rain shutter delay	= ");Serial.print(ee.rainsecs);Serial.println(" secs");
	Serial.print("  j pitch correct = "); Serial.println(ee.pitch_correct, 1);  //v29
	Serial.print("  J roll correct correct = ");	Serial.println(ee.roll_correct, 1);	 //v29
	Serial.print("  L Low adc value = ");  Serial.println(ee.Low);
	Serial.print("  D TransitionTime (secs) = ");  Serial.println(ee.TransitionTime);
	Serial.print("  N nadir msec delay to bottom = ");Serial.println(ee.nadirdelay);
	Serial.print("  T mfer temp set point = "); Serial.println(ee.MfrSetTemp);
	Serial.print("  MFR heater configuration = ");	Serial.println(ee.mfrheat, DEC);
	Serial.print(" Rref = ");Serial.print(ee.Rref[0],1); Serial.print("  ");Serial.println(ee.Rref[1],1);
	Serial.print(" NadirDisable = ");Serial.println(ee.NadirDisable);
	return;
}
/***************************************************************************************/
unsigned long ElapsedTime (char *ddhhmmss) {

  unsigned long Ld, Lh, Lm, Ls, ms;
  char ch[3];
  ddhhmmss[0] = '\0';

  // Elapsed time, days, since startup
  ms = millis() - msstart;
  if ( ms < 0 ) { msstart = millis(); ms = 0; }

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
//=================================================================
void FillAdcSweep(void){
	unsigned int isamp,iy;
	byte ib;
	
	isamp=0;
	sweeptime=micros();  // mark first sample time
	for(iy=0; iy<NSWEEP; iy++){
		for(ib=0; ib<NCHANS; ib++){
			GetAdc10( (adcmv+isamp), Chan[ib] );
			isamp++;
		}
		sweeptime+=SampUsecs;
		while(micros()<=sweeptime){}
	}
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
	
	if(val == MISSING) {
		outstr[0]='-';outstr[1]='9';outstr[2]='9';outstr[3]='9';outstr[4]='\0';
		return outstr;
	}
	
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
void GetAdc10 (int16_t *iv, byte chan) {
	
	*iv = analogRead(chan);
	
	return;
}
//======================================================================================
double GetAdc10Volts(byte ch) {
  // GetAdc10Volts takes nsap (=10) readings of the ADC10 channel ch. 
  // Tosses out the min and max
  // voltages and takes the mean of the remainder.
  //.0049 v / count

  int16_t v, vmn, vmx;
  double vsum, vmean;
  int16_t vi[10];
  unsigned int i, imx, imn, nav;
  nav = 10;
  vsum = 0;
  vmx = -10;
  vmn = 1024;
  imx = imn = 0;
  //  10 samples
  for (i = 0; i < nav; i++) {
		GetAdc10( &v, ch);
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
  //return vmean*0.0043;    return vmean*0.004906; //based on actual volts
  return vmean;
}
//======================================================================================
void GetAdc16 (int16_t *iv, byte chan) {
	//GetAdc16 returns a single read of 16 bit adc channel chan.
	int16_t adcz;
	if (chan >= 0 && chan <= 3){
		adcz = ads0.readADC_SingleEnded(chan);
	}
//   else if (chan >= 4 && chan <= 7)
// 		adcz = ads1.readADC_SingleEnded(chan - 4);
//   else if (chan >= 8 && chan <= 11)
// 		adcz = ads2.readADC_SingleEnded(chan - 8);
	else adcz = 0;
	*iv = (int16_t) ((float)adcz / 8);
	return;
}
//======================================================================================
double GetAdc16Volts(byte ch) {
  //	  okflag = GetAdc16Volts(ch, *vmean){
  // GetAdc16Volts takes nsap (=10) readings of the ADC channel ch. Tosses out the min and max
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
		GetAdc16( &v, ch);
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
//==============================================================
double GetMfrTemp(void) {
/*=======================
First try thermister 1 and if that fails try thermister 2
===========================*/
	TempHead=ThermistorTemp(T1adc,ee.Rref[0]);
	if(TempHead == MISSING) TempHead = ThermistorTemp(T2adc,ee.Rref[1]);	
	return TempHead;
}
//============================================================
unsigned int GetThermCoefs(double c[] ) {
//basic SHH coefs for ysi44006

	double tcal[3] = { 1.025579e-03,	  2.397338e-04,	  1.542038e-07 };

	c[0] = tcal[0];
	c[1] = tcal[1];
	c[2] = tcal[2];
	return 1;
}
//======================================================================================
char HeaterCheck(void)
/*********************************
If necessary, operate the heater
1. read temperature -> TempHead degC
2. Check if necc to turn on heater
return
 0=no heater pulse required
 1=head was cold and a pulse was necessary
170305 rmr adapt from PRP sw.
**********************************/
{
	GetMfrTemp(); // TempHead is temperature
	/************
	Bad temperature -- TURN OFF HEATER, missing = -999
	*************/
	if( TempHead < -40 || TempHead > 55 ) { 
		TempHead=MISSING; 
		MFRHeater(OFF);
		return 0; 
	}

	/****************
	IF HEATER IS OFF AND TEMP < LOW THRESHOLD TURN ON
	ELSE LEAVE OFF
	******************/
	if( HeaterFlag == OFF ) {
		if( TempHead < ee.MfrSetTemp - 0.2 ) return 1;
		else return 0;
	}
	/*****************
	IF HEATER IS ON AND TEMP > HIGH THRESHOLD TURN OFF
	ELSE LEAVE ON
	******************/
	else {
		if( TempHead > ee.MfrSetTemp + 0.2 ) return 0;
		else return 1;
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
		HeaterFlag=ON;
	}
	if (k == OFF) {
		digitalWrite(HEATER, LOW);
		digitalWrite(LED22, LOW);
		HeaterFlag=OFF;
	}
  return;
}
void MotorOn (void)
/*********************************************************
Turn on Switch Power B for motor
170305 moved from prp2 sw
************************************************************/
{
	if(MotorPwrFlag == OFF) {
		//Serial.print("M");
		digitalWrite(MOTOR, HIGH);
		digitalWrite(LED22, HIGH);
		MotorPwrFlag = ON;
	}
	return;
}

void MotorOff (void)
/*************************************************************
Turn off Switch Power B, the motor
170305 moved from prp2 sw
**************************************************************/
{
	if( MotorPwrFlag == ON ) {
		//Serial.print("m");
		digitalWrite(MOTOR, LOW);
		digitalWrite(LED22, LOW);
		MotorPwrFlag = OFF;
	}
	return;
}
char NadirCheck(void)
/**********************************
If the motor is off, then just jump out.
If the motor is on, then
	first: wait for the switch to open, then
	second: wait for the switch to close
	third: return
output: NadirTimeUsecs = microsecs at nadir.
        CycleUsecs = microsecs for a full cycle
170305--adapted from PRP sw
N. Ncx  Ncox  Nco No
170305
**********************************/
{
	unsigned long t0,t0usecs;						// microsecs

	/********************
	Wait for the
	nadir switch to close.  Use a timeout
	to ensure the arm is turning and the
	switch is functional
	**********************/
	if( MotorPwrFlag == ON) {
		t0=micros();
		/*******************
		Called when nadir switch is closed, still in nadir zone
		*******************/
		if( digitalRead(NADIR) == 0 ) {
			//Serial.print("c");
			while(digitalRead(NADIR) == 0) {  // waiting for the switch to go to 1
				if( micros()-t0 > NADIRWAITTIME){ Serial.print("x2 "); return NOTOK; }
			}
		}
		/***********************
		called when nadir switch is open.
		***********************/ 
		//Serial.print("o");
		while(digitalRead(NADIR) == 1) {  // waiting for the switch to go to 0
			if( micros()-t0 > NADIRWAITTIME){ Serial.print("x3 ");return NOTOK; }
		}
		/******************
		The arm may already be in its nadir position.
		We check for this situation and update the
		timer only if this is a new nadir position.
		 *******************/
		delay(ee.nadirdelay);  // adjust for true bottom millisecs
		t0usecs = micros();
		CycleUsecs = t0usecs - NadirTimeUsecs;
		NadirTimeUsecs = t0usecs;
		if ( CycleUsecs < 4000000 ){Serial.print(CycleUsecs);Serial.println(" x4");CycleUsecs=6400000;}
		if ( CycleUsecs > 8000000 ){ CycleUsecs=6500000;}
		return OK;
	} 
	return OK;
}
//======================================================================
void NadirShutdown(void)
{
	unsigned long t0;
	//Serial.print("NadirShutdown: MotorPwrFlag="); Serial.println(MotorPwrFlag);
	//wait for nadir==1
	t0=millis();
	while(digitalRead(NADIR) == 0 ){
		if(millis()-t0>10000){
			Serial.println("NadirShutdown timeout.");
			MotorOff();
			return;
		}
		delay(10);
	}
	while(digitalRead(NADIR) == 1 ){
		if(millis()-t0>10000){
			Serial.println("NadirShutdown timeout.");
			MotorOff();
			return;
		}
		delay(10);
	}
	MotorOff();
}
//================================================================================
void PackBlock0( void )
{
	unsigned long	ichar;
	int	ib, ic;
	char bufx[10];
	
	//Serial.println("PackBlock");	
	df_index = 0;
	strcpy(OutStr,"$FSR03,\0");		
	df_index+=7;
		
	/*****************
	FrsrState OFF when motor is off and no sweep data are collected.
	******************/
	if(RunMode == HIGH) OutStr[df_index] = 'H';
	else if (RunMode == LOW) OutStr[df_index] = 'L';
	else OutStr[df_index] = 'T';
	df_index++;
	OutStr[df_index]=',';df_index++;
	
//  		floatToString(userbuf,T1,1,6);
//  		Serial.print("T1 ");PrintBuf( userbuf, 0, strlen(userbuf));
//  		floatToString(userbuf,T2,1,6);
//  		Serial.print("   T2 ");PrintBuf( userbuf, 0, strlen(userbuf));Serial.println("");
// 		Serial.println(bufgps);
	// MFR Temp
	floatToString(userbuf, (double)TempHead, 1, 4);
	strcat(OutStr,userbuf); df_index+=4;
	
	// GPS
	strcat(OutStr,",<<"); df_index+=3;
	df_index=MoveBuf(bufgps,OutStr,0,df_index,60);
	strcat(OutStr,">>,"); df_index+=3;
	
	
	// Head Temperature - 2 bytes -20.0 to 40.0 and more -- 0 to 600
	ichar = (unsigned int) ((T1 + 20) * 10 + 0.5);
	PsuedoAscii( ichar, 2);
	ichar = (unsigned int) ((T2 + 20) * 10 + 0.5);
	PsuedoAscii( ichar, 2);
	OutStr[df_index]=',';df_index++;

	// pitch
	ichar = (pitch1+20)*100;	// 0--4000
	PsuedoAscii( ichar, 2);
	ichar = (pitch2+20)*100;	// 0--4000
	PsuedoAscii( ichar, 2);
	OutStr[df_index]=',';df_index++;
	// roll
	ichar = (roll1+20)*100;	// 0--4000
	PsuedoAscii( ichar, 2);
	ichar = (roll2+20)*100;	// 0--4000
	PsuedoAscii( ichar, 2);
	OutStr[df_index]=',';df_index++;	
	// G1 - 2 bytes each
	for(ic=0; ic<NCHANS; ic++) {
		PsuedoAscii( G[ic][0], 2);
	}
	OutStr[df_index]=',';df_index++;
	// G2
	for(ic=0; ic<NCHANS; ic++) {
		PsuedoAscii( G[ic][1], 2);
	}
	OutStr[df_index]=',';df_index++;

	//shadowratio 2 bytes
	if(RunMode==LOW) ichar=0;
	else ichar = Shadow[ee.shadowchannel-1]*10.0;  // from 0 to maybe 400
	PsuedoAscii( ichar, 2);		
	// Threshold shadow ratio - 2 bytes
	ichar = ee.shadowlimit*10;   // limit=[0-127] -> [0-1270]
	PsuedoAscii( ichar, 2);
	OutStr[df_index]=',';df_index++;
	
	OutStr[df_index] = '\0';	
	return;
}
//================================================================================
void PackBlockH( void )
{
	unsigned long	ichar;
	int	ib, ic;

	// SWEEPS BIN DATA IF SHADOW
	if(Shadow[ee.shadowchannel-1] >= ee.shadowlimit){
		for(ic=0; ic<NCHANS; ic++){
			for(ib=0; ib< NBLKS*2+1; ib++) {
				PsuedoAscii(sweepBlk[ib][ic], 2);
			}
			OutStr[df_index]=',';df_index++;
		}
	}
	OutStr[df_index] = '\0';	
	return;
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
//=================================================================
void PsuedoAscii(unsigned long ul, unsigned int nc)
/************************************
df_index is the pointer to the OutStr[df_index] string.
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
	n=1;
	u1 = ul;
	while( n < nc ) {												//if nc=2, 0,1
		OutStr[df_index] = u1 % 64 + 48;	// put the character into the OutStr
// 		Serial.print("dfindex=");Serial.println(df_index);
// 		Serial.print("df=");Serial.println(OutStr[df_index],DEC);
		df_index++;							// increment the pointer
		u1 = u1/64;							// divide and convert to integer
		n++;								// increment the character number
	}
	// Last character
	OutStr[df_index] = u1 % 64 + 48;
// 	Serial.print("dfindex=");Serial.println(df_index);
// 	Serial.print("df=");Serial.println(OutStr[df_index],DEC);
	df_index++;
	return;
}
//============================================================================
void		ReadTilt(double *pitch, double *roll)
{
// 	*pitch=1.202;
// 	*roll=3.1415;
// 	Serial.print("pitch/roll  ");Serial.print(*pitch,2);Serial.print(", ");Serial.println(*roll,2);
// 	return;
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



// ulong ComputePeriod(void)
// /**********************************
// Use consecutive nadir transitions
// to compute cycle time.
// RMR 000715
// **********************************/
// {
// 	static unsigned	msec0;    // remember the last time
// 	unsigned 		msec, sweep_msec, l1, l2;  // the
// 	unsigned 		msec_default; // default cycle period
// 	char			str[10];
// 
// 
// 	msec_default = 6200;  // msec default cycle number
// 	l1 = 5900;  l2 = 6900;
// 
// 	msec = TensMilliSecs();  // note time in 100 Hz counts
// 
// 	if(msec0 == 0 )
// 		sweep_msec = msec_default;
// 	else
// 	{
// 		sweep_msec = 10 * (msec - msec0);
// 		sprintf(str," %d",sweep_msec); SerPutStr(str);
// 		if( sweep_msec > l2 || sweep_msec < l1 )
// 		{
// 			sprintf(str,"x "); SerPutStr(str);
// 			sweep_msec = msec_default;
// 		}
// 		else
// 		{
// 			sprintf(str," "); SerPutStr(str);
// 		}
// 	}
// 	msec0=msec;
// 	return (sweep_msec);
// }
// 
// 
//=======================================================================
void SendPacket(void)
{
	Serial.println("");
	//Serial.print(OutStr);
	PrintBuf(OutStr,0,NOutStr);
	checksum = checksum_nmea(OutStr+1);
	Serial.print("*"); 
	Serial.println(checksum,HEX); 
	Serial.println(""); 
	return;
}
//=======================================================================
void SetMode(char mode)
/*****************************
Puts the system in the correct state for a particular mode
mode=-1 -- test, standby park the arm and go to low power mode
rmr 170305
*****************************/
{
	RunMode = mode; // set global variable RunMode to control all operations

	// ANY DETAILS RELATED TO THIS MODE
	switch (mode)
	{
		// TEST MODE
		case TEST:
			Serial.print("-Stby-");
			MotorOff();
			break;

		// LOW MODE
		case LOW:
			Serial.print("-Low-");
			break;

		// TRANSITION MODE
		case TRANSITION:
			Serial.print("-Trans-");
			break;

		// HIGH MODE
		case HIGH:
			Serial.print("-High-");
			break;

		default:
			Serial.println("Bad input to SetMode()");
	}
	return;
}
//===========================================================================
double	ShadowRatio(byte chan)
{
	double shadowratio;
	if(Sstd[chan]<=0){return 0;}
	
	shadowratio = (Smean[chan] - Smin[chan]) / Sstd[chan];
	return max(shadowratio,0);
}
//============================================================================
int sign (float input) {
  return ((input < 0.0) ? NEG : POS);
}
//==============================================================================
void	SweepAnalysis(void) // minimums, globals, noise, shadow
{
	byte ib;
	double ddum,fdum;
	
	for(ib=0;ib<NCHANS;ib++){
		Imin[ib]=SweepMinimum(ib);
		SweepGlobals(ib,10,&ddum,&fdum);
		G[ib][0]=ddum;
		G[ib][1]=fdum;
		SweepNoise(ib, 10, (Smean+ib), (Sstd+ib) );
		Shadow[ib]=ShadowRatio(ib);
	}
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
	
	v=GetAdc16Volts(nadc);	
	//check bad input
	if(v<AdcThreshold[0]) return 0;
	if(v>AdcThreshold[1]) return 50;
	vx = 2.045*GetAdc16Volts(3); // Vref adc16 chan 3
	//Serial.print("vref="); Serial.println(vx,1);
	
	r = Rref * (v / (vx-v));
	//Serial.print("test rtherm="); Serial.println(r,2);
	if(r<1000||r>30000){
		//Serial.print("Error. rtherm="); Serial.println(r,2);
		return MISSING;
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
	if(t<TempLim[0] || t>TempLim[1]) t=MISSING;
	return t;
}
//=================================================================
byte ReadGps(void) {
	byte bytestatus;
	byte j,iTry;
	byte k;

	iTry=0;
	while(iTry<1){
		//Serial.println("");Serial.print("iTry=");Serial.println(iTry);
		byteGPS=-1;
		// reset
		ctr1 = ctr2 = 0;
		// clear the buffer
		for(j=0;j<LENGPS;j++) bufgps[j]='\0';
		// clear serial uart
		while( Serial3.available() ) Serial3.read();
		// Wait for the '$'
		udum = (unsigned long)millis();
		while(1){
			// Serial dead time
			if( millis() - udum > 1200 ) {
				//Serial.println("");Serial.print("millis = ");Serial.println(millis());
				strcpy(bufgps,"$GPRMC,NaN*XX");
				//Serial.println(bufgps);
				return NOTOK;
			}
			// Check for '$'
			//Serial.print("-u-");
			if( Serial3.available() ) {
				k = Serial3.read();
				//Serial.print((char)k);Serial.print("-");
				if( k == '$'){
					//Serial.println("");
					ctr1=1; ctr2=0; bufgps[0]='$';
					break;
				}
			}
			else delay(10);
		}
		// loop reading all bytes from the gps
		j=0;
		udum=millis();
		while(j<LENGPS){
			if(millis()-udum > 200){
				//Serial.println("-to-");
				break;
			}
			if( Serial3.available() ) {
				byteGPS=Serial3.read();
				j++;
				//Serial.print("-");Serial.print((char)byteGPS);
				bytestatus = handle_byte(byteGPS);
				// packet fails, start over
				if( bytestatus == NOTOK ) {		
					break;
				}
				// packet complete
				else if( bytestatus == DONE ){
					bufgps[ctr1-2]='\0';
					return OK;
				}
			}
		}	
		iTry++;
	}
	strcpy(bufgps,"$GPRMC,NaN*XX");
	return NOTOK;	
}
//=============================================
void PrintBuf( char *buff, int istart, int numchars){
	int j;
	
	j=istart;
	while(1){
		Serial.print((char)buff[j]);
		j++; if(j-istart >= numchars) return;
		if(buff[j]=='\0') return;
	}
}
//=============================================
int MoveBuf( char *buf1, char *buf2, int i1, int i2, int numchars)
{
	int j1,j2;
	
	j1=i1; j2=i2;
	while(1){
		buf2[j2]=buf1[j1];
		j1++; j2++;
		if(j1-i1 >= numchars || buf1[j1] == '\0') {
			buf2[j2]='\0';
			return j2; // index of the NULL
		}
	}
}
//============================================================
int handle_byte(int byteGPS) {
	byte j;
	bufgps[ctr1] = byteGPS;
	ctr1++; bufgps[ctr1]='\0';
	if( ctr1 >= LENGPS ) return NOTOK;
	// trap commas
	if( byteGPS == ',' || byteGPS == '*') {
		offsets[ctr2] = ctr1-1;
		//Serial.print("ctr2 ");Serial.print(ctr2);Serial.print(" offsets=");
			//Serial.println(offsets[ctr2]);
		ctr2++;
		if( ctr2 > RMCCOMMAS + 1 ) {
			//Serial.println("-C-");
			return NOTOK;
		}
	}
	// trap CR
	if( byteGPS == 13) {
		//Serial.print("13  CR ");
	}
	// trap LF
	if( byteGPS == 10) {
		// 12 fields and useable header
		if( ctr2 != RMCCOMMAS+1 ) {
			//Serial.print("-F-");
			//Serial.print(byteGPS);Serial.print(",");
			//Serial.print(ctr1-1);Serial.print(",");Serial.println(ctr2);
			//Serial.print("bufgps ");Serial.println(bufgps);
			return NOTOK;
		}
		// header GPRMC
		for( j=0; j<6; j++ ) {
			//Serial.print(bufgps[j]);
			if( bufgps[j] != gpsid[j] ) {
				//Serial.println("-H-");
				return NOTOK;
			}
		}
		//checksum
		if( nmea_validateChecksum(bufgps) != OK ) {
			//Serial.print("-S-");Serial.println();
			return NOTOK;
		}
		return DONE;
	}
	return OK;
}
//=================================================================
byte nmea_validateChecksum(char *strPtr)
// this takes a nmea gps string, and validates it againts the checksum
// at the end if the string. (requires first byte to be $)
// Requires <string.h>
{
	int p,len;
	char c;
	byte chksum;
	char hx[3] = "0x00";
	char hc[3];
	
	//flagValid = OK; // we start OK, and make it false if things are not right
	len = strlen(strPtr);
	if (len <= 6 || strPtr[0] != '$' ) { 
		return NOTOK; 
	} else {
		c = strPtr[1]; // get first chr
		chksum = c;
		p = 2;
		while ( p < len + 1 ) {
			c = strPtr[p]; // get next chr
			if ( c == '*' ) break;
			else {
				chksum = chksum ^ c;
			}
			p++;
		}
		// at this point we are either at * or at end of string
		p++;
		hx[0] = strPtr[p];
		hx[1] = strPtr[p+1];
		hx[2] = '\0';
		hc[0] = (int)chksum/16 + 48;
		hc[1] = (int) chksum % 16 + 48;
		hc[2] = '\0';
		if ( atoi(hx) == atoi(hc) ) return OK; 
		else return NOTOK;
	}
	return NOTOK;
}
