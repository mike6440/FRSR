//NOTE:

#define PROGRAMNAME "frsr_main"
#define VERSION		"14"
#define EDITDATE	"20170825T174009Z"
#define	EEPROM_ID  2
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
// v6
// √-- Add inverting op amps to mimic the frsr-v2
// v7
// √-- Switch to 12bit adc ads1015.  ADS1015_a0 CONVERSIONDELAY = 1
// v8
// √-- tilt sensor
// v9
//  √-- GPS input
// v10 -- Ship ready
// v11 -- √ rain, √ reduce local variables, 
//  √_Choose T1 or T2.  
//	√_Heater on timer.
// v12 -- rmr 170526
//	√_rain timer, see 'r'   √_ autostart from menu
// v13 -- skip. Bad luck
// v14 -- diff heater plan
  
  
//NOTE   INCLUDES
#include <string.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

#include <Wire.h>
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads0(0x48);	// ad0, u13, construct an ads1115 at address 0x48

//NOTE   DEFINES
#define PKTHEADER	"$FSR03,\0"	// the start of the packet
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
#define LowMode 0
#define TRANSITION  2
#define HighMode 1
// SPECTRON TILT
#define C1  0.0129
#define C2  0
#define C3  -0.0000000003
#define P1  1 // use -1 for a reversed direction
#define P0  0
#define R1  1 // use -1 for a reversed direction
#define R0  0
// THERMISTOR
#define BETA0 1.025579e-03
#define BETA1 2.397338e-04
#define BETA2 1.542038e-07
// EEPROM DEFAULT
#define default_FrsrState 1  // 1 => ON
#define default_rain_threshold .090
#define default_rainsecs 600
#define default_pitch_correct 0 //v29
#define default_roll_correct 0  //v29
#define default_mfrheat OFF //OFF
#define default_mfrT 1 //OFF
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

// GENERAL PURPOSE
unsigned long	Udum;	// floatToString
unsigned long frac; // floatToString
unsigned long frac1;  // floatToString
double			fdum,ddum;
double 			x,xs,x1,xs1;

int 			ix,iy,iz;
int16_t 		va, vmn, vmx; //adc
double 			vsum, vmean; //GetAdc10Volts, GetAdc16Volts
unsigned int 	iadc, imx, imn, nav;
unsigned int	iloop, idum;
int16_t 		vi[10];
char 			tempbuf[16];  //floatToString



// GPS special
char 	bufstr[LENGPS];		// GPS input string
byte	offsets[13];	// indexes of commas and the *
int 	byteGPS;		// GPS processing
int		ctr1, ctr2;		// counters for GPS processing
char 	gpsid[7] = "$GPRMC";

// i/o
char	userbuf[11];
char 	tempbuf2[9];

// system time
unsigned long msstart;	//set at start, milliseconds
unsigned long menustart;

char	RunMode;
unsigned long modetime;  //millisec setting for mode change

// ADC10
byte ichan;
byte Chan[]={0,1,2,3,4,5,6};
int16_t adcmv[NCHANS*NSWEEP];			// All filter outputs
double	adcx, adcy;							// has the value for the mfr head level test

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
byte	RainState;  // 0 => dry, 1 => wet
unsigned int secs_lapsed;  // secs in transition
int RainSecs;     // secs until dry state


struct eeprom {
  byte id;
  byte shadowlimit, shadowchannel;
  byte FrsrState;
  //char NadirDisable;
  float MfrSetTemp;
  byte mfrT;
  float rain_threshold;
  float pitch_correct, roll_correct; 
  int rainsecs;  // number of secs to wait
  byte mfrheat; // on or off
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
byte		CheckRain(void);
unsigned int checksum_nmea(char *);
// unsigned long ElapsedTime (char *);  // v11 not implemented
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
void 		Heater(void);
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
	// 4 bytes
	
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
				//menu timeout 
				if ( millis() - menustart > 600000) break;
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
	// ======== HighMode ==============================================================
	if( (RunMode == HighMode || RunMode == TRANSITION) ) {
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
			// force heater off
			Serial.print(" h ");
			//v14 MFRHeater(OFF);
			// read gps
			ReadGps();
			
			/*************
			Horizon 1
			**************/
			while( micros() < horiz1usec ); // wait for the time
			MFRHeater(OFF);  //v14 turn off for the top sweep
			
			/**************
			Read tilt at horiz 1
			***************/
			Serial.print(" P");
			ReadTilt(&pitch1, &roll1);
			
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
			Heater(); //Turn on for lower half.
			
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
			MODE CHECK -- LowMode, TRANSITION, HighMode
			SET NEXT CYCLE START TIME
			*******************************/
			CheckMode();	// check for high,low,transition modes
			if(RunMode == LowMode) {
				NadirShutdown();
			}
		} else {
			/**********************************
			NADIR SWITCH FAILS
			Revert to low mode sampling
			***********************************/
			RunMode = LowMode;
			MotorOff();
		}
	} else { // RunMode == LowMode 
		/**************
		head temp and heater
		***************/
		Heater(); delay(3000); MFRHeater(OFF); //v14 , was 1000 msec under v12
		
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
		}

		/*****************************
		 PACK DATA TO SEND
		******************************/
		PackBlock0();
		/*****************************
		Send out the data file
		******************************/
		//xSerial.print(" df_index=");Serial.print(df_index);
		SendPacket();
		
		// MODE CHECK
		CheckMode();
		/*********************
		At this point the system is in LowMode mode. If the RunMode comes back HighMode/TRANSITION
		the motor must be started and the system synched.
		**********************/
		if(RunMode==HighMode){
			MotorOn();
			NadirTimeUsecs = micros();  // assume the band is at nadir
		}
		// LowMode mode, wait for default time.
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
	// use bufstr[], char bufstr[50];
	char eok = 0;
	//double fdum, ddum;
	byte ib,ic;
	//unsigned int isamp,ix,iy,iz;
	int n, n1;
	int16_t i16;
	// 8 bytes
	
	// TAKE ACTION AND PREPARE AN OUTPUT MESSAGE IN out_message
	if (cmd[0] == '?') {
	PrintProgramID();
	Serial.println("------- FUNCTIONS -----------------------------------");
	Serial.println("a -- ADC10 Chan 0-8           An -- ADC16 Chan n(0-3)");
	Serial.println("b -- Hemisphere sample");
	Serial.println("C  -- shadowband motor ON     c  -- shadowband OFF");
	//Serial.println("D  -- 5REF ON                 d  -- 5REF OFF");
	Serial.println("H  -- MFR heater ON           h  -- MFR heater OFF");
	Serial.println("i  -- Warm up MFR head");
	//Serial.println("i  -- MFR temp hold");
	Serial.println("j  -- Thermister 1 & 2        J  -- MFR Temperature");
	Serial.println("k  -- GPS");
	Serial.println("n  -- Nadir switch            N  -- nadir loop microsecs");
	Serial.println("p  -- pitch/roll");
	Serial.println("r  -- Rain check loop");
	Serial.println("t  -- Read system clock       T  -- Set default f.p. day");
	Serial.println("u -- Test floatToString");
	Serial.println("v/V -- Program version");
	Serial.println("g/G -- Continue sampling.");
	}
	   
  // READ ADC10 CHANNEL
	else if (cmd[0] == 'a') {
		Serial.println("ADC       a0    a1    a2    a3    a4    a5    a6    a7    a8");
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
			iloop=0;
			for(iy=0; iy<NSWEEP; iy++){
				Serial.print(iy,DEC); Serial.print(" "); // sample count
				for(iz=0; iz<NCHANS; iz++){
					Serial.print(adcmv[iloop],DEC); Serial.print(" ");
					//Serial.print(iloop); Serial.print(" ");
					iloop++;
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
		Serial.println(bufstr);
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
		//ix = cmd[1] - 48;
		Serial.print("MFR HEATER ON");
		MFRHeater(ON);
	}
	// MFR HEATER OFF
	else if (cmd[0] == 'h') {
		Serial.println("MFR HEATER OFF");
		MFRHeater(OFF);
	}
	// WARM UP MFR
	else if (cmd[0] == 'i') {
		if(ee.mfrheat == OFF){
			Serial.println("Heater Off");
		} else {
			ix = 0;
			while (! Serial.available()) {
				Serial.print(ix);
				Heater();
				Serial.print(TempHead,2);
				delay(3000);  //v14, under v12 it was 1000
				MFRHeater(OFF);
				Serial.println(" h");
				ix++;
				delay(3000); //v14, under v12 it was 5000
			}
		}
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
			//byte j;
			//unsigned long t1,t2;
			df_index=0;	
			Udum=millis();
			ReadGps();
			Udum=millis()-Udum;
			Serial.print(Udum);Serial.print("   ");
			Serial.println(bufstr);
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
		Serial.print("TempHead = ");Serial.println(TempHead,2);
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
		Serial.println("Elapsed time as dd.hhmmss is not implemented.");
// 		Serial.println("Elapsed time as dd.hhmmss---");
// 		while (1) {
// 			Udum = ElapsedTime(bufstr);
// 			Serial.print("Millisecs elapsed: "); Serial.print(Udum);
// 			Serial.print("   "); Serial.println(bufstr);
// 			if ( Serial.available()) {
// 				break;
// 			}
// 			delay(2000);
// 		}
	}
	else if (cmd[0] == 'u') {
		Serial.println("floatToString(userbuf, 123.456, 2, 6);  ");
		floatToString(userbuf, 123.456, 2, 6);
		Serial.println(userbuf);
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
				ee.FrsrState = atoi(cmd+2);
				Serial.println(ee.FrsrState);
				eok = 1;
			}
			else if ( cmd[1] == 'H' ) {
				ee.mfrheat = atoi(cmd+2);
				Serial.print("HEATER OFF/ON (0/1): ");
				Serial.println(ee.mfrheat);
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
			else if ( cmd[1] == 'r' ) {
				ee.rainsecs = atoi(cmd + 2);
				Serial.print("SET RAIN DELAY SECS = ");
				Serial.println(ee.rainsecs);
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
			//Heater temp
			else if ( cmd[1] == 'T' ) {
				ddum = atof(cmd + 2);
				Serial.print("MFR SET TEMP = ");
				Serial.println(ddum, 2);
				ee.MfrSetTemp = ddum;
				eok = 1;
			}
			// thremistor (0/1)
			else if ( cmd[1] == 't' ) {
				ee.mfrT = atoi(cmd + 2);
				Serial.print("MFR THERM = ");
				Serial.println(ee.mfrT);
				eok = 1;
			}
			else if ( cmd[1] == 'E' ) {
				Serial.print("SET MODE 0=L, 1=H: ");
				ix = atoi(cmd + 2);
				if (ix == 0) {
					Serial.println("LowMode");
				}
				else if (ix == 1) {
					Serial.println("HighMode");
				}
				else {
					Serial.println("UNDEFINED, NO CHANGE");
				}
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
		Serial.println(" N	STATUS	 VOLTS	 SECS ");
		ix = 0;
		while (! Serial.available()) {
			// index
			Serial.print(ix, DEC);
			Serial.print("  ");
			CheckRain();
			Serial.print(RainState, DEC);
			Serial.print("   ");
			Serial.print(adcy, 2);
			Serial.print("  ");
			Serial.println( ee.rainsecs - secs_lapsed );
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
	  digitalWrite(n1, HighMode);
	  delay(500);
	  digitalWrite(n1, LowMode);
	  delay(500);
	}
  }
	// VERSION
	else if (cmd[0] == 'v' || cmd[0] == 'V') {
		PrintProgramID();
	}
	// RETURN TO RUN
	else if (cmd[0] == 'g' || cmd[0] == 'G') {
		RunMode == HighMode;
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
	Read the adc counts from the selected channel 1/50/25
	****************/	
	adcx=GetAdc10Volts(ee.shadowchannel-1);
	Serial.print(ee.shadowchannel-1);
	Serial.print("/"); Serial.print(adcx,1);
	Serial.print("/"); Serial.println(ee.Low);
	// SET TO LowMode MODE IF FORCED BY FRSRSTATE VARIABLE
	// OR IF  IT IS RAINING
	//byte CheckRain(void)
	CheckRain();
	if( ee.FrsrState == 0 || RainState == 1 ) {
		SetMode(LowMode);
	}
	// Low or Transition -->> High
	else if( (int)adcx >= ee.Low ) {
		if( RunMode != HighMode ){
			SetMode(HighMode);
		}
	}
	// BELOW THRESHOLD
	else{
// 		Serial.print(" cm "); Serial.print(millis());
// 		Serial.print("  "); Serial.print(modetime);
		// start up
		if(istart==1) SetMode(LowMode);
		// Transition -->> Low
		else if( RunMode == TRANSITION && millis() > modetime ){ 
			SetMode(LowMode);
		} 
		// High -->> Transition
		else if( RunMode == HighMode ) {
			modetime = millis() + (unsigned long) ee.TransitionTime*1000;
			SetMode(TRANSITION);
		}
	}
	return;
}
//======================================================================================
byte CheckRain()
// Globals
//		adcy = analog (volts)
//		 = seconds until shutter opens
// RETURN 1 or 0 for rain mode and shutter control.
// Global in
//	ee.rain_threshold;
//	ee.rainsecs;
// Global out
//	 unsigned long	millisec_rain;	// clock count for opening
//	 RainState
{

	adcy=GetAdc10Volts(Arain);  //adc counts. volts = adc*0.004906

	//Serial.print("Chan ");	Serial.print(Arain,DEC); Serial.print("	 ");  Serial.println(adcy,4);

	// RAIN
	if ( adcy > ee.rain_threshold ) {
		RainState = 1;
		millisec_rain = millis();
	}
	// NO RAIN
	else {
		// SHUTTER IS CLOSED
		if ( RainState == 1 ) {
			secs_lapsed = (unsigned int) (millis() - millisec_rain)/1000;
			// TIME TO OPEN
			if ( secs_lapsed >= ee.rainsecs ) {
				RainState = 0;
				secs_lapsed = 0;
			}
		}
		// SHUTTER OPEN
		else {
		  RainState = 0;
		}
	}
	return RainState;
}
//***************************************************************************
unsigned int checksum_nmea(char *strPtr) {
  // code --  http://www.mculabs.com/snippets/nmea_checksum.html
  // validate -- http://www.hhhh.org/wiml/proj/nmeaxor.html
//   int p; --> ix
  char c;
  byte chksum;
	// 2 bytes
	
  c = strPtr[0]; // get first chr
  chksum = c;
  ix = 1;
  while ( c != 0x00 ) {
	c = strPtr[ix]; // get next chr
	if ( c != 0x00 ) {
	  chksum = chksum ^ c;
	}
	ix++;
  }
  return chksum;
}
//*******************************************************************
void EepromDefault() {
  Serial.println("Initialize eeprom...");
  ee.id = EEPROM_ID;
  ee.FrsrState = default_FrsrState;
  ee.shadowlimit = SHADOWLIMIT;
  ee.shadowchannel = SHADOWCHANNEL;
  ee.rain_threshold = default_rain_threshold;
  ee.rainsecs = default_rainsecs;
  ee.Rref[0]=default_Rref1;
  ee.Rref[1]=default_Rref2;
  ee.mfrheat = ON;
  ee.mfrT = default_mfrT;
  ee.pitch_correct = default_pitch_correct;
  ee.roll_correct = default_roll_correct;
  ee.nadirdelay = default_nadirdelay;
  ee.MfrSetTemp = default_MfrSetTemp;
  ee.Low = default_Low;
  ee.TransitionTime = default_TransitionTime;
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
  // address of eeprom structure
  byte* a = &(ee.id);

  // for each byte in the eeprom structure
  for (iloop = 0; iloop < eeSize; iloop++) {
	EEPROM.write(iloop, *a );	// store this byte
	a++;
  }
  return;
}

//=============================================================================
void EepromRead()
{
  // pointer to structure ee
  byte* a = &(ee.id);
  // for each byte in the eeprom structure
  for (iloop = 0; iloop < eeSize; iloop++)
  {
	*a = EEPROM.read(iloop);  // get the byte
	a++;
  }
  return;
}

//===============================================================================
void EepromPrint()
{
	Serial.println("EepromPrint: ");
	Serial.print("  ID = ");  Serial.println(ee.id);
	Serial.print("  F FrsrState = "); Serial.println(ee.FrsrState);
	Serial.print("  s Shadow limit = "); Serial.println(ee.shadowlimit);
	Serial.print("  S Shadow channel = "); Serial.println(ee.shadowchannel);
	Serial.print("  R Rain threshold = ");Serial.print(ee.rain_threshold, 2);Serial.println(" volts");
	Serial.print("  r Rain shutter delay = ");Serial.print(ee.rainsecs);Serial.println(" secs");
	Serial.print("  j pitch correct = "); Serial.println(ee.pitch_correct, 1);  //v29
	Serial.print("  J roll correct correct = ");	Serial.println(ee.roll_correct, 1);	 //v29
	Serial.print("  L Low adc value = ");  Serial.println(ee.Low);
	Serial.print("  D TransitionTime (secs) = ");  Serial.println(ee.TransitionTime);
	Serial.print("  N nadir msec delay to bottom = ");Serial.println(ee.nadirdelay);
	Serial.print("  T MFR set temp = "); Serial.println(ee.MfrSetTemp);
	Serial.print("  H MFR heater off/on (0/1) = ");	Serial.println(ee.mfrheat);
	Serial.print("  t MFR Thermistor Number = ");	Serial.println(ee.mfrT);
	Serial.print(" Rref = ");Serial.print(ee.Rref[0],1); Serial.print("  ");Serial.println(ee.Rref[1],1);
	return;
}
/***************************************************************************************/
// unsigned long ElapsedTime (char *ddhhmmss) {
// 
//   unsigned long  ms;
//   unsigned int Ld, Lh, Lm, Ls;
//   char ch[3];
//   // 15 bytes
//   
//   ddhhmmss[0] = '\0';
// 	// 
//   // Elapsed time, days, since startup
//   ms = millis() - msstart;
//   if ( ms < 0 ) { msstart = millis(); ms = 0; }
// 
//   //days
//   Ld = (unsigned int) ms / 86400000;		  // number of days
//   if (Ld >= 0 && Ld < 100) {
// 	ch[2] = '\0'; ch[1] = Ld % 10 + 48; ch[0] = Ld / 10 + 48;
//   } else {
// 	strcpy(ch, "xx");
//   }
//   strcat(ddhhmmss, ch);
//   strcat(ddhhmmss, ".");
//   //	 Serial.print("days = ");  Serial.print(Ld);
//   //	 Serial.print("	  string:");  Serial.print(ch);
//   //	 Serial.print("	  ddhhmmss:");	Serial.println(ddhhmmss);
// 
//   //hours
//   Lh = (unsigned int) (ms - (unsigned long) Ld * 86400000) / 3600000;	  // number hours
//   if (Lh >= 0 && Lh < 100) {
// 	ch[2] = '\0'; ch[1] = Lh % 10 + 48; ch[0] = Lh / 10 + 48;
//   } else {
// 	strcpy(ch, "xx");
//   }
//   strcat(ddhhmmss, ch);
//   //	 Serial.print("hours = ");	Serial.print(Lh);
//   //	 Serial.print("	 string = ");  Serial.print(ch);
//   //	 Serial.print("	 ddhhmmss:");  Serial.println(ddhhmmss);
// 
//   //min
//   Lm = (unsigned int) (ms - (unsigned long) Ld * 86400000 - (unsigned long) Lh * 3600000) / 60000;
//   if (Lm >= 0 && Lm < 100) {
// 	ch[2] = '\0'; ch[1] = Lm % 10 + 48; ch[0] = Lm / 10 + 48;
//   } else {
// 	strcpy(ch, "xx");
//   }
//   strcat(ddhhmmss, ch);
//   //	 Serial.print("mins = ");  Serial.print(Lm);
//   //	 Serial.print("	 string = ");  Serial.print(ch);
//   //	 Serial.print("	 ddhhmmss:");  Serial.println(ddhhmmss);
// 
//   //sec
//   Ls = (unsigned int) (ms - (unsigned long) Ld * 86400000 - (unsigned long) Lh * 3600000 - (unsigned long) Lm * 60000) / 1000;
//   if (Ls >= 0 && Ls < 100) {
// 	ch[2] = '\0'; ch[1] = Ls % 10 + 48; ch[0] = Ls / 10 + 48;
//   } else {
// 	strcpy(ch, "xx");
//   }
//   strcat(ddhhmmss, ch);
//   //	 Serial.print("secs = ");  Serial.print(Ls);
//   //	 Serial.print("	 string = ");  Serial.print(ch);
//   //	 Serial.print("	 ddhhmmss:");  Serial.println(ddhhmmss);
// 
//   return ms;
// }
//=================================================================
void FillAdcSweep(void){
// 	unsigned int iy;
	byte ib;
	// 1 bytes
	
	iloop=0;
	sweeptime=micros();  // mark first sample time
	for(iy=0; iy<NSWEEP; iy++){
		for(ib=0; ib<NCHANS; ib++){
			GetAdc10( (adcmv+iloop), Chan[ib] );
			iloop++;
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
	double roundingFactor = 0.5;
// 4 bytes

	
	if(val == MISSING) {
		outstr[0]='-';outstr[1]='9';outstr[2]='9';outstr[3]='9';outstr[4]='\0';
		return outstr;
	}
	
	// ROUNDING
	Udum = 1;
	for (iloop = 0; iloop < precision; iloop++)
	{ //			 *
	roundingFactor /= 10.0;		// .5, .05, .005, ...
	Udum *= 10;					// 1,  10,	100, ...
	}


	// OUTSTRING
	tempbuf[0] = '\0';
	outstr[0] = '\0';

	// NEGATIVE NUMBERS
	if (val < 0.0) {
	strcpy(outstr, "-\0");
	val = -val;
	}

	// 123.461
	val += roundingFactor;
	strcat(outstr, ltoa(long(val), tempbuf, 10));   //prints the int part

	if ( precision > 0) {
	strcat(outstr, ".\0"); // print the decimal point

	Udum = 1;
	byte padding = precision - 1;
	while (precision--)
	  Udum *= 10;

	if (val >= 0)
	  frac = (val - long(val)) * Udum;
	else
	  frac = (long(val) - val ) * Udum;

	frac1 = frac;
	while (frac1 /= 10)
	  padding--;

	while (padding--)
	  strcat(outstr, "0\0");

	strcat(outstr, ltoa(frac, tempbuf, 10));
	}
	//Serial.print("test2 ");Serial.println(outstr);

	// generate space padding
	if ((widthp != 0) && (widthp >= strlen(outstr))) {
	byte J = 0;
	J = widthp - strlen(outstr);

	for (iloop = 0; iloop < J; iloop++) {
	  tempbuf[iloop] = ' ';
	}

	tempbuf[iloop++] = '\0';
	strcat(tempbuf, outstr);
	strcpy(outstr, tempbuf);
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

  
  nav = 10;
  vsum = 0;
  vmx = -10;
  vmn = 1024;
  imx = imn = 0;
  //  10 samples
  
  for (iadc = 0; iadc < nav; iadc++) {
		GetAdc10( &va, ch);
		vi[iadc]=va;		// in millivolts
		vsum += (double)va ;
		//max and min
		if (va < vmn) {
	  	imn = iadc;
	  	vmn = va;
		}
		if (va > vmx) {
			imx = iadc;
			vmx = va;
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
	// 2 bytes
	
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

  nav = 10;
  vsum = 0;
  vmx = -4096;
  vmn = 4096;
  imx = imn = 0;
  //  10 samples
  for (iadc = 0; iadc < nav; iadc++) {
		GetAdc16( &va, ch);
		vi[iadc]=va;		// in millivolts
		vsum += (double)va ;
		//max and min
		if (va < vmn) {
	  	imn = iadc;
	  	vmn = va;
		}
		if (va > vmx) {
			imx = iadc;
			vmx = va;
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
	TempHead=MISSING;
	if(ee.mfrT==1) TempHead=ThermistorTemp(T1adc,ee.Rref[0]);
	else if(ee.mfrT==2) TempHead=ThermistorTemp(T2adc,ee.Rref[1]);	
	return TempHead;
}
//===============================================================
void Heater(void) {
	if( HeaterCheck() && ee.mfrheat == ON) {
		Serial.print(" H ");
		MFRHeater(ON); 
	} else { 
		Serial.print(" h ");
		MFRHeater(OFF);
	}
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
	IF HEATER IS OFF AND TEMP < LowMode THRESHOLD TURN ON
	ELSE LEAVE OFF
	******************/
	if( HeaterFlag == OFF ) {
		if( TempHead < ee.MfrSetTemp ) return 1;
		else return 0;
	}
	/*****************
	IF HEATER IS ON AND TEMP > HighMode THRESHOLD TURN OFF
	ELSE LEAVE ON
	******************/
	else {
		if( TempHead > ee.MfrSetTemp ) return 0;
		else return 1;
	}
}
//==========================================================================
long Hex2Dec(char *e)
// Convert a 3-char hex string to a decimal number
{
  unsigned int j;
  byte i;
  //3 bytes

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
		digitalWrite(HEATER, HighMode);
		digitalWrite(LED22, HighMode);
		HeaterFlag=ON;
	}
	if (k == OFF) {
		digitalWrite(HEATER, LowMode);
		digitalWrite(LED22, LowMode);
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
		digitalWrite(MOTOR, HighMode);
		digitalWrite(LED22, HighMode);
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
		digitalWrite(MOTOR, LowMode);
		digitalWrite(LED22, LowMode);
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
	// 8 bytes

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
// 	unsigned long t0; --> Udum
	// 0 bytes
	
	//Serial.print("NadirShutdown: MotorPwrFlag="); Serial.println(MotorPwrFlag);
	//wait for nadir==1
	Udum=millis();
	while(digitalRead(NADIR) == 0 ){
		if(millis()-Udum>10000){
			Serial.println("NadirShutdown timeout.");
			MotorOff();
			return;
		}
		delay(10);
	}
	while(digitalRead(NADIR) == 1 ){
		if(millis()-Udum>10000){
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
	unsigned int 	ichar;
// 	int				ic; -->ix
	// 2 bytes
	
	//Serial.println("PackBlock");	
	df_index = 0;
	strcpy(OutStr,PKTHEADER);  // defined "$FSR03,\0"		
	df_index+=7;
	
	Serial.print(" TempHead=");Serial.print(TempHead,2);Serial.print(", ");
	Serial.print(" FrsrState=");Serial.print(ee.FrsrState);Serial.print(", ");
	Serial.print("Rain ");Serial.print(RainState,DEC);Serial.print("/");Serial.print(RainSecs);Serial.print(" ");
	/*****************
	FrsrState OFF when motor is off and no sweep data are collected.
	******************/
	if(RunMode == HighMode) OutStr[df_index] = 'H';
	else if (RunMode == LowMode) OutStr[df_index] = 'L';
	else OutStr[df_index] = 'T';
	df_index++;
	OutStr[df_index]=',';df_index++;
	
	// MFR Temp
	floatToString(tempbuf2, (double)TempHead, 1, 4);
	for(iloop=0;iloop<4;iloop++){
		OutStr[df_index]=tempbuf[iloop]; df_index++;
	}
// 	OutStr[df_index+1]='\0';
// 	strcat(OutStr,tempbuf2); df_index+=4;
// 	Serial.print(" TempHead=");Serial.print(TempHead,2);Serial.print(", ");Serial.println(tempbuf);
// 	Serial.print(df_index);Serial.print(" ");
// 	Serial.print((char)OutStr[9]);Serial.print(" ");Serial.print((char)OutStr[10]);Serial.print(" ");
// 	Serial.print((char)OutStr[11]);Serial.print(" ");Serial.print((char)OutStr[12]);Serial.println("");
	// GPS
	strcat(OutStr,",<<"); df_index+=3;
	df_index=MoveBuf(bufstr,OutStr,0,df_index,60);
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
	for(ix=0; ix<NCHANS; ix++) {
		PsuedoAscii( G[ix][0], 2);
	}
	OutStr[df_index]=',';df_index++;
	// G2
	for(ix=0; ix<NCHANS; ix++) {
		PsuedoAscii( G[ix][1], 2);
	}
	OutStr[df_index]=',';df_index++;

	//shadowratio 2 bytes
	if(RunMode==LowMode) ichar=0;
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
	// SWEEPS BIN DATA IF SHADOW
	if(Shadow[ee.shadowchannel-1] >= ee.shadowlimit){
		for(iloop=0; iloop<NCHANS; iloop++){
			for(ix=0; ix< NBLKS*2+1; ix++) {
				PsuedoAscii(sweepBlk[ix][iloop], 2);
			}
			OutStr[df_index]=',';df_index++;
		}
	}
	OutStr[df_index] = '\0';	
	return;
}
//==================================================================
void PrintBytes(char *buf) {
//   int i, iloop;
  // 0 bytes

  ix = strlen(buf);
  Serial.print("PrintBytes: ");
  for (iloop = 0; iloop < ix; iloop++) {
	Serial.print(buf[iloop], HEX);
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
global df_index is the pointer to the OutStr[df_index] string.
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
	// 6 bytes
	
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
//=================================================================
byte ReadGps(void) {
	byte bytestatus;
	byte j,iTry;
	byte k;
	// 4 bytes

	iTry=0;
	while(iTry<1){
		//Serial.println("");Serial.print("iTry=");Serial.println(iTry);
		byteGPS=-1;
		// reset
		ctr1 = ctr2 = 0;
		// clear the buffer
		for(j=0;j<LENGPS;j++) bufstr[j]='\0';
		// clear serial uart
		while( Serial3.available() ) Serial3.read();
		// Wait for the '$'
		Udum = (unsigned long)millis();
		while(1){
			// Serial dead time
			if( millis() - Udum > 1200 ) {
				//Serial.println("");Serial.print("millis = ");Serial.println(millis());
				strcpy(bufstr,"$GPRMC,NaN*XX");
				//Serial.println(bufstr);
				return NOTOK;
			}
			// Check for '$'
			//Serial.print("-u-");
			if( Serial3.available() ) {
				k = Serial3.read();
				//Serial.print((char)k);Serial.print("-");
				if( k == '$'){
					//Serial.println("");
					ctr1=1; ctr2=0; bufstr[0]='$';
					break;
				}
			}
			else delay(10);
		}
		// loop reading all bytes from the gps
		j=0;
		Udum=millis();
		while(j<LENGPS){
			if(millis()-Udum > 200){
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
					bufstr[ctr1-2]='\0';
					return OK;
				}
			}
		}	
		iTry++;
	}
	strcpy(bufstr,"$GPRMC,NaN*XX");
	return NOTOK;	
}
//============================================================================
	void		ReadTilt(double *pitch, double *roll)
	{
	// 	unsigned long microstart; // define input wait time
	char e[4], chrin;
	byte i, count;
	double ddum;

	Serial2.setTimeout(100);

	// Clear buffer
	//while( Serial2.available() ) Serial2.read();
	// READ IN PITCH, TRY SEVERAL TIMES
	//	count=0;
	//  Serial.print("pitch ");
	//	while( count < 3 )	  {
	//	  count++;
	Serial2.write(66);
	//delay(100); //v6
	Serial2.readBytesUntil('\n', e, 4);
	e[3] = '\0';
	//Serial.print("[0]");Serial.print(e[0],DEC);Serial.print(" [1]");Serial.print(e[1],DEC);Serial.print(" [2] ");Serial.println(e[2],DEC);
	//  }
	ddum = (double) Hex2Dec(e);
	//Serial.print("e=");Serial.print(e);Serial.print(" ddum=");Serial.println(ddum,2);
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
	//while( Serial2.available() ) Serial2.read();
	//	count=0;
	//  Serial.print("roll ");
	//while( count < 3 )	  {
	//	count++;
	Serial2.write(67);
	//delay(20); //v6
	Serial2.readBytesUntil('\n', e, 4);
	e[3] = '\0';
	//Serial.print("[0]");Serial.print(e[0],DEC);Serial.print(" [1]");Serial.print(e[1],DEC);Serial.print(" [2] ");Serial.println(e[2],DEC);
	//  }
	ddum = (double) Hex2Dec(e);
	//Serial.print("  e=");Serial.print(e);Serial.print(" r=");Serial.println(ddum,2);
	if (ddum != MISSING && ddum >= 0 && ddum <= 4097) {
	ddum -= 2048;
	if (ddum < -1660) ddum = -1660;
	if (ddum > 1660) ddum = 1660;
	*roll = R1 * (C1 * ddum + C2 * ddum * ddum + C3 * ddum * ddum * ddum) + R0 + ee.roll_correct; //v29
	}
	else *roll = MISSING;
	return;
}
//==================================================================
unsigned	Ref(int K) {
  if (K == ON) {
	digitalWrite(REFSW, LowMode);
	digitalWrite(LED32, HighMode);
	return OK;
  }
  else if (K == OFF) {
	digitalWrite(REFSW, HighMode);
	digitalWrite(LED32, LowMode);
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
// 	char			bufstr[10];
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
// 		sprintf(bufstr," %d",sweep_msec); SerPutStr(bufstr);
// 		if( sweep_msec > l2 || sweep_msec < l1 )
// 		{
// 			sprintf(bufstr,"x "); SerPutStr(bufstr);
// 			sweep_msec = msec_default;
// 		}
// 		else
// 		{
// 			sprintf(bufstr," "); SerPutStr(bufstr);
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

		// LowMode MODE
		case LowMode:
			Serial.print("-Low-");
			break;

		// TRANSITION MODE
		case TRANSITION:
			Serial.print("-Trans-");
			break;

		// HighMode MODE
		case HighMode:
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
	// 4 bytes

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
// 	double ddum,fdum;
	// 0 bytes
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
	//unsigned int ix,iy;
	// 0 bytes
	
	// LEFT SIDE
	*g1 = 0;
	for(ix=0; ix<nsum; ix++){
		iy = chan + ix*NCHANS;	//[0,7,14...1743] [1,8,15...1744]...[6,13,...1749] 
		*g1 += (double)adcmv[iy];
	}
	*g1 /= (double) nsum;
	// RIGHT SIDE
	*g2 = 0;
	for(ix=NSWEEP-nsum; ix<NSWEEP; ix++){
		iy = chan + ix*NCHANS;	//[0,7,14...1743] [1,8,15...1744]...[6,13,...1749] 
		*g2 += (double) adcmv[iy];
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
	//unsigned int ix,iy;
	// 4 bytes
	
	for(ix=0;ix<NSWEEP;ix++){		//0,1,2,...249
		iy=chan + ix*NCHANS;	//[0,7,14...1743] [1,8,15...1744]...[6,13,...1749] 
		if(adcmv[iy]<adcmin){
			imin=ix;
			adcmin=adcmv[iy];
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
	unsigned int isum=0;
	byte jx;
// 	double x,xs,x1,xs1;
	// 2 bytes

	// LEFT SIDE
	x = 0; xs=0;isum=0;
	for(jx=0; jx<nsum; jx++){
		idum = chan + jx*NCHANS;	//[0,7,14...1743] [1,8,15...1744]...[6,13,...1749] 
		x += (double)adcmv[idum];
		xs += (double)adcmv[idum]*(double)adcmv[idum];
		isum++;
	}
	MeanStdev(&x, &xs, isum, 0); 


	// RIGHT SIDE
	x1=0; xs1=0;isum=0;
	for(jx=NSWEEP-nsum; jx<NSWEEP; jx++){
		idum = chan + jx*NCHANS;	//[0,7,14...1743] [1,8,15...1744]...[6,13,...1749] 
			x1 += (double)adcmv[idum];
			xs1 += (double)adcmv[idum]*(double)adcmv[idum];
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
// 	int i1,i2; --> ix, iy
	int i1;
	byte npts;
// 	double ddum;
	// 4 bytes
	
	// channels
	for(ic=0; ic<NCHANS; ic++){		// [0,1,...249]
		// blocks
		for(ib = 0; ib < 2 * NBLKS + 1; ib++) sweepBlk[ib][ic] = 0;
		// RIGHT SIDE
		iy = Imin[ic];
		for(ibk=0; ibk<NBLKS; ibk++){  // 0,1,...,10
			// INDEX IN THE BLOCK ARRAY
			ib = NBLKS + ibk + 1; // 12,...22
			// SUMMATION LIMITS
			ix = iy+1; 							// just to right of the shadow 
			iy = ix + Nsz[ibk] - 1; 			// Nsz points in mean
			if( iy >= NSWEEP) iy = NSWEEP-1;  	// do not overrun the array
			ddum = 0;  npts=0;					// initialize
			for(i1=ix; i1<=iy; i1++){
				ddum += (double)adcmv[i1*NCHANS+ic];
				npts++;
			}
			sweepBlk[ib][ic] = (int) round(ddum / (double)npts);
			if( iy == NSWEEP-1 ) break;
		}
		//Serial.print(" ");Serial.print(ic);Serial.print("x");
		// LEFT SIDE BLOCK AVGS
		ix = Imin[ic];
		for(ibk=0; ibk<NBLKS; ibk++){  // 0,1,...,10
			// INDEX IN THE BLOCK ARRAY
			ib = NBLKS - ibk - 1; // 10,9,...,0
			// SUMMATION LIMITS -- from shadow to left
			iy = ix - 1;
			ix = iy - Nsz[ibk] + 1;
			//Serial.print(ix,DEC);Serial.print(" ");Serial.println(iy,DEC);
			if( ix < 0 ) ix = 0;
			ddum = 0;  npts=0;
			for(i1=ix; i1<=iy; i1++){
				ddum += (double)adcmv[i1*NCHANS+ic];
				npts++;
			}
			sweepBlk[ib][ic] = (int) round(ddum / (double) npts);
			if( ix ==  0 ) break;
		}
		//Serial.print(" ");Serial.print(ic);Serial.print("y");
		// MINIMUM
		/**********************************
		MINIMUM VALUES AT THE MIN INDEX
		***********************************/
		sweepBlk[NBLKS][ic] = Smin[ic];  // single min at block 11
		//Serial.print("z");
	}
	return;
}
//==========================================================================
double ThermistorTemp(byte nadc, double Rref){
/* nadc = 0 or 1
Rref usually = 10000
*/
	//double ddum, fdum;
	//double r;
	//double t, t2;
	//int16_t i16;
	//double beta[]={ 1.025579e-03,   2.397338e-04,   1.542038e-07};
	// 8 bytes
	
	fdum=GetAdc16Volts(nadc);	
	//check bad input
	if(fdum<AdcThreshold[0]) return 0;
	if(fdum>AdcThreshold[1]) return 50;
	ddum = 2.045*GetAdc16Volts(3); // Vref adc16 chan 3
	//Serial.print("vref="); Serial.println(ddum,1);
	
	ddum = Rref * (fdum / (ddum-fdum));
	//Serial.print("test rtherm="); Serial.println(r,2);
	if(ddum<1000 || ddum>30000){
		//Serial.print("Error. rtherm="); Serial.println(ddum,2);
		return MISSING;
	}
	ddum = log(ddum);
//	Serial.print("log(r)="); Serial.println(ddum,3);
// 	Serial.print("beta0="); Serial.print(beta[0],8);
// 	Serial.print(";	beta1="); Serial.print(beta[1],8);
// 	Serial.print(";	beta2="); Serial.println(beta[2],10);
	//fitted curve
// 	fdum = beta[0] + beta[1] * ddum + beta[2] * ddum * ddum * ddum;
	fdum = (double)BETA0 + BETA1 * ddum + BETA2 * ddum * ddum * ddum;
	ddum = 1 / fdum - (double)273.15;
	// from FRSR v3 code (PRP2)
// #define A1 -3.4868183734e-7
// #define A2 -8.9323216739e-6
// #define A3 -3.308255931317e-4
// #define A4 7.92320546465e-4
// 	d = log(1/rt);
// 	d = A1 * d * d * d + A2 * d * d + A3 * d + A4;
// 	T1 = 1.0 / d - 273.15;

// 	Serial.print("T="); Serial.println(t,3);
	if(ddum<TempLim[0] || ddum>TempLim[1]) ddum=MISSING;
	return ddum;
}
//=============================================
void PrintBuf( char *buff, int istart, int numchars){
// 	int j;
	// 2 bytes
	
	ix=istart;
	while(1){
		Serial.print((char)buff[ix]);
		ix++; if(ix-istart >= numchars) return;
		if(buff[ix]=='\0') return;
	}
}
//=============================================
int MoveBuf( char *buf1, char *buf2, int i1, int i2, int numchars)
{
// 	int j1,j2; --> ix,iy
	// 4 bytes
	
	ix=i1; iy=i2;
	while(1){
		buf2[iy]=buf1[ix];
		ix++; iy++;
		if(ix-i1 >= numchars || buf1[ix] == '\0') {
			buf2[iy]='\0';
			return iy; // index of the NULL
		}
	}
}
//============================================================
int handle_byte(int byteGPS) {
	byte j;
	// 1 bytes
	
	bufstr[ctr1] = byteGPS;
	ctr1++; bufstr[ctr1]='\0';
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
			//Serial.print("bufstr ");Serial.println(bufstr);
			return NOTOK;
		}
		// header GPRMC
		for( j=0; j<6; j++ ) {
			//Serial.print(bufstr[j]);
			if( bufstr[j] != gpsid[j] ) {
				//Serial.println("-H-");
				return NOTOK;
			}
		}
		//checksum
		if( nmea_validateChecksum(bufstr) != OK ) {
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
	char hx[3]; // = "0x00"
	char hc[3];
	// 12 bytes
	
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

