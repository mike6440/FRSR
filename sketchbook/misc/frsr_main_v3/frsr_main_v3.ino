//161213--We modify the rosr arduino program for the frsr circuit board.

#define PROGRAMNAME "frsr_main"
#define VERSION     "4"//"3"
#define EDITDATE    "170106"//"161222"
const byte  EEPROM_ID = 3;
//v2 - add adc stuff in action
//v3 - Nsweep in eeprom
/*to do 
 -- adc speed test. 250 samples
 -- GetSweep() 
*/
//NOTE ====== INCLUDES
#include <string.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

//Added for the ADS
#include <Wire.h>
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads0;  // ad0, u13, construct an ads1115 at address 0x48
Adafruit_ADS1115 ads1(0x49);    // ad1, u16, construct an ads1115 at address 0x49
Adafruit_ADS1115 ads2(0x4A);    // ad2, u14, construct an ads1115 at address 0x4A

//NOTE ====== DIGITAL LINES
const int RAIN = 4;         // RAIN DETECTOR -- Drives the LED
const int NADIR = 5;         // INPUT Nadir Switch 1
const int MOTOR = 9;          // shadowband motor on/off
const int HEATER = 10;         // mfr heater on/off
const int LED11 = 24;       // Green Scan Drum motor
const int LED12 = 26;       // Red shutter motor
const int LED21 = 28;       // green BB2 heater on
const int LED22 = 30;       // red BB1 heat on
const int LED31 = 32;       // green -- continuous on = power
const int LED32 = 13;       // red -- heartbeat. red 5REF
const int REFSW = 22;       // BB thermistor reference voltage

// ANALOG ASSIGNMENTS
// three 4-chan 16 bit adc, adc#=(unsigned)(x/4), chan#=x%4;
const char ADCMAX = 12;      // =12 normally. maximum number of ADC channels
const int  NSWEEP = 250;
const int T1adc = 0;      	// MFR Temp 1
const int T2adc = 1;      	// MFR T2
const int mfr1 = 2;   	// MFR CHAN 1 (filter 1)
const int NRF = 3;      // 5REF/2
const int mfr2 = 4;     // MFR CHAN 2 (filter 2)
const int mfr3 = 5;     // MFR CHAN 3 (filter 3)
const int mfr4 = 6;     // MFR CHAN 4 (filter 4)
const int mfr5 = 7;     // MFR CHAN 5 (filter 5)
const int mfr6 = 8;     // MFR CHAN 6 (filter 6)
const int NVin = 9;     // VIN/4
const int Arain = 10;   // Rain / 4
const int mfr7 = 11;    // MFR CHAN 7 (filter 7)

//NOTE ====== CONSTANTS
const char OK = 1;
const char NOTOK = 0;
const int MISSING = -999;
const int POS = 1;
const int NEG = -1;
const int CCW = 1;          // forward direction
const int CW = -1;          // reverse direction
const int STOP = 0;         // motor stop
const int ON = 1;
const int OFF = 0;
const int OPEN = 1;
const int CLOSED = 0;
const char SPACE = ' ';
const char ZERO = '0';
// test menu
const char TEST = -1;
const char RUN = 1;
// TEMPERATURE
const int WARMUPMILLISECS = 50;
const int ADCREADWAIT = 100;
const double TMAX = 70;
const double TMIN = -5;
// SPECTRON TILT
const double C1 = 0.0129;
const double C2 = 0;
const double C3 = -0.0000000003;
const double P1 = 1; // use -1 for a reversed direction
const double P0 = 0;
const double R1 = 1; // use -1 for a reversed direction
const double R0 = 0;


//=========================================================================
//NOTE ====== GLOBAL VARIABLES
//=========================================================================
// system time
unsigned long msstart;  //set at start, milliseconds
unsigned long menustart;

// user i/o
char    RunMode;

// READING THE ADC
double vsweep[ADCMAX][NSWEEP];		// all channels for a full sweep
double vmean[ADCMAX];                       // adc16 read
unsigned int nsamps[ADCMAX];
double vstd[ADCMAX];
double Vref;                                //compute from adc16;

// ADC16 limits for BB temperature cal. volts
double threshold[2] = {
  1.00, 4.00
};

// TILT
double pitav = 0;
double pitsd = 0;
double rolav = 0;
double rolsd = 0;
int npit = 0;
int nrol = 0;

// OTHER TEMPS
double  T1 = 0; // MFR temp T1
double  T2 = 0; // MFR temp T2

// RAIN
unsigned long  millisec_rain;   // the clock count to open.  0 if open
char    RainState;
unsigned long secs_lapsed;


//NOTE ====== EEPROM
// Scan angles: abb1, abb2, asky, asea, a1, a2
const float default_rain_threshold = .090;
const unsigned long default_rainsecs = 600;
const unsigned long default_Nsweep = 250;
const float default_pitch_correct = 0; //v29
const float default_roll_correct = 0;  //v29
const char default_mfrheat = OFF;
const double    default_Rref[2] = {
  10000, 10000
}; // T1, T2


struct eeprom {
  byte id;
  unsigned long Nsweep;  // total samps over the hemisphere.
  float rain_threshold;
  float pitch_correct, roll_correct; 
  unsigned long rainsecs;
  char mfrheat; // on or off
  double Rref[2];
};
struct eeprom ee;
int eeSize;

int istart; // set to 1 at startup so we can go to a menu.

//NOTE ====== FUNCTIONS
// i/o
void        Action(char*);
void        MFRHeater(int);
void        CheckRain(double *, unsigned long *);
unsigned int checksum_nmea(char *);
unsigned long ElapsedTime (char *);
void        EepromDefault(void);
void        EepromStore(void);
void        EepromRead(void);
void        EepromPrint(void);
char *      floatToString(char *, double, byte, byte);
double      GetAdcVolts (unsigned int );   // ads adc
unsigned int GetAdcSample(unsigned int ch, double *vmean);   // ads adc
unsigned int GetThermCoefs(unsigned int, double *coef );   // ads adc
long        Hex2Dec(char *);
void        MeanStdev(double *, double *, int, double );
void        PrintProgramID(void);
void        PrintBytes(char *);
void        ReadTilt(double*, double*);
unsigned    Ref(int);
int         sign (float);
double      SteinhartHart(double beta[], double r);
double      ThermistorTemp(double v, double Vref, double Rref, unsigned int ntherm);

//============================================================================
void setup() {
  // USER Serial setup
  Serial.begin(38400);
//   ENCODER serial setup
//   Serial1.begin(9600);    // serial
  // TILT
  Serial2.begin(19200);   // serial
//   KT15
//   Serial3.begin(9600);    // serial kt15

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
  pinMode(NADIR, INPUT);           // shadowband nadir switch

  // set mem for eeprom
  eeSize = sizeof(struct eeprom);
  //  Serial.print("Check EEPROM.  ");
  EepromRead();
  Serial.print("test EEPROM_ID = "); Serial.println(EEPROM_ID);
  Serial.print("test ee.id = "); Serial.println(ee.id);
  if (ee.id != EEPROM_ID ) {
    Serial.println("ID NO  match => use default.");
    EepromDefault();
  } else Serial.println("test ID MATCH.");


  // ADS1115 ADC
	Serial.println("Getting single-ended readings from AIN0..3");
	Serial.println("ADC Range: +/- 6.144V (1 bit = .15mV)");
  ads0.begin();
  ads0.setGain(GAIN_ONE);     // 1x gain   +/- 4.096V
  //  ads0.setGain(GAIN_ONE);     // 1x gain   +/- 4.096V  1 bit = 2mV
  //  ads0.setGain(GAIN_TWO);     // 2x gain   +/- 2.048V  1 bit = 1mV
  //  ads0.setGain(GAIN_FOUR);    // 4x gain   +/- 1.024V  1 bit = 0.5mV
  //  ads0.setGain(GAIN_EIGHT);   // 8x gain   +/- 0.512V  1 bit = 0.25mV
  //  ads0.setGain(GAIN_SIXTEEN); // 16x gain  +/- 0.256V
  ads1.begin();
  ads1.setGain(GAIN_ONE);     // 1x gain   +/- 4.096V
  ads2.begin();
  ads2.setGain(GAIN_ONE);     // 1x gain   +/- 4.096V


  istart = 1;
}

//=============================================================================
void loop() {
  int     i = 0;
  unsigned int    b = 0;
  char    buf[20];
  char    OutStr[1024], AvStr[512], SsstStr[512];
  double ddum, fdum;
  unsigned long ldum;


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

  // OUTPUT STRING
  // header
  strcpy(OutStr, "$RSR01,\0");

  // Elapsed time, days, since startup
  ElapsedTime(buf);
  strcat(OutStr, buf); // end of AvStr strcat(AvStr,",");
  strcat(OutStr, ",\0");

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
  CheckRain(&ddum, &ldum);
  if (ddum == MISSING) {
    strcat(OutStr, "-999,-999");
  }
  else {
    floatToString(buf, ddum, 1, 4);
    strcat(OutStr, buf);
    strcat(OutStr, ",");
    floatToString(buf, (double)ldum, 0, 3);
    strcat(OutStr, buf); // End of OutStr.  strcat(OutStr,",");
  }
  // CHECKSUM AND END
  b = checksum_nmea(OutStr + 1);
  Serial.print(OutStr);
  Serial.print("*");
  Serial.print(b, HEX);
  Serial.print("\r\n");

}
// =============== END OF LOOP =======================================


//*****************************************************************
void    Action(char *cmd)
{
  //  Read message and take action.
  // Create an output packet in out_buffer and send;
  // input:
  // in_buffer = pointer to a message string
  // in_fromlist = char string, e.g. "121115"
  //  meaning the message came from 12 in the route 15->11->12
  char str[50], str1[10];
  char ix, yesno[4], eok = 0;
  double fdum, ddum, av1, av2, fsum1, fsq1, fsum2, fsq2;
  char *stop_at;
  byte i, ib;
  unsigned long Ldum;
  unsigned long sweeptime;
  unsigned int isamp;
  int n, n1;

  Serial.print("test Action cmd = ");  Serial.println(cmd);


  // TAKE ACTION AND PREPARE AN OUTPUT MESSAGE IN out_message
  if (cmd[0] == '?') {
    PrintProgramID();
    Serial.println("------- EEPROM -----------------------------------");
    Serial.println("E       -- show eeprom ");
    Serial.println("EDfff.f -- EEn     -- MODE: 0=Run, 1=Cal");
    Serial.println("ERfff.f -- Rain threshold volts     Ernn    -- Shutter open delay, nn secs");
    Serial.println("Ejff.f  -- Pitch correct deg        EJff.f  -- Roll correct deg");
    Serial.println("ENiii  -- Nsweep sweep");
    Serial.println("");
    Serial.println("------- FUNCTIONS -----------------------------------");
    Serial.println("an -- ADC Chan n                  A  -- ADC all hemisphere");
    Serial.println("B  -- shadowband motor ON         b  -- shadowband OFF");
    Serial.println("H  -- MFR heater ON               h  -- shadowband OFF");
    Serial.println("C  -- 5REF ON                     c  -- 5REF OFF");
    Serial.println("n  -- Nadir switch                N  -- nadir loop");
    Serial.println("p  -- pitch/roll single           P  -- pitch/roll loop");
    Serial.println("r  -- Rain check single           R  -- Rain check loop");
    Serial.println("t  -- Read system clock           T  -- Set default f.p. day");
    Serial.println("sn -- LED test:  n=1 Heartbeat   2 Drum   3 Shutter/Rain");
    Serial.println("v/V -- Program version");
    Serial.println("g/G -- Continue sampling.");
  }
	
  // READ ADC CHANNEL
  else if (cmd[0] == 'a' && strlen(cmd) > 1) {
  	if(strlen(cmd)==2){ix = cmd[1] - 48;}
  	else{ ix=10*(cmd[1]-48)+(cmd[2]-48); }
		if (ix < 0 || ix > ADCMAX-1) {
			Serial.println("Error");
		} else {
	    Serial.print("Chan ");
  	  Serial.println(ix, DEC);
			while ( !Serial.available()  ) {
				GetAdcSample(ix, (vmean + ix));
				Serial.println(vmean[ix], 4);
			}
		}
  }
  
  // ADC LOOP
  //Read all ADCMAX channels 250 times as fast as possible.
  //Output means and stdevs. Get the time for the full process.
  else if (cmd[0] == 'A') {
    //clear register
		for (ix = 0; ix < ADCMAX; ix++) {
			vmean[ix]=0;   vstd[ix]=0;   nsamps[ix]=0;
		}
		// Mark start time
		sweeptime=millis();
    //Loop 
    for( isamp=0; isamp<ee.Nsweep; isamp++){ 
    	// Read all adc chans
			for (ix = 0; ix < ADCMAX; ix++) {
				ddum=GetAdcVolts(ix);
				vmean[ix] += ddum;
				vstd[ix] += ddum*ddum;
				nsamps[ix] ++;
			}
    }
		// elapsed time
		sweeptime=millis() - sweeptime;
		Serial.print("Sample time: "); Serial.print(sweeptime); Serial.println(" msecs");
    // Compute means and stdevs
    //void    MeanStdev(double *sum, double *sum2, int N, double missing)
		for (ix = 0; ix < ADCMAX; ix++) {
			MeanStdev((vmean+ix),(vstd+ix),nsamps[ix],MISSING);
		}
		//Print results
  	//header
    Serial.print("Read ");Serial.print(ee.Nsweep,DEC); Serial.println(" scans.");
    Serial.println("    v0    v1    v2    v3    v4    v5    v6    v7    v8    v9    v10   v11");
		//vmean
		for (ix = 0; ix < ADCMAX; ix++) {
			Serial.print(vmean[ix],3);
		}
		Serial.println("");
		//stdev
		for (ix = 0; ix < ADCMAX; ix++) {
			Serial.print(vstd[ix],3);
		}
		Serial.println("");		
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
    ix = cmd[1] - 48;
    Serial.print("SHADOWBAND MOTOR ON");
    Shadowband(ON);
  }
  // MFR HEATER OFF
  else if (cmd[0] == 'b') {
  	Serial.println("SHADOWBAND MOTOR OFF");
    Shadowband(OFF);
  }
  
  // 5REF   ON/OFF
  else if (cmd[0] == 'C' ) {
    Serial.println("5REF ON");
    Ref(ON);
  }
  else if (cmd[0] == 'c' ) {
    Serial.println("5REF OFF");
    Ref(OFF);
  }
  
  // NADIR SWITCH
  else if (cmd[0] == 'n' || cmd[0] == 'N' ) {
    Serial.println("0=CLOSED   1=OPEN");
    while ( !Serial.available()  ) {
      Serial.println(digitalRead(NADIR));
      delay(1000);
    }
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
    Serial.print("    roll = ");
    Serial.println(fdum, 2);
  }
  else if (cmd[0] == 'P') {
    Serial.println(" N  PITCH  ROLL");
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
    Serial.print("   Sec to open = ");
    Serial.print(Ldum);
    Serial.print("   State = ");
    Serial.println(RainState, DEC);
  }
  else if (cmd[0] == 'R') {
    Serial.println(" N  STATUS   VOLTS   SECS");
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
//      v = analog (volts)
//      rainsecs = seconds until shutter opens
// RETURN 1 or 0 for rain mode and shutter control.
// Global in
//  ee.rain_threshold;
//  ee.rainsecs;
// Global out
//   unsigned long  millisec_rain;  // clock count for opening
//   RainState
{
  int a;

  GetAdcSample(Arain, v);
  *v *= 4;
  //Serial.print("Chan ");  Serial.print(Arain,DEC); Serial.print("  ");  Serial.println(*v,4);

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
  ee.rain_threshold = default_rain_threshold;
  ee.rainsecs = default_rainsecs;
  for (i = 0; i < 2; i++) {
    ee.Rref[i] = default_Rref[i];
  } 
  ee.mfrheat = default_mfrheat;
  ee.pitch_correct = default_pitch_correct;
  ee.roll_correct = default_roll_correct;
  ee.Nsweep = default_Nsweep;
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
    EEPROM.write(i, *a );   // store this byte
    a++;
  }
  return;
}

//=============================================================================
void EepromRead()
{
  Serial.println("ReadUee:");
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
//   byte id; xxxx
//   float rain_threshold;
//   float pitch_correct, roll_correct; 
//   unsigned long rainsecs;
//   char mfrheat; // on or off
//   double Rref[2];
// 
  int i;
  Serial.println("EepromPrint: ");
  Serial.print("  ID = ");
  Serial.print(ee.id);
  Serial.println(" ");
  Serial.print("  R Rain threshold  = ");
  Serial.print(ee.rain_threshold, 2);
  Serial.println(" volts");
  Serial.print("  r Rain shutter delay  = ");
  Serial.print(ee.rainsecs);
  Serial.println(" secs");
  Serial.print("  j pitch correct = "); //v29
  Serial.print(ee.pitch_correct, 1);  //v29
  Serial.println(""); //v29
  Serial.print("  J roll correct correct = "); //v29
  Serial.print(ee.roll_correct, 1);  //v29
  Serial.println(""); //v29
  Serial.print("  MFR heater configuration = ");
  Serial.print(ee.mfrheat, DEC);
  Serial.println("");
  Serial.print(" Sweep samples = ");
  Serial.print(ee.Nsweep, DEC);
  Serial.println("");

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
  Ld = ms / 86400000;         // number of days
  if (Ld >= 0 && Ld < 100) {
    ch[2] = '\0'; ch[1] = Ld % 10 + 48; ch[0] = Ld / 10 + 48;
  } else {
    strcpy(ch, "xx");
  }
  strcat(ddhhmmss, ch);
  strcat(ddhhmmss, ".");
  //     Serial.print("days = ");  Serial.print(Ld);
  //     Serial.print("   string:");  Serial.print(ch);
  //     Serial.print("   ddhhmmss:");  Serial.println(ddhhmmss);

  //hours
  Lh = (ms - Ld * 86400000) / 3600000;    // number hours
  if (Lh >= 0 && Lh < 100) {
    ch[2] = '\0'; ch[1] = Lh % 10 + 48; ch[0] = Lh / 10 + 48;
  } else {
    strcpy(ch, "xx");
  }
  strcat(ddhhmmss, ch);
  //     Serial.print("hours = ");  Serial.print(Lh);
  //     Serial.print("  string = ");  Serial.print(ch);
  //     Serial.print("  ddhhmmss:");  Serial.println(ddhhmmss);

  //min
  Lm = (ms - Ld * 86400000 - Lh * 3600000) / 60000;
  if (Lm >= 0 && Lm < 100) {
    ch[2] = '\0'; ch[1] = Lm % 10 + 48; ch[0] = Lm / 10 + 48;
  } else {
    strcpy(ch, "xx");
  }
  strcat(ddhhmmss, ch);
  //     Serial.print("mins = ");  Serial.print(Lm);
  //     Serial.print("  string = ");  Serial.print(ch);
  //     Serial.print("  ddhhmmss:");  Serial.println(ddhhmmss);

  //sec
  Ls = (ms - Ld * 86400000 - Lh * 3600000 - Lm * 60000) / 1000;
  if (Ls >= 0 && Ls < 100) {
    ch[2] = '\0'; ch[1] = Ls % 10 + 48; ch[0] = Ls / 10 + 48;
  } else {
    strcpy(ch, "xx");
  }
  strcat(ddhhmmss, ch);
  //     Serial.print("secs = ");  Serial.print(Ls);
  //     Serial.print("  string = ");  Serial.print(ch);
  //     Serial.print("  ddhhmmss:");  Serial.println(ddhhmmss);

  return ms;
}
//*******************************************************************
char * floatToString(char * outstr, double val, byte precision, byte widthp)
/********
 * Convert a double prec variable to a string of characters.
 * Example   floatToString(s, 123.456, 2, 8) returns
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
  { //           *
    roundingFactor /= 10.0;     // .5, .05, .005, ...
    mult *= 10;                 // 1,  10,  100, ...
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
double GetAdcVolts (unsigned int chan) {
  //GetAdcVolts returns a single read of 16 bit adc channel ch.
  double v;
  int16_t adc;
  if (chan >= 0 && chan <= 3)
    adc = ads0.readADC_SingleEnded(chan);
  else if (chan >= 4 && chan <= 7)
    adc = ads1.readADC_SingleEnded(chan - 4);
  else if (chan >= 8 && chan <= 11)
    adc = ads2.readADC_SingleEnded(chan - 8);
  else adc = 0;
  v = (double)adc / 8000;
  return v;
}


//======================================================================================
unsigned int GetAdcSample(unsigned int ch, double *vmean) {
  //      okflag = GetAdcSample(ch, *vmean){
  // GetAdcSample takes nsap (=10) readings of the ADC channel ch. Tosses out the min and max
  // voltages and takes the mean of the remainder.

  double v, vi[10], vsum, vmn, vmx;
  unsigned int i, imx, imn, nav;
  nav = 10;
  vsum = 0;
  vmx = -1e15;
  vmn = 1e15;
  imx = imn = 0;
  for (i = 0; i < nav; i++) {
    v = vi[i] = GetAdcVolts(ch);
    vsum += vi[i];
    if (v < vmn) {
      imn = i;
      vmn = v;
    }
    if (v > vmx) {
      imx = i;
      vmx = v;
    }
  }
  *vmean = (vsum - vi[imx] - vi[imn]) / (double)(nav - 2);
  return 1;
}
//============================================================
unsigned int GetThermCoefs(unsigned int nt, double c[] ) {
  //   revision, add 9,10,11,12
  // Give the correct SHH coefs for the thermnumber
  // See 04.20.18.10../Tcal3_1605_thermistor_cal_rosr3/Tcal1605_report.key.pdf
  // See BB_Therm_Master_List spreadsheet we need to revise this for each rosr

  double tcal[13][3] = {  //v28  !! 13 rows, add to this for new calibrations.
    //0 standard ysi
    { 1.025579e-03,   2.397338e-04,   1.542038e-07 },
    //1 Therm #1, Rosr1 T11
    { 1.0108115e-03, 2.4212099e-04,   1.4525424e-07 },
    //2 Therm #2, ROSR1, T12
    { 1.0138029e-03, 2.4156995e-04,   1.4628056e-07 },
    //3 Therm #3, ROSR1, T21
    { 1.0101740e-03, 2.4208389e-04,   1.4485814e-07 },
    //4 Therm #4, ROSR1, T22
    { 1.0137647e-03, 2.4161708e-04,   1.4619775e-07 },
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
void    MeanStdev(double *sum, double *sum2, int N, double missing)
//  Compute mean and standard deviation from
//  the count, the sum and the sum of squares.
{
  if ( N <= 2 ) {
    *sum = missing;
    *sum2 = missing;
  }
  else {
    *sum /= (double)N;      // mean value
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
void        ReadTilt(double *pitch, double *roll)
{
  unsigned long microstart; // define input wait time
  char e[4], chrin;
  byte i, count;
  double ddum;

  Serial2.setTimeout(100);

  // Clear buffer
  //  while( Serial2.available() ) Serial2.read();

  // READ IN PITCH, TRY SEVERAL TIMES
  //    count=0;
  //  Serial.print("pitch ");
  //    while( count < 3 )    {
  //      count++;
  Serial2.write(66);
  delay(10); //v6
  Serial2.readBytesUntil('\n', e, 4);
  e[3] = '\0';
  //      Serial.print(e[0],DEC);Serial.print("  ");
  //      Serial.print(e[1],DEC);Serial.print("  ");
  //      Serial.println(e[2],DEC);
  //  }
  ddum = (double) Hex2Dec(e);
  if (ddum != MISSING && ddum >= 0 && ddum <= 4097) {
    ddum -= 2048;
    if (ddum < -1660) ddum = -1660;
    if (ddum > 1660) ddum = 1660;
    *pitch = P1 * (C1 * ddum + C2 * ddum * ddum + C3 * ddum * ddum * ddum) + P0 + ee.pitch_correct;  //v29
  }
  else *pitch = MISSING;
  //  Serial.print("Pitch = "); Serial.println(*pitch,1);


  // READ IN ROLL, TRY SEVERAL TIMES
  // Clear buffer
  //  while( Serial2.available() ) Serial2.read();
  //    count=0;
  //  Serial.print("roll ");
  //    while( count < 3 )    {
  //      count++;
  Serial2.write(67);
  delay(10); //v6
  Serial2.readBytesUntil('\n', e, 4);
  e[3] = '\0';
  //      Serial.print(e[0],DEC);Serial.print("  ");
  //      Serial.print(e[1],DEC);Serial.print("  ");
  //      Serial.println(e[2],DEC);
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
unsigned    Ref(int K) {
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
	}
	if (k == OFF) {
		digitalWrite(MOTOR, LOW);
		digitalWrite(LED22, LOW);
		//Serial.println("Shadowband motor OFF.");
	}
  return;
}

//============================================================================
int sign (float input) {
  return ((input < 0.0) ? NEG : POS);
}
//===========================================================================
double SteinhartHart(double beta[], double r)
//input
// beta is the 3x1 calibration vector where temp is computed by
//     xt = beta(1) + beta(2)*xr + beta(3)*xr^3
//   where
//      xr = log(1/R)
//      T = 1/xt - 273.15
// r = measured resistance, ohms
//
//output
// t = calibrated temperature vector, degC
{
  double vx, t2, t;

  vx = log(r);
  //Serial.print("r="); Serial.print(r,3);
  //Serial.print("  vx="); Serial.print(vx,3);
  //Serial.print("  beta="); Serial.print(beta
  //fitted curve
  t2 = beta[0] + beta[1] * vx + beta[2] * vx * vx * vx;
  t = 1 / t2 - (double)273.15;
  return t;
}
//==========================================================================
double ThermistorTemp(double v, double Vref, double Rref, unsigned int ntherm) {

  double r;
  double a[3], t;

  if (v < threshold[0]) return 0;
  if (v > threshold[1]) return 200;

  GetThermCoefs(0, a);
  r = Rref * (v / (Vref - v));
  t = SteinhartHart(a, r);
  return t;
}
//===============================================================================
int GetSweep(unsigned
/*
Sample all 12 channels at a rate determined by 
*/
{
	char ix;
	unsigned int isamp;
	
	//clear register
	for (ix = 0; ix < ADCMAX; ix++) {
		vmean[ix]=0;   vstd[ix]=0;   nsamps[ix]=0;
	}
	
	//Loop 
	for( isamp=0; isamp<Nsweep; isamp++){ //ee.Nsweep  testing
		// Read all adc chans
		for (ix = 0; ix < ADCMAX; ix++) {
			vadc[ix]=GetAdcVolts(ix);
		}
	}
}