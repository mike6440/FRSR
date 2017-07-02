// #include <stdio.h>
// #include <stdlib.h>
#include <string.h>


#define		DONE	3
#define 	OK	 	1
#define		NOTOK 	0
#define		LENGPS	100
#define		RMCCOMMAS 11

// GLOBAL VARIABLES
unsigned long		udum;
char	OutStr[300];
unsigned int df_index;
char 	bufgps[LENGPS];		// GPS input string
byte	offsets[13];	// indexes of commas and the *
int 	byteGPS;		// GPS processing
int		ctr1, ctr2;		// GPS processing
char 	gpsid[7] = "$GPRMC";


//NOTE FUNCTIONS
byte nmea_validateChecksum(char *strPtr);
byte ReadGps(void);
void PrintBuf( char *buff, int istart, int numchars);

// SETUP
void setup() {
	Serial.begin(115200);
	Serial3.begin(9600);
	Serial.println("START GPSDEV edit 20170330T231646Z");
	Serial3.setTimeout(200);
}

// LOOP
void loop() 
{
	byte j;
	unsigned long t1,t2;
	df_index=0;
	
	t1=millis();
	ReadGps();
	t2=millis()-t1;
	Serial.print(t2);Serial.print("   ");
	
	// load OutStr[]
	for(j=7;j<strlen(bufgps)-3;j++){
		OutStr[df_index]=bufgps[j]; df_index++;
	}
	OutStr[df_index]='\0';
	Serial.print("<<");Serial.print(OutStr);Serial.println(">>");

	delay(4000);	
// 	while( ! Serial.available()	);
// 	Serial.read();
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
			if( millis() - udum > 1500 ) {
				//Serial.println("");Serial.print("millis = ");Serial.println(millis());
				strcpy(bufgps,"$GPRMC,NaN*XX");
				Serial.println(bufgps);
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
	
	for(j=istart;j<istart+numchars-1;j++) 
		Serial.print((char)buff[j]);
	Serial.println("");
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
		if( ctr2 != RMCCOMMAS + 1 ) {
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
/* GPS lat,lon,sog,cog
$GPRMC

Recommended minimum specific GPS/Transit data

eg1. $GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62
eg2. $GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68


           225446       Time of fix 22:54:46 UTC
           A            Navigation receiver warning A = OK, V = warning
           4916.45,N    Latitude 49 deg. 16.45 min North
           12311.12,W   Longitude 123 deg. 11.12 min West
           000.5        Speed over ground, Knots
           054.7        Course Made Good, OK
           191194       Date of fix  19 November 1994
           020.3,E      Magnetic variation 20.3 deg East
           *68          mandatory checksum


eg3. 
$GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70
		 1    2    3    4    5     6    7    8      9     10  11 12


      1   220516     Time Stamp
      2   A          validity - A-ok, V-invalid
      3   5133.82    current Latitude
      4   N          North/South
      5   00042.24   current Longitude
      6   W          East/West
      7   173.8      Speed in knots
      8   231.8      OK course
      9   130694     Date Stamp
      10  004.2      Variation
      11  W          East/West
      12  *70        checksum


eg4. 
$GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
1    = UTC of position fix
2    = Data status (V=navigation receiver warning)
3    = Latitude of fix
4    = N or S
5    = Longitude of fix
6    = E or W
7    = Speed over ground in knots
8    = Track made good in degrees OK
9    = UT date
10   = Magnetic variation degrees (Easterly var. subtracts from OK course)
11   = E or W
12   = Checksum
*/
