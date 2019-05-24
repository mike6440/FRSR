// 20170330T010004Z
// #include <stdio.h>
// #include <stdlib.h>
#include <string.h>


#define		DONE	3
#define 	OK	 	1
#define		NOTOK 	0
#define		LENGPS	300
#define		RMCCOMMAS 12

// GLOBAL VARIABLES
char	OutStr[300];
unsigned int df_index;
char 	bufgps[300];		// GPS input string
byte	offsets[13];	// indexes of commas and the *
int 	byteGPS;		// GPS processing
int		ctr1, ctr2;		// GPS processing
char 	gpsid[7] = "$GPRMC";


// FUNCTIONS
byte nmea_validateChecksum(char *strPtr);
byte ReadGps(void);
// void ParseGps(void);
//byte PullGpsField(byte loc, char *field);

// SETUP
void setup() {
	Serial.begin(115200);
	Serial3.begin(9600);
	Serial.println("START GPSDEV");
	Serial3.setTimeout(100);
}

// LOOP
void loop() 
{
	int k;
	//read in gps
	df_index=0;
	k = ReadGps();
	if( k == OK){
		//Serial.println(bufgps);
	} else {
		k=ReadGps();
		if(k==OK){
			//Serial.println(bufgps);
		}
		else { 
			//Serial.println("Bad");
			strcpy(bufgps,"$GPRMC,NaN*XX");
		}
	}
	ParseGps();
	Serial.print("<<");Serial.print(OutStr);Serial.println(">>");
	delay(4000);
	//while( !Serial.available() );
	//Serial.read();
}
//=================================================================
byte ReadGps(void) {
	byte bytestatus;
	byte j;

	byteGPS=-1;
	// reset
	ctr1 = ctr2 = 0;
	// clear the buffer
	for(j=0;j<200;j++) bufgps[j]='\0';
	// clear serial uart
	while( Serial3.available() ) Serial3.read();
	// Wait for the '$'
	while(1){
		if(Serial3.available() && Serial3.read() == '$'){
			ctr1=1; ctr2=0; bufgps[0]='$';
			break;
		}
		else delay(100);
	}
	 
	// loop reading all bytes from the gps
	for(j=0;j<200;j++){
		delay(2); // allow time for the next char
		byteGPS=Serial3.read();
		bytestatus = handle_byte(byteGPS);
		// packet fails, start over
		if( bytestatus == NOTOK ) {		
			return NOTOK;
		}
		// packet complete
		else if( bytestatus == DONE ){
			bufgps[ctr1-2]='\0';
			return OK;
		}
	}
	return NOTOK;
}
//============================================================
int handle_byte(int byteGPS) {
	byte j;
	bufgps[ctr1] = byteGPS;
	ctr1++;
	if( ctr1 >= 300 ) return NOTOK;
	// trap commas
	if( byteGPS == ',') {
		ctr2++;
		//Serial.print(ctr2);Serial.print(" ");
		offsets[ctr2] = ctr1;
		if( ctr2 >= 13 ) {
			Serial.print("-C-");
			return NOTOK;
		}
	}
	// trap '*'
	if( byteGPS == '*') {
		//Serial.print(" asteric ");
		ctr2++;
		offsets[ctr2] = ctr1;
	}
	// trap CR
	if( byteGPS == 13) {
		//Serial.print("13  CR ");
	}
	// trap LF
	if( byteGPS == 10) {
		//Serial.println("10  LF ");
		// test print line
		//Serial.print("packet: ");
		//Serial.print("[");
		//for( j=0; j<ctr1-2; j++ ) Serial.print(bufgps[j]);
		//Serial.println("");
		
		// 12 fields and useable header
		if( ctr2 != RMCCOMMAS ) {
			Serial.print("-F-");
			Serial.print(byteGPS);Serial.print(",");
			Serial.print(ctr1);Serial.print(",");Serial.print(ctr2);
			Serial.println("");
			for(j=0;j<ctr1-2;j++){
				Serial.print((char)bufgps[j]);
			}
			Serial.println("");
			return NOTOK;
		}
		// useable header
		if( fieldsize(0) != 6 ) {
			Serial.print("-J-");
			for(j=0;j<=6;j++){
				Serial.print((char)bufgps[j]);			}
			return NOTOK;
		}
		//Serial.println("");
		// header GPRMC
		for( j=0; j<6; j++ ) {
			//Serial.print(bufgps[j]);
			if( bufgps[j] != gpsid[j] ) {
				Serial.print("-H-");
				return NOTOK;
			}
		}
		//checksum
		if( nmea_validateChecksum(bufgps) != OK ) {
			Serial.print("-S-");
			return NOTOK;
		}
		// valid data
// 		if( bufgps[14] != 'A' ){
// 			Serial.println("-A-");
// 			return NOTOK;
// 		}
			//Serial.println("  CHK OK");
		return DONE;
	}
	//Serial.println("");
	return OK;
}
//=======================================================
void ParseGps(void){
//$GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70
//-->> "<<220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W>>"
//-->> "<<NaN,V,NaN,NaN,NaN,NaN,NaN,NaN>>"
	int j;
	
	for(j=7;j<strlen(bufgps)-3;j++){
		OutStr[df_index]=bufgps[j]; df_index++;
	}
	OutStr[df_index]='\0';
	return;
}
//===================================================
int fieldsize(int j) {
	return offsets[j+1] - offsets[j] - 1;
}
// =================================================
// byte PullGpsField(byte loc, char *field)
// // loc = the comma from 0 at the start of the string
// //         0     1   2     3  4       5  6     7     8      9    10
// // $GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70
// {
// 	byte j,n;
// 	if(loc<0 || loc > 11){
// 		Serial.println("PullGpsField bad loc.");
// 		return NOTOK;
// 	}
// 	n=fieldsize(loc);
// 	if(n<=0) strcpy(field,"NaN");
// 	else {
// 		j=0;
// 		while( j < n ) {
// 			field[j] = bufgps[offset[j]+j];
// 			j++;
// 		}
// 		field[j]='\0';
// 	}
// }
// 	// 
//=================================================================
// this takes a nmea gps string, and validates it againts the checksum
// at the end if the string. (requires first byte to be $)
byte nmea_validateChecksum(char *strPtr)
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
// 
// //$GPRMC,190824,A,4737.0000,S,12300.0000,W,002.1,202.0,210210,019.0,W*62
// sub NmeaChecksum
// // $cc = NmeaChecksum($str) where $str is the NMEA string that starts with '$' and ends with '*'.
// {
//     my ($line) = @_;
//     my $csum = 0;
//     $csum ^= unpack("C",(substr($line,$_,1))) for(1..length($line)-2);
//     return (sprintf("%2.2X",$csum));
// }
// 

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
