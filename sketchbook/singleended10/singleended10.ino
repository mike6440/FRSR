#define	NSWEEP  250
#define NCHANS  7

#define Adc0  0
#define Adc1  1
#define Adc2  2
#define Adc3  3
#define Adc4  4
#define Adc5  5
#define Adc6  6

unsigned long SampUsecs, SweepUsecs;
int adcmv[NCHANS*NSWEEP];			// All filter outputs
byte Chan[]={Adc0,Adc1,Adc2,Adc3,Adc4,Adc5,Adc6};
unsigned long sweeptime;
unsigned int isamp,ix,iy,iz;
byte i,ib,ic;
unsigned long tm0,tm1;

void setup() {
  Serial.begin(9600);
  SweepUsecs=3e6;
  SampUsecs=SweepUsecs/NSWEEP;
}

void loop() 
{
	int a;
	// SAMPLE 7 CHANS, 250 SEC
	tm0=sweeptime=micros();
	isamp=0;
	Serial.print("Start sample.....");
	for(iy=0; iy<NSWEEP; iy++){
		for(ib=0; ib<NCHANS; ib++){
			*(adcmv+isamp) = analogRead(Chan[ib]);
			isamp++;
		}
		sweeptime+=SampUsecs;
		while(micros()<=sweeptime){}
	}
	Serial.println("end");
	// print result
	tm1=micros();
	isamp=0;
	for(iy=0; iy<NSWEEP; iy++){
		Serial.print(iy,DEC); Serial.print(" "); // sample count
		for(iz=0; iz<NCHANS; iz++){
			Serial.print(adcmv[isamp],DEC); Serial.print(" ");
			isamp++;
		}
		Serial.println("");
	}
	// PRINT OUT STATS
	Serial.print("Sample time: "); Serial.print(tm1-tm0,DEC); Serial.println(" usecs");		

// 	a=analogRead(0);
// 	Serial.println(a,DEC);
	Serial.println("Waiting");
	delay(1000);
	while(Serial.available() <= 0);
}
