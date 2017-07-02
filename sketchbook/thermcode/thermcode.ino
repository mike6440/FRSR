//==========================================================================
double ThermistorTemp(int16_t v, double Vref, double Rref, unsigned int ntherm){

	double Vref;
	double r;
	double t, t2;
	double beta[]={ 1.025579e-03,   2.397338e-04,   1.542038e-07};

// 	if(v<threshold[0]) return 0;
// 	if(v>threshold[1]) return 200;
//unsigned int GetAdcVolts(unsigned int ch, double *vmean) {
	Vref = GetAdcVolts(3);

	r = Rref * (v / (Vref-v));
	vx = log(r);
	//Serial.print("r="); Serial.print(r,3);
	//Serial.print("	vx="); Serial.print(vx,3);
	//Serial.print("	beta="); Serial.print(beta
	//fitted curve
	t2 = beta[0] + beta[1] * vx + beta[2] * vx * vx * vx;
	t = 1 / t2 - (double)273.15;

	return t;
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
//============================================================  
unsigned int GetThermCoefs(unsigned int nt, double c[] ){
    // Give the correct SHH coefs for the thermnumber
    // COMPUTE TEML

    double tcal[8][3]={
        {
            1.025579e-03,   2.397338e-04,   1.542038e-07                        }
        ,   // standard ysi
        {
            1.0108115e-03, 2.4212099e-04,   1.4525424e-07                        }
        ,
        {
            1.0138029e-03, 2.4156995e-04,   1.4628056e-07                        }
        ,
        {
            1.0101740e-03, 2.4208389e-04,   1.4485814e-07                        }
        ,
        {
            1.0137647e-03, 2.4161708e-04,   1.4619775e-07                        }
        ,
        {
            1.0136495e-03, 2.4158562e-04,   1.4769608e-07                        }
        ,
        {
            1.0116767e-03, 2.4183780e-04,   1.4673176e-07                        }
        ,
        {
            1.0077377e-03, 2.4235481e-04,   1.4556543e-07                        }
    };

    if(nt < 0 || nt > 7){
        Serial.print("Error in GetThermCoefs() -- bad thermnumber=");
        Serial.println(nt,DEC);
        c[0]=c[1]=c[2]=0;
        return 0;
    } 
    else {
        c[0]=tcal[nt][0]; 
        c[1]=tcal[nt][1]; 
        c[2]=tcal[nt][2];
        return 1;
    } 
}
=============FRSR_V2===========================================

float GetHeadTemp(void)
/*************************************
Read the analog circuit corresponding to the head temperature.
Put degC into TempHead
RMR 991101
*************************************/
{
	float TempHead;
	int id;
	char str[8];
	double r1,r2, rt;
	double vref, v,d;
	
	vref = 4097; // millivolts measured on TT8
	r1 = 4980; // k ohms 
	r2 = 4983; // k ohms measured from TT8 plug to ground.

	ReadAnalog(7, &id, Missing);  // read temp channel and put into arrays
	v = (double)id; // AD6, millivolts
	//printf("Chan 7 millivolts = %.1lf\n", v);
	if ( v < 100 || v > 4000 ) return Missing;
	
	// vref ---rt ----r1--<adc6>---r2---gnd
	rt = (r2 - (v/vref)*(r1+r2)) / (v/vref);
	if ( rt < 2000 || rt > 50000 ) return Missing;	
	//printf("Head therm resistance = %.3lf\n", rt);
	
	// SH-H equation
	d = log(1/rt);
	d = A1 * d * d * d + A2 * d * d + A3 * d + A4;
	TempHead = 1.0 / d - 273.15;

	/***********
	Error check
	************/
	if( TempHead < -20 || TempHead > 50)
	{
		TempHead = Missing;
		SerPutByte('x');
	}
	return TempHead;
}
