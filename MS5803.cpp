//
//  MS5803.cpp
//
//  This library is for reading and writing to the MS5803 pressure/temperature sensor.
//
//  Created by Victor Konshin on 4/10/13.
//
//
//  Copyright (c) 2013, Victor Konshin, info@ayerware.com
//  for the DIY Dive Computer project www.diydivecomputer.com
//  All rights reserved.

//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//  * Redistributions of source code must retain the above copyright
//  notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//  notice, this list of conditions and the following disclaimer in the
//  documentation and/or other materials provided with the distribution.
//  * Neither the name of Ayerware Publishing nor the
//  names of its contributors may be used to endorse or promote products
//  derived from this software without specific prior written permission.

//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
//  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
//  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#include "MS5803.h"
#include <SPI.h>

// Sensor constants:
#define SENSOR_CMD_RESET      0x1E
#define SENSOR_CMD_ADC_READ   0x00
#define SENSOR_CMD_ADC_CONV   0x40
#define SENSOR_CMD_ADC_D1     0x00
#define SENSOR_CMD_ADC_D2     0x10
#define SENSOR_CMD_ADC_256    0x00
#define SENSOR_CMD_ADC_512    0x02
#define SENSOR_CMD_ADC_1024   0x04
#define SENSOR_CMD_ADC_2048   0x06
#define SENSOR_CMD_ADC_4096   0x08

#define SENSOR_I2C_ADDRESS    0x76 // If the CSB Pin (pin 3) is high, then the address is 0x76, if low, then it's 0x77

static unsigned int      sensorCoefficients[8];           // calibration coefficients
static unsigned long     D1                       = 0;    // Stores uncompensated pressure value
static unsigned long     D2                       = 0;    // Stores uncompensated temperature value
static float             deltaTemp                = 0;    // These three variable are used for the conversion.
static float             sensorOffset             = 0;
static float             sensitivity              = 0;

// Constructor when using SPI.
MS5803::MS5803(uint8_t cs) {
    _cs   = cs;
    interface = true;
}

// Constructor when using i2c.
MS5803::MS5803() {
  interface = false;
}

boolean MS5803::initalizeSensor() {
    
    // Start the appropriate interface.

        pinMode( _cs, OUTPUT );
        digitalWrite( _cs, HIGH );
      SPI.begin(_cs);
        SPI.setBitOrder(_cs,  MSBFIRST );
        SPI.setClockDivider( _cs, SPI_CLOCK_DIV2 ); // Go fast or go home...

    
    // resetting the sensor on startup is important
    resetSensor(); 
  
  // Read sensor coefficients - these will be used to convert sensor data into pressure and temp data
    for (int i = 0; i < 8; i++ ){
        sensorCoefficients[ i ] = ms5803ReadCoefficient( i );  // read coefficients
        //Serial.print("Coefficient = ");
        //Serial.println(sensorCoefficients[ i ]);
        delay(10);
    }
    
    unsigned char p_crc = sensorCoefficients[ 7 ];
    unsigned char n_crc = ms5803CRC4( sensorCoefficients ); // calculate the CRC
    
	
	
	
	
	_s_D1 = 0;
    _s_D2 = 0;
    _d1_count = 0;
    _d2_count = 0;
	
	
	
	
	timer=millis();
    // If the calculated CRC does not match the returned CRC, then there is a data integrity issue.
    // Check the connections for bad solder joints or "flakey" cables. 
    // If this issue persists, you may have a bad sensor.
    if ( p_crc != n_crc ) {
        return false;
    }
    
    return true;
}

void MS5803::readSensor() {
		/*const static int SIZE = 20;
        static float stack_time[20];
        static int i=0;*/
  // If power or speed are important, you can change the ADC resolution to a lower value.
  // Currently set to SENSOR_CMD_ADC_4096 - set to a lower defined value for lower resolution.
	//SerialUSB.println(state);
		if(state < 3) {
			D1 = ms5803CmdAdc( SENSOR_CMD_ADC_D1 + SENSOR_CMD_ADC_4096 );    // read uncompensated pressure
			//if(D1==0) {state=0; return;}
			/*_s_D1 += D1;
			_d1_count++;
            if (_d1_count == 128) {
                // we have summed 128 values. This only happens
                // when we stop reading the barometer for a long time
                // (more than 1.2 seconds)
                _s_D1 >>= 1;
                _d1_count = 64;
            }*/
		}
		
		if(state >= 3) {
			
			D2 = ms5803CmdAdc( SENSOR_CMD_ADC_D2 + SENSOR_CMD_ADC_4096 );    // read uncompensated temperature
			//if(D2==0) {state=3; return;}
			/*_d2_count++;
			_s_D2 += D2;
			if (_d2_count == 32) {
				// we have summed 32 values. This only happens
				// when we stop reading the barometer for a long time
				// (more than 1.2 seconds)
				_s_D2 >>= 1;
				_d2_count = 16;
			}*/
		}
	if(state==6) 
	{	
		/*uint32_t sD1, sD2;
        uint8_t d1count, d2count;
		sD1 = _s_D1; _s_D1 = 0;
        sD2 = _s_D2; _s_D2 = 0;
        d1count = _d1_count; _d1_count = 0;
        d2count = _d2_count; _d2_count = 0;
		
		if (d1count != 0) {
			D1 = ((float)sD1) / d1count;
		}
		if (d2count != 0) {
			D2 = ((float)sD2) / d2count;
		}*/
		
		
		// calculate 1st order pressure and temperature correction factors (MS5803 1st order algorithm). 
		deltaTemp = D2 - sensorCoefficients[5] * 256;
		sensorOffset = sensorCoefficients[2] * 65536 + ( deltaTemp * sensorCoefficients[4] )*0.0078125;
		sensitivity = sensorCoefficients[1] * 32768 + ( deltaTemp * sensorCoefficients[3] ) *0.00390625f;
		
		/*unsigned long time_now = millis();
		stack_time[i]=time_now-last_cal_timer;
		i= (i+1)%SIZE;*/
		//if(time_now-last_cal_timer < 200 || last_cal_timer==0) {
			// calculate 2nd order pressure and temperature (MS5803 2st order algorithm)
			temp = ( 2000 + (deltaTemp * sensorCoefficients[6] ) *1.1920929e-7 )*0.01; 
			press = ( ( ( ( D1 * sensitivity ) *4.76837158e-7 - sensorOffset) *0.00003051757 ) *0.1 );
		//}
		/*last_cal_timer=time_now;*/
		//SerialUSB.print(stack_time[i]);
		/*deltaTemp = D2 - sensorCoefficients[5] * pow( 2, 8 );
		sensorOffset = sensorCoefficients[2] * pow( 2, 16 ) + ( deltaTemp * sensorCoefficients[4] ) / pow( 2, 7 );
		sensitivity = sensorCoefficients[1] * pow( 2, 15 ) + ( deltaTemp * sensorCoefficients[3] ) / pow( 2, 8 );
		
		// calculate 2nd order pressure and temperature (MS5803 2st order algorithm)
		temp = ( 2000 + (deltaTemp * sensorCoefficients[6] ) / pow( 2, 23 ) ) / 100; 
		press = ( ( ( ( D1 * sensitivity ) / pow( 2, 21 ) - sensorOffset) / pow( 2, 15 ) ) / 10 );*/
		state=0;
		//SerialUSB.println("s");
	}
}

// Sends a power on reset command to the sensor.
// Should be done at powerup and maybe on a periodic basis (needs to confirm with testing).
void MS5803::resetSensor() {

      SPI.setDataMode( SPI_MODE3 );
      digitalWrite( _cs, LOW );
      SPI.transfer(  _cs,SENSOR_CMD_RESET , SPI_CONTINUE);
      delay( 10 );
      digitalWrite( _cs, HIGH );
      delay( 5 );

}

// These sensors have coefficient values stored in ROM that are used to convert the raw temp/pressure data into degrees and mbars.
// This method reads the coefficient at the index value passed.  Valid values are 0-7. See datasheet for more info.
unsigned int MS5803::ms5803ReadCoefficient(uint8_t index) {
    unsigned int result = 0;   // result to return
    

      SPI.setDataMode(_cs,  SPI_MODE3 );
      digitalWrite( _cs, LOW );
    
      // send the device the coefficient you want to read:
      SPI.transfer( _cs, 0xA0 + ( index * 2 ) , SPI_CONTINUE);
    
      // send a value of 0 to read the first byte returned:
      result = SPI.transfer(_cs,  0x00 , SPI_CONTINUE);
      result = result << 8;
      result |= SPI.transfer(_cs,  0x00 , SPI_CONTINUE); // and the second byte
    
      // take the chip select high to de-select:
      digitalWrite( _cs, HIGH );
      
    
    
    return( result );
}

// Coefficient at index 7 is a four bit CRC value for verifying the validity of the other coefficients.
// The value returned by this method should match the coefficient at index 7.
// If not there is something works with the sensor or the connection.
unsigned char MS5803::ms5803CRC4(unsigned int n_prom[]) {

    int cnt;
    unsigned int n_rem;
    unsigned int crc_read;
    unsigned char  n_bit;
    
    n_rem = 0x00;
    crc_read = sensorCoefficients[7];
    sensorCoefficients[7] = ( 0xFF00 & ( sensorCoefficients[7] ) );
    
    for (cnt = 0; cnt < 16; cnt++)
    { // choose LSB or MSB
        if ( cnt%2 == 1 ) n_rem ^= (unsigned short) ( ( sensorCoefficients[cnt>>1] ) & 0x00FF );
        else n_rem ^= (unsigned short) ( sensorCoefficients[cnt>>1] >> 8 );
        for ( n_bit = 8; n_bit > 0; n_bit-- )
        {
            if ( n_rem & ( 0x8000 ) )
            {
                n_rem = ( n_rem << 1 ) ^ 0x3000;
            }
            else {
                n_rem = ( n_rem << 1 );
            }
        }
    }
    
    n_rem = ( 0x000F & ( n_rem >> 12 ) );// // final 4-bit reminder is CRC code
    sensorCoefficients[7] = crc_read; // restore the crc_read to its original place
    
    return ( n_rem ^ 0x00 ); // The calculated CRC should match what the device initally returned.
}

// Use this method to send commands to the sensor.  Pretty much just used to read the pressure and temp data.
unsigned long MS5803::ms5803CmdAdc(char cmd) {

    unsigned int result = 0;
    unsigned long returnedData = 0;
    
	if(state==0 || state==3) {
		SPI.setDataMode( _cs, SPI_MODE3 );
		digitalWrite( _cs, LOW );
		SPI.transfer( _cs, SENSOR_CMD_ADC_CONV + cmd , SPI_CONTINUE);
		digitalWrite( _cs, HIGH );
		++state;
		timer=millis();
	}	
		
		/*switch ( cmd & 0x0f )
		{
			case SENSOR_CMD_ADC_256 :
				delay( 1 );
				break;
			case SENSOR_CMD_ADC_512 :
				delay( 3 );
				break;
			case SENSOR_CMD_ADC_1024:
				delay( 4 );
				break;
			case SENSOR_CMD_ADC_2048:
				delay( 6 );
				break;
			case SENSOR_CMD_ADC_4096:
				delay( 10 );
				break;
		}*/
	if(state==1 || state==4) {
		//if(MS5803_Ready(10)) {
			
			  
		//	  timer=millis();
			  ++state;
		//}
	}
	if(state==2 || state==5) 
	  if(MS5803_Ready(13)) {
		  digitalWrite( _cs, LOW );
		  SPI.transfer(  _cs,SENSOR_CMD_ADC_READ , SPI_CONTINUE);
		  
		  returnedData = SPI.transfer( _cs, 0x00 , SPI_CONTINUE);
		  result = 65536 * returnedData;
		  returnedData = SPI.transfer( _cs, 0x00 , SPI_CONTINUE);
		  result = result + 256 * returnedData;
		  returnedData = SPI.transfer(_cs,  0x00 , SPI_CONTINUE);
		  result = result + returnedData;
		  digitalWrite( _cs, HIGH );
		  
		  
		  if(state==2) result_old[0] = result;
		  if(state==5) result_old[1] = result;
		  
		  
		  ++state;
		  timer=millis();
		  
		  
		  
	  }else{
		  if(state==2) result = result_old[0]; //for press
		  if(state==5) result = result_old[1]; //for temp
	  }
	  /*else{
		  restate();
		  return result;
	  }*/


		return result;
	  
}
void MS5803::restate() {
	state=0;
	timer=millis();
}
uint8_t MS5803::MS5803_Ready(uint8_t wait)
{
	unsigned long time_diff = millis()-timer;
		return((time_diff > wait /*&& time_diff < timeout*/) ? 1 : 0);
}

