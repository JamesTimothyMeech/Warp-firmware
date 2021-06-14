/*
	Authored 2016-2018. Phillip Stanley-Marbell, Youchao Wang, James Meech.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdlib.h>
#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"
#include "fsl_port_hal.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
#include <math.h>

#define BME680_CONCAT_BYTES(msb, lsb)	(((uint16_t)msb << 8) | (uint16_t)lsb)
#define BME680_CONCAT_BYTESxlsb(msb, xlsb)	(((uint32_t)msb << 4) | (uint32_t)xlsb)

float calcT(uint32_t temp_adc, uint8_t CalibVals[]);
float calcH(uint8_t hum_msb, uint8_t hum_lsb, uint8_t CalibVals[], float temp_comp); 
float equationCalcT(uint8_t temp_msb, uint8_t temp_lsb, uint8_t temp_xlsb, uint8_t CalibVals[]);
float equationCalcH(uint8_t hum_msb, uint8_t hum_lsb, uint8_t CalibVals[], float temp_comp);
float equationCalcP(uint8_t pres_msb, uint8_t pres_lsb, uint8_t pres_xlsb, uint8_t CalibVals[], float t_fine);
float calcP(uint8_t pres_msb, uint8_t pres_lsb, uint8_t pres_xlsb, uint8_t CalibVals[], float t_fine);
float findMean(float samples[], int size);
float findStandardDeviation(float samples[], float mean, int size);
float findCorrelationCoefficient(float samplesX[], float samplesY[], float meanX, float meanY, float standardDeviationX, float standardDeviationY, int size);
float Skewness(float samples[], float mean, float standardDeviation, int size);
float Kurtosis(float samples[], float mean, float standardDeviation, int size);
float findNewUncertainty(float sigmaH, float rhoHT);

extern volatile WarpI2CDeviceState	deviceBME680State;
extern volatile uint8_t			deviceBME680CalibrationValues[];
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;

void
initBME680(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (kWarpTypeMaskPressure | kWarpTypeMaskTemperature);
	return;
}

WarpStatus
writeSensorRegisterBME680(uint8_t deviceRegister, uint8_t payload)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	status;

	if (deviceRegister > 0xFF)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
	{
		.address = deviceBME680State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							1,
							gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
readSensorRegisterBME680(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);

	/*
	 *	We only check to see if it is past the config registers.
	 *
	 *	TODO: We should eventually numerate all the valid register addresses
	 *	(configuration, control, and calibration) here.
	 */
	if (deviceRegister > kWarpSensorConfigurationRegisterBME680CalibrationRegion2End)
	{
		return kWarpStatusBadDeviceCommand;
	}

	i2c_device_t slave =
	{
		.address = deviceBME680State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	cmdBuf[0] = deviceRegister;

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceBME680State.i2cBuffer,
							1,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}


WarpStatus
configureSensorBME680(uint8_t payloadCtrl_Hum, uint8_t payloadCtrl_Meas, uint8_t payloadGas_0, uint8_t payloadGas_1)
{
	WarpStatus	status1, status2, status3, status4 = 0;

	status1 = writeSensorRegisterBME680(kWarpSensorConfigurationRegisterBME680Ctrl_Hum,
							payloadCtrl_Hum);

	status2 = writeSensorRegisterBME680(kWarpSensorConfigurationRegisterBME680Ctrl_Meas,
							payloadCtrl_Meas);
 
	status3 = writeSensorRegisterBME680(kWarpSensorConfigurationRegisterBME680Ctrl_Gas_0,
							payloadGas_0);
							
	status4 = writeSensorRegisterBME680(kWarpSensorConfigurationRegisterBME680Ctrl_Gas_1,
							payloadGas_1);
	/*
	 *	Read the calibration registers
	 */
	uint8_t addr = 0x89; 
	for (int i = 0; i < 25; i++)
	{
		status4 |= readSensorRegisterBME680(addr, 1 /* numberOfBytes */);
		deviceBME680CalibrationValues[i] = deviceBME680State.i2cBuffer[0];
		addr++;
	}
	addr = 0xE1;
	for (int i = 0; i < 16; i++)
	{
		status4 |= readSensorRegisterBME680(addr, 1 /* numberOfBytes */);
		deviceBME680CalibrationValues[i+25] = deviceBME680State.i2cBuffer[0];
		addr++;
	}

	return (status1 | status2 | status3 | status4);
}

float equationCalcT(uint8_t temp_msb, uint8_t temp_lsb, uint8_t temp_xlsb, uint8_t CalibVals[]) 
{
	// Put together the  temperature calibration parameters out of values from the calibration register
	uint16_t par_t1 =(uint16_t) (BME680_CONCAT_BYTES(CalibVals[34], CalibVals[33])); 
	int16_t  par_t2 =(uint16_t) (BME680_CONCAT_BYTES(CalibVals[2], CalibVals[1])); 	
	int8_t   par_t3 =(uint8_t)  CalibVals[3];   
	
	// Get the whole temperature value from the three registers it is spread across	 
	uint32_t temp_adc = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4); 
	float t_out = (par_t2/5120.0f)*((temp_adc/16384.0f)-(par_t1/1024.0f)) + (par_t3/5242880.0f)*((temp_adc/16384.0f)-(par_t1/1024))*((temp_adc/16384.0f)-(par_t1/1024));
	return t_out;
}

float calcT(uint32_t temp_adc, uint8_t CalibVals[]) 
	{ 	 
	uint16_t par_t1 =(uint16_t) (BME680_CONCAT_BYTES(CalibVals[34], CalibVals[33])); 
	int16_t  par_t2 =(uint16_t) (BME680_CONCAT_BYTES(CalibVals[2], CalibVals[1])); 	
	int8_t   par_t3 =(uint8_t)  CalibVals[3];   	 	 
		 	 	 	 
	// Define variables to be used to calculate the compensated temperature
	float t_fine;
	float var1 = 0;
	float var2 = 0;
	float calc_temp = 0;

	/* calculate var1 data */
	var1  = ((((float)temp_adc / 16384.0f) - ((float)par_t1 / 1024.0f))
			* ((float)par_t2));

	/* calculate var2 data */
	var2  = (((((float)temp_adc / 131072.0f) - ((float)par_t1 / 8192.0f)) *
		(((float)temp_adc / 131072.0f) - ((float)par_t1 / 8192.0f))) *
		((float)par_t3 * 16.0f));

	/* t_fine value*/
	t_fine = (var1 + var2); 

	/* compensated temperature data*/
	calc_temp  = ((t_fine) / 5120.0f);
	// Store the calculated temperature in the array that was passed by reference
	return calc_temp; 
	} 
	
float 
calcMonteCarloT(uint8_t temp_msb, uint8_t temp_lsb, uint8_t temp_xlsb, float par_t1, float  par_t2, float par_t3) 
{ 	 	 		
	
	// Get the whole temperature value from the three registers it is spread across	 
	uint32_t temp_adc = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4); 
	float t_out = (par_t2/5120.0f)*(((float)temp_adc/16384.0f)-(par_t1/1024.0f)) + (par_t3/5242880.0f)*(((float)temp_adc/16384.0f)-(par_t1/1024))*(((float)temp_adc/16384.0f)-(par_t1/1024.0f));
	return t_out;
} 
	
	float calcH(uint8_t hum_msb, uint8_t hum_lsb, uint8_t CalibVals[], float temp_comp)
	{ 

	int16_t hum_adc= BME680_CONCAT_BYTES(hum_msb,hum_lsb);   

	// Combine the componets from the calibration array to get the calibration parameters
	uint16_t par_h1 = (uint16_t)(((uint16_t) CalibVals[27] << 4) | (CalibVals[26] & 0x0F));
	uint16_t par_h2 = (uint16_t)(((uint16_t) CalibVals[25] << 4) | (CalibVals[26] >> 4));
	int8_t   par_h3 = (int8_t) CalibVals[28]; 
	int8_t   par_h4 = (int8_t) CalibVals[29];
	int8_t   par_h5 = (int8_t) CalibVals[30]; 
	uint8_t  par_h6 = (uint8_t) CalibVals[31];
	int8_t   par_h7 = (int8_t) CalibVals[32];

	// Define variables to be used to calculate the compensated humidity
	float calc_hum = 0; 
	float var1 = 0;
	float var2 = 0;
    float var3 = 0;
	float var4 = 0;

	
        // Calculate humidity 
	var1 = (float)((float)hum_adc) - (((float)par_h1 * 16.0f) + (((float)par_h3 / 2.0f)
		* temp_comp));

	var2 = var1 * ((float)(((float) par_h2 / 262144.0f) * (1.0f + (((float)par_h4 / 16384.0f)
		* temp_comp) + (((float)par_h5 / 1048576.0f) * temp_comp * temp_comp))));

	var3 = (float) par_h6 / 16384.0f;

	var4 = (float) par_h7 / 2097152.0f;


	calc_hum = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);
	// Humidity  can't be more than 100 % or less than 0 %
	if (calc_hum > 100.0f)
		calc_hum = 100.0f;
	else if (calc_hum < 0.0f)
		calc_hum = 0.0f;

	// Store the calculated temperature in the array that was passed by reference
	return calc_hum; 
	}

	float equationCalcH(uint8_t hum_msb, uint8_t hum_lsb, uint8_t CalibVals[], float temp_comp)
	{ 

	int16_t hum_adc= BME680_CONCAT_BYTES(hum_msb,hum_lsb);   

	// Combine the componets from the calibration array to get the calibration parameters
	uint16_t par_h1 = (uint16_t)(((uint16_t) CalibVals[27] << 4) | (CalibVals[26] & 0x0F));
	uint16_t par_h2 = (uint16_t)(((uint16_t) CalibVals[25] << 4) | (CalibVals[26] >> 4));
	int8_t   par_h3 = (int8_t) CalibVals[28]; 
	int8_t   par_h4 = (int8_t) CalibVals[29];
	int8_t   par_h5 = (int8_t) CalibVals[30]; 
	uint8_t  par_h6 = (uint8_t) CalibVals[31];
	int8_t   par_h7 = (int8_t) CalibVals[32];

	float hum_comp = (par_h2/262144.0f) + (par_h2*par_h4/4294967296.0f)*temp_comp + (par_h2*par_h5/274877906944.0f)*temp_comp*temp_comp;
	float calc_hum = hum_comp*(hum_adc - 16.0f*par_h1 + (par_h3/2.0f)*temp_comp) + ((par_h6/16384.0f) + ((par_h7*temp_comp)/2097152.0f))*hum_comp*hum_comp*(hum_adc - par_h1*16.0f + (par_h3/2.0f)*temp_comp)*(hum_adc - par_h1*16.0f + (par_h3/2.0f)*temp_comp);
	return calc_hum; 
	}
	
	float calcP(uint8_t pres_msb, uint8_t pres_lsb, uint8_t pres_xlsb, uint8_t CalibVals[], float t_fine)  
		{  
	    uint32_t pres_adc; 

		uint8_t	 par_p10; 
		uint16_t par_p1; 
		int16_t  par_p2, par_p4, par_p5, par_p8, par_p9;
		int8_t   par_p3, par_p6, par_p7;

		// Combine the components to get the whole pressure value
		pres_adc = (pres_msb << 12) | (pres_lsb << 4) | (pres_xlsb >> 4); 
		//pres_adc = 419376; 
		SEGGER_RTT_printf(0, "\nOriginal pres_adc = %u,", pres_adc);
		// Combine the componets from the calibration array to get the calibration parameters
		par_p1 = BME680_CONCAT_BYTES(CalibVals[6],CalibVals[5]);
		par_p2 = BME680_CONCAT_BYTES(CalibVals[8],CalibVals[7]);
		par_p3 = CalibVals[9]; 
		par_p4 = BME680_CONCAT_BYTES(CalibVals[12],CalibVals[11]);
		par_p5 = BME680_CONCAT_BYTES(CalibVals[14],CalibVals[13]);
		par_p6 = CalibVals[16];
		par_p7 = CalibVals[15];
	    par_p8 = BME680_CONCAT_BYTES(CalibVals[20],CalibVals[19]);
		par_p9 = BME680_CONCAT_BYTES(CalibVals[22],CalibVals[21]);
		par_p10 = CalibVals[23];

		// 
		float calc_pres1 = 0; 
		float calc_pres2 = 0;
		float calc_pres3 = 0;
		float var1 = 0;
		float var2 = 0;
	    float var3 = 0;
		float var4 = 0; 
		float var5 = 0;
		float var6 = 0;
		float var8 = 0;
		float var9 = 0;
		float var10 = 0;
		//t_fine = 21.47483647;
		SEGGER_RTT_printf(0, "\nOriginal t_fine = %u,", (int)(t_fine*1000000000));
	    t_fine = t_fine * 5120.0f;

		 // Calculate pressure
		var1 = (((float)t_fine / 2.0f) - 64000.0f);
		SEGGER_RTT_printf(0, "\nOriginal Var 1 = %u,", (int)(var1*100));
		var2 = var1 * var1 * (((float)par_p6) / (131072.0f));
		SEGGER_RTT_printf(0, "\nOriginal Var 2 = %u,", (int)(var2*100));
		var3 = var2 + (var1 * ((float)par_p5) * 2.0f);
		SEGGER_RTT_printf(0, "\nOriginal Var 3 = %u,", (int)(var3*100));
		var4 = (var3 / 4.0f) + (((float)par_p4) * 65536.0f);
		SEGGER_RTT_printf(0, "\nOriginal Var 4 = %u,", (int)(var4*100));
		var5 = (((((float)par_p3 * var1 * var1) / 16384.0f) + ((float)par_p2 * var1)) / 524288.0f);
		SEGGER_RTT_printf(0, "\nOriginal Var 5 = %u,", (int)(var5*100));
		var6 = ((1.0f + (var5 / 32768.0f)) * ((float)par_p1));
		SEGGER_RTT_printf(0, "\nOriginal Var 6 = %u,", (int)(var6*100));
		calc_pres1 = (1048576.0f - ((float)pres_adc));
		SEGGER_RTT_printf(0, "\nOriginal calc_pres1 = %u,", (int)(calc_pres1*100));

		/* Avoid exception caused by division by zero */
		if ((int)var1 != 0) 
			{
			calc_pres2 = (((calc_pres1 - (var4 / 4096.0f)) * 6250.0f) / var6);
			SEGGER_RTT_printf(0, "\nOriginal calc_pres2 = %u,", (int)(calc_pres2*100));
			var8 = (((float)par_p9) * calc_pres2 * calc_pres2) / 2147483648.0f;
			SEGGER_RTT_printf(0, "\nOriginal var8 = %u,", (int)(var8*100));
			var9 = calc_pres2 * (((float)par_p8) / 32768.0f);
			SEGGER_RTT_printf(0, "\nOriginal var9 = %u,", (int)(var9*100));
			var10 = ((calc_pres2 / 256.0f) * (calc_pres2 / 256.0f) * (calc_pres2 / 256.0f) * (par_p10 / 131072.0f));
			SEGGER_RTT_printf(0, "\nOriginal var10 = %u,", (int)(var10*100));
			calc_pres3 = (calc_pres2 + (var8 + var9 + var10 + ((float)par_p7 * 128.0f)) / 16.0f);
			SEGGER_RTT_printf(0, "\nOriginal calc_pres3 = %u,", (int)(calc_pres3*100));
			} 
	
		else 
			{
			calc_pres3 = 0;
			} 
		// Store the calculated pressure in the array that was passed by reference
		return calc_pres3;
		}


		float equationCalcP(uint8_t pres_msb, uint8_t pres_lsb, uint8_t pres_xlsb, uint8_t CalibVals[], float t_fine)  
			{  
		    uint32_t pres_adc; 

			uint8_t	 par_p10; 
			uint16_t par_p1; 
			int16_t  par_p2, par_p4, par_p5, par_p8, par_p9;
			int8_t   par_p3, par_p6, par_p7;
			
			// Combine the components to get the whole pressure value
			pres_adc = (pres_msb << 12) | (pres_lsb << 4) | (pres_xlsb >> 4); 
			
			// Combine the componets from the calibration array to get the calibration parameters
			par_p1 = BME680_CONCAT_BYTES(CalibVals[6],CalibVals[5]);
			par_p2 = BME680_CONCAT_BYTES(CalibVals[8],CalibVals[7]);
			par_p3 = CalibVals[9]; 
			par_p4 = BME680_CONCAT_BYTES(CalibVals[12],CalibVals[11]);
			par_p5 = BME680_CONCAT_BYTES(CalibVals[14],CalibVals[13]);
			par_p6 = CalibVals[16];
			par_p7 = CalibVals[15];
		    par_p8 = BME680_CONCAT_BYTES(CalibVals[20],CalibVals[19]);
			par_p9 = BME680_CONCAT_BYTES(CalibVals[22],CalibVals[21]);
			par_p10 = CalibVals[23];

	
			float p_comp1 = (par_p6/524288.0f)*(2560.0f*t_fine - 64000.0f)*(2560.0f*t_fine - 64000.0f) + par_p5*(1280.0f*t_fine - 32000.0f) + 65536.0f*par_p4;
			SEGGER_RTT_printf(0, "\nCheck p_comp1 = %u,", (int)(p_comp1*100));
			float p_comp2 = (par_p1*par_p3/281474976710656.0f)*(2560.0f*t_fine - 64000.0f)*(2560.0f*t_fine - 64000.0f) + (par_p1*par_p2/17179869184.0f)*(2560.0f*t_fine - 64000.0f) + par_p1;
			SEGGER_RTT_printf(0, "\nCheck p_comp2 = %u,", (int)(p_comp2*100));
			float p_comp3 = (6250.0f/p_comp2)*(1048576.0f - pres_adc - (p_comp1/4096.0f));
			SEGGER_RTT_printf(0, "\nCheck p_comp3 = %u,", (int)(p_comp3*100));
			float calc_pres = (par_p10*p_comp3*p_comp3*p_comp3/35184372088832.0f) + (par_p9*p_comp3*p_comp3/34359738368.0f) + (1.0f + (par_p8/524288.0f))*p_comp3 + 8.0f*par_p7;
			SEGGER_RTT_printf(0, "\nCheck calc_pres = %u,", (int)(calc_pres*100));
			return calc_pres;
			}
  
float findMean(float samples[], int size)
{ 
    float sum = 0;
    for(int i = 0; i < size; i++)
    {
        sum += samples[i];
    }
    sum = sum/size; 
    return sum; 
}


float findStandardDeviation(float samples[], float mean, int size)
{
    float sum = 0;
    for(int i = 0; i < size; i++) 
    {
        sum += (samples[i]-mean)*(samples[i]-mean);
    }
    sum = sqrtf(sum/size); 
    return sum; 
}


float findCorrelationCoefficient(float samplesX[], float samplesY[], float meanX, float meanY, float standardDeviationX, float standardDeviationY, int size)
{ 
    float sumTop = 0;
    float sumBottom = 0;
    for(int i = 0; i < size; i++) 
    {
        sumTop += (samplesX[i]-meanX)*(samplesY[i]-meanY); 
    }
    sumBottom = sqrtf(standardDeviationX*standardDeviationX*size*standardDeviationY*standardDeviationY*size);
    return sumTop/sumBottom; 
}


float Skewness(float samples[], float mean, float standardDeviation, int size)
{
  float Skewness = 0;
  for(int i = 0; i < size; i++)
  {
  Skewness += (samples[i] - mean)*(samples[i] - mean)*(samples[i] - mean);
  }
  Skewness = Skewness/size;
  Skewness = Skewness/(standardDeviation*standardDeviation*standardDeviation);
  return Skewness;
}


float Kurtosis(float samples[], float mean, float standardDeviation, int size)
{
  float Kurtosis = 0;
  for(int i = 0; i < size; i++)
  {
  Kurtosis += (samples[i] - mean)*(samples[i] - mean)*(samples[i] - mean)*(samples[i] - mean);
  }
  Kurtosis = Kurtosis/size;
  Kurtosis = Kurtosis/(standardDeviation*standardDeviation*standardDeviation*standardDeviation);
  return Kurtosis;
}

float findNewUncertainty(float sigmaH, float rhoHT)
{
    float newSigmaH = sigmaH*sqrtf(fabsf(1-(rhoHT*rhoHT)));    
    return newSigmaH;
}