/*
	Authored 2016-2018. Phillip Stanley-Marbell.

	Additional contributions, 2018 onwards: Jan Heck, Chatura Samarakoon, Youchao Wang, Sam Willis.

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
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"
#include "fsl_lpuart_driver.h"
#include "adc_low_power.h"
#include "fsl_smc_hal.h"
#include "fsl_pmc_hal.h"
#include "fsl_adc16_driver.h"
#include "fsl_adc_irq.c"
#include "lptmr_trigger.c"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
#include "board.h"
#include "devBME680.h"

#define ADC_0                   (0U)
#define CHANNEL_0               (0U)
#define LED_ON                  (0U)
#define LED_OFF                 (1U)
/*!
 * @brief These values are used to get the temperature. DO NOT MODIFY
 * The method used in this demo to calculate temperature of chip is mapped to
 * Temperature Sensor for the HCS08 Microcontroller Family document (Document Number: AN3031)
 */
#define ADCR_VDD                (4096U)    /*! Maximum value when use 16b resolution */
#define V_BG                    (1000U)     /*! BANDGAP voltage in mV (trim to 1.0V) */
#define V_TEMP25                (716U)      /*! Typical VTEMP25 in mV */
#define M                       (1620U)     /*! Typical slope: (mV x 1000)/oC */
#define STANDARD_TEMP           (25)

#define UPPER_VALUE_LIMIT       (1U)        /*! This value/10 is going to be added to current Temp to set the upper boundary*/
#define LOWER_VALUE_LIMIT       (1U)        /*! This Value/10 is going to be subtracted from current Temp to set the lower boundary*/
#define UPDATE_BOUNDARIES_TIME  (20U)       /*! This value indicates the number of cycles needed to update boundaries. To know the Time it will take, multiply this value times LPTMR_COMPARE_VALUE*/
#define kAdcChannelTemperature  (26U)       /*! ADC channel of temperature sensor */
#define kAdcChannelPTA8		    (3U)       /*! ADC channel of PTA8 */
#define kAdcChannelPTA9		    (2U)       /*! ADC channel of temperature sensor */
#define kAdcChannelBandgap      (27U)       /*! ADC channel of BANDGAP */

#define WARP_FRDMKL03
#define BME680_CONCAT_BYTES(msb, lsb)	(((uint16_t)msb << 8) | (uint16_t)lsb)
#define BME680_CONCAT_BYTESxlsb(msb, xlsb)	(((uint32_t)msb << 4) | (uint32_t)xlsb)

#define WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
//#define WARP_BUILD_BOOT_TO_CSVSTREAM


/*
*	BTstack includes WIP
*/
// #include "btstack_main.h"


#define						kWarpConstantStringI2cFailure		"\rI2C failed, reg 0x%02x, code %d\n"
#define						kWarpConstantStringErrorInvalidVoltage	"\rInvalid supply voltage [%d] mV!"
#define						kWarpConstantStringErrorSanity		"\rSanity check failed!"

extern void init_trigger_source(uint32_t instance);
/*
 *	TODO: move this and possibly others into a global structure
 */
volatile i2c_master_state_t			i2cMasterState;
volatile spi_master_state_t			spiMasterState;
volatile spi_master_user_config_t		spiUserConfig;
volatile lpuart_user_config_t 			lpuartUserConfig;
volatile lpuart_state_t 			lpuartState;
volatile WarpSPIDeviceState			deviceISL23415State;

/*
 *	TODO: move magic default numbers into constant definitions.
 */
volatile uint32_t			gWarpI2cBaudRateKbps		= 1;
volatile uint32_t			gWarpUartBaudRateKbps		= 1;
volatile uint32_t			gWarpSpiBaudRateKbps		= 1;
volatile uint32_t			gWarpSleeptimeSeconds		= 0;
volatile WarpModeMask			gWarpMode			= kWarpModeDisableAdcOnSleep;
volatile uint32_t			gWarpI2cTimeoutMilliseconds	= 5;
volatile uint32_t			gWarpSpiTimeoutMicroseconds	= 5000;
volatile uint32_t			gWarpMenuPrintDelayMilliseconds	= 10;
volatile uint32_t			gWarpSupplySettlingDelayMilliseconds = 1;
volatile WarpI2CDeviceState			deviceBME680State;
volatile uint8_t				deviceBME680CalibrationValues[kWarpSizesBME680CalibrationValuesCount];

void					sleepUntilReset(void);
void					lowPowerPinStates(void);

void					printPinDirections(void);
void					dumpProcessorState(void);
void					repeatRegisterReadForDeviceAndAddress(WarpSensorDevice warpSensorDevice, uint8_t baseAddress, 
								uint8_t pullupValue, bool autoIncrement, int chunkReadsPerAddress, bool chatty,
								int spinDelay, int repetitionsPerAddress, uint16_t sssupplyMillivolts,
								uint16_t adaptiveSssupplyMaxMillivolts, uint8_t referenceByte);
int						char2int(int character);
void					enableSssupply(uint16_t voltageMillivolts);
void					disableSssupply(void);
void					activateAllLowPowerSensorModes(bool verbose);
void					powerupAllSensors(void);
uint8_t					readHexByte(void);
int						read4digits(void);
void					printAllSensors(bool printHeadersAndCalibration, bool hexModeFlag, int menuDelayBetweenEachRun, int i2cPullupValue);
void 					enableI2Cpins(void);
float gaussToGauss(float sample, float targetMu, float targetSigma);


/*
 *	TODO: change the following to take byte arrays
 */
WarpStatus				writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte);
WarpStatus				writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength);

volatile bool conversionCompleted = false;  /*! Conversion is completed Flag */
const uint32_t gSimBaseAddr[] = SIM_BASE_ADDRS;


void					warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState);



/*
 *	From KSDK power_manager_demo.c <<BEGIN>>>
 */

clock_manager_error_code_t clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData);

/*
 *	static clock callback table.
 */
clock_manager_callback_user_config_t		clockManagerCallbackUserlevelStructure =
									{
										.callback	= clockManagerCallbackRoutine,
										.callbackType	= kClockManagerCallbackBeforeAfter,
										.callbackData	= NULL
									};

static clock_manager_callback_user_config_t *	clockCallbackTable[] =
									{
										&clockManagerCallbackUserlevelStructure
									};

clock_manager_error_code_t
clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData)
{
	clock_manager_error_code_t result = kClockManagerSuccess;

	switch (notify->notifyType)
	{
		case kClockManagerNotifyBefore:
			break;
		case kClockManagerNotifyRecover:
		case kClockManagerNotifyAfter:
			break;
		default:
			result = kClockManagerError;
		break;
	}

	return result;
}

uint8_t calculateOffsetPotSetting(float valueADC)
{
	float offset = 0.0160179826285282*valueADC*valueADC -2.94290424692658*valueADC + 204.442678174054;
	return (uint8_t) round(offset);
}

uint8_t calculateGainPotSetting(float valueADC)
{
	float offset = 0.0357020277485262*valueADC*valueADC + 4.04032562151349*valueADC + 20.408353362387;
	return (uint8_t) round(offset);
}

int
printADCValue(adc16_chn_config_t adcChnConfig)
{
    ADC16_DRV_ConfigConvChn(ADC_0, CHANNEL_0, &adcChnConfig);

    // Wait for the conversion to be done
    ADC16_DRV_WaitConvDone(ADC_0, CHANNEL_0);

    // Get current ADC BANDGAP value
    int value = ADC16_DRV_GetConvValueRAW(ADC_0, CHANNEL_0);
    //bandgapValue = ADC16_DRV_ConvRAWData(bandgapValue, false, adcUserConfig.resolutionMode);
   
    // ADC stop conversion
    ADC16_DRV_PauseConv(ADC_0, CHANNEL_0);
	return value;
}

float
sampleFromKernelDensity(float means[], float covars[], float weights[], uint32_t rand, float sample)
{
	
	uint8_t index = 6;
	float valuef = rand / 4294967295.0;
	if(valuef <= weights[0])
	{
		index = 0;
		sample = gaussToGauss(sample, means[index], covars[index]);
		return sample;
	}
	if(valuef <= weights[1] + weights[0])
	{
		index = 1;
		sample = gaussToGauss(sample, means[index], covars[index]);
		return sample;
	}
	if(valuef <= weights[0] + weights[1] + weights[2])
	{
		index = 2;
		sample = gaussToGauss(sample, means[index], covars[index]);
		return sample;
	}
	if(valuef <= weights[0] + weights[1] + weights[2] + weights[3])
	{
		index = 3;
		sample = gaussToGauss(sample, means[index], covars[index]);
		return sample;
	}
	if(valuef <= weights[0] + weights[1] + weights[2] + weights[3] + weights[4]) 
	{
		index = 4;
		sample = gaussToGauss(sample, means[index], covars[index]);
		return sample;
	}
	
}

/*
 *	Override the RTC IRQ handler
 */
void
RTC_IRQHandler(void)
{
	if (RTC_DRV_IsAlarmPending(0))
	{
		RTC_DRV_SetAlarmIntCmd(0, false);
	}
}

/*
 *	Override the RTC Second IRQ handler
 */
void
RTC_Seconds_IRQHandler(void)
{
	gWarpSleeptimeSeconds++;
}

/*
 *	Power manager user callback
 */
power_manager_error_code_t callback0(power_manager_notify_struct_t *  notify,
					power_manager_callback_data_t *  dataPtr)
{
	WarpPowerManagerCallbackStructure *		callbackUserData = (WarpPowerManagerCallbackStructure *) dataPtr;
	power_manager_error_code_t			status = kPowerManagerError;

	switch (notify->notifyType)
	{
		case kPowerManagerNotifyBefore:
			status = kPowerManagerSuccess;
			break;
		case kPowerManagerNotifyAfter:
			status = kPowerManagerSuccess;
			break;
		default:
			callbackUserData->errorCount++;
			break;
	}

	return status;
}

// TODO: add pin states for pan1326 lp states
void
lowPowerPinStates(void)
{
	/*
	 *	Following Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf),
	 *	we configure all pins as output and set them to a known state. We choose
	 *	to set them all to '0' since it happens that the devices we want to keep
	 *	deactivated (SI4705, PAN1326) also need '0'.
	 */

	/*
	 *			PORT A
	 */
	/*
	 *	For now, don't touch the PTA0/1/2 SWD pins. Revisit in the future.
	 */
	/*
	PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAsGpio);
	*/

	/*
	 *	PTA3 and PTA4 are the EXTAL/XTAL
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);

	PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);

	/*
	 *	NOTE: The KL03 has no PTA10 or PTA11
	 */

	PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortMuxAsGpio);



	/*
	 *			PORT B
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);

	/*
	 *	PTB1 is connected to KL03_VDD. We have a choice of:
	 *		(1) Keep 'disabled as analog'.
	 *		(2) Set as output and drive high.
	 *
	 *	Pin state "disabled" means default functionality (ADC) is _active_
	 */
	if (gWarpMode & kWarpModeDisableAdcOnSleep)
	{
		PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortMuxAsGpio);
	}
	else
	{
		PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortPinDisabled);
	}

	PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);

	/*
	 *	PTB3 and PTB3 (I2C pins) are true open-drain
	 *	and we purposefully leave them disabled.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);


	PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortMuxAsGpio);

	/*
	 *	NOTE: The KL03 has no PTB8 or PTB9
	 */

	PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortMuxAsGpio);

	/*
	 *	NOTE: The KL03 has no PTB12
	 */

	PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortMuxAsGpio);



	/*
	 *	Now, set all the pins (except kWarpPinKL03_VDD_ADC, the SWD pins, and the XTAL/EXTAL) to 0
	 */



	/*
	 *	If we are in mode where we disable the ADC, then drive the pin high since it is tied to KL03_VDD
	 */
	if (gWarpMode & kWarpModeDisableAdcOnSleep)
	{
		GPIO_DRV_SetPinOutput(kWarpPinKL03_VDD_ADC);
	}
#ifndef WARP_BUILD_ENABLE_THERMALCHAMBERANALYSIS
#ifdef WARP_BUILD_ENABLE_DEVPAN1326
	GPIO_DRV_ClearPinOutput(kWarpPinPAN1326_nSHUTD);
#endif
#endif

	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_CTLEN);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740B_CTLEN);

	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);

#ifndef WARP_BUILD_ENABLE_THERMALCHAMBERANALYSIS
	GPIO_DRV_ClearPinOutput(kWarpPinCLKOUT32K);
#endif

	GPIO_DRV_ClearPinOutput(kWarpPinTS5A3154_IN);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);



	/*
	 *	When the PAN1326 is installed, note that it has the
	 *	following pull-up/down by default:
	 *
	 *		HCI_RX / kWarpPinI2C0_SCL	: pull up
	 *		HCI_TX / kWarpPinI2C0_SDA	: pull up
	 *		HCI_RTS / kWarpPinSPI_MISO	: pull up
	 *		HCI_CTS / kWarpPinSPI_MOSI	: pull up
	 *
	 *	These I/Os are 8mA (see panasonic_PAN13xx.pdf, page 10),
	 *	so we really don't want to be driving them low. We
	 *	however also have to be careful of the I2C pullup and
	 *	pull-up gating. However, driving them high leads to
	 *	higher board power dissipation even when SSSUPPLY is off
	 *	by ~80mW on board #003 (PAN1326 populated).
	 *
	 *	In revB board, with the ISL23415 DCP pullups, we also
	 *	want I2C_SCL and I2C_SDA driven high since when we
	 *	send a shutdown command to the DCP it will connect
	 *	those lines to 25570_VOUT.
	 *
	 *	For now, we therefore leave the SPI pins low and the
	 *	I2C pins (PTB3, PTB4, which are true open-drain) disabled.
	 */

	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SDA);
	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SCL);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK);

	/*
	 *	HCI_RX / kWarpPinI2C0_SCL is an input. Set it low.
	 */
	//GPIO_DRV_SetPinOutput(kWarpPinI2C0_SCL);

	/*
	 *	HCI_TX / kWarpPinI2C0_SDA is an output. Set it high.
	 */
	//GPIO_DRV_SetPinOutput(kWarpPinI2C0_SDA);

	/*
	 *	HCI_RTS / kWarpPinSPI_MISO is an output. Set it high.
	 */
	//GPIO_DRV_SetPinOutput(kWarpPinSPI_MISO);

	/*
	 *	From PAN1326 manual, page 10:
	 *
	 *		"When HCI_CTS is high, then CC256X is not allowed to send data to Host device"
	 */
	//GPIO_DRV_SetPinOutput(kWarpPinSPI_MOSI);
}

uint8_t
checkSum(uint8_t *  pointer, uint16_t length) /*	Adapted from https://stackoverflow.com/questions/31151032/writing-an-8-bit-checksum-in-c	*/
{
	unsigned int sum;
	for ( sum = 0 ; length != 0 ; length-- )
	{
		sum += *(pointer++);
	}
	return (uint8_t)sum;
}

void setWiperPot(uint8_t tap, uint32_t pin)
{
	uint8_t		outBuffer[2];
	outBuffer[0] = 0x11;
	outBuffer[1] = tap;
	
	GPIO_DRV_SetPinOutput(pin);
	enableSPIpins();
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(pin);
	writeBytesToSpi(outBuffer /* payloadByte */, 2 /* payloadLength */);
	GPIO_DRV_SetPinOutput(pin);
	OSA_TimeDelay(10);
	disableSPIpins();
}

float gaussToGauss(float sample, float targetMu, float targetSigma)
{
	
	float a = targetSigma/0.092449093063203;
	float b = targetMu - 0.499214020341909*a;
	return (sample*a + b);
}

void
enableSPIpins(void)
{
	CLOCK_SYS_EnableSpiClock(0);

	/*	Warp KL03_SPI_MISO	--> PTA6	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAlt3);

	/*	Warp KL03_SPI_MOSI	--> PTA7	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAlt3);

	/*	Warp KL03_SPI_SCK	--> PTB0	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAlt3);


	/*
	 *	Initialize SPI master. See KSDK13APIRM.pdf Section 70.4
	 *
	 */
	uint32_t			calculatedBaudRate;
	spiUserConfig.polarity		= kSpiClockPolarity_ActiveLow;
	spiUserConfig.phase		= kSpiClockPhase_FirstEdge;
	spiUserConfig.direction		= kSpiMsbFirst;
	spiUserConfig.bitsPerSec	= 1;
	SPI_DRV_MasterInit(0 /* SPI master instance */, (spi_master_state_t *)&spiMasterState);
	SPI_DRV_MasterConfigureBus(0 /* SPI master instance */, (spi_master_user_config_t *)&spiUserConfig, &calculatedBaudRate);
}

void
disableSPIpins(void)
{
	SPI_DRV_MasterDeinit(0);


	/*	Warp KL03_SPI_MISO	--> PTA6	(GPI)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);

	/*	Warp KL03_SPI_MOSI	--> PTA7	(GPIO)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

	/*	Warp KL03_SPI_SCK	--> PTB0	(GPIO)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);

	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK);


	CLOCK_SYS_DisableSpiClock(0);
}

uint32_t 
linearCongruential(uint32_t previous)
{
	return (1664525*previous + 1013904223) % 4294967296;
}

float 
boxMueller(float mu, float sigma, float U1, float U2)
{  
  float R2 = -2*logf(U1); 
  float R = sqrtf(R2);
  float theta = 2*M_PI*U2;  
  return R*cosf(theta)*sigma + mu;
}

void
enableI2Cpins(void)
{
	CLOCK_SYS_EnableI2cClock(0);

	/*	Warp KL03_I2C0_SCL	--> PTB3	(ALT2 == I2C)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt2);

	/*	Warp KL03_I2C0_SDA	--> PTB4	(ALT2 == I2C)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt2);


	I2C_DRV_MasterInit(0 /* I2C instance */, (i2c_master_state_t *)&i2cMasterState);
}

WarpStatus
writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength)
{
	uint8_t		inBuffer[payloadLength];
	spi_status_t	status;

	
	status = SPI_DRV_MasterTransferBlocking(0		/* master instance */,
						NULL		/* spi_master_user_config_t */,
						payloadBytes,
						inBuffer,
						payloadLength	/* transfer size */,
						1000		/* timeout in microseconds (unlike I2C which is ms) */);
	//disableSPIpins();

	return (status == kStatus_SPI_Success ? kWarpStatusOK : kWarpStatusCommsError);
}

int
main(void)
{
	rtc_datetime_t				warpBootDate;

	power_manager_user_config_t		warpPowerModeWaitConfig;
	power_manager_user_config_t		warpPowerModeStopConfig;
	power_manager_user_config_t		warpPowerModeVlpwConfig;
	power_manager_user_config_t		warpPowerModeVlpsConfig;
	power_manager_user_config_t		warpPowerModeVlls0Config;
	power_manager_user_config_t		warpPowerModeVlls1Config;
	power_manager_user_config_t		warpPowerModeVlls3Config;
	power_manager_user_config_t		warpPowerModeRunConfig;

	const power_manager_user_config_t	warpPowerModeVlprConfig = {
							.mode			= kPowerManagerVlpr,
							.sleepOnExitValue	= false,
							.sleepOnExitOption	= false
						};

	power_manager_user_config_t const *	powerConfigs[] = {
							/*
							 *	NOTE: This order is depended on by POWER_SYS_SetMode()
							 *
							 *	See KSDK13APIRM.pdf Section 55.5.3
							 */
							&warpPowerModeWaitConfig,
							&warpPowerModeStopConfig,
							&warpPowerModeVlprConfig,
							&warpPowerModeVlpwConfig,
							&warpPowerModeVlpsConfig,
							&warpPowerModeVlls0Config,
							&warpPowerModeVlls1Config,
							&warpPowerModeVlls3Config,
							&warpPowerModeRunConfig,
						};

	WarpPowerManagerCallbackStructure			powerManagerCallbackStructure;

	/*
	 *	Callback configuration structure for power manager
	 */
	const power_manager_callback_user_config_t callbackCfg0 = {
							callback0,
							kPowerManagerCallbackBeforeAfter,
							(power_manager_callback_data_t *) &powerManagerCallbackStructure};

	/*
	 *	Pointers to power manager callbacks.
	 */
	power_manager_callback_user_config_t const *	callbacks[] = {
								&callbackCfg0
						};



	/*
	 *	Enable clock for I/O PORT A and PORT B
	 */
	CLOCK_SYS_EnablePortClock(0);
	CLOCK_SYS_EnablePortClock(1);



	/*
	 *	Setup board clock source.
	 */
	g_xtal0ClkFreq = 32768U;



	/*
	 *	Initialize KSDK Operating System Abstraction layer (OSA) layer.
	 */
	OSA_Init();



	/*
	 *	Setup SEGGER RTT to output as much as fits in buffers.
	 *
	 *	Using SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL can lead to deadlock, since
	 *	we might have SWD disabled at time of blockage.
	 */
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);


	SEGGER_RTT_WriteString(0, "\n\n\n\rBooting Warp, in 3... ");
	OSA_TimeDelay(200);
	SEGGER_RTT_WriteString(0, "2... ");
	OSA_TimeDelay(200);
	SEGGER_RTT_WriteString(0, "1...\n\r");
	OSA_TimeDelay(200);



	/*
	 *	Configure Clock Manager to default, and set callback for Clock Manager mode transition.
	 *
	 *	See "Clocks and Low Power modes with KSDK and Processor Expert" document (Low_Power_KSDK_PEx.pdf)
	 */
	CLOCK_SYS_Init(	g_defaultClockConfigurations,
			CLOCK_CONFIG_NUM,
			&clockCallbackTable,
			ARRAY_SIZE(clockCallbackTable)
			);
	CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);



	/*
	 *	Initialize RTC Driver
	 */
	RTC_DRV_Init(0);



	/*
	 *	Set initial date to 1st January 2016 00:00, and set date via RTC driver
	 */
	warpBootDate.year	= 2016U;
	warpBootDate.month	= 1U;
	warpBootDate.day	= 1U;
	warpBootDate.hour	= 0U;
	warpBootDate.minute	= 0U;
	warpBootDate.second	= 0U;
	RTC_DRV_SetDatetime(0, &warpBootDate);



	/*
	 *	Setup Power Manager Driver
	 */
	memset(&powerManagerCallbackStructure, 0, sizeof(WarpPowerManagerCallbackStructure));


	warpPowerModeVlpwConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpwConfig.mode = kPowerManagerVlpw;

	warpPowerModeVlpsConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpsConfig.mode = kPowerManagerVlps;

	warpPowerModeWaitConfig = warpPowerModeVlprConfig;
	warpPowerModeWaitConfig.mode = kPowerManagerWait;

	warpPowerModeStopConfig = warpPowerModeVlprConfig;
	warpPowerModeStopConfig.mode = kPowerManagerStop;

	warpPowerModeVlls0Config = warpPowerModeVlprConfig;
	warpPowerModeVlls0Config.mode = kPowerManagerVlls0;

	warpPowerModeVlls1Config = warpPowerModeVlprConfig;
	warpPowerModeVlls1Config.mode = kPowerManagerVlls1;

	warpPowerModeVlls3Config = warpPowerModeVlprConfig;
	warpPowerModeVlls3Config.mode = kPowerManagerVlls3;

	warpPowerModeRunConfig.mode = kPowerManagerRun;

	POWER_SYS_Init(	&powerConfigs,
			sizeof(powerConfigs)/sizeof(power_manager_user_config_t *),
			&callbacks,
			sizeof(callbacks)/sizeof(power_manager_callback_user_config_t *)
			);



	/*
	 *	Switch CPU to Very Low Power Run (VLPR) mode
	 */
	warpSetLowPowerMode(kWarpPowerModeVLPR, 0);



	/*
	 *	Initialize the GPIO pins with the appropriate pull-up, etc.,
	 *	defined in the inputPins and outputPins arrays (gpio_pins.c).
	 *
	 *	See also Section 30.3.3 GPIO Initialization of KSDK13APIRM.pdf
	 */
	GPIO_DRV_Init(inputPins  /* input pins */, outputPins  /* output pins */);

	/*
	 *	Note that it is lowPowerPinStates() that sets the pin mux mode,
	 *	so until we call it pins are in their default state.
	 */
	lowPowerPinStates();



	/*
	 *	Toggle LED3 (kWarpPinSI4705_nRST)
	 */
	GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(200);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(200);
	GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(200);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(200);
	GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(200);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
	
#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    adc16_calibration_param_t adcCalibraitionParam;
#endif
    adc16_user_config_t adcUserConfig;
    adc16_chn_config_t adcChnConfigTemperature;
	adc16_chn_config_t adcChnConfigNoise;

#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    // Auto calibration
    ADC16_DRV_GetAutoCalibrationParam(ADC_0, &adcCalibraitionParam);
    ADC16_DRV_SetCalibrationParam(ADC_0, &adcCalibraitionParam);
#endif

    // Enable BANDGAP reference voltage
    PMC_HAL_SetBandgapBufferCmd(PMC_BASE, true);

    // Initialization ADC for
    // 16bit resolution, interrupt mode, hw trigger disabled.
    // normal convert speed, VREFH/L as reference,
    // disable continuous convert mode.
    ADC16_DRV_StructInitUserConfigDefault(&adcUserConfig);
    adcUserConfig.resolutionMode = kAdcResolutionBitOf12or13;
	// Change to true
    adcUserConfig.continuousConvEnable = true;
    adcUserConfig.clkSrcMode =  kAdcClkSrcOfBusClk;
	ADC16_DRV_DisableHwAverage(ADC_0);
	ADC16_DRV_DisableLongSample(ADC_0);
    ADC16_DRV_Init(ADC_0, &adcUserConfig);
	


    adcChnConfigNoise.chnNum =kAdcChannelPTA8;
    adcChnConfigNoise.diffEnable = false;
    adcChnConfigNoise.intEnable = false;
    adcChnConfigNoise.chnMux = kAdcChnMuxOfA;
	
    adcChnConfigTemperature.chnNum =kAdcChannelTemperature;
    adcChnConfigTemperature.diffEnable = false;
    adcChnConfigTemperature.intEnable = false;
    adcChnConfigTemperature.chnMux = kAdcChnMuxOfA;
	
	float valuef = 0; 
	uint32_t value = 0;
	uint32_t uniform1 = 437365735; 

	float means1[5] = {-26.0910794, -28.9200358, -22.3299326, -31.2545249, -24.1708072};
	float sds1[5] = {830565.6952, 1045690.198, 881892.0575, 1036736.085, 816746.5641};
	float covars1[5] = {0.689839374, 1.09346799, 0.777733601, 1.07482171, 0.667074950};
	float weights1[5] = {0.26561657, 0.11522003, 0.22869728, 0.13056866, 0.25989746};
	float means2[5] = {339.6, 338.1, 344.88, 342.94, 338.62};
	float sds2[5] = {85.44848893, 64.66542987, 57.4458928, 89.85784935, 70.88380123};
	float covars2[5] = {7.30144426e-03, 4.18161782e-03, 3.30003060e-03, 8.07443309e-03, 5.02451328e-03};
	float weights2[5] = {0.23538615, 0.21981091, 0.13647497, 0.07250419, 0.33582379};
	float means3[5] = {-138.59384823, -137.63772743, -139.91446049, -139.3033656, -138.09661793};
	float sds3[5] = {0.2141309646, 0.2197095355, 0.265789221, 0.2728457073, 0.2073725633};
	float covars3[5] = {0.21413096, 0.21970953, 0.26578923, 0.27284571, 0.20737257};
	float weights3[5] = {0.27174589, 0.22555792, 0.13933054, 0.11332549, 0.25004015};
	float temperaturef = 0; 
	uint32_t temperature = 0;
	
	float par_t1 = 0;
	float par_t2 = 0;
	float par_t3 = 0; 
	
	
	//while(1)
	//{
	//SEGGER_RTT_printf(0, "fortyRepeat%d = [", x);
		value = 0;
		for(int i = 0 ; i < 10; i++)
		{
			value = value + printADCValue(adcChnConfigTemperature);
		}
		valuef = value/10;
		valuef = 376.473 - 0.359*valuef;
		// Set offset
		setWiperPot(calculateOffsetPotSetting(valuef), kWarpPinISL23415_nCS);
		// Set gain
		setWiperPot(calculateGainPotSetting(valuef), kWarpPinPAN1326_nSHUTD);
		
		uint16_t	readSensorRegisterValueLSB;
		uint16_t	readSensorRegisterValueMSB;
		uint16_t	readSensorRegisterValueXLSB;
	
		initBME680(	0x76	/* i2cAddress */,	&deviceBME680State	);
	
		enableI2Cpins();
		gWarpI2cBaudRateKbps = 1000000;
		configureSensorBME680(0b00000001,	/*	Humidity oversampling (OSRS) to 16x				*/
				          0b10100100,	/*	Temperature oversample 16x, pressure overdsample 16x, mode 00	*/
						  0b00001000,	/*	Turn off heater							*/
						  0b00000000);
	
	
		SEGGER_RTT_printf(0, "\n%d, %d,",RTC->TSR, RTC->TPR);
		writeSensorRegisterBME680(kWarpSensorConfigurationRegisterBME680Ctrl_Meas, 0b00100101);
		readSensorRegisterBME680(kWarpSensorOutputRegisterBME680temp_msb, 1);
		readSensorRegisterValueMSB = deviceBME680State.i2cBuffer[0];
		readSensorRegisterBME680(kWarpSensorOutputRegisterBME680temp_lsb, 1);
		readSensorRegisterValueLSB = deviceBME680State.i2cBuffer[0];
		readSensorRegisterBME680(kWarpSensorOutputRegisterBME680temp_xlsb, 1);
		readSensorRegisterValueXLSB = deviceBME680State.i2cBuffer[0];
		temperature = (readSensorRegisterValueMSB << 12) | (readSensorRegisterValueLSB << 4) | (readSensorRegisterValueXLSB >> 4);
		for(int j = 0 ; j < 10000 ; j++)
		{
			
			((*(__IO hw_adc_sc1n_t *)((0x4003B000))).U = 0x43);
			while ( !((*(volatile uint32_t*)(0x5383B000))))
			{}
			uniform1 = linearCongruential(uniform1);
			value = (((*(volatile uint32_t*)(0x507BB010))) << 20) + (uniform1 & 0b00000000000011111111111111111111);
			valuef = value / 4294967295.0;
			uniform1 = linearCongruential(uniform1);
			par_t3 = sampleFromKernelDensity(means3, covars3, weights3, uniform1, valuef);
			//SEGGER_RTT_printf(0, "\npar_t3 = %d",(int) (par_t3*1000000));
			
			((*(__IO hw_adc_sc1n_t *)((0x4003B000))).U = 0x43);
			while ( !((*(volatile uint32_t*)(0x5383B000))))
			{}
			uniform1 = linearCongruential(uniform1);
			value = ((4095-(*(volatile uint32_t*)(0x507BB010))) << 20) + (uniform1 & 0b00000000000011111111111111111111);
			valuef = value / 4294967295.0;
			uniform1 = linearCongruential(uniform1);
			par_t2 = sampleFromKernelDensity(means2, covars2, weights2, uniform1, valuef);
			//SEGGER_RTT_printf(0, "\npar_t2 = %d",(int) (par_t2*1000000));
			
			((*(__IO hw_adc_sc1n_t *)((0x4003B000))).U = 0x43);
			while ( !((*(volatile uint32_t*)(0x5383B000))))
			{}
			uniform1 = linearCongruential(uniform1);
			value = (((*(volatile uint32_t*)(0x507BB010))) << 20) + (uniform1 & 0b00000000000011111111111111111111);
			valuef = value / 4294967295.0;
			uniform1 = linearCongruential(uniform1);
			par_t1 = sampleFromKernelDensity(means1, covars1, weights1, uniform1, valuef);
			//SEGGER_RTT_printf(0, "\npar_t1 = %d",(int) (par_t1*1000000));
			temperature = par_t1*temperature*temperature/1E24 + temperature*par_t2/1000000.0 + par_t3;
			
			((*(__IO hw_adc_sc1n_t *)((0x4003B000))).U = 0x43);
			while ( !((*(volatile uint32_t*)(0x5383B000))))
			{}
			uniform1 = linearCongruential(uniform1);
			value = ((4095-(*(volatile uint32_t*)(0x507BB010))) << 20) + (uniform1 & 0b00000000000011111111111111111111);
			valuef = value / 4294967295.0;
			uniform1 = linearCongruential(uniform1);
			par_t3 = sampleFromKernelDensity(means3, covars3, weights3, uniform1, valuef);
			//SEGGER_RTT_printf(0, "\npar_t3 = %d",(int) (par_t3*1000000));
			
			((*(__IO hw_adc_sc1n_t *)((0x4003B000))).U = 0x43);
			while ( !((*(volatile uint32_t*)(0x5383B000))))
			{}
			uniform1 = linearCongruential(uniform1);
			value = (((*(volatile uint32_t*)(0x507BB010))) << 20) + (uniform1 & 0b00000000000011111111111111111111);
			valuef = value / 4294967295.0;
			uniform1 = linearCongruential(uniform1);
			par_t2 = sampleFromKernelDensity(means2, covars2, weights2, uniform1, valuef);
			//SEGGER_RTT_printf(0, "\npar_t2 = %d",(int) (par_t2*1000000));
			
			((*(__IO hw_adc_sc1n_t *)((0x4003B000))).U = 0x43);
			while ( !((*(volatile uint32_t*)(0x5383B000))))
			{}
			uniform1 = linearCongruential(uniform1);
			value = ((4095-(*(volatile uint32_t*)(0x507BB010))) << 20) + (uniform1 & 0b00000000000011111111111111111111);
			valuef = value / 4294967295.0;
			uniform1 = linearCongruential(uniform1);
			par_t1 = sampleFromKernelDensity(means1, covars1, weights1, uniform1, valuef);
			//SEGGER_RTT_printf(0, "\npar_t1 = %d",(int) (par_t1*1000000));
			temperaturef = par_t1*temperature*temperature/1E24 + temperature*par_t2/1000000.0 + par_t3;
			
		}
		SEGGER_RTT_printf(0, "\n%d, %d,",RTC->TSR, RTC->TPR);
	//}
	
	
	
	
	
	
	// Set offset
	//setWiperPot(0x79, kWarpPinISL23415_nCS);
	

  	//uint8_t wiper = 0x00;
	//for(int i = 0 ; i < 256 ; i++)
	//{
		
	// Set gain
	//setWiperPot(wiper, kWarpPinPAN1326_nSHUTD);	
	
	//SEGGER_RTT_printf(0, "\n\nOffsetWiper%u \n", wiper);

	//OSA_TimeDelay(5000);
	//wiper++;
	//uint16_t value = 0;
	//SEGGER_RTT_printf(0, "\n%d\n", RTC->TSR);
	//for(int j = 0 ; j < 1000000 ; j++)
	//{
		// Read noise
		//((*(__IO hw_adc_sc1n_t *)((0x4003B000))).U = 0x43);
		//while ( !((*(volatile uint32_t*)(0x5383B000))))
		//{}
		//value = (*(volatile uint32_t*)(0x507BB010));
	//}
	//SEGGER_RTT_printf(0, "\n%d\n", RTC->TSR);
	//}
	
	//float means1[5] = {-138.59385849, -137.6377566, -139.91443101, -139.30349006, -138.09659595};
	//float covars1[5] = {0.01058122, 0.01113395, 0.01628832, 0.01716978, 0.00993019};
	//float weights1[5] = {0.27175676, 0.22551561, 0.13932415, 0.11330306, 0.25010041};
	//uint32_t uniform1 = 9948;
	//for(int i = 0; i < 1000; i++)
	//{
	//	uniform1 = linearCongruential(uniform1);
	//	((*(__IO hw_adc_sc1n_t *)((0x4003B000))).U = 0x43);
	//	while ( !((*(volatile uint32_t*)(0x5383B000))))
	//	{}
	//	float sample = sampleFromKernelDensity(means1, covars1, weights1, uniform1, (((*(volatile uint32_t*)(0x507BB010)))));
	//	SEGGER_RTT_printf(0, "%d\n", (int) (sample*1000000));
	//}
	
	//uint16_t	readSensorRegisterValueLSB;
	//uint16_t	readSensorRegisterValueMSB;
	//uint16_t	readSensorRegisterValueXLSB;
	
	//initBME680(	0x76	/* i2cAddress */,	&deviceBME680State	);
	
	//enableI2Cpins();
	//gWarpI2cBaudRateKbps = 1000000;
	

	
	//configureSensorBME680(0b00000001,	/*	Humidity oversampling (OSRS) to 16x				*/
	//			          0b10100100,	/*	Temperature oversample 16x, pressure overdsample 16x, mode 00	*/
	//					  0b00001000,	/*	Turn off heater							*/
	//					  0b00000000);
								
	//uint32_t uniform1 = 9948;
	//uint32_t uniform2 = 14837564;
	
	//uint16_t par_t1 = (uint16_t) (BME680_CONCAT_BYTES(deviceBME680CalibrationValues[34], deviceBME680CalibrationValues[33]));
	//uint16_t par_t2 = (uint16_t) (BME680_CONCAT_BYTES(deviceBME680CalibrationValues[2], deviceBME680CalibrationValues[1]));
	//uint8_t  par_t3 = (uint8_t)  deviceBME680CalibrationValues[3];
	//SEGGER_RTT_printf(0, "par_t3 = %u\n", par_t3);
	//uint32_t temperature = 0;
	//uint32_t pressure = 0;
	//uint32_t humidity = 0; 
	//uint32_t value = 0; 
	//float temperaturef = 0;
	
	//writeSensorRegisterBME680(kWarpSensorConfigurationRegisterBME680Ctrl_Meas, 0b00100101);
	//readSensorRegisterBME680(kWarpSensorOutputRegisterBME680temp_msb, 1);
	//readSensorRegisterValueMSB = deviceBME680State.i2cBuffer[0];
	//SEGGER_RTT_printf(0, "\nMSB = 0x%X,", readSensorRegisterValueMSB);
	//readSensorRegisterBME680(kWarpSensorOutputRegisterBME680temp_lsb, 1);
	//readSensorRegisterValueLSB = deviceBME680State.i2cBuffer[0];
	//SEGGER_RTT_printf(0, "\nLSB = 0x%X,", readSensorRegisterValueLSB);
	//readSensorRegisterBME680(kWarpSensorOutputRegisterBME680temp_xlsb, 1);
	//readSensorRegisterValueXLSB = deviceBME680State.i2cBuffer[0];
	//SEGGER_RTT_printf(0, "\nXLSB = 0x%X,", readSensorRegisterValueXLSB);
	//temperature = (readSensorRegisterValueMSB << 12) | (readSensorRegisterValueLSB << 4) | (readSensorRegisterValueXLSB >> 4);
	
	//SEGGER_RTT_printf(0, "\n%d\n", RTC->TSR);
	//for(int j = 0 ; j < 1000 ; j++)
	//{
	 							
		 
		
		//readSensorRegisterBME680(kWarpSensorOutputRegisterBME680press_msb, 1);
		//readSensorRegisterValueMSB = deviceBME680State.i2cBuffer[0];
		//readSensorRegisterBME680(kWarpSensorOutputRegisterBME680press_lsb, 1);
		//readSensorRegisterValueLSB = deviceBME680State.i2cBuffer[0];
		//readSensorRegisterBME680(kWarpSensorOutputRegisterBME680press_xlsb, 1);
		//readSensorRegisterValueXLSB = deviceBME680State.i2cBuffer[0];
		//pressure = (readSensorRegisterValueMSB << 12) | (readSensorRegisterValueLSB << 4) | (readSensorRegisterValueXLSB >> 4); 
		
		//readSensorRegisterBME680(kWarpSensorOutputRegisterBME680hum_msb, 1);
		//readSensorRegisterValueMSB = deviceBME680State.i2cBuffer[0];
		//readSensorRegisterBME680(kWarpSensorOutputRegisterBME680hum_lsb, 1);
		//readSensorRegisterValueLSB = deviceBME680State.i2cBuffer[0];
		//humidity = BME680_CONCAT_BYTES(readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		
		
		//SEGGER_RTT_printf(0, "\n%u, %u, %u", temperature, pressure, humidity);
		
		/* Get rands from Box Mueller */
		//uniform1 = linearCongruential(uniform1);
		//uniform2 = linearCongruential(uniform2);
		//SEGGER_RTT_printf(0, "\npar_t1 = %u",par_t1);
		//float par_t1_f = boxMueller((int) par_t1, 0.01*((int) par_t1), ((float)uniform1/(float)UINT32_MAX), ((float)uniform2/(float)UINT32_MAX));
		//SEGGER_RTT_printf(0, "\npar_t1_f = %d", (int) (par_t1_f*100));
		//uniform1 = linearCongruential(uniform1);
		//uniform2 = linearCongruential(uniform2);
		//SEGGER_RTT_printf(0, "\npar_t2 = %u",par_t2);
		//float par_t2_f = boxMueller((int) par_t2, 0.01*((int) par_t2), ((float)uniform1/(float)UINT32_MAX), ((float)uniform2/(float)UINT32_MAX));
		//SEGGER_RTT_printf(0, "\npar_t2_f = %d", (int) (par_t2_f*100));
		//uniform1 = linearCongruential(uniform1);
		//uniform2 = linearCongruential(uniform2);
		//SEGGER_RTT_printf(0, "\npar_t3 = %u",par_t3);
		//float par_t3_f = boxMueller((int) par_t3, 0.01*((int) par_t3), ((float)uniform1/(float)UINT32_MAX), ((float)uniform2/(float)UINT32_MAX));
		//SEGGER_RTT_printf(0, "\npar_t3_f = %d", (int) (par_t3_f*100));
		
		
		/* Get rands from PRVA*/
		//((*(__IO hw_adc_sc1n_t *)((0x4003B000))).U = 0x43);
		//while ( !((*(volatile uint32_t*)(0x5383B000))))
		//{}
		//uniform1 = linearCongruential(uniform1);
		//value = (((*(volatile uint32_t*)(0x507BB010))) << 20) + (uniform1 & 0b00000000000011111111111111111111);
		//float valuef = value / 4294967295.0;
		//float par_t1_f = gaussToGauss(valuef, (int) par_t1, 0.01*((int) par_t1));
		//SEGGER_RTT_printf(0, "\npar_t1_f = %d", (int) (par_t1_f*100));
			
		//((*(__IO hw_adc_sc1n_t *)((0x4003B000))).U = 0x43);
		//while ( !((*(volatile uint32_t*)(0x5383B000))))
		//{}
		//uniform1 = linearCongruential(uniform1);
		//value = (((*(volatile uint32_t*)(0x507BB010))) << 20) + (uniform1 & 0b00000000000011111111111111111111);
		//valuef = value / 4294967295.0;
		//float par_t2_f = gaussToGauss(valuef, (int) par_t2, 0.01*((int) par_t2));
		//SEGGER_RTT_printf(0, "\npar_t2_f = %d", (int) (par_t2_f*100));
		
		//((*(__IO hw_adc_sc1n_t *)((0x4003B000))).U = 0x43);
		//while ( !((*(volatile uint32_t*)(0x5383B000))))
		//{}
		//uniform1 = linearCongruential(uniform1);
		//value = (((*(volatile uint32_t*)(0x507BB010))) << 20) + (uniform1 & 0b00000000000011111111111111111111);
		//valuef = value / 4294967295.0;
		//float par_t3_f = gaussToGauss(valuef, (int) par_t3, 0.01*((int) par_t3));
		//SEGGER_RTT_printf(0, "\npar_t3_f = %d", (int) (par_t3_f*100));
		
		//temperaturef = calcMonteCarloT(readSensorRegisterValueMSB, readSensorRegisterValueLSB, readSensorRegisterValueLSB, par_t1_f, par_t2_f, par_t3_f); 
  		//SEGGER_RTT_printf(0, "\n%d", (int) (temperaturef*100));
	//}
	//SEGGER_RTT_printf(0, "\n%d\n", RTC->TSR);
	return 0;
}