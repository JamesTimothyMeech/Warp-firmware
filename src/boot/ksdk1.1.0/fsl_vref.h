/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _FSL_VREF_H_
#define _FSL_VREF_H_

#include <stdint.h>
#include <assert.h>
#include "/Users/james/Desktop/Warp-firmware/tools/sdk/ksdk1.1.0/platform/CMSIS/Include/device/MKL03Z4/MKL03Z4.h"

/*! @brief Clock ip name array for VREF. */
#define VREF_CLOCKS  \
    {                \
        kCLOCK_Vref0 \
    }

#define VREF_SC_VREFEN(x)                        (((uint8_t)(((uint8_t)(x)) << VREF_SC_VREFEN_SHIFT)) & VREF_SC_VREFEN_MASK)
#define VREF_SC_REGEN(x)                         (((uint8_t)(((uint8_t)(x)) << VREF_SC_REGEN_SHIFT)) & VREF_SC_REGEN_MASK)
#define CLK_GATE_BIT_SHIFT_SHIFT 0U
#define CLK_GATE_REG_OFFSET_SHIFT 8U
#define CLK_GATE_REG_OFFSET_MASK 0xFFFFFF00U
#define CLK_GATE_BIT_SHIFT_MASK 0x000000FFU
#define CLK_GATE_ABSTRACT_REG_OFFSET(x) (((x)&CLK_GATE_REG_OFFSET_MASK) >> CLK_GATE_REG_OFFSET_SHIFT)
#define CLK_GATE_ABSTRACT_BITS_SHIFT(x) (((x)&CLK_GATE_BIT_SHIFT_MASK) >> CLK_GATE_BIT_SHIFT_SHIFT)
/* @brief VREF availability on the SoC. */
#define FSL_FEATURE_SOC_VREF_COUNT (1)	
	
#define CLK_GATE_DEFINE(reg_offset, bit_shift)                                  \
    ((((reg_offset) << CLK_GATE_REG_OFFSET_SHIFT) & CLK_GATE_REG_OFFSET_MASK) | \
     (((bit_shift) << CLK_GATE_BIT_SHIFT_SHIFT) & CLK_GATE_BIT_SHIFT_MASK))
	
	typedef enum _clock_ip_name
	{
	    kCLOCK_IpInvalid = 0U,
	    kCLOCK_I2c2      = CLK_GATE_DEFINE(0x1028U, 6U),
	    kCLOCK_Uart4     = CLK_GATE_DEFINE(0x1028U, 10U),
	    kCLOCK_Uart5     = CLK_GATE_DEFINE(0x1028U, 11U),

	    kCLOCK_Enet0 = CLK_GATE_DEFINE(0x102CU, 0U),
	    kCLOCK_Dac0  = CLK_GATE_DEFINE(0x102CU, 12U),
	    kCLOCK_Dac1  = CLK_GATE_DEFINE(0x102CU, 13U),

	    kCLOCK_Spi2  = CLK_GATE_DEFINE(0x1030U, 12U),
	    kCLOCK_Sdhc0 = CLK_GATE_DEFINE(0x1030U, 17U),
	    kCLOCK_Ftm3  = CLK_GATE_DEFINE(0x1030U, 25U),
	    kCLOCK_Adc1  = CLK_GATE_DEFINE(0x1030U, 27U),

	    kCLOCK_Ewm0   = CLK_GATE_DEFINE(0x1034U, 1U),
	    kCLOCK_Cmt0   = CLK_GATE_DEFINE(0x1034U, 2U),
	    kCLOCK_I2c0   = CLK_GATE_DEFINE(0x1034U, 6U),
	    kCLOCK_I2c1   = CLK_GATE_DEFINE(0x1034U, 7U),
	    kCLOCK_Uart0  = CLK_GATE_DEFINE(0x1034U, 10U),
	    kCLOCK_Uart1  = CLK_GATE_DEFINE(0x1034U, 11U),
	    kCLOCK_Uart2  = CLK_GATE_DEFINE(0x1034U, 12U),
	    kCLOCK_Uart3  = CLK_GATE_DEFINE(0x1034U, 13U),
	    kCLOCK_Usbfs0 = CLK_GATE_DEFINE(0x1034U, 18U),
	    kCLOCK_Cmp0   = CLK_GATE_DEFINE(0x1034U, 19U),
	    kCLOCK_Cmp1   = CLK_GATE_DEFINE(0x1034U, 19U),
	    kCLOCK_Cmp2   = CLK_GATE_DEFINE(0x1034U, 19U),
	    kCLOCK_Vref0  = CLK_GATE_DEFINE(0x1034U, 20U),

	    kCLOCK_Lptmr0 = CLK_GATE_DEFINE(0x1038U, 0U),
	    kCLOCK_PortA  = CLK_GATE_DEFINE(0x1038U, 9U),
	    kCLOCK_PortB  = CLK_GATE_DEFINE(0x1038U, 10U),
	    kCLOCK_PortC  = CLK_GATE_DEFINE(0x1038U, 11U),
	    kCLOCK_PortD  = CLK_GATE_DEFINE(0x1038U, 12U),
	    kCLOCK_PortE  = CLK_GATE_DEFINE(0x1038U, 13U),

	    kCLOCK_Ftf0     = CLK_GATE_DEFINE(0x103CU, 0U),
	    kCLOCK_Dmamux0  = CLK_GATE_DEFINE(0x103CU, 1U),
	    kCLOCK_Flexcan0 = CLK_GATE_DEFINE(0x103CU, 4U),
	    kCLOCK_Rnga0    = CLK_GATE_DEFINE(0x103CU, 9U),
	    kCLOCK_Spi0     = CLK_GATE_DEFINE(0x103CU, 12U),
	    kCLOCK_Spi1     = CLK_GATE_DEFINE(0x103CU, 13U),
	    kCLOCK_Sai0     = CLK_GATE_DEFINE(0x103CU, 15U),
	    kCLOCK_Crc0     = CLK_GATE_DEFINE(0x103CU, 18U),
	    kCLOCK_Usbdcd0  = CLK_GATE_DEFINE(0x103CU, 21U),
	    kCLOCK_Pdb0     = CLK_GATE_DEFINE(0x103CU, 22U),
	    kCLOCK_Pit0     = CLK_GATE_DEFINE(0x103CU, 23U),
	    kCLOCK_Ftm0     = CLK_GATE_DEFINE(0x103CU, 24U),
	    kCLOCK_Ftm1     = CLK_GATE_DEFINE(0x103CU, 25U),
	    kCLOCK_Ftm2     = CLK_GATE_DEFINE(0x103CU, 26U),
	    kCLOCK_Adc0     = CLK_GATE_DEFINE(0x103CU, 27U),
	    kCLOCK_Rtc0     = CLK_GATE_DEFINE(0x103CU, 29U),

	    kCLOCK_Flexbus0 = CLK_GATE_DEFINE(0x1040U, 0U),
	    kCLOCK_Dma0     = CLK_GATE_DEFINE(0x1040U, 1U),
	    kCLOCK_Sysmpu0  = CLK_GATE_DEFINE(0x1040U, 2U),
	} clock_ip_name_t;

/*!
 * @addtogroup vref
 * @{
 */

/*! @file */

/******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
#define FSL_VREF_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0. */
/*@}*/

/* Those macros below defined to support SoC family which have VREFL (0.4V) reference */
#if defined(FSL_FEATURE_VREF_HAS_LOW_REFERENCE) && FSL_FEATURE_VREF_HAS_LOW_REFERENCE
#define SC VREFH_SC
#define VREF_SC_MODE_LV VREF_VREFH_SC_MODE_LV
#define VREF_SC_REGEN VREF_VREFH_SC_REGEN
#define VREF_SC_VREFEN VREF_VREFH_SC_VREFEN
#define VREF_SC_ICOMPEN VREF_VREFH_SC_ICOMPEN
#define VREF_SC_REGEN_MASK VREF_VREFH_SC_REGEN_MASK
#define VREF_SC_VREFST_MASK VREF_VREFH_SC_VREFST_MASK
#define VREF_SC_VREFEN_MASK VREF_VREFH_SC_VREFEN_MASK
#define VREF_SC_MODE_LV_MASK VREF_VREFH_SC_MODE_LV_MASK
#define VREF_SC_ICOMPEN_MASK VREF_VREFH_SC_ICOMPEN_MASK
#define TRM VREFH_TRM
#define VREF_TRM_TRIM VREF_VREFH_TRM_TRIM
#define VREF_TRM_CHOPEN_MASK VREF_VREFH_TRM_CHOPEN_MASK
#define VREF_TRM_TRIM_MASK VREF_VREFH_TRM_TRIM_MASK
#define VREF_TRM_CHOPEN_SHIFT VREF_VREFH_TRM_CHOPEN_SHIFT
#define VREF_TRM_TRIM_SHIFT VREF_VREFH_TRM_TRIM_SHIFT
#define VREF_SC_MODE_LV_SHIFT VREF_VREFH_SC_MODE_LV_SHIFT
#define VREF_SC_REGEN_SHIFT VREF_VREFH_SC_REGEN_SHIFT
#define VREF_SC_VREFST_SHIFT VREF_VREFH_SC_VREFST_SHIFT
#define VREF_SC_ICOMPEN_SHIFT VREF_VREFH_SC_ICOMPEN_SHIFT
#endif /* FSL_FEATURE_VREF_HAS_LOW_REFERENCE */

/*!
 * @brief VREF modes.
 */
typedef enum _vref_buffer_mode
{
    kVREF_ModeBandgapOnly = 0U, /*!< Bandgap on only, for stabilization and startup */
#if defined(FSL_FEATURE_VREF_MODE_LV_TYPE) && FSL_FEATURE_VREF_MODE_LV_TYPE
    kVREF_ModeHighPowerBuffer = 1U, /*!< High power buffer mode enabled */
    kVREF_ModeLowPowerBuffer = 2U   /*!< Low power buffer mode enabled */
#else
    kVREF_ModeTightRegulationBuffer = 2U /*!< Tight regulation buffer enabled */
#endif /* FSL_FEATURE_VREF_MODE_LV_TYPE */
} vref_buffer_mode_t;

/*!
 * @brief The description structure for the VREF module.
 */
typedef struct _vref_config
{
    vref_buffer_mode_t bufferMode; /*!< Buffer mode selection */
#if defined(FSL_FEATURE_VREF_HAS_LOW_REFERENCE) && FSL_FEATURE_VREF_HAS_LOW_REFERENCE
    bool enableLowRef;          /*!< Set VREFL (0.4 V) reference buffer enable or disable */
    bool enableExternalVoltRef; /*!< Select external voltage reference or not (internal) */
#endif                          /* FSL_FEATURE_VREF_HAS_LOW_REFERENCE */
} vref_config_t;

/******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*!
 * @name VREF functional operation
 * @{
 */
	
static inline void CLOCK_EnableClock(clock_ip_name_t name)
{
	uint32_t regAddr = SIM_BASE + CLK_GATE_ABSTRACT_REG_OFFSET((uint32_t)name);
	(*(volatile uint32_t *)regAddr) |= (1UL << CLK_GATE_ABSTRACT_BITS_SHIFT((uint32_t)name));
}

	/*!
	 * @brief Disable the clock for specific IP.
	 *
	 * @param name  Which clock to disable, see \ref clock_ip_name_t.
	 */
static inline void CLOCK_DisableClock(clock_ip_name_t name)
{
	uint32_t regAddr = SIM_BASE + CLK_GATE_ABSTRACT_REG_OFFSET((uint32_t)name);
	(*(volatile uint32_t *)regAddr) &= ~(1UL << CLK_GATE_ABSTRACT_BITS_SHIFT((uint32_t)name));
}

/*!
 * @brief Enables the clock gate and configures the VREF module according to the configuration structure.
 *
 * This function must be called before calling all the other VREF driver functions,
 * read/write registers, and configurations with user-defined settings.
 * The example below shows how to set up  vref_config_t parameters and
 * how to call the VREF_Init function by passing in these parameters:
 * Example:
 * @code
 *   vref_config_t vrefConfig;
 *   vrefConfig.bufferMode = kVREF_ModeHighPowerBuffer;
 *   vrefConfig.enableExternalVoltRef = false;
 *   vrefConfig.enableLowRef = false;
 *   VREF_Init(VREF, &vrefConfig);
 * @endcode
 *
 * @param base VREF peripheral address.
 * @param config Pointer to the configuration structure.
 */
void VREF_Init(VREF_Type *base, const vref_config_t *config);

/*!
 * @brief Stops and disables the clock for the VREF module.
 *
 * This function should be called to shut down the module.
 * Example:
 * @code
 *   vref_config_t vrefUserConfig;
 *   VREF_Init(VREF);
 *   VREF_GetDefaultConfig(&vrefUserConfig);
 *   ...
 *   VREF_Deinit(VREF);
 * @endcode
 *
 * @param base VREF peripheral address.
 */
void VREF_Deinit(VREF_Type *base);

/*!
 * @brief Initializes the VREF configuration structure.
 *
 * This function initializes the VREF configuration structure to a default value.
 * Example:
 * @code
 *   vrefConfig->bufferMode = kVREF_ModeHighPowerBuffer;
 *   vrefConfig->enableExternalVoltRef = false;
 *   vrefConfig->enableLowRef = false;
 * @endcode
 *
 * @param config Pointer to the initialization structure.
 */
void VREF_GetDefaultConfig(vref_config_t *config);

/*!
 * @brief Sets a TRIM value for reference voltage.
 *
 * This function sets a TRIM value for reference voltage.
 * Note that the TRIM value maximum is 0x3F.
 *
 * @param base VREF peripheral address.
 * @param trimValue Value of the trim register to set the output reference voltage (maximum 0x3F (6-bit)).
 */
void VREF_SetTrimVal(VREF_Type *base, uint8_t trimValue);

/*!
 * @brief Reads the value of the TRIM meaning output voltage.
 *
 * This function gets the TRIM value from the TRM register.
 *
 * @param base VREF peripheral address.
 * @return Six-bit value of trim setting.
 */
static inline uint8_t VREF_GetTrimVal(VREF_Type *base)
{
    return (base->TRM & VREF_TRM_TRIM_MASK);
}
#if defined(FSL_FEATURE_VREF_HAS_LOW_REFERENCE) && FSL_FEATURE_VREF_HAS_LOW_REFERENCE

/*!
 * @brief Sets the TRIM value for low voltage reference.
 *
 * This function sets the TRIM value for low reference voltage.
 * NOTE:
 *      - The TRIM value maximum is 0x05U
 *      - The values 111b and 110b are not valid/allowed.
 *
 * @param base VREF peripheral address.
 * @param trimValue Value of the trim register to set output low reference voltage (maximum 0x05U (3-bit)).
 */
void VREF_SetLowReferenceTrimVal(VREF_Type *base, uint8_t trimValue);

/*!
 * @brief Reads the value of the TRIM meaning output voltage.
 *
 * This function gets the TRIM value from the VREFL_TRM register.
 *
 * @param base VREF peripheral address.
 * @return Three-bit value of the trim setting.
 */
static inline uint8_t VREF_GetLowReferenceTrimVal(VREF_Type *base)
{
    return (base->VREFL_TRM & VREF_VREFL_TRM_VREFL_TRIM_MASK);
}
#endif /* FSL_FEATURE_VREF_HAS_LOW_REFERENCE */

/*@}*/

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/*! @}*/

#endif /* _FSL_VREF_H_ */