// This file has been prepared for Doxygen automatic documentation generation.
/*! \file *********************************************************************
 *
 * \brief  Interface for AVRRAVEN's NTC.
 *
 * \ingroup lcdraven
 *
 * The AVRRAVEN has an onboard NTC resistor that can be used to measure
 * temperature.
 *
 * \par Application note:
 *      AVR2017: RZRAVEN FW
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Id: temp.h,v 1.1 2009/05/20 20:52:01 mvidales Exp $
 *
 * Copyright (c) 2008 , Atmel Corporation. All rights reserved.
 *
 * Licensed under Atmel’s Limited License Agreement (RZRaven Evaluation and Starter Kit).
 *****************************************************************************/

#ifndef __TEMP_H__
#define __TEMP_H__
/*========================= INCLUDES                 =========================*/


#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "adc.h"

#define PROGMEM_DECLARE(x) x __attribute__((__progmem__))

//! \addtogroup lcd
//! @{
/*========================= MACROS                   =========================*/


/*========================= TYEPDEFS                 =========================*/
//! Type used with \ref temp_get() to select temperature unit
typedef enum {
    TEMP_UNIT_CELCIUS,
    TEMP_UNIT_FAHRENHEIT
} temp_unit_t;

/// @name Definition of Port Pin for temp sensor.
/// @{
#define TEMP_PORT       PORTF
#define TEMP_DDR        DDRF
#define TEMP_PIN        PINF
#define TEMP_BIT_IN     4
#define TEMP_BIT_PWR    6
/// @}

/*========================= PUBLIC VARIABLES         =========================*/

/*========================= PUBLIC FUNCTIONS         =========================*/
/*! \brief          Initialize the temperature sensor
 *
 * \return                              EOF on error
 */
int temp_init(void);


/*! \brief          De-initialize the temperature sensor
 *
 */
void temp_deinit(void);

/*! \brief          Read current temperature
 *
 * \param[in]		unit			Selected temperature unit (\ref temp_unit_t)
 *
 *
 * \return                              EOF on error
 */
int16_t temp_get(temp_unit_t unit);
//! @}
#endif // __TEMP_H__
/*EOF*/
