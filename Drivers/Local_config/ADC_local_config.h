/*
 * ADC_local_config.h
 *
 *  Created on: Oct 25, 2022
 *      Author: Bazhen
 */

#ifndef LOCAL_CONFIG_ADC_LOCAL_CONFIG_H_
#define LOCAL_CONFIG_ADC_LOCAL_CONFIG_H_

	#include "B12vc_local_config.h"

#ifdef ADC_EN
	#define 	ADC_MODULE
	#define 	VOLT_CHANNEL		ADC_CHANNEL_1
	#define 	AMPER_CHANNEL		ADC_CHANNEL_2
	#define		VOLT_COEFFICIENT	2285
#endif


#endif /* LOCAL_CONFIG_ADC_LOCAL_CONFIG_H_ */
