/**
 * @file    VL53L0X.c
 * @brief   High level functions to use the VL53L0X TOF sensor.
 *
 * @author  Eliot Ferragni
 */

#include "ch.h"
#include "hal.h"
#include "VL53L0X.h"
#include "Api/core/inc/vl53l0x_api.h"
#include "../LED_RGB/led_rgb.h"
#include "shell.h"
#include "chprintf.h"
#include "usbcfg.h"


//////////////////// PUBLIC FUNCTIONS /////////////////////////
static THD_WORKING_AREA(waVL53L0XThd, 2048);
static THD_FUNCTION(VL53L0XThd, arg) {

	(void)arg;
	uint8_t i = 0;
	uint16_t value[3];
	uint8_t deviceAddr[3] = {VL53L0X_1_DEV_CARD_ADDR, VL53L0X_2_DEV_CARD_ADDR, VL53L0X_3_DEV_CARD_ADDR};
	uint8_t ledAddr[3] = {LED1_DEV_CARD_ADDR, LED2_DEV_CARD_ADDR, LED3_DEV_CARD_ADDR};


	VL53L0X_Dev_t device[3];

    for(i = 0 ; i < 3 ; i++){

    	device[i].I2cDevAddr = deviceAddr[i];
    	VL53L0X_init(&device[i]);
    	VL53L0X_configAccuracy(&device[i], VL53L0X_LONG_RANGE);
    	VL53L0X_startMeasure(&device[i], VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    	led_rgb_set_intensity_all(ledAddr[i], 0, 0, 0);
    	led_rgb_set_current(ledAddr[i], LED_RGB_MAX_VALUE);
    }


    /* Reader thread loop.*/
    while (TRUE) {
    	for(i = 0 ; i < 3 ; i++){

    		VL53L0X_getLastMeasure(&device[i]);

	    	value[i] = device[i].Data.LastRangeMeasure.RangeMilliMeter;
	    
	    	led_rgb_set_intensity(ledAddr[i], LED_RGB_GREEN, value[i]>>5);
	    	
    	}
		//chprintf((BaseSequentialStream *)&SDU1, "capteur 1 = %d    capteur 2 = %d      capteur 3 = %d\n" , value[0], value[1], value[2]);
		chprintf((BaseSequentialStream *)&SDU1, "%d\n" , value[2]);

    	
    	chThdSleepMilliseconds(100);
    }
}


VL53L0X_Error VL53L0X_init(VL53L0X_Dev_t* device){

	VL53L0X_Error status = VL53L0X_ERROR_NONE;

	uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;

//init
	if(status == VL53L0X_ERROR_NONE)
    {
    	// Structure and device initialisation
        status = VL53L0X_DataInit(device);
    }

    if(status == VL53L0X_ERROR_NONE)
    {
    	// Get device info
        status = VL53L0X_GetDeviceInfo(device, &(device->DeviceInfo));
    }

    if(status == VL53L0X_ERROR_NONE)
    {
    	// Device Initialization
        status = VL53L0X_StaticInit(device);
    }

//calibration
 	if(status == VL53L0X_ERROR_NONE)
    {
    	// SPAD calibration
        status = VL53L0X_PerformRefSpadManagement(device,
        		&refSpadCount, &isApertureSpads);
    }

    if(status == VL53L0X_ERROR_NONE)
    {
    	// Calibration
        status = VL53L0X_PerformRefCalibration(device,
        		&VhvSettings, &PhaseCal);
    }

    return status;
}

VL53L0X_Error VL53L0X_configAccuracy(VL53L0X_Dev_t* device, VL53L0X_AccuracyMode accuracy){

	VL53L0X_Error status = VL53L0X_ERROR_NONE;

//Activation Limits
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetLimitCheckEnable(device,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetLimitCheckEnable(device,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }

//specific accuracy config
//copied from ST example and API Guide
    switch(accuracy){

    	case VL53L0X_DEFAULT_MODE:
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetLimitCheckEnable(device,
		        		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
		    }

		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetLimitCheckValue(device,
		        		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
		        		(FixPoint1616_t)(1.5*0.023*65536));
		    }
		    break;

		case VL53L0X_HIGH_ACCURACY:
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetLimitCheckValue(device,
		        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
		        		(FixPoint1616_t)(0.25*65536));
			}
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetLimitCheckValue(device,
		        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
		        		(FixPoint1616_t)(18*65536));
		    }
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(device,
		        		200000);
		    }
			break;

		case VL53L0X_LONG_RANGE:
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetLimitCheckValue(device,
		        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
		        		(FixPoint1616_t)(0.1*65536));
			}
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetLimitCheckValue(device,
		        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
		        		(FixPoint1616_t)(60*65536));
		    }
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(device,
		        		33000);
			}
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetVcselPulsePeriod(device,
				        VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
		    }
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetVcselPulsePeriod(device,
				        VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
		    }
			break;

		case VL53L0X_HIGH_SPEED:
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetLimitCheckValue(device,
		        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
		        		(FixPoint1616_t)(0.25*65536));
			}
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetLimitCheckValue(device,
		        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
		        		(FixPoint1616_t)(32*65536));
		    }
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(device,
		        		30000);
		    }
			break;
		default:
			status = VL53L0X_ERROR_INVALID_PARAMS;
			break;
    }

    return status;
}

VL53L0X_Error VL53L0X_startMeasure(VL53L0X_Dev_t* device, VL53L0X_DeviceModes mode){

	VL53L0X_Error status = VL53L0X_ERROR_NONE;

	status = VL53L0X_SetDeviceMode(device, mode);

	if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_StartMeasurement(device);
    }

    return status;
}

VL53L0X_Error VL53L0X_getLastMeasure(VL53L0X_Dev_t* device){
	return VL53L0X_GetRangingMeasurementData(device,
			&(device->Data.LastRangeMeasure));
}

VL53L0X_Error VL53L0X_stopMeasure(VL53L0X_Dev_t* device){
	return VL53L0X_StopMeasurement(device);
}

void VL53L0X_init_demo(void){
	chThdCreateStatic(waVL53L0XThd,
                     sizeof(waVL53L0XThd),
                     NORMALPRIO + 10,
                     VL53L0XThd,
                     NULL);
}

