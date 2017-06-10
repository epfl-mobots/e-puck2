/**
 * @file    VL53L0X.c
 * @brief   High level functions to use the VL53L0X TOF sensor.
 *
 * @author  Eliot Ferragni
 */


#include "VL53L0X.h"
#include "Api/core/inc/vl53l0x_api.h"


//////////////////// PUBLIC FUNCTIONS /////////////////////////

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



