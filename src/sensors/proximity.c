#include "proximity.h"

#include "ch.h"
#include "hal.h"
#include <stdio.h>
#include <string.h>

#include "../main.h"

// The proximity sensors sampling is designed in order to sample two sensors at one time, the couples are chosen
// in order to have as less interference as possible and divided as follow:
// - IR0 (front-right) + IR4 (back-left)
// - IR1 (front-right-45deg) + IR5 (left)
// - IR2 (right) + IR6 (front-left-45deg)
// - IR3 (back-right) + IR7 (front-left)
// A timer is used to handle the sampling (pwm mode), one channel is used to handle the pulse and a second channel
// is used to trigger the sampling at the right time.
// In each pwm period a single couple of sensors is sampled, either with the pulse active to measure the
// "reflected light" or with the pulse inactive to measure the ambient light; the sequence is:
// - cycle0: IR0 + IR4 ambient
// - cycle1: IR0 + IR4 reflected
// - cycle2: IR1 + IR5 ambient
// - cycle3: IR1 + IR5 reflected
// - cycle4: IR2 + IR6 ambient
// - cycle5: IR2 + IR6 reflected
// - cycle6: IR3 + IR7 ambient
// - cycle7: IR3 + IR7 reflected
// The pwm frequency is 800 Hz, the resulting update frequency for the sensors is 100 Hz (8 cycles to get all the sensors).
// The pulse is 300 us, the sampling is done at 260 us, this give time to sample both channels when the pulse is still
// active. Each sampling total time takes about 11.8 us:
// - ADC clock div = 8 => APB2/8 = 84/8 = 10.5 MHz
// - [sampling (112 cycles) + conversion (12 cycles)] x 1'000'000/10'500'000 = about 11.81 us

#define PWM_CLK_FREQ 1000000
#define PWM_FREQUENCY 800
#define PWM_CYCLE (PWM_CLK_FREQ / PWM_FREQUENCY)
/* Max duty cycle is 0.071, 2x safety margin. */
#define TCRT1000_DC 0.24 // 0.24*1000/PWM_FREQUENCY=0.3 ms
#define ON_MEASUREMENT_POS 0.208 // 0.208*1000/PWM_FREQUENCY=0.26 ms
//#define OFF_MEASUREMENT_POS 0.5 // 0.5*1000/PWM_FREQUENCY=1.25 ms
#define NUM_IR_SENSORS 8

#define PROXIMITY_ADC_SAMPLE_TIME ADC_SAMPLE_112
#define DMA_BUFFER_SIZE 1 // 1 sample for each ADC channel

#define EXTSEL_TIM5_CH1 0x0a
#define EXTSEL_TIM5_CH3 0x0c

static unsigned int adc2_values[PROXIMITY_NB_CHANNELS*2] = {0};
static BSEMAPHORE_DECL(adc2_ready, true);
static adcsample_t adc2_proximity_samples[PROXIMITY_NB_CHANNELS*2 * DMA_BUFFER_SIZE];
uint8_t pulseSeqState = 0;
uint8_t calibrationInProgress = 0;
uint8_t calibrationState = 0;
uint8_t calibrationNumSamples = 0;
int32_t calibrationSum[PROXIMITY_NB_CHANNELS] = {0};
proximity_msg_t proxMsg;

void calibrate_ir(void) {
	calibrationState = 0;
	calibrationInProgress = 1;
	while(calibrationInProgress) {
		chThdSleepMilliseconds(50);
	}
}

int get_prox(unsigned int sensor_number) {
	if (sensor_number > 7) {
		return 0;
	} else {
		return proxMsg.delta[sensor_number];
	}
}

int get_calibrated_prox(unsigned int sensor_number) {
	int temp;
	if (sensor_number > 7) {
		return 0;
	} else {
		temp = proxMsg.delta[sensor_number] - proxMsg.initValue[sensor_number];
		if (temp>0) {
			return temp;
		} else {
			return 0;
		}
	}
}

int get_ambient_light(unsigned int sensor_number) {
	if (sensor_number > 7) {
		return 0;
	} else {
		return proxMsg.ambient[sensor_number];
	}
}

static void adc_cb(ADCDriver *adcp, adcsample_t *samples, size_t n)
{
    (void) adcp;
    (void) samples;
    (void) n;

    unsigned int *values = NULL;
    binary_semaphore_t *sem = NULL;
	
	values = adc2_values;
	sem = &adc2_ready;

    /* Reset all samples to zero. */
    memset(values, 0, adcp->grpp->num_channels * sizeof(unsigned int));

    /* Compute the average over the different measurements. */
//    for (size_t j = 0; j < n; j++) {
//        for (size_t i = 0; i < adcp->grpp->num_channels; i++) {
//            values[i] += samples[adcp->grpp->num_channels * j + i];
//        }
//    }
//    for (size_t i = 0; i < adcp->grpp->num_channels; i++) {
//        values[i] /= n;
//    }

    // Finally only one value is sampled for each channel, so it doens't need anymore to average the measurements.
    // Thus simply copy the values to the destination buffer.
    for (size_t i = 0; i < adcp->grpp->num_channels; i++) {
    	values[i] = samples[i];
    }
    //memcpy(values, samples, adcp->grpp->num_channels * sizeof(unsigned int));

    /* Signal the proximity thread that the ADC measurements are done. */
    chSysLockFromISR();
    chBSemSignalI(sem);
    chSysUnlockFromISR();

    pulseSeqState = 1; // Sync with the timer since the first time we get here the ADC and timer could be desync.
}

static const ADCConversionGroup adcgrpcfg2 = {
    .circular = true,
    .num_channels = PROXIMITY_NB_CHANNELS*2, // Both ambient and reflected measures are saved before raising the DMA interrupt (and call the adc callback).
    .end_cb = adc_cb,
    .error_cb = NULL,

    // Discontinuous mode with 2 conversions per trigger.
	// Every time the sampling is triggered by the timer, two channels are sampled:
	// - trigger1: IR0 + IR4 sampling
	// - trigger2: IR1 + IR5 sampling
	// - trigger3: IR2 + IR6 sampling
	// - trigger4: IR3 + IR7 sampling
    .cr1 = ADC_CR1_DISCEN | ADC_CR1_DISCNUM_0,

    /* External trigger on timer 5 CC1. */
    .cr2 = ADC_CR2_EXTEN_1 | ADC_CR2_EXTSEL_SRC(EXTSEL_TIM5_CH1),

    /* Sampling duration, all set to PROXIMITY_ADC_SAMPLE_TIME. */
    .smpr2 = ADC_SMPR2_SMP_AN0(PROXIMITY_ADC_SAMPLE_TIME) |
             ADC_SMPR2_SMP_AN1(PROXIMITY_ADC_SAMPLE_TIME) |
             ADC_SMPR2_SMP_AN2(PROXIMITY_ADC_SAMPLE_TIME) |
             ADC_SMPR2_SMP_AN3(PROXIMITY_ADC_SAMPLE_TIME) |
             ADC_SMPR2_SMP_AN4(PROXIMITY_ADC_SAMPLE_TIME) |
             ADC_SMPR2_SMP_AN5(PROXIMITY_ADC_SAMPLE_TIME) |
             ADC_SMPR2_SMP_AN6(PROXIMITY_ADC_SAMPLE_TIME) |
             ADC_SMPR2_SMP_AN7(PROXIMITY_ADC_SAMPLE_TIME) |
             ADC_SMPR2_SMP_AN8(PROXIMITY_ADC_SAMPLE_TIME) |
             ADC_SMPR2_SMP_AN9(PROXIMITY_ADC_SAMPLE_TIME),
    .smpr1 = ADC_SMPR1_SMP_AN10(PROXIMITY_ADC_SAMPLE_TIME) |
             ADC_SMPR1_SMP_AN11(PROXIMITY_ADC_SAMPLE_TIME) |
             ADC_SMPR1_SMP_AN12(PROXIMITY_ADC_SAMPLE_TIME) |
             ADC_SMPR1_SMP_AN13(PROXIMITY_ADC_SAMPLE_TIME) |
             ADC_SMPR1_SMP_AN14(PROXIMITY_ADC_SAMPLE_TIME) |
             ADC_SMPR1_SMP_AN15(PROXIMITY_ADC_SAMPLE_TIME),

    // Channels are defined starting from front-right sensor and turning clock wise.
	// Proximity sensors channels:
	// IR0 = ADC123_IN12 (PC2)
	// IR1 = ADC123_IN13 (PC3)
	// IR2 = ADC12_IN14 (PC4)
	// IR3 = ADC12_IN15 (PC5)
	// IR4 = ADC12_IN9 (PB1)
	// IR5 = ADC12_IN8 (PB0)
	// IR6 = ADC123_IN10 (PC0)
	// IR7 = ADC123_IN11 (PC1)
	// A single ADC peripheral can be used for all channels => ADC1 or ADC2.
    .sqr3 = ADC_SQR3_SQ1_N(12) |
            ADC_SQR3_SQ2_N(9) |
			ADC_SQR3_SQ3_N(12) |
			ADC_SQR3_SQ4_N(9) |
            ADC_SQR3_SQ5_N(13) |
            ADC_SQR3_SQ6_N(8),
    .sqr2 = ADC_SQR2_SQ7_N(13) |
            ADC_SQR2_SQ8_N(8) |
			ADC_SQR2_SQ9_N(14) |
			ADC_SQR2_SQ10_N(10) |
			ADC_SQR2_SQ11_N(14) |
			ADC_SQR2_SQ12_N(10),
    .sqr1 = ADC_SQR1_SQ13_N(15) |
			ADC_SQR1_SQ14_N(11) |
			ADC_SQR1_SQ15_N(15) |
			ADC_SQR1_SQ16_N(11) |
			ADC_SQR1_NUM_CH(PROXIMITY_NB_CHANNELS*2)
};

static THD_FUNCTION(proximity_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    proximity_msg_t proxMsgTopic;

    messagebus_topic_t proximity_topic;
    MUTEX_DECL(prox_topic_lock);
    CONDVAR_DECL(prox_topic_condvar);
    messagebus_topic_init(&proximity_topic, &prox_topic_lock, &prox_topic_condvar, &proxMsgTopic, sizeof(proxMsgTopic));
    messagebus_advertise_topic(&bus, &proximity_topic, "/proximity");

    while (true) {

    	chBSemWait(&adc2_ready);

        proxMsg.ambient[0] = adc2_values[0];
        proxMsg.ambient[1] = adc2_values[4];
        proxMsg.ambient[2] = adc2_values[8];
        proxMsg.ambient[3] = adc2_values[12];
        proxMsg.ambient[4] = adc2_values[1];
        proxMsg.ambient[5] = adc2_values[5];
        proxMsg.ambient[6] = adc2_values[9];
        proxMsg.ambient[7] = adc2_values[13];

        proxMsg.reflected[0] = adc2_values[2];
        proxMsg.reflected[1] = adc2_values[6];
        proxMsg.reflected[2] = adc2_values[10];
        proxMsg.reflected[3] = adc2_values[14];
        proxMsg.reflected[4] = adc2_values[3];
        proxMsg.reflected[5] = adc2_values[7];
        proxMsg.reflected[6] = adc2_values[11];
        proxMsg.reflected[7] = adc2_values[15];

        for (int i = 0; i < PROXIMITY_NB_CHANNELS; i++) {
        	proxMsg.delta[i] = proxMsg.ambient[i] - proxMsg.reflected[i];
        }

        messagebus_topic_publish(&proximity_topic, &proxMsg, sizeof(proxMsg));

        if(calibrationInProgress) {
        	switch(calibrationState) {
				case 0:
					memset(proxMsg.initValue, 0, PROXIMITY_NB_CHANNELS * sizeof(unsigned int));
					memset(calibrationSum, 0, PROXIMITY_NB_CHANNELS * sizeof(int32_t));
					calibrationNumSamples = 0;
					calibrationState = 1;
					break;

				case 1:
					for(int i=0; i<PROXIMITY_NB_CHANNELS; i++) {
						calibrationSum[i] += get_prox(i);
					}
					calibrationNumSamples++;
					if(calibrationNumSamples == 100) {
						for(int i=0; i<PROXIMITY_NB_CHANNELS; i++) {
							proxMsg.initValue[i] = calibrationSum[i]/100;
						}
						calibrationInProgress = 0;
					}
					break;
        	}
        }

    }
}

static void pwm_reset_cb(PWMDriver *pwmp) {
	(void)pwmp;
	switch(pulseSeqState) {
		case 0:
			break;
		case 1:
			pulseSeqState = 2;
			break;
		case 2:
			palSetPad(GPIOB, GPIOB_PULSE_0);
			pulseSeqState = 3;
			break;
		case 3:
			pulseSeqState = 4;
			break;
		case 4:
			palSetPad(GPIOB, GPIOB_PULSE_1);
			pulseSeqState = 5;
			break;
		case 5:
			pulseSeqState = 6;
			break;
		case 6:
			palSetPad(GPIOE, GPIOE_PULSE_2);
			pulseSeqState = 7;
			break;
		case 7:
			pulseSeqState = 8;
			break;
		case 8:
			palSetPad(GPIOE, GPIOE_PULSE_3);
			pulseSeqState = 1;
			break;
	}
}

static void pwm_ch2_cb(PWMDriver *pwmp) {
	(void)pwmp;
	// Clear all the pulse independently of the one that is actually active.
	palClearPad(GPIOB, GPIOB_PULSE_0);
	palClearPad(GPIOB, GPIOB_PULSE_1);
	palClearPad(GPIOE, GPIOE_PULSE_2);
	palClearPad(GPIOE, GPIOE_PULSE_3);
}

void proximity_start(void)
{
    static const PWMConfig pwmcfg_proximity = {
        /* timer clock frequency */
        .frequency = PWM_CLK_FREQ,
        /* timer period */
        .period = PWM_CYCLE,
        .cr2 = 0,

        /* Enable DMA event generation on channel 1. */
        .dier = TIM_DIER_CC1DE,
        .callback = pwm_reset_cb,
        .channels = {
            // Channel 1 is used to generate ADC trigger for starting sampling for both "pulse active" and ambient measures.
        	// It must be in output mode, although it is not routed to any pin.
            {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = NULL},
            /* Channel 2 is used to generate TCRT1000 drive signals. */
            {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = pwm_ch2_cb},
            {.mode = PWM_OUTPUT_DISABLED, .callback = NULL},
            {.mode = PWM_OUTPUT_DISABLED, .callback = NULL},
        },
    };
	
    adcAcquireBus(&ADCD2);
    adcStartConversion(&ADCD2, &adcgrpcfg2, adc2_proximity_samples, DMA_BUFFER_SIZE); // ADC waiting for the trigger from the timer.

    /* Init PWM */
    pwmStart(&PWMD5, &pwmcfg_proximity);

    pwmEnableChannel(&PWMD5, 1, (pwmcnt_t) (PWM_CYCLE * TCRT1000_DC)); // Enable channel 2 to set duty cycle for TCRT1000 drivers.
	pwmEnableChannelNotification(&PWMD5, 1); // Channel 2 interrupt enable to handle pulse shutdown.
    pwmEnablePeriodicNotification(&PWMD5); // PWM general interrupt at the beginning of the period to handle pulse ignition.
    pwmEnableChannel(&PWMD5, 0, (pwmcnt_t) (PWM_CYCLE * ON_MEASUREMENT_POS)); // Enable channel 1 to trigger the measures.

    static THD_WORKING_AREA(proximity_thd_wa, 2048);
    chThdCreateStatic(proximity_thd_wa, sizeof(proximity_thd_wa), NORMALPRIO, proximity_thd, NULL);
	
}


