#include "ch.h"
#include "hal.h"
#include "sensors/mpu60X0.h"
#include "exti.h"

#define ACC_THRESHOLD   2.3
#define GYRO_THRESHOLD  0.5

static mpu60X0_t mpu6050;
static bool mpu_mode;


void mpu6050_read(float *gyro, float *acc)
{
    float temp;
    mpu60X0_read(&mpu6050, gyro, acc, &temp);
}


int mputest_mode(void)
{
    return mpu_mode;
}


// blink orange led
static THD_WORKING_AREA(mputest_thd_wa, 128);
static THD_FUNCTION(mputest_thd, arg) {

    (void)arg;
    chRegSetThreadName("MPUtest");
    while (TRUE) {
        if(palReadPad(GPIOA, 0)) {  //toggles mpu_mode when user button is pressed
            mpu_mode = !mpu_mode;

            palSetPad(GPIOD, 12);
            chThdSleepMilliseconds(500);

            palSetPad(GPIOD, 13);
            chThdSleepMilliseconds(500);

            palSetPad(GPIOD, 14);
            chThdSleepMilliseconds(500);

            palSetPad(GPIOD, 15);

            chThdSleepMilliseconds(500);

            palClearPad(GPIOD, 12);
            palClearPad(GPIOD, 13);
            palClearPad(GPIOD, 14);
            palClearPad(GPIOD, 15);

        }
        chThdSleepMilliseconds(400);
    }
    return 0;
}


static THD_WORKING_AREA(mpuled_thd_wa, 128);
static THD_FUNCTION(mpuled_thd, arg) {

    (void)arg;
    chRegSetThreadName("MPUled");
    static float buf_acc[3];   /* Last mpudata data.*/
    static float buf_gyro[3];
    while (TRUE) {
        //MPU reading
        mpu6050_read(buf_gyro, buf_acc);

        //LED activation
        if(mpu_mode) {
            //Horizontal LEDs
            if(buf_acc[0] >= ACC_THRESHOLD) {
                palSetPad(GPIOD, 12);
                palClearPad(GPIOD, 14);
            }
            else if(buf_acc[0] <= -ACC_THRESHOLD) {
                palSetPad(GPIOD, 14);
                palClearPad(GPIOD, 12);
            }
            else {
                palClearPad(GPIOD, 14);
                palClearPad(GPIOD, 12);
            }
            //Vertical LEDs
            if(buf_acc[1] >= ACC_THRESHOLD) {
                palSetPad(GPIOD, 15);
                palClearPad(GPIOD, 13);
            }
            else if(buf_acc[1] <= -ACC_THRESHOLD) {
                palSetPad(GPIOD, 13);
                palClearPad(GPIOD, 15);
            }
            else {
                palClearPad(GPIOD, 13);
                palClearPad(GPIOD, 15);
            }
        }
        else {
            //Horizontal LEDs
            if(buf_gyro[1] >= GYRO_THRESHOLD) {
                palSetPad(GPIOD, 14);
                palClearPad(GPIOD, 12);
            }
            else if(buf_gyro[1] <= -GYRO_THRESHOLD) {
                palSetPad(GPIOD, 12);
                palClearPad(GPIOD, 14);
            }
            else {
                palClearPad(GPIOD, 12);
                palClearPad(GPIOD, 14);
            }
            //Vertical LEDs
            if(buf_gyro[0] >= GYRO_THRESHOLD) {
                palSetPad(GPIOD, 13);
                palClearPad(GPIOD, 15);
            }
            else if(buf_gyro[0] <= -GYRO_THRESHOLD) {
                palSetPad(GPIOD, 15);
                palClearPad(GPIOD, 13);
            }
            else {
                palClearPad(GPIOD, 13);
                palClearPad(GPIOD, 15);
            }
        }
        chThdSleepMilliseconds(50);
    }
    return 0;
}

void imu_start(void)
{
    exti_setup();
    chThdCreateStatic(mputest_thd_wa, sizeof(mputest_thd_wa), NORMALPRIO, mputest_thd, NULL);
    chThdCreateStatic(mpuled_thd_wa, sizeof(mpuled_thd_wa), NORMALPRIO, mpuled_thd, NULL);
}


// PB9: I2C1_SDA (AF4)
// PB6: I2C1_SCL (AF4)
void imu_setup(void)
{
    static const I2CConfig i2c_cfg = {
        .op_mode = OPMODE_I2C,
        .clock_speed = 400000,
        .duty_cycle = FAST_DUTY_CYCLE_2
    };

    chSysLock();
    palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_OTYPE_OPENDRAIN);
    palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_OTYPE_OPENDRAIN);
    chSysUnlock();

    i2cStart(&I2CD1, &i2c_cfg);

    mpu60X0_init_using_i2c(&mpu6050, &I2CD1, 0);
    mpu60X0_setup(&mpu6050, MPU60X0_ACC_FULL_RANGE_2G
                          | MPU60X0_GYRO_FULL_RANGE_250DPS
                          | MPU60X0_SAMPLE_RATE_DIV(100)
                          | MPU60X0_LOW_PASS_FILTER_6);
}