#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ch.h"
#include "hal.h"
#include "test.h"
#include "chprintf.h"
#include "shell.h"
#include "usbcfg.h"
#include "cmd.h"
#include "malloc_lock.h"

#include "discovery_demo/accelerometer.h"
#include "discovery_demo/leds.h"

#include "aseba_vm/skel.h"
#include "aseba_vm/aseba_node.h"
#include "aseba_vm/aseba_can_interface.h"

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)


int main(void)
{
    thread_t *shelltp = NULL;

    halInit();
    chSysInit();
    malloc_lock_init();

    // UART2 on PA2(TX) and PA3(RX)
    sdStart(&SD2, NULL);
    palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
    palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

    // serial-over-USB CDC driver.
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1000);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    // Initialise Discovery board demo setup
//    demo_acc_start();
    demo_led_init();

    // Initialise Aseba CAN and VM
    aseba_vm_init();
    aseba_can_start(&vmState);
    aseba_vm_start();

    demo_acc_start(&accelerometer_cb);

    shellInit();

    static const ShellConfig shell_cfg1 = {
        (BaseSequentialStream *)&SDU1,
        shell_commands
    };

    while (TRUE) {
        if (!shelltp) {
            if (SDU1.config->usbp->state == USB_ACTIVE) {
                shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
            }
        } else {
            if (chThdTerminatedX(shelltp)) {
                chThdRelease(shelltp);
                shelltp = NULL;
            }
        }
        chThdSleepMilliseconds(500);
    }

}
