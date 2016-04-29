#include <string.h>
#include <stdio.h>

#include "ch.h"
#include "hal.h"

#include "skel_user.h"

#include "vm/natives.h"
#include "discovery_demo/leds.h"
#include "common/productids.h"
#include "common/consts.h"
#include "main.h"
#include "config_flash_storage.h"

/* Struct used to share Aseba parameters between C-style API and Aseba. */
static parameter_namespace_t aseba_ns;
static parameter_t aseba_settings[SETTINGS_COUNT];
static char aseba_settings_name[SETTINGS_COUNT][10];

struct _vmVariables vmVariables;


const AsebaVMDescription vmDescription = {
	BOARD_NAME,
	{
		// {Number of element in array, Name displayed in aseba studio}
		{1, "_id"},
		{1, "event.source"},
		{VM_VARIABLES_ARG_SIZE, "event.args"},
        {2, "_fwversion"},
		{1, "_productId"},

        {6, "leds"},
        {3, "acc"},
        {SETTINGS_COUNT, "settings"},

		{0, NULL}
	}
};

// Event descriptions
const AsebaLocalEventDescription localEvents[] = {
    {"new_acc", "New accelerometer measurement"},
	{NULL, NULL}
};

void aseba_variables_init(AsebaVMState *vm)
{
    /* Initializes constant variables. */
    memset(&vmVariables, 0, sizeof(vmVariables));
    vmVariables.id = vm->nodeId;

    vmVariables.productId = ASEBA_PID_UNDEFINED;
    vmVariables.fwversion[0] = 0;
    vmVariables.fwversion[1] = 1;

    /* Registers all Aseba settings in global namespace. */
    int i;

    parameter_namespace_declare(&aseba_ns, &parameter_root, "aseba");

    /* Must be in descending order to keep them sorted on display. */
    for (i = SETTINGS_COUNT - 1; i >= 0; i--) {
        sprintf(aseba_settings_name[i], "%d", i);
        parameter_integer_declare_with_default(&aseba_settings[i],
                                               &aseba_ns,
                                               aseba_settings_name[i],
                                               0);
    }
}

void aseba_read_variables_from_system(AsebaVMState *vm)
{
    int i;
    ASEBA_UNUSED(vm);

    for (i = 0; i < SETTINGS_COUNT; i++) {
        vmVariables.settings[i] = parameter_integer_get(&aseba_settings[i]);
    }
}

void aseba_write_variables_to_system(AsebaVMState *vm)
{
    ASEBA_UNUSED(vm);
    int i;
    for (i = 3; i <= 6; i++) {
        demo_led_set(i, vmVariables.leds[i - 1]);
    }

    for (i = 0; i < SETTINGS_COUNT; i++) {
        parameter_integer_set(&aseba_settings[i], vmVariables.settings[i]);
    }
}


// Native functions
static AsebaNativeFunctionDescription AsebaNativeDescription__system_reboot =
{
    "_system.reboot",
    "Reboot the microcontroller",
    {
        {0,0}
    }
};

void AsebaNative__system_reboot(AsebaVMState *vm)
{
    ASEBA_UNUSED(vm);
    NVIC_SystemReset();
}

static AsebaNativeFunctionDescription AsebaNativeDescription_settings_save =
{
    "settings.save",
    "Save settings into flash",
    {
        {0,0}
    }
};

void AsebaNative_settings_save(AsebaVMState *vm)
{
    extern uint32_t _config_start, _config_end;
    size_t len = (size_t)(&_config_end - &_config_start);
    bool success;

    // First write the config to flash
    config_save(&_config_start, len, &parameter_root);

    // Second try to read it back, see if we failed
    success = config_load(&parameter_root, &_config_start, len);

    if (!success) {
        AsebaVMEmitNodeSpecificError(vm, "Config save failed!");
    }
}

static AsebaNativeFunctionDescription AsebaNativeDescription_settings_erase =
{
    "settings.erase",
    "Restore settings to default value (erases flash)",
    {
        {0,0}
    }
};


void AsebaNative_settings_erase(AsebaVMState *vm)
{
    ASEBA_UNUSED(vm);

    extern uint32_t _config_start;

    config_erase(&_config_start);
}

AsebaNativeFunctionDescription AsebaNativeDescription_clear_all_leds = {
    "leds.clear_all",
    "Clear all the LEDs",
    {
        {0, 0}
    }
};


void clear_all_leds(AsebaVMState *vm)
{
    ASEBA_UNUSED(vm);
    int i;

    for(i = 3; i <= 6; i++) {
        demo_led_set(i, 0);
        vmVariables.leds[i - 1] = 0;
    }
}



// Native function descriptions
const AsebaNativeFunctionDescription* nativeFunctionsDescription[] = {
	&AsebaNativeDescription__system_reboot,
	&AsebaNativeDescription_settings_save,
	&AsebaNativeDescription_settings_erase,
    &AsebaNativeDescription_clear_all_leds,
    ASEBA_NATIVES_STD_DESCRIPTIONS,
    0
};

// Native function pointers
AsebaNativeFunctionPointer nativeFunctions[] = {
    AsebaNative__system_reboot,
    AsebaNative_settings_save,
    AsebaNative_settings_erase,
    clear_all_leds,
	ASEBA_NATIVES_STD_FUNCTIONS,
};

const int nativeFunctions_length = sizeof(nativeFunctions) / sizeof(nativeFunctions[0]);
