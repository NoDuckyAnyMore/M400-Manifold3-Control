#include "m400_control_alert.h"

#include <dji_error.h>
#include <dji_hms_customization.h>
#include <dji_logger.h>
#include <dji_platform.h>

#include <stdbool.h>
#include <stdint.h>

static bool s_alertReady = false;
static uint32_t s_pendingAlerts[3] = {0};

void M400ControlAlert_Queue(E_M400ControlAlert event)
{
    if (!__atomic_load_n(&s_alertReady, __ATOMIC_ACQUIRE) ||
        event < M400_CONTROL_ALERT_STARTED || event > M400_CONTROL_ALERT_COMPLETED) return;
    /* Called from SDK callbacks too: this must not wait on a mutex or the PSDK root thread. */
    __atomic_add_fetch(&s_pendingAlerts[event], 1U, __ATOMIC_RELEASE);
}

static bool M400TakeAlert(E_M400ControlAlert *event)
{
    static const E_M400ControlAlert priority[] = {
        M400_CONTROL_ALERT_INTERRUPTED,
        M400_CONTROL_ALERT_COMPLETED,
        M400_CONTROL_ALERT_STARTED,
    };
    for (unsigned int i = 0; i < sizeof(priority) / sizeof(priority[0]); ++i) {
        uint32_t *pending = &s_pendingAlerts[priority[i]];
        uint32_t count = __atomic_load_n(pending, __ATOMIC_ACQUIRE);
        while (count > 0) {
            if (__atomic_compare_exchange_n(pending, &count, count - 1U, false,
                                            __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
                *event = priority[i];
                return true;
            }
        }
    }
    return false;
}

static void *M400AlertTask(void *arg)
{
    (void) arg;
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    while (true) {
        E_M400ControlAlert event;
        if (!M400TakeAlert(&event)) {
            osal->TaskSleepMs(20);
            continue;
        }
        T_DjiHmsAlarmEnhancedSetting setting = {
            .type = event == M400_CONTROL_ALERT_STARTED ?
                    DJI_HMS_ALARM_ENHANCED_PLAY_SOUND :
                    DJI_HMS_ALARM_ENHANCED_PLAY_SOUND_AND_SHAKE_MOTOR,
            .times = event == M400_CONTROL_ALERT_INTERRUPTED ? 3 : 1,
            .interval = event == M400_CONTROL_ALERT_INTERRUPTED ? 500 : 0,
        };
        T_DjiReturnCode rc = DjiHmsCustomization_AlarmEnhancedCtrl(
            DJI_HMS_ALARM_ENHANCED_ACTION_START, setting);
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_WARN("Pilot alert %d failed: 0x%08llX", event, (unsigned long long) rc);
        }
        /* Give the controller time to finish this pattern before sending another one. */
        osal->TaskSleepMs(event == M400_CONTROL_ALERT_INTERRUPTED ? 1600 : 300);
    }
    return NULL;
}

T_DjiReturnCode M400ControlAlert_Init(void)
{
    if (__atomic_load_n(&s_alertReady, __ATOMIC_ACQUIRE)) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    T_DjiReturnCode rc = DjiHmsCustomization_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        rc != DJI_ERROR_SYSTEM_MODULE_CODE_DUPLICATE) return rc;

    T_DjiTaskHandle alertTask;
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    if (osal->TaskCreate("m400_pilot_alert", M400AlertTask, 4096, NULL, &alertTask) !=
        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    __atomic_store_n(&s_alertReady, true, __ATOMIC_RELEASE);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
