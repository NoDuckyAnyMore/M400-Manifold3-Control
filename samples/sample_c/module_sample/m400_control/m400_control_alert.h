#ifndef M400_CONTROL_ALERT_H
#define M400_CONTROL_ALERT_H

#include <dji_typedef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    M400_CONTROL_ALERT_STARTED = 0,
    M400_CONTROL_ALERT_INTERRUPTED,
    M400_CONTROL_ALERT_COMPLETED,
} E_M400ControlAlert;

/* Alert failures must never block or change flight-control decisions. */
T_DjiReturnCode M400ControlAlert_Init(void);
void M400ControlAlert_Queue(E_M400ControlAlert event);

#ifdef __cplusplus
}
#endif

#endif
