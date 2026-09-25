# M400 control service

Public functions are declared in `m400_control_service.h`.

```c
#include "m400_control/m400_control_service.h"

M400Control_StartRecording();
M400Control_StopRecording();

M400Control_MoveRelative(M400_MOVE_FORWARD, 0.5f, 0.5f);
M400Control_YawRelative(10.0f);
M400Control_EmergencyHover();
```

Control calls are synchronous and serialized. Movement and yaw calls require the aircraft to be airborne and the RC
to be in P mode. Each normal command obtains joystick authority, sends a body-frame command, sends a stable zero-speed
command, and then returns authority to the RC. Emergency hover interrupts the active command, holds a stable zero-speed
command for two seconds, and then returns authority to the RC.

The DJI Pilot widget queues control commands so widget callbacks do not block. Recording writes a 50 Hz CSV file under
`data/logs`; recording is off when the application starts.
