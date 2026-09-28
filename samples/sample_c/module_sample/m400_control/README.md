# M400 control service

Public functions are declared in `m400_control_service.h`.

```c
#include "m400_control/m400_control_service.h"

M400Control_StartRecording();
M400Control_StopRecording();

M400Control_MoveRelative(M400_MOVE_FORWARD, 1.0f);
M400Control_YawRelative(90.0f);
T_M400WaypointMove move = {.forwardM = 5, .leftM = 0, .upM = 2,
                           .cruiseSpeedMps = 2, .safeTakeoffHeightM = 40, .finalYawDeg = 90};
M400Control_RunWaypointRelative(&move); // upload/start only; flight is asynchronous
T_M400WaypointTarget fixed = {.latitudeDeg = 22.602549522388877,
                              .longitudeDeg = 113.99172104792952,
                              .heightAboveTakeoffM = 20, .cruiseSpeedMps = 10,
                              .safeTakeoffHeightM = 40, .finalYawDeg = 90};
M400Control_RunWaypointCoordinate(&fixed); // absolute WGS-84 target, within 100 m
M400Control_EmergencyHover();
```

Legacy Joystick control calls are synchronous. Movement and yaw require the aircraft to be airborne and the RC to allow PSDK joystick
authority. Horizontal and vertical movement use position mode in ground coordinates; yaw uses rate mode. A joystick
authority callback cancels the active loop when RC pause or another controller takes authority. The next button press
requests authority again; FC refusal is reported, not overridden. Emergency hover interrupts a legacy Joystick command,
holds a stable zero-speed command for two seconds when authority can be obtained, then releases it. For an active
Waypoint 3 mission, it sends the mission STOP action instead; the aircraft response still needs flight verification.

The DJI Pilot widget queues control commands so widget callbacks do not block. The visible motion UI now uses a
two-point Waypoint 3 mission with an in-memory WPML KMZ. Signed metre offsets are relative to the aircraft heading
at button press; the final yaw is an absolute heading from north. It only starts when airborne. Safe takeoff height
is present in the KMZ but does not affect an airborne aircraft. Waypoint startup is synchronous; flight is not.
The two fixed-target Pilot buttons use the same Waypoint 3 implementation at 2 or 10 m/s, with a second press
within 10 seconds required to start. Their final yaw is the bearing toward the fixed target.
Recording writes a 50 Hz CSV file under `data/logs`; recording is off when the application starts.
