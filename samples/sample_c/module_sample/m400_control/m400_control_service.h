/**
 * @file m400_control_service.h
 * @brief M400 telemetry recorder, Pilot widget and reusable flight-control API.
 */

#ifndef M400_CONTROL_SERVICE_H
#define M400_CONTROL_SERVICE_H

#include <dji_typedef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    M400_MOVE_FORWARD = 0,
    M400_MOVE_BACKWARD,
    M400_MOVE_LEFT,
    M400_MOVE_RIGHT,
} E_M400MoveDirection;

/** Absolute destination for the position-mode controller. Angles are degrees. */
typedef struct {
    double latitudeDeg;
    double longitudeDeg;
    float heightAboveTakeoffM;
    float yawDeg;
    float horizontalToleranceM;
    float verticalToleranceM;
    float yawToleranceDeg;
    uint32_t timeoutMs;
} T_M400GotoTarget;

/** Waypoint 3 two-point move. Signed offsets are in the aircraft body frame at button press. */
typedef struct {
    float forwardM;             /* positive forward, [-10, 10] */
    float leftM;                /* positive left, [-10, 10] */
    float upM;                  /* positive up, [-10, 10] */
    float cruiseSpeedMps;      /* [1, 10] */
    float safeTakeoffHeightM;  /* WPML safe takeoff height; ignored when already airborne */
    float finalYawDeg;         /* absolute heading from north, [-180, 180] */
} T_M400WaypointMove;

/** Absolute WGS-84 destination for a two-point Waypoint 3 mission. */
typedef struct {
    double latitudeDeg;
    double longitudeDeg;
    float heightAboveTakeoffM;
    float cruiseSpeedMps;      /* [1, 10] */
    float safeTakeoffHeightM;  /* Required WPML field; ignored when already airborne */
    float finalYawDeg;         /* absolute heading from north, [-180, 180] */
} T_M400WaypointTarget;

/** Register the DJI Pilot widget. Call before DjiCore_ApplicationStart(). */
T_DjiReturnCode M400ControlWidget_Init(void);

/** Start telemetry subscriptions and worker tasks. Call after DjiCore_ApplicationStart(). */
T_DjiReturnCode M400Control_StartService(void);

/** Start/stop a 50 Hz CSV recording in data/logs. */
T_DjiReturnCode M400Control_StartRecording(void);
T_DjiReturnCode M400Control_StopRecording(void);

/**
 * Reusable synchronous control APIs.
 * The aircraft must already be airborne and the RC must be in P mode.
 */
/** Relative body-frame displacement, executed with horizontal/vertical position mode. */
T_DjiReturnCode M400Control_MoveRelative(E_M400MoveDirection direction, float distanceM);
/** Relative altitude change, executed with horizontal/vertical position mode. */
T_DjiReturnCode M400Control_MoveVertical(float deltaM);
/** Relative turn; yaw-rate command is limited to 30 deg/s and 10 deg/s^2. */
T_DjiReturnCode M400Control_YawRelative(float deltaDegree);
/**
 * Staged flight to WGS-84 coordinates: heading, altitude, then horizontal motion.
 * yawDeg is the desired heading for the first stage; callers may set it to the
 * bearing of the destination. This is not a Waypoint or FlyTo mission.
 * Choose tolerances for the available positioning quality (prefer RTK Fix for short moves).
 * Call from a worker thread; the emergency-hover command can interrupt it.
 */
T_DjiReturnCode M400Control_GotoCoordinate(const T_M400GotoTarget *target);
/** Staged goto with a per-axis horizontal position-offset cap in metres (0 = no clip, max 10). */
T_DjiReturnCode M400Control_GotoCoordinateWithLimit(const T_M400GotoTarget *target,
                                                   float horizontalPositionLimitM);
/** Builds a two-point WPML KMZ in memory, uploads it and starts Waypoint 3 asynchronously. */
T_DjiReturnCode M400Control_RunWaypointRelative(const T_M400WaypointMove *move);
/** Builds and starts a two-point mission to a WGS-84 target, within 100 m of current position. */
T_DjiReturnCode M400Control_RunWaypointCoordinate(const T_M400WaypointTarget *target);
T_DjiReturnCode M400Control_EmergencyHover(void);

#ifdef __cplusplus
}
#endif

#endif
