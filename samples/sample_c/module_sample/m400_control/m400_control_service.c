/**
 * @file m400_control_service.c
 * @brief M400 telemetry recorder, Pilot widget and reusable flight-control API.
 */

#include "m400_control_service.h"
#include "m400_pi4_bridge.h"

#include <dji_fc_subscription.h>
#include <dji_flight_controller.h>
#include <dji_logger.h>
#include <dji_platform.h>
#include <dji_widget.h>

#include <dirent.h>
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define M400_LOG_DIRECTORY                 "data/logs"
#define M400_PI4_LISTEN_INTERFACE           "eth0"
#define M400_PI4_ALLOWED_PEER               "10.88.77.1"
#define M400_PI4_LISTEN_PORT                14560
#define M400_WIDGET_EN_DIRECTORY           "widget/en_big_screen"
#define M400_WIDGET_CN_DIRECTORY           "widget/cn_big_screen"
#define M400_LOG_FREQUENCY_HZ              50
#define M400_LOG_PERIOD_NS                 (1000000000L / M400_LOG_FREQUENCY_HZ)
#define M400_CONTROL_PERIOD_MS              20
#define M400_HOVER_HOLD_MS                  2000
#define M400_MAX_CSV_FILES                 5
#define M400_FILENAME_LENGTH               128
#define M400_RESULT_LENGTH                 96
#define M400_DEG_PER_RAD                   57.29577951308232
#define M400_PI                            3.14159265358979323846
#define M400_EARTH_RADIUS_M                6378137.0
#define M400_POSITION_COMMAND_LIMIT_M      2.0f
#define M400_POSITION_COMMAND_TEST_LIMIT_M 10.0f
#define M400_POSITION_SPEED_TOL_MPS        0.2f
#define M400_POSITION_HOLD_MS              500
#define M400_POSITION_STALE_MS             500
#define M400_MIN_VISIBLE_SATELLITES        8
#define M400_YAW_MAX_RATE_DEG_S             30.0f
#define M400_YAW_ACCEL_DEG_S2               10.0f
#define M400_YAW_TIMEOUT_MS                20000U
#define M400_YAW_POSITION_TOL_DEG           1.5f
#define M400_YAW_RATE_TOL_DEG_S             2.0f
#define M400_YAW_SETTLE_HOLD_MS             400U
#define M400_VERTICAL_SETPOINT_RATE_MPS     1.0f
#define M400_HORIZONTAL_SETPOINT_RATE_MPS   1.0f
#define M400_HORIZONTAL_SETPOINT_ACCEL_MPS2 0.5f
#define M400_GOTO_MAX_DISTANCE_M            100.0
#define M400_GOTO_CONFIRM_MS                10000U
#define M400_TEST_LATITUDE_DEG              22.602549522388877
#define M400_TEST_LONGITUDE_DEG             113.99172104792952
#define M400_TEST_HEIGHT_M                  20.0f
#define M400_WIDGET_COUNT                   23U

// Pilot 2 displays config widgets by widget_index, not by JSON array order.
enum {
    M400_WIDGET_START_RECORD = 0,
    M400_WIDGET_STOP_RECORD = 1,
    M400_WIDGET_RECORD_SWITCH = 2,
    M400_WIDGET_RTK = 3,
    M400_WIDGET_LONGITUDE = 4,
    M400_WIDGET_LATITUDE = 5,
    M400_WIDGET_COMMAND_STATE = 6,
    M400_WIDGET_EMERGENCY_HOVER = 7,
    M400_WIDGET_FORWARD_1M = 8,
    M400_WIDGET_BACKWARD_1M = 9,
    M400_WIDGET_LEFT_1M = 10,
    M400_WIDGET_RIGHT_1M = 11,
    M400_WIDGET_UP_1M = 12,
    M400_WIDGET_DOWN_1M = 13,
    M400_WIDGET_FORWARD_5M = 14,
    M400_WIDGET_YAW_LEFT = 15,
    M400_WIDGET_YAW_RIGHT = 16,
    M400_WIDGET_GOTO_STAGED_2 = 17,
    M400_WIDGET_GOTO_STAGED_10 = 18,
    M400_WIDGET_GOTO_OFFICIAL_2 = 19,
    M400_WIDGET_CSV_SELECT = 20,
    M400_WIDGET_CSV_DATE = 21,
    M400_WIDGET_CSV_TIME = 22,
};

static const char *const s_widgetEventNames[M400_WIDGET_COUNT] = {
    [M400_WIDGET_START_RECORD] = "start_record",
    [M400_WIDGET_STOP_RECORD] = "stop_record",
    [M400_WIDGET_RECORD_SWITCH] = "record_switch",
    [M400_WIDGET_EMERGENCY_HOVER] = "emergency_hover",
    [M400_WIDGET_FORWARD_1M] = "forward_1m",
    [M400_WIDGET_BACKWARD_1M] = "backward_1m",
    [M400_WIDGET_LEFT_1M] = "left_1m",
    [M400_WIDGET_RIGHT_1M] = "right_1m",
    [M400_WIDGET_UP_1M] = "up_1m",
    [M400_WIDGET_DOWN_1M] = "down_1m",
    [M400_WIDGET_FORWARD_5M] = "forward_5m",
    [M400_WIDGET_YAW_LEFT] = "yaw_left",
    [M400_WIDGET_YAW_RIGHT] = "yaw_right",
    [M400_WIDGET_GOTO_STAGED_2] = "goto_staged_f2",
    [M400_WIDGET_GOTO_STAGED_10] = "goto_staged_f10",
    [M400_WIDGET_GOTO_OFFICIAL_2] = "goto_official_f2",
    [M400_WIDGET_CSV_SELECT] = "csv_select",
};

typedef enum {
    M400_COMMAND_NONE = 0,
    M400_COMMAND_FORWARD,
    M400_COMMAND_BACKWARD,
    M400_COMMAND_LEFT,
    M400_COMMAND_RIGHT,
    M400_COMMAND_YAW_LEFT,
    M400_COMMAND_YAW_RIGHT,
    M400_COMMAND_UP,
    M400_COMMAND_DOWN,
    M400_COMMAND_FORWARD_5M,
    M400_COMMAND_GOTO_STAGED,
    M400_COMMAND_GOTO_STAGED_10,
    M400_COMMAND_GOTO_OFFICIAL,
    M400_COMMAND_EMERGENCY_HOVER,
} E_M400QueuedCommand;

typedef enum {
    M400_COMMAND_STATE_IDLE = 0,
    M400_COMMAND_STATE_QUEUED,
    M400_COMMAND_STATE_RUNNING,
    M400_COMMAND_STATE_SUCCESS,
    M400_COMMAND_STATE_REJECTED,
    M400_COMMAND_STATE_FAILED,
    M400_COMMAND_STATE_EMERGENCY,
} E_M400CommandState;

typedef struct {
    T_DjiFcSubscriptionQuaternion quaternion;
    T_DjiFcSubscriptionVelocity velocity;
    T_DjiFcSubscriptionPositionFused fusedPosition;
    T_DjiFcSubscriptionRtkPosition rtkPosition;
    T_DjiFcSubscriptionPositionVO voPosition;
    T_DjiFcSubscriptionFlightStatus flightStatus;
    T_DjiFcSubscriptionGpsDate gpsDate;
    T_DjiFcSubscriptionGpsTime gpsTime;
    T_DjiFcSubscriptionRtkPositionInfo rtkPositionInfo;
    T_DjiFcSubscriptionRTKConnectStatus rtkConnectStatus;
    T_DjiDataTimestamp timestamp;
    bool quaternionValid;
    bool velocityValid;
    bool fusedPositionValid;
    bool rtkPositionValid;
    bool voPositionValid;
    bool flightStatusValid;
    bool gpsDateValid;
    bool gpsTimeValid;
    bool rtkPositionInfoValid;
    bool rtkConnectStatusValid;
} T_M400Telemetry;

static pthread_mutex_t s_stateMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t s_controlMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t s_queueMutex = PTHREAD_MUTEX_INITIALIZER;
static T_M400Telemetry s_telemetry;
static FILE *s_csvFile = NULL;
static bool s_recording = false;
static bool s_serviceStarted = false;
static bool s_flightControllerInitialized = false;
static bool s_abortMotion = false;
static E_M400QueuedCommand s_pendingCommand = M400_COMMAND_NONE;
static char s_currentCsvFile[M400_FILENAME_LENGTH] = "none";
static char s_csvFiles[M400_MAX_CSV_FILES][M400_FILENAME_LENGTH];
static size_t s_csvFileCount = 0;
static size_t s_selectedCsvIndex = 0;
static char s_lastResult[M400_RESULT_LENGTH] = "Ready";
static char s_pi4Message[M400_PI4_MESSAGE_MAX_BYTES + 1] = "";
static int32_t s_widgetValues[M400_WIDGET_COUNT] = {0};
static uint64_t s_buttonFeedbackUntilMs[M400_WIDGET_COUNT] = {0};
static E_M400CommandState s_commandState = M400_COMMAND_STATE_IDLE;
static E_M400QueuedCommand s_armedGotoCommand = M400_COMMAND_NONE;
static uint64_t s_gotoArmUntilMs = 0;

static void *M400TelemetryTask(void *arg);
static void *M400StatusTask(void *arg);
static void *M400ControlTask(void *arg);
static T_DjiReturnCode M400WidgetSetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t value, void *userData);
static T_DjiReturnCode M400WidgetGetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t *value, void *userData);
static float M400CurrentYawDegree(const T_DjiFcSubscriptionQuaternion *q);

static void M400OnPi4Message(const char *message)
{
    pthread_mutex_lock(&s_stateMutex);
    snprintf(s_pi4Message, sizeof(s_pi4Message), "%s", message);
    pthread_mutex_unlock(&s_stateMutex);
    USER_LOG_INFO("Pi4 TCP message received (%zu bytes)", strlen(message));
}

static const T_DjiWidgetHandlerListItem s_widgetHandlers[] = {
    {M400_WIDGET_START_RECORD, DJI_WIDGET_TYPE_BUTTON, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_STOP_RECORD, DJI_WIDGET_TYPE_BUTTON, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_RECORD_SWITCH, DJI_WIDGET_TYPE_SWITCH, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_RTK, DJI_WIDGET_TYPE_LIST, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_LONGITUDE, DJI_WIDGET_TYPE_INT_INPUT_BOX, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_LATITUDE, DJI_WIDGET_TYPE_INT_INPUT_BOX, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_COMMAND_STATE, DJI_WIDGET_TYPE_LIST, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_EMERGENCY_HOVER, DJI_WIDGET_TYPE_BUTTON, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_FORWARD_1M, DJI_WIDGET_TYPE_BUTTON, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_BACKWARD_1M, DJI_WIDGET_TYPE_BUTTON, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_LEFT_1M, DJI_WIDGET_TYPE_BUTTON, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_RIGHT_1M, DJI_WIDGET_TYPE_BUTTON, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_UP_1M, DJI_WIDGET_TYPE_BUTTON, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_DOWN_1M, DJI_WIDGET_TYPE_BUTTON, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_FORWARD_5M, DJI_WIDGET_TYPE_BUTTON, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_YAW_LEFT, DJI_WIDGET_TYPE_BUTTON, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_YAW_RIGHT, DJI_WIDGET_TYPE_BUTTON, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_GOTO_STAGED_2, DJI_WIDGET_TYPE_BUTTON, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_GOTO_STAGED_10, DJI_WIDGET_TYPE_BUTTON, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_GOTO_OFFICIAL_2, DJI_WIDGET_TYPE_BUTTON, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_CSV_SELECT, DJI_WIDGET_TYPE_LIST, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_CSV_DATE, DJI_WIDGET_TYPE_INT_INPUT_BOX, M400WidgetSetValue, M400WidgetGetValue, NULL},
    {M400_WIDGET_CSV_TIME, DJI_WIDGET_TYPE_INT_INPUT_BOX, M400WidgetSetValue, M400WidgetGetValue, NULL},
};

static uint64_t M400GetMonotonicMs(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (uint64_t) now.tv_sec * 1000ULL + (uint64_t) now.tv_nsec / 1000000ULL;
}

static bool M400AbortRequested(void)
{
    return __atomic_load_n(&s_abortMotion, __ATOMIC_ACQUIRE);
}

static void M400SetAbortRequested(bool requested)
{
    __atomic_store_n(&s_abortMotion, requested, __ATOMIC_RELEASE);
}

static void M400SetResult(const char *format, ...)
{
    va_list args;
    pthread_mutex_lock(&s_stateMutex);
    va_start(args, format);
    vsnprintf(s_lastResult, sizeof(s_lastResult), format, args);
    va_end(args);
    pthread_mutex_unlock(&s_stateMutex);
}

static void M400SetCommandState(E_M400CommandState state)
{
    pthread_mutex_lock(&s_stateMutex);
    s_commandState = state;
    pthread_mutex_unlock(&s_stateMutex);
}

static int M400CompareFileNameDescending(const void *left, const void *right)
{
    return -strcmp((const char *) left, (const char *) right);
}

static void M400RefreshCsvFileList(void)
{
    DIR *directory = opendir(M400_LOG_DIRECTORY);
    struct dirent *entry;
    char allFiles[64][M400_FILENAME_LENGTH];
    size_t count = 0;

    if (directory != NULL) {
        while ((entry = readdir(directory)) != NULL && count < 64) {
            size_t length = strlen(entry->d_name);
            if (length > 4 && strcmp(entry->d_name + length - 4, ".csv") == 0) {
                snprintf(allFiles[count], sizeof(allFiles[count]), "%.127s", entry->d_name);
                count++;
            }
        }
        closedir(directory);
    }

    qsort(allFiles, count, sizeof(allFiles[0]), M400CompareFileNameDescending);
    pthread_mutex_lock(&s_stateMutex);
    s_csvFileCount = count < M400_MAX_CSV_FILES ? count : M400_MAX_CSV_FILES;
    for (size_t i = 0; i < s_csvFileCount; ++i) {
        snprintf(s_csvFiles[i], sizeof(s_csvFiles[i]), "%s", allFiles[i]);
    }
    if (s_selectedCsvIndex >= s_csvFileCount) {
        s_selectedCsvIndex = 0;
    }
    pthread_mutex_unlock(&s_stateMutex);
}

static bool M400IsSubscriptionSuccess(T_DjiReturnCode returnCode)
{
    return returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ||
           returnCode == DJI_ERROR_SUBSCRIPTION_MODULE_CODE_TOPIC_DUPLICATE;
}

static T_DjiReturnCode M400SubscribeRequiredTopics(void)
{
    T_DjiReturnCode rc;

#define M400_SUBSCRIBE_REQUIRED(topic, frequency) \
    do { \
        rc = DjiFcSubscription_SubscribeTopic((topic), (frequency), NULL); \
        if (!M400IsSubscriptionSuccess(rc)) { \
            USER_LOG_ERROR("Subscribe required topic %d failed: 0x%08llX", (int) (topic), \
                           (unsigned long long) rc); \
            return rc; \
        } \
    } while (0)

#define M400_SUBSCRIBE_OPTIONAL(topic, frequency) \
    do { \
        rc = DjiFcSubscription_SubscribeTopic((topic), (frequency), NULL); \
        if (!M400IsSubscriptionSuccess(rc)) { \
            USER_LOG_WARN("Subscribe optional topic %d failed: 0x%08llX", (int) (topic), \
                          (unsigned long long) rc); \
        } \
    } while (0)

    M400_SUBSCRIBE_REQUIRED(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ);
    M400_SUBSCRIBE_REQUIRED(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ);
    M400_SUBSCRIBE_REQUIRED(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ);
    M400_SUBSCRIBE_REQUIRED(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ);
    M400_SUBSCRIBE_REQUIRED(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ);
    M400_SUBSCRIBE_REQUIRED(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ);
    M400_SUBSCRIBE_OPTIONAL(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ);
    M400_SUBSCRIBE_OPTIONAL(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ);
    M400_SUBSCRIBE_OPTIONAL(DJI_FC_SUBSCRIPTION_TOPIC_RTK_CONNECT_STATUS, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ);
    M400_SUBSCRIBE_OPTIONAL(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ);
    M400_SUBSCRIBE_OPTIONAL(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ);
    M400_SUBSCRIBE_OPTIONAL(DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ);

#undef M400_SUBSCRIBE_REQUIRED
#undef M400_SUBSCRIBE_OPTIONAL
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static bool M400ReadTopic(E_DjiFcSubscriptionTopic topic, void *value, uint16_t valueSize,
                          T_DjiDataTimestamp *timestamp)
{
    T_DjiDataTimestamp localTimestamp;
    if (timestamp == NULL) timestamp = &localTimestamp;
    return DjiFcSubscription_GetLatestValueOfTopic(topic, (uint8_t *) value, valueSize, timestamp) ==
           DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

// Match the official flight-control sample: height above takeoff is fused altitude
// minus the altitude recorded for the home point, not height above nearby ground.
static bool M400ReadHeightAboveTakeoff(T_DjiFcSubscriptionHeightRelative *height,
                                       uint16_t valueSize, T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionAltitudeFused fusedAltitude;
    T_DjiFcSubscriptionAltitudeOfHomePoint homeAltitude;
    if (valueSize != sizeof(*height) ||
        !M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
                       &homeAltitude, sizeof(homeAltitude), NULL) ||
        !M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
                       &fusedAltitude, sizeof(fusedAltitude), timestamp) ||
        !isfinite(fusedAltitude) || !isfinite(homeAltitude)) return false;
    *height = fusedAltitude - homeAltitude;
    return isfinite(*height);
}

static void M400ReadTelemetry(T_M400Telemetry *telemetry)
{
    memset(telemetry, 0, sizeof(*telemetry));
    telemetry->quaternionValid = M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                               &telemetry->quaternion, sizeof(telemetry->quaternion),
                                               &telemetry->timestamp);
    telemetry->velocityValid = M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                             &telemetry->velocity, sizeof(telemetry->velocity), NULL);
    telemetry->fusedPositionValid = M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
                                                  &telemetry->fusedPosition,
                                                  sizeof(telemetry->fusedPosition), NULL);
    telemetry->rtkPositionValid = M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
                                                &telemetry->rtkPosition, sizeof(telemetry->rtkPosition), NULL);
    telemetry->rtkPositionInfoValid = M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
                                                    &telemetry->rtkPositionInfo,
                                                    sizeof(telemetry->rtkPositionInfo), NULL);
    telemetry->rtkConnectStatusValid = M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_CONNECT_STATUS,
                                                     &telemetry->rtkConnectStatus,
                                                     sizeof(telemetry->rtkConnectStatus), NULL);
    telemetry->voPositionValid = M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO,
                                               &telemetry->voPosition, sizeof(telemetry->voPosition), NULL);
    telemetry->flightStatusValid = M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                 &telemetry->flightStatus,
                                                 sizeof(telemetry->flightStatus), NULL);
    telemetry->gpsDateValid = M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE,
                                            &telemetry->gpsDate, sizeof(telemetry->gpsDate), NULL);
    telemetry->gpsTimeValid = M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME,
                                            &telemetry->gpsTime, sizeof(telemetry->gpsTime), NULL);
}

static const char *M400RtkStatusText(const T_M400Telemetry *telemetry)
{
    if (!telemetry->rtkConnectStatusValid || telemetry->rtkConnectStatus.rtkConnected == 0) {
        return "DISCONNECTED";
    }
    if (!telemetry->rtkPositionInfoValid) {
        return "CONNECTED";
    }
    if (telemetry->rtkPositionInfo >= 48 && telemetry->rtkPositionInfo <= 50) {
        return "FIX";
    }
    if (telemetry->rtkPositionInfo >= 32 && telemetry->rtkPositionInfo <= 34) {
        return "FLOAT";
    }
    if (telemetry->rtkPositionInfo == 16 || telemetry->rtkPositionInfo == 17) {
        return "SINGLE";
    }
    if (telemetry->rtkPositionInfo == 0) {
        return "NO_FIX";
    }
    return "OTHER";
}

static int32_t M400RtkStatusWidgetValue(const T_M400Telemetry *telemetry)
{
    if (!telemetry->rtkConnectStatusValid || telemetry->rtkConnectStatus.rtkConnected == 0) return 0;
    if (!telemetry->rtkPositionInfoValid || telemetry->rtkPositionInfo == 0) return 1;
    if (telemetry->rtkPositionInfo == 16 || telemetry->rtkPositionInfo == 17) return 2;
    if (telemetry->rtkPositionInfo >= 32 && telemetry->rtkPositionInfo <= 34) return 3;
    if (telemetry->rtkPositionInfo >= 48 && telemetry->rtkPositionInfo <= 50) return 4;
    return 5;
}

static void M400GetDisplayPosition(const T_M400Telemetry *telemetry, double *longitude, double *latitude)
{
    *longitude = 0.0;
    *latitude = 0.0;
    if (telemetry->rtkPositionValid) {
        *longitude = telemetry->rtkPosition.longitude;
        *latitude = telemetry->rtkPosition.latitude;
    } else if (telemetry->fusedPositionValid) {
        *longitude = telemetry->fusedPosition.longitude * M400_DEG_PER_RAD;
        *latitude = telemetry->fusedPosition.latitude * M400_DEG_PER_RAD;
    }
}

static void M400WriteCsvRow(FILE *file, const T_M400Telemetry *t)
{
    struct timespec hostTime;
    clock_gettime(CLOCK_REALTIME, &hostTime);
    double hostTimeSeconds = (double) hostTime.tv_sec + (double) hostTime.tv_nsec * 1e-9;
    double uavTimeSeconds = (double) t->timestamp.millisecond * 1e-3 +
                            (double) t->timestamp.microsecond * 1e-6;

    fprintf(file,
            "%.6f,%.6f,%.9f,%.9f,%.9f,%.9f,%.4f,%.4f,%.4f,%.10f,%.10f,%.3f,%u,"
            "%.10f,%.10f,%.3f,%u,%u,%.4f,%.4f,%.4f,%u,%u,%u,%u,%u,%06u\n",
            hostTimeSeconds,
            uavTimeSeconds,
            t->quaternionValid ? t->quaternion.q0 : NAN,
            t->quaternionValid ? t->quaternion.q1 : NAN,
            t->quaternionValid ? t->quaternion.q2 : NAN,
            t->quaternionValid ? t->quaternion.q3 : NAN,
            t->velocityValid ? t->velocity.data.x : NAN,
            t->velocityValid ? t->velocity.data.y : NAN,
            t->velocityValid ? t->velocity.data.z : NAN,
            t->fusedPositionValid ? t->fusedPosition.longitude : NAN,
            t->fusedPositionValid ? t->fusedPosition.latitude : NAN,
            t->fusedPositionValid ? t->fusedPosition.altitude : NAN,
            t->fusedPositionValid ? t->fusedPosition.visibleSatelliteNumber : 0,
            t->rtkPositionValid ? t->rtkPosition.longitude : NAN,
            t->rtkPositionValid ? t->rtkPosition.latitude : NAN,
            t->rtkPositionValid ? t->rtkPosition.hfsl : NAN,
            t->rtkPositionInfoValid ? t->rtkPositionInfo : 0,
            t->rtkConnectStatusValid ? t->rtkConnectStatus.rtkConnected : 0,
            t->voPositionValid ? t->voPosition.x : NAN,
            t->voPositionValid ? t->voPosition.y : NAN,
            t->voPositionValid ? t->voPosition.z : NAN,
            t->voPositionValid ? t->voPosition.xHealth : 0,
            t->voPositionValid ? t->voPosition.yHealth : 0,
            t->voPositionValid ? t->voPosition.zHealth : 0,
            t->flightStatusValid ? t->flightStatus : 0,
            t->gpsDateValid ? t->gpsDate : 0,
            t->gpsTimeValid ? t->gpsTime : 0);
}

static void *M400TelemetryTask(void *arg)
{
    (void) arg;
    struct timespec nextTick;
    unsigned int flushCounter = 0;
    clock_gettime(CLOCK_MONOTONIC, &nextTick);

    while (true) {
        nextTick.tv_nsec += M400_LOG_PERIOD_NS;
        if (nextTick.tv_nsec >= 1000000000L) {
            nextTick.tv_sec++;
            nextTick.tv_nsec -= 1000000000L;
        }
        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &nextTick, NULL) == EINTR) {
        }

        T_M400Telemetry telemetry;
        M400ReadTelemetry(&telemetry);
        pthread_mutex_lock(&s_stateMutex);
        s_telemetry = telemetry;
        if (s_recording && s_csvFile != NULL) {
            M400WriteCsvRow(s_csvFile, &telemetry);
            if (++flushCounter >= M400_LOG_FREQUENCY_HZ * 5) {
                fflush(s_csvFile);
                flushCounter = 0;
            }
        } else {
            flushCounter = 0;
        }
        pthread_mutex_unlock(&s_stateMutex);
    }
    return NULL;
}

static void *M400StatusTask(void *arg)
{
    (void) arg;
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    while (true) {
        char message[DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN];
        char result[M400_RESULT_LENGTH];
        char pi4Message[sizeof(s_pi4Message)];
        T_M400Telemetry telemetry;
        bool recording;

        pthread_mutex_lock(&s_stateMutex);
        telemetry = s_telemetry;
        recording = s_recording;
        snprintf(result, sizeof(result), "%s", s_lastResult);
        snprintf(pi4Message, sizeof(pi4Message), "%s", s_pi4Message);
        pthread_mutex_unlock(&s_stateMutex);

        double longitude = NAN;
        double latitude = NAN;
        const char *positionSource = "NONE";
        if (telemetry.rtkPositionValid) {
            longitude = telemetry.rtkPosition.longitude;
            latitude = telemetry.rtkPosition.latitude;
            positionSource = "RTK";
        } else if (telemetry.fusedPositionValid) {
            longitude = telemetry.fusedPosition.longitude * M400_DEG_PER_RAD;
            latitude = telemetry.fusedPosition.latitude * M400_DEG_PER_RAD;
            positionSource = "FUSED";
        }

        if (pi4Message[0] != '\0') {
            int written = snprintf(message, sizeof(message),
                                   "REC:%s RTK:%s POS:%s\r\nLon:%.7f Lat:%.7f\r\nCMD:%s\r\nPI4:%s",
                                   recording ? "ON" : "OFF", M400RtkStatusText(&telemetry), positionSource,
                                   longitude, latitude, result, pi4Message);
            if (written < 0 || (size_t) written >= sizeof(message)) {
                snprintf(message, sizeof(message), "PI4:%s", pi4Message);
            }
        } else {
            snprintf(message, sizeof(message),
                     "REC:%s RTK:%s POS:%s\r\nLon:%.7f Lat:%.7f\r\nCMD:%s",
                     recording ? "ON" : "OFF", M400RtkStatusText(&telemetry), positionSource,
                     longitude, latitude, result);
        }
        T_DjiReturnCode rc = DjiWidgetFloatingWindow_ShowMessage(message);
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_WARN("Update widget status failed: 0x%08llX", (unsigned long long) rc);
        }
        osal->TaskSleepMs(1000);
    }
    return NULL;
}

static T_DjiReturnCode M400EnsureFlightControllerInitialized(void)
{
    if (s_flightControllerInitialized) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    T_M400Telemetry telemetry;
    pthread_mutex_lock(&s_stateMutex);
    telemetry = s_telemetry;
    pthread_mutex_unlock(&s_stateMutex);
    if (!telemetry.fusedPositionValid) {
        M400SetResult("Control denied: position unavailable");
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }

    T_DjiFlightControllerRidInfo ridInfo = {0};
    ridInfo.latitude = telemetry.fusedPosition.latitude * M400_DEG_PER_RAD;
    ridInfo.longitude = telemetry.fusedPosition.longitude * M400_DEG_PER_RAD;
    double altitude = telemetry.fusedPosition.altitude;
    if (altitude < 0.0) altitude = 0.0;
    if (altitude > 65535.0) altitude = 65535.0;
    ridInfo.altitude = (uint16_t) altitude;

    T_DjiReturnCode rc = DjiFlightController_Init(ridInfo);
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS || rc == DJI_ERROR_SYSTEM_MODULE_CODE_DUPLICATE) {
        s_flightControllerInitialized = true;
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    M400SetResult("FC init failed: 0x%08llX", (unsigned long long) rc);
    return rc;
}

static T_DjiReturnCode M400CheckAircraftInAir(void)
{
    T_DjiFcSubscriptionFlightStatus flightStatus;
    bool valid;
    pthread_mutex_lock(&s_stateMutex);
    flightStatus = s_telemetry.flightStatus;
    valid = s_telemetry.flightStatusValid;
    pthread_mutex_unlock(&s_stateMutex);
    if (!valid || flightStatus != DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
        M400SetResult("Control denied: aircraft not airborne");
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode M400ObtainAuthority(void)
{
    T_DjiReturnCode rc = M400EnsureFlightControllerInitialized();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return rc;
    rc = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        M400SetResult("Authority failed: RC must be P mode (0x%08llX)", (unsigned long long) rc);
    }
    return rc;
}

static T_DjiReturnCode M400SendStableZero(unsigned int durationMs)
{
    T_DjiFlightControllerJoystickMode mode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };
    T_DjiFlightControllerJoystickCommand zeroCommand = {0};
    DjiFlightController_SetJoystickMode(mode);
    T_DjiReturnCode lastRc = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    for (unsigned int elapsed = 0; elapsed < durationMs; elapsed += M400_CONTROL_PERIOD_MS) {
        T_DjiReturnCode rc = DjiFlightController_ExecuteJoystickAction(zeroCommand);
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) lastRc = rc;
        usleep(M400_CONTROL_PERIOD_MS * 1000);
    }
    return lastRc;
}

static bool M400ReadMoveOrigin(T_DjiFcSubscriptionPositionFused *position,
                               T_DjiFcSubscriptionHeightRelative *height, float *yawDeg)
{
    T_DjiFcSubscriptionQuaternion quaternion;
    T_DjiDataTimestamp positionTimestamp, heightTimestamp, yawTimestamp;
    if (!M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, position,
                       sizeof(*position), &positionTimestamp) ||
        !M400ReadHeightAboveTakeoff(height, sizeof(*height), &heightTimestamp) ||
        !M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, &quaternion,
                       sizeof(quaternion), &yawTimestamp) ||
        position->visibleSatelliteNumber < M400_MIN_VISIBLE_SATELLITES) return false;

    const uint64_t startMs = M400GetMonotonicMs();
    bool positionFresh = false, heightFresh = false, yawFresh = false;
    while (M400GetMonotonicMs() - startMs < M400_POSITION_STALE_MS &&
           (!positionFresh || !heightFresh || !yawFresh)) {
        if (M400AbortRequested()) return false;
        usleep(M400_CONTROL_PERIOD_MS * 1000);
        T_DjiDataTimestamp nextPosition, nextHeight, nextYaw;
        if (!M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, position,
                           sizeof(*position), &nextPosition) ||
            !M400ReadHeightAboveTakeoff(height, sizeof(*height), &nextHeight) ||
            !M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, &quaternion,
                           sizeof(quaternion), &nextYaw)) return false;
        positionFresh |= nextPosition.millisecond != positionTimestamp.millisecond ||
                         nextPosition.microsecond != positionTimestamp.microsecond;
        heightFresh |= nextHeight.millisecond != heightTimestamp.millisecond ||
                       nextHeight.microsecond != heightTimestamp.microsecond;
        yawFresh |= nextYaw.millisecond != yawTimestamp.millisecond ||
                    nextYaw.microsecond != yawTimestamp.microsecond;
        positionTimestamp = nextPosition;
        heightTimestamp = nextHeight;
        yawTimestamp = nextYaw;
    }
    if (!positionFresh || !heightFresh || !yawFresh ||
        position->visibleSatelliteNumber < M400_MIN_VISIBLE_SATELLITES ||
        !isfinite(position->latitude) ||
        !isfinite(position->longitude) || !isfinite(*height) ||
        *height < 0.0f || *height > 120.0f ||
        fabs(position->latitude) > 85.0 / M400_DEG_PER_RAD) return false;
    *yawDeg = M400CurrentYawDegree(&quaternion);
    return isfinite(*yawDeg);
}

T_DjiReturnCode M400Control_MoveRelative(E_M400MoveDirection direction, float distanceM)
{
    if (!isfinite(distanceM) || distanceM <= 0.0f || distanceM > 10.0f ||
        direction < M400_MOVE_FORWARD || direction > M400_MOVE_RIGHT) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    T_DjiReturnCode rc = M400CheckAircraftInAir();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return rc;
    T_DjiFcSubscriptionPositionFused position;
    T_DjiFcSubscriptionHeightRelative height;
    float yawDeg;
    if (!M400ReadMoveOrigin(&position, &height, &yawDeg)) {
        M400SetResult("Move denied: positioning unavailable");
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }
    const double yawRad = yawDeg / M400_DEG_PER_RAD;
    double forwardM = 0.0, rightM = 0.0;
    if (direction == M400_MOVE_FORWARD) forwardM = distanceM;
    if (direction == M400_MOVE_BACKWARD) forwardM = -distanceM;
    if (direction == M400_MOVE_LEFT) rightM = -distanceM;
    if (direction == M400_MOVE_RIGHT) rightM = distanceM;
    const double northM = cos(yawRad) * forwardM - sin(yawRad) * rightM;
    const double eastM = sin(yawRad) * forwardM + cos(yawRad) * rightM;
    const T_M400GotoTarget target = {
        .latitudeDeg = (position.latitude + northM / M400_EARTH_RADIUS_M) * M400_DEG_PER_RAD,
        .longitudeDeg = (position.longitude + eastM /
                         (M400_EARTH_RADIUS_M * cos(position.latitude))) * M400_DEG_PER_RAD,
        .heightAboveTakeoffM = height,
        .yawDeg = yawDeg,
        .horizontalToleranceM = 0.4f,
        .verticalToleranceM = 0.3f,
        .yawToleranceDeg = 3.0f,
        .timeoutMs = distanceM <= 1.0f ? 30000U : 60000U,
    };
    // The 1 m buttons send the remaining position error without a speedFactor clip.
    // Longer relative moves keep the existing +/-2 m per-axis command cap.
    return distanceM <= 1.0f ? M400Control_GotoCoordinateWithLimit(&target, 0.0f) :
                              M400Control_GotoCoordinate(&target);
}

T_DjiReturnCode M400Control_MoveVertical(float deltaM)
{
    if (!isfinite(deltaM) || fabsf(deltaM) < 0.1f || fabsf(deltaM) > 10.0f) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    T_DjiReturnCode rc = M400CheckAircraftInAir();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return rc;
    T_DjiFcSubscriptionPositionFused position;
    T_DjiFcSubscriptionHeightRelative height;
    float yawDeg;
    if (!M400ReadMoveOrigin(&position, &height, &yawDeg) ||
        height + deltaM < 0.0f || height + deltaM > 120.0f) {
        M400SetResult("Vertical move denied: height/position invalid");
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }
    const T_M400GotoTarget target = {
        .latitudeDeg = position.latitude * M400_DEG_PER_RAD,
        .longitudeDeg = position.longitude * M400_DEG_PER_RAD,
        .heightAboveTakeoffM = height + deltaM,
        .yawDeg = yawDeg,
        .horizontalToleranceM = 0.7f,
        .verticalToleranceM = 0.25f,
        .yawToleranceDeg = 3.0f,
        .timeoutMs = 30000U,
    };
    return M400Control_GotoCoordinate(&target);
}

static float M400NormalizeYaw(float yawDegree)
{
    while (yawDegree > 180.0f) yawDegree -= 360.0f;
    while (yawDegree < -180.0f) yawDegree += 360.0f;
    return yawDegree;
}

static float M400CurrentYawDegree(const T_DjiFcSubscriptionQuaternion *q)
{
    double t0 = 1.0 - 2.0 * (q->q2 * q->q2 + q->q3 * q->q3);
    double t1 = 2.0 * (q->q1 * q->q2 + q->q0 * q->q3);
    return (float) (atan2(t1, t0) * M400_DEG_PER_RAD);
}

static float M400NextYawRateCommand(float targetYawDeg, float measuredYawDeg,
                                     float previousCommandDegS, float dtSec)
{
    const float errorDeg = M400NormalizeYaw(targetYawDeg - measuredYawDeg);
    const float magnitude = fabsf(errorDeg);
    const float brakingRate = sqrtf(2.0f * M400_YAW_ACCEL_DEG_S2 * magnitude);
    const float proportionalRate = 2.0f * magnitude;
    const float targetRate = copysignf(fminf(M400_YAW_MAX_RATE_DEG_S,
                                             fminf(brakingRate, proportionalRate)), errorDeg);
    const float maxChange = M400_YAW_ACCEL_DEG_S2 * fminf(dtSec, 0.1f);
    return previousCommandDegS + fmaxf(-maxChange,
                                        fminf(maxChange, targetRate - previousCommandDegS));
}

T_DjiReturnCode M400Control_YawRelative(float deltaDegree)
{
    if (!isfinite(deltaDegree) || fabsf(deltaDegree) < 0.1f || fabsf(deltaDegree) > 180.0f) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    if (pthread_mutex_trylock(&s_controlMutex) != 0) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }

    T_DjiReturnCode rc = M400CheckAircraftInAir();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) goto exit;
    rc = M400ObtainAuthority();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) goto exit;

    T_DjiFcSubscriptionQuaternion currentQuaternion;
    T_DjiDataTimestamp previousTimestamp;
    if (!M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, &currentQuaternion,
                       sizeof(currentQuaternion), &previousTimestamp)) {
        M400SetResult("Yaw denied: attitude unavailable");
        rc = DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
        goto release;
    }
    // Do not start a relative turn from a cached heading that stopped updating.
    const uint64_t freshWaitStartMs = M400GetMonotonicMs();
    bool initialSampleFresh = false;
    while (M400GetMonotonicMs() - freshWaitStartMs < M400_POSITION_STALE_MS) {
        if (M400AbortRequested()) {
            rc = DJI_ERROR_SYSTEM_MODULE_CODE_EXECUTING_HIGHER_PRIORITY_TASK;
            goto release;
        }
        usleep(M400_CONTROL_PERIOD_MS * 1000);
        T_DjiDataTimestamp nextTimestamp;
        if (!M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, &currentQuaternion,
                           sizeof(currentQuaternion), &nextTimestamp)) {
            rc = DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
            goto release;
        }
        if (nextTimestamp.millisecond != previousTimestamp.millisecond ||
            nextTimestamp.microsecond != previousTimestamp.microsecond) {
            previousTimestamp = nextTimestamp;
            initialSampleFresh = true;
            break;
        }
    }
    if (!initialSampleFresh) {
        M400SetResult("Yaw denied: attitude sample stale");
        rc = DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT;
        goto release;
    }
    T_DjiFcSubscriptionHeightRelative holdHeight;
    T_DjiDataTimestamp heightTimestamp;
    if (!M400ReadHeightAboveTakeoff(&holdHeight, sizeof(holdHeight), &heightTimestamp)) {
        rc = DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
        goto release;
    }
    bool heightFresh = false;
    const uint64_t heightWaitStartMs = M400GetMonotonicMs();
    while (M400GetMonotonicMs() - heightWaitStartMs < M400_POSITION_STALE_MS) {
        if (M400AbortRequested()) {
            rc = DJI_ERROR_SYSTEM_MODULE_CODE_EXECUTING_HIGHER_PRIORITY_TASK;
            goto release;
        }
        usleep(M400_CONTROL_PERIOD_MS * 1000);
        T_DjiDataTimestamp nextHeightTimestamp;
        if (!M400ReadHeightAboveTakeoff(&holdHeight, sizeof(holdHeight), &nextHeightTimestamp)) {
            rc = DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
            goto release;
        }
        if (nextHeightTimestamp.millisecond != heightTimestamp.millisecond ||
            nextHeightTimestamp.microsecond != heightTimestamp.microsecond) {
            heightFresh = true;
            break;
        }
    }
    if (!heightFresh || !isfinite(holdHeight) || holdHeight < 0.0f || holdHeight > 120.0f ||
        !M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, &currentQuaternion,
                       sizeof(currentQuaternion), &previousTimestamp)) {
        rc = DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
        goto release;
    }
    const float startYaw = M400CurrentYawDegree(&currentQuaternion);
    if (!isfinite(startYaw)) {
        M400SetResult("Yaw denied: attitude invalid");
        rc = DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
        goto release;
    }
    const float targetYaw = M400NormalizeYaw(startYaw + deltaDegree);
    T_DjiFlightControllerJoystickMode mode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_POSITION_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_POSITION_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };
    T_DjiFlightControllerJoystickCommand command = {0};
    command.z = holdHeight;
    DjiFlightController_SetJoystickMode(mode);

    const uint64_t startMs = M400GetMonotonicMs();
    uint64_t lastFreshMs = startMs;
    uint64_t previousSampleMs = startMs;
    uint64_t previousCommandMs = startMs;
    uint64_t settledSinceMs = 0;
    float previousYaw = startYaw;
    float filteredYawRateDegS = 0.0f;
    bool haveYawRate = false;
    bool finished = false;
    while (M400GetMonotonicMs() - startMs < M400_YAW_TIMEOUT_MS) {
        if (M400AbortRequested()) {
            rc = DJI_ERROR_SYSTEM_MODULE_CODE_EXECUTING_HIGHER_PRIORITY_TASK;
            break;
        }
        rc = M400CheckAircraftInAir();
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) break;

        const uint64_t nowMs = M400GetMonotonicMs();
        T_DjiDataTimestamp currentTimestamp;
        if (!M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, &currentQuaternion,
                           sizeof(currentQuaternion), &currentTimestamp)) {
            rc = DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
            break;
        }
        const float measuredYaw = M400CurrentYawDegree(&currentQuaternion);
        if (!isfinite(measuredYaw)) {
            rc = DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
            break;
        }
        const bool fresh = currentTimestamp.millisecond != previousTimestamp.millisecond ||
                           currentTimestamp.microsecond != previousTimestamp.microsecond;
        float measuredRateDegS = INFINITY;
        if (fresh) {
            const uint64_t sampleIntervalMs = nowMs - previousSampleMs;
            if (sampleIntervalMs > 0) {
                const float instantRateDegS = M400NormalizeYaw(measuredYaw - previousYaw) *
                                              1000.0f / sampleIntervalMs;
                // Smooth quaternion quantization noise before deciding it has stopped.
                filteredYawRateDegS = haveYawRate ?
                                      0.8f * filteredYawRateDegS + 0.2f * instantRateDegS :
                                      instantRateDegS;
                haveYawRate = true;
                measuredRateDegS = fabsf(filteredYawRateDegS);
            }
            previousTimestamp = currentTimestamp;
            previousSampleMs = nowMs;
            previousYaw = measuredYaw;
            lastFreshMs = nowMs;
        } else if (nowMs - lastFreshMs > M400_POSITION_STALE_MS) {
            rc = DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT;
            break;
        }

        const float dtSec = (float) (nowMs - previousCommandMs) / 1000.0f;
        command.yaw = M400NextYawRateCommand(targetYaw, measuredYaw, command.yaw, dtSec);
        previousCommandMs = nowMs;
        rc = DjiFlightController_ExecuteJoystickAction(command);
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) break;

        if (fresh && fabsf(M400NormalizeYaw(targetYaw - measuredYaw)) <= M400_YAW_POSITION_TOL_DEG &&
            measuredRateDegS <= M400_YAW_RATE_TOL_DEG_S && fabsf(command.yaw) <= 1.0f) {
            if (settledSinceMs == 0) settledSinceMs = nowMs;
            if (nowMs - settledSinceMs >= M400_YAW_SETTLE_HOLD_MS) {
                finished = true;
                break;
            }
        } else if (fresh) {
            settledSinceMs = 0;
        }
        usleep(M400_CONTROL_PERIOD_MS * 1000);
    }
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && !finished) {
        rc = DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT;
    }
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        for (unsigned int elapsed = 0; elapsed < 500; elapsed += M400_CONTROL_PERIOD_MS) {
            if (M400AbortRequested()) {
                rc = DJI_ERROR_SYSTEM_MODULE_CODE_EXECUTING_HIGHER_PRIORITY_TASK;
                break;
            }
            const float maxChange = M400_YAW_ACCEL_DEG_S2 * M400_CONTROL_PERIOD_MS / 1000.0f;
            command.yaw += fmaxf(-maxChange, fminf(maxChange, -command.yaw));
            rc = DjiFlightController_ExecuteJoystickAction(command);
            if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) break;
            usleep(M400_CONTROL_PERIOD_MS * 1000);
        }
    }
release:
    // A failure or emergency may need a prompt brake. Normal completion leaves
    // the controller in position/zero-yaw-rate mode until authority is released.
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        (void) M400SendStableZero(500);
    }
    T_DjiReturnCode releaseRc = DjiFlightController_ReleaseJoystickCtrlAuthority();
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) rc = releaseRc;
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        M400SetResult("YAW %+.0fdeg: OK", deltaDegree);
    } else if (M400AbortRequested()) {
        M400SetResult("YAW interrupted by emergency hover");
    } else {
        M400SetResult("YAW failed: 0x%08llX", (unsigned long long) rc);
    }

exit:
    pthread_mutex_unlock(&s_controlMutex);
    return rc;
}

static void M400OffsetToTarget(double targetLatitudeRad, double targetLongitudeRad,
                                const T_DjiFcSubscriptionPositionFused *position,
                                double *northM, double *eastM)
{
    double longitudeDelta = targetLongitudeRad - position->longitude;
    if (longitudeDelta > M400_PI) longitudeDelta -= 2.0 * M400_PI;
    if (longitudeDelta < -M400_PI) longitudeDelta += 2.0 * M400_PI;
    *northM = (targetLatitudeRad - position->latitude) * M400_EARTH_RADIUS_M;
    *eastM = longitudeDelta * M400_EARTH_RADIUS_M *
             cos((targetLatitudeRad + position->latitude) * 0.5);
}

static float M400ClampPositionAxis(double valueM, float limitM)
{
    if (limitM == 0.0f) return (float) valueM;
    return (float) fmax(-limitM, fmin(limitM, valueM));
}

static T_DjiReturnCode M400RunGotoCoordinate(const T_M400GotoTarget *target, bool officialStyle,
                                            float horizontalLimitM)
{
    if (!isfinite(horizontalLimitM) || horizontalLimitM < 0.0f ||
        horizontalLimitM > M400_POSITION_COMMAND_TEST_LIMIT_M ||
        target == NULL || !isfinite(target->latitudeDeg) || !isfinite(target->longitudeDeg) ||
        !isfinite(target->heightAboveTakeoffM) || !isfinite(target->yawDeg) ||
        !isfinite(target->horizontalToleranceM) || !isfinite(target->verticalToleranceM) ||
        !isfinite(target->yawToleranceDeg) || fabs(target->latitudeDeg) > 85.0 ||
        fabs(target->longitudeDeg) > 180.0 || target->heightAboveTakeoffM < 0.0f ||
        target->heightAboveTakeoffM > 120.0f || target->horizontalToleranceM <= 0.0f ||
        target->verticalToleranceM <= 0.0f || target->yawToleranceDeg <= 0.0f ||
        target->timeoutMs < 1000 || target->timeoutMs > 120000) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    if (pthread_mutex_trylock(&s_controlMutex) != 0) return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    T_DjiReturnCode rc = M400CheckAircraftInAir();
    float lastHorizontalErrorM = 0.0f;
    const char *failureDetail = NULL;
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) goto exit;

    T_DjiFcSubscriptionPositionFused position;
    T_DjiFcSubscriptionHeightRelative height;
    T_DjiFcSubscriptionQuaternion quaternion;
    T_DjiDataTimestamp positionTimestamp, yawTimestamp, heightTimestamp;
    if (!M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, &position,
                       sizeof(position), &positionTimestamp) ||
        !M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, &quaternion,
                       sizeof(quaternion), &yawTimestamp) ||
        !M400ReadHeightAboveTakeoff(&height, sizeof(height), &heightTimestamp) ||
        position.visibleSatelliteNumber < M400_MIN_VISIBLE_SATELLITES ||
        !isfinite(position.latitude) || !isfinite(position.longitude) ||
        !isfinite(height) || height < 0.0f || height > 120.0f) {
        M400SetResult("Goto denied: positioning/height invalid");
        rc = DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
        goto exit;
    }

    // Require fresh position and heading samples before any motion command.
    const uint64_t freshWaitStartMs = M400GetMonotonicMs();
    bool positionFresh = false, yawFresh = false, heightFresh = false;
    while (M400GetMonotonicMs() - freshWaitStartMs < M400_POSITION_STALE_MS &&
           (!positionFresh || !yawFresh || !heightFresh)) {
        if (M400AbortRequested()) {
            rc = DJI_ERROR_SYSTEM_MODULE_CODE_EXECUTING_HIGHER_PRIORITY_TASK;
            goto exit;
        }
        usleep(M400_CONTROL_PERIOD_MS * 1000);
        T_DjiDataTimestamp nextPositionTimestamp, nextYawTimestamp, nextHeightTimestamp;
        if (!M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, &position,
                           sizeof(position), &nextPositionTimestamp) ||
            !M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, &quaternion,
                           sizeof(quaternion), &nextYawTimestamp) ||
            !M400ReadHeightAboveTakeoff(&height, sizeof(height), &nextHeightTimestamp)) {
            rc = DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
            goto exit;
        }
        positionFresh |= nextPositionTimestamp.millisecond != positionTimestamp.millisecond ||
                         nextPositionTimestamp.microsecond != positionTimestamp.microsecond;
        yawFresh |= nextYawTimestamp.millisecond != yawTimestamp.millisecond ||
                    nextYawTimestamp.microsecond != yawTimestamp.microsecond;
        heightFresh |= nextHeightTimestamp.millisecond != heightTimestamp.millisecond ||
                       nextHeightTimestamp.microsecond != heightTimestamp.microsecond;
        positionTimestamp = nextPositionTimestamp;
        yawTimestamp = nextYawTimestamp;
        heightTimestamp = nextHeightTimestamp;
    }
    if (!positionFresh || !yawFresh || !heightFresh || !isfinite(height) ||
        !isfinite(position.latitude) ||
        !isfinite(position.longitude) || height < 0.0f || height > 120.0f ||
        !isfinite(M400CurrentYawDegree(&quaternion))) {
        M400SetResult("Goto denied: telemetry stale");
        rc = DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT;
        goto exit;
    }

    const double targetLatitudeRad = target->latitudeDeg / M400_DEG_PER_RAD;
    const double targetLongitudeRad = target->longitudeDeg / M400_DEG_PER_RAD;
    double northM, eastM;
    M400OffsetToTarget(targetLatitudeRad, targetLongitudeRad, &position, &northM, &eastM);
    if (hypot(northM, eastM) > M400_GOTO_MAX_DISTANCE_M) {
        M400SetResult("Goto denied: target beyond 100 m test limit");
        rc = DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
        goto exit;
    }

    rc = M400ObtainAuthority();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) goto exit;
    const T_DjiFlightControllerJoystickMode mode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_POSITION_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_POSITION_CONTROL_MODE,
        officialStyle ? DJI_FLIGHT_CONTROLLER_YAW_ANGLE_CONTROL_MODE :
                        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };
    DjiFlightController_SetJoystickMode(mode);

    const float targetYaw = M400NormalizeYaw(target->yawDeg);
    const float holdHeight = height;
    float verticalSetpoint = height;
    T_DjiFlightControllerJoystickCommand command = {0};
    const uint64_t startMs = M400GetMonotonicMs();
    uint64_t lastFreshPositionMs = startMs, lastFreshYawMs = startMs;
    uint64_t lastFreshHeightMs = startMs;
    uint64_t previousYawSampleMs = startMs, previousCommandMs = startMs;
    uint64_t withinThresholdSinceMs = 0;
    float previousYaw = M400CurrentYawDegree(&quaternion);
    float filteredYawRateDegS = 0.0f;
    bool haveYawRate = false, finished = false;
    int stage = officialStyle ? 3 : 0; // 0=turn, 1=height, 2=horizontal, 3=simultaneous
    M400SetCommandState(M400_COMMAND_STATE_RUNNING);
    if (officialStyle) M400SetResult("Official-style goto F2: simultaneous");
    else M400SetResult("Staged goto F%.0f 1/3: heading", horizontalLimitM);

    while (M400GetMonotonicMs() - startMs < target->timeoutMs) {
        if (M400AbortRequested()) {
            rc = DJI_ERROR_SYSTEM_MODULE_CODE_EXECUTING_HIGHER_PRIORITY_TASK;
            break;
        }
        rc = M400CheckAircraftInAir();
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) break;

        T_DjiFcSubscriptionVelocity velocity;
        T_DjiDataTimestamp nextPositionTimestamp, nextYawTimestamp, nextHeightTimestamp;
        if (!M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, &position,
                           sizeof(position), &nextPositionTimestamp) ||
            !M400ReadHeightAboveTakeoff(&height, sizeof(height), &nextHeightTimestamp) ||
            !M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, &quaternion,
                           sizeof(quaternion), &nextYawTimestamp) ||
            !M400ReadTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, &velocity,
                           sizeof(velocity), NULL) ||
            position.visibleSatelliteNumber < M400_MIN_VISIBLE_SATELLITES ||
            !isfinite(position.latitude) || !isfinite(position.longitude) ||
            !isfinite(height) || !isfinite(velocity.data.x) ||
            !isfinite(velocity.data.y) || !isfinite(velocity.data.z)) {
            rc = DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
            failureDetail = "positioning/telemetry invalid";
            break;
        }
        const uint64_t nowMs = M400GetMonotonicMs();
        const bool newPosition = nextPositionTimestamp.millisecond != positionTimestamp.millisecond ||
                                 nextPositionTimestamp.microsecond != positionTimestamp.microsecond;
        const bool newYaw = nextYawTimestamp.millisecond != yawTimestamp.millisecond ||
                            nextYawTimestamp.microsecond != yawTimestamp.microsecond;
        const bool newHeight = nextHeightTimestamp.millisecond != heightTimestamp.millisecond ||
                               nextHeightTimestamp.microsecond != heightTimestamp.microsecond;
        if (newPosition) { positionTimestamp = nextPositionTimestamp; lastFreshPositionMs = nowMs; }
        if (newYaw) { yawTimestamp = nextYawTimestamp; lastFreshYawMs = nowMs; }
        if (newHeight) { heightTimestamp = nextHeightTimestamp; lastFreshHeightMs = nowMs; }
        if (nowMs - lastFreshPositionMs > M400_POSITION_STALE_MS ||
            nowMs - lastFreshYawMs > M400_POSITION_STALE_MS ||
            nowMs - lastFreshHeightMs > M400_POSITION_STALE_MS) {
            rc = DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT;
            failureDetail = "position/heading sample stale";
            break;
        }

        const float measuredYaw = M400CurrentYawDegree(&quaternion);
        if (!isfinite(measuredYaw)) {
            rc = DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
            failureDetail = "heading invalid";
            break;
        }
        if (newYaw) {
            const uint64_t yawDtMs = nowMs - previousYawSampleMs;
            if (yawDtMs > 0) {
                const float instantRate = M400NormalizeYaw(measuredYaw - previousYaw) * 1000.0f / yawDtMs;
                filteredYawRateDegS = haveYawRate ?
                                      0.8f * filteredYawRateDegS + 0.2f * instantRate : instantRate;
                haveYawRate = true;
            }
            previousYaw = measuredYaw;
            previousYawSampleMs = nowMs;
        }

        M400OffsetToTarget(targetLatitudeRad, targetLongitudeRad, &position, &northM, &eastM);
        const double horizontalErrorM = hypot(northM, eastM);
        if (horizontalErrorM > M400_GOTO_MAX_DISTANCE_M + 5.0) {
            rc = DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
            failureDetail = "target range exceeded in flight";
            break;
        }
        lastHorizontalErrorM = (float) horizontalErrorM;
        const float verticalErrorM = target->heightAboveTakeoffM - height;
        const float yawErrorDeg = fabsf(M400NormalizeYaw(targetYaw - measuredYaw));
        const float horizontalSpeedMps = hypotf(velocity.data.x, velocity.data.y);
        const float dtSec = (float) (nowMs - previousCommandMs) / 1000.0f;
        previousCommandMs = nowMs;
        command.x = command.y = 0.0f;

        if (officialStyle) {
            // DJI sample style: clip each position-offset axis independently.
            command.x = M400ClampPositionAxis(northM, horizontalLimitM);
            command.y = M400ClampPositionAxis(eastM, horizontalLimitM);
            command.z = target->heightAboveTakeoffM;
            command.yaw = targetYaw;
        } else {
            command.yaw = M400NextYawRateCommand(targetYaw, measuredYaw, command.yaw, dtSec);
            if (stage == 0) {
                command.z = holdHeight;
            } else if (stage == 1) {
                const float maxStepM = M400_VERTICAL_SETPOINT_RATE_MPS * fminf(dtSec, 0.1f);
                verticalSetpoint += fmaxf(-maxStepM,
                                           fminf(maxStepM, target->heightAboveTakeoffM - verticalSetpoint));
                command.z = verticalSetpoint;
            } else {
                command.x = M400ClampPositionAxis(northM, horizontalLimitM);
                command.y = M400ClampPositionAxis(eastM, horizontalLimitM);
                command.z = target->heightAboveTakeoffM;
            }
        }
        rc = DjiFlightController_ExecuteJoystickAction(command);
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) break;

        const bool yawReady = yawErrorDeg <= target->yawToleranceDeg &&
                              (!officialStyle ?
                               (newYaw && haveYawRate &&
                                fabsf(filteredYawRateDegS) <= M400_YAW_RATE_TOL_DEG_S &&
                                fabsf(command.yaw) <= 1.0f) : true);
        bool stageReady = false;
        if (stage == 0) {
            stageReady = yawReady;
        } else if (stage == 1) {
            stageReady = yawReady && fabsf(verticalErrorM) <= target->verticalToleranceM &&
                         fabsf(velocity.data.z) <= M400_POSITION_SPEED_TOL_MPS;
        } else {
            stageReady = yawReady && horizontalErrorM <= target->horizontalToleranceM &&
                         fabsf(verticalErrorM) <= target->verticalToleranceM &&
                         horizontalSpeedMps <= M400_POSITION_SPEED_TOL_MPS &&
                         fabsf(velocity.data.z) <= M400_POSITION_SPEED_TOL_MPS;
        }
        if (stageReady && (newPosition || newYaw)) {
            if (withinThresholdSinceMs == 0) withinThresholdSinceMs = nowMs;
            if (nowMs - withinThresholdSinceMs >= M400_POSITION_HOLD_MS) {
                if (stage == 0) {
                    stage = 1;
                    M400SetResult("Staged goto F%.0f 2/3: altitude", horizontalLimitM);
                } else if (stage == 1) {
                    stage = 2;
                    M400SetResult("Staged goto F%.0f 3/3: horizontal", horizontalLimitM);
                } else {
                    finished = true;
                    break;
                }
                withinThresholdSinceMs = 0;
            }
        } else if (newPosition || newYaw) {
            withinThresholdSinceMs = 0;
        }
        usleep(M400_CONTROL_PERIOD_MS * 1000);
    }
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && !finished) rc = DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT;

    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        // Hold the final position and heading without a mode switch before release.
        command.x = command.y = 0.0f;
        command.z = target->heightAboveTakeoffM;
        if (officialStyle) command.yaw = targetYaw;
        for (unsigned int elapsed = 0; elapsed < 500; elapsed += M400_CONTROL_PERIOD_MS) {
            if (M400AbortRequested()) {
                rc = DJI_ERROR_SYSTEM_MODULE_CODE_EXECUTING_HIGHER_PRIORITY_TASK;
                break;
            }
            if (!officialStyle) {
                const float maxChange = M400_YAW_ACCEL_DEG_S2 * M400_CONTROL_PERIOD_MS / 1000.0f;
                command.yaw += fmaxf(-maxChange, fminf(maxChange, -command.yaw));
            }
            rc = DjiFlightController_ExecuteJoystickAction(command);
            if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) break;
            usleep(M400_CONTROL_PERIOD_MS * 1000);
        }
    }
release:
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) (void) M400SendStableZero(500);
    T_DjiReturnCode releaseRc = DjiFlightController_ReleaseJoystickCtrlAuthority();
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) rc = releaseRc;
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        M400SetCommandState(M400_COMMAND_STATE_SUCCESS);
        if (officialStyle) M400SetResult("Official-style goto F2 arrived: horizontal error %.2fm",
                                        lastHorizontalErrorM);
        else M400SetResult("Staged goto F%.0f arrived: horizontal error %.2fm",
                           horizontalLimitM, lastHorizontalErrorM);
    } else if (M400AbortRequested()) {
        M400SetCommandState(M400_COMMAND_STATE_EMERGENCY);
        M400SetResult("Goto interrupted by emergency hover");
    } else if (failureDetail != NULL) {
        M400SetCommandState(M400_COMMAND_STATE_FAILED);
        M400SetResult("Goto stopped: %s", failureDetail);
    } else {
        M400SetCommandState(M400_COMMAND_STATE_FAILED);
        M400SetResult("Goto failed: 0x%08llX", (unsigned long long) rc);
    }
exit:
    pthread_mutex_unlock(&s_controlMutex);
    return rc;
}

T_DjiReturnCode M400Control_GotoCoordinate(const T_M400GotoTarget *target)
{
    return M400RunGotoCoordinate(target, false, M400_POSITION_COMMAND_LIMIT_M);
}

T_DjiReturnCode M400Control_GotoCoordinateWithLimit(const T_M400GotoTarget *target,
                                                   float horizontalPositionLimitM)
{
    return M400RunGotoCoordinate(target, false, horizontalPositionLimitM);
}

T_DjiReturnCode M400Control_GotoCoordinateOfficialDemo(const T_M400GotoTarget *target)
{
    return M400RunGotoCoordinate(target, true, M400_POSITION_COMMAND_LIMIT_M);
}

static T_DjiReturnCode M400BuildFixedTestTarget(T_M400GotoTarget *target, double *distanceM)
{
    T_DjiReturnCode rc = M400CheckAircraftInAir();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return rc;
    T_DjiFcSubscriptionPositionFused position;
    T_DjiFcSubscriptionHeightRelative height;
    float currentYaw;
    if (!M400ReadMoveOrigin(&position, &height, &currentYaw)) {
        M400SetResult("Fixed goto denied: positioning unavailable");
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }
    double northM, eastM;
    M400OffsetToTarget(M400_TEST_LATITUDE_DEG / M400_DEG_PER_RAD,
                       M400_TEST_LONGITUDE_DEG / M400_DEG_PER_RAD,
                       &position, &northM, &eastM);
    *distanceM = hypot(northM, eastM);
    if (*distanceM > M400_GOTO_MAX_DISTANCE_M) {
        M400SetResult("Fixed goto denied: %.1fm exceeds 100m test limit", *distanceM);
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    const float bearingDeg = *distanceM > 0.5 ?
                             (float) (atan2(eastM, northM) * M400_DEG_PER_RAD) : currentYaw;
    *target = (T_M400GotoTarget) {
        .latitudeDeg = M400_TEST_LATITUDE_DEG,
        .longitudeDeg = M400_TEST_LONGITUDE_DEG,
        .heightAboveTakeoffM = M400_TEST_HEIGHT_M,
        .yawDeg = bearingDeg,
        .horizontalToleranceM = 0.6f,
        .verticalToleranceM = 0.4f,
        .yawToleranceDeg = 3.0f,
        .timeoutMs = 120000U,
    };
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode M400Control_EmergencyHover(void)
{
    M400SetAbortRequested(true);
    pthread_mutex_lock(&s_controlMutex);
    T_DjiReturnCode rc = M400CheckAircraftInAir();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        M400SetResult("Emergency hover: aircraft already on ground");
        M400SetAbortRequested(false);
        pthread_mutex_unlock(&s_controlMutex);
        return rc;
    }
    rc = M400ObtainAuthority();
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        rc = M400SendStableZero(M400_HOVER_HOLD_MS);
        T_DjiReturnCode releaseRc = DjiFlightController_ReleaseJoystickCtrlAuthority();
        if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) rc = releaseRc;
    }
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        M400SetResult("EMERGENCY HOVER OK; RC authority restored");
    } else {
        M400SetResult("EMERGENCY HOVER failed: 0x%08llX", (unsigned long long) rc);
    }
    M400SetAbortRequested(false);
    pthread_mutex_unlock(&s_controlMutex);
    return rc;
}

static T_DjiReturnCode M400QueueCommand(E_M400QueuedCommand command)
{
    const bool fixedGoto = command == M400_COMMAND_GOTO_STAGED ||
                           command == M400_COMMAND_GOTO_STAGED_10 ||
                           command == M400_COMMAND_GOTO_OFFICIAL;
    double previewDistanceM = 0.0;
    if (fixedGoto) {
        T_M400GotoTarget previewTarget;
        T_DjiReturnCode previewRc = M400BuildFixedTestTarget(&previewTarget, &previewDistanceM);
        if (previewRc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return previewRc;
    }
    pthread_mutex_lock(&s_queueMutex);
    if (command == M400_COMMAND_EMERGENCY_HOVER) {
        M400SetAbortRequested(true);
        s_pendingCommand = command;
        s_armedGotoCommand = M400_COMMAND_NONE;
        s_gotoArmUntilMs = 0;
        pthread_mutex_unlock(&s_queueMutex);
        M400SetCommandState(M400_COMMAND_STATE_EMERGENCY);
        M400SetResult("Emergency hover queued");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    if (s_pendingCommand != M400_COMMAND_NONE || pthread_mutex_trylock(&s_controlMutex) != 0) {
        pthread_mutex_unlock(&s_queueMutex);
        M400SetCommandState(M400_COMMAND_STATE_REJECTED);
        M400SetResult("Command rejected: controller busy");
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }
    pthread_mutex_unlock(&s_controlMutex);
    if (fixedGoto &&
        (s_armedGotoCommand != command || M400GetMonotonicMs() > s_gotoArmUntilMs)) {
        s_armedGotoCommand = command;
        s_gotoArmUntilMs = M400GetMonotonicMs() + M400_GOTO_CONFIRM_MS;
        pthread_mutex_unlock(&s_queueMutex);
        M400SetCommandState(M400_COMMAND_STATE_IDLE);
        M400SetResult("%s 22.6025495,113.9917210 H20, %.0fm: press again <10s",
                      command == M400_COMMAND_GOTO_STAGED ? "STAGED F2" :
                      command == M400_COMMAND_GOTO_STAGED_10 ? "STAGED F10" : "DEMO F2",
                      previewDistanceM);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    s_armedGotoCommand = M400_COMMAND_NONE;
    s_gotoArmUntilMs = 0;
    s_pendingCommand = command;
    pthread_mutex_unlock(&s_queueMutex);
    M400SetCommandState(M400_COMMAND_STATE_QUEUED);
    M400SetResult("Command queued");
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static void *M400ControlTask(void *arg)
{
    (void) arg;
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    while (true) {
        E_M400QueuedCommand command = M400_COMMAND_NONE;
        T_DjiReturnCode commandRc = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        pthread_mutex_lock(&s_queueMutex);
        if (s_pendingCommand != M400_COMMAND_NONE) {
            command = s_pendingCommand;
            s_pendingCommand = M400_COMMAND_NONE;
        }
        pthread_mutex_unlock(&s_queueMutex);

        if (command != M400_COMMAND_NONE && command != M400_COMMAND_EMERGENCY_HOVER) {
            M400SetCommandState(M400_COMMAND_STATE_RUNNING);
        }
        switch (command) {
            case M400_COMMAND_FORWARD: commandRc = M400Control_MoveRelative(M400_MOVE_FORWARD, 1.0f); break;
            case M400_COMMAND_BACKWARD: commandRc = M400Control_MoveRelative(M400_MOVE_BACKWARD, 1.0f); break;
            case M400_COMMAND_LEFT: commandRc = M400Control_MoveRelative(M400_MOVE_LEFT, 1.0f); break;
            case M400_COMMAND_RIGHT: commandRc = M400Control_MoveRelative(M400_MOVE_RIGHT, 1.0f); break;
            case M400_COMMAND_YAW_LEFT: commandRc = M400Control_YawRelative(-90.0f); break;
            case M400_COMMAND_YAW_RIGHT: commandRc = M400Control_YawRelative(90.0f); break;
            case M400_COMMAND_UP: commandRc = M400Control_MoveVertical(1.0f); break;
            case M400_COMMAND_DOWN: commandRc = M400Control_MoveVertical(-1.0f); break;
            case M400_COMMAND_FORWARD_5M: commandRc = M400Control_MoveRelative(M400_MOVE_FORWARD, 5.0f); break;
            case M400_COMMAND_GOTO_STAGED:
            case M400_COMMAND_GOTO_STAGED_10:
            case M400_COMMAND_GOTO_OFFICIAL: {
                T_M400GotoTarget target;
                double distanceM;
                commandRc = M400BuildFixedTestTarget(&target, &distanceM);
                if (commandRc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    commandRc = command == M400_COMMAND_GOTO_STAGED ?
                                M400Control_GotoCoordinate(&target) :
                                command == M400_COMMAND_GOTO_STAGED_10 ?
                                M400Control_GotoCoordinateWithLimit(&target, M400_POSITION_COMMAND_TEST_LIMIT_M) :
                                M400Control_GotoCoordinateOfficialDemo(&target);
                }
                break;
            }
            case M400_COMMAND_EMERGENCY_HOVER: commandRc = M400Control_EmergencyHover(); break;
            default: break;
        }
        if (command == M400_COMMAND_EMERGENCY_HOVER) {
            M400SetCommandState(commandRc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ?
                               M400_COMMAND_STATE_EMERGENCY : M400_COMMAND_STATE_FAILED);
        } else if (command != M400_COMMAND_NONE) {
            if (commandRc == DJI_ERROR_SYSTEM_MODULE_CODE_EXECUTING_HIGHER_PRIORITY_TASK &&
                M400AbortRequested()) {
                M400SetCommandState(M400_COMMAND_STATE_EMERGENCY);
            } else if (commandRc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                M400SetCommandState(M400_COMMAND_STATE_SUCCESS);
            } else if (commandRc == DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE ||
                       commandRc == DJI_ERROR_SYSTEM_MODULE_CODE_BUSY) {
                M400SetCommandState(M400_COMMAND_STATE_REJECTED);
            } else {
                M400SetCommandState(M400_COMMAND_STATE_FAILED);
            }
        }
        osal->TaskSleepMs(M400_CONTROL_PERIOD_MS);
    }
    return NULL;
}

T_DjiReturnCode M400Control_StartRecording(void)
{
    if (mkdir("data", 0755) != 0 && errno != EEXIST) {
        M400SetResult("Create data directory failed: %d", errno);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    if (mkdir(M400_LOG_DIRECTORY, 0755) != 0 && errno != EEXIST) {
        M400SetResult("Create log directory failed: %d", errno);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    pthread_mutex_lock(&s_stateMutex);
    if (s_recording) {
        pthread_mutex_unlock(&s_stateMutex);
        M400SetResult("Recording already running");
        return DJI_ERROR_SYSTEM_MODULE_CODE_DUPLICATE;
    }

    time_t now = time(NULL);
    struct tm localTime;
    localtime_r(&now, &localTime);
    char filePath[256];
    snprintf(s_currentCsvFile, sizeof(s_currentCsvFile), "%04d%02d%02d_%02d%02d%02d_Debug.csv",
             localTime.tm_year + 1900, localTime.tm_mon + 1, localTime.tm_mday,
             localTime.tm_hour, localTime.tm_min, localTime.tm_sec);
    snprintf(filePath, sizeof(filePath), "%s/%s", M400_LOG_DIRECTORY, s_currentCsvFile);
    for (unsigned int suffix = 1; access(filePath, F_OK) == 0 && suffix < 100; ++suffix) {
        snprintf(s_currentCsvFile, sizeof(s_currentCsvFile), "%04d%02d%02d_%02d%02d%02d_Debug_%02u.csv",
                 localTime.tm_year + 1900, localTime.tm_mon + 1, localTime.tm_mday,
                 localTime.tm_hour, localTime.tm_min, localTime.tm_sec, suffix);
        snprintf(filePath, sizeof(filePath), "%s/%s", M400_LOG_DIRECTORY, s_currentCsvFile);
    }

    s_csvFile = fopen(filePath, "w");
    if (s_csvFile == NULL) {
        int openError = errno;
        pthread_mutex_unlock(&s_stateMutex);
        M400SetResult("Open CSV failed: %d", openError);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    fprintf(s_csvFile,
            "host_unix_time_s,uav_timestamp_s,quaternion_q0,quaternion_q1,quaternion_q2,quaternion_q3,"
            "velocity_x_mps,velocity_y_mps,velocity_z_mps,fused_longitude_rad,fused_latitude_rad,"
            "fused_altitude_m,fused_visible_satellites,rtk_longitude_deg,rtk_latitude_deg,rtk_hfsl_m,"
            "rtk_position_info,rtk_connected,vo_x_m,vo_y_m,vo_z_m,vo_x_health,vo_y_health,vo_z_health,"
            "flight_status,gps_date_yyyymmdd,gps_time_hhmmss\n");
    fflush(s_csvFile);
    s_recording = true;
    pthread_mutex_unlock(&s_stateMutex);
    M400RefreshCsvFileList();
    M400SetResult("Recording started");
    USER_LOG_INFO("Recording started: %s", filePath);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode M400Control_StopRecording(void)
{
    char stoppedFile[M400_FILENAME_LENGTH];
    pthread_mutex_lock(&s_stateMutex);
    if (!s_recording) {
        pthread_mutex_unlock(&s_stateMutex);
        M400SetResult("Recording already stopped");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    snprintf(stoppedFile, sizeof(stoppedFile), "%s", s_currentCsvFile);
    fflush(s_csvFile);
    fclose(s_csvFile);
    s_csvFile = NULL;
    s_recording = false;
    pthread_mutex_unlock(&s_stateMutex);
    M400RefreshCsvFileList();
    M400SetResult("Recording stopped");
    USER_LOG_INFO("Recording stopped: %s", stoppedFile);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode M400WidgetSetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t value, void *userData)
{
    (void) userData;
    if (index >= sizeof(s_widgetValues) / sizeof(s_widgetValues[0])) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    if (index == M400_WIDGET_RTK || index == M400_WIDGET_LONGITUDE || index == M400_WIDGET_LATITUDE ||
        index == M400_WIDGET_COMMAND_STATE || index == M400_WIDGET_CSV_DATE || index == M400_WIDGET_CSV_TIME) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    s_widgetValues[index] = value;
    if (widgetType == DJI_WIDGET_TYPE_BUTTON && value != DJI_WIDGET_BUTTON_STATE_PRESS_DOWN) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    if (widgetType == DJI_WIDGET_TYPE_BUTTON) {
        M400Pi4Bridge_PublishEvent("BUTTON", index, s_widgetEventNames[index], "PRESS_DOWN");
        pthread_mutex_lock(&s_stateMutex);
        s_buttonFeedbackUntilMs[index] = M400GetMonotonicMs() +
                                         (index == M400_WIDGET_EMERGENCY_HOVER ? 2500ULL : 1000ULL);
        pthread_mutex_unlock(&s_stateMutex);
    } else if (widgetType == DJI_WIDGET_TYPE_SWITCH && index == M400_WIDGET_RECORD_SWITCH) {
        M400Pi4Bridge_PublishEvent("SWITCH", index, s_widgetEventNames[index],
                                   value == DJI_WIDGET_SWITCH_STATE_ON ? "ON" : "OFF");
    } else if (widgetType == DJI_WIDGET_TYPE_LIST && index == M400_WIDGET_CSV_SELECT) {
        char selectedIndex[16];
        snprintf(selectedIndex, sizeof(selectedIndex), "%d", value);
        M400Pi4Bridge_PublishEvent("LIST", index, s_widgetEventNames[index], selectedIndex);
    }

    switch (index) {
        case M400_WIDGET_START_RECORD: return M400Control_StartRecording();
        case M400_WIDGET_STOP_RECORD: return M400Control_StopRecording();
        case M400_WIDGET_CSV_SELECT:
            M400RefreshCsvFileList();
            pthread_mutex_lock(&s_stateMutex);
            if (s_csvFileCount == 0) {
                snprintf(s_lastResult, sizeof(s_lastResult), "No CSV files found");
            } else {
                s_selectedCsvIndex = (size_t) value < s_csvFileCount ? (size_t) value : 0;
                snprintf(s_lastResult, sizeof(s_lastResult), "CSV selected; see Payload Settings");
            }
            pthread_mutex_unlock(&s_stateMutex);
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        case M400_WIDGET_FORWARD_1M: return M400QueueCommand(M400_COMMAND_FORWARD);
        case M400_WIDGET_BACKWARD_1M: return M400QueueCommand(M400_COMMAND_BACKWARD);
        case M400_WIDGET_LEFT_1M: return M400QueueCommand(M400_COMMAND_LEFT);
        case M400_WIDGET_RIGHT_1M: return M400QueueCommand(M400_COMMAND_RIGHT);
        case M400_WIDGET_EMERGENCY_HOVER: return M400QueueCommand(M400_COMMAND_EMERGENCY_HOVER);
        case M400_WIDGET_YAW_LEFT: return M400QueueCommand(M400_COMMAND_YAW_LEFT);
        case M400_WIDGET_YAW_RIGHT: return M400QueueCommand(M400_COMMAND_YAW_RIGHT);
        case M400_WIDGET_UP_1M: return M400QueueCommand(M400_COMMAND_UP);
        case M400_WIDGET_DOWN_1M: return M400QueueCommand(M400_COMMAND_DOWN);
        case M400_WIDGET_FORWARD_5M: return M400QueueCommand(M400_COMMAND_FORWARD_5M);
        case M400_WIDGET_GOTO_STAGED_2: return M400QueueCommand(M400_COMMAND_GOTO_STAGED);
        case M400_WIDGET_GOTO_OFFICIAL_2: return M400QueueCommand(M400_COMMAND_GOTO_OFFICIAL);
        case M400_WIDGET_GOTO_STAGED_10: return M400QueueCommand(M400_COMMAND_GOTO_STAGED_10);
        case M400_WIDGET_RECORD_SWITCH:
            return value == DJI_WIDGET_SWITCH_STATE_ON ?
                   M400Control_StartRecording() : M400Control_StopRecording();
        default: return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
}

static T_DjiReturnCode M400WidgetGetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t *value, void *userData)
{
    (void) widgetType;
    (void) userData;
    if (index >= sizeof(s_widgetValues) / sizeof(s_widgetValues[0]) || value == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    if (widgetType == DJI_WIDGET_TYPE_BUTTON &&
        (index == M400_WIDGET_START_RECORD || index == M400_WIDGET_STOP_RECORD ||
         index == M400_WIDGET_EMERGENCY_HOVER || index == M400_WIDGET_UP_1M ||
         index == M400_WIDGET_DOWN_1M || index == M400_WIDGET_FORWARD_5M ||
         index == M400_WIDGET_GOTO_STAGED_2 || index == M400_WIDGET_GOTO_STAGED_10 ||
         index == M400_WIDGET_GOTO_OFFICIAL_2)) {
        bool recording;
        uint64_t feedbackUntilMs;
        pthread_mutex_lock(&s_stateMutex);
        recording = s_recording;
        feedbackUntilMs = s_buttonFeedbackUntilMs[index];
        pthread_mutex_unlock(&s_stateMutex);
        bool selected = M400GetMonotonicMs() < feedbackUntilMs;
        if (index == M400_WIDGET_START_RECORD && recording) {
            selected = true;
        }
        *value = selected ? DJI_WIDGET_BUTTON_STATE_PRESS_DOWN : DJI_WIDGET_BUTTON_STATE_RELEASE_UP;
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    if (index == M400_WIDGET_RECORD_SWITCH || index == M400_WIDGET_RTK ||
        index == M400_WIDGET_LONGITUDE || index == M400_WIDGET_LATITUDE ||
        index == M400_WIDGET_COMMAND_STATE || index == M400_WIDGET_CSV_DATE ||
        index == M400_WIDGET_CSV_TIME) {
        T_M400Telemetry telemetry;
        bool recording;
        E_M400CommandState commandState;
        char selectedFile[M400_FILENAME_LENGTH] = "";
        pthread_mutex_lock(&s_stateMutex);
        telemetry = s_telemetry;
        recording = s_recording;
        commandState = s_commandState;
        if (s_csvFileCount > 0 && s_selectedCsvIndex < s_csvFileCount) {
            snprintf(selectedFile, sizeof(selectedFile), "%s", s_csvFiles[s_selectedCsvIndex]);
        }
        pthread_mutex_unlock(&s_stateMutex);
        if (index == M400_WIDGET_RTK) {
            *value = M400RtkStatusWidgetValue(&telemetry);
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        if (index == M400_WIDGET_LONGITUDE || index == M400_WIDGET_LATITUDE) {
            double longitude;
            double latitude;
            M400GetDisplayPosition(&telemetry, &longitude, &latitude);
            *value = (int32_t) llround((index == M400_WIDGET_LONGITUDE ? longitude : latitude) * 10000000.0);
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        if (index == M400_WIDGET_RECORD_SWITCH) {
            *value = recording ? DJI_WIDGET_SWITCH_STATE_ON : DJI_WIDGET_SWITCH_STATE_OFF;
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        if (index == M400_WIDGET_COMMAND_STATE) {
            *value = commandState;
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        if (index == M400_WIDGET_CSV_DATE || index == M400_WIDGET_CSV_TIME) {
            unsigned int fileDate = 0;
            unsigned int fileTime = 0;
            if (sscanf(selectedFile, "%8u_%6u", &fileDate, &fileTime) != 2) {
                fileDate = 0;
                fileTime = 0;
            }
            *value = (int32_t) (index == M400_WIDGET_CSV_DATE ? fileDate : fileTime);
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
    }
    *value = s_widgetValues[index];
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode M400ControlWidget_Init(void)
{
    T_DjiReturnCode rc = DjiWidget_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return rc;
    rc = DjiWidget_RegDefaultUiConfigByDirPath(M400_WIDGET_EN_DIRECTORY);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return rc;
    rc = DjiWidget_RegUiConfigByDirPath(DJI_MOBILE_APP_LANGUAGE_ENGLISH,
                                        DJI_MOBILE_APP_SCREEN_TYPE_BIG_SCREEN,
                                        M400_WIDGET_EN_DIRECTORY);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return rc;
    rc = DjiWidget_RegUiConfigByDirPath(DJI_MOBILE_APP_LANGUAGE_CHINESE,
                                        DJI_MOBILE_APP_SCREEN_TYPE_BIG_SCREEN,
                                        M400_WIDGET_CN_DIRECTORY);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return rc;
    return DjiWidget_RegHandlerList(s_widgetHandlers,
                                    sizeof(s_widgetHandlers) / sizeof(s_widgetHandlers[0]));
}

T_DjiReturnCode M400Control_StartService(void)
{
    if (s_serviceStarted) return DJI_ERROR_SYSTEM_MODULE_CODE_DUPLICATE;
    if (mkdir("data", 0755) != 0 && errno != EEXIST) return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    if (mkdir(M400_LOG_DIRECTORY, 0755) != 0 && errno != EEXIST) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    T_DjiReturnCode rc = DjiFcSubscription_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && rc != DJI_ERROR_SYSTEM_MODULE_CODE_DUPLICATE) return rc;
    rc = M400SubscribeRequiredTopics();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return rc;

    M400RefreshCsvFileList();
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    T_DjiTaskHandle telemetryTask;
    T_DjiTaskHandle statusTask;
    T_DjiTaskHandle controlTask;
    if (osal->TaskCreate("m400_telemetry", M400TelemetryTask, 4096, NULL, &telemetryTask) !=
        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    if (osal->TaskCreate("m400_widget_status", M400StatusTask, 4096, NULL, &statusTask) !=
        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    if (osal->TaskCreate("m400_control", M400ControlTask, 4096, NULL, &controlTask) !=
        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    if (M400Pi4Bridge_Start(M400_PI4_LISTEN_INTERFACE, M400_PI4_ALLOWED_PEER,
                            M400_PI4_LISTEN_PORT, M400OnPi4Message) != 0) {
        USER_LOG_ERROR("Start Pi4 TCP listener failed");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    s_serviceStarted = true;
    USER_LOG_INFO("M400 control service started. Pi4 TCP configured on port %d, %s; recording OFF.",
                  M400_PI4_LISTEN_PORT, M400_PI4_LISTEN_INTERFACE);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
