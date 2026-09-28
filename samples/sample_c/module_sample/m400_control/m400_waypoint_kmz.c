#include "m400_waypoint_kmz.h"

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define M400_XML_CAPACITY 8192U
#define M400_ZIP_ENTRY_COUNT 2U

static int M400Append(char *buffer, size_t capacity, size_t *used, const char *format, ...)
{
    if (*used >= capacity) return -1;
    va_list args;
    va_start(args, format);
    int written = vsnprintf(buffer + *used, capacity - *used, format, args);
    va_end(args);
    if (written < 0 || (size_t) written >= capacity - *used) return -1;
    *used += (size_t) written;
    return 0;
}

static int M400BuildXml(const T_M400WaypointKmzSpec *spec, bool executable,
                        char *xml, size_t *length)
{
    size_t used = 0;
#define ADD(...) do { if (M400Append(xml, M400_XML_CAPACITY, &used, __VA_ARGS__) != 0) return -1; } while (0)
    ADD("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        "<kml xmlns=\"http://www.opengis.net/kml/2.2\" "
        "xmlns:wpml=\"http://www.dji.com/wpmz/1.0.6\"><Document>\n");
    ADD("<wpml:missionConfig>"
        "<wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>"
        "<wpml:finishAction>noAction</wpml:finishAction>"
        "<wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>"
        "<wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>"
        "<wpml:takeOffSecurityHeight>%.1f</wpml:takeOffSecurityHeight>"
        "<wpml:globalTransitionalSpeed>%.1f</wpml:globalTransitionalSpeed>"
        "<wpml:globalRTHHeight>50</wpml:globalRTHHeight>"
        "<wpml:droneInfo><wpml:droneEnumValue>103</wpml:droneEnumValue>"
        "<wpml:droneSubEnumValue>0</wpml:droneSubEnumValue></wpml:droneInfo>"
        "</wpml:missionConfig><Folder>\n",
        spec->safeTakeoffHeightM, spec->cruiseSpeedMps);
    if (executable) {
        ADD("<wpml:templateId>0</wpml:templateId>"
            "<wpml:executeHeightMode>relativeToStartPoint</wpml:executeHeightMode>"
            "<wpml:waylineId>0</wpml:waylineId>"
            "<wpml:autoFlightSpeed>%.1f</wpml:autoFlightSpeed>\n", spec->cruiseSpeedMps);
    } else {
        ADD("<wpml:templateType>waypoint</wpml:templateType>"
            "<wpml:templateId>0</wpml:templateId>"
            "<wpml:waylineCoordinateSysParam>"
            "<wpml:coordinateMode>WGS84</wpml:coordinateMode>"
            "<wpml:heightMode>relativeToStartPoint</wpml:heightMode>"
            "<wpml:positioningType>GPS</wpml:positioningType>"
            "</wpml:waylineCoordinateSysParam>"
            "<wpml:autoFlightSpeed>%.1f</wpml:autoFlightSpeed>"
            "<wpml:globalHeight>%.2f</wpml:globalHeight>"
            "<wpml:gimbalPitchMode>manual</wpml:gimbalPitchMode>"
            "<wpml:globalWaypointHeadingParam>"
            "<wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>"
            "</wpml:globalWaypointHeadingParam>"
            "<wpml:globalWaypointTurnMode>toPointAndStopWithDiscontinuityCurvature"
            "</wpml:globalWaypointTurnMode>"
            "<wpml:globalUseStraightLine>1</wpml:globalUseStraightLine>\n",
            spec->cruiseSpeedMps, spec->startHeightM);
    }
    for (int index = 0; index < 2; ++index) {
        const double latitude = index == 0 ? spec->startLatitudeDeg : spec->endLatitudeDeg;
        const double longitude = index == 0 ? spec->startLongitudeDeg : spec->endLongitudeDeg;
        const float height = index == 0 ? spec->startHeightM : spec->endHeightM;
        ADD("<Placemark><Point><coordinates>%.10f,%.10f</coordinates></Point>"
            "<wpml:index>%d</wpml:index>", longitude, latitude, index);
        if (executable) {
            ADD("<wpml:executeHeight>%.2f</wpml:executeHeight>"
                "<wpml:waypointSpeed>%.1f</wpml:waypointSpeed>"
                "<wpml:waypointHeadingParam>"
                "<wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>"
                "</wpml:waypointHeadingParam>"
                "<wpml:waypointTurnParam>"
                "<wpml:waypointTurnMode>toPointAndStopWithDiscontinuityCurvature"
                "</wpml:waypointTurnMode>"
                "<wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist>"
                "</wpml:waypointTurnParam>"
                "<wpml:useStraightLine>1</wpml:useStraightLine>",
                height, spec->cruiseSpeedMps);
        } else {
            ADD("<wpml:ellipsoidHeight>%.2f</wpml:ellipsoidHeight>"
                "<wpml:height>%.2f</wpml:height>"
                "<wpml:useGlobalHeight>0</wpml:useGlobalHeight>"
                "<wpml:useGlobalSpeed>1</wpml:useGlobalSpeed>"
                "<wpml:useGlobalHeadingParam>1</wpml:useGlobalHeadingParam>"
                "<wpml:useGlobalTurnParam>1</wpml:useGlobalTurnParam>"
                "<wpml:useStraightLine>1</wpml:useStraightLine>", height, height);
        }
        if (index == 1) {
            ADD("<wpml:actionGroup>"
                "<wpml:actionGroupId>0</wpml:actionGroupId>"
                "<wpml:actionGroupStartIndex>1</wpml:actionGroupStartIndex>"
                "<wpml:actionGroupEndIndex>1</wpml:actionGroupEndIndex>"
                "<wpml:actionGroupMode>sequence</wpml:actionGroupMode>"
                "<wpml:actionTrigger><wpml:actionTriggerType>reachPoint"
                "</wpml:actionTriggerType></wpml:actionTrigger>"
                "<wpml:action><wpml:actionId>0</wpml:actionId>"
                "<wpml:actionActuatorFunc>rotateYaw</wpml:actionActuatorFunc>"
                "<wpml:actionActuatorFuncParam><wpml:aircraftHeading>%.1f"
                "</wpml:aircraftHeading></wpml:actionActuatorFuncParam>"
                "</wpml:action></wpml:actionGroup>", spec->endYawDeg);
        }
        ADD("</Placemark>\n");
    }
    ADD("</Folder></Document></kml>\n");
#undef ADD
    *length = used;
    return 0;
}

static void M400Put16(uint8_t *buffer, size_t offset, uint16_t value)
{
    buffer[offset] = (uint8_t) value;
    buffer[offset + 1] = (uint8_t) (value >> 8);
}

static void M400Put32(uint8_t *buffer, size_t offset, uint32_t value)
{
    M400Put16(buffer, offset, (uint16_t) value);
    M400Put16(buffer, offset + 2, (uint16_t) (value >> 16));
}

static uint32_t M400Crc32(const uint8_t *bytes, size_t length)
{
    uint32_t crc = 0xffffffffU;
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
        }
    }
    return ~crc;
}

int M400WaypointKmz_Build(const T_M400WaypointKmzSpec *spec, uint8_t **data, uint32_t *size)
{
    if (data == NULL || size == NULL || spec == NULL ||
        !isfinite(spec->startLatitudeDeg) || !isfinite(spec->startLongitudeDeg) ||
        !isfinite(spec->endLatitudeDeg) || !isfinite(spec->endLongitudeDeg) ||
        !isfinite(spec->startHeightM) || !isfinite(spec->endHeightM) ||
        !isfinite(spec->cruiseSpeedMps) || !isfinite(spec->safeTakeoffHeightM) ||
        !isfinite(spec->endYawDeg) ||
        fabs(spec->startLatitudeDeg) > 90.0 || fabs(spec->endLatitudeDeg) > 90.0 ||
        fabs(spec->startLongitudeDeg) > 180.0 || fabs(spec->endLongitudeDeg) > 180.0 ||
        spec->cruiseSpeedMps < 1.0f || spec->cruiseSpeedMps > 10.0f ||
        fabsf(spec->endYawDeg) > 180.0f) return -1;
    *data = NULL;
    *size = 0;
    char *templateXml = (char *) malloc(M400_XML_CAPACITY);
    char *waylinesXml = (char *) malloc(M400_XML_CAPACITY);
    if (templateXml == NULL || waylinesXml == NULL) {
        free(templateXml);
        free(waylinesXml);
        return -1;
    }
    size_t templateLength = 0, waylinesLength = 0;
    if (M400BuildXml(spec, false, templateXml, &templateLength) != 0 ||
        M400BuildXml(spec, true, waylinesXml, &waylinesLength) != 0) {
        free(templateXml);
        free(waylinesXml);
        return -1;
    }
    const char *names[M400_ZIP_ENTRY_COUNT] = {"wpmz/template.kml", "wpmz/waylines.wpml"};
    const uint8_t *contents[M400_ZIP_ENTRY_COUNT] = {
        (const uint8_t *) templateXml, (const uint8_t *) waylinesXml
    };
    const size_t lengths[M400_ZIP_ENTRY_COUNT] = {templateLength, waylinesLength};
    uint32_t checksums[M400_ZIP_ENTRY_COUNT], localOffsets[M400_ZIP_ENTRY_COUNT];
    size_t capacity = 22;
    for (size_t i = 0; i < M400_ZIP_ENTRY_COUNT; ++i) {
        capacity += 30 + strlen(names[i]) + lengths[i] + 46 + strlen(names[i]);
    }
    uint8_t *zip = (uint8_t *) calloc(1, capacity);
    if (zip == NULL) {
        free(templateXml);
        free(waylinesXml);
        return -1;
    }
    size_t offset = 0;
    for (size_t i = 0; i < M400_ZIP_ENTRY_COUNT; ++i) {
        size_t nameLength = strlen(names[i]);
        checksums[i] = M400Crc32(contents[i], lengths[i]);
        localOffsets[i] = (uint32_t) offset;
        M400Put32(zip, offset, 0x04034b50U);
        M400Put16(zip, offset + 4, 20);
        M400Put32(zip, offset + 14, checksums[i]);
        M400Put32(zip, offset + 18, (uint32_t) lengths[i]);
        M400Put32(zip, offset + 22, (uint32_t) lengths[i]);
        M400Put16(zip, offset + 26, (uint16_t) nameLength);
        offset += 30;
        memcpy(zip + offset, names[i], nameLength);
        offset += nameLength;
        memcpy(zip + offset, contents[i], lengths[i]);
        offset += lengths[i];
    }
    const uint32_t centralOffset = (uint32_t) offset;
    for (size_t i = 0; i < M400_ZIP_ENTRY_COUNT; ++i) {
        size_t nameLength = strlen(names[i]);
        M400Put32(zip, offset, 0x02014b50U);
        M400Put16(zip, offset + 4, 20);
        M400Put16(zip, offset + 6, 20);
        M400Put32(zip, offset + 16, checksums[i]);
        M400Put32(zip, offset + 20, (uint32_t) lengths[i]);
        M400Put32(zip, offset + 24, (uint32_t) lengths[i]);
        M400Put16(zip, offset + 28, (uint16_t) nameLength);
        M400Put32(zip, offset + 42, localOffsets[i]);
        offset += 46;
        memcpy(zip + offset, names[i], nameLength);
        offset += nameLength;
    }
    M400Put32(zip, offset, 0x06054b50U);
    M400Put16(zip, offset + 8, M400_ZIP_ENTRY_COUNT);
    M400Put16(zip, offset + 10, M400_ZIP_ENTRY_COUNT);
    M400Put32(zip, offset + 12, (uint32_t) offset - centralOffset);
    M400Put32(zip, offset + 16, centralOffset);
    offset += 22;
    *data = zip;
    *size = (uint32_t) offset;
    free(templateXml);
    free(waylinesXml);
    return 0;
}
