#ifndef M400_WAYPOINT_KMZ_H
#define M400_WAYPOINT_KMZ_H

#include <stdint.h>

typedef struct {
    double startLatitudeDeg;
    double startLongitudeDeg;
    double endLatitudeDeg;
    double endLongitudeDeg;
    float startHeightM;
    float endHeightM;
    float cruiseSpeedMps;
    float safeTakeoffHeightM;
    float endYawDeg;
} T_M400WaypointKmzSpec;

/* Builds an uncompressed WPML KMZ in memory. Caller owns *data and must free it. */
int M400WaypointKmz_Build(const T_M400WaypointKmzSpec *spec, uint8_t **data, uint32_t *size);

#endif
