#ifndef M400_PI4_BRIDGE_H
#define M400_PI4_BRIDGE_H

#include <stdint.h>

#define M400_PI4_MESSAGE_MAX_BYTES 240

/* The callback receives a single printable UTF-8 line, up to 240 bytes. */
typedef void (*M400Pi4MessageCallback)(const char *message);

/* Starts a detached TCP listener. It retries while the network interface is down. */
int M400Pi4Bridge_Start(const char *interfaceName, const char *allowedPeerIp,
                        uint16_t port, M400Pi4MessageCallback callback);

/* Sends one Pilot widget event to the Pi4 subscriber, if connected. */
void M400Pi4Bridge_PublishEvent(const char *type, uint32_t index,
                                const char *name, const char *state);

#endif
