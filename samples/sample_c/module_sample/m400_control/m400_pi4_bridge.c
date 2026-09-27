#include "m400_pi4_bridge.h"

#include <arpa/inet.h>
#include <errno.h>
#include <ifaddrs.h>
#include <inttypes.h>
#include <net/if.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define M400_PI4_LINE_MAX 512
#define M400_PI4_MESSAGE_CAPACITY (M400_PI4_MESSAGE_MAX_BYTES + 1)
#define M400_PI4_TIME_POLL_INTERVAL_SEC 10

typedef struct {
    char interfaceName[IFNAMSIZ];
    struct in_addr allowedPeer;
    uint16_t port;
    M400Pi4MessageCallback callback;
} T_M400Pi4Bridge;

static T_M400Pi4Bridge s_bridge;
static bool s_started = false;
static pthread_mutex_t s_subscriberMutex = PTHREAD_MUTEX_INITIALIZER;
static int s_subscriber = -1;

static void M400Pi4CloseSubscriber(void)
{
    pthread_mutex_lock(&s_subscriberMutex);
    if (s_subscriber >= 0) {
        shutdown(s_subscriber, SHUT_RDWR);
        s_subscriber = -1;
    }
    pthread_mutex_unlock(&s_subscriberMutex);
}

static void M400Pi4SendSubscriberLine(const char *line, size_t length)
{
    pthread_mutex_lock(&s_subscriberMutex);
    if (s_subscriber >= 0 &&
        send(s_subscriber, line, length, MSG_NOSIGNAL | MSG_DONTWAIT) != (ssize_t) length) {
        shutdown(s_subscriber, SHUT_RDWR);
        s_subscriber = -1;
    }
    pthread_mutex_unlock(&s_subscriberMutex);
}

void M400Pi4Bridge_PublishEvent(const char *type, uint32_t index,
                                const char *name, const char *state)
{
    if (type == NULL || name == NULL || state == NULL) return;
    char line[128];
    int length = snprintf(line, sizeof(line), "%s\t%u\t%s\t%s\n", type, index, name, state);
    if (length <= 0 || (size_t) length >= sizeof(line)) return;
    M400Pi4SendSubscriberLine(line, (size_t) length);
}

static void M400Pi4SendClientLine(int client, const char *line)
{
    pthread_mutex_lock(&s_subscriberMutex);
    if (s_subscriber == client &&
        send(client, line, strlen(line), MSG_NOSIGNAL | MSG_DONTWAIT) != (ssize_t) strlen(line)) {
        shutdown(client, SHUT_RDWR);
        s_subscriber = -1;
    }
    pthread_mutex_unlock(&s_subscriberMutex);
}

static uint64_t M400Pi4TimestampNs(const struct timespec *timestamp)
{
    return (uint64_t) timestamp->tv_sec * 1000000000ULL + (uint64_t) timestamp->tv_nsec;
}

static void M400Pi4HandleTimeRequest(int client, const char *line, size_t length,
                                     const struct timespec *receivedAt)
{
    if (length <= 9 || length > 29 || memcmp(line, "TIME_REQ\t", 9) != 0) {
        M400Pi4SendClientLine(client, "TIME_ERR\tbad_request\n");
        return;
    }
    char origin[21];
    size_t originLength = length - 9;
    for (size_t i = 0; i < originLength; ++i) {
        if (line[9 + i] < '0' || line[9 + i] > '9') {
            M400Pi4SendClientLine(client, "TIME_ERR\tbad_request\n");
            return;
        }
    }
    memcpy(origin, line + 9, originLength);
    origin[originLength] = '\0';

    pthread_mutex_lock(&s_subscriberMutex);
    if (s_subscriber == client) {
        struct timespec transmittedAt;
        clock_gettime(CLOCK_REALTIME, &transmittedAt);
        char reply[128];
        int written = snprintf(reply, sizeof(reply), "TIME_RESP\t%s\t%" PRIu64 "\t%" PRIu64 "\n",
                               origin, M400Pi4TimestampNs(receivedAt),
                               M400Pi4TimestampNs(&transmittedAt));
        if (written <= 0 || (size_t) written >= sizeof(reply) ||
            send(client, reply, (size_t) written, MSG_NOSIGNAL | MSG_DONTWAIT) != written) {
            shutdown(client, SHUT_RDWR);
            s_subscriber = -1;
        }
    }
    pthread_mutex_unlock(&s_subscriberMutex);
}

static bool M400Pi4InterfaceAddress(const char *name, struct in_addr *address)
{
    struct ifaddrs *interfaces = NULL;
    if (getifaddrs(&interfaces) != 0) return false;
    bool found = false;
    for (const struct ifaddrs *item = interfaces; item != NULL; item = item->ifa_next) {
        if (item->ifa_addr != NULL && item->ifa_addr->sa_family == AF_INET &&
            strcmp(item->ifa_name, name) == 0 && (item->ifa_flags & IFF_UP) != 0) {
            *address = ((const struct sockaddr_in *) item->ifa_addr)->sin_addr;
            found = true;
            break;
        }
    }
    freeifaddrs(interfaces);
    return found;
}

/* Keep only printable UTF-8. Replace control characters and malformed bytes. */
static bool M400Pi4CleanMessage(const char *input, size_t inputLength,
                               char output[M400_PI4_MESSAGE_CAPACITY])
{
    size_t readAt = 0, writeAt = 0;
    while (readAt < inputLength) {
        if (writeAt + 1 >= M400_PI4_MESSAGE_CAPACITY) return false;
        const unsigned char first = (unsigned char) input[readAt];
        if (first < 0x20 || first == 0x7f) {
            output[writeAt++] = ' ';
            readAt++;
            continue;
        }
        if (first < 0x80) {
            output[writeAt++] = (char) first;
            readAt++;
            continue;
        }
        size_t width = first >= 0xc2 && first <= 0xdf ? 2 :
                       first >= 0xe0 && first <= 0xef ? 3 :
                       first >= 0xf0 && first <= 0xf4 ? 4 : 0;
        if (width == 0 || readAt + width > inputLength) {
            output[writeAt++] = '?';
            readAt++;
            continue;
        }
        if (writeAt + width >= M400_PI4_MESSAGE_CAPACITY) return false;
        bool valid = true;
        for (size_t i = 1; i < width; ++i) {
            if (((unsigned char) input[readAt + i] & 0xc0) != 0x80) valid = false;
        }
        if (width == 3 && ((first == 0xe0 && (unsigned char) input[readAt + 1] < 0xa0) ||
                           (first == 0xed && (unsigned char) input[readAt + 1] >= 0xa0))) valid = false;
        if (width == 4 && ((first == 0xf0 && (unsigned char) input[readAt + 1] < 0x90) ||
                           (first == 0xf4 && (unsigned char) input[readAt + 1] >= 0x90))) valid = false;
        if (!valid) {
            output[writeAt++] = '?';
            readAt++;
            continue;
        }
        memcpy(output + writeAt, input + readAt, width);
        writeAt += width;
        readAt += width;
    }
    while (writeAt > 0 && output[writeAt - 1] == ' ') writeAt--;
    output[writeAt] = '\0';
    return true;
}

/* Read one newline-terminated request from either kind of connection. */
static bool M400Pi4ReadLine(int client, char line[M400_PI4_LINE_MAX], size_t *length)
{
    *length = 0;
    while (*length < M400_PI4_LINE_MAX) {
        char byte;
        ssize_t count = recv(client, &byte, 1, 0);
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) return false;
        if (byte == '\n') {
            if (*length > 0 && line[*length - 1] == '\r') (*length)--;
            return true;
        }
        line[(*length)++] = byte;
    }
    return false;
}

static void M400Pi4HandleText(int client, const T_M400Pi4Bridge *bridge,
                              const char *line, size_t length, bool subscribed)
{
    const char *reply;
    if (subscribed) {
        if (length < 5 || memcmp(line, "TEXT\t", 5) != 0) {
            M400Pi4SendClientLine(client, "ACK\tERR expected TEXT\n");
            return;
        }
        line += 5;
        length -= 5;
    }
    char message[M400_PI4_MESSAGE_CAPACITY];
    if (!M400Pi4CleanMessage(line, length, message)) {
        reply = subscribed ? "ACK\tERR message exceeds 240 UTF-8 bytes\n" :
                             "ERR message exceeds 240 UTF-8 bytes\n";
    } else if (message[0] == '\0') {
        reply = subscribed ? "ACK\tERR empty message\n" : "ERR empty message\n";
    } else {
        bridge->callback(message);
        reply = subscribed ? "ACK\tOK\n" : "OK\n";
    }
    if (subscribed) M400Pi4SendClientLine(client, reply);
    else (void) send(client, reply, strlen(reply), MSG_NOSIGNAL);
}

static void *M400Pi4SubscriberTask(void *arg)
{
    int client = (int) (intptr_t) arg;
    char line[M400_PI4_LINE_MAX];
    size_t length;
    while (M400Pi4ReadLine(client, line, &length)) {
        struct timespec receivedAt;
        clock_gettime(CLOCK_REALTIME, &receivedAt);
        if (length >= 9 && memcmp(line, "TIME_REQ\t", 9) == 0) {
            M400Pi4HandleTimeRequest(client, line, length, &receivedAt);
        } else {
            M400Pi4HandleText(client, &s_bridge, line, length, true);
        }
    }
    pthread_mutex_lock(&s_subscriberMutex);
    if (s_subscriber == client) s_subscriber = -1;
    pthread_mutex_unlock(&s_subscriberMutex);
    close(client);
    return NULL;
}

/* A subscribed socket stays open and receives text in both directions. */
static bool M400Pi4HandleClient(int client, const T_M400Pi4Bridge *bridge)
{
    struct timeval timeout = {.tv_sec = 2, .tv_usec = 0};
    setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    char line[M400_PI4_LINE_MAX];
    size_t length;
    if (!M400Pi4ReadLine(client, line, &length)) {
        static const char reply[] = "ERR line too long or incomplete\n";
        (void) send(client, reply, sizeof(reply) - 1, MSG_NOSIGNAL);
        return false;
    }
    if (length == sizeof("SUBSCRIBE_BUTTONS") - 1 &&
        memcmp(line, "SUBSCRIBE_BUTTONS", length) == 0) {
        struct timeval noTimeout = {.tv_sec = 0, .tv_usec = 0};
        setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &noTimeout, sizeof(noTimeout));
        pthread_mutex_lock(&s_subscriberMutex);
        bool subscribed = send(client, "OK\n", 3, MSG_NOSIGNAL | MSG_DONTWAIT) == 3;
        if (subscribed) {
            pthread_t reader;
            if (pthread_create(&reader, NULL, M400Pi4SubscriberTask,
                               (void *) (intptr_t) client) == 0) {
                if (s_subscriber >= 0) shutdown(s_subscriber, SHUT_RDWR);
                s_subscriber = client;
                pthread_detach(reader);
            } else {
                subscribed = false;
            }
        }
        pthread_mutex_unlock(&s_subscriberMutex);
        if (subscribed) M400Pi4SendSubscriberLine("TIME_POLL\n", 10);
        return subscribed;
    }
    M400Pi4HandleText(client, bridge, line, length, false);
    return false;
}

static void *M400Pi4ListenTask(void *arg)
{
    const T_M400Pi4Bridge *bridge = (const T_M400Pi4Bridge *) arg;
    for (;;) {
        struct in_addr listenAddress;
        if (!M400Pi4InterfaceAddress(bridge->interfaceName, &listenAddress)) {
            sleep(2);
            continue;
        }
        int server = socket(AF_INET, SOCK_STREAM, 0);
        if (server < 0) {
            sleep(2);
            continue;
        }
        int reuse = 1;
        setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        struct sockaddr_in local = {.sin_family = AF_INET, .sin_port = htons(bridge->port),
                                    .sin_addr = listenAddress};
        if (bind(server, (struct sockaddr *) &local, sizeof(local)) != 0 || listen(server, 4) != 0) {
            close(server);
            sleep(2);
            continue;
        }
        struct timeval acceptTimeout = {.tv_sec = 2, .tv_usec = 0};
        setsockopt(server, SOL_SOCKET, SO_RCVTIMEO, &acceptTimeout, sizeof(acceptTimeout));
        fprintf(stderr, "Pi4 TCP listener ready on %s:%u\n", bridge->interfaceName, bridge->port);
        struct timespec pollClock;
        clock_gettime(CLOCK_MONOTONIC, &pollClock);
        uint64_t lastTimePollNs = M400Pi4TimestampNs(&pollClock);
        for (;;) {
            clock_gettime(CLOCK_MONOTONIC, &pollClock);
            uint64_t nowNs = M400Pi4TimestampNs(&pollClock);
            if (nowNs - lastTimePollNs >= M400_PI4_TIME_POLL_INTERVAL_SEC * 1000000000ULL) {
                M400Pi4SendSubscriberLine("TIME_POLL\n", 10);
                lastTimePollNs = nowNs;
            }
            struct in_addr currentAddress;
            if (!M400Pi4InterfaceAddress(bridge->interfaceName, &currentAddress) ||
                currentAddress.s_addr != listenAddress.s_addr) break;
            struct sockaddr_in peer;
            socklen_t peerLength = sizeof(peer);
            int client = accept(server, (struct sockaddr *) &peer, &peerLength);
            if (client < 0) {
                if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                    M400Pi4SendSubscriberLine("PING\n", 5);
                    continue;
                }
                break;
            }
            bool subscribed = peer.sin_family == AF_INET &&
                              peer.sin_addr.s_addr == bridge->allowedPeer.s_addr &&
                              M400Pi4HandleClient(client, bridge);
            if (!subscribed) close(client);
        }
        close(server);
        M400Pi4CloseSubscriber();
        sleep(2);
    }
    return NULL;
}

int M400Pi4Bridge_Start(const char *interfaceName, const char *allowedPeerIp,
                        uint16_t port, M400Pi4MessageCallback callback)
{
    if (s_started || interfaceName == NULL || allowedPeerIp == NULL || callback == NULL || port == 0 ||
        strlen(interfaceName) >= sizeof(s_bridge.interfaceName)) return -1;
    memset(&s_bridge, 0, sizeof(s_bridge));
    strcpy(s_bridge.interfaceName, interfaceName);
    if (inet_pton(AF_INET, allowedPeerIp, &s_bridge.allowedPeer) != 1) return -1;
    s_bridge.port = port;
    s_bridge.callback = callback;
    pthread_t thread;
    if (pthread_create(&thread, NULL, M400Pi4ListenTask, &s_bridge) != 0) return -1;
    pthread_detach(thread);
    s_started = true;
    return 0;
}
