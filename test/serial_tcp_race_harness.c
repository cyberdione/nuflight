/*
 * This file is part of nuflight (Betaflight SITL tooling).
 *
 * Thread-safety regression harness for the SITL TCP serial bridge
 * (drivers/serial_tcp.c + lib/main/dyad).
 *
 * Dyad is a single-threaded event library. The firmware bug this harness
 * guards against: tcpWrite() used to drain the tx ring on the *flight* thread
 * (calling dyad_write() off-thread) while the tcp worker concurrently ran
 * dyad_update(), which flushes, clears, reallocates and frees the very same
 * dyad buffers and streams. In the field that race corrupted the heap and
 * aborted the SITL process with: malloc_printerr -> calloc -> tcpThread.
 *
 * The harness links the real serial_tcp.c and dyad.c and drives them from two
 * threads exactly the way sitl.c does:
 *
 *   worker thread : dyad_init() then a poll loop (dyad_update() or
 *                   tcpWorkerPoll()) and dyad_shutdown() on exit.
 *   flight thread : tcpWrite() bytes in MSP-sized frames, throttled by
 *                   tcpTotalTxBytesFree() the way MSP checks serialTxBytesFree().
 *
 * A loopback TCP client connects to the listener port (5761, UART1), reads
 * the stream and checks it byte-for-byte. Two connection generations run
 * back to back so the close/destroy/re-accept stream lifecycle is exercised
 * too — the window where the old code dereferenced a dyad stream pointer
 * that the worker had just freed.
 *
 * Modes (see run-serial-tcp-race.sh):
 *
 *   default              Built against the fixed sources in the tree. Must
 *                        pass: every byte arrives exactly once, in order,
 *                        and the process runs clean under ThreadSanitizer
 *                        and AddressSanitizer.
 *
 *   -DOLD_UNSAFE_PATTERN The pre-fix worker shape (bare dyad_update(), as
 *                        sitl.c's tcpThread used to call it). ONLY meaningful
 *                        when also compiled against the PRE-FIX sources —
 *                        the runner extracts those from git history, so the
 *                        original tcpWrite() drains the ring (dyad_write!)
 *                        on the flight thread itself and every pre-fix race
 *                        is exercised. Expected to FAIL: ThreadSanitizer
 *                        reports the data races (deterministically), and
 *                        without sanitizers the transfer breaks or the
 *                        process aborts. Building this mode against the
 *                        fixed sources would deadlock serialInit's setup
 *                        handoff (the fixed worker loop is what services
 *                        it) — don't.
 *
 * Build (from the repository root, same include set as `make TARGET=SITL`):
 *
 *   gcc -Isrc/main -Isrc/platform/SIMULATOR -Isrc/platform/SIMULATOR/include \
 *       -Ilib/main/dyad -Isrc/platform/SIMULATOR/target/SITL -std=gnu17 \
 *       test/serial_tcp_race_harness.c src/main/drivers/serial_tcp.c \
 *       lib/main/dyad/dyad.c -lpthread -o obj/test/serial_tcp_race
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <poll.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "drivers/serial_tcp.h"
#include "dyad.h"

// BASE_PORT + id + 1 for id 0 (the stub identifier resolver below returns 0).
#define TEST_PORT 5761

#ifndef FRAMES
#define FRAMES 2000
#endif
#define FRAME_LEN 48
// Instrumentation: every SAMPLE_EVERY-th frame gets a write-side and a
// read-side CLOCK_MONOTONIC timestamp so the harness can report the
// write->arrival latency distribution through the tx ring + dyad, plus the
// peak tx-ring occupancy (via serialTotalTxFree, the same call MSP uses for
// flow control) and sustained throughput.
#ifndef SAMPLE_EVERY
#define SAMPLE_EVERY 100
#endif
#define SAMPLE_COUNT ((FRAMES - 1) / SAMPLE_EVERY + 1)
#define MAX_SAMPLES (20000 / SAMPLE_EVERY + 2)
#if SAMPLE_COUNT > MAX_SAMPLES
#error "adjust MAX_SAMPLES for this SAMPLE_EVERY"
#endif

static struct timespec g_writeT[MAX_SAMPLES];
static struct timespec g_readT[MAX_SAMPLES];
static int g_minTxFree = FRAME_LEN + 1; // smallest ring headroom observed

static double tsDeltaMs(const struct timespec *a, const struct timespec *b)
{
    return (double)(b->tv_sec - a->tv_sec) * 1e3 + (double)(b->tv_nsec - a->tv_nsec) / 1e6;
}

// serial_tcp.c expects this from io/serial.c; the harness maps any
// identifier to tcpSerialPorts[0].
int findSerialPortIndexByIdentifier(serialPortIdentifier_e identifier)
{
    (void)identifier;
    return 0;
}

static atomic_int g_workerRun = 1;
static int g_phase = 0;

static void die(const char *what)
{
    fprintf(stderr, "FAIL: %s (errno=%d %s)\n", what, errno, strerror(errno));
    exit(1);
}

static uint8_t frameByte(int phase, int seq, int i)
{
    return (uint8_t)(0x5Au + 0x2Cu * (unsigned)phase + 7u * (unsigned)seq + 13u * (unsigned)i);
}

// The tcp worker thread, mirroring sitl.c tcpThread.
static void *workerThread(void *arg)
{
    (void)arg;

    dyad_init();
    dyad_setTickInterval(0.2);
    dyad_setUpdateTimeout(0.01);

    while (atomic_load(&g_workerRun)) {
#ifdef OLD_UNSAFE_PATTERN
        dyad_update();            // pre-fix sitl.c tcpThread shape
#else
        tcpWorkerPoll();          // fixed: setup handoff + pump + update
#endif
    }

    dyad_shutdown();
    return NULL;
}

// The flight thread, mirroring scheduler tasks doing MSP-sized writes. The
// writes go through the serial vTable — the same path io/serial.c uses —
// because tcpWrite()/tcpTotalTxBytesFree() are static in serial_tcp.c.
static void *flightThread(void *arg)
{
    serialPort_t *port = arg;

    for (int seq = 0; seq < FRAMES; seq++) {
        // MSP-style flow control: never write more than the tx ring can hold
        // (msp_serial.c checks serialTxBytesFree the same way, and
        // serialWriteBuf blocks while the ring is full).
        int freeBytes;
        while ((freeBytes = (int)port->vTable->serialTotalTxFree(port)) < FRAME_LEN) {
            struct timespec ts = { .tv_sec = 0, .tv_nsec = 100000 };
            nanosleep(&ts, NULL);
        }
        if (freeBytes < g_minTxFree) {
            g_minTxFree = freeBytes;
        }
        if (seq % SAMPLE_EVERY == 0) {
            clock_gettime(CLOCK_MONOTONIC, &g_writeT[seq / SAMPLE_EVERY]);
        }
        for (int i = 0; i < FRAME_LEN; i++) {
            // Against pre-fix sources this IS the old racy shape: the
            // original tcpWrite() drained the ring (dyad_write) itself.
            port->vTable->serialWrite(port, frameByte(g_phase, seq, i));
        }
    }
    return NULL;
}

// The fixed code binds the listener asynchronously (the worker thread
// materialises it on its next poll after serTcpOpen publishes the setup
// request), so a client must retry briefly — exactly like the fpvhero bridge,
// which retries its MSP connection for seconds at startup.
static int connectClient(void)
{
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(TEST_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    for (int attempt = 0; attempt < 500; attempt++) { // ~10s at 20ms
        int fd = socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) {
            die("socket");
        }
        if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) == 0) {
            return fd;
        }
        close(fd);
        struct timespec ts = { .tv_sec = 0, .tv_nsec = 20000000 };
        nanosleep(&ts, NULL);
    }
    die("connect to SITL UART1 listener (is another SITL running on port 5761?)");
    return -1; // unreachable
}

// Read the generation's byte stream, verifying each chunk against the
// expected pattern as it arrives and timestamping the arrival of every
// sampled frame. The write-side timestamps (g_writeT) are read later, after
// the flight thread has been joined, so no unsynchronised access happens.
static double g_elapsedMs[2];

static void verifyGeneration(int fd, int phase)
{
    static uint8_t buf[8192];
    const size_t total = (size_t)FRAMES * FRAME_LEN;
    size_t got = 0;
    struct timespec t0, tLast;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    while (got < total) {
        struct pollfd pfd = { .fd = fd, .events = POLLIN };
        int r = poll(&pfd, 1, 20000);
        if (r <= 0) {
            die("timed out waiting for the byte stream from the SITL TCP port");
        }
        ssize_t k = read(fd, buf, sizeof(buf));
        if (k <= 0) {
            die("reading the byte stream failed (connection broken mid-transfer)");
        }
        for (ssize_t i = 0; i < k; i++) {
            const size_t idx = got + (size_t)i;
            const uint8_t want = frameByte(phase, (int)(idx / FRAME_LEN), (int)(idx % FRAME_LEN));
            if (buf[i] != want) {
                fprintf(stderr, "FAIL: byte %zu of generation %d: got 0x%02x, want 0x%02x\n",
                        idx, phase, buf[i], want);
                exit(1);
            }
        }
        clock_gettime(CLOCK_MONOTONIC, &tLast);
        // Sample s covers frame s*SAMPLE_EVERY; that frame has fully arrived
        // once byte (s*SAMPLE_EVERY + 1)*FRAME_LEN (1-based) is within this
        // chunk. The timestamp is taken after the chunk, so the reported
        // latency carries a small upper-bound error.
        for (int s = 0; s < SAMPLE_COUNT; s++) {
            const size_t boundary = ((size_t)s * SAMPLE_EVERY + 1) * FRAME_LEN;
            if (boundary > got + (size_t)k) {
                break;
            }
            if (boundary > got) {
                g_readT[s] = tLast;
            }
        }
        got += (size_t)k;
    }
    g_elapsedMs[phase] = tsDeltaMs(&t0, &tLast);
    printf("generation %d: %zu bytes verified byte-for-byte\n", phase, total);
}

static int cmpDouble(const void *a, const void *b)
{
    const double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

// Called after pthread_join(flight): g_writeT/g_minTxFree are stable now.
static void printStats(int phase)
{
    double lat[SAMPLE_COUNT];
    int n = 0;
    for (int s = 0; s < SAMPLE_COUNT; s++) {
        if (g_readT[s].tv_sec != 0 || g_readT[s].tv_nsec != 0) {
            lat[n++] = tsDeltaMs(&g_writeT[s], &g_readT[s]);
        }
    }
    if (n == 0) {
        return;
    }
    qsort(lat, (size_t)n, sizeof(lat[0]), cmpDouble);
    printf("generation %d stats: throughput %.2f MB/s, write->arrival latency ms (min/median/max): %.3f / %.3f / %.3f over %d samples, min tx-ring free headroom %d bytes\n",
           phase,
           (double)FRAMES * FRAME_LEN / 1024.0 / 1024.0 / (g_elapsedMs[phase] / 1e3),
           lat[0], lat[n / 2], lat[n - 1], n, g_minTxFree);
}

int main(void)
{
    // Firmware order: the tcp worker starts (systemInit) before the serial
    // ports are opened (serialInit -> serTcpOpen), so init-time listener
    // setup runs while the worker is already polling.
    pthread_t worker;
    pthread_create(&worker, NULL, workerThread, NULL);

    serialPort_t *port = serTcpOpen(SERIAL_PORT_USART1, NULL, NULL, 115200, MODE_RXTX, SERIAL_NOT_INVERTED);
    if (!port) {
        die("serTcpOpen");
    }

    for (int generation = 0; generation < 2; generation++) {
        g_phase = generation;
        memset(g_readT, 0, sizeof(g_readT));
        g_minTxFree = FRAME_LEN + 1;
        int cfd = connectClient();

        pthread_t flight;
        pthread_create(&flight, NULL, flightThread, port);

        verifyGeneration(cfd, generation);
        pthread_join(flight, NULL);
        printStats(generation);
        close(cfd);

        if (generation == 0) {
            // Let the worker observe EOF: onClose() -> conn = NULL, then
            // destroyClosedStreams() frees the old dyad stream. The next
            // generation exercises accept() of a replacement stream while
            // the flight thread keeps writing.
            usleep(200000);
        }
    }

    atomic_store(&g_workerRun, 0);
    pthread_join(worker, NULL);

    printf("PASS: %d bytes over %d connection generations, byte-perfect, clean shutdown\n",
           2 * FRAMES * FRAME_LEN, 2);
    return 0;
}
