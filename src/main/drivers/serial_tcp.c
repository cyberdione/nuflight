/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Authors:
 * Dominic Clifton - Serial port abstraction, Separation of common STM32 code for cleanflight, various cleanups.
 * Hamasaki/Timecop - Initial baseflight code
*/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/utils.h"

#include "io/serial.h"
#include "serial_tcp.h"

#define BASE_PORT 5760

static const struct serialPortVTable tcpVTable; // Forward
static tcpPort_t tcpSerialPorts[SERIAL_PORT_COUNT];
static bool tcpPortInitialized[SERIAL_PORT_COUNT];
static bool tcpSetupPending[SERIAL_PORT_COUNT];
static bool tcpStart = false;

// Dyad is a single-threaded event library: every dyad call — stream creation,
// write, update, close, free — must run on the tcp worker thread (sitl.c
// tcpThread). This file enforces that ownership as follows:
//
//   - The worker loop calls tcpWorkerPoll(), which services pending listener
//     setups, drains the serial tx rings into dyad, then runs dyad_update().
//     dyad_update() blocks in select() for up to the update timeout, so no
//     lock may be held across it (an earlier draft held one there and the
//     spinning worker starved serialInit's lock acquisition entirely).
//   - The flight thread only ever touches the txLock-guarded tx ring
//     (tcpWrite); it never calls into dyad.
//   - serialInit runs on the main thread while the worker is already
//     polling, so tcpReconfigure() cannot create dyad streams itself.
//     Instead it publishes the port struct fields, marks tcpSetupPending[],
//     and the worker creates/binds the listener stream on a later
//     iteration. Binding is therefore deferred by at most one poll (~10ms
//     at boot), which no client can observe.
//
// tcpStateLock guards only the two flag arrays (and is held for nanoseconds,
// never across a dyad call or select): port publication
// (tcpPortInitialized[], set by serTcpOpen once the port struct is fully
// initialised, read by the worker's pump snapshot and by onAccept) and the
// setup handoff (tcpSetupPending[], written by tcpReconfigure, consumed by
// the worker).
static pthread_mutex_t tcpStateLock = PTHREAD_MUTEX_INITIALIZER;

bool tcpIsStart(void)
{
    return tcpStart;
}

static void onData(dyad_Event *e)
{
    tcpPort_t* s = (tcpPort_t*)(e->udata);
    tcpDataIn(s, (uint8_t*)e->data, e->size);
}

static void onClose(dyad_Event *e)
{
    tcpPort_t* s = (tcpPort_t*)(e->udata);
    s->clientCount--;
    s->conn = NULL;
    fprintf(stderr, "[CLS]UART%u: %d,%d\n", s->id + 1U, s->connected, s->clientCount);
    if (s->clientCount == 0) {
        s->connected = false;
    }
}

static void onAccept(dyad_Event *e)
{
    tcpPort_t* s = (tcpPort_t*)(e->udata);
    fprintf(stderr, "New connection on UART%u, %d\n", s->id + 1U, s->clientCount);

    pthread_mutex_lock(&tcpStateLock);
    const bool published = tcpPortInitialized[s->id];
    pthread_mutex_unlock(&tcpStateLock);
    if (!published) {
        // serTcpOpen() is still initialising this port; touching it here would
        // race the initialisation. Reject the early client instead.
        dyad_close(e->remote);
        return;
    }

    s->connected = true;
    if (s->clientCount > 0) {
        dyad_close(e->remote);
        return;
    }
    s->clientCount++;
    fprintf(stderr, "[NEW]UART%u: %d,%d\n", s->id + 1U, s->connected, s->clientCount);
    s->conn = e->remote;
    dyad_setNoDelay(e->remote, 1);
    dyad_setTimeout(e->remote, 120);
    dyad_addListener(e->remote, DYAD_EVENT_DATA, onData, e->udata);
    dyad_addListener(e->remote, DYAD_EVENT_CLOSE, onClose, e->udata);
}

static tcpPort_t* tcpReconfigure(tcpPort_t *s, int id)
{
    if (tcpPortInitialized[id]) {
        fprintf(stderr, "port is already initialized!\n");
        return s;
    }

    if (pthread_mutex_init(&s->txLock, NULL) != 0) {
        fprintf(stderr, "TX mutex init failed - %d\n", errno);
        // TODO: clean up & re-init
        return NULL;
    }

    if (pthread_mutex_init(&s->rxLock, NULL) != 0) {
        fprintf(stderr, "RX mutex init failed - %d\n", errno);
        // TODO: clean up & re-init
        return NULL;
    }

    tcpStart = true;

    s->connected = false;
    s->clientCount = 0;
    s->id = id;
    s->conn = NULL;

    // Do NOT create the dyad listener here: the tcp worker may already be
    // inside dyad_update() traversing the stream list (and would even free a
    // half-built DYAD_STATE_CLOSED stream in destroyClosedStreams()). Mark
    // the setup pending; the worker creates and binds the listener stream on
    // its next poll, keeping every dyad call on that one thread.
    pthread_mutex_lock(&tcpStateLock);
    tcpSetupPending[id] = true;
    pthread_mutex_unlock(&tcpStateLock);
    return s;
}

// Worker-thread counterpart of the tcpReconfigure() handoff above. Runs with
// no lock held: only the worker touches dyad, and the port struct fields it
// reads were written before tcpSetupPending[] was published.
static void tcpCreateListener(int id)
{
    tcpPort_t *s = &tcpSerialPorts[id];

    s->serv = dyad_newStream();
    dyad_setNoDelay(s->serv, 1);
    dyad_addListener(s->serv, DYAD_EVENT_ACCEPT, onAccept, s);

    if (dyad_listenEx(s->serv, NULL, BASE_PORT + id + 1, 10) == 0) {
        fprintf(stderr, "bind port %u for UART%u\n", (unsigned)BASE_PORT + id + 1, (unsigned)id + 1);
    } else {
        fprintf(stderr, "bind port %u for UART%u failed!!\n", (unsigned)BASE_PORT + id + 1, (unsigned)id + 1);
    }
}

serialPort_t *serTcpOpen(serialPortIdentifier_e identifier, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    tcpPort_t *s = NULL;

    int id = findSerialPortIndexByIdentifier(identifier);

    if (id >= 0 && id < (int)ARRAYLEN(tcpSerialPorts)) {
        s = tcpReconfigure(&tcpSerialPorts[id], id);
    }

    if (!s) {
        return NULL;
    }

    s->port.vTable = &tcpVTable;

    // common serial initialisation code should move to serialPort::init()
    // (the ring resets take the port locks so a re-open cannot race the
    // worker's pump or the flight thread's writes)
    pthread_mutex_lock(&s->rxLock);
    s->port.rxBufferHead = s->port.rxBufferTail = 0;
    pthread_mutex_unlock(&s->rxLock);
    pthread_mutex_lock(&s->txLock);
    s->port.txBufferHead = s->port.txBufferTail = 0;
    pthread_mutex_unlock(&s->txLock);
    s->port.rxBufferSize = RX_BUFFER_SIZE;
    s->port.txBufferSize = TX_BUFFER_SIZE;
    s->port.rxBuffer = s->rxBuffer;
    s->port.txBuffer = s->txBuffer;

    // callback works for IRQ-based RX ONLY
    s->port.rxCallback = rxCallback;
    s->port.rxCallbackData = rxCallbackData;
    s->port.mode = mode;
    s->port.baudRate = baudRate;
    s->port.options = options;

    // Fully initialised: publish it to the tcp worker. All fields the worker
    // touches (via tcpWorkerPoll and the dyad callbacks) were written above.
    pthread_mutex_lock(&tcpStateLock);
    tcpPortInitialized[id] = true;
    pthread_mutex_unlock(&tcpStateLock);

    return (serialPort_t *)s;
}

static uint32_t tcpTotalRxBytesWaiting(const serialPort_t *instance)
{
    tcpPort_t *s = (tcpPort_t*)instance;
    uint32_t count;
    pthread_mutex_lock(&s->rxLock);
    if (s->port.rxBufferHead >= s->port.rxBufferTail) {
        count = s->port.rxBufferHead - s->port.rxBufferTail;
    } else {
        count = s->port.rxBufferSize + s->port.rxBufferHead - s->port.rxBufferTail;
    }
    pthread_mutex_unlock(&s->rxLock);

    return count;
}

static uint32_t tcpTotalTxBytesFree(const serialPort_t *instance)
{
    tcpPort_t *s = (tcpPort_t*)instance;
    uint32_t bytesUsed;

    pthread_mutex_lock(&s->txLock);
    if (s->port.txBufferHead >= s->port.txBufferTail) {
        bytesUsed = s->port.txBufferHead - s->port.txBufferTail;
    } else {
        bytesUsed = s->port.txBufferSize + s->port.txBufferHead - s->port.txBufferTail;
    }
    uint32_t bytesFree = (s->port.txBufferSize - 1) - bytesUsed;
    pthread_mutex_unlock(&s->txLock);

    return bytesFree;
}

static bool isTcpTransmitBufferEmpty(const serialPort_t *instance)
{
    tcpPort_t *s = (tcpPort_t *)instance;
    pthread_mutex_lock(&s->txLock);
    bool isEmpty = s->port.txBufferTail == s->port.txBufferHead;
    pthread_mutex_unlock(&s->txLock);
    return isEmpty;
}

static uint8_t tcpRead(serialPort_t *instance)
{
    uint8_t ch;
    tcpPort_t *s = (tcpPort_t *)instance;
    pthread_mutex_lock(&s->rxLock);

    ch = s->port.rxBuffer[s->port.rxBufferTail];
    if (s->port.rxBufferTail + 1 >= s->port.rxBufferSize) {
        s->port.rxBufferTail = 0;
    } else {
        s->port.rxBufferTail++;
    }
    pthread_mutex_unlock(&s->rxLock);

    return ch;
}

static void tcpWrite(serialPort_t *instance, uint8_t ch)
{
    tcpPort_t *s = (tcpPort_t *)instance;
    pthread_mutex_lock(&s->txLock);

    s->port.txBuffer[s->port.txBufferHead] = ch;
    if (s->port.txBufferHead + 1 >= s->port.txBufferSize) {
        s->port.txBufferHead = 0;
    } else {
        s->port.txBufferHead++;
    }
    pthread_mutex_unlock(&s->txLock);

    // Do NOT drain here. Draining used to call dyad_write() from the flight
    // thread while the tcp worker flushed and freed the same dyad buffers in
    // dyad_update() — the SITL heap-corruption race (malloc_printerr ->
    // calloc -> tcpThread). tcpWorkerPoll() drains the ring on the worker.
}

// Drain the tx ring of one port into the dyad write buffer. Runs on the tcp
// worker thread only (called from tcpWorkerPoll, with no lock held).
void tcpDataOut(tcpPort_t *instance)
{
    tcpPort_t *s = (tcpPort_t *)instance;
    if (s->conn == NULL) return;
    pthread_mutex_lock(&s->txLock);

    if (s->port.txBufferHead < s->port.txBufferTail) {
        // send data till end of buffer
        int chunk = s->port.txBufferSize - s->port.txBufferTail;
        dyad_write(s->conn, (const void *)&s->port.txBuffer[s->port.txBufferTail], chunk);
        s->port.txBufferTail = 0;
    }
    int chunk = s->port.txBufferHead - s->port.txBufferTail;
    if (chunk)
        dyad_write(s->conn, (const void*)&s->port.txBuffer[s->port.txBufferTail], chunk);
    s->port.txBufferTail = s->port.txBufferHead;

    pthread_mutex_unlock(&s->txLock);
}

void tcpDataIn(tcpPort_t *instance, uint8_t* ch, int size)
{
    tcpPort_t *s = (tcpPort_t *)instance;
    pthread_mutex_lock(&s->rxLock);

    while (size--) {
//        printf("%c", *ch);
        s->port.rxBuffer[s->port.rxBufferHead] = *(ch++);
        if (s->port.rxBufferHead + 1 >= s->port.rxBufferSize) {
            s->port.rxBufferHead = 0;
        } else {
            s->port.rxBufferHead++;
        }
    }
    pthread_mutex_unlock(&s->rxLock);
//    printf("\n");
}

// One iteration of the tcp worker: service pending listener setups, drain
// every published port's tx ring into its dyad stream, then run the dyad
// event loop (accept, recv, flush, close, timeouts, stream destruction).
// Called only from the sitl.c tcpThread; this is the single thread that owns
// dyad, so no lock is needed (or permitted) around the dyad calls —
// dyad_update() blocks in select() for up to the update timeout, and a lock
// held across it would starve tcpReconfigure()'s flag handoff. The state
// lock is taken only to snapshot/clear the two flag arrays. Pumping before
// dyad_update() lets the update flush the freshly written bytes in the same
// iteration.
void tcpWorkerPoll(void)
{
    bool pending[SERIAL_PORT_COUNT];
    bool initialized[SERIAL_PORT_COUNT];

    pthread_mutex_lock(&tcpStateLock);
    memcpy(pending, tcpSetupPending, sizeof(pending));
    memcpy(initialized, tcpPortInitialized, sizeof(initialized));
    memset(tcpSetupPending, 0, sizeof(tcpSetupPending));
    pthread_mutex_unlock(&tcpStateLock);

    for (unsigned id = 0; id < ARRAYLEN(tcpSerialPorts); id++) {
        if (pending[id]) {
            tcpCreateListener(id);
        }
        if (initialized[id]) {
            tcpDataOut(&tcpSerialPorts[id]);
        }
    }
    dyad_update();
}

static const struct serialPortVTable tcpVTable = {
        .serialWrite = tcpWrite,
        .serialTotalRxWaiting = tcpTotalRxBytesWaiting,
        .serialTotalTxFree = tcpTotalTxBytesFree,
        .serialRead = tcpRead,
        .serialSetBaudRate = NULL,
        .isSerialTransmitBufferEmpty = isTcpTransmitBufferEmpty,
        .setMode = NULL,
        .setCtrlLineStateCb = NULL,
        .setBaudRateCb = NULL,
        .writeBuf = NULL,
        .beginWrite = NULL,
        .endWrite = NULL,
};
