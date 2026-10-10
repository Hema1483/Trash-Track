/*
 * waste_scheduler.c
 *
 * QNX eHACK 2026 - PS37: Intelligent Waste Collection Scheduler
 * Team: Deadline Drivers
 *
 * Simulates 4 street-side smart bins. Each bin has a simulated
 * ultrasonic fill-level reading and a simulated presence sensor that
 * triggers a door state machine. A Route Scheduler periodically
 * recomputes the truck's visiting order using a weighted-score
 * greedy algorithm. An Emergency Task (highest priority) watches
 * every reading and immediately overrides the route if a bin
 * overflows. A Vehicle Task consumes the route and services bins.
 *
 * IPC style follows the course material: native QNX channels and
 * pulses (ChannelCreate / MsgSendPulse / MsgReceive), POSIX timers
 * that deliver a pulse (timer_create + SIGEV_PULSE_INIT), and
 * pthread mutexes to protect the shared bin-status map.
 *
 * Build (QNX Momentics / command line):
 *   qcc -Vgcc_ntox86_64 -lm -o waste_scheduler waste_scheduler.c
 *
 * Run:
 *   ./waste_scheduler
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <pthread.h>
#include <sched.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>

/* ---------------------------------------------------------- *
 *  Configuration
 * ---------------------------------------------------------- */

#define NUM_STREETS          4
#define OVERFLOW_THRESHOLD   90.0f
#define ROUTE_PERIOD_SEC     5          /* Route Scheduler wake period      */
#define BIN_UPDATE_MS        1000       /* Bin Monitor sampling period      */
#define WATCHDOG_TIMEOUT_MS  300        /* Max allowed override response    */
#define LOG_FILE             "waste_scheduler.log"

/* Pulse codes (must be >= _PULSE_CODE_MINAVAIL) */
#define ROUTE_TIMER_PULSE    (_PULSE_CODE_MINAVAIL + 1)
#define BIN_READING_PULSE    (_PULSE_CODE_MINAVAIL + 2)
#define OVERRIDE_PULSE       (_PULSE_CODE_MINAVAIL + 3)
#define WATCHDOG_PULSE       (_PULSE_CODE_MINAVAIL + 4)

/* Thread priorities (SCHED_FIFO) - higher number = higher priority */
#define PRIO_EMERGENCY       25
#define PRIO_ROUTE           20
#define PRIO_BINMON          15
#define PRIO_VEHICLE         10

/* Door state machine states */
typedef enum { DOOR_IDLE, DOOR_OPEN, DOOR_LOCKED } door_state_t;

/* ---------------------------------------------------------- *
 *  Shared data (protected by bin_map_mutex)
 * ---------------------------------------------------------- */

typedef struct {
    int          street_id;
    float        fill_level;     /* 0.0 - 100.0                      */
    float        distance_km;    /* fixed, for route scoring         */
    float        wait_hours;     /* time since last collection       */
    door_state_t door;
    int          overflowed;     /* 1 once threshold crossed         */
} bin_t;

static bin_t            bin_map[NUM_STREETS];
static pthread_mutex_t  bin_map_mutex = PTHREAD_MUTEX_INITIALIZER;

static int               route[NUM_STREETS];      /* ordered street_ids */
static pthread_mutex_t   route_mutex = PTHREAD_MUTEX_INITIALIZER;

/* signaled whenever any bin's door goes from LOCKED back to IDLE
 * (i.e. the Vehicle Task has just emptied it) - lets a waiting
 * dumper wake up instead of polling */
static pthread_cond_t    space_cond = PTHREAD_COND_INITIALIZER;

static int                emergency_coid = -1;     /* connection to Emergency Task's channel */
static int                route_coid     = -1;     /* connection to Route Scheduler's channel */

static FILE *logfp;
static pthread_mutex_t log_mutex = PTHREAD_MUTEX_INITIALIZER;

/* ---------------------------------------------------------- *
 *  Helpers
 * ---------------------------------------------------------- */

static double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000.0) + (ts.tv_nsec / 1000000.0);
}

static void log_event(const char *fmt, ...)
{
    va_list ap;
    char buf[256];

    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    pthread_mutex_lock(&log_mutex);
    printf("[%.0f ms] %s\n", now_ms(), buf);
    fflush(stdout);
    if (logfp) {
        fprintf(logfp, "[%.0f ms] %s\n", now_ms(), buf);
        fflush(logfp);
    }
    pthread_mutex_unlock(&log_mutex);
}

/* set a thread's scheduling policy/priority (SCHED_FIFO) */
static void set_thread_priority(pthread_attr_t *attr, int priority)
{
    struct sched_param param;

    pthread_attr_init(attr);
    pthread_attr_setinheritsched(attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(attr, SCHED_FIFO);
    param.sched_priority = priority;
    pthread_attr_setschedparam(attr, &param);
}

/* create a repeating timer that delivers a pulse to the given connection */
static timer_t create_pulse_timer(int coid, int pulse_code,
                                   int first_sec, int interval_sec)
{
    struct sigevent  event;
    struct itimerspec it;
    timer_t          timerid;

    SIGEV_PULSE_INIT(&event, coid, SIGEV_PULSE_PRIO_INHERIT, pulse_code, 0);

    if (timer_create(CLOCK_MONOTONIC, &event, &timerid) == -1) {
        perror("timer_create");
        exit(EXIT_FAILURE);
    }

    it.it_value.tv_sec     = first_sec;
    it.it_value.tv_nsec    = 0;
    it.it_interval.tv_sec  = interval_sec;
    it.it_interval.tv_nsec = 0;

    if (timer_settime(timerid, 0, &it, NULL) == -1) {
        perror("timer_settime");
        exit(EXIT_FAILURE);
    }
    return timerid;
}

/* ---------------------------------------------------------- *
 *  Weighted-score greedy route ordering (Algorithm #2)
 * ---------------------------------------------------------- */

static int compare_bins(const void *a, const void *b)
{
    const bin_t *ba = (const bin_t *)a;
    const bin_t *bb = (const bin_t *)b;
    /* overflowed bins always sort first, regardless of score */
    if (ba->overflowed != bb->overflowed)
        return bb->overflowed - ba->overflowed;

    float score_a = 0.6f * ba->fill_level + 0.3f * (1.0f / ba->distance_km) + 0.1f * ba->wait_hours;
    float score_b = 0.6f * bb->fill_level + 0.3f * (1.0f / bb->distance_km) + 0.1f * bb->wait_hours;
    return (score_b > score_a) - (score_b < score_a); /* descending */
}

static void recompute_route(void)
{
    bin_t snapshot[NUM_STREETS];

    pthread_mutex_lock(&bin_map_mutex);
    memcpy(snapshot, bin_map, sizeof(snapshot));
    pthread_mutex_unlock(&bin_map_mutex);

    qsort(snapshot, NUM_STREETS, sizeof(bin_t), compare_bins);

    pthread_mutex_lock(&route_mutex);
    for (int i = 0; i < NUM_STREETS; i++)
        route[i] = snapshot[i].street_id;
    pthread_mutex_unlock(&route_mutex);

    printf("\n--- ROUTE UPDATED ---\n");
    for (int i = 0; i < NUM_STREETS; i++) {
        printf("  %d. Street %d  (fill=%.1f%%%s)\n", i + 1, snapshot[i].street_id,
               snapshot[i].fill_level, snapshot[i].overflowed ? ", OVERFLOW" : "");
    }
    printf("---------------------\n\n");

    log_event("Route recomputed, head = Street %d", route[0]);
}

/* ---------------------------------------------------------- *
 *  Route Scheduler task  (High priority)
 * ---------------------------------------------------------- */

static void *route_scheduler_thread(void *arg)
{
    int   chid, coid;
    rcvid_t rcvid;
    struct _pulse pulse;

    chid = ChannelCreate(0);
    if (chid == -1) { perror("ChannelCreate (route)"); exit(EXIT_FAILURE); }

    coid = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);
    if (coid == -1) { perror("ConnectAttach (route)"); exit(EXIT_FAILURE); }

    route_coid = coid;   /* let Emergency Task send override pulses here */

    create_pulse_timer(coid, ROUTE_TIMER_PULSE, ROUTE_PERIOD_SEC, ROUTE_PERIOD_SEC);

    log_event("Route Scheduler started (priority %d, period %ds)", PRIO_ROUTE, ROUTE_PERIOD_SEC);

    while (1) {
        rcvid = MsgReceive(chid, &pulse, sizeof(pulse), NULL);
        if (rcvid == -1) continue;
        if (rcvid != 0) continue;    /* we only expect pulses on this channel */

        switch (pulse.code) {
        case ROUTE_TIMER_PULSE:
            recompute_route();
            break;
        case OVERRIDE_PULSE:
            log_event("OVERRIDE received for Street %d - recomputing immediately",
                      pulse.value.sival_int);
            recompute_route();
            break;
        default:
            break;
        }
    }
    return NULL;
}

/* ---------------------------------------------------------- *
 *  Emergency Task  (Highest priority)
 * ---------------------------------------------------------- */

static void *emergency_task_thread(void *arg)
{
    int   chid, coid;
    rcvid_t rcvid;
    struct _pulse pulse;
    timer_t watchdog = 0;
    int watchdog_armed = 0;
    struct sigevent wd_event;
    struct itimerspec wd_it;

    chid = ChannelCreate(0);
    if (chid == -1) { perror("ChannelCreate (emergency)"); exit(EXIT_FAILURE); }

    coid = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);
    if (coid == -1) { perror("ConnectAttach (emergency)"); exit(EXIT_FAILURE); }

    emergency_coid = coid;   /* Bin Monitor threads send readings here */

    /* watchdog pulse goes back to this same channel */
    SIGEV_PULSE_INIT(&wd_event, coid, SIGEV_PULSE_PRIO_INHERIT, WATCHDOG_PULSE, 0);
    timer_create(CLOCK_MONOTONIC, &wd_event, &watchdog);

    log_event("Emergency Task started (priority %d, threshold %.0f%%)",
              PRIO_EMERGENCY, OVERFLOW_THRESHOLD);

    while (1) {
        rcvid = MsgReceive(chid, &pulse, sizeof(pulse), NULL);
        if (rcvid == -1) continue;
        if (rcvid != 0) continue;

        if (pulse.code == BIN_READING_PULSE) {
            int street_id = pulse.value.sival_int;
            float fill;
            int already_flagged;

            pthread_mutex_lock(&bin_map_mutex);
            fill = bin_map[street_id].fill_level;
            already_flagged = bin_map[street_id].overflowed;
            pthread_mutex_unlock(&bin_map_mutex);

            if (fill >= OVERFLOW_THRESHOLD && !already_flagged) {
                double t0 = now_ms();

                /* Algorithm #4: arm the watchdog before acting */
                wd_it.it_value.tv_sec  = 0;
                wd_it.it_value.tv_nsec = WATCHDOG_TIMEOUT_MS * 1000000L;
                wd_it.it_interval.tv_sec  = 0;
                wd_it.it_interval.tv_nsec = 0;
                timer_settime(watchdog, 0, &wd_it, NULL);
                watchdog_armed = 1;

                pthread_mutex_lock(&bin_map_mutex);
                bin_map[street_id].overflowed = 1;
                bin_map[street_id].door = DOOR_LOCKED;
                pthread_mutex_unlock(&bin_map_mutex);

                log_event("OVERFLOW: Street %d at %.1f%% - door LOCKED", street_id, fill);

                /* send the override to the Route Scheduler */
                MsgSendPulse(route_coid, SIGEV_PULSE_PRIO_INHERIT, OVERRIDE_PULSE, street_id);

                /* response completed - cancel the watchdog (Algorithm #4, "ack" path) */
                wd_it.it_value.tv_sec = 0; wd_it.it_value.tv_nsec = 0;
                timer_settime(watchdog, 0, &wd_it, NULL);
                watchdog_armed = 0;

                log_event("Override response time: %.1f ms (target <= %dms)",
                          now_ms() - t0, WATCHDOG_TIMEOUT_MS);
            }
        } else if (pulse.code == WATCHDOG_PULSE) {
            if (watchdog_armed) {
                log_event("WATCHDOG ESCALATION: override was not completed in time!");
                watchdog_armed = 0;
            }
        }
    }
    return NULL;
}

/* ---------------------------------------------------------- *
 *  Bin Monitor thread (Medium priority) - one per street
 * ---------------------------------------------------------- */

static void *bin_monitor_thread(void *arg)
{
    int street_id = *(int *)arg;
    unsigned seed = (unsigned)time(NULL) ^ (street_id * 7919);

    log_event("Bin Monitor %d started (priority %d)", street_id, PRIO_BINMON);

    while (1) {
        usleep(BIN_UPDATE_MS * 1000);

        int presence = (rand_r(&seed) % 100) < 30;   /* ~30% chance someone shows up to dump */

        pthread_mutex_lock(&bin_map_mutex);
        bin_t *b = &bin_map[street_id];

        if (presence) {
            if (b->door == DOOR_LOCKED) {
                /* Door finite state machine (Algorithm #5): bin is full.
                 * Instead of rejecting instantly, wait up to 2 seconds for
                 * the Vehicle Task to empty it and broadcast space_cond. */
                struct timespec timeout;
                clock_gettime(CLOCK_REALTIME, &timeout);
                timeout.tv_sec += 2;

                printf("  Street %d: bin full, waiting for space...\n", street_id);

                while (b->door == DOOR_LOCKED) {
                    int rc = pthread_cond_timedwait(&space_cond, &bin_map_mutex, &timeout);
                    if (rc == ETIMEDOUT) {
                        printf("  Street %d: gave up waiting, dump rejected\n", street_id);
                        break;
                    }
                    /* spurious wakeup or a DIFFERENT street's space opened up -
                     * the while condition re-checks THIS bin's door before
                     * proceeding, which is why this must be a loop, not an if */
                }
            }

            if (b->door != DOOR_LOCKED) {
                float inc = 5.0f + (rand_r(&seed) % 10);  /* +5 to +14 % per dump */
                b->door = DOOR_OPEN;
                b->fill_level += inc;
                if (b->fill_level > 100.0f) b->fill_level = 100.0f;
                printf("  Street %d: dump accepted, fill now %.1f%%\n", street_id, b->fill_level);
                b->door = DOOR_IDLE; /* door re-locks itself after each dump */
            }
        }
        float reading = b->fill_level;
        pthread_mutex_unlock(&bin_map_mutex);

        /* notify the Emergency Task of every reading (Algorithm #3 happens there) */
        MsgSendPulse(emergency_coid, SIGEV_PULSE_PRIO_INHERIT, BIN_READING_PULSE, street_id);
        (void)reading;
    }
    return NULL;
}

/* ---------------------------------------------------------- *
 *  Vehicle Task (Low priority)
 * ---------------------------------------------------------- */

static void *vehicle_task_thread(void *arg)
{
    log_event("Vehicle Task started (priority %d)", PRIO_VEHICLE);

    while (1) {
        sleep(3);

        int target;
        pthread_mutex_lock(&route_mutex);
        target = route[0];
        pthread_mutex_unlock(&route_mutex);

        pthread_mutex_lock(&bin_map_mutex);
        if (bin_map[target].fill_level > 0.0f) {
            printf("  >> Vehicle servicing Street %d (was %.1f%%)\n",
                   target, bin_map[target].fill_level);
            bin_map[target].fill_level = 0.0f;
            bin_map[target].overflowed = 0;
            bin_map[target].door = DOOR_IDLE;
            bin_map[target].wait_hours = 0.0f;
            log_event("Street %d collected and reset", target);

            /* wake up any Bin Monitor thread waiting on space_cond -
             * each one rechecks its OWN street's door state (see the
             * while loop in bin_monitor_thread), so a broadcast here
             * is safe even though only one bin actually opened up */
            pthread_cond_broadcast(&space_cond);
        }
        pthread_mutex_unlock(&bin_map_mutex);
    }
    return NULL;
}

/* ---------------------------------------------------------- *
 *  main - set up shared state, spawn all tasks
 * ---------------------------------------------------------- */

int main(void)
{
    pthread_t      tid_emergency, tid_route, tid_vehicle, tid_binmon[NUM_STREETS];
    pthread_attr_t attr_emergency, attr_route, attr_binmon, attr_vehicle;
    static int     street_ids[NUM_STREETS];

    logfp = fopen(LOG_FILE, "a");

    /* initialise the shared bin map with distinct distances/wait times */
    for (int i = 0; i < NUM_STREETS; i++) {
        bin_map[i].street_id   = i;
        bin_map[i].fill_level  = 10.0f + i * 5.0f;
        bin_map[i].distance_km = 1.0f + i;
        bin_map[i].wait_hours  = 1.0f;
        bin_map[i].door        = DOOR_IDLE;
        bin_map[i].overflowed  = 0;
        route[i] = i;
    }

    printf("=====================================================\n");
    printf(" QNX eHACK 2026 - PS37 Intelligent Waste Collection\n");
    printf(" Team: Deadline Drivers\n");
    printf("=====================================================\n\n");

    /* Emergency Task and Route Scheduler must exist before Bin Monitors
     * start sending pulses to them, so start them first and give them
     * a moment to create their channels. */
    set_thread_priority(&attr_emergency, PRIO_EMERGENCY);
    pthread_create(&tid_emergency, &attr_emergency, emergency_task_thread, NULL);

    set_thread_priority(&attr_route, PRIO_ROUTE);
    pthread_create(&tid_route, &attr_route, route_scheduler_thread, NULL);

    usleep(200000); /* let both channels/coids come up */

    set_thread_priority(&attr_binmon, PRIO_BINMON);
    for (int i = 0; i < NUM_STREETS; i++) {
        street_ids[i] = i;
        pthread_create(&tid_binmon[i], &attr_binmon, bin_monitor_thread, &street_ids[i]);
    }

    set_thread_priority(&attr_vehicle, PRIO_VEHICLE);
    pthread_create(&tid_vehicle, &attr_vehicle, vehicle_task_thread, NULL);

    /* main thread just keeps the process alive */
    pthread_join(tid_emergency, NULL);

    if (logfp) fclose(logfp);
    return EXIT_SUCCESS;
}
