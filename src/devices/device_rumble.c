#define _POSIX_C_SOURCE 200809L
#include "device_rumble.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <poll.h>

/* Software PWM period: 50 Hz / 20 ms. */
#define RUMBLE_PWM_PERIOD_US 20000u

/* Dead-zone threshold: below ~5% effective magnitude the motor is off. */
#define RUMBLE_MAG_THRESHOLD 0x0CCCu

/* Above ~75% effective magnitude, run motor at full power (no PWM).
 * On a binary motor, PWM above this level is indistinguishable from
 * full-on and just introduces unnecessary buzzing. */
#define RUMBLE_FULL_ON_THRESHOLD 0xC000u

#define RUMBLE_DEBUG_LOG "/tmp/rumble_debug.log"

static FILE *rumble_log_file = NULL;

static void rumble_log_open(void)
{
    if (!rumble_log_file) {
        /* Use O_NOFOLLOW to prevent symlink attacks in /tmp. */
        int fd = open(RUMBLE_DEBUG_LOG,
                      O_WRONLY | O_CREAT | O_APPEND | O_NOFOLLOW,
                      0600);
        if (fd >= 0) {
            rumble_log_file = fdopen(fd, "a");
            if (rumble_log_file) {
                setvbuf(rumble_log_file, NULL, _IOLBF, 0); /* line-buffered */
            } else {
                close(fd);
            }
        }
    }
}

static void rumble_log(const char *fmt, ...)
{
    if (!rumble_log_file) return;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    fprintf(rumble_log_file, "[%ld.%03ld] ",
            (long)ts.tv_sec, ts.tv_nsec / 1000000L);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(rumble_log_file, fmt, ap);
    va_end(ap);
}

static void rumble_log_close(void)
{
    if (rumble_log_file) {
        fclose(rumble_log_file);
        rumble_log_file = NULL;
    }
}

static inline void set_motor(struct device_rumble_state *st, bool on)
{
    if (!st->initialized || !st->driver || !st->driver_ctx) {
        return;
    }
    if (st->motor_on == on) {
        return;
    }
    if (st->driver->set(st->driver_ctx, on) < 0) {
        perror("rumble set");
        return;
    }
    st->motor_on = on;
}

static inline unsigned int effect_magnitude(const struct ff_effect *eff)
{
    if (!eff || eff->type != FF_RUMBLE) {
        return 0;
    }
    unsigned int strong = eff->u.rumble.strong_magnitude;
    unsigned int weak = eff->u.rumble.weak_magnitude;
    return (strong > weak) ? strong : weak;
}

static inline void timespec_add_ms(struct timespec *ts, unsigned int ms)
{
    ts->tv_sec += ms / 1000u;
    ts->tv_nsec += (long)(ms % 1000u) * 1000000L;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec += 1;
        ts->tv_nsec -= 1000000000L;
    }
}

static inline void timespec_add_us(struct timespec *ts, unsigned int us)
{
    ts->tv_sec += us / 1000000u;
    ts->tv_nsec += (long)(us % 1000000u) * 1000L;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec += 1;
        ts->tv_nsec -= 1000000000L;
    }
}

static inline bool timespec_ge(const struct timespec *a, const struct timespec *b)
{
    if (a->tv_sec != b->tv_sec) {
        return a->tv_sec > b->tv_sec;
    }
    return a->tv_nsec >= b->tv_nsec;
}

/*
 * Recalculate motor state from current playing slots and gain.
 *
 * This is the single authority for enabling/disabling PWM mode.
 * Increments pwm_generation every call so stale handle_pwm_transition()
 * invocations can detect they are obsolete.
 */
static void recalculate_pwm(struct device_rumble_state *st)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    unsigned int max_mag = 0;
    for (int i = 0; i < DEVICE_RUMBLE_EFFECT_SLOTS; i++) {
        if (!st->playing[i] || !st->slots[i].used) {
            continue;
        }
        /* Check per-slot expiry. */
        if (st->slots[i].effect.replay.length > 0) {
            struct timespec stop = st->play_start[i];
            timespec_add_ms(&stop, st->slots[i].effect.replay.length);
            if (timespec_ge(&now, &stop)) {
                rumble_log("recalc: slot %d expired (replay.length=%u)\n",
                           i, st->slots[i].effect.replay.length);
                st->playing[i] = false;
                continue;
            }
        }
        unsigned int mag = effect_magnitude(&st->slots[i].effect);
        rumble_log("recalc: slot %d playing mag=0x%04x\n", i, mag);
        if (mag > max_mag) {
            max_mag = mag;
        }
    }

    unsigned int effective = (unsigned int)(((uint64_t)max_mag * st->gain) / 0xFFFFu);
    st->target_magnitude = effective;

    /* Increment generation before changing PWM state so any in-flight
     * handle_pwm_transition() call sees a mismatched generation. */
    st->pwm_generation++;

    if (effective < RUMBLE_MAG_THRESHOLD) {
        rumble_log("recalc: max_mag=0x%04x gain=0x%04x effective=0x%04x -> OFF\n",
                   max_mag, st->gain, effective);
        set_motor(st, false);
        st->pwm_active = false;
    } else if (effective >= RUMBLE_FULL_ON_THRESHOLD) {
        /* Common RetroArch magnitudes (e.g. strong_magnitude=0x7FFF) at full
         * gain fall well below 0xFFFF; treat anything above ~75% as full-on. */
        rumble_log("recalc: max_mag=0x%04x gain=0x%04x effective=0x%04x -> FULL ON\n",
                   max_mag, st->gain, effective);
        set_motor(st, true);
        st->pwm_active = false;
    } else {
        /* Start a fresh PWM on-phase. */
        unsigned int on_us = (effective * RUMBLE_PWM_PERIOD_US) / 0xFFFFu;
        rumble_log("recalc: max_mag=0x%04x gain=0x%04x effective=0x%04x -> PWM on_us=%u\n",
                   max_mag, st->gain, effective, on_us);
        set_motor(st, true);
        st->pwm_active = true;
        st->pwm_phase_end = now;
        timespec_add_us(&st->pwm_phase_end, on_us);
    }
}

/*
 * Advance the PWM state machine by one phase transition if the current
 * phase deadline has passed.
 *
 * The generation parameter must equal st->pwm_generation at the time the
 * caller decided to invoke this function.  If recalculate_pwm() ran since
 * then (incrementing the generation), the transition is stale and we bail
 * out immediately.
 *
 * Crucially, this function never sets pwm_active = false.  Only
 * recalculate_pwm() may do that, preventing the "on_us rounds to 0 →
 * PWM permanently dead" failure mode from PR #2.
 */
static void handle_pwm_transition(struct device_rumble_state *st,
                                   const struct timespec *now,
                                   unsigned int generation)
{
    if (!st->pwm_active) {
        return;
    }
    if (st->pwm_generation != generation) {
        /* recalculate_pwm() ran after we captured the generation; skip. */
        rumble_log("pwm_transition: stale gen=%u current=%u, skipping\n",
                   generation, st->pwm_generation);
        return;
    }
    if (!timespec_ge(now, &st->pwm_phase_end)) {
        return;
    }

    unsigned int on_us = (st->target_magnitude * RUMBLE_PWM_PERIOD_US) / 0xFFFFu;

    if (st->motor_on) {
        /* ON phase ended → start OFF phase. */
        unsigned int off_us = RUMBLE_PWM_PERIOD_US - on_us;
        rumble_log("pwm_transition: ON->OFF off_us=%u\n", off_us);
        set_motor(st, false);
        st->pwm_phase_end = *now;
        timespec_add_us(&st->pwm_phase_end,
                        off_us > 0 ? off_us : RUMBLE_PWM_PERIOD_US);
    } else {
        /* OFF phase ended → start ON phase.
         * If on_us == 0 (magnitude at boundary), do NOT kill PWM; keep motor
         * off and schedule another full period so recalculate_pwm() can
         * recover when the gain/magnitude changes. */
        if (on_us == 0) {
            rumble_log("pwm_transition: OFF->OFF (on_us=0), rescheduling\n");
            st->pwm_phase_end = *now;
            timespec_add_us(&st->pwm_phase_end, RUMBLE_PWM_PERIOD_US);
        } else {
            rumble_log("pwm_transition: OFF->ON on_us=%u\n", on_us);
            set_motor(st, true);
            st->pwm_phase_end = *now;
            timespec_add_us(&st->pwm_phase_end, on_us);
        }
    }
}

/*
 * Check for expired effect slots and drive PWM transitions.
 *
 * If any slot just expired, recalculate_pwm() is called and
 * handle_pwm_transition() is skipped for this iteration to prevent the
 * race where a fresh recalculate is immediately overridden by a stale
 * transition.
 */
static void service_timers(struct device_rumble_state *st)
{
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) < 0) {
        return;
    }

    bool any_expired = false;
    for (int i = 0; i < DEVICE_RUMBLE_EFFECT_SLOTS; i++) {
        if (!st->playing[i] || !st->slots[i].used) {
            continue;
        }
        if (st->slots[i].effect.replay.length == 0) {
            continue;
        }
        struct timespec stop = st->play_start[i];
        timespec_add_ms(&stop, st->slots[i].effect.replay.length);
        if (timespec_ge(&now, &stop)) {
            any_expired = true;
            break;
        }
    }

    if (any_expired) {
        rumble_log("service_timers: slot expired, calling recalculate_pwm\n");
        recalculate_pwm(st);
        return;
    }

    /* No recalculation this iteration — safe to advance PWM phase. */
    unsigned int gen = st->pwm_generation;
    handle_pwm_transition(st, &now, gen);
}

/*
 * Return the poll timeout in milliseconds: the minimum of the next PWM phase
 * deadline, the next effect slot expiry, and a 200 ms fallback.  Always at
 * least 1 ms.
 */
static int compute_poll_timeout(const struct device_rumble_state *st)
{
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) < 0) {
        return 200;
    }

    long min_ms = 200;

    if (st->pwm_active) {
        long ms = (st->pwm_phase_end.tv_sec - now.tv_sec) * 1000L
                  + (st->pwm_phase_end.tv_nsec - now.tv_nsec) / 1000000L;
        if (ms < min_ms) {
            min_ms = ms;
        }
    }

    for (int i = 0; i < DEVICE_RUMBLE_EFFECT_SLOTS; i++) {
        if (!st->playing[i] || !st->slots[i].used) {
            continue;
        }
        if (st->slots[i].effect.replay.length == 0) {
            continue;
        }
        struct timespec stop = st->play_start[i];
        timespec_add_ms(&stop, st->slots[i].effect.replay.length);
        long ms = (stop.tv_sec - now.tv_sec) * 1000L
                  + (stop.tv_nsec - now.tv_nsec) / 1000000L;
        if (ms < min_ms) {
            min_ms = ms;
        }
    }

    return (min_ms > 1) ? (int)min_ms : 1;
}

static void handle_play_event(struct device_rumble_state *st, int effect_id, int value)
{
    if (effect_id < 0 || effect_id >= DEVICE_RUMBLE_EFFECT_SLOTS) {
        rumble_log("play_event: effect_id=%d out of range\n", effect_id);
        return;
    }

    if (value == 0) {
        rumble_log("play_event: effect_id=%d STOP (was playing=%d)\n",
                   effect_id, (int)st->playing[effect_id]);
        st->playing[effect_id] = false;
        recalculate_pwm(st);
        return;
    }

    if (!st->slots[effect_id].used || st->slots[effect_id].effect.type != FF_RUMBLE) {
        rumble_log("play_event: effect_id=%d START but slot unused or not FF_RUMBLE\n",
                   effect_id);
        return;
    }

    rumble_log("play_event: effect_id=%d START mag=0x%04x\n",
               effect_id, effect_magnitude(&st->slots[effect_id].effect));
    clock_gettime(CLOCK_MONOTONIC, &st->play_start[effect_id]);
    st->playing[effect_id] = true;
    recalculate_pwm(st);
}

static void handle_upload(struct device_rumble_state *st, int fd, uint32_t request_id)
{
    struct uinput_ff_upload upload;
    memset(&upload, 0, sizeof(upload));
    upload.request_id = request_id;
    upload.retval = 0;

    if (ioctl(fd, UI_BEGIN_FF_UPLOAD, &upload) < 0) {
        perror("UI_BEGIN_FF_UPLOAD");
        return;
    }

    int id = upload.effect.id;
    if (upload.effect.type != FF_RUMBLE || id < 0 || id >= DEVICE_RUMBLE_EFFECT_SLOTS) {
        rumble_log("upload: effect id=%d type=%d rejected (not FF_RUMBLE or out of range)\n",
                   id, upload.effect.type);
        upload.retval = -EINVAL;
    } else {
        rumble_log("upload: effect id=%d strong=0x%04x weak=0x%04x replay.length=%u\n",
                   id,
                   upload.effect.u.rumble.strong_magnitude,
                   upload.effect.u.rumble.weak_magnitude,
                   upload.effect.replay.length);
        st->slots[id].effect = upload.effect;
        st->slots[id].used = true;
        if (st->playing[id]) {
            recalculate_pwm(st);
        }
    }

    if (ioctl(fd, UI_END_FF_UPLOAD, &upload) < 0) {
        perror("UI_END_FF_UPLOAD");
    }
}

static void handle_erase(struct device_rumble_state *st, int fd, uint32_t request_id)
{
    struct uinput_ff_erase erase;
    memset(&erase, 0, sizeof(erase));
    erase.request_id = request_id;
    erase.retval = 0;

    if (ioctl(fd, UI_BEGIN_FF_ERASE, &erase) < 0) {
        perror("UI_BEGIN_FF_ERASE");
        return;
    }

    int id = (int)erase.effect_id;
    if (id < 0 || id >= DEVICE_RUMBLE_EFFECT_SLOTS) {
        rumble_log("erase: effect id=%d out of range\n", id);
        erase.retval = -EINVAL;
    } else {
        rumble_log("erase: effect id=%d\n", id);
        st->slots[id].used = false;
        st->playing[id] = false;
        recalculate_pwm(st);
    }

    if (ioctl(fd, UI_END_FF_ERASE, &erase) < 0) {
        perror("UI_END_FF_ERASE");
    }
}

int device_rumble_init(struct device_rumble_state *st, const struct device_rumble_driver *driver)
{
    if (!st || !driver) {
        return -1;
    }
    memset(st, 0, sizeof(*st));
    st->gain = 0xFFFF;
    st->ff_fd = -1;
    st->driver = driver;

    if (!driver->init || !driver->set || !driver->close || driver->ctx_size == 0) {
        return -1;
    }

    st->driver_ctx = calloc(1, driver->ctx_size);
    if (!st->driver_ctx) {
        return -1;
    }

    if (driver->init(st->driver_ctx) < 0) {
        free(st->driver_ctx);
        st->driver_ctx = NULL;
        st->driver = NULL;
        return -1;
    }

    st->initialized = true;
    return 0;
}

void device_rumble_close(struct device_rumble_state *st)
{
    if (!st) {
        return;
    }
    if (st->driver && st->driver_ctx && st->driver->close) {
        st->driver->close(st->driver_ctx);
    }
    free(st->driver_ctx);
    memset(st, 0, sizeof(*st));
}

static void *ff_thread_fn(void *arg)
{
    struct device_rumble_state *st = arg;
    int fd = st->ff_fd;

    rumble_log_open();
    rumble_log("ff_thread_fn: started fd=%d\n", fd);

    struct pollfd pfd = {.fd = fd, .events = POLLIN};

    while (st->ff_running) {
        int timeout_ms = compute_poll_timeout(st);

        pfd.revents = 0;
        int r = poll(&pfd, 1, timeout_ms);

        if (r < 0) {
            if (errno == EINTR) {
                continue;
            }
            perror("FF thread: poll failed");
            break;
        }

        if (r == 0) {
            rumble_log("ff_thread_fn: poll timeout (%d ms)\n", timeout_ms);
            service_timers(st);
            continue;
        }

        if (!(pfd.revents & POLLIN)) {
            continue;
        }

        rumble_log("ff_thread_fn: poll data ready\n");

        struct input_event ev;
        for (;;) {
            ssize_t n = read(fd, &ev, sizeof(ev));
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    break;
                }
                if (errno == EINTR) {
                    continue;
                }
                perror("FF thread: read failed");
                st->ff_running = 0;
                break;
            }
            if (n == 0) {
                break;
            }
            if ((size_t)n < sizeof(ev)) {
                break;
            }

            rumble_log("ff_thread_fn: event type=%u code=%u value=%d\n",
                       ev.type, ev.code, ev.value);

            if (ev.type == EV_UINPUT) {
                if (ev.code == UI_FF_UPLOAD) {
                    handle_upload(st, fd, (uint32_t)ev.value);
                } else if (ev.code == UI_FF_ERASE) {
                    handle_erase(st, fd, (uint32_t)ev.value);
                }
            } else if (ev.type == EV_FF) {
                if (ev.code == FF_GAIN) {
                    unsigned int gain = (unsigned int)ev.value;
                    if (gain > 0xFFFF) {
                        gain = 0xFFFF;
                    }
                    rumble_log("ff_thread_fn: FF_GAIN new=0x%04x old=0x%04x\n",
                               gain, st->gain);
                    st->gain = gain;
                    recalculate_pwm(st);
                } else {
                    handle_play_event(st, (int)ev.code, (int)ev.value);
                }
            }
        }

        service_timers(st);
    }

    rumble_log("ff_thread_fn: exiting\n");
    rumble_log_close();
    return NULL;
}

int device_rumble_start_thread(struct device_rumble_state *st, struct gamepad *gp)
{
    if (!st || !gp) {
        return -1;
    }
    int fd = gamepad_get_fd(gp);
    if (fd < 0) {
        return -1;
    }
    st->ff_fd = fd;
    st->ff_running = 1;
    if (pthread_create(&st->ff_thread, NULL, ff_thread_fn, st) != 0) {
        perror("pthread_create ff_thread");
        st->ff_running = 0;
        st->ff_fd = -1;
        return -1;
    }
    return 0;
}

void device_rumble_stop_thread(struct device_rumble_state *st)
{
    if (!st || !st->ff_running) {
        return;
    }
    st->ff_running = 0;
    if (pthread_join(st->ff_thread, NULL) != 0) {
        perror("pthread_join ff_thread");
    }
    st->ff_fd = -1;
}
