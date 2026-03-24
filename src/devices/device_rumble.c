#define _POSIX_C_SOURCE 200809L
#include "device_rumble.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <poll.h>

/* Software PWM parameters for intensity modulation of the binary motor. */
#define RUMBLE_PWM_PERIOD_US 20000u   /* 20 ms period = 50 Hz */
#define RUMBLE_MAG_THRESHOLD 0x0CCCu  /* ~5% of 0xFFFF — below this, skip the motor */

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
    while (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec++;
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

/* Returns milliseconds until slot i's replay expires, or -1 if no timeout. */
static long slot_ms_until_expiry(const struct device_rumble_state *st, int i,
                                  const struct timespec *now)
{
    const struct ff_effect *eff = &st->slots[i].effect;
    if (eff->replay.length == 0) {
        return -1;
    }
    struct timespec expiry = st->play_start[i];
    timespec_add_ms(&expiry, eff->replay.length);
    long ms = (expiry.tv_sec - now->tv_sec) * 1000L
              + (expiry.tv_nsec - now->tv_nsec) / 1000000L;
    return ms;
}

/*
 * Recalculate the effective motor intensity from all currently-playing,
 * non-expired effect slots and the global gain.  Expire any timed-out slots
 * and update the PWM state accordingly.
 *
 *  effective = max_magnitude_across_playing_slots * gain / 0xFFFF
 *
 * Three operating modes:
 *   effective < THRESHOLD   → motor off,   no PWM
 *   effective >= 0xFFFF     → motor full on, no PWM
 *   otherwise               → software PWM (50 Hz), duty ∝ effective
 */
static void recalculate_pwm(struct device_rumble_state *st)
{
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) < 0) {
        return;
    }

    unsigned int max_mag = 0;
    for (int i = 0; i < DEVICE_RUMBLE_EFFECT_SLOTS; i++) {
        if (!st->playing[i] || !st->slots[i].used) {
            continue;
        }
        if (st->slots[i].effect.replay.length > 0 &&
            slot_ms_until_expiry(st, i, &now) <= 0) {
            st->playing[i] = false;
            continue;
        }
        unsigned int mag = effect_magnitude(&st->slots[i].effect);
        if (mag > max_mag) {
            max_mag = mag;
        }
    }

    unsigned int effective =
        (unsigned int)(((uint64_t)max_mag * st->gain) / 0xFFFFu);
    st->target_magnitude = effective;

    if (effective < RUMBLE_MAG_THRESHOLD) {
        set_motor(st, false);
        st->pwm_active = false;
        return;
    }

    if (effective >= 0xFFFFu) {
        set_motor(st, true);
        st->pwm_active = false;
        return;
    }

    /* Start a fresh PWM on-phase. */
    unsigned int on_us = (effective * RUMBLE_PWM_PERIOD_US) / 0xFFFFu;
    if (on_us == 0) {
        set_motor(st, false);
        st->pwm_active = false;
        return;
    }
    set_motor(st, true);
    st->pwm_active = true;
    st->pwm_phase_end = now;
    timespec_add_us(&st->pwm_phase_end, on_us);
}

/*
 * Handle a PWM phase transition when pwm_phase_end has been reached.
 * Toggles the motor on→off or off→on and schedules the next phase.
 */
static void handle_pwm_transition(struct device_rumble_state *st,
                                   const struct timespec *now)
{
    if (!st->pwm_active || !timespec_ge(now, &st->pwm_phase_end)) {
        return;
    }

    unsigned int on_us = (st->target_magnitude * RUMBLE_PWM_PERIOD_US) / 0xFFFFu;

    if (st->motor_on) {
        /* ON phase ended → start OFF phase. */
        unsigned int off_us = RUMBLE_PWM_PERIOD_US - on_us;
        if (off_us == 0) {
            /* Essentially 100% duty — stay on and reschedule a full period. */
            st->pwm_phase_end = *now;
            timespec_add_us(&st->pwm_phase_end, RUMBLE_PWM_PERIOD_US);
            return;
        }
        set_motor(st, false);
        st->pwm_phase_end = *now;
        timespec_add_us(&st->pwm_phase_end, off_us);
    } else {
        /* OFF phase ended → start ON phase. */
        if (on_us == 0) {
            set_motor(st, false);
            st->pwm_active = false;
            return;
        }
        set_motor(st, true);
        st->pwm_phase_end = *now;
        timespec_add_us(&st->pwm_phase_end, on_us);
    }
}

/*
 * Check for expired slots and handle PWM phase transitions.
 * Called on every poll wakeup (timeout or event).
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
        if (st->slots[i].effect.replay.length > 0 &&
            slot_ms_until_expiry(st, i, &now) <= 0) {
            st->playing[i] = false;
            any_expired = true;
        }
    }

    if (any_expired) {
        recalculate_pwm(st);
        return;
    }

    handle_pwm_transition(st, &now);
}

/* Compute the next poll timeout in milliseconds. */
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
        long ms = slot_ms_until_expiry(st, i, &now);
        if (ms >= 0 && ms < min_ms) {
            min_ms = ms;
        }
    }

    return (min_ms > 1) ? (int)min_ms : 1;
}

static void handle_play_event(struct device_rumble_state *st, int effect_id, int value)
{
    if (effect_id < 0 || effect_id >= DEVICE_RUMBLE_EFFECT_SLOTS) {
        return;
    }

    if (value == 0) {
        st->playing[effect_id] = false;
    } else {
        if (!st->slots[effect_id].used ||
            st->slots[effect_id].effect.type != FF_RUMBLE) {
            return;
        }
        st->playing[effect_id] = true;
        if (clock_gettime(CLOCK_MONOTONIC, &st->play_start[effect_id]) < 0) {
            /* Cannot record start time; skip starting this effect. */
            st->playing[effect_id] = false;
            return;
        }
    }

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
        upload.retval = -EINVAL;
    } else {
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
        erase.retval = -EINVAL;
    } else {
        st->slots[id].used = false;
        if (st->playing[id]) {
            st->playing[id] = false;
            recalculate_pwm(st);
        }
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
    st->gain = 0xFFFFu;  /* Default to 100% — unchanged for clients that don't send FF_GAIN. */
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
            service_timers(st);
            continue;
        }

        if (!(pfd.revents & POLLIN)) {
            service_timers(st);
            continue;
        }

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

            if (ev.type == EV_UINPUT) {
                if (ev.code == UI_FF_UPLOAD) {
                    handle_upload(st, fd, (uint32_t)ev.value);
                } else if (ev.code == UI_FF_ERASE) {
                    handle_erase(st, fd, (uint32_t)ev.value);
                }
            } else if (ev.type == EV_FF) {
                if (ev.code == FF_GAIN) {
                    unsigned int g = (unsigned int)ev.value;
                    if (g > 0xFFFFu) {
                        g = 0xFFFFu;
                    }
                    st->gain = g;
                    recalculate_pwm(st);
                } else {
                    handle_play_event(st, (int)ev.code, (int)ev.value);
                }
            }
        }

        service_timers(st);
    }

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
