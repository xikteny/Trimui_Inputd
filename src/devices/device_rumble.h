#pragma once

#include <stdbool.h>
#include <time.h>
#include <stddef.h>
#include <signal.h>
#include <pthread.h>
#include <linux/input.h>
#include <linux/uinput.h>

#include "../gamepad/uinput.h"

enum { DEVICE_RUMBLE_EFFECT_SLOTS = 4 };

struct device_rumble_driver {
    const char *name;
    size_t ctx_size;
    int  (*init)(void *ctx);
    int  (*set)(void *ctx, bool on);
    void (*close)(void *ctx);
};

struct device_rumble_slot {
    bool used;
    struct ff_effect effect;
};

struct device_rumble_state {
    struct device_rumble_slot slots[DEVICE_RUMBLE_EFFECT_SLOTS];
    bool playing[DEVICE_RUMBLE_EFFECT_SLOTS];
    struct timespec play_start[DEVICE_RUMBLE_EFFECT_SLOTS];
    unsigned int gain;
    unsigned int target_magnitude;
    struct timespec pwm_phase_end;
    bool pwm_active;
    unsigned int pwm_generation;
    const struct device_rumble_driver *driver;
    void *driver_ctx;
    bool motor_on;
    bool initialized;
    pthread_t ff_thread;
    volatile sig_atomic_t ff_running;
    int ff_fd;
};

/**
 * Initialize rumble handling using a provided hardware driver.
 *
 * @param st Rumble state to initialize.
 * @param driver Hardware driver operations (e.g., A133 or future A527).
 * @return 0 on success, -1 on failure.
 */
int device_rumble_init(struct device_rumble_state *st, const struct device_rumble_driver *driver);

/**
 * Tear down rumble handling and release driver resources.
 *
 * @param st Rumble state previously initialized.
 * @return void.
 */
void device_rumble_close(struct device_rumble_state *st);

/**
 * Spawn a dedicated pthread that services FF events immediately via poll().
 *
 * @param st Rumble state previously initialized with device_rumble_init().
 * @param gp Gamepad handle whose uinput fd is monitored for EV_UINPUT/EV_FF events.
 * @return 0 on success, -1 on failure.
 */
int device_rumble_start_thread(struct device_rumble_state *st, struct gamepad *gp);

/**
 * Signal the FF thread to stop and join it.
 *
 * @param st Rumble state with a running FF thread.
 */
void device_rumble_stop_thread(struct device_rumble_state *st);

/**
 * Accessor for the current A133 rumble driver (Smart Pro / Brick).
 *
 * @return Pointer to the driver descriptor.
 */
const struct device_rumble_driver *rumble_a133_driver(void);
