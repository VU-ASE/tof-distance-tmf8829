/*
 ************************************************************************************
 * TMF8829 TOF Rover Service
 ************************************************************************************
 */

#include "../lib/include/roverlib.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>

#include "tmf8829.h"
#include "tmf8829_driver.h"
#include "tmf8829_shim.h"
#include "tmf8829_frameparser.h"

#ifdef ENABLE_JSON_LOGGING
#include "tmf8829_json.h"
#endif

#ifdef ENABLE_KEYSTONE
#include "tmf8829_keystone.h"
#endif

static tmf8829_chip g_tof_chip;
static volatile sig_atomic_t g_stop_requested = 0;

static void catch_signal(int signum)
{
    (void)signum;
    g_stop_requested = 1;
}

static int get_config_number(Service_configuration *configuration, const char *name, int default_value)
{
    double *value = get_float_value_safe(configuration, (char *)name);

    if (value == NULL)
    {
        printf("Could not read config value '%s', using default %d\n", name, default_value);
        return default_value;
    }

    return (int)(*value);
}

int user_program(Service service, Service_configuration *configuration)
{
    (void)service;

    setbuf(stdout, NULL);

    if (configuration == NULL)
    {
        printf("Configuration cannot be accessed\n");
        return 1;
    }

    int bus_type = get_config_number(configuration, "bus-type", 0);
    int mode = get_config_number(configuration, "mode", 0);
    int timeout = get_config_number(configuration, "timeout", 0);
    int threshold = get_config_number(configuration, "confidence-threshold", 6);
    int period = get_config_number(configuration, "period-ms", 33);
    int iterations = get_config_number(configuration, "iterations", 1800);
    int short_iterations = get_config_number(configuration, "short-iterations", 100);
    int debug = get_config_number(configuration, "debug", 0);

    if (debug)
    {
        g_debug_enabled = 1;
    }

    printf("TMF8829 TOF service starting\n");
    printf("bus-type: %d (%s)\n", bus_type, bus_type == 0 ? "I2C" : "SPI");
    printf("mode: %d\n", mode);
    printf("timeout: %d\n", timeout);
    printf("confidence-threshold: %d\n", threshold);
    printf("period-ms: %d\n", period);
    printf("iterations: %d\n", iterations);
    printf("short-iterations: %d\n", short_iterations);
    printf("debug: %d\n", debug);

    signal(SIGINT, catch_signal);
    signal(SIGTERM, catch_signal);

    int preConfiguration = 0;
    int dualMode = 0;
    time_t start_time;

    tmf8829_cfg_t tof_cfg;
    tmf8829_chip *tof_chip = &g_tof_chip;

    memset(tof_chip, 0, sizeof(*tof_chip));

    /*
     * IMPORTANT:
     *
     * The original ams-OSRAM example tried to toggle GPIO 40:
     *
     *   tof_chip->gpiod_enable = 40;
     *   enablePinLow(tof_chip);
     *   enablePinHigh(tof_chip);
     *
     * On our Debix rover, the TMF8829 EN pin is not connected to GPIO 40.
     * That caused:
     *
     *   Error initializing CE pin
     *
     * We skip GPIO enable control for now because the sensor is already visible
     * on /dev/i2c-5 at address 0x41.
     */

    calculatePreconfigurationAndDualMode(mode, &preConfiguration, &dualMode);

    tof_cfg.conf_threshold = threshold;
    tof_cfg.deadtime = 60;
    tof_cfg.period = period;

    /*
     * Result format:
     * 0x01 enables basic result output.
     * We keep one object/peak for now.
     */
    tof_cfg.resultFormat = 0x01;
    tof_cfg.resultFormat =
        (tof_cfg.resultFormat & ~TMF8829_CFG_RESULT_FORMAT_NR_PEAKS_MASK) |
        (1 & TMF8829_CFG_RESULT_FORMAT_NR_PEAKS_MASK);

    tof_cfg.iteration = iterations;
    tof_cfg.shortIteration = short_iterations;

#ifdef ENABLE_HISTOGRAM
    tof_cfg.histogram_dump = 0;
#else
    tof_cfg.histogram_dump = 0;
#endif

    tof_cfg.dualMode = dualMode;
    tof_cfg.fpMode = preConfiguration;

#ifdef ENABLE_JSON_LOGGING
    tof_chip->json_enabled = 0;
#endif

#ifdef ENABLE_KEYSTONE
    tof_chip->keystoneEnabled = 0;
#endif

    /*
     * In this driver:
     * bus_type = 0 means I2C
     * bus_type = 1 means SPI
     */
    tmf8829_set_busType(tof_chip, bus_type);

    if (tmf8829_probe(tof_chip) == -1)
    {
        printf("Failed to probe TMF8829\n");
        return 1;
    }

    if (tmf8829ConfigMode(tof_chip, preConfiguration) != 0)
    {
        printf("Failed to configure TMF8829 mode\n");
        return 1;
    }

    if (tmf8829SettingConfiguration(tof_chip, &tof_cfg) != 0)
    {
        printf("Failed to apply TMF8829 configuration\n");
        return 1;
    }

    if (tmf8829StartMeasurement(&tof_chip->tof_core) != 0)
    {
        printf("Failed to start TMF8829 measurement\n");
        return 1;
    }

    printf("TMF8829 measurement started\n");

    start_time = time(NULL);

    while (!g_stop_requested)
    {
        /*
         * This function reads and processes one sensor frame.
         * Currently the ams-OSRAM frame parser prints the 8x8 values.
         *
         * Later we will replace/extend this part with:
         *   - build protobuf message
         *   - write to roverlib stream
         */
        tmf8829_app_process_irq(tof_chip);

        if (timeout > 0 && (time(NULL) - start_time) >= timeout)
        {
            printf("Timeout reached (%d seconds), stopping...\n", timeout);
            g_stop_requested = 1;
        }
    }

    printf("Stopping TMF8829 measurement...\n");
    fflush(stdout);

    tmf8829PrintFpsStats(&tof_chip->frameParser);
    tmf8829_cleanup(tof_chip);
    
    printf("TMF8829 TOF service stopped\n");
    fflush(stdout);
    return 0;
}

int on_terminate(int signum)
{
    printf("Service terminated with signal %d, gracefully shutting down\n", signum);
    fflush(stdout);

    g_stop_requested = 1;

    return 0;
}

int main()
{
    return run(user_program, on_terminate);
}