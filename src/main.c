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
#include <stdint.h>
#include <limits.h>

// i2c headers
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

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

#define MAX_ZONES 64 // Important: Make sure this is consistent with service.yaml: 64 for 8 * 8 resolution.

#define I2C_BUS_PATH "/dev/i2c-5"

#define MUX_ADDR 0x70
#define EXPANDER_ADDR 0x20

#define MUX_SENSOR_A 0x01   // Right
#define MUX_SENSOR_B 0x08   // Left
#define MUX_EXPANDER 0x10

#define MUX_SWITCH_DELAY_US 1000

#define SENSOR_POWER_DOWN_US 60000
#define SENSOR_BOOT_DELAY_US 60000

/*
 * Reset only if a sensor gives no parsed frame for this long.
 * With period-ms = 200, each sensor should produce roughly one frame every ~200ms.
 * 1500ms gives enough tolerance.
 */
#define WATCHDOG_TIMEOUT_MS 1500

/*
 * Data to send over the stream is not compatible with rovercom,
 * so we use the custom packet and send it using write_bytes().
 */
typedef struct __attribute__((packed)) {
    uint8_t rows;
    uint8_t cols;
    uint16_t distances[MAX_ZONES]; // Flattened distance grid matrix
} TofDataPacket;

static tmf8829_chip g_tof_chip_a; // Right
static tmf8829_chip g_tof_chip_b; // Left

static volatile sig_atomic_t g_stop_requested = 0;

static void catch_signal(int signum)
{
    (void)signum;
    g_stop_requested = 1;
}

static void mark_now(struct timespec *target)
{
    clock_gettime(CLOCK_MONOTONIC, target);
}

static long elapsed_ms_since(const struct timespec *start)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    return (now.tv_sec - start->tv_sec) * 1000L +
           (now.tv_nsec - start->tv_nsec) / 1000000L;
}

// I2C helper functions
static int set_mux_channel(int fd, unsigned char channel_mask)
{
    if (ioctl(fd, I2C_SLAVE, MUX_ADDR) < 0)
    {
        perror("Mux ioctl address select failed");
        return -1;
    }

    if (write(fd, &channel_mask, 1) != 1)
    {
        perror("Mux channel write failed");
        return -1;
    }

    return 0;
}

static int set_expander_reg(int fd, unsigned char reg, unsigned char val)
{
    if (ioctl(fd, I2C_SLAVE, EXPANDER_ADDR) < 0)
    {
        perror("Expander ioctl address select failed");
        return -1;
    }

    unsigned char buf[2] = {reg, val};

    if (write(fd, buf, 2) != 2)
    {
        perror("Expander register write failed");
        return -1;
    }

    return 0;
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

static int power_cycle_sensors(int mux_fd)
{
    printf("[RESET] Power-cycling both TMF8829 sensors...\n");

    // Connect to IO expander
    if (set_mux_channel(mux_fd, MUX_EXPANDER) != 0)
    {
        return -1;
    }

    usleep(2000);

    // use expander to reset sensors by power-cycling them
    if (set_expander_reg(mux_fd, 0x03, 0x66) != 0)
    {
        return -1;
    }

    usleep(2000);

    // Set state low on active pins.
    if (set_expander_reg(mux_fd, 0x01, 0x00) != 0)
    {
        return -1;
    }

    usleep(SENSOR_POWER_DOWN_US);

    // Set state high on active pins.
    if (set_expander_reg(mux_fd, 0x01, 0x99) != 0)
    {
        return -1;
    }

    usleep(SENSOR_BOOT_DELAY_US);

    printf("[RESET] Power-cycle complete\n");

    return 0;
}

static void fill_tof_config(
    tmf8829_cfg_t *tof_cfg,
    int threshold,
    int period,
    int iterations,
    int short_iterations,
    int dualMode,
    int preConfiguration
)
{
    memset(tof_cfg, 0, sizeof(*tof_cfg));

    tof_cfg->conf_threshold = threshold;
    tof_cfg->deadtime = 60;
    tof_cfg->period = period;

    /*
     * Result format:
     * 0x01 means one peak per zone.
     * That gives us one distance value per pixel/zone.
     */
    tof_cfg->resultFormat = 0x01;
    tof_cfg->resultFormat =
        (tof_cfg->resultFormat & ~TMF8829_CFG_RESULT_FORMAT_NR_PEAKS_MASK) |
        (1 & TMF8829_CFG_RESULT_FORMAT_NR_PEAKS_MASK);

    tof_cfg->iteration = iterations;
    tof_cfg->shortIteration = short_iterations;

    /*
     * Keep histogram disabled. 
     * Enabling histogram caused I2C read failures on this setup.
     */
    tof_cfg->histogram_dump = 0;

    tof_cfg->dualMode = dualMode;
    tof_cfg->fpMode = preConfiguration;
}

static int init_sensor(
    int mux_fd,
    unsigned char mux_channel,
    const char *name,
    tmf8829_chip *tof_chip,
    int preConfiguration,
    tmf8829_cfg_t *tof_cfg
)
{
    printf("[INIT] Initializing %s...\n", name);

    if (set_mux_channel(mux_fd, mux_channel) != 0)
    {
        printf("[INIT] Failed to select mux channel for %s\n", name);
        return -1;
    }

    usleep(MUX_SWITCH_DELAY_US);

    memset(tof_chip, 0, sizeof(*tof_chip));

#ifdef ENABLE_JSON_LOGGING
    tof_chip->json_enabled = 0;
#endif

#ifdef ENABLE_KEYSTONE
    tof_chip->keystoneEnabled = 0;
#endif

    tmf8829_set_busType(tof_chip, 0); // I2C

    if (tmf8829_probe(tof_chip) == -1)
    {
        printf("[INIT] Failed to probe %s\n", name);
        return -1;
    }

    if (tmf8829ConfigMode(tof_chip, preConfiguration) != 0)
    {
        printf("[INIT] Failed to configure mode for %s\n", name);
        return -1;
    }

    if (tmf8829SettingConfiguration(tof_chip, tof_cfg) != 0)
    {
        printf("[INIT] Failed to apply configuration for %s\n", name);
        return -1;
    }

    if (tmf8829StartMeasurement(&tof_chip->tof_core) != 0)
    {
        printf("[INIT] Failed to start measurement for %s\n", name);
        return -1;
    }

    printf("[INIT] %s config applied: period=%d iterations=%d shortIterations=%d threshold=%d histogram_dump=%d\n",
           name,
           tof_cfg->period,
           tof_cfg->iteration,
           tof_cfg->shortIteration,
           tof_cfg->conf_threshold,
           tof_cfg->histogram_dump);

    printf("[INIT] %s started successfully\n", name);

    return 0;
}

static int init_both_sensors(
    int mux_fd,
    tmf8829_chip *tof_chip_a,
    tmf8829_chip *tof_chip_b,
    int preConfiguration,
    tmf8829_cfg_t *tof_cfg
)
{
    // Sensor A / Right
    if (init_sensor(
            mux_fd,
            MUX_SENSOR_A,
            "Sensor A / Right",
            tof_chip_a,
            preConfiguration,
            tof_cfg
        ) != 0)
    {
        return -1;
    }

    // Sensor B / Left
    if (init_sensor(
            mux_fd,
            MUX_SENSOR_B,
            "Sensor B / Left",
            tof_chip_b,
            preConfiguration,
            tof_cfg
        ) != 0)
    {
        return -1;
    }

    return 0;
}

static int reset_and_reinit_both_sensors(
    int mux_fd,
    tmf8829_chip *tof_chip_a,
    tmf8829_chip *tof_chip_b,
    int preConfiguration,
    tmf8829_cfg_t *tof_cfg
)
{
    printf("[WATCHDOG] Resetting and reinitializing both sensors...\n");

    /*
     * Do not call tmf8829StopMeasurement().
     * It previously hung on this setup.
     * Hardware reset is safer here.
     */
    if (power_cycle_sensors(mux_fd) != 0)
    {
        printf("[WATCHDOG] Power cycle failed\n");
        return -1;
    }

    if (init_both_sensors(
            mux_fd,
            tof_chip_a,
            tof_chip_b,
            preConfiguration,
            tof_cfg
        ) != 0)
    {
        printf("[WATCHDOG] Reinitialization failed\n");
        return -1;
    }

    printf("[WATCHDOG] Reset and reinitialization complete\n");

    return 0;
}

static void fill_packet_from_parser(TofDataPacket *packet, tmf8829FrameParser_t *parser)
{
    memset(packet, 0, sizeof(*packet));

    packet->rows = parser->frame.numRows;
    packet->cols = parser->frame.numCols;

    int total_pixels = packet->rows * packet->cols;

    if (total_pixels > MAX_ZONES)
    {
        total_pixels = MAX_ZONES;
    }

    for (int i = 0; i < total_pixels; i++)
    {
        packet->distances[i] = parser->pixelResults[i].peaks[0].distance;
    }
}

/*
 * This prints the exact custom packet format:
 *
 * rows
 * cols
 * flattened distances[64]
 *
 * The matrix display is only for readability. The actual data order is:
 * distances[row * cols + col]
 */
static void print_tof_packet(
    const char *name,
    int local_frame_counter,
    int sensor_frame_number,
    const TofDataPacket *packet
)
{
    int rows = packet->rows;
    int cols = packet->cols;
    int total = rows * cols;

    if (total <= 0 || total > MAX_ZONES)
    {
        printf("[%s] PACKET local_frame=%d sensor_frame=%d invalid rows=%d cols=%d\n",
               name,
               local_frame_counter,
               sensor_frame_number,
               rows,
               cols);
        return;
    }

    printf("[%s] PACKET local_frame=%d sensor_frame=%d rows=%u cols=%u flattened=[",
           name,
           local_frame_counter,
           sensor_frame_number,
           packet->rows,
           packet->cols);

    for (int i = 0; i < total; i++)
    {
        printf("%u", packet->distances[i]);

        if (i < total - 1)
        {
            printf(",");
        }
    }

    printf("]\n");

    printf("[%s] PACKET matrix:\n", name);

    for (int r = 0; r < rows; r++)
    {
        printf("  ");

        for (int c = 0; c < cols; c++)
        {
            int idx = r * cols + c;
            printf("%5u ", packet->distances[idx]);
        }

        printf("\n");
    }
}

static void print_tof_summary(const char *name, int local_frame_counter, tmf8829FrameParser_t *parser)
{
    int rows = parser->frame.numRows;
    int cols = parser->frame.numCols;
    int total = rows * cols;

    if (total <= 0)
    {
        printf("[%s] SUMMARY local_frame=%d sensor_frame=%d invalid dimensions rows=%d cols=%d\n",
               name,
               local_frame_counter,
               parser->frame.frameNumber,
               rows,
               cols);
        return;
    }

    if (total > MAX_ZONES)
    {
        total = MAX_ZONES;
    }

    uint16_t min_nonzero = UINT16_MAX;
    uint16_t max_distance = 0;

    int zero_count = 0;
    int nonzero_count = 0;

    for (int i = 0; i < total; i++)
    {
        uint16_t d = parser->pixelResults[i].peaks[0].distance;

        if (d == 0)
        {
            zero_count++;
            continue;
        }

        nonzero_count++;

        if (d < min_nonzero)
        {
            min_nonzero = d;
        }

        if (d > max_distance)
        {
            max_distance = d;
        }
    }

    printf("[%s] SUMMARY local_frame=%d sensor_frame=%d rows=%d cols=%d zero=%d nonzero=%d/%d ",
           name,
           local_frame_counter,
           parser->frame.frameNumber,
           rows,
           cols,
           zero_count,
           nonzero_count,
           total);

    if (nonzero_count > 0)
    {
        printf("min_nonzero=%u max=%u\n", min_nonzero, max_distance);
    }
    else
    {
        printf("ALL_ZERO\n");
    }
}

static int process_one_sensor(
    int mux_fd,
    unsigned char mux_channel,
    const char *name,
    tmf8829_chip *tof_chip,
    write_stream *sensor_stream,
    int *local_frame_counter,
    struct timespec *last_parsed_frame_time,
    int debug
)
{
    if (set_mux_channel(mux_fd, mux_channel) != 0)
    {
        printf("[%s] Failed to select mux channel\n", name);
        return -1;
    }

    usleep(MUX_SWITCH_DELAY_US);

    int result = tmf8829_app_process_irq(tof_chip);

    if (result == 1)
    {
        (*local_frame_counter)++;

        /*
         * IMPORTANT:
         * We mark this as alive because a parsed frame arrived.
         * We do NOT reject frames with 0/small distances, because close-to-ground
         * readings may legitimately appear as 0 or very small values in your setup.
         */
        mark_now(last_parsed_frame_time);

        tmf8829FrameParser_t *parser = &tof_chip->frameParser;

        TofDataPacket packet;
        fill_packet_from_parser(&packet, parser);

        /*
         * Send the custom packet format over the stream:
         * uint8_t rows
         * uint8_t cols
         * uint16_t distances[64]
         */
        write_bytes(sensor_stream, (void *)&packet, sizeof(TofDataPacket));

        /*
         * Keep printing for safety/debugging.
         * Summary is printed for every parsed frame.
         * Full packet is printed every 10 local frames to avoid flooding the terminal.
         */
        print_tof_summary(name, *local_frame_counter, parser);

        if (debug && ((*local_frame_counter) % 10 == 0))
        {
            print_tof_packet(name, *local_frame_counter, parser->frame.frameNumber, &packet);
        }

        return 1;
    }

    return 0;
}

int user_program(Service service, Service_configuration *configuration)
{
    setbuf(stdout, NULL);

    if (configuration == NULL)
    {
        printf("Configuration cannot be accessed\n");
        return 1;
    }

    int mode = get_config_number(configuration, "mode", 0);
    int timeout = get_config_number(configuration, "timeout", 0);
    int threshold = get_config_number(configuration, "confidence-threshold", 6);
    int period = get_config_number(configuration, "period-ms", 200);
    int iterations = get_config_number(configuration, "iterations", 1800);
    int short_iterations = get_config_number(configuration, "short-iterations", 100);
    int debug = get_config_number(configuration, "debug", 0);

    write_stream *sensor_right = get_write_stream(&service, "sensor-right");

    if (sensor_right == NULL)
    {
        printf("Failed to create write stream 'sensor-right'\n");
        return 1;
    }

    write_stream *sensor_left = get_write_stream(&service, "sensor-left");

    if (sensor_left == NULL)
    {
        printf("Failed to create write stream 'sensor-left'\n");
        return 1;
    }

    /*
     * Do not set g_debug_enabled here.
     * The ams driver debug output is noisy and not sensor-tagged.
     * We use our own tagged summary/packet printing instead.
     */
    if (debug)
    {
        printf("[DEBUG] Debug config enabled, but internal driver frame printing is disabled.\n");
    }

    printf("TMF8829 TOF service starting\n");
    printf("mode: %d\n", mode);
    printf("timeout: %d\n", timeout);
    printf("confidence-threshold: %d\n", threshold);
    printf("period-ms: %d\n", period);
    printf("iterations: %d\n", iterations);
    printf("short-iterations: %d\n", short_iterations);
    printf("debug: %d\n", debug);

    signal(SIGINT, catch_signal);
    signal(SIGTERM, catch_signal);

    // Open control handle for native multiplexing.
    int mux_fd = open(I2C_BUS_PATH, O_RDWR);

    if (mux_fd < 0)
    {
        perror("Fatal: Failed to open I2C bus for MUX control");
        return 1;
    }

    printf("Executing initial hardware power-on and reset sequence...\n");

    if (power_cycle_sensors(mux_fd) != 0)
    {
        printf("Fatal: initial hardware reset failed\n");
        close(mux_fd);
        return 1;
    }

    int preConfiguration = 0;
    int dualMode = 0;

    calculatePreconfigurationAndDualMode(mode, &preConfiguration, &dualMode);

    tmf8829_cfg_t tof_cfg;

    fill_tof_config(
        &tof_cfg,
        threshold,
        period,
        iterations,
        short_iterations,
        dualMode,
        preConfiguration
    );

    tmf8829_chip *tof_chip_a = &g_tof_chip_a;
    tmf8829_chip *tof_chip_b = &g_tof_chip_b;

    if (init_both_sensors(
            mux_fd,
            tof_chip_a,
            tof_chip_b,
            preConfiguration,
            &tof_cfg
        ) != 0)
    {
        printf("Fatal: failed to initialize both sensors\n");
        close(mux_fd);
        return 1;
    }

    printf("Both TMF8829 measurements successfully started\n");

    time_t start_time = time(NULL);

    struct timespec last_frame_a;
    struct timespec last_frame_b;

    mark_now(&last_frame_a);
    mark_now(&last_frame_b);

    int frame_counter_a = 0;
    int frame_counter_b = 0;

    while (!g_stop_requested)
    {
        // =================================================================
        // --- Process Sensor A (Right) ---
        // =================================================================
        process_one_sensor(
            mux_fd,
            MUX_SENSOR_A,
            "RIGHT/A",
            tof_chip_a,
            sensor_right,
            &frame_counter_a,
            &last_frame_a,
            debug
        );

        // =================================================================
        // --- Process Sensor B (Left) ---
        // =================================================================
        process_one_sensor(
            mux_fd,
            MUX_SENSOR_B,
            "LEFT/B",
            tof_chip_b,
            sensor_left,
            &frame_counter_b,
            &last_frame_b,
            debug
        );

        // =================================================================
        // --- Watchdog reset ---
        // =================================================================
        long a_silent_ms = elapsed_ms_since(&last_frame_a);
        long b_silent_ms = elapsed_ms_since(&last_frame_b);

        if (a_silent_ms > WATCHDOG_TIMEOUT_MS || b_silent_ms > WATCHDOG_TIMEOUT_MS)
        {
            printf("[WATCHDOG] Missing parsed frames: A silent=%ldms, B silent=%ldms\n",
                   a_silent_ms,
                   b_silent_ms);

            if (reset_and_reinit_both_sensors(
                    mux_fd,
                    tof_chip_a,
                    tof_chip_b,
                    preConfiguration,
                    &tof_cfg
                ) != 0)
            {
                printf("[WATCHDOG] Reset failed. Will retry if frames do not recover.\n");
            }

            mark_now(&last_frame_a);
            mark_now(&last_frame_b);

            frame_counter_a = 0;
            frame_counter_b = 0;
        }

        if (timeout > 0 && (time(NULL) - start_time) >= timeout)
        {
            printf("Timeout reached (%d seconds), stopping...\n", timeout);
            g_stop_requested = 1;
        }

        usleep(1000);
    }

    printf("Stopping TMF8829 service...\n");
    fflush(stdout);

    // Cleanup Sensor A
    set_mux_channel(mux_fd, MUX_SENSOR_A);
    usleep(MUX_SWITCH_DELAY_US);
    tmf8829PrintFpsStats(&tof_chip_a->frameParser);
    tmf8829_cleanup(tof_chip_a);

    // Cleanup Sensor B
    set_mux_channel(mux_fd, MUX_SENSOR_B);
    usleep(MUX_SWITCH_DELAY_US);
    tmf8829PrintFpsStats(&tof_chip_b->frameParser);
    tmf8829_cleanup(tof_chip_b);

    // Turn off sensors via Expander on exit to guarantee cold-boot next run.
    printf("Powering down sensors via GPIO expander to prepare for future runs...\n");

    set_mux_channel(mux_fd, MUX_EXPANDER);
    usleep(2000);
    set_expander_reg(mux_fd, 0x01, 0x00);
    usleep(5000);

    // Close MUX handle.
    close(mux_fd);

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