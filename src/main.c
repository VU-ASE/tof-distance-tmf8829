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

#define MAX_ZONES 64 // Important: Make sure this is consistent with service.yaml (64 for 8 * 8 resolution) and your receiving side code.

// Data to send over the stream is not compatible with rovercom, so we (temporarily) use our custom packet and send using send_bytes()
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

// I2C helper functions
static int set_mux_channel(int fd, unsigned char channel_mask)
{
    if (ioctl(fd, I2C_SLAVE, 0x70) < 0)
    {
        printf("Mux ioctl address select failed");
        return -1;
    }
    if (write(fd, &channel_mask, 1) != 1)
    {
        printf("Mux channel write failed");
        return -1;
    }
    return 0;
}

static int set_expander_reg(int fd, unsigned char reg, unsigned char val)
{
    if (ioctl(fd, I2C_SLAVE, 0x20) < 0)
    {
        printf("Expander ioctl address select failed");
        return -1;
    }
    unsigned char buf[2] = {reg, val};
    if (write(fd, buf, 2) != 2)
    {
        printf("Expander register write failed");
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

int user_program(Service service, Service_configuration *configuration)
{
    (void)service;

    setbuf(stdout, NULL);

    if (configuration == NULL)
    {
        printf("Configuration cannot be accessed\n");
        return 1;
    }

    int mode = get_config_number(configuration, "mode", 0);
    int timeout = get_config_number(configuration, "timeout", 0);
    int threshold = get_config_number(configuration, "confidence-threshold", 6);
    int period = get_config_number(configuration, "period-ms", 33);
    int iterations = get_config_number(configuration, "iterations", 1800);
    int short_iterations = get_config_number(configuration, "short-iterations", 100);
    int debug = get_config_number(configuration, "debug", 0);


    write_stream *sensor_right = get_write_stream(&service, "sensor-right");
    if(sensor_right == NULL){
        printf("Failed to create write stream 'sensor-right'\n");
        return 1;
    }

    write_stream *sensor_left = get_write_stream(&service, "sensor-left");
    if(sensor_left == NULL){
        printf("Failed to create write stream 'sensor-left'\n");
        return 1;
    }

    if (debug)
    {
        g_debug_enabled = 1;
    }

    printf("TMF8829 TOF service starting\n");
    printf("mode: %d\n", mode);
    printf("timeout: %d\n", timeout);
    printf("period-ms: %d\n", period);
    printf("iterations: %d\n", iterations);
    printf("short-iterations: %d\n", short_iterations);
    printf("debug: %d\n", debug);

    signal(SIGINT, catch_signal);
    signal(SIGTERM, catch_signal);

    // Open control handle for native multiplexing 
    int mux_fd = open("/dev/i2c-5", O_RDWR);
    if (mux_fd < 0)
    {
        perror("Fatal: Failed to open /dev/i2c-5 for MUX control");
        return 1;
    }

    printf("Executing hardware power-on and reset sequence...\n");
    
    // Connect to IO expander
    set_mux_channel(mux_fd, 0x10); 
    usleep(2000);
    
    // Set input/output pins
    set_expander_reg(mux_fd, 0x03, 0x66);
    usleep(2000);

    // Set state low on active pins (clear state)
    set_expander_reg(mux_fd, 0x01, 0x00); 
    usleep(30000); 

    // Set state high on active pins
    set_expander_reg(mux_fd, 0x01, 0x99); 
    usleep(60000); 

    int preConfiguration = 0;
    int dualMode = 0;
    time_t start_time;

    tmf8829_cfg_t tof_cfg;
    tmf8829_chip *tof_chip_a = &g_tof_chip_a;
    tmf8829_chip *tof_chip_b = &g_tof_chip_b;

    memset(tof_chip_a, 0, sizeof(*tof_chip_a));
    memset(tof_chip_b, 0, sizeof(*tof_chip_b));

    calculatePreconfigurationAndDualMode(mode, &preConfiguration, &dualMode);

    tof_cfg.conf_threshold = threshold;
    tof_cfg.deadtime = 60;
    tof_cfg.period = period;

    tof_cfg.resultFormat = 0x01;
    tof_cfg.resultFormat =
        (tof_cfg.resultFormat & ~TMF8829_CFG_RESULT_FORMAT_NR_PEAKS_MASK) |
        (1 & TMF8829_CFG_RESULT_FORMAT_NR_PEAKS_MASK);

    tof_cfg.iteration = iterations;
    tof_cfg.shortIteration = short_iterations;
    tof_cfg.histogram_dump = 0;
    tof_cfg.dualMode = dualMode;
    tof_cfg.fpMode = preConfiguration;

#ifdef ENABLE_JSON_LOGGING
    tof_chip_a->json_enabled = 0;
    tof_chip_b->json_enabled = 0;
#endif

#ifdef ENABLE_KEYSTONE
    tof_chip_a->keystoneEnabled = 0;
    tof_chip_b->keystoneEnabled = 0;
#endif

    tmf8829_set_busType(tof_chip_a, 0);
    tmf8829_set_busType(tof_chip_b, 0);

    // Sensor A
    printf("Initializing Sensor A (Right)...\n");
    set_mux_channel(mux_fd, 0x01);
    usleep(5000); 

    if (tmf8829_probe(tof_chip_a) == -1 ||
        tmf8829ConfigMode(tof_chip_a, preConfiguration) != 0 ||
        tmf8829SettingConfiguration(tof_chip_a, &tof_cfg) != 0 ||
        tmf8829StartMeasurement(&tof_chip_a->tof_core) != 0)
    {
        printf("Failed to initialize Sensor A\n");
        close(mux_fd);
        return 1;
    }

    // Sensor B
    printf("Initializing Sensor B (Left)...\n");
    set_mux_channel(mux_fd, 0x08);
    usleep(5000);

    if (tmf8829_probe(tof_chip_b) == -1 ||
        tmf8829ConfigMode(tof_chip_b, preConfiguration) != 0 ||
        tmf8829SettingConfiguration(tof_chip_b, &tof_cfg) != 0 ||
        tmf8829StartMeasurement(&tof_chip_b->tof_core) != 0)
    {
        printf("Failed to initialize Sensor B\n");
        close(mux_fd);
        return 1;
    }

    printf("Both TMF8829 measurements successfully started\n");
    start_time = time(NULL);

    while (!g_stop_requested)
    {
        TofDataPacket packet;

        // --- Process Sensor A (Right) ---
        printf("------Reading From Sensor A start-----\n");
        set_mux_channel(mux_fd, 0x01);
        
        
        // Package and send Sensor A data
        if(tmf8829_app_process_irq(tof_chip_a) == 1){
            tmf8829FrameParser_t *parser = &tof_chip_a->frameParser;
            int total_pixels = parser->frame.numRows * parser->frame.numCols;
            if (total_pixels > MAX_ZONES) total_pixels = MAX_ZONES;

            memset(&packet, 0, sizeof(packet));
            packet.rows = parser->frame.numRows;
            packet.cols = parser->frame.numCols;

            printf("cols A: %d\n", packet.cols);
            printf("rows A: %d\n", packet.rows);
            
            for (int i = 0; i < total_pixels; i++)
            {
                packet.distances[i] = parser->pixelResults[i].peaks[0].distance;
            }

            write_bytes(sensor_right, (void*)&packet, sizeof(TofDataPacket));

            
        }
        printf("------Reading From Sensor A end  -----\n");

        usleep(5000);

        // --- Process Sensor B (Left) ---
        printf("------Reading From Sensor B start-----\n");
        set_mux_channel(mux_fd, 0x08);
        
        
        // Package and send Sensor B data
        if(tmf8829_app_process_irq(tof_chip_b) == 1){
            tmf8829FrameParser_t *parser = &tof_chip_b->frameParser;
            int total_pixels = parser->frame.numRows * parser->frame.numCols;
            if (total_pixels > MAX_ZONES) total_pixels = MAX_ZONES;

            memset(&packet, 0, sizeof(packet));
            packet.rows = parser->frame.numRows;
            packet.cols = parser->frame.numCols;

            printf("cols B: %d\n", packet.cols);
            printf("rows B: %d\n", packet.rows);


            for (int i = 0; i < total_pixels; i++)
            {
                packet.distances[i] = parser->pixelResults[i].peaks[0].distance;
            }

            write_bytes(sensor_left, (void*)&packet, sizeof(TofDataPacket));
            
        }
        printf("------Reading From Sensor B end-----\n");

        if (timeout > 0 && (time(NULL) - start_time) >= timeout)
        {
            printf("Timeout reached (%d seconds), stopping...\n", timeout);
            g_stop_requested = 1;
        }
    }

    printf("Stopping TMF8829 measurement...\n");
    fflush(stdout);

    // Cleanup Sensor A
    set_mux_channel(mux_fd, 0x01);
    tmf8829PrintFpsStats(&tof_chip_a->frameParser);
    tmf8829_cleanup(tof_chip_a);

    // Cleanup Sensor B
    set_mux_channel(mux_fd, 0x08);
    tmf8829PrintFpsStats(&tof_chip_b->frameParser);
    tmf8829_cleanup(tof_chip_b);

    // Turn off sensors via Expander on exit to guarantee cold-boot next run
    printf("Powering down sensors via GPIO expander to prepare for future runs...\n");
    set_mux_channel(mux_fd, 0x10); // Route back to Expander ONLY
    usleep(2000);
    set_expander_reg(mux_fd, 0x01, 0x00); // Set state low
    usleep(5000);

    // Close MUX handle
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
    sleep(2); // Give enough time for main program to clean up.
    return 0;
}

int main()
{
    return run(user_program, on_terminate);
}