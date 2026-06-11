/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For the full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

/** @file This is the shim for raspberry pi
 * Defines, macro and functions to match the target platform.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <time.h>

#include "tmf8829_driver.h"
#include "tmf8829_shim.h"

/** @brief Debug print macros for two-level print control
 *  PRINT_DEBUG: Controlled by --debug flag, prints debug and tracking information
 *  PRINT_INFO:  Always prints, prints errors and important messages
 */
int g_debug_enabled = 0;

/////////////////////////////////////////////////

static char *sysfs_export = "/sys/class/gpio/export";
static char *sysfs_unexport = "/sys/class/gpio/unexport";
static char *sysfs_gpio = "/sys/class/gpio/gpio";

enum gpio_direction {
    IN_DIR,
    OUT_DIR,
    INVAL_DIR,
};

enum gpio_value {
    OFF = 0,
    ON  = 1,
};

bool gpio_is_init(uint32_t gpio)
{
    int32_t error = 0;
    char cmd[255] = {0};

    if (gpio == 0)
        return false;

    (void) snprintf(cmd, sizeof(cmd), "ls %s%u/value > /dev/null 2>&1", sysfs_gpio, gpio);

    error = system(cmd);
    return error ? false : true;
}

int32_t init_gpio(uint32_t gpio, uint32_t direction)
{
    int32_t error = 0;
    char cmd[255] = {0};

    if (gpio == 0 || direction >= INVAL_DIR)
        return -1;

    if (!gpio_is_init(gpio)) {
        (void) snprintf(cmd, sizeof(cmd), "echo %u > %s", gpio, sysfs_export);

        error = system(cmd);
        if (error)
            return error;
    }

    (void) snprintf(cmd, sizeof(cmd), "echo %s > %s%u/direction", direction ? "out" : "in", sysfs_gpio, gpio);

    error = system(cmd);

    return error;
}

int32_t deinit_gpio(uint32_t gpio)
{
    int32_t error = 0;
    char cmd[255] = {0};

    if (gpio == 0 || !gpio_is_init(gpio))
        return -1;

    (void) snprintf(cmd, sizeof(cmd), "echo %u > %s", gpio, sysfs_unexport);

    error = system(cmd);
    return error;
}

int32_t read_gpio(uint32_t gpio, uint32_t *value)
{
    int32_t error = 0;
    char cmd[255] = {0};
    FILE *fp;

    if (!gpio_is_init(gpio) || value == NULL)
        return -1;

    (void) snprintf(cmd, sizeof(cmd), "cat %s%u/value", sysfs_gpio, gpio);

    fp = popen(cmd, "r");
    if (fp == NULL)
        return -1;

    error = fscanf(fp, "%u", value);
    (void)pclose(fp);

    return error;
}

int32_t write_gpio(uint32_t gpio, uint32_t value)
{
    int32_t error = 0;
    char cmd[255] = {0};

    if (!gpio_is_init(gpio))
        return -1;

    (void) snprintf(cmd, sizeof(cmd), "echo %u > %s%u/value", value, sysfs_gpio, gpio);

    error = system(cmd);
    return error;
}
/////////////////////////////////////////////////

/////////////////////////////////////////////////
// spi implemented function

#define SPI_CHANNEL                0
#define SPI_MAX_SPEED              20000000
#define SPI_WR_CMD                 0x02
#define SPI_RD_CMD                 0x03

static char data_buff[1024];
static char *spi_devname = "/dev/spidev0.0";
static int fd_spi = -1;

void print_data(unsigned char *data, int len, char *msg)
{
    int i;
    int n = 0;

    return;
    if (len > 200)
    {
        PRINT_DEBUG("data len is too long, only print 100 bytes\n");
        len = 200;
    }
    //PRINT_DEBUG("%s"\n", msg);

    memset(data_buff, 0, sizeof(data_buff));
    n += sprintf(data_buff+n, "%s:", msg);
    for (i = 0; i <= len; i++)
    {
        if (i >= len)
        {
            PRINT_DEBUG("%s\n", data_buff);
            break;
        }
        if (i != 0 && i%8 == 0)
        {
            PRINT_DEBUG("%s\n", data_buff);
            memset(data_buff, 0, sizeof(data_buff));
            n = 0;
        }
        n += sprintf(data_buff+n, "0x%x ", data[i]);
    }
}

int spi_data_rw(unsigned char *writeData, int writeLen, unsigned char *readData, int readLen)
{
  struct spi_ioc_transfer spi[2] ;

    memset (spi, 0, sizeof (spi));

    if (writeLen)
    {
        spi[0].tx_buf        = (unsigned long)writeData ;
        //spi[0].rx_buf        = (unsigned long)data ;
        spi[0].len           = writeLen ;
        spi[0].delay_usecs   = 0 ;
        spi[0].speed_hz      = SPI_MAX_SPEED;
        spi[0].bits_per_word = 8 ;
    }

    if (readLen)
    {
        //spi[0].tx_buf        = (unsigned long)writeData ;
        spi[1].rx_buf        = (unsigned long)readData ;
        spi[1].len           = readLen;
        spi[1].delay_usecs   = 0 ;
        spi[1].speed_hz      = SPI_MAX_SPEED;
        spi[1].bits_per_word = 8 ;
    }

    int transfer_count = (writeLen > 0 ? 1 : 0) + (readLen > 0 ? 1 : 0);
    if (transfer_count == 0) {
        PRINT_INFO("SPI data transfer: no data to transfer\n");
        return -1;
    }

    return ioctl(fd_spi, SPI_IOC_MESSAGE(transfer_count), spi) ;
}

int spi_init(void)
{
    char spi_bits;
    int spi_speed;
    int mode;
    int ret;

    fd_spi = open(spi_devname, O_RDWR);
    if (fd_spi < 0)
    {
        PRINT_INFO("Failed to open SPI device %s, fd_spi = %d\n", spi_devname, fd_spi);
        return -1;
    }

    mode = 0;
    ret = ioctl(fd_spi, SPI_IOC_WR_MODE, &mode);
    if (ret < 0)
    {
        PRINT_INFO("Failed to set SPI mode\n");
        close(fd_spi);
        return ret;
    }

    spi_bits = 8;
    ret = ioctl(fd_spi, SPI_IOC_WR_BITS_PER_WORD, &spi_bits);
    if (ret < 0)
    {
        PRINT_INFO("Failed to set SPI bits per word\n");
        close(fd_spi);
        return ret;
    }

    spi_speed = SPI_MAX_SPEED;
    ret = ioctl(fd_spi, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    if (ret < 0)
    {
        PRINT_INFO("Failed to set SPI max speed\n");
        close(fd_spi);
        return ret;
    }
    PRINT_DEBUG("SPI mode: 0x%x\n", mode);
    PRINT_DEBUG("SPI bits per word: %d\n", spi_bits);
    PRINT_DEBUG("SPI max speed: %d Hz (%d KHz)\n", spi_speed, spi_speed/1000);
    PRINT_INFO("Successfully initialized SPI %s, fd_spi = %d\n", spi_devname, fd_spi);

    return fd_spi;
}

int spi_uninit(void)
{
    int ret;

    if (fd_spi < 0)
        return 0;

    PRINT_DEBUG("SPI uninitializing, fd_spi: %d\n", fd_spi);
    ret = close(fd_spi);
    if (0 != ret)
    {
        PRINT_INFO("Failed to close SPI, ret = %d\n", ret);
    }
    else
    {
        fd_spi = -1;
        PRINT_INFO("Successfully closed SPI, ret = %d\n", ret);
    }

    return 0;
}

int spi_write(uint8_t regAddr, const char *data, int len)
{
    int ret;
	int i;
	unsigned char writebuf[1024];

    writebuf[0] = SPI_WR_CMD;
    writebuf[1] = regAddr;

    for(i = 0 ;i < len;i++)
    {
        writebuf[i+2] = (unsigned char)data[i];
    }

    print_data(writebuf, len + 2, "spi write");

    ret = spi_data_rw(writebuf, len + 2, 0, 0);
    if (ret < 0)
    {
        PRINT_INFO("SPI write failed\n");
        return ret;
    }

    return 0;
}

int spi_read(uint8_t regAddr, unsigned char *rxData, int len)
{
    int ret = 0;
    unsigned char wrbuf[3];
    int offset = 0;
    int chunkSize;
    int maxChunkSize = 4096;  /* Linux SPI driver typically has 4KB limit */

    wrbuf[0] = SPI_RD_CMD;
    wrbuf[1] = regAddr;
    wrbuf[2] = 0;          // dummy

    print_data(wrbuf, 2, "spi read cmd");

    /* Read data in chunks for large transfers */
    while (offset < len)
    {
        chunkSize = len - offset;
        if (chunkSize > maxChunkSize)
        {
            chunkSize = maxChunkSize;
        }

        ret = spi_data_rw(wrbuf, 3, rxData + offset, chunkSize);
        if (ret < 0)
        {
            PRINT_INFO("SPI read failed (offset %d, chunk %d)\n", offset, chunkSize);
            return ret;
        }

        offset += chunkSize;
    }

    print_data(rxData, len, "spi read data");

    return 0;
}
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// i2c implemented function
static const char *i2cdev_fp = "/dev/i2c-7";

void tmf8829_shim_set_i2c_bus(const char *bus_path) {
    if (bus_path != NULL) {
        i2cdev_fp = bus_path;
        //PRINT_INFO("I2C bus path set to: %s\n", i2cdev_fp);
    } else {
        PRINT_INFO("Invalid I2C bus path provided, using default: %s\n", i2cdev_fp);
    }
}

int write_i2c_block(uint32_t slave_addr, uint8_t reg, const uint8_t *buf, uint32_t len)
{
    int32_t i2c_fd;

    if (buf == NULL || len == 0) {
        return -1;
    }

    i2c_fd = open(i2cdev_fp, O_RDWR);
    if (i2c_fd < 0) {
        perror("I2C open failed");
        return -1;
    }

    if (ioctl(i2c_fd, I2C_SLAVE, slave_addr) < 0) {
        perror("I2C set slave failed");
        close(i2c_fd);
        return -1;
    }

    /*
     * CP2112 Linux driver can only carry 61 bytes in one write request.
     * Since our write format is:
     *
     *   [register byte] + [payload bytes]
     *
     * keep payload <= 60.
     */
    const uint32_t max_payload = 60;

    uint32_t offset = 0;

    while (offset < len)
    {
        uint32_t chunk = len - offset;

        if (chunk > max_payload)
        {
            chunk = max_payload;
        }

        uint8_t *outbuf = (uint8_t *)malloc(chunk + 1);

        if (outbuf == NULL)
        {
            close(i2c_fd);
            return -1;
        }

        /*
         * For normal register writes, continue at the next register.
         * Example:
         *   first chunk:  reg 0x22, data[0..59]
         *   second chunk: reg 0x5E, data[60..119]
         */
        outbuf[0] = (uint8_t)(reg + offset);
        memcpy(outbuf + 1, buf + offset, chunk);

        if (write(i2c_fd, outbuf, chunk + 1) != (ssize_t)(chunk + 1))
        {
            printf("I2C write failed details: bus=%s slave=0x%02x reg=0x%02x offset=%u chunk=%u total_original=%u actual_reg=0x%02x\n",
                   i2cdev_fp,
                   slave_addr,
                   reg,
                   offset,
                   chunk,
                   len,
                   outbuf[0]);
            perror("I2C write failed");

            free(outbuf);
            close(i2c_fd);
            return -1;
        }

        free(outbuf);

        offset += chunk;
        usleep(1000);
    }

    close(i2c_fd);

    return 0;
}

int read_i2c_block(uint32_t slave_addr, uint8_t reg, uint8_t *buf, uint32_t len)
{
    int32_t i2c_fd;

    if (buf == NULL || len == 0) {
        return -1;
    }

    i2c_fd = open(i2cdev_fp, O_RDWR);
    if (i2c_fd < 0) {
        perror("I2C open failed");
        return -1;
    }

    const uint32_t max_read_chunk = 60;
    uint32_t offset = 0;

    while (offset < len)
    {
        uint32_t chunk = len - offset;

        if (chunk > max_read_chunk)
        {
            chunk = max_read_chunk;
        }

        uint8_t reg_to_read;

        /*
         * FIFO is special:
         * Always read from register 0xff.
         * Do NOT use reg + offset.
         */
        if (reg == 0xff)
        {
            reg_to_read = reg;
        }
        else
        {
            reg_to_read = (uint8_t)(reg + offset);
        }

        struct i2c_msg messages[2];

        messages[0].addr = slave_addr;
        messages[0].flags = 0;
        messages[0].len = 1;
        messages[0].buf = &reg_to_read;

        messages[1].addr = slave_addr;
        messages[1].flags = I2C_M_RD;
        messages[1].len = chunk;
        messages[1].buf = buf + offset;

        struct i2c_rdwr_ioctl_data ioctl_data;
        ioctl_data.msgs = messages;
        ioctl_data.nmsgs = 2;

        if (ioctl(i2c_fd, I2C_RDWR, &ioctl_data) < 0)
        {
            printf("I2C read failed details: bus=%s slave=0x%02x reg=0x%02x actual_reg=0x%02x offset=%u chunk=%u total_original=%u\n",
                   i2cdev_fp,
                   slave_addr,
                   reg,
                   reg_to_read,
                   offset,
                   chunk,
                   len);
            perror("I2C read failed");

            close(i2c_fd);
            return -1;
        }

        offset += chunk;
        usleep(1000);
    }

    close(i2c_fd);

    return 0;
}
////////////////////////////////////////////////////////////////

static int writePin(uint8_t gpio, uint8_t value)
{
    return write_gpio(gpio, value);
}

void delayInMicroseconds(uint32_t wait)
{
    usleep(wait);
}

uint32_t getSysTick(void) //Note: is only for 70 minutes
{
    struct timespec ts;

    clock_gettime(CLOCK_REALTIME, &ts);

    return (uint32_t)(ts.tv_sec * 1000000 + ts.tv_nsec / 1000);
}

uint8_t readProgramMemoryByte(uintptr_t address)
{
    uintptr_t *ptr = (void *) address;
    uint8_t byte = *ptr;
    return byte;
}

int8_t txReg(void *dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toTx, uint8_t *txData)
{
    tmf8829_chip *driver = (tmf8829_chip *)dptr;
    if (driver->bustype == BUS_I2C)
        return (int8_t)write_i2c_block(slaveAddr, regAddr, txData, toTx);
    return spi_write(regAddr, (const char *)txData, toTx);
}

int8_t rxReg(void *dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toRx, uint8_t *rxData)
{
    tmf8829_chip *driver = (tmf8829_chip *)dptr;
    if (driver->bustype == BUS_I2C)
        return (int8_t)read_i2c_block(slaveAddr, regAddr, rxData, toRx);
    return spi_read(regAddr, rxData, toRx);
}

int enablePinHigh(void *dptr)
{
    tmf8829_chip *driver = (tmf8829_chip *)dptr;

	if (init_gpio(driver->gpiod_enable,  OUT_DIR)) {
        PRINT_INFO("Error initializing CE pin\n");
        return -1;
    }
    return writePin(driver->gpiod_enable, 1);
}

int enablePinLow(void *dptr)
{
    tmf8829_chip *driver = (tmf8829_chip *)dptr;

	writePin(driver->gpiod_enable, 0);
	deinit_gpio(driver->gpiod_enable);

    return 1;
}
