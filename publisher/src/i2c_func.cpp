#include "i2c_func.h"
#include "publisher.h"
#include <bsp/bsp.h>
#include <coresrv/hal/hal_api.h>
#include <memory>
#include <i2c/i2c.h>

#include <rtl/stdbool.h>
#include <rtl/countof.h>
#include <stdlib.h>

#define BUF_SIZE 1024
#define I2C_FREQ 100000
#define SLAVE_ADDR 0x76

//Интервал обновления калибровочных значений
#define BOSCH_CAL_UPDATE_INTERVAL 60000

#define TEMP_CAL_START_ADDR 0x88
#define PRESS_CAL_START_ADDR 0x8E
#define HUM_CAL_H1_ADDR 0xA1
#define HUM_CAL_H2_ADDR 0xE1

#define BMP280_ID 0x58
#define BME280_ID 0x60

#define BMx280_I2C_ADDR_MIN (0x76 << 1)
#define BMx280_I2C_ADDR_MAX (0x77 << 1)

#define BMx280_REG_STATUS 0xF3
#define BMx280_REG_CTRL_MEAS 0xF4
#define BMx280_REG_CONFIG 0xF5
#define BME280_REG_CTRL_HUM 0xF2
//Преддескретизация температуры
#define BMx280_TEMP_OVERSAMPLING_SKIP 0b00000000
#define BMx280_TEMP_OVERSAMPLING_1 0b00100000
#define BMx280_TEMP_OVERSAMPLING_2 0b01000000
#define BMx280_TEMP_OVERSAMPLING_4 0b01100000
#define BMx280_TEMP_OVERSAMPLING_8 0b10000000
#define BMx280_TEMP_OVERSAMPLING_16 0b10100000
//Преддескретизация давления
#define BMx280_PRESS_OVERSAMPLING_SKIP 0b00000000
#define BMx280_PRESS_OVERSAMPLING_1 0b00000100
#define BMx280_PRESS_OVERSAMPLING_2 0b00001000
#define BMx280_PRESS_OVERSAMPLING_4 0b00001100
#define BMx280_PRESS_OVERSAMPLING_8 0b00010000
#define BMx280_PRESS_OVERSAMPLING_16 0b00010100
//Преддескретизация влажности
#define BME280_HUM_OVERSAMPLING_SKIP 0b00000000
#define BME280_HUM_OVERSAMPLING_1 0b00000001
#define BME280_HUM_OVERSAMPLING_2 0b00000010
#define BME280_HUM_OVERSAMPLING_4 0b00000011
#define BME280_HUM_OVERSAMPLING_8 0b00000100
#define BME280_HUM_OVERSAMPLING_16 0b00000101u
//Режимы работы датчика
#define BMx280_MODE_SLEEP 0b00000000 //Наелся и спит
#define BMx280_MODE_FORCED 0b00000001 //Обновляет значения 1 раз, после чего уходит в сон
#define BMx280_MODE_NORMAL 0b00000011 //Регулярно обновляет значения
//Период обновления в нормальном режиме
#define BMx280_STANDBY_TIME_0_5 0b00000000
#define BMx280_STANDBY_TIME_62_5 0b00100000
#define BMx280_STANDBY_TIME_125 0b01000000
#define BMx280_STANDBY_TIME_250 0b01100000
#define BMx280_STANDBY_TIME_500 0b10000000
#define BMx280_STANDBY_TIME_1000 0b10100000
#define BMx280_STANDBY_TIME_2000 0b11000000
#define BMx280_STANDBY_TIME_4000 0b11100000
//Коэффициент фильтрации значений
#define BMx280_FILTER_COEFF_1 0b00000000
#define BMx280_FILTER_COEFF_2 0b00000100
#define BMx280_FILTER_COEFF_4 0b00001000
#define BMx280_FILTER_COEFF_8 0b00001100
#define BMx280_FILTER_COEFF_16 0b00010000
//Разрешить работу по SPI
#define BMx280_SPI_3W_ENABLE 0b00000001
#define BMx280_SPI_3W_DISABLE 0b00000000


int SetBspConfig(const char *i2cChannelName, const char *i2cPmuxConfig)
{
    Retcode rc                      = rcFail;
    char    buf[BUF_SIZE]           = {0};
    char    boardName[BSP_NAME_MAX] = {0};

    rc = BspInit(NULL);

    if (rc != rcOk)
    {
        fprintf(stderr, "Failed to initialize BSP\n");
        return EXIT_FAILURE;
    }

    rc = BspEnableModule(i2cChannelName);
    if (rc != rcOk)
    {
        fprintf(stderr, "Failed to enable %s module\n", i2cChannelName);
        return EXIT_FAILURE;
    }

    if (KnHalGetEnv("board", boardName, sizeof(boardName)) != rcOk)
    {
        fprintf(stderr, "KnHalGetEnv() failed\n");
        return EXIT_FAILURE;
    }

    int ret = snprintf(buf, BUF_SIZE, "%s.%s", boardName, i2cPmuxConfig);
    if (ret < 0)
    {
        fprintf(stderr, "BSP config name generation failed\n");
        return EXIT_FAILURE;
    }

    rc = BspSetConfig(i2cChannelName, buf);
    if (rc != rcOk)
    {
        fprintf(stderr, "Failed to set pmux configuration for %s channel\n",
                i2cChannelName);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}


I2cHandle InitI2cChannel(const char *i2cChannelName)
{
    I2cHandle handle        = NULL;
    char      buf[BUF_SIZE] = {0};
    I2cError  rc            = I2C_EFAIL;

    rc = I2cInit();

    if (I2C_FAIL(rc))
    {
        fprintf(stderr, "Failed to initialize I2C\n");
        return handle;
    }

    rc = I2cEnumChannels(0, sizeof(buf), buf);
    if (I2C_FAIL(rc))
    {
        fprintf(stderr, "Failed to enumerate I2C channels\n");
        return handle;
    }

    rc = I2cOpenChannel(i2cChannelName, &handle);
    if (I2C_FAIL(rc))
    {
        fprintf(stderr, "Failed to open the %s channel\n", i2cChannelName);
        return handle;
    }

    return handle;
}
