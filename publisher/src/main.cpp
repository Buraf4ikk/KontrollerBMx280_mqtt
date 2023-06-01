#include <cstring>
#include <exception>
#include <iostream>
#include <memory>

#include <rtl/stdbool.h>
#include <rtl/countof.h>

#include <bsp/bsp.h>

#include <kos_net.h>

#include "general.h"
#include "publisher.h"
#include "i2c_func.h"

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



#define I2C_CHANNEL_NAME       "i2c0"
#define I2C_PMUX_CONFIG_SUFFIX "pins_0_1"

namespace consts {
constexpr int PublicationIntervalInSec = 5;
} // namespace consts

typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
} BMx280_temp_cal;

typedef struct {
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} BMx280_press_cal;

typedef struct {
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
} BME280_hum_cal;

int32_t BMx280_compensate_temperature(BMx280_temp_cal *cal, int32_t adc_T) {
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)cal->dig_T1 << 1))) *
            ((int32_t)cal->dig_T2)) >>
           11;
    var2 = (((((adc_T >> 4) - ((int32_t)cal->dig_T1)) *
              ((adc_T >> 4) - ((int32_t)cal->dig_T1))) >>
             12) *
            ((int32_t)cal->dig_T3)) >>
           14;
    return var1 + var2;
}



float BMx280_compensate_pressure(BMx280_press_cal *cal, int32_t adc_P, int32_t t_fine) {
    double var1, var2, p;
    var1 = ((double)t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)cal->dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double)cal->dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double)cal->dig_P4) * 65536.0);
    var1 =
        (((double)cal->dig_P3) * var1 * var1 / 524288.0 + ((double)cal->dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)cal->dig_P1);
    if (var1 == 0.0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576.0 - (double)adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double)cal->dig_P9) * p * p / 2147483648.0;
    var2 = p * ((double)cal->dig_P8) / 32768.0;
    p = p + (var1 + var2 + ((double)cal->dig_P7)) / 16.0;
    return p;
}



float BMx280_compensate_humidity(BME280_hum_cal *cal, int32_t adc_H, int32_t t_fine) {
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));

    v_x1_u32r =
        (((((adc_H << 14) - (((int32_t)cal->dig_H4) << 20) -
            (((int32_t)cal->dig_H5) * v_x1_u32r)) +
           ((int32_t)16384)) >>
          15) *
         (((((((v_x1_u32r * ((int32_t)cal->dig_H6)) >> 10) *
              (((v_x1_u32r * ((int32_t)cal->dig_H3)) >> 11) +
               ((int32_t)32768))) >>
             10) +
            ((int32_t)2097152)) *
               ((int32_t)cal->dig_H2) +
           8192) >>
          14));

    v_x1_u32r =
        (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                       ((int32_t)cal->dig_H1)) >>
                      4));

    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return ((uint32_t)(v_x1_u32r >> 12)) / 1024.0f;
}




const char* GetBrokerAddress()
{
    const char* brokerAddress = getenv("MQTT_BROKER_ADDRESS");
    if (brokerAddress)
    {
        return brokerAddress;
    }
    throw std::runtime_error{"Failed to get MQTT broker address."};
}

int GetBrokerPort()
{
    if (auto brokerPortEnvVariable = getenv("MQTT_BROKER_PORT"))
    {
        try
        {
            return std::stoi(brokerPortEnvVariable);
        }
        catch (const std::logic_error&)
        {}
    }
    throw std::runtime_error{"Failed to get MQTT broker port."};
}

// Functions to work with BMP280 sensor with i2c protocol

int main(void) try
{
    if (!wait_for_network())
    {
        throw std::runtime_error{"Error: Wait for network failed!"};
    }

    mosqpp::lib_init();
    auto pub = std::make_unique<Publisher>("publisher", GetBrokerAddress(), GetBrokerPort());

    I2cHandle handle = NULL;
    uint8_t regBuf[3];
    // Initialize i2c
    pub->send_log_message("Initializing i2c...\n");

    if (SetBspConfig(I2C_CHANNEL_NAME, I2C_PMUX_CONFIG_SUFFIX) != EXIT_SUCCESS)
    {
        pub->send_log_message("[Error]: Can`t set bsp config!\n");
        return EXIT_FAILURE;
    } else {
        pub->send_log_message("Bsp config set successfully!\n");
    }

    char      buf[BUF_SIZE] = {0};
    I2cError  rc            = I2C_EFAIL;

    rc = I2cInit();

    if (I2C_FAIL(rc))
    {
        pub->send_log_message("[Error]: Failed to initialize I2C\n");
        pub->send_log_message("Error code: " + std::to_string(rc) + "\n");
    }

    rc = I2cEnumChannels(0, sizeof(buf), buf);
    if (I2C_FAIL(rc))
    {
        pub->send_log_message("[Error]: Failed to enumerate I2C channels\n");
        pub->send_log_message("Error code: " + std::to_string(rc) + "\n");
    }

    rc = I2cOpenChannel(I2C_CHANNEL_NAME, &handle);
    if (I2C_FAIL(rc))
    {
        pub->send_log_message("[Error]: Failed to open I2C channel\n");
        pub->send_log_message("Error code: " + std::to_string(rc) + "\n");
    }

    // INIT THE SENSOR
    // Reboot the sensor
    I2cMsg i2cMsgReset[1];
    uint8_t cmdBuf[2] = {0xE0, 0xB6};

    i2cMsgReset[0].addr = SLAVE_ADDR;
    i2cMsgReset[0].flags = 0;
    i2cMsgReset[0].len = rtl_countof(cmdBuf);
    i2cMsgReset[0].buf = cmdBuf;

    pub->send_log_message("Rebooting the sensor\n");

    rc = I2cXfer(handle, I2C_FREQ, i2cMsgReset, 1);

    if (I2C_FAIL(rc))
    {
        pub->send_log_message("Failed to reboot the sensor\n");
        return rc;
    }

    // Read the chip ID
    I2cMsg i2cMsgId[2];
    uint8_t cmdBufId[1] = {0xD0};
    uint8_t chipId[1] = {0};

    i2cMsgId[0].addr = SLAVE_ADDR;
    i2cMsgId[0].flags = 0;
    i2cMsgId[0].len = rtl_countof(cmdBufId);
    i2cMsgId[0].buf = cmdBufId;

    i2cMsgId[1].addr = SLAVE_ADDR;
    i2cMsgId[1].flags = I2C_FLAG_RD;
    i2cMsgId[1].len = 1;
    i2cMsgId[1].buf = cmdBufId;

    pub->send_log_message("Reading the chip ID\n");

    rc = I2cXfer(handle, I2C_FREQ, i2cMsgId, 2);

    if (I2C_FAIL(rc))
    {
        pub->send_log_message("Failed to read the chip ID\n");
        // Send the error code
        pub->send_log_message("Error code: " + std::to_string(rc) + "\n");
        return rc;
    }

    uint8_t sensor_id = cmdBufId[0];

    pub->send_log_message("Chip ID: " + std::to_string(sensor_id) + "\n");

    // If not BMP280 or BME280, return error
    if (sensor_id != BMP280_ID && sensor_id != BME280_ID)
    {
        pub->send_log_message("Error: Wrong chip ID\n");
        return rc;
    }

    // Set the sensor's modes.
    if (sensor_id == BME280_ID) {
        // Set the humidity settings
        I2cMsg i2cMsgHumSettings[1];
        uint8_t cmdBufHumSettings[2] = {BME280_REG_CTRL_HUM, 0x01};

        i2cMsgHumSettings[0].addr = SLAVE_ADDR;
        i2cMsgHumSettings[0].flags = 0;
        i2cMsgHumSettings[0].len = rtl_countof(cmdBufHumSettings);
        i2cMsgHumSettings[0].buf = cmdBufHumSettings;

        pub->send_log_message("Setting the humidity settings\n");

        rc = I2cXfer(handle, I2C_FREQ, i2cMsgHumSettings, 1);

        if (I2C_FAIL(rc))
        {
            pub->send_log_message("Failed to set the humidity settings\n");
            // Send the error code
            pub->send_log_message("Error code: " + std::to_string(rc) + "\n");
            return rc;
        }
    }

    // Set the oversampling settings
    I2cMsg i2cMsgOversampling[1];
    uint8_t cmdBufOversampling[2] = {BMx280_REG_CTRL_MEAS,
    BMx280_TEMP_OVERSAMPLING_2 | BMx280_PRESS_OVERSAMPLING_4 | BMx280_MODE_NORMAL};

    i2cMsgOversampling[0].addr = SLAVE_ADDR;
    i2cMsgOversampling[0].flags = 0;
    i2cMsgOversampling[0].len = rtl_countof(cmdBufOversampling);
    i2cMsgOversampling[0].buf = cmdBufOversampling;

    pub->send_log_message("Setting the oversampling settings\n");

    rc = I2cXfer(handle, I2C_FREQ, i2cMsgOversampling, 1);

    if (I2C_FAIL(rc))
    {
        pub->send_log_message("Failed to set the oversampling settings\n");
        // Send the error code
        pub->send_log_message("Error code: " + std::to_string(rc) + "\n");
        return rc;
    }

    // Set the config settings
    I2cMsg i2cMsgConfig[1];
    uint8_t cmdBufConfig[2] = {BMx280_REG_CONFIG,
                                 BMx280_STANDBY_TIME_500 | BMx280_FILTER_COEFF_16 | BMx280_SPI_3W_DISABLE};

    i2cMsgConfig[0].addr = SLAVE_ADDR;
    i2cMsgConfig[0].flags = 0;
    i2cMsgConfig[0].len = rtl_countof(cmdBufConfig);
    i2cMsgConfig[0].buf = cmdBufConfig;

    pub->send_log_message("Setting the config settings\n");

    rc = I2cXfer(handle, I2C_FREQ, i2cMsgConfig, 1);

    if (I2C_FAIL(rc))
    {
        pub->send_log_message("Failed to set the config settings\n");
        // Send the error code
        pub->send_log_message("Error code: " + std::to_string(rc) + "\n");
        return rc;
    }

    // Read the calibration data
    I2cMsg i2cMsgCalibTemp[2];
    uint8_t cmdBufCalibTemp[1] = {TEMP_CAL_START_ADDR};
    uint8_t dataBufCalibTemp[6];

    i2cMsgCalibTemp[0].addr = SLAVE_ADDR;
    i2cMsgCalibTemp[0].flags = 0;
    i2cMsgCalibTemp[0].len = rtl_countof(cmdBufCalibTemp);
    i2cMsgCalibTemp[0].buf = cmdBufCalibTemp;

    i2cMsgCalibTemp[1].addr = SLAVE_ADDR;
    i2cMsgCalibTemp[1].flags = I2C_FLAG_RD;
    i2cMsgCalibTemp[1].len = rtl_countof(dataBufCalibTemp);
    i2cMsgCalibTemp[1].buf = dataBufCalibTemp;

    pub->send_log_message("Reading the temperature calibration data\n");

    rc = I2cXfer(handle, I2C_FREQ, i2cMsgCalibTemp, 2);

    if (I2C_FAIL(rc))
    {
        pub->send_log_message("Failed to read the temperature calibration data\n");
        // Send the error code
        pub->send_log_message("Error code: " + std::to_string(rc) + "\n");
        return rc;
    }

    // Parse the temperature calibration data
    BMx280_temp_cal tempCal;
    tempCal.dig_T1 = (dataBufCalibTemp[1] << 8) | dataBufCalibTemp[0];
    tempCal.dig_T2 = (dataBufCalibTemp[3] << 8) | dataBufCalibTemp[2];
    tempCal.dig_T3 = (dataBufCalibTemp[5] << 8) | dataBufCalibTemp[4];

    // Read the pressure calibration data. It is 18 bytes long.

    I2cMsg i2cMsgCalibPress[2];
    uint8_t cmdBufCalibPress[1] = {PRESS_CAL_START_ADDR};
    uint8_t dataBufCalibPress[18];

    i2cMsgCalibPress[0].addr = SLAVE_ADDR;
    i2cMsgCalibPress[0].flags = 0;
    i2cMsgCalibPress[0].len = rtl_countof(cmdBufCalibPress);
    i2cMsgCalibPress[0].buf = cmdBufCalibPress;

    i2cMsgCalibPress[1].addr = SLAVE_ADDR;
    i2cMsgCalibPress[1].flags = I2C_FLAG_RD;
    i2cMsgCalibPress[1].len = rtl_countof(dataBufCalibPress);
    i2cMsgCalibPress[1].buf = dataBufCalibPress;

    pub->send_log_message("Reading the pressure calibration data\n");

    rc = I2cXfer(handle, I2C_FREQ, i2cMsgCalibPress, 2);

    if (I2C_FAIL(rc))
    {
        pub->send_log_message("Failed to read the pressure calibration data\n");
        // Send the error code
        pub->send_log_message("Error code: " + std::to_string(rc) + "\n");
        return rc;
    }

    // Parse the pressure calibration data
    BMx280_press_cal pressCal;

    pressCal.dig_P1 = (dataBufCalibPress[1] << 8) | dataBufCalibPress[0];
    pressCal.dig_P2 = (dataBufCalibPress[3] << 8) | dataBufCalibPress[2];
    pressCal.dig_P3 = (dataBufCalibPress[5] << 8) | dataBufCalibPress[4];
    pressCal.dig_P4 = (dataBufCalibPress[7] << 8) | dataBufCalibPress[6];
    pressCal.dig_P5 = (dataBufCalibPress[9] << 8) | dataBufCalibPress[8];
    pressCal.dig_P6 = (dataBufCalibPress[11] << 8) | dataBufCalibPress[10];
    pressCal.dig_P7 = (dataBufCalibPress[13] << 8) | dataBufCalibPress[12];
    pressCal.dig_P8 = (dataBufCalibPress[15] << 8) | dataBufCalibPress[14];
    pressCal.dig_P9 = (dataBufCalibPress[17] << 8) | dataBufCalibPress[16];

    BME280_hum_cal humCal;
    // Read the humidity calibration data.
    if (sensor_id == BME280_ID) {
        // The H1 is on register HUM_CAL_H1_ADDR. All other data is on register HUM_CAL_H2_ADDR.

        I2cMsg i2cMsgCalibHum1[2];
        uint8_t cmdBufCalibHum1[1] = {HUM_CAL_H1_ADDR};
        uint8_t dataBufCalibHum1[1];

        i2cMsgCalibHum1[0].addr = SLAVE_ADDR;
        i2cMsgCalibHum1[0].flags = 0;
        i2cMsgCalibHum1[0].len = rtl_countof(cmdBufCalibHum1);
        i2cMsgCalibHum1[0].buf = cmdBufCalibHum1;

        i2cMsgCalibHum1[1].addr = SLAVE_ADDR;
        i2cMsgCalibHum1[1].flags = I2C_FLAG_RD;
        i2cMsgCalibHum1[1].len = rtl_countof(dataBufCalibHum1);
        i2cMsgCalibHum1[1].buf = dataBufCalibHum1;

        pub->send_log_message("Reading the humidity calibration data\n");

        rc = I2cXfer(handle, I2C_FREQ, i2cMsgCalibHum1, 2);

        if (I2C_FAIL(rc))
        {
            pub->send_log_message("Failed to read the humidity calibration data\n");
            // Send the error code
            pub->send_log_message("Error code: " + std::to_string(rc) + "\n");
            return rc;
        }

        // Parse the humidity calibration data
        humCal.dig_H1 = dataBufCalibHum1[0];

        I2cMsg i2cMsgCalibHum2[2];
        uint8_t cmdBufCalibHum2[1] = {HUM_CAL_H2_ADDR};
        uint8_t dataBufCalibHum2[7];

        i2cMsgCalibHum2[0].addr = SLAVE_ADDR;
        i2cMsgCalibHum2[0].flags = 0;
        i2cMsgCalibHum2[0].len = rtl_countof(cmdBufCalibHum2);
        i2cMsgCalibHum2[0].buf = cmdBufCalibHum2;

        i2cMsgCalibHum2[1].addr = SLAVE_ADDR;
        i2cMsgCalibHum2[1].flags = I2C_FLAG_RD;
        i2cMsgCalibHum2[1].len = rtl_countof(dataBufCalibHum2);
        i2cMsgCalibHum2[1].buf = dataBufCalibHum2;

        rc = I2cXfer(handle, I2C_FREQ, i2cMsgCalibHum2, 2);

        if (I2C_FAIL(rc))
        {
            pub->send_log_message("Failed to read the humidity calibration data\n");
            // Send the error code
            pub->send_log_message("Error code: " + std::to_string(rc) + "\n");
            return rc;
        }

        humCal.dig_H2 = (dataBufCalibHum2[1] << 8) | dataBufCalibHum2[0];
        humCal.dig_H3 = dataBufCalibHum2[2];
        humCal.dig_H4 = (dataBufCalibHum2[3] << 4) | (dataBufCalibHum2[4] & 0x0F);
        humCal.dig_H5 = (dataBufCalibHum2[5] << 4) | ((dataBufCalibHum2[4] >> 4) & 0x0F);
        humCal.dig_H6 = dataBufCalibHum2[6];
    }


    // Send the calibration data
    pub->send_log_calib("T1 " + std::to_string(tempCal.dig_T1) + "\n");
    pub->send_log_calib("T2 " + std::to_string(tempCal.dig_T2) + "\n");
    pub->send_log_calib("T3 " + std::to_string(tempCal.dig_T3) + "\n");

    pub->send_log_calib("P1 " + std::to_string(pressCal.dig_P1) + "\n");
    pub->send_log_calib("P2 " + std::to_string(pressCal.dig_P2) + "\n");
    pub->send_log_calib("P3 " + std::to_string(pressCal.dig_P3) + "\n");
    pub->send_log_calib("P4 " + std::to_string(pressCal.dig_P4) + "\n");
    pub->send_log_calib("P5 " + std::to_string(pressCal.dig_P5) + "\n");
    pub->send_log_calib("P6 " + std::to_string(pressCal.dig_P6) + "\n");
    pub->send_log_calib("P7 " + std::to_string(pressCal.dig_P7) + "\n");
    pub->send_log_calib("P8 " + std::to_string(pressCal.dig_P8) + "\n");
    pub->send_log_calib("P9 " + std::to_string(pressCal.dig_P9) + "\n");


    while (true)
    {
        pub->send_log_message("Waiting for 2 seconds...\n");
        sleep(2);
        // Read the temperature
        pub->send_log_message("Reading the temperature\n");
        I2cMsg i2cMsgTemp[2];
        uint8_t cmdBufTemp[1] = {0xFA};
        uint8_t dataBufTemp[3];

        i2cMsgTemp[0].addr = SLAVE_ADDR;
        i2cMsgTemp[0].flags = 0;
        i2cMsgTemp[0].len = rtl_countof(cmdBufTemp);
        i2cMsgTemp[0].buf = cmdBufTemp;

        i2cMsgTemp[1].addr = SLAVE_ADDR;
        i2cMsgTemp[1].flags = I2C_FLAG_RD;
        i2cMsgTemp[1].len = rtl_countof(dataBufTemp);
        i2cMsgTemp[1].buf = dataBufTemp;

        rc = I2cXfer(handle, I2C_FREQ, i2cMsgTemp, 2);

        if (I2C_FAIL(rc))
        {
            pub->send_log_message("Failed to read the temperature\n");
            // Send the error code
            pub->send_log_message("Error code: " + std::to_string(rc) + "\n");
            return rc;
        }

        uint32_t adc_T = ((int32_t)dataBufTemp[0] << 12) | ((int32_t)dataBufTemp[1] << 4) | ((int32_t)dataBufTemp[2] >> 4);

        // Read the pressure
        pub->send_log_message("Reading the pressure\n");
        I2cMsg i2cMsgPress[2];
        uint8_t cmdBufPress[1] = {0xF7};
        uint8_t dataBufPress[3];

        i2cMsgPress[0].addr = SLAVE_ADDR;
        i2cMsgPress[0].flags = 0;
        i2cMsgPress[0].len = rtl_countof(cmdBufPress);
        i2cMsgPress[0].buf = cmdBufPress;

        i2cMsgPress[1].addr = SLAVE_ADDR;
        i2cMsgPress[1].flags = I2C_FLAG_RD;
        i2cMsgPress[1].len = rtl_countof(dataBufPress);
        i2cMsgPress[1].buf = dataBufPress;

        rc = I2cXfer(handle, I2C_FREQ, i2cMsgPress, 2);

        if (I2C_FAIL(rc))
        {
            pub->send_log_message("Failed to read the pressure\n");
            // Send the error code
            pub->send_log_message("Error code: " + std::to_string(rc) + "\n");
            return rc;
        }

        uint32_t adc_P = ((int32_t)dataBufPress[0] << 12) | ((int32_t)dataBufPress[1] << 4) | ((int32_t)dataBufPress[2] >> 4);

        // Read the humidity
        pub->send_log_message("Reading the humidity\n");
        I2cMsg i2cMsgHum[2];
        uint8_t cmdBufHum[1] = {0xFD};
        uint8_t dataBufHum[2];

        i2cMsgHum[0].addr = SLAVE_ADDR;
        i2cMsgHum[0].flags = 0;
        i2cMsgHum[0].len = rtl_countof(cmdBufHum);
        i2cMsgHum[0].buf = cmdBufHum;

        i2cMsgHum[1].addr = SLAVE_ADDR;
        i2cMsgHum[1].flags = I2C_FLAG_RD;
        i2cMsgHum[1].len = rtl_countof(dataBufHum);
        i2cMsgHum[1].buf = dataBufHum;

        uint32_t adc_H;

        if (sensor_id == BME280_ID) {
            rc = I2cXfer(handle, I2C_FREQ, i2cMsgHum, 2);

            if (I2C_FAIL(rc))
            {
                pub->send_log_message("Failed to read the humidity\n");
                // Send the error code
                pub->send_log_message("Error code: " + std::to_string(rc) + "\n");
                return rc;
            }

            uint32_t adc_H = ((int32_t)dataBufHum[0] << 8) | ((int32_t)dataBufHum[1]);

        }
        pub->send_log_message("The raw temperature is: " + std::to_string(adc_T) + "\n");
        pub->send_log_message("The raw pressure is: " + std::to_string(adc_P) + "\n");

        pub->send_log_message("Compensating the temperature, pressure and humidity\n");
        int32_t t_fine = BMx280_compensate_temperature(&tempCal, adc_T);
        pub->send_log_message("The fine temperature is: " + std::to_string(t_fine) + "\n");
        float temperature = ((t_fine * 5 + 128) >> 8) / 100.0f;
        pub->send_log_message("The temperature is: " + std::to_string(temperature) + "\n");
        float pressure = BMx280_compensate_pressure(&pressCal, adc_P, t_fine);

        pub->send_log_message("The pressure is: " + std::to_string(pressure) + "\n");
        float humidity;
        if (sensor_id == BME280_ID) {
            humidity = BMx280_compensate_humidity(&humCal, adc_H, t_fine);
        }

        // Send temperature and humidity

        // Send data to broker
        pub->send_temperature(temperature);
        if (sensor_id == BME280_ID) {
            pub->send_humidity(humidity);
        }
        pub->send_pressure(pressure);
    }

    mosqpp::lib_cleanup();
    return EXIT_SUCCESS;
}
catch (const std::exception& exc)
{
    std::cerr << app::AppTag << exc.what() << std::endl;
    return EXIT_FAILURE;
}
