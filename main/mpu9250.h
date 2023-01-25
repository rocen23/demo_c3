#pragma once

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_MASTER_SCL_IO           6 //CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           5 //CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU9250_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU9250 sensor */
#define MPU9250_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */
#define MPU9250_ACCEL_XOUT_H                0x3B
#define MPU9250_ACCEL_XOUT_L                0x3C
#define MPU9250_ACCEL_YOUT_H                0x3D
#define MPU9250_ACCEL_YOUT_L                0x3E
#define MPU9250_ACCEL_ZOUT_H                0x3F
#define MPU9250_ACCEL_ZOUT_L                0x40

#define MPU9250_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU9250_RESET_BIT                   7

esp_err_t i2c_master_init(void);
esp_err_t mpu9250_register_write_byte(uint8_t reg_addr, uint8_t data);
esp_err_t mpu9250_register_read(uint8_t reg_addr, uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif