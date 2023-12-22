/**
 * @file bme.h
 * @author Trương Quốc Ánh (you@domain.com)
 * @brief bme library
 * @version 0.2
 * @date 2023-12-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef __BME_H__
#define __BME_H__

#include <string.h>
#include <stdio.h>

#define TAG_BME280 "BME280"                     /** This tag used for ESP_LOGI */
//extern const char* TAG_BME280 = "BME280";

#define I2C_MASTER_ACK_T  0                        /** ACK help slave confirm that master receive data successfully  */
#define I2C_MASTER_NACK_T 1                       /** When NACK (Not acknowledge) mode, change Pin to High signal level to stop receiving data*/

/**
 * @brief Initialize BME280 I2C master
 * 
 * @param SDA_PIN: SDA pin configured by user
 * @param SCL_PIN: SCL pin configured by user
 */
void i2c_master_bme_init(int SDA_PIN, int SCL_PIN);

/**
 * @brief bme280 reader task
 * @note: Must be called in xTaskCreate function.
 * @param ignore 
 */
void bme280_reader_task (void *ignore);

#endif