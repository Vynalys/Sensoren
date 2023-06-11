/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>


#define I2C_NODE DT_NODELABEL(i2c1)

#define SCD41_I2C_ADDR      0x62
#define SVM_ADDR            0x6a

#define REL_HUMIDITY_SCALE_FACTOR   100
#define TEMPERATURE_SCALE_FACTOR    200
#define VOC_SCALE_FACTOR          10
#define NOX_SCALE_FACTOR          10

static const struct device* i2c_dev = DEVICE_DT_GET(I2C_NODE);


// --- SCD41 FUNCTIONS ---
// check if data is ready
bool scd_data_is_ready(){
    uint8_t data[3] = {0xe4, 0xb8};
    
    int ret = i2c_write(i2c_dev, data, 2, SCD41_I2C_ADDR);      // Request data status
    if(ret != 0){
        printk("SCD: Write Data Ready failed. (err %i)\n", ret);
    }

    k_msleep(2);

    ret = i2c_read(i2c_dev, data, 3, SCD41_I2C_ADDR);       // Recive data status
    if(ret != 0){
        printk("SCD: Get Data Ready failed. (err %i)\n", ret);
    }

    //printk("data[0]: %i, data[1]: %i\n", data[0], data[1]);

    if(data[1] == 0x00 && data[0] == 0x80){                // check if data is not ready
        //printk("Data not ready\n");
        return false;
    }
    
    return true;
}

// Sends stop and start command to sensor
uint8_t scd_start_measurment(){
    int ret;
    uint8_t cmd[2] = {0x3f, 0x86};  // Stop continuous measurement command

    k_msleep(5000);

    ret = i2c_write(i2c_dev, cmd, sizeof(cmd), SCD41_I2C_ADDR);

    if (ret != 0) {
        printk("SCD: Stop Pereodic Measurement failed. (err %d)\n", ret);
        //return ret;
    }

    k_msleep(1000); // Waitingtime for Stop pereodic measurement

    uint8_t cmd_2[2] = {0x21, 0xb1};  // Start continuous measurement command

    ret = i2c_write(i2c_dev, cmd_2, sizeof(cmd_2), SCD41_I2C_ADDR);
    if (ret != 0) {
        printk("SCD: Error sending start command to sensor Error Code: %d\n", ret);
    }

    return ret;
}

// Reads data from sensor
int scd_read_print_data(){
    int ret; 
    uint8_t data[18];
    uint16_t temperature, co2, humidity;

    data[0] = 0xec, data[1] = 0x05;                     // read data command
    ret = i2c_write(i2c_dev, data, 2, SCD41_I2C_ADDR);
    if (ret != 0) {
        printk("SCD: Error writing read command to sensor. (err %i)\n", ret);
    }

    k_msleep(5000);

    ret = i2c_read(i2c_dev, data, sizeof(data), SCD41_I2C_ADDR);    // read data
    if (ret != 0) {
        printk("SCD: Error reading data from sensor. (err %i)\n", ret);
    }

        
    // Extract CO2, temperature, and humidity from the received data
    co2 = (float)((uint16_t)data[0] << 8 | data[1]);
    temperature = -45 + 175 * (float)((uint16_t)data[3] << 8 | data[4]) / 65536;
    humidity = 100 * (float)((uint16_t)data[6] << 8 | data[7]) / 65536;

    printk("\nSCD:\nCO2: %u ppm\tTemperature: %u °C\tHumidity: %u %%\n\n",
    co2, temperature, humidity);  

    return ret;  
}


// --- SVM FUNCTIONS ---
// Stops and starts sensor again
uint8_t svm_start_measurment(){
    int ret;
    uint8_t cmd[2] = {0x01, 0x04};  // Stop continuous measurement command
    ret = i2c_write(i2c_dev, cmd, sizeof(cmd), SVM_ADDR);
    if (ret != 0) {
        printk("SVM: Stop Pereodic Measurement failed. (err %d)\n", ret);
        //return ret;
    }

    k_msleep(500); // Waitingtime for Stop pereodic measurement

    uint8_t cmd_2[2] = {0x00, 0x10};  // Start continuous measurement command

    ret = i2c_write(i2c_dev, cmd_2, sizeof(cmd_2), SVM_ADDR);
    if (ret != 0) {
        printk("SVM: Error sending command to sensor Error Code: %d\n", ret);
        //return ret;
    }

    return ret;
}

// read data from sensor
int svm_read_print_data(){
    int ret; 
    uint8_t data[18];
    uint16_t temperature, humidity, voc, nox;

    data[0] = 0x04, data[1] = 0x05;                     // read data command
    ret = i2c_write(i2c_dev, data, 2, SVM_ADDR);
    if (ret != 0) {
        printk("SVM: Error writing read command to sensor. (err %i)\n", ret);
        //return ret;
    }

    k_msleep(45000);

    ret = i2c_read(i2c_dev, data, sizeof(data), SVM_ADDR);    // read data
    if (ret != 0) {
        printk("SVM: Error reading data from sensor. (err %i)\n", ret);
        //return ret;
    }
        
    // Extract CO2, temperature, and humidity from the received data
    humidity = ((uint16_t)data[0] << 8 | data[1]) / REL_HUMIDITY_SCALE_FACTOR;
    temperature = ((uint16_t)data[3] << 8 | data[4]) / TEMPERATURE_SCALE_FACTOR;
    voc = ((uint16_t)data[6] << 8 | data[7]) / VOC_SCALE_FACTOR;
    nox = ((uint16_t)data[9] << 8 | data[10]) / NOX_SCALE_FACTOR;
    
    

    printk("\nSVM:\nVOC: %u\tNOx: %u\tTemperature: %u °C\tHumidity: %u %%\n\n",
    voc, nox, temperature, humidity);  

    return ret;  
}



void main(void)
{
    if (!i2c_dev) {
        printk("I2C device not found\n");
        return;
    }



    scd_start_measurment();
    svm_start_measurment();
    k_msleep(1000);

    while (1) {
        if(scd_data_is_ready()){
            scd_read_print_data();
        }

        svm_read_print_data();
        k_msleep(5000);
    }

}