/*
 * opt3001.c
 *
 *  Created on: 16 May 2021
 *      Author: Emmanuel Savage
 */

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include <drivers/opt3001.h>

/**************************************************************************************************
 * @fn          sensorOpt3001Enable
 *
 * @brief       Turn the sensor on/off
 *
 * @return      none
 **************************************************************************************************/
void sensorOpt3001Enable(I2C_Transaction *i2cTransaction, uint8_t *txBuffer, uint8_t *rxBuffer, bool enable){
    uint16_t val;

    if (enable)
    {
        val = CONFIG_ENABLE;
    }
    else
    {
        val = CONFIG_DISABLE;
    }

    txBuffer[0] = REG_CONFIGURATION;
    txBuffer[1] = (uint8_t)(val & 0x00FF);
    txBuffer[2] = (uint8_t)((val >> 8) & 0x00FF);
    i2cTransaction->slaveAddress = OPT3001_I2C_ADDRESS;
    i2cTransaction->writeBuf = txBuffer;
    i2cTransaction->writeCount = 3;
    i2cTransaction->readBuf = rxBuffer;
    i2cTransaction->readCount = 2;
}

/**************************************************************************************************
 * @fn          sensorOpt3001Convert
 *
 * @brief       Convert raw data to object and ambience temperature
 *
 * @param       rawData - raw data from sensor
 *
 * @param       convertedLux - converted value (lux)
 *
 * @return      none
 **************************************************************************************************/
void sensorOpt3001Convert(uint16_t rawData, float *convertedLux) {
    uint16_t e, m;

    m = rawData & 0x0FFF;
    e = (rawData & 0xF000) >> 12;

    *convertedLux = m * (0.01 * exp2(e));
}
