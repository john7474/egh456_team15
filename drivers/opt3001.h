/*
 * opt3001.h
 *
 *  Created on: 16 May 2021
 *      Author: Emmanuel Savage
 */

#ifndef OPT3001_H
#define OPT3001_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <ti/drivers/I2C.h>
#include <drivers/opt3001.h>
#include <xdc/runtime/System.h>
#include <Board.h>

/*********************************************************************
 * CONSTANTS
 */
/* Register addresses */
#define REG_RESULT                      0x00
#define REG_CONFIGURATION               0x01
#define REG_LOW_LIMIT                   0x02
#define REG_HIGH_LIMIT                  0x03

#define REG_MANUFACTURER_ID             0x7E
#define REG_DEVICE_ID                   0x7F

/* Register values */
#define MANUFACTURER_ID                 0x5449  // TI
#define DEVICE_ID                       0x3001  // Opt 3001
#define CONFIG_RESET                    0xC810
#define CONFIG_TEST                     0xCC10
#define CONFIG_ENABLE                   0x10C4 //0x10C4 // 0xC410   - 100 ms, continuous
#define CONFIG_DISABLE                  0x10C0 // 0xC010   - 100 ms, shutdown

#define LOW_LUX                         0xFF0F

/* Bit values */
#define DATA_RDY_BIT                    0x0080  // Data ready
#define LOW_FLAG_BIT                    0x0020  // Low flag triggered

/* Register length */
#define REGISTER_LENGTH                 2

/* Sensor data size */
#define DATA_LENGTH                     2

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * FUNCTIONS
 */
extern void sensorOpt3001Enable(I2C_Transaction *i2cTransaction, uint8_t *txBuffer, uint8_t *rxBuffer, bool enable);
extern void sensorOpt3001Convert(uint16_t rawData, float *convertedLux);

#ifdef __cplusplus
}
#endif

#endif /* OPT3001_H */
