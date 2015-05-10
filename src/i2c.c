#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "bsp.h"
#include "segmentlcd.h"
#include "bsp_trace.h"
#include "em_i2c.h"
#include "em_gpio.h"
#include "em_rtc.h"
#include "em_emu.h"
//#include "em_math.h"

/* Defines*/
#define CORE_FREQUENCY              14000000
#define RTC_MIN_TIMEOUT                32000
#define I2C_ADDRESS                     0x68
#define I2C_RXBUFFER_SIZE                 14

/***************************************************************************//**
 * Global variables
 ******************************************************************************/
uint8_t i2c_txBuffer[2] = {0x6B, 0x00};
uint8_t i2c_txBufferSize = sizeof(i2c_txBuffer);
uint8_t i2c_rxBuffer[I2C_RXBUFFER_SIZE];
uint8_t i2c_rxBufferIndex;
uint16_t G[3];

uint8_t data=0x3B;
uint8_t tam=sizeof(data);
int xx=0;

void i2cInit(void)
{
  setupOscillators();
  setupI2C();
}

void setupI2C(void)
{
    // Using default settings
    I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

    /* Using PD6 (SDA) and PD7 (SCL) */
    GPIO_PinModeSet(gpioPortD, 7, gpioModeWiredAndPullUpFilter, 1);
    GPIO_PinModeSet(gpioPortD, 6, gpioModeWiredAndPullUpFilter, 1);

    /* Enable pins at location 1 */
    I2C0->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN | (1 << _I2C_ROUTE_LOCATION_SHIFT);

    /* Initializing the I2C */
    I2C_Init(I2C0, &i2cInit);

    i2c_rxBufferIndex = 0;

    I2C0->CTRL |= I2C_CTRL_AUTOACK | I2C_CTRL_AUTOSN;
}

void setupOscillators(void)
{
    /* Enabling clock to the I2C, GPIO, LE */
    CMU_ClockEnable(cmuClock_I2C0, true);
    CMU_ClockEnable(cmuClock_GPIO, true);
    CMU_ClockEnable(cmuClock_CORELE, true);

    // Enabling USART0 (see errata)
    CMU_ClockEnable(cmuClock_USART0, true);

    /* Starting LFXO and waiting until it is stable */
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

    /* Routing the LFXO clock to the RTC */
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
    CMU_ClockEnable(cmuClock_RTC, true);
}


void performI2CTransfer(void)
{
    /* Transfer structure */
    I2C_TransferSeq_TypeDef i2cTransfer;

    /* Initializing I2C transfer */
    i2cTransfer.addr = (I2C_ADDRESS<<1);
    i2cTransfer.flags = I2C_FLAG_WRITE;
    i2cTransfer.buf[0].data = i2c_txBuffer;
    i2cTransfer.buf[0].len = i2c_txBufferSize;
    i2cTransfer.buf[1].data = i2c_rxBuffer;
    i2cTransfer.buf[1].len = I2C_RXBUFFER_SIZE;

    I2C_TransferInit(I2C0, &i2cTransfer);

    /* Sending data */
    while (I2C_Transfer(I2C0) == i2cTransferInProgress)
    {
        ;
    }

    i2cTransfer.addr = (I2C_ADDRESS<<1);
    i2cTransfer.flags = I2C_FLAG_WRITE_READ;
    i2cTransfer.buf[0].data = &data;
    i2cTransfer.buf[0].len = tam;
    i2cTransfer.buf[1].data = i2c_rxBuffer;
    i2cTransfer.buf[1].len = I2C_RXBUFFER_SIZE;

    I2C_TransferInit(I2C0, &i2cTransfer);

        /* Sending data */
        while (I2C_Transfer(I2C0) == i2cTransferInProgress)
        {
            ;
        }

       // AcX=i2c_rxBuffer[0]<<8|i2c_rxBuffer[1];  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
        //AcX= (AcX- (-74))*(2);
       // AcY=i2c_rxBuffer[2]<<8|i2c_rxBuffer[3];  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        //AcY= (AcY- (-74))*(2);
        //AcZ=i2c_rxBuffer[4]<<8|i2c_rxBuffer[5];  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        G[0]=i2c_rxBuffer[0]<<8|i2c_rxBuffer[1];  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
        G[1]=i2c_rxBuffer[2]<<8|i2c_rxBuffer[3];  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
        G[2]=i2c_rxBuffer[4]<<8|i2c_rxBuffer[5];  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

        /*arx = (180/3.141592) * atan(AcX / sqrt(pow(AcY, 2) + pow(AcZ, 2)));
        ary = (180/3.141592) * atan(AcY / sqrt(pow(AcX, 2) + pow(AcZ, 2)));
        arz = (180/3.141592) * atan(sqrt(pow(AcY,2) + pow(AcX,2)) / AcZ);

        rx = (0.96 * arx) + (0.04 * GyX);
        ry = (0.96 * ary) + (0.04 * GyY);
        rz = (0.96 * arz) + (0.04 * GyZ);*/
}
