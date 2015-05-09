#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>

extern uint16_t G[3];

void i2cInit(void);
void performI2CTransfer(void);

#endif /* I2C_H_ */
