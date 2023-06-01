#ifndef I2C_FUNC_H
#define I2C_FUNC_H

#include <stdlib.h>
#include <stdio.h>
#include <i2c/i2c.h>


int       SetBspConfig(const char *i2cChannelName, const char *i2cPmuxConfig);
I2cHandle InitI2cChannel(const char *i2cChannelName);

#endif
