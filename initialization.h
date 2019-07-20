#pragma once

#include "MyRio_lib/MyRio.h"
#include "MyRio_lib/I2C.h"
#include "MyRio_lib/DIO.h"
#include <iostream>
#include "Config.h"

extern NiFpga_Session myrio_session;

NiFpga_Status initHardware(NiFpga_Status* status, MyRio_I2c* i2cA, MyRio_Dio* Button);