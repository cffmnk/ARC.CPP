#pragma once

#include <stdio.h>
#include "MyRio_lib/MyRio.h"
#include "MyRio_lib/NiFpga_MyRio1900Fpga30.h"
#include "MyRio_lib/I2C.h"
#include <time.h>
#include <iostream>

#include "Config.h"

extern NiFpga_Session myrio_session;

NiFpga_Status initHardware(MyRio_I2c* i2c);