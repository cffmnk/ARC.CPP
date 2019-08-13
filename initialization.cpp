#include "initialization.h"

NiFpga_Status initHardware(MyRio_I2c* i2c) {

	NiFpga_Status status;
	
	status = MyRio_Open();

	uint8_t selectReg;

	/*
	 * Initialize the I2C struct with registers from the FPGA personality.
	 */
	i2c->addr = NiFpga_MyRio1900Fpga30_ControlU8_I2CAADDR; //I2CAADDR;
	i2c->cnfg = NiFpga_MyRio1900Fpga30_ControlU8_I2CACNFG; //I2CBCNFG;
	i2c->cntl = NiFpga_MyRio1900Fpga30_ControlU8_I2CACNTL; //I2CBCNTL;
	i2c->cntr = NiFpga_MyRio1900Fpga30_ControlU8_I2CACNTR; //I2CBCNTR;
	i2c->dati = NiFpga_MyRio1900Fpga30_IndicatorU8_I2CADATI; //I2CBDATI;
	i2c->dato = NiFpga_MyRio1900Fpga30_ControlU8_I2CADATO; //I2CBDATO;
	i2c->go = NiFpga_MyRio1900Fpga30_ControlBool_I2CAGO; //I2CBGO;
	i2c->stat = NiFpga_MyRio1900Fpga30_IndicatorU8_I2CASTAT; //I2CBSTAT;

	/*
	 * Enable the I2C functionality on Connector B
	 * Read the value of the SELECTB register.
	 */
	status = NiFpga_ReadU8(myrio_session, NiFpga_MyRio1900Fpga30_ControlU8_SYSSELECTA, &selectReg);
	MyRio_ReturnValueIfNotSuccess(status, status, "Could not read from the SYSSELECTB register!");

	/*
	 * Set bit7 of the SELECT register to enable the I2C functionality. The
	 * functionality of this bit is specified in the documentation.
	 */
	selectReg = selectReg | (1 << 7);

	/*
	 * Write the updated value of the SELECT register.
	 */
	status = NiFpga_WriteU8(myrio_session, NiFpga_MyRio1900Fpga30_ControlU8_SYSSELECTA, selectReg);
	MyRio_ReturnValueIfNotSuccess(status, status, "Could not write to the SYSSELECTB register!");

	/*
	 * Set the speed of the I2C block.
	 *
	 * Standard mode (100 kbps) = 213.
	 * Fast mode (400 kbps) = 63.
	 */
	I2c_Counter(i2c, 213);

	/*
	 * Enable the I2C block.
	 */
	I2c_Configure(i2c, I2c_Enabled);

	return status;
}