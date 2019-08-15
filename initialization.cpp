#include "initialization.h"

NiFpga_Status initHardware(NiFpga_Status* status, MyRio_I2c* i2cA, MyRio_Dio* ButtonL, MyRio_Dio* ButtonR, MyRio_Dio* LED1, MyRio_Dio* LED2)
{
	uint8_t selectReg;

	/*
     * Initialize the I2C struct with registers from the FPGA personality.
     */
	i2cA->addr = I2CAADDR;
	i2cA->cnfg = I2CACNFG;
	i2cA->cntl = I2CACNTL;
	i2cA->cntr = I2CACNTR;
	i2cA->dati = I2CADATI;
	i2cA->dato = I2CADATO;
	i2cA->go = I2CAGO;
	i2cA->stat = I2CASTAT;
	
	/*
     * Open the myRIO NiFpga Session.
     * This function MUST be called before all other functions. After this call
     * is complete the FPGA will be loaded and the myRIO target will be ready
     * to be used.
     */
	*status = MyRio_Open();
	if (MyRio_IsNotSuccess(*status))
	{
		return *status;
	}
	
	/*
     * Enable the I2C functionality on Connector A
     *
     * Read the value of the SELECTA register.
     */
	*status = NiFpga_ReadU8(myrio_session, SYSSELECTA, &selectReg);

	MyRio_ReturnValueIfNotSuccess(*status,
		*status,
		"Could not read from the SYSSELECTA register!");

	/*
	 * Set bit7 of the SELECT register to enable the I2C functionality. The
	 * functionality of this bit is specified in the documentation.
	 */
	selectReg = selectReg | (1 << 7);

	/*
	 * Write the updated value of the SELECT register.
	 */
	*status = NiFpga_WriteU8(myrio_session, SYSSELECTA, selectReg);

	MyRio_ReturnValueIfNotSuccess(*status,
		*status,
		"Could not write to the SYSSELECTA register!");

	/*
	 * Set the speed of the I2C block.
	 *
	 * Standard mode (100 kbps) = 187.
	 * Fast mode (400 kbps) = 51.
	 *
	 * These values are calculated using the formula:
	 *   f_SCL = f_clk / (2 * CNTR) - 4
	 *
	 * where:
	 *   f_SCL = the desired frequency of the I2C transmission
	 *   f_clk = the frequency of the myRIO FPGA clock (40 Mhz default)
	 *
	 * This formula and its rationale can be found in the documentation.
	 */
//	if (I2C_CLOCK == 400)
//		I2c_Counter(i2cA, 51);
//	else
//		I2c_Counter(i2cA, 187);
	
	I2c_Counter(i2cA, 187);

	/*
	 * Enable the I2C block.
	 */
	I2c_Configure(i2cA, I2c_Enabled);
	//printf("I2C init succeed!\n");
	
	
	ButtonR->dir = DIOC_70DIR;
	ButtonR->out = DIOC_70OUT;
	ButtonR->in = DIOC_70IN;
	ButtonR->bit = 7;
	
	ButtonL->dir = DIOC_70DIR;
	ButtonL->out = DIOC_70OUT;
	ButtonL->in = DIOC_70IN;
	ButtonL->bit = 6;
	
	LED1->dir = DIOA_158DIR;
	LED1->out = DIOA_158OUT;
	LED1->in = DIOA_158IN;
	LED1->bit = 2;
	
	LED2->dir = DIOA_158DIR;
	LED2->out = DIOA_158OUT;
	LED2->in = DIOA_158IN;
	LED2->bit = 1;
	//printf("DIO init succeed!\n");
	
	return *status;
}