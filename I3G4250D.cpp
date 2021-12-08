
#include "stdint.h"
#include "Wire.h"
#include "I3G4250D.h"
#include "Arduino.h"

// Scale includes 245,500,2000 dps
static int I3G4250D_Scale = 245;


/*!
 *  @brief  Instantiates a new I3G4250D class in I2C
 */
I3G4250D::I3G4250D()
{
    I2CAddress = I3G4250D_I2C_ADDR;
}
/*!
 *  @brief  I2c bus write
 *  @param  registerAddress,writeData,size
 */
void I3G4250D::writeRegister(uint8_t registerAddress, uint8_t *writeData, uint8_t size)
{
    uint8_t addr = 0;
    Wire.beginTransmission(I2CAddress);
    Wire.write(registerAddress);
    for (size_t i = 0; i < size; i++)
    {
        Wire.write(writeData[i]);
    }
    Wire.endTransmission();
}
/*!
 *  @brief  I2c bus read
 *  @param  registerAddress,writeData,size
 */
void I3G4250D::readRegister(uint8_t registerAddress, uint8_t *readData, uint8_t size)
{
	Wire.beginTransmission(I2CAddress);
    Wire.write(registerAddress | 0x80);
    Wire.endTransmission();    
	Wire.requestFrom(I2CAddress, size);
    size_t i = 0;
	while ( Wire.available() ) // slave may send less than requested
	{
		readData[i] = Wire.read(); // receive a byte as a proper uint8_t
        i++;
	}

}
/*!
 *  @brief  I3G4250D_Init, it will set basic configuration to I3G4250D
 *  @param  five reg control in the datasheet.
 */
uint8_t I3G4250D::I3G4250D_Init(uint8_t reg1, uint8_t reg2,  uint8_t reg3, uint8_t reg4, uint8_t reg5, I3G4250D_SCALE scale)
{
    Wire.begin();
    uint8_t i2cData = 0;

    i2cData = reg1;
    writeRegister(I3G4250D_CTRL_REG1, &i2cData, 1);

    i2cData = reg2;
    writeRegister(I3G4250D_CTRL_REG2, &i2cData, 1);

    i2cData = reg3;
    writeRegister(I3G4250D_CTRL_REG3, &i2cData, 1);

    i2cData = reg4;
    writeRegister(I3G4250D_CTRL_REG4, &i2cData, 1);

    i2cData = reg5;
    writeRegister(I3G4250D_CTRL_REG5, &i2cData, 1);

    switch (scale)
    {
    case I3G4250D_SCALE_245:
        I3G4250D_Scale = 245;
        break;

    case I3G4250D_SCALE_500:
        I3G4250D_Scale = 500;
        break;

    case I3G4250D_SCALE_2000:
        I3G4250D_Scale = 2000;
        break;

    case I3G4250D_SCALE_2000_2:
        I3G4250D_Scale = 2000;
        break;
    }
    return 0;
}
/*!
 *  @brief  I3G4250D_GetRawData, it will return basic data of 3 - axis
 *  @return  I3G4250D_DataRaw, data of 3 - axis
 */
I3G4250D_DataRaw I3G4250D::I3G4250D_GetRawData(void)
{
    uint8_t spiBuffer[2];
    I3G4250D_DataRaw tempRawData;

    //read X axis data
    readRegister(I3G4250D_OUT_X_L_ADDR, spiBuffer, 2);
    tempRawData.x = ((spiBuffer[1] << 8) + spiBuffer[0]);

    //read Y axis data
    readRegister(I3G4250D_OUT_Y_L_ADDR, spiBuffer, 2);
    tempRawData.y = ((spiBuffer[1] << 8) + spiBuffer[0]);

    //read Z axis data
    readRegister(I3G4250D_OUT_Z_L_ADDR, spiBuffer, 2);
    tempRawData.z = ((spiBuffer[1] << 8) + spiBuffer[0]);

    return tempRawData;
}
/*!
 *  @brief  I3G4250D_GetScaledData, it will return scaled data of 3 - axis
 *  @return  I3G4250D_DataScaled, scaled data of 3 - axis
 */
I3G4250D_DataScaled I3G4250D::I3G4250D_GetScaledData(void)
{
    I3G4250D_DataRaw tempRawData = I3G4250D_GetRawData();

    // Scaling and return
    // TODO: Check if these values are applicable to the I3G4250D, 16 bit 
    I3G4250D_DataScaled scaledData;
    scaledData.x = (float)tempRawData.x * (I3G4250D_Scale * 2)/65536;
    scaledData.y = (float)tempRawData.y * (I3G4250D_Scale * 2)/65536;
    scaledData.z = (float)tempRawData.z * (I3G4250D_Scale * 2)/65536;

    return scaledData;
}

/*!
 *  @brief  I3G4250D_MODE, it will change the mode
 *  @return  I3G4250D_MODE, power down/ sleep/ normal
 */
void I3G4250D::I3G4250D_Mode(I3G4250D_MODE mode)
{
    uint8_t i2cData = 0;
    //read reg1 data
    readRegister(I3G4250D_CTRL_REG1, &i2cData, 1);
    switch (mode)
    {
    case I3G4250D_POWER_DOWN:
        i2cData &= 0xF7 ;
        break;

    case I3G4250D_POWER_SLEEP:
        i2cData &= 0xFF ;
        break;

    case I3G4250D_POWER_NORMAL:
        i2cData &= 0xFF ;
        break;
    }
    writeRegister(I3G4250D_CTRL_REG1, &i2cData, 1);
}

/*!
 *  @brief  Enable high pass filter
 *  @param  NULL
 */
void I3G4250D::I3G4250D_EnableHighPassFilter(void)
{
    uint8_t i2cData = 0;
    readRegister(I3G4250D_CTRL_REG4, &i2cData, 1);
    i2cData |= 0x01<<4;     // 1: HPF enabled.
    writeRegister(I3G4250D_CTRL_REG4, &i2cData, 1);
}

/*!
 *  @brief  Disable high pass filter
 *  @param  NULL
 */
void I3G4250D::I3G4250D_DisableHighPassFilter(void)
{
    uint8_t i2cData = 0;
    readRegister(I3G4250D_CTRL_REG4, &i2cData, 1);
    i2cData &= ~(0x01<<4);     // 0: HPF disabled.
    writeRegister(I3G4250D_CTRL_REG4, &i2cData, 1); 
}

/*!
 *  @brief  Set three-axis trigger threshold
 *  @param  X , Y , Z axis trigger threshold
 */
void I3G4250D::I3G4250D_SetTresholds(int16_t THS_X, int16_t THS_Y, int16_t THS_Z)
{
    uint8_t i2cData = 0;
    THS_X &= 0x7FFF; 
    THS_Y &= 0x7FFF; 
    THS_Z &= 0x7FFF; 

    i2cData = (uint8_t)(THS_X / 256U) & 0x7FU;    
    writeRegister(I3G4250D_INT1_THS_XH_ADDR, &i2cData, 1);
    i2cData = (uint8_t)(THS_X & 0xff);
    writeRegister(I3G4250D_INT1_THS_XL_ADDR, &i2cData, 1);

    i2cData = (uint8_t)(THS_Y / 256U) & 0x7FU; 
    writeRegister(I3G4250D_INT1_THS_YH_ADDR, &i2cData, 1);
    i2cData = (uint8_t)(THS_Y & 0xff);
    writeRegister(I3G4250D_INT1_THS_YL_ADDR, &i2cData, 1);

    i2cData = (uint8_t)(THS_Z / 256U) & 0x7FU; 
    writeRegister(I3G4250D_INT1_THS_ZH_ADDR, &i2cData, 1);
    i2cData = (uint8_t)(THS_Z & 0xff);
    writeRegister(I3G4250D_INT1_THS_ZL_ADDR, &i2cData, 1);
}

/*!
 *  @brief  Configuration for interrupt source
 *  @param  I3G4250D_INT_CTR 
 *          I3G4250D_INT_CTR_XLI_ON/I3G4250D_INT_CTR_XHI_ON
 *          I3G4250D_INT_CTR_YLI_ON/I3G4250D_INT_CTR_YHI_ON
 *          I3G4250D_INT_CTR_ZLI_ON/I3G4250D_INT_CTR_ZHI_ON
 */
void I3G4250D::I3G4250D_InterruptCtrl(uint8_t intCtrl)
{
    uint8_t i2cData = 0;
    i2cData = 0x00 ;  // Default : close all interrupts.
    i2cData |= intCtrl;
    IntSrc  = intCtrl;
    writeRegister(I3G4250D_INT1_CFG_ADDR, &i2cData, 1);
}

/*!
 *  @brief  Get interrupt source
 *  @param  Return the value in the INT1_SRC (31h) register
 *  @return Return the value in the INT1_SRC (31h) register
 *  		ZH | ZL | YH | YL | XH | XL     See more in the data sheet Table 51. INT1_SRC description
 */
uint8_t I3G4250D::I3G4250D_GetInterruptSrc(void)
{
    uint8_t i2cData = 0;
    readRegister(I3G4250D_INT1_SRC_ADDR, &i2cData, 1);
    i2cData &= IntSrc;
    return i2cData ;
}

/*!
 *  @brief  Enable INT1 interrupt
 *  @param  NULL
 */
void I3G4250D::I3G4250D_Enable_INT1(void)
{
    uint8_t i2cData = 0;
    
    readRegister(I3G4250D_INT1_CFG_ADDR, &i2cData, 1);
    i2cData = 0x80;
    writeRegister(I3G4250D_CTRL_REG3, &i2cData, 1);
}

/*!
 *  @brief  Disable INT1 interrupt
 *  @param  NULL
 */
void I3G4250D::I3G4250D_Disable_INT1(void)
{
    uint8_t i2cData = 0;
    
    readRegister(I3G4250D_INT1_CFG_ADDR, &i2cData, 1);
    i2cData &= ~(0x01<<7);
    writeRegister(I3G4250D_CTRL_REG3, &i2cData, 1);
}
