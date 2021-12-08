
#include "stdint.h"
#include <stdbool.h>
#include <string.h>


#define I3G4250D_I2C_ADDR                0x68

// Register adresses
#define I3G4250D_WHO_AM_I_ADDR           0x0F
#define I3G4250D_CTRL_REG1               0x20
#define I3G4250D_CTRL_REG2               0x21
#define I3G4250D_CTRL_REG3               0x22
#define I3G4250D_CTRL_REG4               0x23
#define I3G4250D_CTRL_REG5               0x24

#define I3G4250D_STATUS_ADDR             0x27

#define I3G4250D_OUT_X_L_ADDR            0x28
#define I3G4250D_OUT_X_H_ADDR            0x29
#define I3G4250D_OUT_Y_L_ADDR            0x2A
#define I3G4250D_OUT_Y_H_ADDR            0x2B
#define I3G4250D_OUT_Z_L_ADDR            0x2C
#define I3G4250D_OUT_Z_H_ADDR            0x2D

#define I3G4250D_INT1_CFG_ADDR           0x30
#define I3G4250D_INT1_SRC_ADDR           0x31

#define I3G4250D_INT1_THS_XH_ADDR        0x32
#define I3G4250D_INT1_THS_XL_ADDR        0x33
#define I3G4250D_INT1_THS_YH_ADDR        0x34
#define I3G4250D_INT1_THS_YL_ADDR        0x35
#define I3G4250D_INT1_THS_ZH_ADDR        0x36
#define I3G4250D_INT1_THS_ZL_ADDR        0x37

// Datarate
#define I3G4250D_DATARATE_100            ((uint8_t)0x00)       // 100 HZ
#define I3G4250D_DATARATE_200            ((uint8_t)0x40)       // 200 HZ
#define I3G4250D_DR_200_BW_12_5          ((uint8_t)0x80)       // 400 HZ
#define I3G4250D_DR_400_BW_25            ((uint8_t)0xC0)       // 800 HZ

// Bandwidth
#define I3G4250D_BANDWIDTH_12_5          ((uint8_t)0x00)       // 12.5 Cutoff
#define I3G4250D_BANDWIDTH_25            ((uint8_t)0x40)       // 25 Cutoff
#define I3G4250D_BANDWIDTH_50            ((uint8_t)0x80)       // 50 Cutoff
#define I3G4250D_BANDWIDTH_70            ((uint8_t)0x80)       // 70 Cutoff

// Datarate and bandwidth presets
#define I3G4250D_ODR_BW_LOW              ((uint8_t)0x10)       // 100HZ and a cutoff of 12.5
#define I3G4250D_ODR_BW_MEDIUM           ((uint8_t)0x60)       // 200HZ and a cuttoff of 50
#define I3G4250D_ODR_BW_HIGH             ((uint8_t)0xB0)       // 400HZ and a cuttoff of 110
#define I3G4250D_ODR_BW_ULTRA            ((uint8_t)0xF0)       // 800HZ and a cuttoff of 110

// High pass filter mode
#define I3G4250D_HPF_MODE_NORMAL        ((uint8_t)0x00)
#define I3G4250D_HPF_MODE_REFERENCE     ((uint8_t)0x01)
#define I3G4250D_HPF_MODE_NORMAL_2      ((uint8_t)0x02)
#define I3G4250D_HPF_MODE_AUTORESET     ((uint8_t)0x03)

// Highpass filter cutoff frequencies (See reference table 27 for more information.)
#define I3G4250D_HPCF_MODE_1             ((uint8_t)0x00)       // 100HZ > 8 | 200HZ > 15 | 400HZ > 30 | 800HZ > 56
#define I3G4250D_HPCF_MODE_2             ((uint8_t)0x10)       // 100HZ > 4 | 200HZ > 8 | 400HZ > 15 | 800HZ > 30
#define I3G4250D_HPCF_MODE_3             ((uint8_t)0x20)       // 100HZ > 2 | 200HZ > 4 | 400HZ > 8 | 800HZ > 15
#define I3G4250D_HPCF_MODE_4             ((uint8_t)0x30)       // 100HZ > 1 | 200HZ > 2 | 400HZ > 4 | 800HZ > 8
#define I3G4250D_HPCF_MODE_5             ((uint8_t)0x40)       // 100HZ > 0.5 | 200HZ > 1 | 400HZ > 2 | 800HZ > 4
#define I3G4250D_HPCF_MODE_6             ((uint8_t)0x50)       // 100HZ > 0.2 | 200HZ > 0.5 | 400HZ > 1 | 800HZ > 2
#define I3G4250D_HPCF_MODE_7             ((uint8_t)0x60)       // 100HZ > 0.1 | 200HZ > 0.2 | 400HZ > 0.5 | 800HZ > 1
#define I3G4250D_HPCF_MODE_8             ((uint8_t)0x70)       // 100HZ > 0.05 | 200HZ > 0.1 | 400HZ > 0.2 | 800HZ > 0.5
#define I3G4250D_HPCF_MODE_9             ((uint8_t)0x80)       // 100HZ > 0.02 | 200HZ > 0.05 | 400HZ > 0.1 | 800HZ > 0.2
#define I3G4250D_HPCF_MODE_10            ((uint8_t)0x90)       // 100HZ > 0.01 | 200HZ > 0.02 | 400HZ > 0.05 | 800HZ > 0.1


// Work mode
typedef enum I3G4250D_MODE
{
      I3G4250D_POWER_DOWN,
      I3G4250D_POWER_SLEEP,
      I3G4250D_POWER_NORMAL,
}I3G4250D_MODE;

typedef enum I3G4250D_SCALE
{
      I3G4250D_SCALE_245 = ((uint8_t)0x00),
      I3G4250D_SCALE_500 = ((uint8_t)0x40),
      I3G4250D_SCALE_2000 = ((uint8_t)0x80),
      I3G4250D_SCALE_2000_2 = ((uint8_t)0xC0),
}I3G4250D_SCALE;

// Configuration for interrupt source.
typedef enum I3G4250D_INT_CTR
{
      I3G4250D_INT_CTR_XLI_ON  = ((uint8_t)(0x01 << 0)),    // enable interrupt request on measured rate value lower than preset threshold.
      I3G4250D_INT_CTR_XHI_ON  = ((uint8_t)(0x01 << 1)),    // enable interrupt request on measured rate value higher than preset threshold.

      I3G4250D_INT_CTR_YLI_ON  = ((uint8_t)(0x01 << 2)),
      I3G4250D_INT_CTR_YHI_ON  = ((uint8_t)(0x01 << 3)),

      I3G4250D_INT_CTR_ZLI_ON  = ((uint8_t)(0x01 << 4)),
      I3G4250D_INT_CTR_ZHI_ON  = ((uint8_t)(0x01 << 5)),

      I3G4250D_INT_CTR_ALL_ON  = ((uint8_t)(0x3F)),          // enable all interrupt request.
      I3G4250D_INT_CTR_ALL_OFF = ((uint8_t)(0x00)),           // disable all interrupt request.
}I3G4250D_INT_CTR;

// Enabled Axis
#define I3G4250D_ENABLE_ALL_AXIS         ((uint8_t)0x0F)       // Enables all gyroscope Axis
#define I3G4250D_ENABLE_Z                ((uint8_t)0x0C)       // Enables ONLY the Z axis
#define I3G4250D_ENABLE_Y                ((uint8_t)0x0A)       // Enables ONLY the Y axis
#define I3G4250D_ENABLE_X                ((uint8_t)0x09)       // Enables ONLY the X axis


// Accelerometer data
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} I3G4250D_DataRaw;

typedef struct
{
    float x;
    float y;
    float z;
} I3G4250D_DataScaled;


class I3G4250D
{
public:
    I3G4250D(void);
	
    uint8_t I3G4250D_Init(uint8_t reg1, uint8_t reg2,  uint8_t reg3, uint8_t reg4, uint8_t reg5, I3G4250D_SCALE scale);
	
	// readRegister reads one 8-bit register
    void readRegister(uint8_t registerAddress, uint8_t *readData, uint8_t size);
		
	// Writes an 8-bit byte;
    void writeRegister(uint8_t registerAddress, uint8_t *writeData, uint8_t size);

    I3G4250D_DataRaw I3G4250D_GetRawData(void);

    I3G4250D_DataScaled I3G4250D_GetScaledData(void);
	void I3G4250D_Mode(I3G4250D_MODE mode);

    void I3G4250D_EnableHighPassFilter(void);
    void I3G4250D_DisableHighPassFilter(void);

    void I3G4250D_SetTresholds(int16_t THS_X, int16_t THS_Y, int16_t THS_Z);
    void I3G4250D_InterruptCtrl(uint8_t intCtrl);
    
    void I3G4250D_Enable_INT1(void);
    void I3G4250D_Disable_INT1(void);

    uint8_t I3G4250D_GetInterruptSrc(void);
    
private:
	uint8_t I2CAddress;
  uint8_t IntSrc;
};


