#include "nrf_drv_twi.h"

#ifndef ADXL355_H__
#define ADXL355_H__

#define ADXL355_ADDRESS_LEN  1         //ADXL355
#define ADXL355_ADDRESS     0x1D       //ADXL355 Device Address
#define ADXL355_MEMS_ID     0x1D       //ADXL355 Device Address
#define ADXL355_PART_ID     0xED       //ADXL355 Device Address
#define ADXL355_DEVID_AD     0xADU     //Analog Devices ID

//ADXL355 Registers addresses, see datasheet for more info and each register's function

#define ADXL355_DEVID_AD_REG        0x00  // This register contains the Analog Devices ID, 0xAD. 
#define ADXL355_DEVID_MST_REG       0x01 // This register contains the Analog Devices MEMS ID, 0x1D
#define ADXL355_PARTID_REG          0x02  // This register contains the device ID, 0xED
#define ADXL355_REVID_REG           0x03
#define ADXL355_STATUS_REG          0x04
#define ADXL355_FIFO_ENTRIES_REG    0x05
#define ADXL355_TEMP2_REG           0x06
#define ADXL355_TEMP1_REG           0x07
#define ADXL355_XDATA3_REG          0x08
#define ADXL355_XDATA2_REG          0x09
#define ADXL355_XDATA1_REG          0x0A
#define ADXL355_YDATA3_REG          0x0B
#define ADXL355_YDATA2_REG          0x0C
#define ADXL355_YDATA1_REG          0x0D
#define ADXL355_ZDATA3_REG          0x0E
#define ADXL355_ZDATA2_REG          0x0F
#define ADXL355_ZDATA1_REG          0x10
#define ADXL355_FIFO_DATA_REG       0x11
#define ADXL355_OFFSET_X_H_REG      0x1E
#define ADXL355_OFFSET_X_L_REG      0x1F
#define ADXL355_OFFSET_Y_H_REG      0x20
#define ADXL355_OFFSET_Y_L_REG      0x21
#define ADXL355_OFFSET_Z_H_REG      0x22
#define ADXL355_OFFSET_Z_L_REG      0x23
#define ADXL355_ACT_EN_REG          0x24
#define ADXL355_ACT_THRESH_H_REG    0x25
#define ADXL355_ACT_THRESH_L_REG    0x26
#define ADXL355_ACT_COUNT_REG       0x27
#define ADXL355_FILTER_REG          0x28
#define ADXL355_FIFO_SAMPLES_REG    0x29
#define ADXL355_INT_MAP_REG         0x2A
#define ADXL355_SYNC_REG            0x2B
#define ADXL355_RANGE_REG           0x2C
#define ADXL355_POWER_CTL_REG       0x2D
#define ADXL355_SELF_TEST_REG       0x2E
#define ADXL355_RESET_REG           0x2F

#define ADXL355_POWER_CONTROL_FLAG_MEASUREMENT_MODE (uint8_t)0x00
#define ADXL355_POWER_CONTROL_FLAG_STANDBY (uint8_t)0x01
#define ADXL355_POWER_CONTROL_FLAG_TEMP_OFF (uint8_t)0x02
#define ADXL355_POWER_CONTROL_FLAG_TEMP_ON (uint8_t)0x00
#define ADXL355_POWER_CONTROL_FLAG_DRRDY_OFF (uint8_t)0x04
#define ADXL355_POWER_CONTROL_FLAG_DRRDY_ON (uint8_t)0x00


#define ADXL355_HPF_CORNER_DISABLED   0x00
#define ADXL335_HPF_CORNER_24_7       0x10
#define ADXL355_HPF_CORNER_6_2084     0x20
#define ADXL355_HPF_CORNER_1_5545     0x30
#define ADXL355_HPF_CORNER_0_3862     0x40
#define ADXL355_HPF_CORNER_0_0954     0x50
#define ADXL355_HPF_CORNER_0_0238     0x60

#define ADXL355_ODR_LPF_4000HZ_1000HZ     0x00
#define ADXL355_ODR_LPF_2000HZ_500HZ      0x01
#define ADXL355_ODR_LPF_1000HZ_250HZ      0x02
#define ADXL355_ODR_LPF_500HZ_125HZ       0x03
#define ADXL355_ODR_LPF_250HZ_62_5HZ      0x04
#define ADXL355_ODR_LPF_125HZ_31_25HZ     0x05
#define ADXL355_ODR_LPF_62_5HZ_15_625HZ   0x06
#define ADXL355_ODR_LPF_31_25HZ_7_813HZ   0x07
#define ADXL355_ODR_LPF_15_625HZ_3_906HZ  0x08
#define ADXL355_ODR_LPF_7_813HZ_1_953HZ   0x09
#define ADXL355_ODR_LPF_3_906HZ_0_977HZ   0x0A

typedef struct ADXL355 
{
  const nrf_drv_twi_t *mHandle;

  bool mTransferDone;
} ADXL355;

typedef enum ADXL_RANGE
{
  ADXL_RANGE_2G = 1,
  ADXL_RANGE_4G = 2,
  ADXL_RANGE_8G = 3
}ADXL_RANGE; 


/**
  @brief Function for Initialising a ADXL355 I2C device.
  @param[in] sensor Pointer to ADXL355 structure
  @param[in] m_twi nrf driver I2C handle
  @retval true initialisation succeeded
  @retval false initialisation failed
*/
bool adxl355_init(ADXL355 *sensor, const nrf_drv_twi_t *m_twi);

/**
  @brief Function for writing a ADXL355 register contents over TWI.
  @param[in] sensor Pointer to ADXL355 structure
  @param[in] register_address adress of register to write to
  @param[in] value Value to write into register_address
  @retval true write succeeded
  @retval false write failed
*/
bool adxl355_register_write(ADXL355 *sensor, uint8_t register_address, const uint8_t value);

/**
  @brief Function for reading ADXL355 register contents over TWI.
  Reads one or more consecutive registers.
  @param[in] sensor Pointer to ADXL355 structure
  @param[in]  register_address Register address to start reading from
  @param[in]  number_of_bytes Number of bytes to read
  @param[out] destination Pointer to a data buffer where read data will be stored
  @retval true Register read succeeded
  @retval false Register read failed
*/
bool adxl355_register_read(ADXL355 *sensor, uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes);

/**
  @brief Function for reading and verifying Analog Devices ID.
  @param[in] sensor Pointer to ADXL355 structure
  @retval true ID is what was expected
  @retval false ID was not what was expected
*/  
bool adxl355_verify_product_id(ADXL355 *sensor);

/**
  @brief Function for reading and verifying the ADXL355 MEMs ID.
  @param[in] sensor Pointer to ADXL355 structure
  @retval true ID is what was expected
  @retval false ID was not what was expected
*/  
bool adxl355_verify_mems_id(ADXL355 *sensor);

/**
  @brief Function for reading and verifying the ADXL355 Part ID.
  @param[in] sensor Pointer to ADXL355 structure
  @retval true ID is what was expected
  @retval false ID was not what was expected
*/  
bool adxl355_verify_part_id(ADXL355 *sensor);

/**
  @brief Function for reading accelerometer data from ADXL355.
  @param[in] sensor Pointer to ADXL355 structure
  @param[out] pACC_X Pointer to a where read X aceleration data will be stored
  @param[out] pACC_Y Pointer to a where read Y aceleration data will be stored
  @param[out] pACC_Z Pointer to a where read Z aceleration data will be stored
  @retval true data was read successfully
  @retval false data read failed
*/  
bool adxl355_ReadAcc(ADXL355 *sensor, int32_t *pACC_X , int32_t *pACC_Y , int32_t *pACC_Z );

/**
  @brief Function for reading temperature register from ADXL355 device.
  @param[in] sensor Pointer to ADXL355 structure
  @param[out] pTemp Pointer to a where read temperature data will be stored
  @retval true data was read successfully
  @retval false data read failed
*/   
bool adxl355_ReadTemp(ADXL355 *sensor, uint16_t *pTemp );

/**
  @brief Function check if there is new accelerometer data available to be read from ADXL355 device.
  @param[in] sensor Pointer to ADXL355 structure
  @retval true data is available to be read from ADXL355 device
  @retval false no new data is available to be read
*/
bool adxl355_DataReady(ADXL355 *sensor);

/**
  @brief Function to set the power control flags of ADXL355 device.
  @param[in] sensor Pointer to ADXL355 structure
  @param[in] flags the flags for setting the power control register.
              Valid flags:
              - ADXL355_POWER_CONTROL_FLAG_MEASUREMENT_MODE (uint8_t)0x00
              - ADXL355_POWER_CONTROL_FLAG_STANDBY
              - ADXL355_POWER_CONTROL_FLAG_TEMP_OFF
              - ADXL355_POWER_CONTROL_FLAG_TEMP_ON
              - ADXL355_POWER_CONTROL_FLAG_DRRDY_OFF
              - ADXL355_POWER_CONTROL_FLAG_DRRDY_ON
  @retval true power control flags were successfully written to device
  @retval false writing to power control registers failed
*/
bool adxl355_setPowerControl(ADXL355 *sensor, uint8_t flags);

/**
  @brief Function to set the internal high-pass and low-pass filter settings.
  @param[in] sensor Pointer to ADXL355 structure
  @param[in] flags flags for setting the filter setting
              Valid Flags:
              - ADXL355_HPF_CORNER_DISABLED
              - ADXL335_HPF_CORNER_24_7
              - ADXL355_HPF_CORNER_6_2084
              - ADXL355_HPF_CORNER_1_5545
              - ADXL355_HPF_CORNER_0_3862
              - ADXL355_HPF_CORNER_0_0954
              - ADXL355_HPF_CORNER_0_0238

              - ADXL355_ODR_LPF_4000HZ_1000HZ
              - ADXL355_ODR_LPF_2000HZ_500HZ
              - ADXL355_ODR_LPF_1000HZ_250HZ
              - ADXL355_ODR_LPF_500HZ_125HZ
              - ADXL355_ODR_LPF_250HZ_62_5HZ
              - ADXL355_ODR_LPF_125HZ_31_25HZ
              - ADXL355_ODR_LPF_62_5HZ_15_625HZ
              - ADXL355_ODR_LPF_31_25HZ_7_813HZ
              - ADXL355_ODR_LPF_15_625HZ_3_906HZ
              - ADXL355_ODR_LPF_7_813HZ_1_953HZ
              - ADXL355_ODR_LPF_3_906HZ_0_977HZ

  @retval true flags were successfully written to device
  @retval false writing to filter settings registers failed
*/
bool adxl355_setFilterSettings(ADXL355 *sensor, uint8_t flags);

/**
  @brief Function to set the acceleration range ADXL355 device.
  @param[in] sensor Pointer to ADXL355 structure
  @param[in] rangeMode The acceleration range to sent the sensor to 
  @retval true Acceleration range was sucessfully written to device
  @retval false Setting acceleration range failed
*/
bool adxl_setRange(ADXL355 *sensor, ADXL_RANGE rangeMode);

#endif


