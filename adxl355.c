
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_twi.h"
#include "adxl355.h"

bool adxl355_register_write(ADXL355 *sensor, uint8_t register_address, uint8_t value)
{
    ret_code_t err_code;
    uint8_t tx_buf[ADXL355_ADDRESS_LEN+1];
	
    //Write the register address and data into transmit buffer
    tx_buf[0] = register_address;
    tx_buf[1] = value;

    //Set the flag to false to show the transmission is not yet completed
    sensor->mTransferDone = false;
    
    //Transmit the data over TWI Bus
    err_code = nrf_drv_twi_tx(sensor->mHandle, ADXL355_ADDRESS, tx_buf, ADXL355_ADDRESS_LEN+1, false);
    
    //Wait until the transmission of the data is finished
    while (sensor->mTransferDone == false) {}

    // if there is no error then return true else return false
    if (NRF_SUCCESS != err_code)
    {
        return false;
    }
    
    return true;	
}


bool adxl355_register_read(ADXL355 *sensor, uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes)
{
    ret_code_t err_code;

    //Set the flag to false to show the receiving is not yet completed
    sensor->mTransferDone = false;
    
    // Send the Register address where we want to write the data
    err_code = nrf_drv_twi_tx(sensor->mHandle, ADXL355_ADDRESS, &register_address, 1, true);
	  
    //Wait for the transmission to get completed
    while (sensor->mTransferDone == false){}
    
    // If transmission was not successful, exit the function with false as return value
    if (NRF_SUCCESS != err_code)
    {
        return false;
    }

    //set the flag again so that we can read data from the ADXL355's internal register
    sensor->mTransferDone = false;
	  
    // Receive the data from the ADXL355
    err_code = nrf_drv_twi_rx(sensor->mHandle, ADXL355_ADDRESS, destination, number_of_bytes);
		
    //wait until the transmission is completed
    while (sensor->mTransferDone == false){}
	
    // if data was successfully read, return true else return false
    if (NRF_SUCCESS != err_code)
    {
        return false;
    }
    
    return true;
}


bool adxl355_verify_product_id(ADXL355 *sensor)
{
  uint8_t readId; // create a variable to hold the who am i value

  if (!adxl355_register_read(sensor, ADXL355_DEVID_AD_REG, &readId, 1))
  {
    return false;
  }
  
  if (readId != ADXL355_DEVID_AD)
  {
    return false;
  }

  return true;
}

bool adxl355_verify_mems_id(ADXL355 *sensor)
{
  uint8_t regData; // create a variable to hold the who am i value

  if (!adxl355_register_read(sensor, ADXL355_DEVID_MST_REG, &regData, 1))
  {
    return false;
  }
  
  if (regData != ADXL355_MEMS_ID)
  {
    return false;
  }

  return true;
}

bool adxl355_verify_part_id(ADXL355 *sensor)
{
  uint8_t regData; // create a variable to hold the who am i value

  if (!adxl355_register_read(sensor, ADXL355_PARTID_REG, &regData, 1))
  {
    return false;
  }

  if (regData != ADXL355_PART_ID)
  {
      return false;
  }
  return true;
}

/*
  Function to initialize the ADXL355
*/ 
bool adxl355_init(ADXL355 *sensor, const nrf_drv_twi_t *m_twi)
{   
  sensor->mHandle = m_twi;
  sensor->mTransferDone = false;
  sensor->initialised = false;

  //Check the id to confirm that we are communicating with the right device
  if(adxl355_verify_product_id(sensor) == false)
  {
    return false;
  }

  if(adxl355_verify_mems_id(sensor) == false)
  {
    return false;
  }

  if(adxl355_verify_part_id(sensor) == false)
  {
    return false;
  }

  sensor->initialised = true;
  return true;

}


/*
  A Function to read accelerometer's values from the internal registers of ADXL355
*/ 
bool adxl355_ReadAcc(ADXL355 *sensor, int32_t *pACC_X , int32_t *pACC_Y , int32_t *pACC_Z )
{
  uint8_t buf[9];
  bool ret = false;

  if(adxl355_register_read(sensor, ADXL355_XDATA3_REG, buf, 9) == true)
  {
    
    *pACC_X = ((((int32_t)buf[0] << 24) | ((int32_t)buf[1] << 16 ) | ((int32_t)(buf[2] & 0xF0)) << 8) >> 12);
    *pACC_Y = ((((int32_t)buf[3] << 24) | ((int32_t)buf[4] << 16 ) | ((int32_t)(buf[5] & 0xF0)) << 8) >> 12);
    *pACC_Z = ((((int32_t)buf[6] << 24) | ((int32_t)buf[7] << 16 ) | ((int32_t)(buf[8] & 0xF0)) << 8) >> 12);
	
    ret = true;
  }		
  
  return ret;
}


/*
  A Function to read tempersture values from the internal registers of ADXL355
*/ 
bool adxl355_ReadTemp(ADXL355 *sensor, uint16_t *pTemp )
{
  uint8_t buf[2];
  bool ret = false;
  
  if(adxl355_register_read(sensor, ADXL355_TEMP2_REG, buf, 2) == true)
  {
    *pTemp = (((buf[0] & 0x0F) << 8) | buf[1]) ;
    	
    ret = true;
  }		
  
  return ret;
}


/*
  A Function to determine if there is new accelerometer data from ADXL355
*/ 
bool adxl355_DataReady(ADXL355 *sensor)
{
  uint8_t buf[1];
  bool ret = false;
  
  if(adxl355_register_read(sensor, ADXL355_STATUS_REG, buf, 1) == true)
  {
      if (buf[0] & 0b00000001 == true)
      {	
        ret = true;
      }
  }		
  
  return ret;
}

bool adxl355_setPowerControl(ADXL355 *sensor, uint8_t flags)
{
  return adxl355_register_write(sensor, ADXL355_POWER_CTL_REG, flags);
}

bool adxl355_setFilterSettings(ADXL355 *sensor, uint8_t flags)
{
  return adxl355_register_write(sensor, ADXL355_FILTER_REG, flags);
}

bool adxl_setRange(ADXL355 *sensor, ADXL_RANGE rangeMode)
{
  uint8_t bitMask = 0;
  bitMask |= rangeMode;
  return adxl355_register_write(sensor, ADXL355_RANGE_REG, bitMask);
}