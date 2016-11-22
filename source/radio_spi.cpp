/**
******************************************************************************
* @file    radio_spi.c
* @author  System Lab - NOIDA
* @version V1.0.0
* @date    15-May-2014
* @brief   This file provides code for the configuration of the SPI instances.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/


/* Includes ------------------------------------------------------------------*/
#include "radio_spi.h"

#include "SimpleSpirit1.h"


/**
 * @addtogroup BSP
 * @{
 */


/**
 * @addtogroup X-NUCLEO-IDS02Ax
 * @{
 */


/**
 * @defgroup RADIO_SPI_Private_TypesDefinitions       RADIO_SPI Private Types Definitions
 * @{
 */

/**
 * @}
 */


/**
 * @defgroup RADIO_SPI_Private_Defines                RADIO_SPI Private Defines
 * @{
 */

/**
 * @}
 */


/**
 * @defgroup RADIO_SPI_Private_Macros                 RADIO_SPI Private Macros
 * @{
 */

/**
 * @}
 */


/**
 * @defgroup RADIO_SPI_Private_Variables              RADIO_SPI Private Variables
 * @{
 */

/**
 * @}
 */


/**
 * @defgroup RADIO_SPI_Private_FunctionPrototypes     RADIO_SPI Private Function Prototypes
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup RADIO_SPI_Private_Functions              RADIO_SPI Private Functions
 * @{
 */

/**
* @}
*/

/**
* @brief  Write single or multiple RF Transceivers register
* @param  cRegAddress: base register's address to be write
* @param  cNbBytes: number of registers and bytes to be write
* @param  pcBuffer: pointer to the buffer of values have to be written into registers
* @retval StatusBytes
*/
StatusBytes SdkEvalSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
	return SimpleSpirit1::Instance().SdkEvalSpiWriteRegisters(cRegAddress, cNbBytes, pcBuffer);
}

StatusBytes SimpleSpirit1::SdkEvalSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t aHeader[2] = {0};
  uint16_t tmpstatus = 0x0000;
  StatusBytes *pStatus=(StatusBytes *)&tmpstatus;

  /* Built the aHeader bytes */
  aHeader[0] = WRITE_HEADER;
  aHeader[1] = cRegAddress;

  /* Puts the SPI chip select low to start the transaction */
  chip_sync_select();

  /* Write the aHeader bytes and read the SPIRIT1 status bytes */
  tmpstatus = _spi.write(aHeader[0]);
  tmpstatus = tmpstatus << 8;

  /* Write the aHeader bytes and read the SPIRIT1 status bytes */
  tmpstatus |= _spi.write(aHeader[1]);

  /* Writes the registers according to the number of bytes */
  for (int index = 0; index < cNbBytes; index++)
  {
	  _spi.write(pcBuffer[index]);
  }

  /* Puts the SPI chip select high to end the transaction */
  chip_sync_unselect();

  return *pStatus;
}


/**
* @brief  Read single or multiple SPIRIT1 register
* @param  cRegAddress: base register's address to be read
* @param  cNbBytes: number of registers and bytes to be read
* @param  pcBuffer: pointer to the buffer of registers' values read
* @retval StatusBytes
*/
StatusBytes SdkEvalSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
	return SimpleSpirit1::Instance().SdkEvalSpiReadRegisters(cRegAddress, cNbBytes, pcBuffer);
}

StatusBytes SimpleSpirit1::SdkEvalSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint16_t tmpstatus = 0x00;
  StatusBytes *pStatus = (StatusBytes *)&tmpstatus;

  uint8_t aHeader[2] = {0};

  /* Built the aHeader bytes */
  aHeader[0] = READ_HEADER;
  aHeader[1] = cRegAddress;

  /* Put the SPI chip select low to start the transaction */
  chip_sync_select();

  /* Write the aHeader bytes and read the SPIRIT1 status bytes */
  tmpstatus = _spi.write(aHeader[0]);
  tmpstatus = tmpstatus << 8;

  /* Write the aHeader bytes and read the SPIRIT1 status bytes */
  tmpstatus |= _spi.write(aHeader[1]);

  for (int index = 0; index < cNbBytes; index++)
  {
	  pcBuffer[index] = _spi.write(0xFF);
  }

  /* Put the SPI chip select high to end the transaction */
  chip_sync_unselect();

  return *pStatus;
}


/**
* @brief  Send a command
* @param  cCommandCode: command code to be sent
* @retval StatusBytes
*/
StatusBytes SdkEvalSpiCommandStrobes(uint8_t cCommandCode)
{
	return SimpleSpirit1::Instance().SdkEvalSpiCommandStrobes(cCommandCode);
}

StatusBytes SimpleSpirit1::SdkEvalSpiCommandStrobes(uint8_t cCommandCode)
{
  uint8_t aHeader[2] = {0};
  uint16_t tmpstatus = 0x0000;

  StatusBytes *pStatus = (StatusBytes *)&tmpstatus;

  /* Built the aHeader bytes */
  aHeader[0] = COMMAND_HEADER;
  aHeader[1] = cCommandCode;

  /* Puts the SPI chip select low to start the transaction */
  chip_sync_select();

  /* Write the aHeader bytes and read the SPIRIT1 status bytes */
  tmpstatus = _spi.write(aHeader[0]);
  tmpstatus = tmpstatus<<8;

  /* Write the aHeader bytes and read the SPIRIT1 status bytes */
  tmpstatus |= _spi.write(aHeader[1]);

  /* Puts the SPI chip select high to end the transaction */
  chip_sync_unselect();

  return *pStatus;
}


/**
* @brief  Write data into TX FIFO
* @param  cNbBytes: number of bytes to be written into TX FIFO
* @param  pcBuffer: pointer to data to write
* @retval StatusBytes
*/
StatusBytes SdkEvalSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
	return SimpleSpirit1::Instance().SdkEvalSpiWriteFifo(cNbBytes, pcBuffer);
}

StatusBytes SimpleSpirit1::SdkEvalSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint16_t tmpstatus = 0x0000;
  StatusBytes *pStatus = (StatusBytes *)&tmpstatus;

  uint8_t aHeader[2] = {0};

  /* Built the aHeader bytes */
  aHeader[0] = WRITE_HEADER;
  aHeader[1] = LINEAR_FIFO_ADDRESS;

  /* Put the SPI chip select low to start the transaction */
  chip_sync_select();

  /* Write the aHeader bytes and read the SPIRIT1 status bytes */
  tmpstatus = _spi.write(aHeader[0]);
  tmpstatus = tmpstatus<<8;

  /* Write the aHeader bytes and read the SPIRIT1 status bytes */
  tmpstatus |= _spi.write(aHeader[1]);

  /* Writes the registers according to the number of bytes */
  for (int index = 0; index < cNbBytes; index++)
  {
	  _spi.write(pcBuffer[index]);
  }

  /* Put the SPI chip select high to end the transaction */
  chip_sync_unselect();

  return *pStatus;
}

/**
* @brief  Read data from RX FIFO
* @param  cNbBytes: number of bytes to read from RX FIFO
* @param  pcBuffer: pointer to data read from RX FIFO
* @retval StatusBytes
*/
StatusBytes SdkEvalSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
	return SimpleSpirit1::Instance().SdkEvalSpiReadFifo(cNbBytes, pcBuffer);
}

StatusBytes SimpleSpirit1::SdkEvalSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint16_t tmpstatus = 0x0000;
  StatusBytes *pStatus = (StatusBytes *)&tmpstatus;

  uint8_t aHeader[2];

  /* Built the aHeader bytes */
  aHeader[0]=READ_HEADER;
  aHeader[1]=LINEAR_FIFO_ADDRESS;

  /* Put the SPI chip select low to start the transaction */
  chip_sync_select();

  /* Write the aHeader bytes and read the SPIRIT1 status bytes */
  tmpstatus = _spi.write(aHeader[0]);
  tmpstatus = tmpstatus<<8;

  /* Write the aHeader bytes and read the SPIRIT1 status bytes */
  tmpstatus |= _spi.write(aHeader[1]);

  for (int index = 0; index < cNbBytes; index++)
  {
	  pcBuffer[index] = _spi.write(0xFF);
  }

  /* Put the SPI chip select high to end the transaction */
  chip_sync_unselect();

  return *pStatus;
}


/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
