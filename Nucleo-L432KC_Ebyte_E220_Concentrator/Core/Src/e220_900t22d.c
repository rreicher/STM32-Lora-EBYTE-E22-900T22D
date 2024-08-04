/**************************************************************************//**
 *   Copyright (C) 2015 romain reicher
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *   About
 *   ---------------------------------------------------------------------
 *   romain reicher
 *   Date     : 29 may 2024
 *   Revision : 1.1.0
 *
 *   History;
 *   Rev 1.1.0
 *      Change E220_AUXPinCheck() with timeout instead dirty delay
 *
 *
 *   Rev 1.0.0
 *      Initial implementation
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "e220_900t22d.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

/* Private typedef -----------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define E220_MODE_TIMEOUT_MS    40U     // Timeout for E220 Mode switching
#define E220_TX_TIMEOUT_MS       2U     // Timeout for E220 packet send

/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart1;       // Normally defined in main by CubeMX
static int32_t mode = -1;

/* Private function prototypes -----------------------------------------------*/
static void E220_Set_Mode(E220_OperatingMode opMode);
static E220_StatusTypeDef E220_AUXPinCheck(uint32_t timeout);
static E220_StatusTypeDef E220_Read_Param(E220_Parameters* parameters);
static E220_StatusTypeDef E220_Write_Param(E220_Parameters* parameters, bool save);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief Initialyze E220 Module
  * @param1 None
  * @retval E220_StatusTypeDef enum.
  */
E220_StatusTypeDef E220_Module_Init(void)
{
  E220_StatusTypeDef ret = E220_OK;
  E220_Parameters currentParams;

  /**> Set the operating mode */
  E220_Set_Mode(ModeSleep);

#if (DEBUG_TRACE == 1)
  /**> Read actual module configuration */
  printf("\r\nE220 currents parameters reading after RESET\r\n");
#endif
  if (E220_Read_Param(&currentParams) != E220_OK)
  {
#if (DEBUG_TRACE == 1)
    printf("Reading Error\r\n");
#endif
    ret = E220_ERROR;
  }
  else
  {
#if (DEBUG_TRACE == 1)
    printf("Reading Done\r\n");
#endif

    /**> Display the list of module parameters */
    E220_Display_Parameter(&currentParams);

    /**> Fill the new parameters in the structure */
    E220_Parameters defaultParams;
    defaultParams.addh = EB_E220_DEFAULT_ADDRESS_HIGH;
    defaultParams.addl = EB_E220_DEFAULT_ADDRESS_LOW;
    defaultParams.chan = EB_E220_DEFAULT_CHANNEL;
    defaultParams.sped = EB_E220_DEFAULT_UART_BAUD |
                         EB_E220_DEFAULT_UART_MODE |
                         EB_E220_DEFAULT_DATA_RATE;
    defaultParams.opt1 = EB_E220_DEFAULT_TX_POWER;
    defaultParams.opt2 = EB_E220_DEFAULT_WOR_CYCLE;

#ifdef RH_E220_RSSI_BYTE_ENABLED
    defaultParams.opt2 |= RH_E220_PARAM_OPT2_RSSI_BYTE_ENABLE;
#endif

    /**> Write the new module configuration */
    if (memcmp(&defaultParams, &currentParams, sizeof(E220_Parameters)) != 0)
    {
#if (DEBUG_TRACE == 1)
      printf("E220 news parameters writing\r\n");
#endif
      if (E220_Write_Param(&defaultParams, true) != E220_OK)
      {
#if (DEBUG_TRACE == 1)
        printf("Writing Error\r\n");
#endif
        ret = E220_ERROR;
      }
    }
#if (DEBUG_TRACE == 1)
    printf("Writing Done\r\n");
    /**> Read new module configuration */
    printf("\r\nE220 new parameters reading\r\n");
#endif
    if (E220_Read_Param(&currentParams) != E220_OK)
    {
#if (DEBUG_TRACE == 1)
      printf("Reading Error\r\n");
#endif
      ret = E220_ERROR;
    }
#if (DEBUG_TRACE == 1)
    printf("Reading Done\r\n");
#endif
    /**> Display the list of module parameters */
    E220_Display_Parameter(&currentParams);

    ret = E220_OK;
  }
  /**> Set back module in operating mode */
  E220_Set_Mode(ModeNormal);

  return ret;
}

/**
  * @brief Receive an amount of data
  * @param1 Buffer to store data to receive
  * @param2 Length in byte
  * @retval E220_StatusTypeDef enum.
  */
E220_StatusTypeDef E220_Receive_Packet(uint8_t* data, uint8_t len)
{
  E220_StatusTypeDef ret = E220_OK;

  /**> Check if module is Idle */
  E220_AUXPinCheck(E220_TX_TIMEOUT_MS);

  /**> Flush the USART Data Register */
  __HAL_UART_FLUSH_DRREGISTER(&huart1);

  /**> Transmit the buffer */
  if (HAL_UART_Receive(&huart1, data, len, 0xFFF) != HAL_OK)
    ret = E220_ERROR;

  return ret;
}

/**
  * @brief Transmit an amount of data
  * @param1 Buffer of data to transmit
  * @param2 Length in byte
  * @retval E220_StatusTypeDef enum.
  */
E220_StatusTypeDef E220_Transmit_Packet(uint8_t* data, uint8_t len)
{
  E220_StatusTypeDef ret = E220_OK;

  /**> Check if module is Idle */
  E220_AUXPinCheck(E220_TX_TIMEOUT_MS);

  /**> Flush the USART Data Register */
  __HAL_UART_FLUSH_DRREGISTER(&huart1);

  /** Receive a buffer */
  if (HAL_UART_Transmit(&huart1, data, len, 0xFFF) != HAL_OK)
    ret = E220_ERROR;

  return ret;
}

/**
  * @brief Wait module idle and ready before new transmit/receive
  * @param1 None
  * @retval E220_StatusTypeDef enum.
  */
E220_StatusTypeDef E220_Wait_Packet_Send(void)
{
   return E220_AUXPinCheck(E220_TX_TIMEOUT_MS);
}

/**
  * @brief Display all parameters of the module
  * @param1 Current Parameters E220_Parameters Structure
  * @retval None.
  */
void E220_Display_Parameter(E220_Parameters* parameters)
{
#if (DEBUG_TRACE == 1)
  printf("E220-900T22D Parameters List\r\n");
  printf("Address = 0x%X\r\n", (parameters->addh << 8) | parameters->addl);

  /**> REG0 Serial baud rate Settings */
  if ((parameters->sped & EB_E220_UART_BAUD_MASK) == EB_E220_UART_BAUD_115200)
    printf("UART Baudrate = 115200\r\n");
  if ((parameters->sped & EB_E220_UART_BAUD_MASK) == EB_E220_UART_BAUD_57600)
    printf("UART Baudrate = 57600\r\n");
  if ((parameters->sped & EB_E220_UART_BAUD_MASK) == EB_E220_UART_BAUD_38400)
    printf("UART Baudrate = 38400\r\n");
  if ((parameters->sped & EB_E220_UART_BAUD_MASK) == EB_E220_UART_BAUD_19200)
    printf("UART Baudrate = 19200\r\n");
  if ((parameters->sped & EB_E220_UART_BAUD_MASK) == EB_E220_UART_BAUD_9600)
    printf("UART Baudrate = 9600 (Default)\r\n");
  if ((parameters->sped & EB_E220_UART_BAUD_MASK) == EB_E220_UART_BAUD_4800)
    printf("UART Baudrate = 4800\r\n");
  if ((parameters->sped & EB_E220_UART_BAUD_MASK) == EB_E220_UART_BAUD_2400)
    printf("UART Baudrate = 2400\r\n");

  /**> REG0 Serial Parity */
  if ((parameters->sped & EB_E220_UART_MODE_MASK) == EB_E220_UART_MODE_8N1)
    printf("UART Parity Mode = 8N1 (Default)\r\n");
  if ((parameters->sped & EB_E220_UART_MODE_MASK) == EB_E220_UART_MODE_8E1)
    printf("UART Parity Mode = 8E1\r\n");
  if ((parameters->sped & EB_E220_UART_MODE_MASK) == EB_E220_UART_MODE_8O1)
    printf("UART Parity Mode = 8O1\r\n");

  /**> REG0 RF Air Data rate Settings */
  if ((parameters->sped & EB_E220_DATA_RATE_MASK) == EB_E220_DATA_RATE_62500)
    printf("RF Data Rate = 62.5kbps\r\n");
  if ((parameters->sped & EB_E220_DATA_RATE_MASK) == EB_E220_DATA_RATE_38400)
    printf("RF Data Rate = 38.4kbps\r\n");
  if ((parameters->sped & EB_E220_DATA_RATE_MASK) == EB_E220_DATA_RATE_19200)
    printf("RF Data Rate = 19.2kbps\r\n");
  if ((parameters->sped & EB_E220_DATA_RATE_MASK) == EB_E220_DATA_RATE_9600)
    printf("RF Data Rate = 9600bps\r\n");
  if ((parameters->sped & EB_E220_DATA_RATE_MASK) == EB_E220_DATA_RATE_4800)
    printf("RF Data Rate = 4800bps\r\n");
  if ((parameters->sped & EB_E220_DATA_RATE_MASK) == EB_E220_DATA_RATE_2400)
    printf("RF Data Rate = 2400bps (Default)\r\n");

  /**> REG1 Sub Packet Settings*/
  if ((parameters->opt1 & EB_E220_SUBPACKET_LEN_MASK) == EB_E220_SUBPACKET_LEN_200B)
    printf("RF Sub-Packet Settings = 200Bytes (Default)\r\n");
  if ((parameters->opt1 & EB_E220_SUBPACKET_LEN_MASK) == EB_E220_SUBPACKET_LEN_128B)
    printf("RF Sub-Packet Settings = 128Bytes\r\n");
  if ((parameters->opt1 & EB_E220_SUBPACKET_LEN_MASK) == EB_E220_SUBPACKET_LEN_64B)
    printf("RF Sub-Packet Settings = 64Bytes\r\n");
  if ((parameters->opt1 & EB_E220_SUBPACKET_LEN_MASK) == EB_E220_SUBPACKET_LEN_32B)
    printf("RF Sub-Packet Settings = 32Bytes\r\n");

  /**> REG1 RSSI Ambient Noise */
  if ((parameters->opt1 & EB_E220_RSSI_NOISE_MASK) == EB_E220_RSSI_NOISE_ENABLE)
    printf("RF RSSI Ambiant Noise [Enabled]\r\n");
  if ((parameters->opt1 & EB_E220_RSSI_NOISE_MASK) == EB_E220_RSSI_NOISE_DISABLE)
    printf("RF RSSI Ambiant Noise [Disabled] (Default)\r\n");

  /**> REG1 Tx Power dBm */
  if ((parameters->opt1 & EB_E220_TX_POWER_MASK) == EB_E220_TX_POWER_22DB)
    printf("RF Tx Power = 22dBm (Default)\r\n");
  if ((parameters->opt1 & EB_E220_TX_POWER_MASK) == EB_E220_TX_POWER_17DB)
    printf("RF Tx Power = 17dBm\r\n");
  if ((parameters->opt1 & EB_E220_TX_POWER_MASK) == EB_E220_TX_POWER_13DB)
    printf("RF Tx Power = 13dBm\r\n");
  if ((parameters->opt1 & EB_E220_TX_POWER_MASK) == EB_E220_TX_POWER_10DB)
    printf("RF Tx Power = 10dBm\r\n");

  /**> REG2 Channel Frequency */
  printf("RF Channel = %d\r\n", parameters->chan);
  /**> Module frequency band equation: 850.125 + Channel * 1E6 */
  printf("RF Frequency = %luMHz\r\n", (uint32_t)(parameters->chan + 850.125));

  /**> REG3 RSSI Byte Settings */
  if ((parameters->opt2 & EB_E220_RSSI_BYTE_MASK) == EB_E220_RSSI_BYTE_ENABLE)
    printf("RF RSSI Byte [Enabled]\r\n");
  if ((parameters->opt2 & EB_E220_RSSI_BYTE_MASK) == EB_E220_RSSI_BYTE_DISABLE)
    printf("RF RSSI Byte [Disabled] (Default)\r\n");

  /**> REG3 Transmit Method Settings */
  if ((parameters->opt2 & EB_E220_TX_METHOD_MASK) == EB_E220_TX_METHOD_FIXED)
    printf("RF TX Method Fixed\r\n");
  if ((parameters->opt2 & EB_E220_TX_METHOD_MASK) == EB_E220_TX_METHOD_TRANSPARENT)
    printf("RF TX Method Transparent (default)\r\n");

  /**> REG3 LBT Settings */
  if ((parameters->opt2 & EB_E220_LBT_MASK) == EB_E220_LBT_ENABLE)
    printf("RF LBT [Enabled]\r\n");
  if ((parameters->opt2 & EB_E220_TX_METHOD_MASK) == EB_E220_LBT_DISABLE)
    printf("RF LBT [Disabled] (default)\r\n");

  /**> REG3 WOR Settings */
  if ((parameters->opt2 & EB_E220_WOR_CYCLE_MASK) == EB_E220_WOR_CYCLE_4000)
    printf("WOR Cycle = 4000ms\r\n");
  if ((parameters->opt2 & EB_E220_WOR_CYCLE_MASK) == EB_E220_WOR_CYCLE_3500)
    printf("WOR Cycle = 3500ms\r\n");
  if ((parameters->opt2 & EB_E220_WOR_CYCLE_MASK) == EB_E220_WOR_CYCLE_3000)
    printf("WOR Cycle = 3000ms\r\n");
  if ((parameters->opt2 & EB_E220_WOR_CYCLE_MASK) == EB_E220_WOR_CYCLE_2500)
    printf("WOR Cycle = 2500ms\r\n");
  if ((parameters->opt2 & EB_E220_WOR_CYCLE_MASK) == EB_E220_WOR_CYCLE_2000)
    printf("WOR Cycle = 2000ms\r\n");
  if ((parameters->opt2 & EB_E220_WOR_CYCLE_MASK) == EB_E220_WOR_CYCLE_1500)
    printf("WOR Cycle = 1500ms\r\n");
  if ((parameters->opt2 & EB_E220_WOR_CYCLE_MASK) == EB_E220_WOR_CYCLE_1000)
    printf("WOR Cycle = 1000ms\r\n");
  if ((parameters->opt2 & EB_E220_WOR_CYCLE_MASK) == EB_E220_WOR_CYCLE_500)
    printf("WOR Cycle = 500ms\r\n");
#endif
}


/* Local functions ----------------------------------------------------------*/

/**
  * @brief Set the operating mode of the Module
  * @param1 Operating mode defined in E220_OperatingMode enum
  * @retval None
  */
static void E220_Set_Mode(E220_OperatingMode opMode)
{
 /**> Configure M0 & M1 IO according mode */
 switch (opMode)
 {
   case ModeNormal: // Mode Normal
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      mode = ModeNormal;
      break;
    case ModeWakeUp: // Mode Wakeup
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      mode = ModeWakeUp;
      break;
    case ModePowerSaving: // Mode Power Saving
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
      mode = ModePowerSaving;
      break;
    case ModeSleep: // Mode Sleep
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
      mode = ModeSleep;
      break;
    default:    // Mode Normal as default
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      mode = ModeNormal;
      break;
  }
 /**> Wait a delay */
  HAL_Delay(10);

 /**> Check again if module is Idle */
 E220_AUXPinCheck(E220_MODE_TIMEOUT_MS);
}

/**
  * @brief Check and wait with timeout if the module is ready to communicate
  * @param1 None
  * @retval E220_StatusTypeDef enum.
  */
static E220_StatusTypeDef E220_AUXPinCheck(uint32_t timeout)
{
  E220_StatusTypeDef ret = E220_OK;
  __IO uint32_t tickstart = HAL_GetTick();
  uint32_t timevalue = 0;

  if ((uint32_t)(tickstart + timeout) == 0)
    tickstart = 0;

  /**> Loop during AUX pin = low, meaning the module is nor ready to receive or transmit */
  while (HAL_GPIO_ReadPin(E220_AUX_GPIO_Port, E220_AUX_Pin) == GPIO_PIN_RESET)
  {
    timevalue = (uint32_t)(HAL_GetTick() - tickstart);
    if (timevalue > timeout)
    {
      ret = E220_TIMEOUT;
#if (DEBUG_TRACE == 1)
      printf("AUX timeout %lu ms\r\n", timevalue);
#endif
      return ret;
    }
  }
#if (DEBUG_TRACE == 1)
  printf("AUX High %lu ms\r\n", timevalue);
#endif
  return ret;
}

/**
  * @brief Read the parameters
  * @param1 Current Parameters E220_Parameters Structure
  * @retval E220_StatusTypeDef enum.
  */
static uint8_t E220_Read_Param(E220_Parameters* parameters)
{
  E220_StatusTypeDef ret = E220_OK;

  if (mode == ModeSleep)
  {
    uint16_t outBuffLen;
    uint16_t inBuffLen;

    /**> Construct serial read response Ex: {C1, 0x00, Lenght} */
    uint8_t readParamsResponse[11];
    memset(readParamsResponse, 0, sizeof(readParamsResponse));
    /**> Get the lenght of the read command array */
    inBuffLen = sizeof(readParamsResponse);

    /**> Construct serial read command Ex: {C1, 0x00, Lenght} */
    uint8_t readParamsCommand[] = {EB_E220_CMD_READ_PARAMS, 0x0, 8};
    /**> Get the lenght of the read command array */
    outBuffLen = sizeof(readParamsCommand);

    /**> Flush the USART Data Register */
    __HAL_UART_FLUSH_DRREGISTER(&huart1);

    /**> Transmit the read command array to the module */
    if (HAL_UART_Transmit(&huart1, (uint8_t *)readParamsCommand, outBuffLen, 0x100) != HAL_OK)
      ret = E220_ERROR;

    /**> Receive the response of the module */
    if (HAL_UART_Receive(&huart1, (uint8_t *)readParamsResponse, inBuffLen, 0x100) != HAL_OK)
      ret = E220_ERROR;

    /**> Check error response of module */
    if ((readParamsResponse[0] == 0xff) &&
        (readParamsResponse[1] == 0xff) &&
        (readParamsResponse[2] == 0xff))
    {
#if (DEBUG_TRACE == 1)
      printf("E220 Module responds FF\r\n");
#endif
      ret = E220_ERROR;
    }
    /**> Check good response  of module */
    if (readParamsResponse[0] == 0xc1)
    {
#if (DEBUG_TRACE == 1)
      printf("E220 Module responds C1\r\n");
#endif
      ret = E220_OK;
    }

    /**> Update the actual parameters structure */
    parameters->addh = readParamsResponse[3];
    parameters->addl = readParamsResponse[4];
    parameters->sped = readParamsResponse[5];
    parameters->opt1 = readParamsResponse[6];
    parameters->chan = readParamsResponse[7];
    parameters->opt2 = readParamsResponse[8];
    parameters->crypth = readParamsResponse[9];
    parameters->cryptl = readParamsResponse[10];
  }
  else
    ret = E220_ERROR;

  return ret;
}

/**
  * @brief Write the parameters of the module
  * @param1 Current Parameters E220_Parameters Structure
  * @param2 Save module parameters if = 1, no save otherwise
  * @retval E220_StatusTypeDef enum.
  */
static E220_StatusTypeDef E220_Write_Param(E220_Parameters* parameters, bool save)
{
  E220_StatusTypeDef ret = E220_OK;

  if (mode == ModeSleep)
  {
    uint16_t outBuffLen;
    uint16_t inBuffLen;

    /**> Construct serial write response Ex: {C1, 0x00, Lenght} */
    uint8_t writeParamsResponse[11];
    memset(writeParamsResponse, 0, sizeof(writeParamsResponse));
    /**> Get the lenght of the read command array */
    inBuffLen = sizeof(writeParamsResponse);

    /**> Select Serial write command, Save or Not Save option */
    uint8_t header = save ? EB_E220_CMD_WRITE_PARAMS_S : EB_E220_CMD_WRITE_PARAMS_NS;
    /**> Construct serial write command Ex: {C0, 0x00, Lenght} */
    uint8_t writeParamsCommand[] = {header, 0x00, 8, parameters->addh,
                                    parameters->addl, parameters->sped,
                                    parameters->opt1, parameters->chan,
                                    parameters->opt2, parameters->crypth,
                                    parameters->cryptl};
    /**> Get the lenght of the write command array */
    outBuffLen = sizeof(writeParamsCommand);

    /**> Flush the USART Data Register */
    __HAL_UART_FLUSH_DRREGISTER(&huart1);

    /**> Transmit the write command array to the module */
    if (HAL_UART_Transmit(&huart1, (uint8_t *)writeParamsCommand, outBuffLen, 0x100) != HAL_OK)
      ret = E220_ERROR;

    /**> Receive the response of the module */
    if (HAL_UART_Receive(&huart1, (uint8_t *)writeParamsResponse, inBuffLen, 0x100) != HAL_OK)
      ret = E220_ERROR;

    /**> Check error response of module */
    if ((writeParamsResponse[0] == 0xff) &&
        (writeParamsResponse[1] == 0xff) &&
        (writeParamsResponse[2] == 0xff))
    {
#if (DEBUG_TRACE == 1)
      printf("E220 Module responds FF\r\n");
#endif
      ret = E220_ERROR;
    }

    /**> Check good response  of module */
    if (writeParamsResponse[0] == 0xc1)
    {
#if (DEBUG_TRACE == 1)
      printf("E220 Module responds C1\r\n");
#endif
      ret = E220_OK;
    }
  }
  else
    return E220_ERROR;

  return ret;
}
