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
 *   Date     : 08 feb 2023
 *   Revision : 1.0.0
 *
 *
 *                              Nucleo-L432KC Pinout                    Nucleo-L452RE Pinout
 *                                                                  CN7                         CN10
 *   E220_900T22D           PA9  []              [] VIN                 [] []                   [] []
 *   M0  [] PB0             PA10 []              [] GND                 [] []                   [] []
 *   M1  [] PB1             NRST []              [] NRST                [] []                   [] []
 *   RxD [] PA9             GND  []              [] +5V                 [] []                   [] []
 *   TxD [] PA10            PA12 []              [] PA2                 [] []                   [] []
 *   AUX [] PA12/PB2        PB0  []              [] PA7                 [] []                   [] []
 *   VCC [] +5V             PB7  []              [] PA6 PB6 (SB16)      [] []                   [] []
 *   GND [] GND             PB6  []              [] PA5 PB7 (SB18)      [] []                   [] []
 *                          PB1  []              [] PA4                 [] []+5V                [] []
 *                          PC14 []              [] PA3                 [] []                   [] []
 *                          PC15 []              [] PA1                 [] []GND        PA9/1_TX[] []PB2/AUX
 *                          PA8  []              [] PA0                 [] []                   [] []PB1/M1
 *                          PA11 []              [] AREF                [] []                   [] []
 *                          PB5  []              [] +3V3                [] []                   [] []
 *                          PB4  []              [] PB3                 [] []                   [] []
 *                                                                      [] []                   [] []
 *                                                                      [] []PB0/M0    PA10/1_RX[] []
 *                                                                      [] []                   [] []
 *                                                                      [] []                   [] []
 *
 */

#ifndef INC_E220_900T22D_H_
#define INC_E220_900T22D_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/

/* Private defines ----------------------------------------------------------*/
/**> E-Byte E220-900TD22 command parameters */
#define EB_E220_CMD_WRITE_PARAMS_S      0xC0U
#define EB_E220_CMD_READ_PARAMS         0xC1U
#define EB_E220_CMD_WRITE_PARAMS_NS     0xC2U
#define EB_E220_CMD_READ_VERSION        0xC3U
#define EB_E220_CMD_RESET               0xC4U

/**> E-Byte E220-900TD22 UART baudrate      */
#define EB_E220_UART_BAUD_MASK          0xE0U
#define EB_E220_UART_BAUD_2400          0x20U
#define EB_E220_UART_BAUD_4800          0x40U
#define EB_E220_UART_BAUD_9600          0x60U
#define EB_E220_UART_BAUD_19200         0x80U
#define EB_E220_UART_BAUD_38400         0xA0U
#define EB_E220_UART_BAUD_57600         0xC0U
#define EB_E220_UART_BAUD_115200        0xE0U

/**> E-Byte E220-900TD22 UART mode          */
#define EB_E220_UART_MODE_MASK          0x18U
#define EB_E220_UART_MODE_8N1           0x00U
#define EB_E220_UART_MODE_8O1           0x08U
#define EB_E220_UART_MODE_8E1           0x18U

/**> E-Byte E220-900TD22 Air Data Rate      */
#define EB_E220_DATA_RATE_MASK          0x07U
#define EB_E220_DATA_RATE_2400          0x02U
#define EB_E220_DATA_RATE_4800          0x03U
#define EB_E220_DATA_RATE_9600          0x04U
#define EB_E220_DATA_RATE_19200         0x05U
#define EB_E220_DATA_RATE_38400         0x06U
#define EB_E220_DATA_RATE_62500         0x07U

/**> E-Byte E220-900TD22 Sub-Packet         */
#define EB_E220_SUBPACKET_LEN_MASK      0xC0U
#define EB_E220_SUBPACKET_LEN_200B      0x00U
#define EB_E220_SUBPACKET_LEN_128B      0x40U
#define EB_E220_SUBPACKET_LEN_64B       0x80U
#define EB_E220_SUBPACKET_LEN_32B       0xC0U

/**> E-Byte E220-900TD22 RSSI Noise         */
#define EB_E220_RSSI_NOISE_MASK         0x20U
#define EB_E220_RSSI_NOISE_DISABLE      0x00U
#define EB_E220_RSSI_NOISE_ENABLE       0x20U

/**> E-Byte E220-900TD22 Tx Power           */
#define EB_E220_TX_POWER_MASK           0x03U
#define EB_E220_TX_POWER_22DB           0x00U
#define EB_E220_TX_POWER_17DB           0x01U
#define EB_E220_TX_POWER_13DB           0x02U
#define EB_E220_TX_POWER_10DB           0x03U

/**> E-Byte E220-900TD22 RSSI Byte          */
#define EB_E220_RSSI_BYTE_MASK          0x80U
#define EB_E220_RSSI_BYTE_DISABLE       0x00U
#define EB_E220_RSSI_BYTE_ENABLE        0x80U

/**> E-Byte E220-900TD22 Tx Method */
#define EB_E220_TX_METHOD_MASK          0x40U
#define EB_E220_TX_METHOD_TRANSPARENT   0x00U
#define EB_E220_TX_METHOD_FIXED         0x40U

/**> E-Byte E220-900TD22 LBT                */
#define EB_E220_LBT_MASK                0x10U
#define EB_E220_LBT_DISABLE             0x00U
#define EB_E220_LBT_ENABLE              0x10U

/**> E-Byte E220-900TD22 WOR Cycle          */
#define EB_E220_WOR_CYCLE_MASK          0x07U
#define EB_E220_WOR_CYCLE_500           0x00U
#define EB_E220_WOR_CYCLE_1000          0x01U
#define EB_E220_WOR_CYCLE_1500          0x02U
#define EB_E220_WOR_CYCLE_2000          0x03U
#define EB_E220_WOR_CYCLE_2500          0x04U
#define EB_E220_WOR_CYCLE_3000          0x05U
#define EB_E220_WOR_CYCLE_3500          0x06U
#define EB_E220_WOR_CYCLE_4000          0x07U

/**> E-Byte E220-900TD22 Default config     */
#define EB_E220_DEFAULT_ADDRESS_HIGH    0xFFU
#define EB_E220_DEFAULT_ADDRESS_LOW     0xFFU
#define EB_E220_DEFAULT_CHANNEL         0x17U
#define EB_E220_DEFAULT_WOR_CYCLE       EB_E220_WOR_CYCLE_2000
#define EB_E220_DEFAULT_TX_POWER        EB_E220_TX_POWER_10DB
#define EB_E220_DEFAULT_DATA_RATE       EB_E220_DATA_RATE_2400
#define EB_E220_DEFAULT_UART_MODE       EB_E220_UART_MODE_8N1
#define EB_E220_DEFAULT_UART_BAUD       EB_E220_UART_BAUD_9600

#define EB_E220_RSSI_BYTE_ENABLED       0U
#define EB_E220_STARTUP_MS              10U     // E220-900-T22D Startup time 10ms

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  E220_OK       = 0x00,
  E220_ERROR    = 0x01,
  E220_BUSY     = 0x02,
  E220_TIMEOUT  = 0x03
} E220_StatusTypeDef;

typedef enum
{
  ModeNormal = 0,       // Normal mode for sending and receiving usage
  ModeWakeUp,           // Adds a long preamble to transmission to allow destination receivers to wake up
  ModePowerSaving,      // Receiver sleeps until a message is received
  ModeSleep             // Parameter setting usage
} E220_OperatingMode;

typedef struct
{
  uint8_t addh;         // High address byte ADDH Register
  uint8_t addl;         // Low address byte ADDL Register
  uint8_t sped;         // Data rate and UART speed REG0 Register
  uint8_t opt1;         // Various config options1  REG1 Register
  uint8_t chan;         // Radio channel REG2 Register
  uint8_t opt2;         // Various config options2 REG3 Register
  uint8_t crypth;       // High address byte ADDH Register
  uint8_t cryptl;       // Low address byte ADDL Register
} E220_Parameters;


/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
E220_StatusTypeDef E220_Module_Init(void);
E220_StatusTypeDef E220_Receive_Packet(uint8_t* data, uint8_t len);
E220_StatusTypeDef E220_Transmit_Packet(uint8_t* data, uint8_t len);
E220_StatusTypeDef E220_Set_Channel(uint8_t channel);
E220_StatusTypeDef E220_Wait_Packet_Send(void);
void E220_Display_Parameter(E220_Parameters* parameters);


/* Private defines -----------------------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /* INC_E220_900T22D_H_ */
