/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ads131m0x.h"

#include <stdlib.h>
#include <math.h>
//#include <arm_math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

typedef struct ADData24 {
	  uint8_t b0;
	  uint8_t b1;
    uint8_t b2;
	  uint8_t datas[8][3];
    uint8_t b3;
    uint8_t b4;
    uint8_t b5;
    uint8_t b6;
    uint8_t b7;
} ADData24;
ADData24 addata24;
ADData24 addata24s[uint8_ad_adc_number];
ADData24 addata24sw[uint8_ad_adc_number];

#define datasBuffersize (uint8_ad_adc_number)

//#define dataBuffer110size (2 + uint8_ad_chan_number * (3) * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 1)
#define dataBuffer110size (2 + uint8_ad_chan_number * (1 + 3) * uint8_ad_adc_number + uint8_accel_chan_number * 2 + 1)

uint8_t dataBuffer110[dataBuffer110size];

uint8_t datasBuffer[datasBuffersize][uint8_ad_number];
//uint8_t aTxdatasBuffer[datasBuffersize][uint8_ad_number];
//uint8_t aRxdatasBuffer[datasBuffersize][uint8_ad_number];

uint8_t aTxBuffer[uint8_ad_number];
uint8_t aRxBuffer[uint8_ad_number];

ads131m0x_dev *devices[uint8_ad_adc_number];
//ad7779_dev *devices[uint8_ad_adc_number];

uint8_t ui8SampleNumber=-1;
uint8_t ui8SampleNumber2=-1;
uint32_t ui32SampleNumber=-1;

//const uint8_t uint8_data_number_print = 200;
//const uint8_t uint8_data_number_print2 = 200;
#define uint8_data_number_print  1300
uint8_t dataBuffer_print[uint8_data_number_print];

char buff[256];

bool flag_nDRDY_INTERRUPT = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void print_hex(int v, int num_places);
void print_binary(int v, int num_places );
void print_symbol(uint8_t v);
void print_text(const char * t);
void print_line();
void print_text_line(const char * t);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int32_t interpret24bitAsInt32( uint8_t * byteBuffer)
{
	// create a big endian so that we could adapt to host architecture later on
//	int32_t newInt = (byteBuffer[0] << 24) | (byteBuffer[1] << 16) | byteBuffer[2] << 8;
	int32_t newInt = (byteBuffer[2] << 24) | (byteBuffer[1] << 16) | byteBuffer[0] << 8;
	// depending on most significant byte, set positive or negative value
	if ((newInt & 0x00008000) > 0)
	{
		newInt |= 0x000000FF;
	} else {
		newInt &= 0xFFFFFF00;
	}
	// convert back from big endian (network order) to host
	return newInt;
//	return htonl(newInt);
}

void interpretInt32As24bit(uint8_t * byteBuffer, int32_t oldInt)
{
	byteBuffer[2] = oldInt >> 24;
	byteBuffer[1] = oldInt >> 16;
	byteBuffer[0] = oldInt >> 8;
//	byteBuffer[0] = oldInt >> 24;
//	byteBuffer[1] = oldInt >> 16;
//	byteBuffer[2] = oldInt >> 8;
}

void UART_Printf(const char* fmt, ...) {
//    char buff[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buff, sizeof(buff), fmt, args);
//    while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
//    {
//    }
//    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)buff, strlen(buff));
//    if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
//    {
//        HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff),
//                          HAL_MAX_DELAY);
//    }
    if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
    {
        HAL_UART_Transmit(&huart2, (uint8_t*)buff, strlen(buff),
                          HAL_MAX_DELAY);
    }
    va_end(args);
}

void print_hex(int v, int num_places)
{
	  const uint32_t uint32_data_number = (num_places % 4 == 0 ? num_places / 4 : num_places / 4 + 1);
	  uint32_t uint32_data_number_written;
//  uint8_t dataBuffer[uint8_data_number];
//	  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
//	  {
//		  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
//		  {
//		  }
//	  }
	  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
	  {
		  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
		  {
		  }
	  }
  int mask=0, n, num_nibbles, digit, nibbles_max;

    for (n=1; n<=num_places; n++)
    {
        mask = (mask << 1) | 0x0001;
    }
    v = v & mask; // truncate v to specified number of places

    num_nibbles = num_places / 4;
    if ((num_places % 4) != 0)
    {
        ++num_nibbles;
    }
    nibbles_max=num_nibbles;

    do
    {
        digit = ((v >> (num_nibbles-1) * 4)) & 0x0f;
        if(digit == 0)  dataBuffer_print[ nibbles_max-num_nibbles ] = '0';
        if(digit == 1)  dataBuffer_print[ nibbles_max-num_nibbles ] = '1';
        if(digit == 2)  dataBuffer_print[ nibbles_max-num_nibbles ] = '2';
        if(digit == 3)  dataBuffer_print[ nibbles_max-num_nibbles ] = '3';
        if(digit == 4)  dataBuffer_print[ nibbles_max-num_nibbles ] = '4';
        if(digit == 5)  dataBuffer_print[ nibbles_max-num_nibbles ] = '5';
        if(digit == 6)  dataBuffer_print[ nibbles_max-num_nibbles ] = '6';
        if(digit == 7)  dataBuffer_print[ nibbles_max-num_nibbles ] = '7';
        if(digit == 8)  dataBuffer_print[ nibbles_max-num_nibbles ] = '8';
        if(digit == 9)  dataBuffer_print[ nibbles_max-num_nibbles ] = '9';
        if(digit == 10) dataBuffer_print[ nibbles_max-num_nibbles ] = 'A';
        if(digit == 11) dataBuffer_print[ nibbles_max-num_nibbles ] = 'B';
        if(digit == 12) dataBuffer_print[ nibbles_max-num_nibbles ] = 'C';
        if(digit == 13) dataBuffer_print[ nibbles_max-num_nibbles ] = 'D';
        if(digit == 14) dataBuffer_print[ nibbles_max-num_nibbles ] = 'E';
        if(digit == 15) dataBuffer_print[ nibbles_max-num_nibbles ] = 'F';
    } while(--num_nibbles);
//    HAL_UART_Transmit_DMA(&huart1, dataBuffer, uint8_data_number);
//    HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);

//    if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
//    {
//  	  if(UART_DMA)
//  	  {
//            if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer_print, uint32_data_number) != HAL_OK)
//            {
//              Error_Handler();
//            }
//  	  } else
//  	  {
//  	      if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
//  	      {
//  	          if(HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer_print, uint32_data_number,5000) != HAL_OK)
//  	          {
//  	            Error_Handler();
//  	          }
//            }
//        }
//    }
    if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
    {
  	  if(UART_DMA)
  	  {
            if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dataBuffer_print, uint32_data_number) != HAL_OK)
            {
              Error_Handler();
            }
  	  } else
  	  {
            if(HAL_UART_Transmit(&huart2, (uint8_t*)dataBuffer_print, uint32_data_number,5000) != HAL_OK)
            {
              Error_Handler();
            }
        }
    }
}


void print_binary(int v, int num_places  )
{
  const uint32_t uint32_data_number = num_places;
  uint32_t uint32_data_number_written;
//  uint8_t dataBuffer[uint8_data_number];
//  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
//  {
//	  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
//	  {
//	  }
//  }
  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
  {
	  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
	  {
	  }
  }
    int mask=0, n;

    for (n=1; n<=num_places; n++)
    {
        mask = (mask << 1) | 0x0001;
    }
    v = v & mask;  // truncate v to specified number of places

    while(num_places)
    {

        if (v & (0x0001 << (num_places-1)))
        {
//             Serial.print("1");
             dataBuffer_print[ uint32_data_number-num_places ] = '1';
        }
        else
        {
//             Serial.print("0");
          dataBuffer_print[ uint32_data_number-num_places ] = '0';
         }

        --num_places;
        if(((num_places%4) == 0) && (num_places != 0))
        {
//            Serial.print("_");
        }
    }
//    if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
//    {
//  	  if(UART_DMA)
//  	  {
//            if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer_print, uint32_data_number) != HAL_OK)
//            {
//              Error_Handler();
//            }
//  	  } else
//  	  {
//  	      if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
//  	      {
//  	          if(HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer_print, uint32_data_number,5000) != HAL_OK)
//  	          {
//  	            Error_Handler();
//  	          }
//            }
//        }
//    }
    if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
    {
  	  if(UART_DMA)
  	  {
            if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dataBuffer_print, uint32_data_number) != HAL_OK)
            {
              Error_Handler();
            }
  	  } else
  	  {
            if(HAL_UART_Transmit(&huart2, (uint8_t*)dataBuffer_print, uint32_data_number,5000) != HAL_OK)
            {
              Error_Handler();
            }
        }
    }
//    HAL_UART_Transmit_DMA(&huart1, dataBuffer, uint8_data_number);
//    HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);

    //    while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
    {
    }
}


void print_symbol(uint8_t v)
{
  const uint32_t uint32_data_number = 1;
  uint32_t uint32_data_number_written;
//  uint8_t dataBuffer[uint8_data_number];
//  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
//  {
//	  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
//	  {
//	  }
//  }
  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
  {
	  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
	  {
	  }
  }
  dataBuffer_print[ 0 ] = v;
//  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer, uint8_data_number);
//  HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);
//  HAL_UART_Transmit_IT(&huart1, (uint8_t*)dataBuffer, uint8_data_number);
//  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
//  {
//	  if(UART_DMA)
//	  {
//          if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer_print, uint32_data_number) != HAL_OK)
//          {
//            Error_Handler();
//          }
//	  } else
//	  {
//	      if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
//	      {
//	          if(HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer_print, uint32_data_number,5000) != HAL_OK)
//	          {
//	            Error_Handler();
//	          }
//          }
//      }
//  }
  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
  {
	  if(UART_DMA)
	  {
          if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dataBuffer_print, uint32_data_number) != HAL_OK)
          {
            Error_Handler();
          }
	  } else
	  {
          if(HAL_UART_Transmit(&huart2, (uint8_t*)dataBuffer_print, uint32_data_number,5000) != HAL_OK)
          {
            Error_Handler();
          }
      }
  }


//  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
  {
  }
}

void print_text(const char * t)
{
  int text_length = strlen(t);
  const uint32_t uint32_data_number = text_length;
  uint32_t uint32_data_number_written;
//  uint8_t dataBuffer[uint8_data_number];
//  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
//  {
//	  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
//	  {
//	  }
//  }
  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
  {
	  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
	  {
	  }
  }
  for (int n=0; n<text_length; n++)
  {
//	  dataBuffer[n] = (uint8_t)(t[n]);
	  dataBuffer_print[n] = (uint8_t)(t[n]);
  }
//  HAL_UART_Transmit_DMA(&huart1, dataBuffer, uint8_data_number);
//  HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);
//  HAL_UART_Transmit_IT(&huart1, (uint8_t*)dataBuffer, uint8_data_number);
//  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
//  {
//	  if(UART_DMA)
//	  {
//          if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer_print, uint32_data_number) != HAL_OK)
//          {
//            Error_Handler();
//          }
//	  } else
//	  {
//	      if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
//	      {
//	          if(HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer_print, uint32_data_number,5000) != HAL_OK)
//	          {
//	            Error_Handler();
//	          }
//          }
//      }
//  }
  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
  {
	  if(UART_DMA)
	  {
          if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dataBuffer_print, uint32_data_number) != HAL_OK)
          {
            Error_Handler();
          }
	  } else
	  {
          if(HAL_UART_Transmit(&huart2, (uint8_t*)dataBuffer_print, uint32_data_number,5000) != HAL_OK)
          {
            Error_Handler();
          }
      }
  }


}

void print_line()
{
  const uint32_t uint32_data_number = 2;
  uint32_t uint32_data_number_written;
//  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
//  {
//	  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
//	  {
//	  }
//  }
  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
  {
	  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
	  {
	  }
  }
  dataBuffer_print[ 0 ] = '\r';
  dataBuffer_print[ 1 ] = '\n';
  //    HAL_UART_Transmit_DMA(&huart1, dataBuffer, uint8_data_number);
//      HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer, uint8_data_number, 5000);
//  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
//  {
//	  if(UART_DMA)
//	  {
//          if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer_print, uint32_data_number) != HAL_OK)
//          {
//            Error_Handler();
//          }
//	  } else
//	  {
//	      if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
//	      {
//	          if(HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer_print, uint32_data_number,5000) != HAL_OK)
//	          {
//	            Error_Handler();
//	          }
//          }
//      }
//  }
  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
  {
	  if(UART_DMA)
	  {
          if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dataBuffer_print, uint32_data_number) != HAL_OK)
          {
            Error_Handler();
          }
	  } else
	  {
          if(HAL_UART_Transmit(&huart2, (uint8_t*)dataBuffer_print, uint32_data_number,5000) != HAL_OK)
          {
            Error_Handler();
          }
      }
  }


//  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
  {
  }
}

void print_text_line(const char * t)
{
  print_text(t);
  print_line();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  if((FREEEEG16_OUT & FREEEEG16_TEST_COUNTER))
  {
    uint32_t test_counter=0;
    uint32_t test_blink_counter=0;
    while (1)
    {
      if(FREEEEG16_OUT & FREEEEG16_TEST_COUNTER)
      {
        if(test_counter%1000000==0)
//        if(test_counter%10000<5000)
        {
          print_hex(test_counter, 32);
          print_symbol(';');
//            print7_hex(test_counter, 32);
//            print7_symbol(';');
//        print2_hex(test_counter, 32);
//        print2_symbol(';');
        }
        test_counter++;
      }


//        if(FREESMARTEEG_OUT & FREESMARTEEG_TEST_BLINK)
//        {
//          if(test_blink_counter%1000000<50000)
//  //        if(test_blink_counter%1000000<500000)
//  //        if(test_blink_counter%1000000<500000)
//          {
//            HAL_GPIO_WritePin(led_ports[3], led_pins[3], GPIO_PIN_SET);
//  //          HAL_GPIO_WritePin(led_ports[0], led_pins[0], GPIO_PIN_SET);
//          }
//          else
//          {
//            HAL_GPIO_WritePin(led_ports[3], led_pins[3], GPIO_PIN_RESET);
//  //          HAL_GPIO_WritePin(led_ports[0], led_pins[0], GPIO_PIN_RESET);
//          }
//          test_blink_counter++;
//        }

    }
  }

  ads131m0x_dev *device1 = NULL;
  ads131m0x_init_param init_param;
  uint8_t i;
//  init_param.spi_dev.dev = &hspi2;
//  init_param.spi_dev.chip_select_port = SPI2_NSS_GPIO_Port;
//  init_param.spi_dev.chip_select_pin = SPI2_NSS_Pin;
  init_param.spi_dev.dev = &hspi1;
  init_param.spi_dev.chip_select_port = SPI1_NSS_GPIO_Port;
  init_param.spi_dev.chip_select_pin = SPI1_NSS_Pin;
  ads131m0x_setup(&device1, init_param);
  uint16_t response = InitADC(device1, true);
  devices[0]=device1;

//    ad7779_set_spi_op_mode(device, AD7779_SD_CONV);

  if(uint8_ad_adc_number >= 2)
  {
	  ads131m0x_dev *device2 = NULL;
	  ads131m0x_init_param init_param2 = init_param;

//    init_param2.spi_dev.dev = &hspi4;
//      init_param2.spi_dev.dev = &hspi3;
//      init_param2.spi_dev.dev = &hspi2;
//    init_param2.spi_dev.dev = &hspi1;
//      init_param2.spi_dev.dev = &hspi1;
//      init_param2.spi_dev.chip_select_port = SPI1_NSS_GPIO_Port;
//      init_param2.spi_dev.chip_select_pin = SPI1_NSS_Pin;
      init_param2.spi_dev.dev = &hspi2;
      init_param2.spi_dev.chip_select_port = SPI2_NSS_GPIO_Port;
      init_param2.spi_dev.chip_select_pin = SPI2_NSS_Pin;
//  init_param2.spi_dev.dev = &hspi1;
//init_param2.spi_dev.chip_select_port = SPI1_NSS2_GPIO_Port;
//init_param2.spi_dev.chip_select_pin = SPI1_NSS2_Pin;
//      init_param2.spi_dev.chip_select_port = SPI2_NSS_AD2_GPIO_Port;
//      init_param2.spi_dev.chip_select_pin = SPI2_NSS_AD2_Pin;
//  //    init_param2.spi_dev.dev = &hspi3;
//    init_param2.spi_dev.chip_select_port = AD4_CS_GPIO_Port;
//    init_param2.spi_dev.chip_select_pin = AD4_CS_Pin;
//    init_param2.spi_dev.dev = &hspi3;
//    init_param2.spi_dev.chip_select_port = AD3_CS_GPIO_Port;
//    init_param2.spi_dev.chip_select_pin = AD3_CS_Pin;

//    init_param2.dec_rate_int = 0x1000;//hr 0.5 kHz

//    HAL_GPIO_WritePin(init_param2.spi_dev.chip_select_port, init_param2.spi_dev.chip_select_pin, GPIO_PIN_SET);

  ads131m0x_setup(&device2, init_param2);
  InitADC(device2, false);

//      ad7779_spi_int_reg_write_mask(device2,AD7779_REG_SRC_UPDATE,AD7779_SRC_LOAD_SOURCE,AD7779_DISABLE);
//      ad7779_spi_int_reg_write_mask(device2,AD7779_REG_SRC_UPDATE,AD7779_SRC_LOAD_UPDATE,AD7779_ENABLE);
//      HAL_Delay(1);
//      ad7779_spi_int_reg_write_mask(device2,AD7779_REG_SRC_UPDATE,AD7779_SRC_LOAD_UPDATE,AD7779_DISABLE);

  devices[1]=device2;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  long data_counter=0;
  long data_counter1=0;
  long data_counter2=0;
  long data_counter3=0;
  long data_counter4=0;
  long data_counter5=0;


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      if(FREEEEG16_OUT & FREEEEG16_ADS131M08_SPI_TEST_INT)
      {
          if(flag_nDRDY_INTERRUPT)
          {
            	flag_nDRDY_INTERRUPT=false;

//        	waitForDRDYinterrupt(device1, 5000, &flag_nDRDY_INTERRUPT);
//            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
//            {

//            readData(devices[ad_adc], &DataStruct);

//                if(data_counter%4000==0)
//                if(data_counter%16000==0)
                if(data_counter%42==0)
//                    if(data_counter%84==0)
//                if(data_counter%250==0)
  //        if(test_counter%10000<5000)
              {
                  print_hex(data_counter1-data_counter2, 32);
              	data_counter2=data_counter1;
                  print_symbol(';');
                  print_hex(data_counter, 32);
                  print_symbol(';');
                  print_line();
//            print7_hex(test_counter, 32);
//            print7_symbol(';');
  //        print2_hex(test_counter, 32);
  //        print2_symbol(';');
              }
              data_counter++;
          }
          data_counter1++;
      }

      if(FREEEEG16_OUT & FREEEEG16_ADS131M08_SPI_TEST_REGISTERS_INT)
      {
          for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
          {
//          	if(0)
          	{
          	    writeSingleRegister(devices[ad_adc], CLOCK_ADDRESS, (CLOCK_DEFAULT
          	    		& ~CLOCK_OSR_MASK
          				& ~CLOCK_XTAL_DIS_MASK
          				& ~CLOCK_EXTREF_EN_MASK
          				& ~CLOCK_PWR_MASK)
          	//    		| CLOCK_OSR_4096
          	//    		| CLOCK_OSR_8192
          	    		| CLOCK_OSR_16384
          	//			| CLOCK_XTAL_DIS_DISABLED
          				| CLOCK_XTAL_DIS_ENABLED
          	//			| CLOCK_EXTREF_EN_ENABLED
          				| CLOCK_EXTREF_EN_DISABLED
          				| CLOCK_PWR_HR);

                  writeSingleRegister(devices[ad_adc], GAIN1_ADDRESS, (GAIN1_DEFAULT
                  		& ~GAIN1_PGAGAIN3_MASK
              			& ~GAIN1_PGAGAIN2_MASK
              			& ~GAIN1_PGAGAIN1_MASK
              			& ~GAIN1_PGAGAIN0_MASK)
                  		| GAIN1_PGAGAIN3_32
              			| GAIN1_PGAGAIN2_32
              			| GAIN1_PGAGAIN1_32
              			| GAIN1_PGAGAIN0_32);
                  writeSingleRegister(devices[ad_adc], GAIN2_ADDRESS, (GAIN2_DEFAULT
                  		& ~GAIN2_PGAGAIN7_MASK
              			& ~GAIN2_PGAGAIN6_MASK
              			& ~GAIN2_PGAGAIN5_MASK
              			& ~GAIN2_PGAGAIN4_MASK)
                  		| GAIN2_PGAGAIN7_32
              			| GAIN2_PGAGAIN6_32
              			| GAIN2_PGAGAIN5_32
              			| GAIN2_PGAGAIN4_32);

                  writeSingleRegister(devices[ad_adc], CH0_CFG_ADDRESS, (CH0_CFG_DEFAULT & ~CH0_CFG_MUX0_MASK) | CH0_CFG_MUX0_ADC_INPUT_SHORT);
                  writeSingleRegister(devices[ad_adc], CH1_CFG_ADDRESS, (CH1_CFG_DEFAULT & ~CH1_CFG_MUX1_MASK) | CH1_CFG_MUX1_ADC_INPUT_SHORT);
                  writeSingleRegister(devices[ad_adc], CH2_CFG_ADDRESS, (CH2_CFG_DEFAULT & ~CH2_CFG_MUX2_MASK) | CH2_CFG_MUX2_ADC_INPUT_SHORT);
                  writeSingleRegister(devices[ad_adc], CH3_CFG_ADDRESS, (CH3_CFG_DEFAULT & ~CH3_CFG_MUX3_MASK) | CH3_CFG_MUX3_ADC_INPUT_SHORT);
                  writeSingleRegister(devices[ad_adc], CH4_CFG_ADDRESS, (CH4_CFG_DEFAULT & ~CH4_CFG_MUX4_MASK) | CH4_CFG_MUX4_ADC_INPUT_SHORT);
                  writeSingleRegister(devices[ad_adc], CH5_CFG_ADDRESS, (CH5_CFG_DEFAULT & ~CH5_CFG_MUX5_MASK) | CH5_CFG_MUX5_ADC_INPUT_SHORT);
                  writeSingleRegister(devices[ad_adc], CH6_CFG_ADDRESS, (CH6_CFG_DEFAULT & ~CH6_CFG_MUX6_MASK) | CH6_CFG_MUX6_ADC_INPUT_SHORT);
                  writeSingleRegister(devices[ad_adc], CH7_CFG_ADDRESS, (CH7_CFG_DEFAULT & ~CH7_CFG_MUX7_MASK) | CH7_CFG_MUX7_ADC_INPUT_SHORT);
          	}

          	int address = readSingleRegister(devices[ad_adc], ID_ADDRESS);
            print_binary(address, 16);
            print_symbol(';');
  //          print_symbol(';');
  //          print_hex(address, 8);          print_symbol(';');          print_symbol(';');
            int status = readSingleRegister(devices[ad_adc], STATUS_ADDRESS);
            print_binary(status, 16);
            print_symbol(';');
            print_symbol(';');
  //          print_hex(status, 8);          print_symbol(';');          print_symbol(';');
            int clock = readSingleRegister(devices[ad_adc], CLOCK_ADDRESS);
            print_binary(clock, 16);
            print_symbol(';');
            print_symbol(';');
  //          print_hex(status, 8);          print_symbol(';');          print_symbol(';');
//              spiSendReceiveArrays(devices[ad_adc], 0);
//              print_binary(clock, 16);
//              print_symbol(';');
//              print_symbol(';');
//    //          print_hex(status, 8);          print_symbol(';');          print_symbol(';');
          }
            print_line();

////        	waitForDRDYinterrupt(device1, 5000, &flag_nDRDY_INTERRUPT);
//            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
//            {
//
////            readData(devices[ad_adc], &DataStruct);
//
////            if(test_counter%1000000==0)
//    //        if(test_counter%10000<5000)
////            {
//              print_hex(data_counter, 32);
//              print_symbol(';');
//  //            print7_hex(test_counter, 32);
//  //            print7_symbol(';');
//    //        print2_hex(test_counter, 32);
//    //        print2_symbol(';');
//            }
//            data_counter++;
      }

    if(FREEEEG16_OUT & FREEEEG16_ADS131M08_SPI_TEXT_UART1_INT)
//          if(readSingleRegister(devices[3], STATUS_ADDRESS)&STATUS_DRDY0_MASK==STATUS_DRDY0_NEW_DATA)
//          if(SPI_RxCplt)
        if(flag_nDRDY_INTERRUPT)
//          for(int i = 0; i < 2; i ++)
    {
          	flag_nDRDY_INTERRUPT=false;


////  		toggleRESET(devices[0]);
////		toggleRESET(devices[1]);
////		toggleRESET(devices[2]);
////		toggleRESET(devices[3]);
//
////          waitForDRDYinterrupt(device1, 5000, &flag_nDRDY_INTERRUPT);
//
////          writeSingleRegister(dev, MODE_ADDRESS, MODE_DEFAULT);
//
//
////          	SPI_RxCplt=0;
//
////              int crc_pair_ok[uint8_ad_adc_number][4];
//
//
////                  print7_hex(ad_adc1, 32);//          print2_symbol(';');
////                  print7_symbol(';');
//
////          numFrameWords = 10;
////          uint32_t spiDummyWord_out[numFrameWords];
////          // Number of words in a full ADS131M08 SPI frame
////          uint32_t spiDummyWord_in[numFrameWords] =
////          {
////          0x00000000,
////          0x00000000,
////          0x00000000,
////          0x00000000,
////          0x00000000,
////          0x00000000,
////          0x00000000,
////          0x00000000,
////          0x00000000,
////          0x00000000};
////          // Dummy word frame to write ADC during ADC data reads
//          for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
//          {
////              while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
////              {
////              }
////              if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, spiDummyWord_in, spiDummyWord_out, numFrameWords*4,5000) != HAL_OK)
////    //                if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, datas[ad_adc], datas[ad_adc], uint8_ad_chan_number*4) != HAL_OK)
////              {
////                Error_Handler();
////              }
////              while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
////              {
////              }
//
//        	  addata24s[ad_adc].b0=0;
//        	  addata24s[ad_adc].b1=0;
//        	  addata24s[ad_adc].b2=0;
//        	  addata24sw[ad_adc].b0=0;
//        	  addata24sw[ad_adc].b1=0;
//        	  addata24sw[ad_adc].b2=0;
//              for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
//            {
//                datas[ad_adc][ad_data_channel * 4 + 0]=0;
//                datas[ad_adc][ad_data_channel * 4 + 1]=0;
//                datas[ad_adc][ad_data_channel * 4 + 2]=0;
//                datas[ad_adc][ad_data_channel * 4 + 3]=0;
//
//            	  addata24s[ad_adc].datas[ad_data_channel][0]=0;
//            	  addata24s[ad_adc].datas[ad_data_channel][1]=0;
//            	  addata24s[ad_adc].datas[ad_data_channel][2]=0;
//            	  addata24sw[ad_adc].datas[ad_data_channel][0]=0;
//            	  addata24sw[ad_adc].datas[ad_data_channel][1]=0;
//            	  addata24sw[ad_adc].datas[ad_data_channel][2]=0;
//
//                  for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
//            	  {
//                	  datas[ad_adc][ad_data_channel * 4 + 0] = 0;
//                	  datas[ad_adc][ad_data_channel * 4 + 1] = 0;
//                	  datas[ad_adc][ad_data_channel * 4 + 2] = 0;
//                	  datas[ad_adc][ad_data_channel * 4 + 3] = 0;
//            	  }
//
//            }
//        	  addata24s[ad_adc].b3=0;
//        	  addata24s[ad_adc].b4=0;
//        	  addata24s[ad_adc].b5=0;
//        	  addata24s[ad_adc].b6=0;
//        	  addata24s[ad_adc].b7=0;
//        	  addata24sw[ad_adc].b3=0;
//        	  addata24sw[ad_adc].b4=0;
//        	  addata24sw[ad_adc].b5=0;
//        	  addata24sw[ad_adc].b6=0;
//        	  addata24sw[ad_adc].b7=0;
//
////              int status = readSingleRegister(devices[ad_adc], STATUS_ADDRESS);
////              print_binary(status, 16);
////              print_symbol(';');
////              print_symbol(';');
////    //          print_hex(status, 8);          print_symbol(';');          print_symbol(';');
//              spiSendReceiveArrays(&(devices[ad_adc]->spi_dev), (uint8_t*)(&addata24sw[ad_adc]), (uint8_t*)(&addata24s[ad_adc]), uint8_ad_chan_number*4);
////              spiSendReceiveArrays(&(devices[ad_adc]->spi_dev), datas[ad_adc], datas[ad_adc], uint8_ad_chan_number*4);
//
//           if (0)
//           {
//            while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
//            {
//            }
//        	if(SPI_NSS_SOFTWARE)
//        	{
//        	    setCS(&devices[ad_adc]->spi_dev, LOW);
//        	}
//            if(SPI_DMA)
//            {
//                if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, datas[ad_adc], datas[ad_adc], uint8_ad_chan_number*4) != HAL_OK)
////                if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, (uint8_t*)(&addata24sw[ad_adc]), (uint8_t*)(&addata24s[ad_adc]), uint8_ad_chan_number*4) != HAL_OK)
//                {
//                  Error_Handler();
//                }
//            } else {
////                if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, datas[ad_adc], datas[ad_adc], uint8_ad_chan_number*4,5000) != HAL_OK)
//                if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, (uint8_t*)(&addata24sw[ad_adc]), (uint8_t*)(&addata24s[ad_adc]), uint8_ad_chan_number*4,5000) != HAL_OK)
//                {
//                  Error_Handler();
//                }
//            }
//            	if(SPI_NSS_SOFTWARE)
//            	{
//            	    setCS(&devices[ad_adc]->spi_dev, HIGH);
//            	}
//            while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
//            {
//            }
//           }
//          }

              for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
              {
            	  addata24s[ad_adc].b0=0;
            	  addata24s[ad_adc].b1=0;
            	  addata24s[ad_adc].b2=0;
                  for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
                {
                	  addata24s[ad_adc].datas[ad_data_channel][0]=0;
                	  addata24s[ad_adc].datas[ad_data_channel][1]=0;
                	  addata24s[ad_adc].datas[ad_data_channel][2]=0;
                }
            	  addata24s[ad_adc].b3=0;
            	  addata24s[ad_adc].b4=0;
            	  addata24s[ad_adc].b5=0;
            	  addata24s[ad_adc].b6=0;
            	  addata24s[ad_adc].b7=0;
              	if(SPI_NSS_SOFTWARE)
              	{

                    for(int ad_adc1 = 0; ad_adc1 < uint8_ad_adc_number; ad_adc1 ++)
                    {
                    	if(devices[ad_adc1]->spi_dev.dev==devices[ad_adc]->spi_dev.dev)
                    	{
                      	    if(getCS(&devices[ad_adc1]->spi_dev)==LOW)
                      	    {
                                while (HAL_SPI_GetState(devices[ad_adc1]->spi_dev.dev) != HAL_SPI_STATE_READY)
                                {
                                }
                        	    setCS(&devices[ad_adc1]->spi_dev, HIGH);
                      	    }
                    	}
                    }

              	    setCS(&devices[ad_adc]->spi_dev, LOW);
              	}
                  while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
                  {
                  }
                  if(SPI_DMA)
                  {
      //                if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, datas[ad_adc], datas[ad_adc], uint8_ad_chan_number*4) != HAL_OK)
                      if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, (uint8_t*)(&addata24s[ad_adc]), (uint8_t*)(&addata24s[ad_adc]), uint8_ad_chan_number*4) != HAL_OK)
                      {
                        Error_Handler();
                      }
                  } else {
      //                if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, datas[ad_adc], datas[ad_adc], uint8_ad_chan_number*4,5000) != HAL_OK)
                      if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, (uint8_t*)(&addata24s[ad_adc]), (uint8_t*)(&addata24s[ad_adc]), uint8_ad_chan_number*4,5000) != HAL_OK)
                      {
                        Error_Handler();
                      }
                  }
    //              while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
    //              {
    //              }
    //              	if(SPI_NSS_SOFTWARE)
    //              	{
    //              	    setCS(&devices[ad_adc]->spi_dev, HIGH);
    //              	}
              }
          	if(SPI_NSS_SOFTWARE)
          	{
                for(int ad_adc1 = 0; ad_adc1 < uint8_ad_adc_number; ad_adc1 ++)
                {
              	    if(getCS(&devices[ad_adc1]->spi_dev)==LOW)
              	    {
                        while (HAL_SPI_GetState(devices[ad_adc1]->spi_dev.dev) != HAL_SPI_STATE_READY)
                        {
                        }
                	    setCS(&devices[ad_adc1]->spi_dev, HIGH);
              	    }
                }
          	}


//              int ad_adc = 2;
        //          for(int ad_adc = 0; ad_adc < 2; ad_adc ++)
          	print_hex(ui8SampleNumber++, 8);
          	print_symbol(';');
            for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
      {
//          readData(devices[ad_adc], &(DataStructs[ad_adc]));
//      	  datas[ad_adc][ad_data_channel * 4 + 0]=&DataStructs[ad_adc].channel0;

//                  print_binary(addata24s[ad_adc].b0, 8);//          print2_symbol(';');
//                  print_binary(addata24s[ad_adc].b1, 8);//          print2_symbol(';');
//                  print_symbol(';');
//                  print_symbol(';');

                for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
        {
//              print_hex((&(DataStructs[ad_adc].channel0))[ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
//              print_hex((&(DataStructs[ad_adc].channel0))[ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
//              print_hex((&(DataStructs[ad_adc].channel0))[ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
//              print_hex((&(DataStructs[ad_adc].channel0))[ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
//              print_binary((&DataStructs[ad_adc].channel0)[ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
//              print_binary((&DataStructs[ad_adc].channel0)[ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
//              print_binary((&DataStructs[ad_adc].channel0)[ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
//              print_binary((&DataStructs[ad_adc].channel0)[ad_data_channel * 4 + 3], 8);//          print2_symbol(';');

              	  print_hex(addata24s[ad_adc].datas[ad_data_channel][0], 8);//          print2_symbol(';');
              	  print_hex(addata24s[ad_adc].datas[ad_data_channel][1], 8);//          print2_symbol(';');
              	  print_hex(addata24s[ad_adc].datas[ad_data_channel][2], 8);//          print2_symbol(';');

//              print_binary(addata24s[ad_adc].datas[ad_data_channel][0], 8);//          print2_symbol(';');
//              print_binary(addata24s[ad_adc].datas[ad_data_channel][1], 8);//          print2_symbol(';');
//              print_binary(addata24s[ad_adc].datas[ad_data_channel][2], 8);//          print2_symbol(';');

//                      print_hex(datas[ad_adc][ad_data_channel * 4 + 0], 8);//          print2_symbol(';');
//                      print_hex(datas[ad_adc][ad_data_channel * 4 + 1], 8);//          print2_symbol(';');
//                      print_hex(datas[ad_adc][ad_data_channel * 4 + 2], 8);//          print2_symbol(';');
//                      print_hex(datas[ad_adc][ad_data_channel * 4 + 3], 8);//          print2_symbol(';');
          print_symbol(';');
        }
//                  print_symbol(';');
//                  print_binary(addata24s[ad_adc].b2, 8);//          print2_symbol(';');
//                  print_binary(addata24s[ad_adc].b3, 8);//          print2_symbol(';');
//                  print_binary(addata24s[ad_adc].b4, 8);//          print2_symbol(';');
//                  print_binary(addata24s[ad_adc].b5, 8);//          print2_symbol(';');
//                  print_binary(addata24s[ad_adc].b6, 8);//          print2_symbol(';');

        print_symbol(';');
//          print_line();
      }
      print_symbol(';');

      print_line();

    }//FREEEEG32_OUT & FREEEEG32_ADS131M08_SPI_TEXT_UART1_INT

    if(FREEEEG16_OUT & FREEEEG16_ADS131M08_SPI_SAMPLECOUNT_INT)
//          if(readSingleRegister(devices[3], STATUS_ADDRESS)&STATUS_DRDY0_MASK==STATUS_DRDY0_NEW_DATA)
//          if(SPI_RxCplt)
        if(flag_nDRDY_INTERRUPT)
//          for(int i = 0; i < 2; i ++)
    {
          	flag_nDRDY_INTERRUPT=false;

              for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
              {
            	  addata24s[ad_adc].b0=0;
            	  addata24s[ad_adc].b1=0;
            	  addata24s[ad_adc].b2=0;
                  for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
                {
                	  addata24s[ad_adc].datas[ad_data_channel][0]=0;
                	  addata24s[ad_adc].datas[ad_data_channel][1]=0;
                	  addata24s[ad_adc].datas[ad_data_channel][2]=0;
                }
            	  addata24s[ad_adc].b3=0;
            	  addata24s[ad_adc].b4=0;
            	  addata24s[ad_adc].b5=0;
            	  addata24s[ad_adc].b6=0;
            	  addata24s[ad_adc].b7=0;
              	if(SPI_NSS_SOFTWARE)
              	{

                    for(int ad_adc1 = 0; ad_adc1 < uint8_ad_adc_number; ad_adc1 ++)
                    {
                    	if(devices[ad_adc1]->spi_dev.dev==devices[ad_adc]->spi_dev.dev)
                    	{
                      	    if(getCS(&devices[ad_adc1]->spi_dev)==LOW)
                      	    {
                                while (HAL_SPI_GetState(devices[ad_adc1]->spi_dev.dev) != HAL_SPI_STATE_READY)
                                {
                                }
                        	      setCS(&devices[ad_adc1]->spi_dev, HIGH);
                      	    }
                    	}
                    }

              	    setCS(&devices[ad_adc]->spi_dev, LOW);
              	}
                  while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
                  {
                  }
                  if(SPI_DMA)
                  {
      //                if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, datas[ad_adc], datas[ad_adc], uint8_ad_chan_number*4) != HAL_OK)
                      if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, (uint8_t*)(&addata24s[ad_adc]), (uint8_t*)(&addata24s[ad_adc]), uint8_ad_chan_number*4) != HAL_OK)
                      {
                        Error_Handler();
                      }
                  } else {
      //                if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, datas[ad_adc], datas[ad_adc], uint8_ad_chan_number*4,5000) != HAL_OK)
                      if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, (uint8_t*)(&addata24s[ad_adc]), (uint8_t*)(&addata24s[ad_adc]), uint8_ad_chan_number*4,5000) != HAL_OK)
                      {
                        Error_Handler();
                      }
                  }
    //              while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
    //              {
    //              }
    //              	if(SPI_NSS_SOFTWARE)
    //              	{
    //              	    setCS(&devices[ad_adc]->spi_dev, HIGH);
    //              	}
              }
          	if(SPI_NSS_SOFTWARE)
          	{
                for(int ad_adc1 = 0; ad_adc1 < uint8_ad_adc_number; ad_adc1 ++)
                {
              	    if(getCS(&devices[ad_adc1]->spi_dev)==LOW)
              	    {
                        while (HAL_SPI_GetState(devices[ad_adc1]->spi_dev.dev) != HAL_SPI_STATE_READY)
                        {
                        }
                	      setCS(&devices[ad_adc1]->spi_dev, HIGH);
              	    }
                }
          	}

          	        ui32SampleNumber++;
          	        if(ui32SampleNumber%250==0)
          	        {
          	        	                      print_hex(ui32SampleNumber/250, 16);//          print2_symbol(';');
          	        	            print_symbol(';');
          	        }

    }//FREEEEG32_OUT & FREEEEG32_ADS131M08_SPI_SAMPLECOUNT_INT

    if(FREEEEG16_OUT & FREEEEG16_ADS131M08_SPI_OPENVIBE_FREEEEG16_CUSTOM_INT)
//            if(0)//openbci
  	  //          if(SPI_RxCplt)
  	            if(flag_nDRDY_INTERRUPT)
//    	                if((readSingleRegister(devices[3], STATUS_ADDRESS)&STATUS_DRDY0_MASK)==STATUS_DRDY0_NEW_DATA)
    {
  	            	flag_nDRDY_INTERRUPT=false;



//          waitForDRDYinterrupt(device1, 5000, &flag_nDRDY_INTERRUPT);
//    	  {
//    		  uint32_t timeout_ms=5000;
//    		    // Convert ms to a # of loop iterations, OR even better use a timer here...
//    		    uint32_t timeout = timeout_ms * 6000;   // convert to # of loop iterations
//
//    		    // Reset interrupt flag
//    		    flag_nDRDY_INTERRUPT = false;
//
//    		    // Enable interrupts
//    		//    IntMasterEnable();
//
//    		    // Wait for nDRDY interrupt or timeout - each iteration is about 20 ticks
//    		    do {
//    		        timeout--;
//    		    } while (!flag_nDRDY_INTERRUPT && (timeout > 0));
//
//    		    // Reset interrupt flag
//    		    flag_nDRDY_INTERRUPT = false;
//    	  }
//          	SPI_RxCplt=0;

        for(int ad_adc = 0; ad_adc < uint8_ad_adc_number; ad_adc ++)
        {
      	  addata24s[ad_adc].b0=0;
      	  addata24s[ad_adc].b1=0;
      	  addata24s[ad_adc].b2=0;
            for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
          {
          	  addata24s[ad_adc].datas[ad_data_channel][0]=0;
          	  addata24s[ad_adc].datas[ad_data_channel][1]=0;
          	  addata24s[ad_adc].datas[ad_data_channel][2]=0;
          }
      	  addata24s[ad_adc].b3=0;
      	  addata24s[ad_adc].b4=0;
      	  addata24s[ad_adc].b5=0;
      	  addata24s[ad_adc].b6=0;
      	  addata24s[ad_adc].b7=0;
        	if(SPI_NSS_SOFTWARE)
        	{

              for(int ad_adc1 = 0; ad_adc1 < uint8_ad_adc_number; ad_adc1 ++)
              {
              	if(devices[ad_adc1]->spi_dev.dev==devices[ad_adc]->spi_dev.dev)
              	{
                	    if(getCS(&devices[ad_adc1]->spi_dev)==LOW)
                	    {
                          while (HAL_SPI_GetState(devices[ad_adc1]->spi_dev.dev) != HAL_SPI_STATE_READY)
                          {
                          }
                  	    setCS(&devices[ad_adc1]->spi_dev, HIGH);
                	    }
              	}
              }

        	    setCS(&devices[ad_adc]->spi_dev, LOW);
        	}
            while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
            {
            }
            if(SPI_DMA)
            {
//                if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, datas[ad_adc], datas[ad_adc], uint8_ad_chan_number*4) != HAL_OK)
                if(HAL_SPI_TransmitReceive_DMA(devices[ad_adc]->spi_dev.dev, (uint8_t*)(&addata24s[ad_adc]), (uint8_t*)(&addata24s[ad_adc]), uint8_ad_chan_number*4) != HAL_OK)
                {
                  Error_Handler();
                }
            } else {
//                if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, datas[ad_adc], datas[ad_adc], uint8_ad_chan_number*4,5000) != HAL_OK)
                if(HAL_SPI_TransmitReceive(devices[ad_adc]->spi_dev.dev, (uint8_t*)(&addata24s[ad_adc]), (uint8_t*)(&addata24s[ad_adc]), uint8_ad_chan_number*4,5000) != HAL_OK)
                {
                  Error_Handler();
                }
            }
//              while (HAL_SPI_GetState(devices[ad_adc]->spi_dev.dev) != HAL_SPI_STATE_READY)
//              {
//              }
//              	if(SPI_NSS_SOFTWARE)
//              	{
//              	    setCS(&devices[ad_adc]->spi_dev, HIGH);
//              	}
        }
    	if(SPI_NSS_SOFTWARE)
    	{
          for(int ad_adc1 = 0; ad_adc1 < uint8_ad_adc_number; ad_adc1 ++)
          {
        	    if(getCS(&devices[ad_adc1]->spi_dev)==LOW)
        	    {
                  while (HAL_SPI_GetState(devices[ad_adc1]->spi_dev.dev) != HAL_SPI_STATE_READY)
                  {
                  }
          	    setCS(&devices[ad_adc1]->spi_dev, HIGH);
        	    }
          }
    	}

//        ui32SampleNumber++;
      if(1)
//        if(ui32SampleNumber%250)
//            for(int i = 0; i < 2; i ++)
      {
//            const uint32_t uint8_data_number = 2 + 1 * 3 * 1 + 0 * 2 + 1;
//              const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * 4 + uint8_accel_chan_number * 2 + 1;

//    	  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
//    	  {
//    		  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
//    		  {
//    		  }
//    	  }
    	  if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
    	  {
    		  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
    		  {
    		  }
    	  }
          const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * uint8_ad_adc_data_bytes_number * (uint8_ad_adc_number) + uint8_accel_chan_number * 2 + 1;
//            const uint32_t uint8_data_number = 2 + uint8_ad_chan_number * 3 * (uint8_ad_adc_number) + uint8_accel_chan_number * 2 + 1;
//          uint8_t dataBuffer[uint8_data_number];

        dataBuffer_print[0] = 0xA0;
        dataBuffer_print[1] = ui8SampleNumber++;

        for(int ad_adc = 0; ad_adc < (uint8_ad_adc_number); ad_adc ++)
        {
//              readData(devices[ad_adc], &DataStructs[ad_adc]);
//          	  datas[ad_adc][ad_data_channel * 4 + 0]=&DataStructs[ad_adc].channel0;
          for(int ad_data_channel = 0; ad_data_channel < uint8_ad_chan_number; ad_data_channel ++)
          {
          	if(uint8_ad_adc_data_bytes_number==3)
          	{
                  dataBuffer_print[2 + uint8_ad_chan_number * uint8_ad_adc_data_bytes_number * ad_adc + ad_data_channel * uint8_ad_adc_data_bytes_number + 0] = addata24s[ad_adc].datas[ad_data_channel][0];
                  dataBuffer_print[2 + uint8_ad_chan_number * uint8_ad_adc_data_bytes_number * ad_adc + ad_data_channel * uint8_ad_adc_data_bytes_number + 1] = addata24s[ad_adc].datas[ad_data_channel][1];
                  dataBuffer_print[2 + uint8_ad_chan_number * uint8_ad_adc_data_bytes_number * ad_adc + ad_data_channel * uint8_ad_adc_data_bytes_number + 2] = addata24s[ad_adc].datas[ad_data_channel][2];
          	}
          	else
          	{
                  dataBuffer_print[2 + uint8_ad_chan_number * uint8_ad_adc_data_bytes_number * ad_adc + ad_data_channel * uint8_ad_adc_data_bytes_number + 0] = addata24s[ad_adc].datas[ad_data_channel][1];
                  dataBuffer_print[2 + uint8_ad_chan_number * uint8_ad_adc_data_bytes_number * ad_adc + ad_data_channel * uint8_ad_adc_data_bytes_number + 1] = addata24s[ad_adc].datas[ad_data_channel][2];
          	}
//                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 0] = (&DataStructs[ad_adc].channel0)[ad_data_channel * 4 + 1];
//                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 1] = (&DataStructs[ad_adc].channel0)[ad_data_channel * 4 + 2];
//                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 2] = (&DataStructs[ad_adc].channel0)[ad_data_channel * 4 + 3];
//                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 0] = datas[ad_adc][ad_data_channel * 4 + 1];
//                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 1] = datas[ad_adc][ad_data_channel * 4 + 2];
//                dataBuffer[2 + uint8_ad_chan_number * 3 * ad_adc + ad_data_channel * 3 + 2] = datas[ad_adc][ad_data_channel * 4 + 3];
          }
        }
        for(int accel_data_channel = 0; accel_data_channel < 0; accel_data_channel ++)
//                for(int accel_data_channel = 0; accel_data_channel < uint8_accel_chan_number; accel_data_channel ++)
        {
          dataBuffer_print[2 + uint8_ad_chan_number * uint8_ad_adc_data_bytes_number * (uint8_ad_adc_number) + accel_data_channel * 2 + 0] = 0;
          dataBuffer_print[2 + uint8_ad_chan_number * uint8_ad_adc_data_bytes_number * (uint8_ad_adc_number) + accel_data_channel * 2 + 1] = 0;
        }
//          dataBuffer_print[2 + 1 * 3 * 1 + 0 * 2 + 0] = 0xC0;
//            dataBuffer[2 + uint8_ad_chan_number * 3 * 4 + uint8_accel_chan_number * 2 + 0] = 0xC0;
          dataBuffer_print[2 + uint8_ad_chan_number * uint8_ad_adc_data_bytes_number * (uint8_ad_adc_number) + uint8_accel_chan_number * 2 + 0] = 0xC0;

//          if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART1)
//          {
//              if(UART_DMA)
//              {
//                  if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dataBuffer_print, uint8_data_number) != HAL_OK)
//                  {
//                    Error_Handler();
//                  }
//              } else
//              {
//                  if(HAL_UART_Transmit(&huart1, (uint8_t*)dataBuffer_print, uint8_data_number,5000) != HAL_OK)
//                  {
//                    Error_Handler();
//                  }
//              }
//          }
          if(FREESMARTEEG_SEND & FREESMARTEEG_SEND_UART2)
          {
              if(UART_DMA)
              {
                  if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dataBuffer_print, uint8_data_number) != HAL_OK)
                  {
                    Error_Handler();
                  }
              } else
              {
                  if(HAL_UART_Transmit(&huart2, (uint8_t*)dataBuffer_print, uint8_data_number,5000) != HAL_OK)
                  {
                    Error_Handler();
                  }
              }
          }
      }


    }//FREEEEG32_OUT & FREEEEG32_ADS131M08_SPI_OPENVIBE_FREEEEG32_CUSTOM_INT

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADC1_SYNC_RESET_GPIO_Port, ADC1_SYNC_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC1_SYNC_RESET_Pin */
  GPIO_InitStruct.Pin = ADC1_SYNC_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADC1_SYNC_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC1_DRDY_Pin */
  GPIO_InitStruct.Pin = ADC1_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC1_DRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_NSS_Pin */
  GPIO_InitStruct.Pin = SPI2_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI2_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
//  if(GPIO_Pin==10)
  if(FREESMARTEEG_ADC & FREESMARTEEG_ADC_ADS131M08_SPI_READ_INT)
  {
	  flag_nDRDY_INTERRUPT = true;
  }

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
