/*
 * Project: W5500 UDP Communication with STM32
 * Author : Sooraj R
 * Date   : 10/25/2024
 *
 * Description:
 * This program demonstrates setting up UDP communication on an STM32
 * microcontroller using the W5500 Ethernet module. The STM32 initializes the W5500,
 * opens a UDP socket, and transmits and receives messages via UDP.
 * Debugging messages are sent via UART for easy monitoring.
 *
 * Requirements:
 * - STM32CubeIDE (for building and flashing)
 * - W5500 Module (SPI connection to STM32)
 * - UART-to-USB converter (optional for UART debugging)
 *
 * Connections:
 * - SPI (STM32 to W5500):
 *     - SPI_MOSI (PA7) <--> MOSI
 *     - SPI_MISO (PA6) <--> MISO
 *     - SPI_SCK  (PA5) <--> SCLK
 *     - SPI_CS   (PA4) <--> SCS
 * - Other W5500 Pins:
 *     - 3v3 <--> VCC (W5500)
 *     - GND <--> GND (W5500)
 * - UART (for Debugging):
 *     - TX (PA2) <--> RX on UART-to-USB converter (PC)
 *
 * Library Link: https://github.com/Wiznet/ioLibrary_Driver.git
 * W5500 Module: https://robu.in/product/w5500-tcp-ip-spi-to-lan-ethernet-interface-spi-to-lan-ethernet-converter/?gad_source=1&gclid=CjwKCAjwg-24BhB_EiwA1ZOx8gmG97EPR4Eeb_cDfobfCvqw8VmkLKJ5izCyKA48QGf5VNaT3iyzrRoCoS0QAvD_BwE
 */



#include "main.h"

#include "w5500.h"
#include "wizchip_conf.h"
#include "socket.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

void uprintf(const char *format, ...) {
    char buffer[256];  // Adjust size as needed for your message
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}


void cs_sel() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //CS LOW
}

void cs_desel() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //CS HIGH
}

uint8_t spi_rb(void) {
	uint8_t rbuf;
	HAL_SPI_Receive(&hspi1, &rbuf, 1, 0xFFFFFFFF);
	return rbuf;
}

void spi_wb(uint8_t b) {
	HAL_SPI_Transmit(&hspi1, &b, 1, 0xFFFFFFFF);
}

unsigned long UDPsendMillis;

int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  reg_wizchip_cs_cbfunc(cs_sel, cs_desel);
  reg_wizchip_spi_cbfunc(spi_rb, spi_wb);

  uprintf("Setup Starting..\r\n");

  uint8_t bufSize[] = {2, 2, 2, 2};
  wizchip_init(bufSize, bufSize);
  wiz_NetInfo netInfo = { .mac 	= {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef},	// Mac address
                           .ip 	= {192, 168, 56, 5},					// IP address
                           .sn 	= {255, 255, 255, 0}};					// Subnet mask
  wizchip_setnetinfo(&netInfo);
  wizchip_getnetinfo(&netInfo);
  uprintf("Setup Complete\r\n");


  int16_t retVal;
  uint8_t socketStatus = 0;
  retVal = socket(0,Sn_MR_UDP, 10050, 0);
  uprintf("Socket Output: %d\r\n",retVal);
  if(retVal == 0){
  	socketStatus=1;
  	uprintf("Socket Creation Success \r\n");
  }
  else{
  	socketStatus=0;
  	uprintf("Socket Creation failed \r\n");
    }


  while (1)
  {

	  uint8_t rcvBuf[128];
	  uint8_t remoteIP[4];
	  uint16_t remotePort;
	  uint16_t receivedDataSize;
	  getsockopt(0, SO_RECVBUF, &receivedDataSize);
	  if(receivedDataSize>0){
	  	int16_t  rcvLen = recvfrom(0,rcvBuf, sizeof(rcvBuf),remoteIP,&remotePort);
	  	if(rcvLen>0){
	  		uprintf("Got some Data: %d \r\n",rcvLen);
	  		rcvBuf[rcvLen] = '\0';
	  		uprintf("Received message: %s from %d.%d.%d.%d:%d\r\n",rcvBuf, remoteIP[0], remoteIP[1], remoteIP[2], remoteIP[3], remotePort);
	  	}
	  }



	  if(HAL_GetTick()-UDPsendMillis>=1000){
	  	UDPsendMillis=HAL_GetTick();
	  	if(socketStatus){
	  		uint8_t destIP[4] = {192,168,56,22};
	  		uint16_t destPORT = 10051;
	  		char udpMSG[] = "This is a message for stm32.. Happy Coding..";
	  		int16_t ret = sendto(0,(uint8_t*)udpMSG, strlen(udpMSG), destIP, destPORT);
	  		uprintf("Send message length: %d \r\n",ret);
	  		if(ret<0){
	  			uprintf("Error sending UDP Message\r\n");
	  		}
	  		else{
	  			uprintf("UDP message has been send\r\n");
	  		}
	  	}
	  }

  }

}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
