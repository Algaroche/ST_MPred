/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "usb_device.h"
#include "usbd_cdc_if.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
EXTI_HandleTypeDef iis3dwb_exti;
static void IIS3DWB_Int_Callback(void);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

uint8_t SPI3InitCounter = 0;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);

int32_t MX_SPI3_Init_WRAP(void);
int32_t MX_SPI3_DeInit_WRAP(void);
int32_t leer_registro(uint16_t Address, uint16_t Reg, uint8_t * pData, uint16_t len);
int32_t escribir_registro(uint16_t Address, uint16_t Reg, uint8_t * pData, uint16_t len);

void InicializaRegistro(uint16_t tamano_buffer, int16_t *Trama);
void InicializaSensores(void);
void TomaMedidas(uint16_t Inicio, int16_t *Trama);
void CreaTrama(uint16_t Inicio, int16_t *Trama);
//void EnviaTrama(uint8_t registro[60]);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

IIS2MDC_Object_t magneto_sensor;
IIS2MDC_IO_t magneto_IO;
IIS2MDC_Axes_t magneto_axes;

LPS22HH_Object_t press_sensor;
LPS22HH_IO_t press_IO;


IIS3DWB_Object_t vibro_sensor;
IIS3DWB_IO_t vibro_IO;
IIS3DWB_Axes_t vibro_axes;

typedef enum estados {Inicializacion, Midiendo, Empaquetando, EnviandoTrama} Estados;
uint8_t len = 3;//18;//

/* USER CODE END 0 */

volatile uint8_t check = 0;
uint8_t fifo_buffer[3000];
/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	Estados estado_actual=Inicializacion;
	//AJUSTAR EL TIPO Y EL TAMAÑO DEL BUFFER A LAS NECESIDADES
	int16_t buffer[100];
	uint16_t posicion_inicio=0;
	uint8_t numero_medidas=0;
	uint8_t max_medidas = sizeof(buffer)/len/sizeof(buffer[0]);
	//uint16_t i=0;

	/* MCU Configuration--------------------------------------------------------*/
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	/* Configure the system clock */
	SystemClock_Config();
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USB_DEVICE_Init();
	MX_SPI3_Init();

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		switch (estado_actual)
		{
		case Inicializacion:
			InicializaRegistro(sizeof(buffer)/sizeof(buffer[0]), &buffer);
			InicializaSensores();
			estado_actual = Midiendo;
			break;

			case Midiendo:
				TomaMedidas(posicion_inicio, &buffer);
				numero_medidas += 1;
				estado_actual = Empaquetando;
				break;

			case Empaquetando:
				//CreaTrama(posicion_inicio,&buffer);
				if (numero_medidas < max_medidas)
				{
					posicion_inicio += len;
					estado_actual = Midiendo;
					break;
				}
				else
				{
					estado_actual = EnviandoTrama;
					break;
				}

			case EnviandoTrama:
				CDC_Transmit_FS(&buffer, sizeof(buffer));
				//EnviaTrama(buffer);
				numero_medidas = 0;
				posicion_inicio = 0;
				estado_actual = Midiendo;
				break;
		}
	}
}

//PONE A CERO TODOS LOS VALORES DEL REGISTRO
void InicializaRegistro(uint16_t tamano_buffer, int16_t *Trama)
{
	uint16_t i=0;
	for (i=0; i<(tamano_buffer);i++)
		Trama[i]=0;
}

//INICIALIZA LOS SENSORES INDICADOS
void InicializaSensores(void)
{
	//INICIALIZA EL SENSOR DE PRESION Y TEMPERATURA

	press_IO.Address		= 0xba;		//1011101b b=0 leer, b=1 escribir
	press_IO.BusType		= 0;		//0 si I2C
	press_IO.DeInit			= BSP_I2C2_DeInit;
	press_IO.GetTick		= BSP_GetTick;
	press_IO.Init			= BSP_I2C2_Init;
	press_IO.ReadReg		= BSP_I2C2_ReadReg;
	press_IO.WriteReg		= BSP_I2C2_WriteReg;

	LPS22HH_RegisterBusIO(&press_sensor , &press_IO);
	LPS22HH_Init(&press_sensor);
	LPS22HH_PRESS_Enable(&press_sensor);
	LPS22HH_TEMP_Enable(&press_sensor);

	//INICIALIZA EL MAGNETOMETRO

	magneto_IO.Address 	= 0x3c;		//0011110b b=0 leer, b=1 escribir
	magneto_IO.BusType 	= 0; 		//0 si I2C
	magneto_IO.DeInit 	= BSP_I2C2_DeInit;
	magneto_IO.GetTick 	= BSP_GetTick;
	magneto_IO.Init 	= BSP_I2C2_Init;
	magneto_IO.ReadReg 	= BSP_I2C2_ReadReg;
	magneto_IO.WriteReg = BSP_I2C2_WriteReg;

	IIS2MDC_RegisterBusIO(&magneto_sensor, &magneto_IO);
	IIS2MDC_Init(&magneto_sensor);
	IIS2MDC_MAG_Enable(&magneto_sensor);
	IIS2MDC_MAG_GetAxes(&magneto_sensor, &magneto_axes);

	//INICIALIZA EL VIBROMETRO

	vibro_IO.Address	= 0;
	vibro_IO.BusType	= 2;	//SPI de 3 cables
	vibro_IO.DeInit		= MX_SPI3_DeInit_WRAP;
	vibro_IO.GetTick	= BSP_GetTick;
	vibro_IO.Init		= MX_SPI3_Init_WRAP;
	vibro_IO.ReadReg	= leer_registro;
	vibro_IO.WriteReg	= escribir_registro;

	IIS3DWB_RegisterBusIO(&vibro_sensor, &vibro_IO);
	IIS3DWB_Init(&vibro_sensor);

	float maxODR =26700.0f;// 0.0f;//
	float sensitivity;
	IIS3DWB_ACC_SetOutputDataRate(&vibro_sensor, maxODR);		// Pongo el ODR al máximo 26.7KHz
	IIS3DWB_ACC_SetFullScale(&vibro_sensor, 2);					//Pongo el fullscale range de la aceleración en 2g
	IIS3DWB_ACC_GetSensitivity(&vibro_sensor, &sensitivity);

	//ACTIVA EL MODO FIFO DEL VIBROMETRO

	//	IIS3DWB_FIFO_Set_Mode(&vibro_sensor, IIS3DWB_FIFO_MODE);
	//	IIS3DWB_INT1_Set_FIFO_Full(&vibro_sensor, IIS3DWB_FIFO_STATUS2);

	IIS3DWB_ACC_Enable(&vibro_sensor);

}

//TOMA LAS MEDIDAS DE LOS SENSORES INDICADOS Y LAS GUARDA EN EL BUFFER QUE SE LE PASA COMO ENTRADA
void TomaMedidas(uint16_t Inicio, int16_t *Trama)
{
	//MEDIDAS DEL SENSOR DE PRESION (INT24) Y TEMPERATURA (INT16)

	float press_value;
	LPS22HH_PRESS_GetPressure(&press_sensor, &press_value);
//		Trama[Inicio+1] = ((uint32_t)(press_value));
//		Trama[Inicio+2] = ((uint32_t)(press_value))>>16;
	float temp_value;
	LPS22HH_TEMP_GetTemperature(&press_sensor, &temp_value);
//		Trama[Inicio+3] = ((uint16_t)(temp_value*100));

	//MEDIDAS DEL MAGNETOMETRO (INT16)

	IIS2MDC_MAG_GetAxes(&magneto_sensor, &magneto_axes);
//		Trama[Inicio+4] = ((magneto_axes.x));
//		Trama[Inicio+5] = ((magneto_axes.y));
//		Trama[Inicio+6] = ((magneto_axes.z));

	//MEDIDAS DEL VIBROMETRO (INT16)

	IIS3DWB_ACC_GetAxes(&vibro_sensor, &vibro_axes);
//		Trama[Inicio+7] = ((uint16_t)(vibro_axes.x));
//		Trama[Inicio+8] = ((uint16_t)(vibro_axes.y));
//		Trama[Inicio+9] = ((uint16_t)(vibro_axes.z));

		Trama[Inicio+0] = ((uint16_t)(vibro_axes.x));
		Trama[Inicio+1] = ((uint16_t)(vibro_axes.y));
		Trama[Inicio+2] = ((uint16_t)(vibro_axes.z));

	//SI SE ACTIVA EL MODO FIFO ESPERA LA INTERRUPCION GENERADA AL LLENARSE LA FIFO PARA LEERLA

	//	while (check == 0){
	//			//SE ESPERA A LA INTERRUPCIÓN DE FIFO FULL
	//
	//		}
	//		IIS3DWB_FIFO_Read(&vibro_sensor, fifo_buffer, 3000);

}

//AÑADE UNA CABECERA Y UN CRC A CADA GRUPO DE MEDIDAS
void CreaTrama(uint16_t Inicio, int16_t *Trama)
{
	Trama[Inicio] = 0x3e;
	unsigned int BCC_CRC = 0;
	for (int i = Inicio; i < (Inicio+len-1); i++)
		BCC_CRC ^= Trama[i];
	Trama[Inicio+len-1]=BCC_CRC;
}

//NO FUNCIONA BIEN, MEJOR HACER EL ENVIO DESDE EL MAIN
//void EnviaTrama(uint8_t registro[60])
//{
//
//	CDC_Transmit_FS(&registro, sizeof(registro));
//
//}

//FUNCIONES PARA CONFIGURAR COMUNICACION POR SPI DE SENSORES
int32_t leer_registro(uint16_t Address, uint16_t Reg, uint8_t * pData, uint16_t len)
{
	uint8_t autoInc = 0x00;

	Reg = Reg | 0x80 | autoInc;

	HAL_GPIO_WritePin(GPIOF  , GPIO_PIN_5 , GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &Reg, 1, 1000);
	HAL_SPI_Receive(&hspi3, pData, len, 1000);
	HAL_GPIO_WritePin(GPIOF  , GPIO_PIN_5 , GPIO_PIN_SET);
	return 0;

	//	HAL_StatusTypeDef Error_Code;
	//	int32_t ret = 0;
	//
	//	//Pongo a cero el ChipSelect del sensor correspondiente
	//	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5,GPIO_PIN_RESET);
	//	//Mando un mensaje indicando el registro que quiero leer
	//	if ((Error_Code = HAL_SPI_Transmit(&hspi3, &Reg, 2, 10000)) != HAL_OK)
	//		ret = Error_Code;
	//	//Recibo datos del registro indicado
	//	if ((Error_Code = HAL_SPI_Receive(&hspi3, &pData, len, 10000)) != HAL_OK)
	//		ret = Error_Code;
	//	//Pongo a uno el ChipSelect del sensor correspondiente
	//	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5,GPIO_PIN_SET);
	//
	//	return ret;
}

int32_t escribir_registro(uint16_t Address, uint16_t Reg, uint8_t * pData, uint16_t len)
{

	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5 , GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &Reg, 1, 1000);
	HAL_SPI_Transmit(&hspi3, pData, len, 1000);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5 , GPIO_PIN_SET);

	return 0;

	//	HAL_StatusTypeDef Error_Code;
	//	int32_t ret = 0;
	//
	//	//Pongo a cero el ChipSelect del sensor correspondiente
	//	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5,GPIO_PIN_RESET);
	//	//Mando un mensaje indicando el registro en el que quiero escribir
	//	if ((Error_Code = HAL_SPI_Transmit(&hspi3, &Reg, 2, 10000)) != HAL_OK)
	//		ret = Error_Code;
	//	//Escribo los datos en el registro indicado
	//	if ((Error_Code = HAL_SPI_Transmit(&hspi3, &pData, len, 10000)) != HAL_OK)
	//		ret = Error_Code;
	//	//Pongo a uno el ChipSelect del sensor correspondiente
	//	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5,GPIO_PIN_SET);
	//
	//	return ret;
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 2;
	RCC_OscInitStruct.PLL.PLLN = 30;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_USB;
	PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

	/* USER CODE BEGIN SPI3_Init 0 */

	/* USER CODE END SPI3_Init 0 */

	/* USER CODE BEGIN SPI3_Init 1 */

	/* USER CODE END SPI3_Init 1 */
	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 7;
	hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI3_Init 2 */

	/* USER CODE END SPI3_Init 2 */

}

//Encapsula la función de inicialización del SPI3 para poder pasarsela al inicializdor del sensor que vaya por SPI

int32_t MX_SPI3_Init_WRAP(void)
{
	int32_t ret = BSP_ERROR_NONE;
	MX_SPI3_Init();
	return ret;

}


int32_t MX_SPI3_DeInit_WRAP(void)
{
	int32_t ret = BSP_ERROR_NONE;

	if (SPI3InitCounter > 0)
	{
		if (--SPI3InitCounter == 0)
		{
			/* DeInit the I2C */
			if (HAL_SPI_DeInit(&hspi3) != HAL_OK)
			{
				ret = BSP_ERROR_BUS_FAILURE;
			}
		}
	}
	return ret;
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
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	HAL_PWREx_EnableVddIO2();
	__HAL_RCC_GPIOF_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, LED1_Pin|DCDC_2_EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, LED2_Pin|WIFI_WAKEUP_Pin|CS_DH_Pin|EX_RESET_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(BLE_RST_GPIO_Port, BLE_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(WIFI_RST_GPIO_Port, WIFI_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOF, CS_WIFI_Pin|C_EN_Pin|CS_ADWB_Pin|STSAFE_RESET_Pin
			|WIFI_BOOT0_Pin|CS_DHC_Pin|SEL3_4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG, BLE_SPI_CS_Pin|SEL1_2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, SPI2_MOSI_p2_Pin|PB11_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : BOOT0_PE0_Pin BLE_TEST8_Pin */
	GPIO_InitStruct.Pin = BOOT0_PE0_Pin|BLE_TEST8_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : PB9_Pin PB8_Pin PB14_Pin CHRGB0_Pin */
	GPIO_InitStruct.Pin = PB9_Pin|PB8_Pin|PB14_Pin|CHRGB0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : BOOT0_PE0H3_Pin */
	GPIO_InitStruct.Pin = BOOT0_PE0H3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BOOT0_PE0H3_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SDMMC_D3_Pin SDMMC_D2_Pin SDMMC_D1_Pin SDMMC_CK_Pin
                           SDMMC_D0_Pin */
	GPIO_InitStruct.Pin = SDMMC_D3_Pin|SDMMC_D2_Pin|SDMMC_D1_Pin|SDMMC_CK_Pin
			|SDMMC_D0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : BLE_TEST9_Pin WIFI_DRDY_Pin INT1_DHC_Pin INT_STT_Pin
                           INT1_ADWB_Pin */
	GPIO_InitStruct.Pin = BLE_TEST9_Pin|WIFI_DRDY_Pin|INT1_DHC_Pin|INT_STT_Pin
			|INT1_ADWB_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : USART2_RX_Pin USART2_RTS_Pin USART2_TX_Pin */
	GPIO_InitStruct.Pin = USART2_RX_Pin|USART2_RTS_Pin|USART2_TX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI2_CLK_Pin */
	GPIO_InitStruct.Pin = SPI2_CLK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(SPI2_CLK_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : EX_PWM_Pin */
	GPIO_InitStruct.Pin = EX_PWM_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(EX_PWM_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SAI1_SCK_A_Pin SAI1_MCLK_A_Pin SAI1_FS_A_DFSDM_D3_Pin SAI1_SD_A_Pin
                           SAI1_SD_B_Pin */
	GPIO_InitStruct.Pin = SAI1_SCK_A_Pin|SAI1_MCLK_A_Pin|SAI1_FS_A_DFSDM_D3_Pin|SAI1_SD_A_Pin
			|SAI1_SD_B_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : LED1_Pin DCDC_2_EN_Pin */
	GPIO_InitStruct.Pin = LED1_Pin|DCDC_2_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : LED2_Pin WIFI_WAKEUP_Pin CS_DH_Pin EX_RESET_Pin */
	GPIO_InitStruct.Pin = LED2_Pin|WIFI_WAKEUP_Pin|CS_DH_Pin|EX_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : PA10_Pin PA9_Pin PA0_Pin DAC1_OUT1_Pin
                           PA1_Pin */
	GPIO_InitStruct.Pin = PA10_Pin|PA9_Pin|PA0_Pin|DAC1_OUT1_Pin
			|PA1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : DFSDM1_DATIN5_Pin DFSDM1_D7_Pin */
	GPIO_InitStruct.Pin = DFSDM1_DATIN5_Pin|DFSDM1_D7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PG12_Pin PG10_Pin PG9_Pin */
	GPIO_InitStruct.Pin = PG12_Pin|PG10_Pin|PG9_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pin : SDMMC_CMD_Pin */
	GPIO_InitStruct.Pin = SDMMC_CMD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
	HAL_GPIO_Init(SDMMC_CMD_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : BLE_RST_Pin */
	GPIO_InitStruct.Pin = BLE_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BLE_RST_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : WIFI_RST_Pin */
	GPIO_InitStruct.Pin = WIFI_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(WIFI_RST_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : CS_WIFI_Pin C_EN_Pin CS_ADWB_Pin STSAFE_RESET_Pin
                           WIFI_BOOT0_Pin CS_DHC_Pin SEL3_4_Pin */
	GPIO_InitStruct.Pin = CS_WIFI_Pin|C_EN_Pin|CS_ADWB_Pin|STSAFE_RESET_Pin
			|WIFI_BOOT0_Pin|CS_DHC_Pin|SEL3_4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : I2C3_SDA_Pin I2C3_SCL_Pin */
	GPIO_InitStruct.Pin = I2C3_SDA_Pin|I2C3_SCL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pin : SW_SEL_Pin */
	GPIO_InitStruct.Pin = SW_SEL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
	HAL_GPIO_Init(SW_SEL_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : INT2_DHC_Pin PGOOD_Pin INT_M_Pin */
	GPIO_InitStruct.Pin = INT2_DHC_Pin|PGOOD_Pin|INT_M_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI1_MISO_Pin SPI1_MOSI_Pin SPI1_CLK_Pin */
	GPIO_InitStruct.Pin = SPI1_MISO_Pin|SPI1_MOSI_Pin|SPI1_CLK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : BLE_SPI_CS_Pin SEL1_2_Pin */
	GPIO_InitStruct.Pin = BLE_SPI_CS_Pin|SEL1_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : INT_HTS_Pin BLE_INT_Pin */
	GPIO_InitStruct.Pin = INT_HTS_Pin|BLE_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : I2C4_SCL_Pin I2C4_SDA_Pin */
	GPIO_InitStruct.Pin = I2C4_SCL_Pin|I2C4_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : ADC1_IN1_Pin ADC1_IN2_Pin uC_ADC_BATT_Pin */
	GPIO_InitStruct.Pin = ADC1_IN1_Pin|ADC1_IN2_Pin|uC_ADC_BATT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI2_MISO_Pin SPI2_MOSI_Pin */
	GPIO_InitStruct.Pin = SPI2_MISO_Pin|SPI2_MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : INT2_ADWB_Pin SD_DETECT_Pin */
	GPIO_InitStruct.Pin = INT2_ADWB_Pin|SD_DETECT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : CHRG_Pin */
	GPIO_InitStruct.Pin = CHRG_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(CHRG_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : BUTTON_PWR_Pin */
	GPIO_InitStruct.Pin = BUTTON_PWR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BUTTON_PWR_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : USART3_RX_Pin USART3_TX_Pin */
	GPIO_InitStruct.Pin = USART3_RX_Pin|USART3_TX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : USART3_RTS_Pin USART3_CTS_Pin */
	GPIO_InitStruct.Pin = USART3_RTS_Pin|USART3_CTS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : DFSDM1_CKOUT_Pin */
	GPIO_InitStruct.Pin = DFSDM1_CKOUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
	HAL_GPIO_Init(DFSDM1_CKOUT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI2_MOSI_p2_Pin PB11_Pin */
	GPIO_InitStruct.Pin = SPI2_MOSI_p2_Pin|PB11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : INT2_DH_Pin */
	GPIO_InitStruct.Pin = INT2_DH_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(INT2_DH_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : EX_ADC_Pin */
	GPIO_InitStruct.Pin = EX_ADC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(EX_ADC_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PE12_Pin */
	GPIO_InitStruct.Pin = PE12_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PE12_GPIO_Port, &GPIO_InitStruct);

	//////////////////////////////////////////////////////////////////////////////////////
	////////////CONFIGURACION DE LOS PINES E INTERRUPCIONES DEL VIBROMETRO///////////////

	/*Configure GPIO pin : IIS3DWB_SPI_CS_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_SET);

	/*Configure GPIO pins : STTS751_INT_Pin IIS3DWB_INT1_Pin */
	GPIO_InitStruct.Pin =  GPIO_PIN_14 ;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	HAL_EXTI_GetHandle(&iis3dwb_exti, EXTI_LINE_14);
	HAL_EXTI_RegisterCallback(&iis3dwb_exti,  HAL_EXTI_COMMON_CB_ID, IIS3DWB_Int_Callback);

	//////////////////////////////////////////////////////////////////////////////////////


}

//FUNCION ASOCIADA A LA INTERRUPCION DE LA FIFO INT1
static void IIS3DWB_Int_Callback(void)
{
	check++;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
