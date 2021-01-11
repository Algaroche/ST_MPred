/*
 * Archivo_Alberto.c
 *
 *  Created on: Dec 26, 2020
 *      Author: Alberto
 */


#include "Archivo_Alberto.h"

extern SPI_HandleTypeDef hspi3;

uint8_t SPI3InitCounter = 0;
IIS2MDC_Object_t magneto_sensor;
IIS2MDC_IO_t magneto_IO;
IIS2MDC_Axes_t magneto_axes;

LPS22HH_Object_t press_sensor;
LPS22HH_IO_t press_IO;


IIS3DWB_Object_t vibro_sensor;
IIS3DWB_IO_t vibro_IO;
IIS3DWB_Axes_t vibro_axes;


uint8_t len = 3;//18;//12;//
volatile uint8_t check = 0;

EXTI_HandleTypeDef iis3dwb_exti;

#define IIS3DWB_SPI_CS_Pin GPIO_PIN_5
#define IIS3DWB_SPI_CS_GPIO_Port GPIOF
#define IIS3DWB_INT1_Pin GPIO_PIN_14
#define IIS3DWB_INT1_GPIO_Port GPIOE
#define IIS3DWB_INT1_EXTI_IRQn EXTI15_10_IRQn
#define IIS3DWB_FIFO_STATUS2                 0x3BU

uint8_t exito;
uint8_t fifo_buffer[3000];

static int16_t buffer[100];
static uint16_t posicion_inicio=0;
static uint16_t numero_medidas=0;
static Estados estado_actual=Inicializacion;

void MaquinaEstados(void)
{

	uint16_t max_medidas = sizeof(buffer)/len/sizeof(buffer[0]);

	switch (estado_actual)
	{
	case Inicializacion:
		InicializaRegistro(sizeof(buffer)/sizeof(buffer[0]), &buffer);
		InicializaSensores();
		estado_actual = Midiendo;
		break;

	case Midiendo:
		TomaMedidas(posicion_inicio, &buffer);
//		Calcula_FFT(posicion_inicio, &buffer);
		numero_medidas += 1;
		estado_actual = Empaquetando;

		break;

	case Empaquetando:
//		CreaTrama(posicion_inicio,&buffer);
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
		//CDC_Transmit_FS(&buffer, sizeof(buffer));
		//EnviaTrama(buffer);
		//InicializaRegistro(sizeof(buffer), &buffer);
		numero_medidas = 0;
		posicion_inicio = 0;
		estado_actual = Midiendo;
		break;
	}
}

void IIS3DWB_Peripheral_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(IIS3DWB_SPI_CS_GPIO_Port, IIS3DWB_SPI_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : IIS3DWB_SPI_CS_Pin */
	GPIO_InitStruct.Pin = IIS3DWB_SPI_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(IIS3DWB_SPI_CS_GPIO_Port, &GPIO_InitStruct);

///////////////////YA SE HA CONFIGURADO ESTO EN EL MAIN////////////////////
	/*Configure GPIO pins : STTS751_INT_Pin IIS3DWB_INT1_Pin */
//	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
//////////////////////////////////////////////////////////////////////////

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	HAL_EXTI_GetHandle(&iis3dwb_exti, EXTI_LINE_14);
	HAL_EXTI_RegisterCallback(&iis3dwb_exti,  HAL_EXTI_COMMON_CB_ID, IIS3DWB_Int_Callback);

}

static void IIS3DWB_Int_Callback(void)
{
	check++;
}

void InicializaRegistro(uint16_t tamano_buffer, int16_t *Trama)
{
	uint16_t i=0;
	for (i=0; i<(tamano_buffer);i++)
		Trama[i]=0;
}
void InicializaSensores(void)
{
	//Configura comunicaciones del sensor de presión y temperatura

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

	//Configura comunicaciones del magnetómetro

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

	//Configura comunicaciones del vibrómetro

		IIS3DWB_Peripheral_Init();

		vibro_IO.Address	= 0;
		vibro_IO.BusType	= 2;	//SPI de 3 cables
		vibro_IO.DeInit		= MX_SPI3_DeInit_WRAP;
		vibro_IO.GetTick	= BSP_GetTick;
		vibro_IO.Init		= MX_SPI3_Init_WRAP;
		vibro_IO.ReadReg	= leer_registro;
		vibro_IO.WriteReg	= escribir_registro;

		IIS3DWB_RegisterBusIO(&vibro_sensor, &vibro_IO);
		IIS3DWB_Init(&vibro_sensor);
		//	IIS3DWB_ACC_GetAxes(&vibro_sensor, &vibro_axes);

		float maxODR =26700.0f;// 0.0f;//
		float sensitivity;
		IIS3DWB_ACC_SetOutputDataRate(&vibro_sensor, maxODR);		// Pongo el ODR al máximo 26.7KHz
		IIS3DWB_ACC_SetFullScale(&vibro_sensor, 2);					//Pongo el fullscale range de la aceleración en 2g
		IIS3DWB_ACC_GetSensitivity(&vibro_sensor, &sensitivity);

//		IIS3DWB_FIFO_Set_Mode(&vibro_sensor, IIS3DWB_FIFO_MODE);
//		IIS3DWB_INT1_Set_FIFO_Full(&vibro_sensor, IIS3DWB_FIFO_STATUS2);

		IIS3DWB_ACC_Enable(&vibro_sensor);
		//	uint8_t status;
		//	IIS3DWB_FIFO_Get_Full_Status(&vibro_sensor, &status);


}
void TomaMedidas(uint16_t Inicio, int16_t *Trama)
{
//		float press_value;
//		LPS22HH_PRESS_GetPressure(&press_sensor, &press_value);
//		Trama[Inicio+1] = ((uint16_t)(press_value*10));
//		Trama[Inicio+2] = ((uint16_t)(press_value*10))>>8;
//		float temp_value;
//		LPS22HH_TEMP_GetTemperature(&press_sensor, &temp_value);
//		Trama[Inicio+3] = ((uint16_t)(temp_value*100));
//		Trama[Inicio+4] = ((uint16_t)(temp_value*100))>>8;
//
//		IIS2MDC_MAG_GetAxes(&magneto_sensor, &magneto_axes);
//		Trama[Inicio+5] = ((uint16_t)(magneto_axes.x));
//		Trama[Inicio+6] = ((uint16_t)(magneto_axes.x))>>8;
//		Trama[Inicio+7] = ((uint16_t)(magneto_axes.y));
//		Trama[Inicio+8] = ((uint16_t)(magneto_axes.y))>>8;
//		Trama[Inicio+9] = ((uint16_t)(magneto_axes.z));
//		Trama[Inicio+10] = ((uint16_t)(magneto_axes.z))>>8;

		IIS3DWB_ACC_GetAxes(&vibro_sensor, &vibro_axes);
//		Trama[Inicio+11] = ((uint16_t)(vibro_axes.x));
//		Trama[Inicio+12] = ((uint16_t)(vibro_axes.x))>>8;
//		Trama[Inicio+13] = ((uint16_t)(vibro_axes.y));
//		Trama[Inicio+14] = ((uint16_t)(vibro_axes.y))>>8;
//		Trama[Inicio+15] = ((uint16_t)(vibro_axes.z));
//		Trama[Inicio+16] = ((uint16_t)(vibro_axes.z))>>8;

		Trama[Inicio+0] = ((uint16_t)(vibro_axes.x));
		Trama[Inicio+1] = ((uint16_t)(vibro_axes.y));
		Trama[Inicio+2] = ((uint16_t)(vibro_axes.z));

//		Trama[Inicio+5] = ((uint16_t)(vibro_axes.x));
//		Trama[Inicio+6] = ((uint16_t)(vibro_axes.x))>>8;
//		Trama[Inicio+7] = ((uint16_t)(vibro_axes.y));
//		Trama[Inicio+8] = ((uint16_t)(vibro_axes.y))>>8;
//		Trama[Inicio+9] = ((uint16_t)(vibro_axes.z));
//		Trama[Inicio+10] = ((uint16_t)(vibro_axes.z))>>8;


}
void CreaTrama(uint16_t Inicio, int16_t *Trama)
{
	Trama[Inicio] = 0x3e;
	unsigned int BCC_CRC = 0;
	for (int i = Inicio; i < (Inicio+len-1); i++)
		BCC_CRC ^= Trama[i];
	Trama[Inicio+len-1]=BCC_CRC;
}

void Calcula_FFT(uint16_t Inicio, int16_t *Trama)
{
//	while (check == 0){
//		//SE ESPERA A LA INTERRUPCIÓN DE FIFO FULL
//
//	}
//	IIS3DWB_FIFO_Read(&vibro_sensor, fifo_buffer, 100);
}

void EnviaTrama(uint8_t registro[60])
{

	CDC_Transmit_FS(&registro, sizeof(registro));

}

//Encapsula la función de inicialización del SPI3 para poder pasarsela al inicializdor del sensor que vaya por SPI

int32_t MX_SPI3_Init_WRAP(void)
{
	int32_t ret = BSP_ERROR_NONE;
	if (HAL_SPI_Init(&hspi3) != HAL_OK)
	{
		ret = BSP_ERROR_BUS_FAILURE;
	}
	//MX_SPI3_Init();
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


int32_t leer_registro(uint16_t Address, uint16_t Reg, uint8_t * pData, uint16_t len)
{
	uint8_t autoInc = 0x00;

	Reg = Reg | 0x80 | autoInc;

	HAL_GPIO_WritePin(GPIOF  , GPIO_PIN_5 , GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &Reg, 1, 1000);
	HAL_SPI_Receive(&hspi3, pData, len, 1000);
	HAL_GPIO_WritePin(GPIOF  , GPIO_PIN_5 , GPIO_PIN_SET);
	return 0;

	//	  HAL_StatusTypeDef Error_Code;
	//	  int32_t ret = 0;
	//
	//	  //Pongo a cero el ChipSelect del sensor correspondiente
	//	  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5,GPIO_PIN_RESET);
	//	  //Mando un mensaje indicando el registro que quiero leer
	//	  if ((Error_Code = HAL_SPI_Transmit(&hspi3, &Reg, 2, 10000)) != HAL_OK)
	//		  ret = Error_Code;
	//	  //Recibo datos del registro indicado
	//	  if ((Error_Code = HAL_SPI_Receive(&hspi3, &pData, len, 10000)) != HAL_OK)
	//		  ret = Error_Code;
	//	  //Pongo a uno el ChipSelect del sensor correspondiente
	//	  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5,GPIO_PIN_SET);
	//
	//	  return ret;

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
