/*
 * Archivo_Alberto.h
 *
 *  Created on: Dec 26, 2020
 *      Author: Alberto
 */

#ifndef INC_ARCHIVO_ALBERTO_H_
#define INC_ARCHIVO_ALBERTO_H_


#include "usb_device.h"
//#include "usbd_cdc_if.h"
#include "stm32l4xx_hal.h"
#include "iis2mdc.h"
#include "lps22hh.h"
#include "iis3dwb.h"
#include "steval_stwinkt1_bus.h"

typedef enum estados {Inicializacion, Midiendo, Empaquetando, EnviandoTrama} Estados;


int32_t MX_SPI3_Init_WRAP(void);
int32_t MX_SPI3_DeInit_WRAP(void);
int32_t leer_registro(uint16_t Address, uint16_t Reg, uint8_t * pData, uint16_t len);
int32_t escribir_registro(uint16_t Address, uint16_t Reg, uint8_t * pData, uint16_t len);

void InicializaRegistro(uint16_t tamano_buffer, uint8_t *Trama);
void InicializaSensores(void);
void TomaMedidas(uint16_t Inicio, uint8_t *Trama);
void CreaTrama(uint16_t Inicio, uint8_t *Trama);
void EnviaTrama(uint8_t registro[60]);
void IIS3DWB_Peripheral_Init(void);
static void IIS3DWB_Int_Callback(void);
void MaquinaEstados(void);

#endif /* INC_ARCHIVO_ALBERTO_H_ */
