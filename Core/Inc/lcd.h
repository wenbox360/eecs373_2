/*

 * lcd.h

 *

 *  Created on: Nov 20, 2025

 *      Author: darrendong

 */

#include "main.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>



#ifndef SRC_LCD_H_

#define SRC_LCD_H_





#define SPI_CS_GPIO    GPIOE

#define SPI_CS_PIN     GPIO_PIN_12

#define SPI_DC_GPIO    GPIOE

#define SPI_DC_PIN     GPIO_PIN_11

#define SPI_RESET_GPIO GPIOE

#define SPI_RESET_PIN  GPIO_PIN_10



#define CS_LOW()       HAL_GPIO_WritePin(SPI_CS_GPIO, SPI_CS_PIN, GPIO_PIN_RESET)

#define CS_HIGH()      HAL_GPIO_WritePin(SPI_CS_GPIO, SPI_CS_PIN, GPIO_PIN_SET)

#define DC_LOW()       HAL_GPIO_WritePin(SPI_DC_GPIO, SPI_DC_PIN, GPIO_PIN_RESET)

#define DC_HIGH()      HAL_GPIO_WritePin(SPI_DC_GPIO, SPI_DC_PIN, GPIO_PIN_SET)

#define RESET_LOW()    HAL_GPIO_WritePin(SPI_RESET_GPIO, SPI_RESET_PIN, GPIO_PIN_RESET)

#define RESET_HIGH()   HAL_GPIO_WritePin(SPI_RESET_GPIO, SPI_RESET_PIN, GPIO_PIN_SET)



#define WIDTH  240

#define HEIGHT 320

#define GRID_DIM 16

#define MAX_DIST 2000

#define MAX_RADIUS (WIDTH/2 - 10)

#define FILTER_THRESHOLD 70

extern uint8_t zone[GRID_DIM*GRID_DIM];


extern uint16_t pingframe[WIDTH*HEIGHT]; // Full screen buffer
extern uint16_t pongframe[WIDTH*HEIGHT]; // Full screen buffer
extern uint16_t* f;

extern int new_scan_flag; // Extract MSB of byte 3



void ILI9341_DisplayFrame(SPI_HandleTypeDef* hspi_addr);



void DrawPoint(float angle, float dist);

void DrawZone(void);

void UpdateZone(float angle, float dist);



void WriteCommand(uint8_t cmd, SPI_HandleTypeDef* hspi_addr);



void WriteData(uint8_t *data, uint16_t size, SPI_HandleTypeDef* hspi_addr);



void ILI9341_Reset(void);



void FillFrame(uint16_t* frame, uint16_t color);



void DrawLine();



#endif /* SRC_LCD_H_ */
