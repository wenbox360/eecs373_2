/*

 * lcd.c

 *

 *  Created on: Nov 20, 2025

 *      Author: darrendong

 */



#include "lcd.h"
#include <stdio.h>
#include <string.h>



uint16_t pingframe[WIDTH*HEIGHT];
uint16_t pongframe[WIDTH*HEIGHT];




 uint16_t bg_colors[6] = {

      0x0000, // black

      0x001F, // blue

      0x07E0, // green

      0xFFE0, // yellow

      0x07FF, // cyan

      0xF81F  // magenta

  };


// void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
// {
//     CS_HIGH(); // End the SPI transaction
// }

void WriteCommand(uint8_t cmd, SPI_HandleTypeDef* hspi_addr){

    DC_LOW();

    CS_LOW();

    HAL_SPI_Transmit(hspi_addr, &cmd, 1, HAL_MAX_DELAY);

    CS_HIGH();

}



void WriteData(uint8_t *data, uint16_t size, SPI_HandleTypeDef* hspi_addr){

    DC_HIGH();

    CS_LOW();

    HAL_SPI_Transmit(hspi_addr, data, size, HAL_MAX_DELAY);

    CS_HIGH();

}



void ILI9341_Reset(void){

    RESET_LOW();

    HAL_Delay(1000);

    RESET_HIGH();

    HAL_Delay(1000);

}



// Fill the frame buffer

void FillFrame(uint16_t* frame, uint16_t color){

    for(uint32_t i=0;i<WIDTH*HEIGHT;i++) frame[i] = color;

}



void DrawPoint(float angle, float dist){

    uint16_t color = 0x001F; // pure blue

    uint8_t OFFSET = 120;    // center of screen

    #define MAX_DIST 2000

    #define MAX_RADIUS (WIDTH/2 - 10)

    // Ignore invalid distances
    if (dist <= 0 || dist > MAX_DIST * 2) {
        return;
    }

    int distance_pixels = dist * MAX_RADIUS / MAX_DIST; // scale to screen

    // Clamp distance to screen bounds
    if (distance_pixels > MAX_RADIUS) {
        distance_pixels = MAX_RADIUS;
    }

    int x = OFFSET - distance_pixels * sin(angle * M_PI / 180.0); // flip x-axis

    int y = OFFSET + distance_pixels * cos(angle * M_PI / 180.0); // y downward

    // Bounds check: must leave room for 3x3 block (1 pixel margin on each side)
    if (x < 1 || x >= WIDTH - 1 || y < 1 || y >= HEIGHT - 1) {
        return;
    }

    // Verify buffer pointer is valid
    if (f == NULL) {
        return;
    }

    // draw a 3x3 pixel block (now safe because we checked bounds above)
    f[y*WIDTH + x] = color;
    f[y*WIDTH + x + 1] = color;
    f[y*WIDTH + x - 1] = color;
    f[(y - 1)*WIDTH + x] = color;
    f[(y + 1)*WIDTH + x] = color;
    f[(y - 1)*WIDTH + x - 1] = color;
    f[(y - 1)*WIDTH + x + 1] = color;
    f[(y + 1)*WIDTH + x - 1] = color;
    f[(y + 1)*WIDTH + x + 1] = color;

}
void DrawLine(){

    uint8_t SIZE = 240;

    uint8_t y = 240;

    uint16_t color = 0xFFFF;



    for(int px = 0; px < SIZE; px++){

        f[y*WIDTH + px] = color;

    }



    y = 120;

    color = 0xF800;



    for(int px = 0; px < SIZE; px++){

        f[y*WIDTH + px] = color;

    }



    for(int px = 0; px < SIZE; px++){

        f[WIDTH*px + SIZE/2] = color;

    }



}



// Send the frame buffer to the display
void ILI9341_DisplayFrame(SPI_HandleTypeDef* hspi_addr) {
    if (f == pingframe && new_scan_flag) {
        FillFrame(pongframe, 0x0000);
        new_scan_flag = false;
    } else if (f == pongframe && new_scan_flag) {
        FillFrame(pingframe, 0x0000);
        new_scan_flag = false;
    }

    DrawLine();

    // Set column address
    WriteCommand(0x2A, hspi_addr);
    uint8_t col[4] = {0x00, 0x00, (WIDTH-1)>>8, (WIDTH-1)&0xFF};
    WriteData(col, 4, hspi_addr);

    // Set page address
    WriteCommand(0x2B, hspi_addr);
    uint8_t page[4] = {0x00, 0x00, (HEIGHT-1)>>8, (HEIGHT-1)&0xFF};
    WriteData(page, 4, hspi_addr);

    // Memory write
    WriteCommand(0x2C, hspi_addr);
    DC_HIGH();
    CS_LOW();

    // Send the frame buffer in chunks (max 65535 bytes per call)
    uint32_t total_bytes = WIDTH * HEIGHT * 2; // 153600 bytes
    uint32_t offset = 0;
    while (total_bytes > 0) {
        uint32_t chunk = (total_bytes > 65535) ? 65535 : total_bytes;
        HAL_SPI_Transmit(hspi_addr, (uint8_t*)f + offset, chunk, HAL_MAX_DELAY);
        offset += chunk;
        total_bytes -= chunk;
    }

    CS_HIGH();
}
