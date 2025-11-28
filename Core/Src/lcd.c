/*

 * lcd.c

 *

 *  Created on: Nov 20, 2025

 *      Author: darrendong

 */



#include "lcd.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>


bool zone_detected[GRID_DIM*GRID_DIM];

uint16_t zone_history[GRID_DIM*GRID_DIM];

uint8_t zone[GRID_DIM*GRID_DIM];

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

extern bool zone_mode;
extern bool filter_mode;

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


    // Ignore invalid distances
    if (dist <= 0 || dist > MAX_DIST * 2) {
        return;
    }

    int distance_pixels = dist * MAX_RADIUS / MAX_DIST; // scale to screen

    // Clamp distance to screen bounds
    if (distance_pixels > MAX_RADIUS) {
        distance_pixels = MAX_RADIUS;
    }

    int x = OFFSET + distance_pixels * sin(angle * M_PI / 180.0); // flip x-axis

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

void UpdateZone(float angle, float dist) {
    uint8_t OFFSET = 120;    // center of screen (same as DrawPoint)

    // Ignore invalid distances
    if (dist <= 0 || dist > MAX_DIST * 2) return;

    int distance_pixels = dist * MAX_RADIUS / MAX_DIST; // scale to screen

    // Clamp distance to screen bounds
    if (distance_pixels > MAX_RADIUS) {
        distance_pixels = MAX_RADIUS;
    }

    int x = OFFSET + distance_pixels * sin(angle * M_PI / 180.0);
    int y = OFFSET + distance_pixels * cos(angle * M_PI / 180.0); // flip y-axis

    // Check if point is within the square area
    if (x < OFFSET - MAX_RADIUS || x >= OFFSET + MAX_RADIUS ||
        y < OFFSET - MAX_RADIUS || y >= OFFSET + MAX_RADIUS) return;

    // Map pixel to grid cell within the square
    uint8_t grid_size = MAX_RADIUS * 2;
    uint8_t cell_size = grid_size / GRID_DIM;

    uint8_t cell_x = (x - (OFFSET - MAX_RADIUS)) / cell_size;
    uint8_t cell_y = (y - (OFFSET - MAX_RADIUS)) / cell_size;

    if (cell_x >= GRID_DIM || cell_y >= GRID_DIM) return;

    uint8_t idx = cell_y * GRID_DIM + cell_x;
    if (zone[idx] < 255) zone[idx]++; // Prevent overflow

    if(filter_mode){
        zone_detected[idx] = true;
    }
}

void DrawZone(void) {
    uint8_t OFFSET = 120;    // center of screen (same as DrawPoint)

    // Example colors: faint to intense (blue shades)
    uint16_t colors[] = {
        0x0010, // very faint blue
        0x001F, // faint blue
        0x03FF, // medium blue
        0x07FF, // intense blue
        0x07FF  // max intensity (repeat for 10+)
    };

    // Calculate grid cell size within the MAX_RADIUS square
    uint8_t grid_size = MAX_RADIUS * 2; // Total size of the square
    uint8_t cell_size = grid_size / GRID_DIM; // Size of each cell

    for (uint8_t gy = 0; gy < GRID_DIM; gy++) {
        for (uint8_t gx = 0; gx < GRID_DIM; gx++) {
            uint8_t idx = gy * GRID_DIM + gx;
            uint8_t count = zone[idx];

            if (filter_mode && zone_history[idx] >= FILTER_THRESHOLD) {
                // Completely skip static cells (leave black)
                continue;
            }

            uint16_t color; // Choose color based on count

            if (count == 0 || (filter_mode && count <= 3)) {
                continue; // Skip empty cells (leave black)
            } else if (count == 1) {
                color = colors[0];
            } else if (count <= 3) {
                color = colors[1];
            } else if (count <= 6) {
                color = colors[2];
            } else if (count <= 9) {
                color = colors[3];
            } else {
                color = colors[4];
            }

            // Calculate the boundaries of this grid cell (within the square)
            int cell_start_x = OFFSET - MAX_RADIUS + gx * cell_size;
            int cell_start_y = OFFSET - MAX_RADIUS + gy * cell_size;
            int cell_end_x = cell_start_x + cell_size;
            int cell_end_y = cell_start_y + cell_size;

            // Fill the entire grid cell
            for (int y = cell_start_y; y < cell_end_y; y++) {
                for (int x = cell_start_x; x < cell_end_x; x++) {
                    // Bounds check
                    if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) {
                        f[y * WIDTH + x] = color;
                    }
                }
            }
        }
    }
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


// void ClearQuadrant(int quadrant) {
//     uint8_t OFFSET = 120;
//     uint16_t color = 0x0000; // black

//     float start_angle = quadrant * 90.0f;
//     float end_angle = start_angle + 90.0f;

//     for (int y = 0; y < HEIGHT; y++) {
//         for (int x = 0; x < WIDTH; x++) {
//             // Convert (x, y) to polar coordinates relative to center
//             float dx = x - OFFSET;
//             float dy = y - OFFSET;
//             float angle = atan2(dx, dy) * 180.0f / M_PI;
//             if (angle < 0) angle += 360.0f;

//             // If pixel is in the quadrant, clear it
//             if (angle >= start_angle && angle < end_angle) {
//                 f[y * WIDTH + x] = color;
//             }
//         }
//     }
// }

// Send the frame buffer to the display
void ILI9341_DisplayFrame(SPI_HandleTypeDef* hspi_addr) {
    // if (f == pingframe && new_scan_flag) {
    //     FillFrame(pongframe, 0x0000);
    //     memset(zone, 0, sizeof(zone));
    //     new_scan_flag = false;
    // } else if (f == pongframe && new_scan_flag) {
    //     FillFrame(pingframe, 0x0000);
    //     memset(zone, 0, sizeof(zone));
    //     new_scan_flag = false;
    // }

    //memset(zone, 0, sizeof(zone));

    DrawLine();

    if(zone_mode){
        DrawZone();
    }

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
