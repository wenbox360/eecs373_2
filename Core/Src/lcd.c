
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

 // 8x8 font for digits 0-9
const uint8_t font_digits[10][8] = {
    {0x3C,0x66,0x6E,0x7E,0x76,0x66,0x3C,0x00}, //0
    {0x18,0x38,0x18,0x18,0x18,0x18,0x3C,0x00}, //1
    {0x3C,0x66,0x06,0x0C,0x18,0x30,0x7E,0x00}, //2
    {0x3C,0x66,0x06,0x1C,0x06,0x66,0x3C,0x00}, //3
    {0x0C,0x1C,0x3C,0x6C,0x7E,0x0C,0x0C,0x00}, //4
    {0x7E,0x60,0x7C,0x06,0x06,0x66,0x3C,0x00}, //5
    {0x1C,0x30,0x60,0x7C,0x66,0x66,0x3C,0x00}, //6
    {0x7E,0x66,0x0C,0x18,0x18,0x18,0x18,0x00}, //7
    {0x3C,0x66,0x66,0x3C,0x66,0x66,0x3C,0x00}, //8
    {0x3C,0x66,0x66,0x3E,0x06,0x0C,0x38,0x00}  //9
};
// 8x8 font for lowercase letters a-z
const uint8_t font_lowercase[26][8] = {
    {0x00,0x00,0x3C,0x06,0x3E,0x66,0x3E,0x00}, // a
    {0x60,0x60,0x7C,0x66,0x66,0x66,0x7C,0x00}, // b
    {0x00,0x00,0x3C,0x60,0x60,0x60,0x3C,0x00}, // c
    {0x06,0x06,0x3E,0x66,0x66,0x66,0x3E,0x00}, // d
    {0x00,0x00,0x3C,0x66,0x7E,0x60,0x3C,0x00}, // e
    {0x0E,0x18,0x3E,0x18,0x18,0x18,0x18,0x00}, // f
    {0x00,0x00,0x3E,0x66,0x66,0x3E,0x06,0x3C}, // g
    {0x60,0x60,0x7C,0x66,0x66,0x66,0x66,0x00}, // h
    {0x18,0x00,0x38,0x18,0x18,0x18,0x3C,0x00}, // i
    {0x06,0x00,0x06,0x06,0x06,0x06,0x06,0x3C}, // j
    {0x60,0x60,0x66,0x6C,0x78,0x6C,0x66,0x00}, // k
    {0x38,0x18,0x18,0x18,0x18,0x18,0x3C,0x00}, // l
    {0x00,0x00,0x6C,0x7E,0x7E,0x66,0x66,0x00}, // m
    {0x00,0x00,0x7C,0x66,0x66,0x66,0x66,0x00}, // n
    {0x00,0x00,0x3C,0x66,0x66,0x66,0x3C,0x00}, // o
    {0x00,0x00,0x7C,0x66,0x66,0x7C,0x60,0x60}, // p
    {0x00,0x00,0x3E,0x66,0x66,0x3E,0x06,0x06}, // q
    {0x00,0x00,0x7C,0x66,0x60,0x60,0x60,0x00}, // r
    {0x00,0x00,0x3E,0x60,0x3C,0x06,0x7C,0x00}, // s
    {0x18,0x18,0x7E,0x18,0x18,0x18,0x0E,0x00}, // t
    {0x00,0x00,0x66,0x66,0x66,0x66,0x3E,0x00}, // u
    {0x00,0x00,0x66,0x66,0x66,0x3C,0x18,0x00}, // v
    {0x00,0x00,0x66,0x66,0x7E,0x7E,0x6C,0x00}, // w
    {0x00,0x00,0x66,0x3C,0x18,0x3C,0x66,0x00}, // x
    {0x00,0x00,0x66,0x66,0x66,0x3E,0x06,0x3C}, // y
    {0x00,0x00,0x7E,0x0C,0x18,0x30,0x7E,0x00}  // z
};

extern bool zone_mode;
extern bool filter_mode;
extern bool startup;

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
    	//SKIP for now
    	return;
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

void coneZone(float angle, float dist){
	    uint16_t color = 0x001F; // pure blue
	    uint8_t OFFSET = 120;    // center of screen


	    // Ignore invalid distances
	    if (dist <= 0 || dist > MAX_DIST * 2) {
	        return;
	    }

	    int distance_pixels = dist * MAX_RADIUS / MAX_DIST; // scale to screen

	    // Clamp distance to screen bounds
	    if (distance_pixels > MAX_RADIUS) {
	    	//SKIP for now
	    	return;
	        distance_pixels = MAX_RADIUS;
	    }

	    for (int i = distance_pixels; i <= MAX_RADIUS; i++){

			int x = OFFSET + i * sin(angle * M_PI / 180.0); // flip x-axis

			int y = OFFSET + i * cos(angle * M_PI / 180.0); // y downward

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

}

void UpdateZone(float angle, float dist) {
    uint8_t OFFSET = 120;    // center of screen (same as DrawPoint)

    // Ignore invalid distances
    if (dist <= 0 || dist > MAX_DIST * 2) return;

    int distance_pixels = dist * MAX_RADIUS / MAX_DIST; // scale to screen

    // Clamp distance to screen bounds
    if (distance_pixels > MAX_RADIUS) {
    	//SKIP for now
    	return;
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

    //if(filter_mode){
    zone_detected[idx] = true;
    //}
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

// ...existing code...
void DrawCharacter(char c, uint16_t x, uint16_t y, uint16_t color, uint16_t bg_color) {
    if (c < 'a' || c > 'z') return;
    const uint8_t letter_index = (uint8_t)(c - 'a');

    const int SRC_W = 8, SRC_H = 8;
    const int DST_W = 12, DST_H = 12;

    // Always draw glyph mirrored (horizontal flip)
    for (int ty = 0; ty < DST_H; ++ty) {
        int sy = (ty * SRC_H) / DST_H; // 0..7
        uint8_t row_data = font_lowercase[letter_index][sy];

        for (int tx = 0; tx < DST_W; ++tx) {
            int sx = (tx * SRC_W) / DST_W; // 0..7
            bool pixel_on = (row_data & (1u << (7 - sx))) != 0;

            uint16_t px_color = pixel_on ? color : bg_color;

            // flip horizontally: draw_tx maps target column tx to flipped position
            int draw_tx = (DST_W - 1 - tx);
            uint16_t px = x + draw_tx;
            uint16_t py = y + ty;

            if (px < WIDTH && py < HEIGHT) {
                f[py * WIDTH + px] = px_color;
            }
        }
    }
}

void DrawString(const char* str) {
    const uint16_t OFFSET = 135;
    const uint16_t MAX_R = MAX_RADIUS;
    const uint16_t free_top = OFFSET + MAX_R;
    const int char_w = 12, char_h = 12;

    int x = WIDTH - char_w;         // start at right edge of free area
    int y = free_top;               // top of free area
    const uint16_t color = 0xFFFF;   // white
    const uint16_t bg_color = 0x001F; // blue

    while (*str) {
        // If not enough room on current line, wrap to next line (move down)
        if (x < 0) {
            x = WIDTH - char_w;
            y += char_h;
            if (y + char_h > HEIGHT) break;
        }

        if (*str >= 'a' && *str <= 'z') {
            DrawCharacter(*str, (uint16_t)x, (uint16_t)y, color, bg_color);
        } else {
            // draw background block for unsupported characters
            for (int ry = 0; ry < char_h; ++ry) {
                for (int rx = 0; rx < char_w; ++rx) {
                    int px = x + rx;
                    int py = y + ry;
                    if (px >= 0 && px < WIDTH && py >= 0 && py < HEIGHT) {
                        f[py * WIDTH + px] = bg_color;
                    }
                }
            }
        }

        x -= char_w; // move left for next character
        ++str;
    }
}
// ...existing code...
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

    if(!startup){
        if(zone_mode){
        DrawZone();
        if(filter_mode){
            DrawString("zone mode filtered");
        } else {
            DrawString("zone mode unfiltered");
        }
    } else {
        DrawString("raw mode");
    }
    } else{
        DrawString("connection lost...");
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
