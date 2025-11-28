/*
 * lidar.c
 *
 *  Created on: Nov 20, 2025
 *      Author: darrendong
 */

#include "lidar.h"
#include "lcd.h"
#include <stdio.h>

extern bool zone_mode;

int new_scan_flag = false;
uint16_t* f = pingframe;


const uint8_t express_scan[9] = {
            0xA5, 0x82,
            0x05,                    // Payload length
            0x00,                    // working_mode (0 for legacy express)
            0x00, 0x00,             // working_flags
            0x00, 0x00,             // param
            0x22                     // Checksum
        };

const uint8_t stop[2] = {
            0xA5, 0x25
        };

const uint8_t scan[2] = {
            0xA5, 0x20
        };

//uint8_t prev_packet[84];
//extern bool has_prev_packet;
//float prev_start_angle = 0;


//float angle_diff(float angle1, float angle2) {
//    if (angle1 <= angle2) {
//        return angle2 - angle1;
//    } else {
//        return 360.0f + angle2 - angle1;
//    }
//}

void send_stop_command(UART_HandleTypeDef* huart_addr) {
    HAL_UART_Transmit(huart_addr, stop, sizeof(stop), HAL_MAX_DELAY);
}



void send_express_scan_command(UART_HandleTypeDef* huart_addr) {
    HAL_UART_Transmit(huart_addr, express_scan, sizeof(express_scan), HAL_MAX_DELAY);
}

void send_scan_command(UART_HandleTypeDef* huart_addr){
    HAL_UART_Transmit(huart_addr, scan, sizeof(scan), HAL_MAX_DELAY);
}

// void decode_normal_scan(uint8_t* capsule_data) {

//     // Normal scan response is 5 bytes per measurement:
//     // Byte 0: quality (bits 7-2), inversed_new_scan (bit 1), new_scan (bit 0)
//     // Byte 1: angle_high (bits 7-1), check_bit (bit 0, must be 1)
//     // Byte 2: angle_low (bits 7-0)
//     // Bytes 3-4: distance (little endian, 16-bit)
// 		// Extract flags
// 		bool new_scan = (capsule_data[0] & 0b1) != 0;
// 		bool inversed_new_scan = ((capsule_data[0] >> 1) & 0b1) != 0;
// 		uint8_t quality = capsule_data[0] >> 2;

// 		if (new_scan){
// 			new_scan_flag = true;
// 			if (f == pingframe) f = pongframe;
// 			else if (f == pongframe) f = pingframe;
// 		}

// 	    // Validate flags
// 	    if (new_scan == inversed_new_scan) {
// 	        //printf("ERROR: New scan flags mismatch\n");
// 	        return;
// 	    }

// 	    // Check bit validation
// 	    uint8_t check_bit = capsule_data[1] & 0b1;
// 	    if (check_bit != 1) {
// 	        //printf("ERROR: Check bit not equal to 1\n");
// 	        return;
// 	    }

// 		// Extract angle: ((raw[1] >> 1) + (raw[2] << 7)) / 64.0
// 		uint16_t angle_raw = ((capsule_data[1] >> 1) + (capsule_data[2] << 7));
// 		float angle = angle_raw / 64.0f;

// 		// Extract distance: (raw[3] + (raw[4] << 8)) / 4.0
// 		uint16_t distance_raw = capsule_data[3] + (capsule_data[4] << 8);
// 		float distance_mm = distance_raw / 4.0f;

// 	//    // Print the result
// 	//    printf("%s theta: %06.1f Dist: %08.2f Q: %d\n",
// 	//           new_scan ? "S " : "  ", angle, distance_mm, quality);
// 		if(quality >=  MIN_SCAN_QUALITY){
// 			if(zone_mode){
//                 UpdateZone(angle, distance_mm);
// 			} else{
// 				DrawPoint(angle, distance_mm);
// 			}
// 		}
// }


// int clear_quadrant = 0;

// void process_angle(int pp_angle) {
//     int quadrant = pp_angle / 5760; // 0 for 0-90, 1 for 91-180, etc.

//     if (quadrant != clear_quadrant) {
//         // Clear the new quadrant
//         ClearQuadrant(quadrant);
// 		ILI9341_DisplayFrame(&hspi1);
//         clear_quadrant = (quadrant + 1) % 4;
//     }
//     // ...existing code to draw/update...
// }



bool decode_normal_scan(uint8_t* capsule_data) {
    // Validate quality
    uint8_t quality = capsule_data[0] >> 2;
    if (quality > 15) {
        return false; // Invalid packet
    }

    // Validate s and ~s bits
    bool new_scan = (capsule_data[0] & 0b1) != 0;
    bool inversed_new_scan = ((capsule_data[0] >> 1) & 0b1) != 0;
    if (new_scan == inversed_new_scan) {
        return false; // Invalid packet
    }

    // Validate c bit
    uint8_t check_bit = capsule_data[1] & 0b1;
    if (check_bit != 1) {
        return false; // Invalid packet
    }

	

    if (new_scan){
		new_scan_flag = true;
	}
	// 	if (f == pingframe) f = pongframe;
	// 	else if (f == pongframe) f = pingframe;
	// }

    // If all validations pass, proceed with decoding
    uint16_t angle_raw = ((capsule_data[1] >> 1) + (capsule_data[2] << 7));

    // process_angle(angle_raw);
    float angle = angle_raw / 64.0f;

    uint16_t distance_raw = capsule_data[3] + (capsule_data[4] << 8);
    float distance_mm = distance_raw / 4.0f;

    if (quality >= MIN_SCAN_QUALITY) {
        if (zone_mode) {
            UpdateZone(angle, distance_mm);
        } else {
            DrawPoint(angle, distance_mm);
        }
    }

    return true; // Valid packet
}

