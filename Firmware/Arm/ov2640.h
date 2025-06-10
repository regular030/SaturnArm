#ifndef OV2640_H
#define OV2640_H

#include "pico/stdlib.h"

// Camera formats
#define OV2640_FORMAT_RGB565 0
#define OV2640_FORMAT_JPEG   1

bool ov2640_init();
void ov2640_set_resolution(uint16_t width, uint16_t height);
void ov2640_set_format(uint8_t format);
bool ov2640_capture_frame(uint8_t *buffer, uint32_t size);

#endif