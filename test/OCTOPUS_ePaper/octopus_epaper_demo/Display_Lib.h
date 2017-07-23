/********************------------------------------------------------------------

Show library, picture, number of hexadecimal

Define the size, delay, and logo,They are all hexadecimal

EPD2X9
EPD02X13
EPD1X54
Pass-type - positive - forward mode - 16 bits per line
------------------------------------------------------------------------------*/
// http://www.digole.com/tools/PicturetoC_Hex_converter.php

#define DELAYTIME 1500

const unsigned char octo[5000] = { /* 0X01,0X01,0XC8,0X00,0XC8,0X00, */
0xff,0xff,0xff,0xff,0xff,0xff,0x80,0x00,0x0f,0xff,0xff,0xff,0x80,0x00,0x00,0x3f,0xff,0xff,0xfc,0x00,0x00,0x00,0x00,0x00,0x7f
,0xff,0xff,0xff,0xff,0xff,0xff,0x80,0x00,0x1f,0xff,0xff,0xff,0x00,0x00,0x00,0x7f,0xff,0xff,0xf8,0x00,0x00,0x00,0x00,0x00,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0x80,0x00,0x1f,0xff,0xff,0xff,0x00,0x00,0x00,0xff,0xff,0xff,0xf0,0x00,0x00,0x00,0x00,0x00,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0x80,0x00,0x1f,0xff,0xff,0xfe,0x00,0x00,0x00,0xff,0xff,0xff,0xe0,0x00,0x00,0x00,0x00,0x01,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0x80,0x00,0x1f,0xff,0xff,0xfc,0x00,0x00,0x01,0xff,0xff,0xff,0xc0,0x00,0x00,0x00,0x00,0x03,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0x80,0x00,0x1f,0xff,0xff,0xf8,0x00,0x00,0x03,0xff,0xff,0xff,0x80,0x00,0x00,0x00,0x00,0x07,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0x80,0x00,0x1f,0xff,0xff,0xf0,0x00,0x00,0x03,0xff,0xff,0xff,0x80,0x00,0x00,0x00,0x00,0x0f,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0x80,0x00,0x1f,0xff,0xff,0xe0,0x00,0x00,0x07,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x1f,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0x80,0x00,0x1f,0xff,0xff,0xc0,0x00,0x00,0x0f,0xff,0xff,0xfe,0x00,0x00,0x00,0x00,0x00,0x3f,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0x80,0x00,0x1f,0xff,0xff,0x80,0x00,0x00,0x0f,0xff,0xff,0xfc,0x00,0x00,0x00,0x00,0x00,0x7f,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0x80,0x00,0x1f,0xff,0xff,0x00,0x00,0x00,0x1f,0xff,0xff,0xf8,0x00,0x00,0x00,0x00,0x00,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0x80,0x00,0x1f,0xff,0xfe,0x00,0x00,0x00,0x3f,0xff,0xff,0xf0,0x00,0x00,0x00,0x00,0x01,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0x80,0x00,0x1f,0xff,0xfc,0x00,0x00,0x00,0x7f,0xff,0xff,0xe0,0x00,0x00,0x00,0x00,0x07,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0x80,0x00,0x1f,0xff,0xf8,0x00,0x00,0x00,0x7f,0xff,0xff,0xe0,0x00,0x00,0x00,0x00,0x0f,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0x80,0x00,0x1f,0xff,0xf0,0x00,0x00,0x00,0xff,0xff,0xff,0xc0,0x00,0x00,0x00,0x00,0x3f,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0x80,0x00,0x1f,0xff,0xe0,0x00,0x00,0x01,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x01,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0x80,0x00,0x1f,0xff,0xc0,0x00,0x00,0x03,0xff,0xff,0xfe,0x00,0x00,0x00,0x00,0x07,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0x80,0x00,0x0f,0xff,0x80,0x00,0x00,0x03,0xff,0xff,0xfc,0x00,0x00,0x00,0x00,0x3f,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xc0,0x00,0x0f,0xff,0x80,0x00,0x00,0x07,0xff,0xff,0xf8,0x00,0x00,0x00,0x01,0xff,0xff,0xff,0xf8
,0xff,0xff,0xff,0xff,0xff,0xff,0xc0,0x00,0x0f,0xff,0x00,0x00,0x00,0x0f,0xff,0xff,0xf0,0x00,0x00,0x00,0x0f,0xff,0xff,0xff,0xc0
,0xff,0xff,0xff,0xff,0xff,0xff,0xc0,0x00,0x0f,0xfe,0x00,0x00,0x00,0x1f,0xff,0xff,0xe0,0x00,0x00,0x00,0x7f,0xff,0xff,0xfe,0x00
,0xff,0xff,0xff,0xff,0xff,0xff,0xc0,0x00,0x0f,0xfe,0x00,0x00,0x00,0x1f,0xff,0xff,0x80,0x00,0x00,0x01,0xff,0xff,0xff,0xf0,0x00
,0xff,0xff,0xff,0xff,0xff,0xff,0xc0,0x00,0x0f,0xfc,0x00,0x00,0x00,0x3f,0xff,0xff,0x00,0x00,0x00,0x07,0xff,0xff,0xff,0x80,0x00
,0xff,0xff,0xff,0xff,0xff,0xff,0xc0,0x00,0x0f,0xf8,0x00,0x00,0x00,0x7f,0xff,0xfe,0x00,0x00,0x00,0x1f,0xff,0xff,0xfc,0x00,0x00
,0xff,0xff,0xff,0xff,0xff,0xff,0xc0,0x00,0x0f,0xf8,0x00,0x00,0x00,0xff,0xff,0xf8,0x00,0x00,0x00,0x7f,0xff,0xff,0xf0,0x00,0x00
,0xff,0xff,0xff,0xff,0xff,0xff,0xe0,0x00,0x07,0xf0,0x00,0x00,0x00,0xff,0xff,0xf0,0x00,0x00,0x01,0xff,0xff,0xff,0x80,0x00,0x00
,0xff,0xff,0xff,0xff,0xff,0xff,0xe0,0x00,0x07,0xf0,0x00,0x00,0x01,0xff,0xff,0xe0,0x00,0x00,0x03,0xff,0xff,0xfc,0x00,0x00,0x00
,0xff,0xff,0xff,0xff,0xff,0xff,0xe0,0x00,0x07,0xe0,0x00,0x00,0x03,0xff,0xff,0x80,0x00,0x00,0x0f,0xff,0xff,0xf0,0x00,0x00,0x00
,0xff,0xff,0xff,0xff,0xff,0xff,0xe0,0x00,0x07,0xe0,0x00,0x00,0x03,0xff,0xff,0x00,0x00,0x00,0x3f,0xff,0xff,0xc0,0x00,0x00,0x00
,0xff,0xff,0xff,0xff,0xff,0xff,0xe0,0x00,0x07,0xc0,0x00,0x00,0x07,0xff,0xfe,0x00,0x00,0x00,0x7f,0xff,0xff,0x00,0x00,0x00,0x00
,0x7f,0xff,0xff,0xff,0xff,0xff,0xe0,0x00,0x07,0xc0,0x00,0x00,0x0f,0xff,0xfc,0x00,0x00,0x00,0xff,0xff,0xfc,0x00,0x00,0x00,0x00
,0x07,0xff,0xff,0xff,0xff,0xff,0xe0,0x00,0x03,0x80,0x00,0x00,0x0f,0xff,0xf8,0x00,0x00,0x01,0xff,0xff,0xf0,0x00,0x00,0x00,0x00
,0x01,0xff,0xff,0xff,0xff,0xff,0xe0,0x00,0x03,0x00,0x00,0x00,0x1f,0xff,0xe0,0x00,0x00,0x03,0xff,0xff,0xe0,0x00,0x00,0x00,0x00
,0x00,0x3f,0xff,0xff,0xff,0xff,0xe0,0x00,0x03,0x00,0x00,0x00,0x3f,0xff,0xc0,0x00,0x00,0x0f,0xff,0xff,0x80,0x00,0x00,0x00,0x00
,0x00,0x0f,0xff,0xff,0xff,0xff,0xe0,0x00,0x03,0x00,0x00,0x00,0x3f,0xff,0x00,0x00,0x00,0x1f,0xff,0xfe,0x00,0x00,0x00,0x00,0x00
,0x00,0x01,0xff,0xff,0xff,0xff,0xe0,0x00,0x02,0x00,0x00,0x00,0x7f,0xfe,0x00,0x00,0x00,0x3f,0xff,0xf8,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x7f,0xff,0xff,0xff,0xe0,0x00,0x00,0x00,0x00,0x00,0x7f,0xfc,0x00,0x00,0x00,0x7f,0xff,0xf0,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x1f,0xff,0xff,0xff,0xe0,0x00,0x00,0x00,0x00,0x00,0xff,0xf0,0x00,0x00,0x00,0x7f,0xff,0xc0,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x07,0xff,0xff,0xff,0xe0,0x00,0x00,0x00,0x00,0x01,0xff,0xe0,0x00,0x00,0x00,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x01,0xff,0xff,0xff,0xe0,0x00,0x00,0x00,0x00,0x01,0xff,0x80,0x00,0x00,0x01,0xff,0xfc,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x7f,0xff,0xff,0xe0,0x00,0x00,0x00,0x00,0x03,0xff,0x00,0x00,0x00,0x03,0xff,0xf0,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x1f,0xff,0xff,0xe0,0x00,0x00,0x00,0x00,0x03,0xfe,0x00,0x00,0x00,0x07,0xff,0xe0,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x07,0xff,0xff,0xc0,0x00,0x00,0x00,0x00,0x07,0xfc,0x00,0x00,0x00,0x0f,0xff,0x80,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x01,0xff,0xff,0xc0,0x00,0x00,0x00,0x00,0x0f,0xf8,0x00,0x00,0x00,0x0f,0xfe,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0xff,0xff,0xc0,0x00,0x00,0x00,0x00,0x0f,0xf0,0x00,0x00,0x00,0x1f,0xf8,0x00,0x00,0x00,0x00,0x00,0x00,0x0f
,0x00,0x00,0x00,0x00,0x3f,0xff,0xc0,0x00,0x00,0x00,0x00,0x1f,0xc0,0x00,0x00,0x00,0x3f,0xf0,0x00,0x00,0x00,0x00,0x00,0x01,0xff
,0x00,0x00,0x00,0x00,0x1f,0xff,0x80,0x00,0x00,0x00,0x00,0x1f,0x80,0x00,0x00,0x00,0x7f,0xc0,0x00,0x00,0x00,0x00,0x00,0x07,0xff
,0x00,0x00,0x00,0x00,0x07,0xff,0x80,0x00,0x00,0x00,0x00,0x3f,0x00,0x00,0x00,0x00,0xff,0x80,0x00,0x00,0x00,0x00,0x00,0x7f,0xff
,0x00,0x00,0x00,0x00,0x03,0xff,0x80,0x00,0x00,0x00,0x00,0x3e,0x00,0x00,0x00,0x00,0xfe,0x00,0x00,0x00,0x00,0x00,0x07,0xff,0xff
,0x00,0x00,0x00,0x00,0x01,0xff,0x00,0x00,0x00,0x00,0x00,0x7e,0x00,0x00,0x00,0x01,0xfc,0x00,0x00,0x00,0x00,0x00,0x3f,0xff,0xff
,0x00,0x00,0x00,0x00,0x00,0x7f,0x00,0x00,0x00,0x00,0x00,0x78,0x00,0x00,0x00,0x03,0xf0,0x00,0x00,0x00,0x00,0x01,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x00,0x3e,0x00,0x00,0x00,0x00,0x00,0xf0,0x00,0x00,0x00,0x03,0xc0,0x00,0x00,0x00,0x00,0x0f,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x00,0x1e,0x00,0x00,0x00,0x00,0x00,0xe0,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x7f,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x00,0x0c,0x00,0x00,0x00,0x00,0x00,0xc0,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x01,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x01,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xff,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xff,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0xff,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7f,0xff,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xff,0xff,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xff,0xff,0xff,0xf0,0x00,0x0f
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0xff,0xff,0xfe,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7f,0xff,0xff,0xe0,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xff,0xff,0xfe,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xff,0xff,0xf0,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0xff,0xff,0xc0,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0xff,0xfe,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xf8,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xff,0xff,0xc0,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0xff,0xfc,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7f,0xff,0xf0,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xc0,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xff,0xf8,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0xff,0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xfc,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x38,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xff,0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x3e,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xfe,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x1f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x0f,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x07,0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x07,0xf8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x03,0xfc,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x01,0xfe,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0xff,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x7f,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xff
,0x00,0x00,0x00,0x00,0x7f,0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0xff,0xff
,0x00,0x00,0x00,0x00,0x3f,0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x3f,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x1f,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x1f,0xf8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x0f,0xf8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xff,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x0f,0xfc,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0xff,0xff,0xfe,0x00,0x00
,0x00,0x00,0x00,0x00,0x0f,0xfc,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x07,0xfc,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xff,0xff,0xfc,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x07,0xfe,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0xff,0xff,0x80,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x07,0xfe,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xf8,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x03,0xfe,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xff,0xff,0xc0,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x03,0xfe,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xff,0xfe,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x03,0xfe,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0xff,0xf0,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x03,0xfe,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7f,0xff,0xc0,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x03,0xfe,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xfe,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x03,0xfe,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xff,0xf8,0x00,0x00,0x00,0x00,0x00,0x07
,0x00,0x00,0x00,0x00,0x03,0xfe,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xff,0xe0,0x00,0x00,0x00,0x00,0x00,0x7f
,0x00,0x00,0x00,0x00,0x03,0xfe,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0xff,0x80,0x00,0x00,0x00,0x00,0x03,0xff
,0x00,0x00,0x00,0x00,0x03,0xfc,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0xfe,0x00,0x00,0x00,0x00,0x00,0x3f,0xff
,0x00,0x00,0x00,0x00,0x03,0xf8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0xf8,0x00,0x00,0x00,0x00,0x01,0xff,0xff
,0x00,0x00,0x00,0x00,0x03,0xf8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0xe0,0x00,0x00,0x00,0x00,0x07,0xff,0xff
,0x00,0x00,0x00,0x00,0x03,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x80,0x00,0x00,0x00,0x00,0x3f,0xff,0xff
,0x00,0x00,0x00,0x00,0x07,0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xfe,0x00,0x00,0x00,0x00,0x01,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x07,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xfc,0x00,0x00,0x00,0x00,0x07,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x0f,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xf0,0x00,0x00,0x00,0x00,0x3f,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x1f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xc0,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x1e,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0x80,0x00,0x00,0x00,0x03,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x38,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1e,0x00,0x00,0x00,0x00,0x07,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x38,0x00,0x00,0x00,0x00,0x1f,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x01,0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x00,0x00,0x00,0x00,0x7f,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x03,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x01,0xff,0xff,0xff,0xff,0xf8
,0x00,0x00,0x00,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xff,0xff,0xff,0xff,0xc0
,0x00,0x00,0x00,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0xff,0xff,0xff,0xfc,0x00
,0x00,0x00,0x00,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0xff,0xff,0xff,0xe0,0x00
,0x00,0x00,0x01,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7f,0xff,0xff,0xff,0x80,0x00
,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xff,0xff,0xff,0xfc,0x00,0x00
,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xff,0xff,0xff,0xf0,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0xff,0xff,0xff,0xc0,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0xff,0xff,0xff,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7f,0xff,0xff,0xfc,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xff,0xff,0xff,0xf0,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xff,0xff,0xff,0xc0,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0xff,0xff,0xff,0x80,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0xff,0xff,0xfe,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7f,0xff,0xff,0xf8,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xff,0xff,0xff,0xf0,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xff,0xff,0xff,0xc0,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0xff,0xff,0xff,0x80,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0xff,0xff,0xfe,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xf8,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xff,0xff,0xff,0xf0,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xff,0xff,0xff,0xc0,0x00,0x00,0x00,0x00,0x01
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x1f
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7f,0xff,0xff,0xfc,0x00,0x00,0x00,0x00,0x00,0xff
,0x00,0x00,0x00,0x00,0x00,0x01,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xf0,0x00,0x00,0x00,0x00,0x07,0xff
,0x00,0x00,0x00,0x00,0x00,0x0f,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xff,0xff,0xff,0xc0,0x00,0x00,0x00,0x00,0x3f,0xff
,0x00,0x00,0x00,0x00,0x00,0x1f,0xfc,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0xff,0xff,0xff,0x80,0x00,0x00,0x00,0x00,0xff,0xff
,0x00,0x00,0x00,0x00,0x00,0x3f,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0xff,0xff,0xfc,0x00,0x00,0x00,0x00,0x07,0xff,0xff
,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xf0,0x00,0x00,0x00,0x00,0x1f,0xff,0xff
,0x00,0x00,0x00,0x00,0x01,0xff,0xff,0xe0,0x00,0x00,0x00,0x00,0x00,0x07,0xff,0xff,0xff,0xc0,0x00,0x00,0x00,0x00,0x7f,0xff,0xff
,0x00,0x00,0x00,0x00,0x03,0xff,0xff,0xf0,0x00,0x00,0x00,0x00,0x00,0x3f,0xff,0xff,0xfe,0x00,0x00,0x00,0x00,0x01,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x07,0xff,0xff,0xfc,0x00,0x00,0x00,0x00,0x00,0x7f,0xff,0xff,0xf0,0x00,0x00,0x00,0x00,0x07,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x0f,0xff,0xff,0xfe,0x00,0x00,0x00,0x00,0x00,0x07,0xff,0xff,0xc0,0x00,0x00,0x00,0x00,0x0f,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x1f,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0xfe,0x00,0x00,0x00,0x00,0x00,0x3f,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0x3f,0xff,0xff,0xff,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x01,0xff,0xff,0xff,0xff,0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x03,0xff,0xff,0xff,0xff,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x0f,0xff,0xff,0xff,0xff,0xf8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7f,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x1f,0xff,0xff,0xff,0xff,0xfc,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0x7f,0xff,0xff,0xff,0xff,0xfe,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xff,0xff,0xff,0xff,0xff
,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xff,0xff,0xff,0xff,0xff
,0x00,0x00,0x03,0xff,0xff,0xff,0xff,0xff,0xff,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0xff,0xff,0xff,0xff,0xff
,0x00,0x00,0x07,0xff,0xff,0xff,0xff,0xff,0xff,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7f,0xff,0xff,0xff,0xff,0xff
,0x00,0x00,0x1f,0xff,0xff,0xff,0xff,0xff,0xff,0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xff,0xff,0xff,0xff,0xff,0xff
,0x00,0x00,0x3f,0xff,0xff,0xff,0xff,0xff,0xff,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xff,0xff,0xff,0xff,0xff,0xff
,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xf8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0xff,0xff,0xff,0xff,0xff,0xff
,0x00,0x01,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfe,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0xff,0xff,0xff,0xff,0xff,0xff
,0x00,0x07,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0x00,0x1f,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0x00,0x7f,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0x03,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0x1f,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xf8,0x00,0x00,0x00,0x00,0x00,0x01,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x07,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xc0,0x00,0x00,0x00,0x00,0x0f,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x80,0x00,0x00,0x07,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff

,
};

static const unsigned char Circle3232[] =  //Circle
{
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xE0,0x3F,0xFF,0xFF,0x80,0x07,0xFF,
	0xFE,0x00,0x03,0xFF,0xFC,0x00,0x01,0xFF,0xF8,0x00,0x00,0x7F,0xF0,0x0F,0x80,0x7F,
	0xE0,0x3F,0xE0,0x3F,0xE0,0x7F,0xF0,0x1F,0xC0,0xFF,0xF8,0x1F,0xC1,0xFF,0xFC,0x1F,
	0x81,0xFF,0xFC,0x0F,0x83,0xFF,0xFE,0x0F,0x83,0xFF,0xFE,0x0F,0x83,0xFF,0xFE,0x0F,
	0x83,0xFF,0xFE,0x0F,0x83,0xFF,0xFE,0x0F,0x81,0xFF,0xFC,0x0F,0xC1,0xFF,0xFC,0x1F,
	0xC0,0xFF,0xF8,0x1F,0xC0,0x7F,0xF8,0x1F,0xE0,0x3F,0xE0,0x3F,0xF0,0x0F,0x80,0x7F,
	0xF8,0x00,0x00,0x7F,0xF8,0x00,0x00,0xFF,0xFE,0x00,0x03,0xFF,0xFF,0x00,0x07,0xFF,
	0xFF,0xE0,0x3F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
};


static const unsigned char Line3232[] =  //Circle
{
	0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,
	0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,
	0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,
	0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,
	0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,
	0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,
	0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,
	0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,0xFF,0xFE,0x3F,0xFF,
};



/***********************************************************
						end file
***********************************************************/


