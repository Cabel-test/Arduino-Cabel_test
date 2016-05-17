#if defined(__AVR__)
	#include <avr/pgmspace.h>
	#define fontdatatype uint8_t
#elif defined(__PIC32MX__)
	#define PROGMEM
	#define fontdatatype const unsigned char
#elif defined(__arm__)
	#define PROGMEM
	#define fontdatatype const unsigned char
#endif

// SmallFont.c
// Modified by Alexey Stepanov (cyrillic symbols added)
// Font Size	: 8x12
// Memory usage	: 1720 bytes
// # characters	: 144
// offset       : 32

//fontdatatype SmallFont[1732] PROGMEM={
fontdatatype SmallRusFont[2008] PROGMEM={
//0x08,0x0C,0x20,0x8F,
0x08,0x0C,0x20,0xA6,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // <Space>
0x00,0x00,0x20,0x20,0x20,0x20,0x20,0x20,0x00,0x20,0x00,0x00, // !
0x00,0x28,0x50,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // "
0x00,0x00,0x28,0x28,0xFC,0x28,0x50,0xFC,0x50,0x50,0x00,0x00, // #
0x00,0x20,0x78,0xA8,0xA0,0x60,0x30,0x28,0xA8,0xF0,0x20,0x00, // $
0x00,0x00,0x48,0xA8,0xB0,0x50,0x28,0x34,0x54,0x48,0x00,0x00, // %
0x00,0x00,0x20,0x50,0x50,0x78,0xA8,0xA8,0x90,0x6C,0x00,0x00, // &
0x00,0x40,0x40,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // '
0x00,0x04,0x08,0x10,0x10,0x10,0x10,0x10,0x10,0x08,0x04,0x00, // (
0x00,0x40,0x20,0x10,0x10,0x10,0x10,0x10,0x10,0x20,0x40,0x00, // )
0x00,0x00,0x00,0x20,0xA8,0x70,0x70,0xA8,0x20,0x00,0x00,0x00, // *
0x00,0x00,0x20,0x20,0x20,0xF8,0x20,0x20,0x20,0x00,0x00,0x00, // +
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x40,0x80, // ,
0x00,0x00,0x00,0x00,0x00,0xF8,0x00,0x00,0x00,0x00,0x00,0x00, // -
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00, // .
0x00,0x08,0x10,0x10,0x10,0x20,0x20,0x40,0x40,0x40,0x80,0x00, // /
0x00,0x00,0x70,0x88,0x88,0x88,0x88,0x88,0x88,0x70,0x00,0x00, // 0
0x00,0x00,0x20,0x60,0x20,0x20,0x20,0x20,0x20,0x70,0x00,0x00, // 1
0x00,0x00,0x70,0x88,0x88,0x10,0x20,0x40,0x80,0xF8,0x00,0x00, // 2
0x00,0x00,0x70,0x88,0x08,0x30,0x08,0x08,0x88,0x70,0x00,0x00, // 3
0x00,0x00,0x10,0x30,0x50,0x50,0x90,0x78,0x10,0x18,0x00,0x00, // 4
0x00,0x00,0xF8,0x80,0x80,0xF0,0x08,0x08,0x88,0x70,0x00,0x00, // 5
0x00,0x00,0x70,0x90,0x80,0xF0,0x88,0x88,0x88,0x70,0x00,0x00, // 6
0x00,0x00,0xF8,0x90,0x10,0x20,0x20,0x20,0x20,0x20,0x00,0x00, // 7
0x00,0x00,0x70,0x88,0x88,0x70,0x88,0x88,0x88,0x70,0x00,0x00, // 8
0x00,0x00,0x70,0x88,0x88,0x88,0x78,0x08,0x48,0x70,0x00,0x00, // 9
0x00,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x20,0x00,0x00, // :
0x00,0x00,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x20,0x00, // ;
0x00,0x04,0x08,0x10,0x20,0x40,0x20,0x10,0x08,0x04,0x00,0x00, // <
0x00,0x00,0x00,0x00,0xF8,0x00,0x00,0xF8,0x00,0x00,0x00,0x00, // =
0x00,0x40,0x20,0x10,0x08,0x04,0x08,0x10,0x20,0x40,0x00,0x00, // >
0x00,0x00,0x70,0x88,0x88,0x10,0x20,0x20,0x00,0x20,0x00,0x00, // ?
0x00,0x00,0x70,0x88,0x98,0xA8,0xA8,0xB8,0x80,0x78,0x00,0x00, // @
0x00,0x00,0x20,0x20,0x30,0x50,0x50,0x78,0x48,0xCC,0x00,0x00, // A
0x00,0x00,0xF0,0x48,0x48,0x70,0x48,0x48,0x48,0xF0,0x00,0x00, // B
0x00,0x00,0x78,0x88,0x80,0x80,0x80,0x80,0x88,0x70,0x00,0x00, // C
0x00,0x00,0xF0,0x48,0x48,0x48,0x48,0x48,0x48,0xF0,0x00,0x00, // D
0x00,0x00,0xF8,0x48,0x50,0x70,0x50,0x40,0x48,0xF8,0x00,0x00, // E
0x00,0x00,0xF8,0x48,0x50,0x70,0x50,0x40,0x40,0xE0,0x00,0x00, // F
0x00,0x00,0x38,0x48,0x80,0x80,0x9C,0x88,0x48,0x30,0x00,0x00, // G
0x00,0x00,0xCC,0x48,0x48,0x78,0x48,0x48,0x48,0xCC,0x00,0x00, // H
0x00,0x00,0xF8,0x20,0x20,0x20,0x20,0x20,0x20,0xF8,0x00,0x00, // I
0x00,0x00,0x7C,0x10,0x10,0x10,0x10,0x10,0x10,0x90,0xE0,0x00, // J
0x00,0x00,0xEC,0x48,0x50,0x60,0x50,0x50,0x48,0xEC,0x00,0x00, // K
0x00,0x00,0xE0,0x40,0x40,0x40,0x40,0x40,0x44,0xFC,0x00,0x00, // L
0x00,0x00,0xD8,0xD8,0xD8,0xD8,0xA8,0xA8,0xA8,0xA8,0x00,0x00, // M
0x00,0x00,0xDC,0x48,0x68,0x68,0x58,0x58,0x48,0xE8,0x00,0x00, // N
0x00,0x00,0x70,0x88,0x88,0x88,0x88,0x88,0x88,0x70,0x00,0x00, // O
0x00,0x00,0xF0,0x48,0x48,0x70,0x40,0x40,0x40,0xE0,0x00,0x00, // P
0x00,0x00,0x70,0x88,0x88,0x88,0x88,0xE8,0x98,0x70,0x18,0x00, // Q
0x00,0x00,0xF0,0x48,0x48,0x70,0x50,0x48,0x48,0xEC,0x00,0x00, // R
0x00,0x00,0x78,0x88,0x80,0x60,0x10,0x08,0x88,0xF0,0x00,0x00, // S
0x00,0x00,0xF8,0xA8,0x20,0x20,0x20,0x20,0x20,0x70,0x00,0x00, // T
0x00,0x00,0xCC,0x48,0x48,0x48,0x48,0x48,0x48,0x30,0x00,0x00, // U
0x00,0x00,0xCC,0x48,0x48,0x50,0x50,0x30,0x20,0x20,0x00,0x00, // V
0x00,0x00,0xA8,0xA8,0xA8,0x70,0x50,0x50,0x50,0x50,0x00,0x00, // W
0x00,0x00,0xD8,0x50,0x50,0x20,0x20,0x50,0x50,0xD8,0x00,0x00, // X
0x00,0x00,0xD8,0x50,0x50,0x20,0x20,0x20,0x20,0x70,0x00,0x00, // Y
0x00,0x00,0xF8,0x90,0x10,0x20,0x20,0x40,0x48,0xF8,0x00,0x00, // Z
0x00,0x38,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x38,0x00, // [
0x00,0x40,0x40,0x40,0x20,0x20,0x10,0x10,0x10,0x08,0x00,0x00, // <Backslash>
0x00,0x70,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x70,0x00, // ]
0x00,0x20,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // ^
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFC, // _
0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // '
0x00,0x00,0x00,0x00,0x00,0x30,0x48,0x38,0x48,0x3C,0x00,0x00, // a
0x00,0x00,0xC0,0x40,0x40,0x70,0x48,0x48,0x48,0x70,0x00,0x00, // b
0x00,0x00,0x00,0x00,0x00,0x38,0x48,0x40,0x40,0x38,0x00,0x00, // c
0x00,0x00,0x18,0x08,0x08,0x38,0x48,0x48,0x48,0x3C,0x00,0x00, // d
0x00,0x00,0x00,0x00,0x00,0x30,0x48,0x78,0x40,0x38,0x00,0x00, // e
0x00,0x00,0x1C,0x20,0x20,0x78,0x20,0x20,0x20,0x78,0x00,0x00, // f
0x00,0x00,0x00,0x00,0x00,0x3C,0x48,0x30,0x40,0x78,0x44,0x38, // g
0x00,0x00,0xC0,0x40,0x40,0x70,0x48,0x48,0x48,0xEC,0x00,0x00, // h
0x00,0x00,0x20,0x00,0x00,0x60,0x20,0x20,0x20,0x70,0x00,0x00, // i
0x00,0x00,0x10,0x00,0x00,0x30,0x10,0x10,0x10,0x10,0x10,0xE0, // j
0x00,0x00,0xC0,0x40,0x40,0x5C,0x50,0x70,0x48,0xEC,0x00,0x00, // k
0x00,0x00,0xE0,0x20,0x20,0x20,0x20,0x20,0x20,0xF8,0x00,0x00, // l
0x00,0x00,0x00,0x00,0x00,0xF0,0xA8,0xA8,0xA8,0xA8,0x00,0x00, // m
0x00,0x00,0x00,0x00,0x00,0xF0,0x48,0x48,0x48,0xEC,0x00,0x00, // n
0x00,0x00,0x00,0x00,0x00,0x30,0x48,0x48,0x48,0x30,0x00,0x00, // o
0x00,0x00,0x00,0x00,0x00,0xF0,0x48,0x48,0x48,0x70,0x40,0xE0, // p
0x00,0x00,0x00,0x00,0x00,0x38,0x48,0x48,0x48,0x38,0x08,0x1C, // q
0x00,0x00,0x00,0x00,0x00,0xD8,0x60,0x40,0x40,0xE0,0x00,0x00, // r
0x00,0x00,0x00,0x00,0x00,0x78,0x40,0x30,0x08,0x78,0x00,0x00, // s
0x00,0x00,0x00,0x20,0x20,0x70,0x20,0x20,0x20,0x18,0x00,0x00, // t
0x00,0x00,0x00,0x00,0x00,0xD8,0x48,0x48,0x48,0x3C,0x00,0x00, // u
0x00,0x00,0x00,0x00,0x00,0xEC,0x48,0x50,0x30,0x20,0x00,0x00, // v
0x00,0x00,0x00,0x00,0x00,0xA8,0xA8,0x70,0x50,0x50,0x00,0x00, // w
0x00,0x00,0x00,0x00,0x00,0xD8,0x50,0x20,0x50,0xD8,0x00,0x00, // x
0x00,0x00,0x00,0x00,0x00,0xEC,0x48,0x50,0x30,0x20,0x20,0xC0, // y
0x00,0x00,0x00,0x00,0x00,0x78,0x10,0x20,0x20,0x78,0x00,0x00, // z
0x00,0x18,0x10,0x10,0x10,0x20,0x10,0x10,0x10,0x10,0x18,0x00, // {
0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10, // |
0x00,0x60,0x20,0x20,0x20,0x10,0x20,0x20,0x20,0x20,0x60,0x00, // }
0x40,0xA4,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // ~
0x00,0x00,0x30,0x48,0x48,0x30,0x00,0x00,0x00,0x00,0x00,0x00, // ������ 127	0x7F
0x00,0x00,0xF8,0x48,0x40,0x70,0x48,0x48,0x48,0xF0,0x00,0x00, // �	128	0x80
0x00,0x00,0xF8,0x48,0x40,0x40,0x40,0x40,0x40,0xE0,0x00,0x00, // �	129	0x81
0x00,0x00,0x38,0x48,0x48,0x48,0x48,0x48,0x48,0xFC,0x84,0x00, // �	130	0x82
0x50,0x00,0xF8,0x48,0x50,0x70,0x50,0x40,0x48,0xF8,0x00,0x00, // �	131	0x83
0x00,0x00,0xA8,0xA8,0xA8,0x70,0xF8,0xA8,0xA8,0xA8,0x00,0x00, // �	132	0x84
0x00,0x00,0xF0,0x88,0x08,0x30,0x08,0x08,0x08,0xF0,0x00,0x00, // �	133	0x85
0x00,0x00,0xEC,0x48,0x58,0x58,0x68,0x68,0x48,0xDC,0x00,0x00, // �	134	0x86
0x30,0x00,0xEC,0x48,0x58,0x58,0x68,0x68,0x48,0xDC,0x00,0x00, // �	135	0x87
0x00,0x00,0x3C,0x48,0x48,0x48,0x48,0x48,0x48,0xCC,0x00,0x00, // �	136	0x88
0x00,0x00,0xFC,0x48,0x48,0x48,0x48,0x48,0x48,0xCC,0x00,0x00, // �	137	0x89
0x00,0x00,0xD8,0x50,0x50,0x20,0x20,0x40,0x40,0x80,0x00,0x00, // �	138	0x8A
0x00,0x00,0x70,0xA8,0xA8,0xA8,0xA8,0x70,0x20,0x70,0x00,0x00, // �	139	0x8B
0x00,0x00,0xCC,0x48,0x48,0x48,0x48,0x48,0x48,0xFC,0x04,0x00, // �	140 0x8C
0x00,0x00,0xCC,0x48,0x48,0x48,0x38,0x08,0x08,0x1C,0x00,0x00, // �	141	0x8D
0x00,0x00,0xD6,0x54,0x54,0x54,0x54,0x54,0x54,0xFE,0x00,0x00, // �	142	0x8E
0x00,0x00,0xD6,0x54,0x54,0x54,0x54,0x54,0x54,0xFE,0x02,0x00, // �	143	0x8F
0x00,0x00,0xE0,0xC0,0x40,0x70,0x48,0x48,0x48,0xF0,0x00,0x00, // �	144	0x90
0x00,0x00,0xEE,0x44,0x44,0x64,0x54,0x54,0x54,0xEE,0x00,0x00, // �	145	0x91
0x00,0x00,0xE0,0x40,0x40,0x70,0x48,0x48,0x48,0xF0,0x00,0x00, // �	146	0x92
0x00,0x00,0xE0,0x90,0x08,0x38,0x08,0x08,0x90,0x60,0x00,0x00, // �	147	0x93
0x00,0x00,0xEC,0x52,0x52,0x52,0x72,0x52,0x52,0xEC,0x00,0x00, // �	148	0x94
0x00,0x00,0x3C,0x48,0x48,0x48,0x38,0x48,0x48,0x9C,0x00,0x00, // �	149	0x95
0x00,0x00,0x08,0x30,0x40,0x70,0x48,0x48,0x48,0x30,0x00,0x00, // �	150	0x96
0x00,0x00,0x00,0x00,0x00,0x70,0x28,0x30,0x28,0x70,0x00,0x00, // �	151	0x97
0x00,0x00,0x00,0x00,0x00,0x78,0x28,0x20,0x20,0x70,0x00,0x00, // �	152	0x98
0x00,0x00,0x00,0x00,0x00,0x18,0x28,0x28,0x28,0x7C,0x44,0x00, // �	153	0x99
0x00,0x00,0x00,0x28,0x00,0x30,0x48,0x78,0x40,0x30,0x00,0x00, // �	154	0x9A
0x00,0x00,0x00,0x00,0x00,0x54,0x54,0x38,0x54,0x54,0x00,0x00, // �	155	0x9B
0x00,0x00,0x00,0x00,0x00,0x70,0x08,0x30,0x08,0x70,0x00,0x00, // �	156	0x9C
0x00,0x00,0x00,0x00,0x00,0xCC,0x48,0x58,0x68,0xCC,0x00,0x00, // �	157	0x9D
0x00,0x00,0x00,0x30,0x00,0xCC,0x48,0x58,0x68,0xCC,0x00,0x00, // �	158	0x9E
0x00,0x00,0x00,0x00,0x00,0x48,0x50,0x60,0x50,0x48,0x00,0x00, // �	159	0x9F
0x00,0x00,0x00,0x00,0x00,0x1C,0x28,0x28,0x28,0x4C,0x00,0x00, // �	160	0xA0
0x00,0x00,0x00,0x00,0x00,0x44,0x6C,0x54,0x44,0x44,0x00,0x00, // �	161	0xA1
0x00,0x00,0x00,0x00,0x00,0xCC,0x48,0x78,0x48,0xCC,0x00,0x00, // �	162	0xA2
0x00,0x00,0x00,0x00,0x00,0xFC,0x48,0x48,0x48,0xCC,0x00,0x00, // �	163	0xA3
0x00,0x00,0x00,0x00,0x00,0x7C,0x54,0x10,0x10,0x38,0x00,0x00, // �	164	0xA4
0x00,0x00,0x00,0x00,0x00,0x38,0x54,0x54,0x54,0x38,0x10,0x38, // �	165	0xA5
0x00,0x00,0x00,0x00,0x00,0xCC,0x48,0x48,0x48,0xFC,0x04,0x00, // �	166	0xA6
0x00,0x00,0x00,0x00,0x00,0xCC,0x48,0x38,0x08,0x0C,0x00,0x00, // �	167	0xA7
0x00,0x00,0x00,0x00,0x00,0x54,0x54,0x54,0x54,0x7C,0x00,0x00, // �	168	0xA8
0x00,0x00,0x00,0x00,0x00,0x54,0x54,0x54,0x54,0x7E,0x02,0x00, // �	169	0xA9
0x00,0x00,0x00,0x00,0x00,0xE0,0xA0,0x30,0x28,0x70,0x00,0x00, // �	170	0xAA
0x00,0x00,0x00,0x00,0x00,0xC6,0x44,0x64,0x54,0xE6,0x00,0x00, // �	171	0xAB
0x00,0x00,0x00,0x00,0x00,0x60,0x20,0x30,0x28,0x70,0x00,0x00, // �	172	0xAC
0x00,0x00,0x00,0x00,0x00,0x30,0x48,0x18,0x48,0x30,0x00,0x00, // �	173	0xAD
0x00,0x00,0x00,0x00,0x00,0xC8,0x54,0x74,0x54,0xC8,0x00,0x00, // �	174	0xAE
0x00,0x00,0x00,0x00,0x00,0x3C,0x48,0x48,0x38,0x4C,0x00,0x00, // �	175	0xAF

0x3C,0xE7,0x81,0xBD,0x81,0xBD,0x81,0xBD,0x81,0xBD,0x81,0xFF, // BATT FULL	176	0xB0
0x3C,0xE7,0x81,0x81,0x81,0xBD,0x81,0xBD,0x81,0xBD,0x81,0xFF, // BATT 75%	177	0xB1
0x3C,0xE7,0x81,0x81,0x81,0x81,0x81,0xBD,0x81,0xBD,0x81,0xFF, // BATT 50%	178	0xB2
0x3C,0xE7,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0xBD,0x81,0xFF, // BATT 25%	179	0xB3
0x3C,0xE7,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0xFF, // BATT EMPTY	180	0xB4
0xFF,0xFF,0x00,0x3F,0x3F,0x00,0x0F,0x0F,0x00,0x03,0x03,0x00, // ������ 100%	181	0xB5
0x00,0x00,0x00,0x3F,0x3F,0x00,0x0F,0x0F,0x00,0x03,0x03,0x00, // ������ 75%	182	0xB6
0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x0F,0x00,0x03,0x03,0x00, // ������ 50%	183	0xB7
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x03,0x00, // ������ 25%	184	0xB8
0x00,0x00,0x10,0x10,0x10,0x10,0x10,0x54,0x38,0x10,0x00,0x00, // ������� �����	185	0xB9
0x00,0x00,0x10,0x38,0x54,0x10,0x10,0x10,0x10,0x10,0x00,0x00, // ������� ����	186	0xBA
0x00,0x00,0x00,0x00,0x10,0x08,0x7C,0x08,0x10,0x00,0x00,0x00, // ������� ������	187	0xBB
0x00,0x00,0x00,0x00,0x10,0x20,0x7C,0x20,0x10,0x00,0x00,0x00, // ������� �����	188	0xBC
0x00,0x00,0x00,0x3C,0x0C,0x14,0x24,0x44,0x80,0x00,0x00,0x00, // ������� ����� ������	189	0xBD
0x00,0x00,0x00,0x80,0x44,0x24,0x14,0x0C,0x3C,0x00,0x00,0x00, // ������� ���� ������	190	0xBE
0x00,0x00,0x3C,0x42,0xA5,0x81,0xA5,0x99,0x42,0x3C,0x00,0x00, // ������� �����	191	0xBF
0x00,0x00,0x3C,0x7E,0xDB,0xFF,0xFF,0xDB,0x66,0x3C,0x00,0x00, // ������� ���� 192	0xC0
0x00,0x00,0x3C,0x42,0xA5,0x81,0x99,0xA5,0x42,0x3C,0x00,0x00, // �������� ����� 	193	0xC1
0x00,0x00,0x3C,0x7E,0xDB,0xFF,0xFF,0xE7,0x5A,0x3C,0x00,0x00, // �������� ���� 194	0xC2
0x00,0x00,0x3C,0x42,0x81,0x81,0x81,0x81,0x42,0x3C,0x00,0x00, // ������ ����	195	0xC3
0x00,0x00,0x3C,0x7E,0xFF,0xFF,0xFF,0xFF,0x7E,0x3C,0x00,0x00, // ������ ���� 196	0xC4
0x00,0x00,0xFF,0x81,0x81,0x81,0x81,0x81,0x81,0xFF,0x00,0x00, // ������� ������ 197	0xC5
0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00, // ������� ������ 198	0xC6
};

// SmallSymbolFont.c (C) 2013 by Alexey Stepanov
// Font Size	: 8x12
// Memory usage	: 280 bytes
// # characters	: 23
// offset       : 32

fontdatatype SmallSymbolFont[280] PROGMEM={
0x08,0x0C,0x20,0x17,
0x3C,0xE7,0x81,0xBD,0x81,0xBD,0x81,0xBD,0x81,0xBD,0x81,0xFF, // BATT FULL	32
0x3C,0xE7,0x81,0x81,0x81,0xBD,0x81,0xBD,0x81,0xBD,0x81,0xFF, // BATT 75%	33
0x3C,0xE7,0x81,0x81,0x81,0x81,0x81,0xBD,0x81,0xBD,0x81,0xFF, // BATT 50%	34
0x3C,0xE7,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0xBD,0x81,0xFF, // BATT 25%	35
0x3C,0xE7,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0xFF, // BATT EMPTY	36
0xFF,0xFF,0x00,0x3F,0x3F,0x00,0x0F,0x0F,0x00,0x03,0x03,0x00, // ������ 100%	37
0x00,0x00,0x00,0x3F,0x3F,0x00,0x0F,0x0F,0x00,0x03,0x03,0x00, // ������ 75%	38
0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x0F,0x00,0x03,0x03,0x00, // ������ 50%	39
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x03,0x00, // ������ 25%	40
0x00,0x00,0x10,0x10,0x10,0x10,0x10,0x54,0x38,0x10,0x00,0x00, // ������� �����	41
0x00,0x00,0x10,0x38,0x54,0x10,0x10,0x10,0x10,0x10,0x00,0x00, // ������� ����	42
0x00,0x00,0x00,0x00,0x10,0x08,0x7C,0x08,0x10,0x00,0x00,0x00, // ������� ������	43
0x00,0x00,0x00,0x00,0x10,0x20,0x7C,0x20,0x10,0x00,0x00,0x00, // ������� �����	44
0x00,0x00,0x00,0x3C,0x0C,0x14,0x24,0x44,0x80,0x00,0x00,0x00, // ������� ����� ������	45
0x00,0x00,0x00,0x80,0x44,0x24,0x14,0x0C,0x3C,0x00,0x00,0x00, // ������� ���� ������	46
0x00,0x00,0x3C,0x42,0xA5,0x81,0xA5,0x99,0x42,0x3C,0x00,0x00, // ������� �����	47
0x00,0x00,0x3C,0x7E,0xDB,0xFF,0xFF,0xDB,0x66,0x3C,0x00,0x00, // ������� ���� 48
0x00,0x00,0x3C,0x42,0xA5,0x81,0x99,0xA5,0x42,0x3C,0x00,0x00, // �������� ����� 	49
0x00,0x00,0x3C,0x7E,0xDB,0xFF,0xFF,0xE7,0x5A,0x3C,0x00,0x00, // �������� ���� 50
0x00,0x00,0x3C,0x42,0x81,0x81,0x81,0x81,0x42,0x3C,0x00,0x00, // ������ ����	51
0x00,0x00,0x3C,0x7E,0xFF,0xFF,0xFF,0xFF,0x7E,0x3C,0x00,0x00, // ������ ���� 52
0x00,0x00,0xFF,0x81,0x81,0x81,0x81,0x81,0x81,0xFF,0x00,0x00, // ������� ������ 53
0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00, // ������� ������ 54
};

// BigFont.c (C)2010 by Henning Karlsen
// Font Size	: 16x16
// Memory usage	: 3044 bytes
// # characters	: 95

fontdatatype BigRusFont[4612] PROGMEM={
0x10,0x10,0x20,0x8F,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //  <Space>
0x00,0x00,0x00,0x00,0x07,0x00,0x0F,0x80,0x0F,0x80,0x0F,0x80,0x0F,0x80,0x0F,0x80,0x07,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x07,0x00,0x07,0x00,0x07,0x00,0x00,0x00, // !
0x00,0x00,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x06,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // "
0x00,0x00,0x0C,0x30,0x0C,0x30,0x0C,0x30,0x7F,0xFE,0x7F,0xFE,0x0C,0x30,0x0C,0x30,0x0C,0x30,0x0C,0x30,0x7F,0xFE,0x7F,0xFE,0x0C,0x30,0x0C,0x30,0x0C,0x30,0x00,0x00, // #
0x00,0x00,0x02,0x40,0x02,0x40,0x0F,0xF8,0x1F,0xF8,0x1A,0x40,0x1A,0x40,0x1F,0xF0,0x0F,0xF8,0x02,0x58,0x02,0x58,0x1F,0xF8,0x1F,0xF0,0x02,0x40,0x02,0x40,0x00,0x00, // $
0x00,0x00,0x00,0x00,0x00,0x00,0x0E,0x10,0x0E,0x30,0x0E,0x70,0x00,0xE0,0x01,0xC0,0x03,0x80,0x07,0x00,0x0E,0x70,0x0C,0x70,0x08,0x70,0x00,0x00,0x00,0x00,0x00,0x00, // %
0x00,0x00,0x00,0x00,0x0F,0x00,0x19,0x80,0x19,0x80,0x19,0x80,0x0F,0x00,0x0F,0x08,0x0F,0x98,0x19,0xF8,0x18,0xF0,0x18,0xE0,0x19,0xF0,0x0F,0x98,0x00,0x00,0x00,0x00, // &
0x00,0x00,0x00,0x00,0x07,0x00,0x07,0x00,0x07,0x00,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // '
0x00,0x00,0x00,0x00,0x00,0xF0,0x01,0xC0,0x03,0x80,0x07,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x07,0x00,0x03,0x80,0x01,0xC0,0x00,0xF0,0x00,0x00,0x00,0x00, // (
0x00,0x00,0x00,0x00,0x0F,0x00,0x03,0x80,0x01,0xC0,0x00,0xE0,0x00,0x70,0x00,0x70,0x00,0x70,0x00,0x70,0x00,0xE0,0x01,0xC0,0x03,0x80,0x0F,0x00,0x00,0x00,0x00,0x00, // )
0x00,0x00,0x00,0x00,0x01,0x80,0x11,0x88,0x09,0x90,0x07,0xE0,0x07,0xE0,0x3F,0xFC,0x3F,0xFC,0x07,0xE0,0x07,0xE0,0x09,0x90,0x11,0x88,0x01,0x80,0x00,0x00,0x00,0x00, // *
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x80,0x01,0x80,0x01,0x80,0x0F,0xF0,0x0F,0xF0,0x01,0x80,0x01,0x80,0x01,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // +
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x00,0x07,0x00,0x07,0x00,0x0E,0x00,0x00,0x00, // ,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0xF8,0x1F,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // -
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x00,0x07,0x00,0x07,0x00,0x00,0x00,0x00,0x00, // ,
0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x06,0x00,0x0E,0x00,0x1C,0x00,0x38,0x00,0x70,0x00,0xE0,0x01,0xC0,0x03,0x80,0x07,0x00,0x0E,0x00,0x1C,0x00,0x00,0x00,0x00,0x00, // /
0x00,0x00,0x00,0x00,0x0F,0xF0,0x1C,0x38,0x1C,0x78,0x1C,0xF8,0x1C,0xF8,0x1D,0xB8,0x1D,0xB8,0x1F,0x38,0x1F,0x38,0x1E,0x38,0x1C,0x38,0x0F,0xF0,0x00,0x00,0x00,0x00, // 0
0x00,0x00,0x00,0x00,0x01,0x80,0x01,0x80,0x03,0x80,0x1F,0x80,0x1F,0x80,0x03,0x80,0x03,0x80,0x03,0x80,0x03,0x80,0x03,0x80,0x03,0x80,0x1F,0xF0,0x00,0x00,0x00,0x00, // 1
0x00,0x00,0x00,0x00,0x0F,0xE0,0x1C,0x70,0x1C,0x38,0x00,0x38,0x00,0x70,0x00,0xE0,0x01,0xC0,0x03,0x80,0x07,0x00,0x0E,0x38,0x1C,0x38,0x1F,0xF8,0x00,0x00,0x00,0x00, // 2
0x00,0x00,0x00,0x00,0x0F,0xE0,0x1C,0x70,0x1C,0x38,0x00,0x38,0x00,0x70,0x03,0xC0,0x03,0xC0,0x00,0x70,0x00,0x38,0x1C,0x38,0x1C,0x70,0x0F,0xE0,0x00,0x00,0x00,0x00, // 3
0x00,0x00,0x00,0x00,0x00,0xE0,0x01,0xE0,0x03,0xE0,0x06,0xE0,0x0C,0xE0,0x18,0xE0,0x1F,0xF8,0x1F,0xF8,0x00,0xE0,0x00,0xE0,0x00,0xE0,0x03,0xF8,0x00,0x00,0x00,0x00, // 4
0x00,0x00,0x00,0x00,0x1F,0xF8,0x1C,0x00,0x1C,0x00,0x1C,0x00,0x1C,0x00,0x1F,0xE0,0x1F,0xF0,0x00,0x78,0x00,0x38,0x1C,0x38,0x1C,0x70,0x0F,0xE0,0x00,0x00,0x00,0x00, // 5
0x00,0x00,0x00,0x00,0x03,0xE0,0x07,0x00,0x0E,0x00,0x1C,0x00,0x1C,0x00,0x1F,0xF0,0x1F,0xF8,0x1C,0x38,0x1C,0x38,0x1C,0x38,0x1C,0x38,0x0F,0xF0,0x00,0x00,0x00,0x00, // 6
0x00,0x00,0x00,0x00,0x1F,0xFC,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x00,0x1C,0x00,0x38,0x00,0x70,0x00,0xE0,0x01,0xC0,0x03,0x80,0x03,0x80,0x03,0x80,0x00,0x00,0x00,0x00, // 7
0x00,0x00,0x00,0x00,0x0F,0xF0,0x1C,0x38,0x1C,0x38,0x1C,0x38,0x1F,0x38,0x07,0xE0,0x07,0xE0,0x1C,0xF8,0x1C,0x38,0x1C,0x38,0x1C,0x38,0x0F,0xF0,0x00,0x00,0x00,0x00, // 8
0x00,0x00,0x00,0x00,0x0F,0xF0,0x1C,0x38,0x1C,0x38,0x1C,0x38,0x1C,0x38,0x1F,0xF8,0x0F,0xF8,0x00,0x38,0x00,0x38,0x00,0x70,0x00,0xE0,0x07,0xC0,0x00,0x00,0x00,0x00, // 9
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x80,0x03,0x80,0x03,0x80,0x00,0x00,0x00,0x00,0x03,0x80,0x03,0x80,0x03,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // :
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x80,0x03,0x80,0x03,0x80,0x00,0x00,0x00,0x00,0x03,0x80,0x03,0x80,0x03,0x80,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // ;
0x00,0x00,0x00,0x70,0x00,0xE0,0x01,0xC0,0x03,0x80,0x07,0x00,0x0E,0x00,0x1C,0x00,0x1C,0x00,0x0E,0x00,0x07,0x00,0x03,0x80,0x01,0xC0,0x00,0xE0,0x00,0x70,0x00,0x00, // <
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3F,0xFC,0x3F,0xFC,0x00,0x00,0x00,0x00,0x3F,0xFC,0x3F,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // =
0x00,0x00,0x1C,0x00,0x0E,0x00,0x07,0x00,0x03,0x80,0x01,0xC0,0x00,0xE0,0x00,0x70,0x00,0x70,0x00,0xE0,0x01,0xC0,0x03,0x80,0x07,0x00,0x0E,0x00,0x1C,0x00,0x00,0x00, // >
0x00,0x00,0x03,0xC0,0x0F,0xF0,0x1E,0x78,0x18,0x38,0x00,0x38,0x00,0x70,0x00,0xE0,0x01,0xC0,0x01,0xC0,0x00,0x00,0x00,0x00,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x00,0x00, // ?
0x00,0x00,0x0F,0xF8,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0xFC,0x1C,0xFC,0x1C,0xFC,0x1C,0xFC,0x1C,0x00,0x1C,0x00,0x1C,0x00,0x1F,0xF0,0x07,0xF8,0x00,0x00, // @
0x00,0x00,0x00,0x00,0x03,0xC0,0x07,0xE0,0x0E,0x70,0x1C,0x38,0x1C,0x38,0x1C,0x38,0x1C,0x38,0x1F,0xF8,0x1C,0x38,0x1C,0x38,0x1C,0x38,0x1C,0x38,0x00,0x00,0x00,0x00, // A
0x00,0x00,0x00,0x00,0x1F,0xF0,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0F,0xF0,0x0F,0xF0,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x1F,0xF0,0x00,0x00,0x00,0x00, // B
0x00,0x00,0x00,0x00,0x07,0xF0,0x0E,0x38,0x1C,0x38,0x1C,0x00,0x1C,0x00,0x1C,0x00,0x1C,0x00,0x1C,0x00,0x1C,0x00,0x1C,0x38,0x0E,0x38,0x07,0xF0,0x00,0x00,0x00,0x00, // C
0x00,0x00,0x00,0x00,0x1F,0xE0,0x0E,0x70,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x70,0x1F,0xE0,0x00,0x00,0x00,0x00, // D
0x00,0x00,0x00,0x00,0x1F,0xF8,0x0E,0x18,0x0E,0x08,0x0E,0x00,0x0E,0x30,0x0F,0xF0,0x0F,0xF0,0x0E,0x30,0x0E,0x00,0x0E,0x08,0x0E,0x18,0x1F,0xF8,0x00,0x00,0x00,0x00, // E
0x00,0x00,0x00,0x00,0x1F,0xF8,0x0E,0x18,0x0E,0x08,0x0E,0x00,0x0E,0x30,0x0F,0xF0,0x0F,0xF0,0x0E,0x30,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x1F,0x00,0x00,0x00,0x00,0x00, // F
0x00,0x00,0x00,0x00,0x07,0xF0,0x0E,0x38,0x1C,0x38,0x1C,0x38,0x1C,0x00,0x1C,0x00,0x1C,0x00,0x1C,0xF8,0x1C,0x38,0x1C,0x38,0x0E,0x38,0x07,0xF8,0x00,0x00,0x00,0x00, // G
0x00,0x00,0x00,0x00,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1F,0xF0,0x1F,0xF0,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x00,0x00,0x00,0x00, // H
0x00,0x00,0x00,0x00,0x0F,0xE0,0x03,0x80,0x03,0x80,0x03,0x80,0x03,0x80,0x03,0x80,0x03,0x80,0x03,0x80,0x03,0x80,0x03,0x80,0x03,0x80,0x0F,0xE0,0x00,0x00,0x00,0x00, // I
0x00,0x00,0x00,0x00,0x01,0xFC,0x00,0x70,0x00,0x70,0x00,0x70,0x00,0x70,0x00,0x70,0x00,0x70,0x38,0x70,0x38,0x70,0x38,0x70,0x38,0x70,0x0F,0xE0,0x00,0x00,0x00,0x00, // J
0x00,0x00,0x00,0x00,0x1E,0x38,0x0E,0x38,0x0E,0x70,0x0E,0xE0,0x0F,0xC0,0x0F,0x80,0x0F,0x80,0x0F,0xC0,0x0E,0xE0,0x0E,0x70,0x0E,0x38,0x1E,0x38,0x00,0x00,0x00,0x00, // K
0x00,0x00,0x00,0x00,0x1F,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x08,0x0E,0x18,0x0E,0x38,0x1F,0xF8,0x00,0x00,0x00,0x00, // L
0x00,0x00,0x00,0x00,0x1C,0x1C,0x1E,0x3C,0x1F,0x7C,0x1F,0xFC,0x1F,0xFC,0x1D,0xDC,0x1C,0x9C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x00,0x00,0x00,0x00, // M
0x00,0x00,0x00,0x00,0x1C,0x1C,0x1C,0x1C,0x1E,0x1C,0x1F,0x1C,0x1F,0x9C,0x1D,0xDC,0x1C,0xFC,0x1C,0x7C,0x1C,0x3C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x00,0x00,0x00,0x00, // N
0x00,0x00,0x00,0x00,0x03,0xE0,0x07,0xF0,0x0E,0x38,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x0E,0x38,0x07,0xF0,0x03,0xE0,0x00,0x00,0x00,0x00, // O
0x00,0x00,0x00,0x00,0x1F,0xF0,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0F,0xF0,0x0F,0xF0,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x1F,0x00,0x00,0x00,0x00,0x00, // P
0x00,0x00,0x00,0x00,0x03,0xE0,0x0F,0x78,0x0E,0x38,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x7C,0x1C,0xFC,0x0F,0xF8,0x0F,0xF8,0x00,0x38,0x00,0xFC,0x00,0x00, // Q
0x00,0x00,0x00,0x00,0x1F,0xF0,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0F,0xF0,0x0F,0xF0,0x0E,0x70,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x1E,0x38,0x00,0x00,0x00,0x00, // R
0x00,0x00,0x00,0x00,0x0F,0xF0,0x1C,0x38,0x1C,0x38,0x1C,0x38,0x1C,0x00,0x0F,0xE0,0x07,0xF0,0x00,0x38,0x1C,0x38,0x1C,0x38,0x1C,0x38,0x0F,0xF0,0x00,0x00,0x00,0x00, // S
0x00,0x00,0x00,0x00,0x1F,0xFC,0x19,0xCC,0x11,0xC4,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x07,0xF0,0x00,0x00,0x00,0x00, // T
0x00,0x00,0x00,0x00,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x0F,0xE0,0x00,0x00,0x00,0x00, // U
0x00,0x00,0x00,0x00,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x0E,0xE0,0x07,0xC0,0x03,0x80,0x00,0x00,0x00,0x00, // V
0x00,0x00,0x00,0x00,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x9C,0x1C,0x9C,0x1C,0x9C,0x0F,0xF8,0x0F,0xF8,0x07,0x70,0x07,0x70,0x00,0x00,0x00,0x00, // W
0x00,0x00,0x00,0x00,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x0E,0xE0,0x07,0xC0,0x03,0x80,0x03,0x80,0x07,0xC0,0x0E,0xE0,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x00,0x00,0x00,0x00, // X
0x00,0x00,0x00,0x00,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x0E,0xE0,0x07,0xC0,0x03,0x80,0x03,0x80,0x03,0x80,0x03,0x80,0x0F,0xE0,0x00,0x00,0x00,0x00, // Y
0x00,0x00,0x00,0x00,0x1F,0xF8,0x1C,0x38,0x18,0x38,0x10,0x70,0x00,0xE0,0x01,0xC0,0x03,0x80,0x07,0x00,0x0E,0x08,0x1C,0x18,0x1C,0x38,0x1F,0xF8,0x00,0x00,0x00,0x00, // Z
0x00,0x00,0x00,0x00,0x07,0xF0,0x07,0x00,0x07,0x00,0x07,0x00,0x07,0x00,0x07,0x00,0x07,0x00,0x07,0x00,0x07,0x00,0x07,0x00,0x07,0x00,0x07,0xF0,0x00,0x00,0x00,0x00, // [
0x00,0x00,0x00,0x00,0x10,0x00,0x18,0x00,0x1C,0x00,0x0E,0x00,0x07,0x00,0x03,0x80,0x01,0xC0,0x00,0xE0,0x00,0x70,0x00,0x38,0x00,0x1C,0x00,0x07,0x00,0x00,0x00,0x00, // <Backslash>
0x00,0x00,0x00,0x00,0x07,0xF0,0x00,0x70,0x00,0x70,0x00,0x70,0x00,0x70,0x00,0x70,0x00,0x70,0x00,0x70,0x00,0x70,0x00,0x70,0x00,0x70,0x07,0xF0,0x00,0x00,0x00,0x00, // ]
0x00,0x00,0x01,0x80,0x03,0xC0,0x07,0xE0,0x0E,0x70,0x1C,0x38,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // ^
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7F,0xFF,0x7F,0xFF, // _
0x00,0x00,0x00,0x00,0x1C,0x00,0x1C,0x00,0x07,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // '
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0xE0,0x00,0x70,0x00,0x70,0x0F,0xF0,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x0F,0xD8,0x00,0x00,0x00,0x00, // a
0x00,0x00,0x00,0x00,0x1E,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x0F,0xF0,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x1B,0xF0,0x00,0x00,0x00,0x00, // b
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0xE0,0x1C,0x70,0x1C,0x70,0x1C,0x00,0x1C,0x00,0x1C,0x70,0x1C,0x70,0x0F,0xE0,0x00,0x00,0x00,0x00, // c
0x00,0x00,0x00,0x00,0x00,0xF8,0x00,0x70,0x00,0x70,0x00,0x70,0x0F,0xF0,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x0F,0xD8,0x00,0x00,0x00,0x00, // d
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0xE0,0x1C,0x70,0x1C,0x70,0x1F,0xF0,0x1C,0x00,0x1C,0x70,0x1C,0x70,0x0F,0xE0,0x00,0x00,0x00,0x00, // e
0x00,0x00,0x00,0x00,0x03,0xE0,0x07,0x70,0x07,0x70,0x07,0x00,0x07,0x00,0x1F,0xE0,0x1F,0xE0,0x07,0x00,0x07,0x00,0x07,0x00,0x07,0x00,0x1F,0xC0,0x00,0x00,0x00,0x00, // f
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0xD8,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x0F,0xF0,0x07,0xF0,0x00,0x70,0x1C,0x70,0x0F,0xE0, // g
0x00,0x00,0x00,0x00,0x1E,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x0E,0xF0,0x0F,0x38,0x0F,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x1E,0x38,0x00,0x00,0x00,0x00, // h
0x00,0x00,0x00,0x00,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x00,0x00,0x0F,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x0F,0xF8,0x00,0x00,0x00,0x00, // i
0x00,0x00,0x00,0x00,0x00,0x70,0x00,0x70,0x00,0x70,0x00,0x00,0x03,0xF0,0x00,0x70,0x00,0x70,0x00,0x70,0x00,0x70,0x00,0x70,0x00,0x70,0x1C,0x70,0x0C,0xF0,0x07,0xE0, // j
0x00,0x00,0x00,0x00,0x1E,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x38,0x0E,0x70,0x0E,0xE0,0x0F,0xC0,0x0E,0xE0,0x0E,0x70,0x0E,0x38,0x1E,0x38,0x00,0x00,0x00,0x00, // k
0x00,0x00,0x00,0x00,0x0F,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x0F,0xF8,0x00,0x00,0x00,0x00, // l
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0xF8,0x1C,0x9C,0x1C,0x9C,0x1C,0x9C,0x1C,0x9C,0x1C,0x9C,0x1C,0x9C,0x1C,0x9C,0x00,0x00,0x00,0x00, // m
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0xE0,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x00,0x00,0x00,0x00, // n
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0xE0,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x0F,0xE0,0x00,0x00,0x00,0x00, // o
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1B,0xF0,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0F,0xF0,0x0E,0x00,0x0E,0x00,0x1F,0x00, // p
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0xB0,0x38,0xE0,0x38,0xE0,0x38,0xE0,0x38,0xE0,0x38,0xE0,0x1F,0xE0,0x00,0xE0,0x00,0xE0,0x01,0xF0, // q
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1E,0xF0,0x0F,0xF8,0x0F,0x38,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x1F,0x00,0x00,0x00,0x00,0x00, // r
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0xE0,0x1C,0x30,0x1C,0x30,0x0F,0x80,0x03,0xE0,0x18,0x70,0x18,0x70,0x0F,0xE0,0x00,0x00,0x00,0x00, // s
0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x03,0x00,0x07,0x00,0x1F,0xF0,0x07,0x00,0x07,0x00,0x07,0x00,0x07,0x00,0x07,0x70,0x07,0x70,0x03,0xE0,0x00,0x00,0x00,0x00, // t
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x0F,0xD8,0x00,0x00,0x00,0x00, // u
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x0E,0xE0,0x07,0xC0,0x03,0x80,0x00,0x00,0x00,0x00, // v
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x9C,0x1C,0x9C,0x0F,0xF8,0x07,0x70,0x07,0x70,0x00,0x00,0x00,0x00, // w
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0xE0,0x1C,0xE0,0x0F,0xC0,0x07,0x80,0x07,0x80,0x0F,0xC0,0x1C,0xE0,0x1C,0xE0,0x00,0x00,0x00,0x00, // x
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x07,0xF0,0x03,0xE0,0x00,0xE0,0x01,0xC0,0x1F,0x80, // y
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0xE0,0x18,0xE0,0x11,0xC0,0x03,0x80,0x07,0x00,0x0E,0x20,0x1C,0x60,0x1F,0xE0,0x00,0x00,0x00,0x00, // z
0x00,0x00,0x00,0x00,0x01,0xF8,0x03,0x80,0x03,0x80,0x03,0x80,0x07,0x00,0x1C,0x00,0x1C,0x00,0x07,0x00,0x03,0x80,0x03,0x80,0x03,0x80,0x01,0xF8,0x00,0x00,0x00,0x00, // {
0x00,0x00,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x00,0x00, // |
0x00,0x00,0x00,0x00,0x1F,0x80,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x00,0xE0,0x00,0x38,0x00,0x38,0x00,0xE0,0x01,0xC0,0x01,0xC0,0x01,0xC0,0x1F,0x80,0x00,0x00,0x00,0x00, // }
0x00,0x00,0x00,0x00,0x1F,0x1C,0x3B,0x9C,0x39,0xDC,0x38,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // ~
0x00,0x00,0x00,0x00,0x00,0xF0,0x01,0xF8,0x01,0x98,0x01,0x98,0x01,0xF8,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // ������
0x00,0x00,0x00,0x00,0x1F,0xF8,0x0E,0x18,0x0E,0x08,0x0E,0x00,0x0E,0x00,0x0F,0xF0,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x1F,0xF0,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x1F,0xF8,0x0E,0x18,0x0E,0x08,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x1F,0x00,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x07,0xF0,0x0C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x3F,0xF8,0x30,0x18,0x20,0x08, // �
0x06,0x60,0x00,0x00,0x1F,0xF8,0x0E,0x18,0x0E,0x08,0x0E,0x00,0x0E,0x30,0x0F,0xF0,0x0F,0xF0,0x0E,0x30,0x0E,0x00,0x0E,0x08,0x0E,0x18,0x1F,0xF8,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x33,0xCC,0x19,0x98,0x19,0x98,0x19,0x98,0x0D,0xB0,0x0F,0xF0,0x0F,0xF0,0x1D,0xB8,0x19,0x98,0x19,0x98,0x19,0x98,0x33,0xCC,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x17,0xE0,0x1E,0x70,0x1C,0x38,0x10,0x38,0x00,0x70,0x03,0xC0,0x03,0xC0,0x00,0x70,0x00,0x38,0x1C,0x38,0x1C,0x70,0x0F,0xE0,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0xF0,0x1D,0xF0,0x1F,0x70,0x1E,0x70,0x1C,0x70,0x1C,0x70,0x00,0x00,0x00,0x00, // �
0x03,0x80,0x00,0x00,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0xF0,0x1D,0xF0,0x1F,0x70,0x1E,0x70,0x1C,0x70,0x1C,0x70,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x03,0xF0,0x06,0x70,0x0E,0x70,0x0E,0x70,0x0E,0x70,0x0E,0x70,0x0E,0x70,0x0E,0x70,0x0E,0x70,0x0E,0x70,0x0E,0x70,0x1C,0x70,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x1F,0xF0,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x0F,0xF0,0x07,0xF0,0x00,0x70,0x00,0x70,0x1C,0xE0,0x0F,0xC0,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x1F,0xF0,0x33,0x98,0x33,0x98,0x33,0x98,0x33,0x98,0x33,0x98,0x33,0x98,0x33,0x98,0x1F,0xF0,0x03,0x80,0x03,0x80,0x07,0xC0,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1F,0xF8,0x00,0x18,0x00,0x08, // �
0x00,0x00,0x00,0x00,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1F,0xF0,0x0F,0xF0,0x00,0x70,0x00,0x70,0x00,0x70,0x00,0x70,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x3F,0xFC,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x3F,0xFE,0x00,0x06,0x00,0x02, // �
0x00,0x00,0x00,0x00,0x3F,0x00,0x2E,0x00,0x2E,0x00,0x0E,0x00,0x0E,0x00,0x0F,0xF0,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x1F,0xF0,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x38,0x1C,0x38,0x1C,0x38,0x1C,0x38,0x1C,0x38,0x1C,0x3F,0x9C,0x39,0xDC,0x39,0xDC,0x39,0xDC,0x39,0xDC,0x39,0xDC,0x3F,0x9C,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x1F,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x0F,0xF0,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x1F,0xF0,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x0F,0xE0,0x1C,0x70,0x1C,0x38,0x00,0x38,0x00,0x38,0x03,0xF8,0x03,0xF8,0x00,0x38,0x00,0x38,0x1C,0x38,0x1C,0x70,0x0F,0xE0,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x3C,0xF8,0x1D,0x9C,0x1D,0x9C,0x1D,0x9C,0x1D,0x9C,0x1F,0x9C,0x1F,0x9C,0x1D,0x9C,0x1D,0x9C,0x1D,0x9C,0x1D,0x9C,0x3C,0xF8,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x0F,0xF8,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1F,0xF0,0x0F,0xF0,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x38,0x78,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x18,0x07,0xF8,0x0F,0x00,0x0E,0x00,0x0E,0x00,0x0F,0xF0,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x07,0xF0,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0xF0,0x0E,0x38,0x0E,0x38,0x0F,0xF0,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x1F,0xF0,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0xF8,0x1C,0x38,0x1C,0x00,0x1C,0x00,0x1C,0x00,0x1C,0x00,0x1C,0x00,0x1C,0x00,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xF0,0x0E,0x70,0x0E,0x70,0x0E,0x70,0x0E,0x70,0x0E,0x70,0x0E,0x70,0x1F,0xF8,0x10,0x08,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0xC0,0x00,0x00,0x0F,0xE0,0x1C,0x70,0x1C,0x70,0x1F,0xF0,0x1C,0x00,0x1C,0x70,0x1C,0x70,0x0F,0xE0,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x19,0xCC,0x19,0xCC,0x19,0xCC,0x0D,0xD8,0x07,0xF0,0x0D,0xD8,0x19,0xCC,0x19,0xCC,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0xE0,0x1C,0x70,0x00,0x70,0x03,0xE0,0x00,0x70,0x00,0x70,0x1C,0x70,0x0F,0xE0,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0xF0,0x1D,0x70,0x1E,0x70,0x1C,0x70,0x1C,0x70,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xC0,0x00,0x00,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0E,0x78,0x0E,0xB8,0x0F,0x38,0x0E,0x38,0x0E,0x38,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0E,0x38,0x0E,0x38,0x0E,0x70,0x0F,0xE0,0x0E,0x70,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xF8,0x07,0x38,0x07,0x38,0x07,0x38,0x07,0x38,0x07,0x38,0x07,0x38,0x0E,0x38,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x38,0x38,0x38,0x38,0x38,0x38,0x3C,0x78,0x3A,0xB8,0x39,0x38,0x38,0x38,0x38,0x38,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1F,0xF0,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0xF0,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0xF0,0x13,0x90,0x03,0x80,0x03,0x80,0x03,0x80,0x03,0x80,0x03,0x80,0x03,0x80,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x80,0x1F,0xF0,0x3B,0xB8,0x3B,0xB8,0x3B,0xB8,0x3B,0xB8,0x0F,0xF0,0x03,0x80,0x03,0x80,0x03,0x80,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1F,0xF8,0x00,0x18,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x0F,0xF0,0x00,0x70,0x00,0x70,0x00,0x70,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x3F,0xFC,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x3F,0xFE,0x00,0x06,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3E,0x00,0x2E,0x00,0x0E,0x00,0x0F,0xF0,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0F,0xF0,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x38,0x1C,0x38,0x1C,0x38,0x1C,0x3F,0x9C,0x39,0xDC,0x39,0xDC,0x39,0xDC,0x3F,0x9C,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0E,0x00,0x0E,0x00,0x0E,0x00,0x0F,0xF0,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x0F,0xF0,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0xE0,0x1C,0x70,0x00,0x70,0x03,0xF0,0x00,0x70,0x00,0x70,0x1C,0x70,0x0F,0xE0,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x38,0xF8,0x39,0x9C,0x39,0x9C,0x39,0x9C,0x3F,0x9C,0x39,0x9C,0x39,0x9C,0x38,0xF8,0x00,0x00,0x00,0x00, // �
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xF8,0x0E,0x38,0x0E,0x38,0x0E,0x38,0x07,0xF8,0x0E,0x38,0x0E,0x38,0x1C,0x38,0x00,0x00,0x00,0x00, // �
};
