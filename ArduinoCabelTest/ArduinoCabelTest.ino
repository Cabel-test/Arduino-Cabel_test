//#include <avr/io.h>
#include "Globals.h"                             // ���������� ���������
#include <SPI.h>
#include <SdFat.h>
#include <SdFatUtil.h>
#include <Wire.h> 
#include <RTClib.h>
#include <MsTimer2.h> 
#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>
#include "MCP23017.h"   
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <stdlib.h> // div, div_t
#include <UTFT.h>
#include <UTouch.h>
#include <UTFT_Buttons.h>
#include <Arduino.h>
#include "AnalogBinLogger.h"

#define led_Green 13                                     // ��������� �� �������� ������ �������
#define led_Red   12                                     // ��������� �� �������� ������ �������
#define led_disp  9                                      // ��������� �� �������� ������ ����������
#define led_instr 10                                     // ��������� �� �������� ������ �����������

MCP23017 mcp_Out1;                                       // ���������� ������ ���������� MCP23017  4 A - Out, B - Out
MCP23017 mcp_Out2;                                       // ���������� ������ ���������� MCP23017  6 A - Out, B - Out





//+++++++++++++++++++ MODBUS ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

modbusDevice regBank;
modbusSlave slave;

//+++++++++++++++++++++++ ��������� ������������ ��������� +++++++++++++++++++++++++++++++++++++
byte resistance          = 0x00;                        // ������������� 0x00..0xFF - 0��..100���

//+++++++++++++++++++++++++++++ ������� ������ +++++++++++++++++++++++++++++++++++++++
int deviceaddress        = 80;                          // ����� ���������� ������
unsigned int eeaddress   =  0;                          // ����� ������ ������
byte hi;                                                // ������� ���� ��� �������������� �����
byte low;                                               // ������� ���� ��� �������������� �����

//********************* ��������� �������� ***********************************
//UTFT        myGLCD(ITDB32S,38,39,40,41);              // ������� 3.2"
UTFT          myGLCD(ITDB24E_8,38,39,40,41);            // ������� 2.4" !! ��������! �������� ��������� UTouchCD.h
UTouch        myTouch(6,5,4,3,2);                       // Standard Arduino Mega/Due shield            : 6,5,4,3,2
UTFT_Buttons  myButtons(&myGLCD, &myTouch);             // Finally we set up UTFT_Buttons :)

boolean default_colors = true;                          // 
uint8_t menu_redraw_required = 0;
                                                        // Declare which fonts we will be using
extern uint8_t SmallFont[];                            
extern uint8_t BigFont[];
extern uint8_t Dingbats1_XL[];
extern uint8_t SmallSymbolFont[];

//+++++++++++++++++++++++++++ ��������� ����� +++++++++++++++++++++++++++++++
uint8_t second = 0;                                    //Initialization time
uint8_t minute = 10;
uint8_t hour   = 10;
uint8_t dow    = 2;
uint8_t day    = 15;
uint8_t month  = 3;
uint16_t year  = 16;
RTC_DS1307 RTC;                                       // define the Real Time Clock object

int clockCenterX               = 119;
int clockCenterY               = 119;
int oldsec                     = 0;
const char* str[]              = {"MON","TUE","WED","THU","FRI","SAT","SUN"};
const char* str1[]             = {"Monday","Tuesday","Wednesday","Thursday","Friday","Saturday","Sunday"};
const char* str_mon[]          = {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
unsigned long wait_time        = 0;                               // ����� ������� �������
unsigned long wait_time_Old    = 0;                               // ����� ������� �������
int time_minute                = 5;                               // ����� ������� �������
//------------------------------------------------------------------------------

const unsigned int adr_control_command    PROGMEM       = 40001;  // ����� �������� ������� �� ���������� 
const unsigned int adr_reg_count_err      PROGMEM       = 40002;  // ����� �������� ���� ������
//-------------------------------------------------------------------------------------------------------

//++++++++++++++++++++++++++++ ���������� ��� �������� ���������� +++++++++++++++++++++++++++++
int x, y, z;
char stCurrent[20]    ="";                                        // ���������� �������� ��������� ������ 
int stCurrentLen      =0;                                         // ���������� �������� ����� ��������� ������ 
int stCurrentLen1     =0;                                         // ���������� ���������� �������� ����� ��������� ������  
char stLast[20]       ="";                                        // ������ � ��������� ������ ������.
int ret               = 0;                                        // ������� ���������� ��������
//-------------------------------------------------------------------------------------------------

//++++++++++++++++++++++++++ ��������� ������������  +++++++++++++++++++++++++++++++++++++++++++++++++++

int dgvh;
int OldSample_osc[254][2];
int x_osc,y_osc;
int mode = 0;
int dTime = 1;
int tmode = 1;
int mode1 = 2;                                                     //������������ ����������������
int Trigger = 0;
int MinAnalog = 500;
int MinAnalog0 = 500;
int MinAnalog1 = 500;
int MaxAnalog = 0;
int MaxAnalog0 = 0;
int MaxAnalog1 = 0;
float koeff_h = 7.759*4;
volatile unsigned int Sample_osc[254][2];
float StartSample = 0; 
float EndSample = 0;
int t_in_mode = 0;
bool strob_start = true;

int Channel_x = 0;
int Channel_trig = 0;
bool Channel0 = true;
bool Channel1 = false;
bool Channel2 = false;
bool Channel3 = false;

int count_pin = 0;
int set_strob = 100;
bool Set_x = false;
bool osc_line_off0 = false;
bool osc_line_off1 = false;
bool osc_line_off2 = false;
bool osc_line_off3 = false;
const int hpos = 95; //set 0v on horizontal  grid
bool sled = false;
bool repeat = false;
int16_t count_repeat = 0;
volatile bool ADC_end = false;
volatile int ADC_i = 0;

volatile uint16_t MyBuff[254];  
volatile uint16_t i_osc=0; 


//------------------------------------------------------------------------------
// Analog pin number list for a sample.  Pins may be in any order and pin
// numbers may be repeated.
//const uint8_t PIN_LIST[] = {0, 1, 2, 3, 4};
const uint8_t PIN_LIST[] = {14};

//------------------------------------------------------------------------------
// Sample rate in samples per second.
const float SAMPLE_RATE = 5000;  // Must be 0.25 or greater.

// The interval between samples in seconds, SAMPLE_INTERVAL, may be set to a
// constant instead of being calculated from SAMPLE_RATE.  SAMPLE_RATE is not
// used in the code below.  For example, setting SAMPLE_INTERVAL = 2.0e-4
// will result in a 200 microsecond sample interval.
const float SAMPLE_INTERVAL = 1.0/SAMPLE_RATE;

// Setting ROUND_SAMPLE_INTERVAL non-zero will cause the sample interval to
// be rounded to a a multiple of the ADC clock period and will reduce sample
// time jitter.
#define ROUND_SAMPLE_INTERVAL 1
//------------------------------------------------------------------------------
// ADC clock rate.
// The ADC clock rate is normally calculated from the pin count and sample
// interval.  The calculation attempts to use the lowest possible ADC clock
// rate.
//
// You can select an ADC clock rate by defining the symbol ADC_PRESCALER to
// one of these values.  You must choose an appropriate ADC clock rate for
// your sample interval. 
// #define ADC_PRESCALER 7 // F_CPU/128 125 kHz on an Uno
// #define ADC_PRESCALER 6 // F_CPU/64  250 kHz on an Uno
// #define ADC_PRESCALER 5 // F_CPU/32  500 kHz on an Uno
// #define ADC_PRESCALER 4 // F_CPU/16 1000 kHz on an Uno
// #define ADC_PRESCALER 3 // F_CPU/8  2000 kHz on an Uno (8-bit mode only)
//------------------------------------------------------------------------------
// Reference voltage.  See the processor data-sheet for reference details.
// uint8_t const ADC_REF = 0; // External Reference AREF pin.
uint8_t const ADC_REF = (1 << REFS0);  // Vcc Reference.
// uint8_t const ADC_REF = (1 << REFS1);  // Internal 1.1 (only 644 1284P Mega)
// uint8_t const ADC_REF = (1 << REFS1) | (1 << REFS0);  // Internal 1.1 or 2.56
//------------------------------------------------------------------------------
// File definitions.
//
// Maximum file size in blocks.
// The program creates a contiguous file with FILE_BLOCK_COUNT 512 byte blocks.
// This file is flash erased using special SD commands.  The file will be
// truncated if logging is stopped early.
const uint32_t FILE_BLOCK_COUNT = 256000;

// log file base name.  Must be six characters or less.
//#define FILE_BASE_NAME "ANALOG"

// Set RECORD_EIGHT_BITS non-zero to record only the high 8-bits of the ADC.
#define RECORD_EIGHT_BITS 0
//------------------------------------------------------------------------------
// Pin definitions.
//
// Digital pin to indicate an error, set to -1 if not used.
// The led blinks for fatal errors. The led goes on solid for SD write
// overrun errors and logging continues.
const int8_t ERROR_LED_PIN = 3;

// SD chip select pin.
const uint8_t SD_CS_PIN = SS;
//------------------------------------------------------------------------------
// Buffer definitions.
//
// The logger will use SdFat's buffer plus BUFFER_BLOCK_COUNT additional 
// buffers.  QUEUE_DIM must be a power of two larger than
//(BUFFER_BLOCK_COUNT + 1).
//
#if RAMEND < 0X8FF
#error Too little SRAM
//
#elif RAMEND < 0X10FF
// Use total of two 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 1;
// Dimension for queues of 512 byte SD blocks.
const uint8_t QUEUE_DIM = 4;  // Must be a power of two!
//
#elif RAMEND < 0X20FF
// Use total of five 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 4;
// Dimension for queues of 512 byte SD blocks.
const uint8_t QUEUE_DIM = 8;  // Must be a power of two!
//
#elif RAMEND < 0X40FF
// Use total of 13 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 12;
// Dimension for queues of 512 byte SD blocks.
const uint8_t QUEUE_DIM = 16;  // Must be a power of two!
//
#else  // RAMEND
// Use total of 29 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 28;
// Dimension for queues of 512 byte SD blocks.
const uint8_t QUEUE_DIM = 32;  // Must be a power of two!
#endif  // RAMEND
//==============================================================================
// End of configuration constants.
//==============================================================================
// Temporary log file.  Will be deleted if a reset or power failure occurs.
#define TMP_FILE_NAME "TMP_LOG.BIN"

// Size of file base name.  Must not be larger than six.
//const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;

// Number of analog pins to log.
const uint8_t PIN_COUNT = sizeof(PIN_LIST)/sizeof(PIN_LIST[0]);

// Minimum ADC clock cycles per sample interval
const uint16_t MIN_ADC_CYCLES = 15;

// Extra cpu cycles to setup ADC with more than one pin per sample.
const uint16_t ISR_SETUP_ADC = 100;

// Maximum cycles for timer0 system interrupt, millis, micros.
const uint16_t ISR_TIMER0 = 160;
//==============================================================================
//SdFat sd;

//SdBaseFile binFile;

//char binName[13] = FILE_BASE_NAME "00.BIN";

#if RECORD_EIGHT_BITS
const size_t SAMPLES_PER_BLOCK = DATA_DIM8/PIN_COUNT;
typedef block8_t block_t;
#else  // RECORD_EIGHT_BITS
const size_t SAMPLES_PER_BLOCK = DATA_DIM16/PIN_COUNT;
typedef block16_t block_t;
#endif // RECORD_EIGHT_BITS

block_t* emptyQueue[QUEUE_DIM];
uint8_t emptyHead;
uint8_t emptyTail;

block_t* fullQueue[QUEUE_DIM];
volatile uint8_t fullHead;  // volatile insures non-interrupt code sees changes.
uint8_t fullTail;

// queueNext assumes QUEUE_DIM is a power of two
inline uint8_t queueNext(uint8_t ht) {return (ht + 1) & (QUEUE_DIM -1);}
//==============================================================================
// Interrupt Service Routines

// Pointer to current buffer.
block_t* isrBuf;

// Need new buffer if true.
bool isrBufNeeded = true;

// overrun count
uint16_t isrOver = 0;

// ADC configuration for each pin.
uint8_t adcmux[PIN_COUNT];
uint8_t adcsra[PIN_COUNT];
uint8_t adcsrb[PIN_COUNT];
uint8_t adcindex = 1;

// Insure no timer events are missed.
volatile bool timerError = false;
volatile bool timerFlag = false;
//------------------------------------------------------------------------------











//-------------------------------------------------------------------------------------------------------
//���������� ���������� ��� �������� � ����� ���� (������)
int but1, but2, but3, but4, but5, but6, but7, but8, but9, but10, butX, butY, butA, butB, butC, butD, but_m1, but_m2, but_m3, but_m4, but_m5, pressed_button;
 //int kbut1, kbut2, kbut3, kbut4, kbut5, kbut6, kbut7, kbut8, kbut9, kbut0, kbut_save,kbut_clear, kbut_exit;
 //int kbutA, kbutB, kbutC, kbutD, kbutE, kbutF;
 int m2 = 1; // ���������� ������ ����

 //------------------------------------------------------------------------------------------------------------------
 // ���������� ���������� ��� �������� �������

char  txt_menu1_1[]            = "Tec\xA4 ""\x9F""a\x96""e\xA0\xAF N1";                                    // ���� ������ N 1
char  txt_menu1_2[]            = "Tec\xA4 ""\x9F""a\x96""e\xA0\xAF N2";                                    // ���� ������ N 2
char  txt_menu1_3[]            = "Tec\xA4 ""\x9F""a\x96""e\xA0\xAF N3";                                    // ���� ������ N 3
char  txt_menu1_4[]            = "Tec\xA4 ""\x9F""a\x96""e\xA0\xAF N4";                                    // ���� ������ N 4
char  txt_menu2_1[]            = "\x89""a""\xA2""e""\xA0\xAC"" ""\x98""ap""\xA2\x9D\xA4""yp";              // ������ ��������                                                // 
char  txt_menu2_2[]            = "menu2_2";                                                                //
char  txt_menu2_3[]            = "menu2_3";                                                                //
char  txt_menu2_4[]            = "Tec""\xA4"" pa""\x9C\xAA""e""\xA1""o""\x97";                             // ���� ��������                                   //
char  txt_menu3_1[]            = "Ta""\x96\xA0\x9D\xA6""a coe""\x99"".";                                   // ������� ����.
char  txt_menu3_2[]            = "Pe""\x99""a""\x9F\xA4"". ""\xA4""a""\x96\xA0\x9D\xA6";                   // ������. ������
char  txt_menu3_3[]            = "\x85""a""\x98""py""\x9C"". y""\xA1""o""\xA0\xA7"".";                     // ������. �����.
char  txt_menu3_4[]            = "Bpe""\xA1\xAF"" ""\xA3""poc""\xA4""o""\xAF";                             // ����� �������
char  txt_menu4_1[]            = "C\x9D\xA2yco\x9D\x99""a";                                                // ���������
char  txt_menu4_2[]            = "\x89\x9D\xA0oo\x96pa\x9C\xA2\xAB\x9E";                                   // ������������
char  txt_menu4_3[]            = "Tpey\x98o\xA0\xAC\xA2\xAB\x9E";                                          // �����������
char  txt_menu4_4[]            = "\x89p\xAF\xA1oy\x98o\xA0\xAC\xA2\xAB\x9E";                               // �������������
char  txt_menu5_1[]            = " ";                                                                      // 
char  txt_menu5_2[]            = " ";                                                                      //
char  txt_menu5_3[]            = " ";                                                                      // 
char  txt_menu5_4[]            = " ";                                                                      // 
char  txt_osc_menu1[]          = "Oc\xA6\x9D\xA0\xA0o\x98pa\xA5";                                          //
char  txt_osc_menu2[]          = "Log data";                                                                       //
char  txt_osc_menu3[]          = "test_ADC";                                                                       //
char  txt_osc_menu4[]          = "";                                                                       //          

const char  txt_pass_ok[]           PROGMEM  = " ";                                                                      //  
const char  txt_pass_no[]           PROGMEM  = " ";                                                                      //  
const char  txt_info1[]             PROGMEM  = "Tec\xA4 ""\x9F""a\x96""e\xA0""e\x9E";                                    // ���� �������
const char  txt_info2[]             PROGMEM  = "Tec\xA4 \x96\xA0o\x9F""a \x98""ap\xA2\x9D\xA4yp";                        // ���� ����� ��������
const char  txt_info3[]             PROGMEM  = "Hac\xA4po\x9E\x9F""a c\x9D""c\xA4""e\xA1\xAB";                           // ��������� �������
const char  txt_info4[]             PROGMEM  = "\x81""e\xA2""epa\xA4op c\x9D\x98\xA2""a\xA0o\x97";                       // ��������� ��������
const char  txt_info5[]             PROGMEM  = "Oc\xA6\x9D\xA0\xA0o\x98pa\xA5";                                          // �����������
const char  txt_botton_clear[]      PROGMEM  = " ";                                                                      //  
const char  txt_botton_otmena[]     PROGMEM  = "O""\xA4\xA1""e""\xA2""a";                                                // "������" 
const char  txt_botton_vvod[]       PROGMEM  = "B\x97o\x99 ";                                                            // ����
const char  txt_botton_ret[]        PROGMEM  = "B""\xAB""x";                                                             // "���"
const char  txt_system_clear3[]     PROGMEM  = " ";                                                                      //  
const char  txt9[]                  PROGMEM  = " ";                                                                      // 
const char  txt10[]                 PROGMEM  = " ";                                                                      //  
const char  txt_time_wait[]         PROGMEM  = "\xA1\x9D\xA2"".""\x97""pe""\xA1\xAF"" ""\xA3""poc""\xA4""o""\xAF";       //  ���. ����� �������
const char  txt_info29[]            PROGMEM  = "Stop->PUSH Disp"; 
const char  txt_info30[]            PROGMEM  = " "; 
const char  txt_test_all[]          PROGMEM  = "Tec""\xA4"" ""\x97""cex pa""\x9C\xAA""e""\xA1""o""\x97";                 // ���� ���� ��������
const char  txt_test_all_exit1[]    PROGMEM  = "\x82\xA0\xAF"" ""\x97\xAB""xo""\x99""a";                                 // ��� ������
const char  txt_test_all_exit2[]    PROGMEM  = "\xA3""p""\x9D\x9F""oc""\xA2\x9D""c""\xAC"" ""\x9F"" ""\xAD\x9F""pa""\xA2""y";  // ���������� � ������
const char  txt_test_end[]          PROGMEM  = "\x85""a""\x97""ep""\xA8\x9D\xA4\xAC";                                    // ���������
const char  txt_test_repeat[]       PROGMEM  = "\x89""o""\x97\xA4""op""\x9D\xA4\xAC";                                    // ���������
const char  txt_error_connect1[]    PROGMEM  = "O""\x8E\x86\x80""KA";                                                    // ������
const char  txt_error_connect2[]    PROGMEM  = "\xA3""o""\x99\x9F\xA0\xAE\xA7""e""\xA2\x9D\xAF"" ""\x9F""a""\x96""e""\xA0\xAF"; //����������� ������
const char  txt_error_connect3[]    PROGMEM  = "O""\xA8\x9D\x96""o""\x9F"" ""\xA2""e""\xA4";                             // ������ ���
const char  txt_error_connect4[]    PROGMEM  = "O""\xA8\x9D\x96""o""\x9F"" -         ";                                  // ������  - 
const char  txt__connect1[]         PROGMEM  = "O""\x96\xA2""apy""\x9B""e""\xA2"" ""\x9F""a""\x96""e""\xA0\xAC"" N1";    // ��������� ������ N1
const char  txt__connect2[]         PROGMEM  = "O""\x96\xA2""apy""\x9B""e""\xA2"" ""\x9F""a""\x96""e""\xA0\xAC"" N2";    // ��������� ������ N2
const char  txt__connect3[]         PROGMEM  = "O""\x96\xA2""apy""\x9B""e""\xA2"" ""\x9F""a""\x96""e""\xA0\xAC"" N3";    // ��������� ������ N3
const char  txt__connect4[]         PROGMEM  = "O""\x96\xA2""apy""\x9B""e""\xA2"" ""\x9F""a""\x96""e""\xA0\xAC"" N4";    // ��������� ������ N4
const char  txt__test_end[]         PROGMEM  = "TECT ""\x85""A""KOH""\x8D""EH";                                          // ���� ��������
const char  txt__panel[]            PROGMEM  = "Tec""\xA4"" c""\x97""e""\xA4""o""\x99\x9D""o""\x99""o""\x97";            // ���� �����������
const char  txt__panel0[]           PROGMEM  = "                     ";                                                  // 
const char  txt__disp[]             PROGMEM  = "Tec""\xA4"" MT""\x81"" ""\x99\x9D""c""\xA3""e""\xA4\xA7""epa";           // ���� ��� ����������
const char  txt__instr[]            PROGMEM  = "Tec""\xA4"" MT""\x81"" ""\x9D\xA2""c""\xA4""py""\x9F\xA4""opa";          // ���� ��� �����������  
const char  txt__MTT[]              PROGMEM  = "Tec""\xA4"" MTT";                                                        // ���� ���
const char  txt__disp_connect[]     PROGMEM  = "Ka""\x96""e""\xA0\xAC"" ""\x99\x9D""c""\xA3"". ""\xA3""o""\x99\x9F\xA0"".";// ������ ����. �����.
const char  txt__disp_disconnect[]  PROGMEM  = "Ka""\x96""e""\xA0\xAC"" ""\x99\x9D""c""\xA3"". o""\xA4\x9F\xA0"".";        // ������ ����.����.
const char  txt__instr_connect[]    PROGMEM  = "Ka""\x96""e""\xA0\xAC"" ""\x9D\xA2""c""\xA4""p.""\xA3""o""\x99\x9F\xA0"".";// ������ �����.�����.
const char  txt__instr_disconnect[] PROGMEM  = "Ka""\x96""e""\xA0\xAC"" ""\x9D\xA2""c""\xA4""p.o""\xA4\x9F\xA0"".";       // ������ �����.����.
const char  txt__clear1[]           PROGMEM  = " ";                                                                       //  
const char  txt__cont1_connect[]    PROGMEM  = "Ko""\xA2\xA4"". N1 - O""\x9F";                                            // ����. N1 - ��
const char  txt__cont2_connect[]    PROGMEM  = "Ko""\xA2\xA4"". N2 - O""\x9F";                                            // ����. N2 - ��
const char  txt__cont3_connect[]    PROGMEM  = "Ko""\xA2\xA4"". N3 - O""\x9F";                                            // ����. N3 - ��
const char  txt__cont4_connect[]    PROGMEM  = "Ko""\xA2\xA4"". N4 - O""\x9F";                                            // ����. N4 - ��
const char  txt__cont5_connect[]    PROGMEM  = "Ko""\xA2\xA4"". N5 - O""\x9F";                                            // ����. N5 - ��
const char  txt__cont6_connect[]    PROGMEM  = "Ko""\xA2\xA4"". N6 - O""\x9F";                                            // ����. N6 - ��
const char  txt__cont7_connect[]    PROGMEM  = "Ko""\xA2\xA4"". N7 - O""\x9F";                                            // ����. N7 - ��
const char  txt__cont8_connect[]    PROGMEM  = "Ko""\xA2\xA4"". N8 - O""\x9F";                                            // ����. N8 - ��
const char  txt__cont9_connect[]    PROGMEM  = "Ko""\xA2\xA4"". N9 - O""\x9F";                                            // ����. N9 - ��
const char  txt__clear2[]           PROGMEM  = " ";                                                                       //  
const char  txt__cont1_disconnect[] PROGMEM  = "Ko""\xA2\xA4"". N1 - He""\xA4""!";                                        // ����. N1 - ���!
const char  txt__cont2_disconnect[] PROGMEM  = "Ko""\xA2\xA4"". N2 - He""\xA4""!";                                        // ����. N2 - ���!
const char  txt__cont3_disconnect[] PROGMEM  = "Ko""\xA2\xA4"". N3 - He""\xA4""!";                                        // ����. N3 - ���! 
const char  txt__cont4_disconnect[] PROGMEM  = "Ko""\xA2\xA4"". N4 - He""\xA4""!";                                        // ����. N4 - ���! 
const char  txt__cont5_disconnect[] PROGMEM  = "Ko""\xA2\xA4"". N5 - He""\xA4""!";                                        // ����. N5 - ���! 
const char  txt__cont6_disconnect[] PROGMEM  = "Ko""\xA2\xA4"". N6 - He""\xA4""!";                                        // ����. N6 - ���! 
const char  txt__cont7_disconnect[] PROGMEM  = "Ko""\xA2\xA4"". N7 - He""\xA4""!";                                        // ����. N7 - ���! 
const char  txt__cont8_disconnect[] PROGMEM  = "Ko""\xA2\xA4"". N8 - He""\xA4""!";                                        // ����. N8 - ���! 
const char  txt__cont9_disconnect[] PROGMEM  = "Ko""\xA2\xA4"". N9 - He""\xA4""!";                                        // ����. N9 - ���! 


char buffer[40];  
const char* const table_message[] PROGMEM = 
{
 txt_pass_ok,             // 0 " ";                                                                      //  
 txt_pass_no,             // 1 " ";                                                                      //  
 txt_info1,               // 2 "Tec\xA4 ""\x9F""a\x96""e\xA0""e\x9E";                                    // ���� �������
 txt_info2,               // 3 "Tec\xA4 \x96\xA0o\x9F""a \x98""ap\xA2\x9D\xA4yp";                        // ���� ����� ��������
 txt_info3,               // 4 "Hac\xA4po\x9E\x9F""a c\x9D""c\xA4""e\xA1\xAB";                           // ��������� �������
 txt_info4,               // 5 "\x81""e\xA2""epa\xA4op c\x9D\x98\xA2""a\xA0o\x97";                       // ��������� ��������
 txt_info5,               // 6 "Oc\xA6\x9D\xA0\xA0o\x98pa\xA5";                                          // �����������
 txt_botton_clear,        // 7 " ";                                                                      //  
 txt_botton_otmena,       // 8 " ";                                                                      //  
 txt_botton_vvod,         // 9 " ";                                                                      //  
 txt_botton_ret,          // 10 ""B""\xAB""x" ";                                                         //  ���
 txt_system_clear3,       // 11 " ";                                                                     // 
 txt9,                    // 12 "B\x97o\x99";                                                             // ����
 txt10,                   // 13 "O""\xA4\xA1""e""\xA2""a";                                                // "������"
 txt_time_wait,           // 14 "\xA1\x9D\xA2"".""\x97""pe""\xA1\xAF"" ""\xA3""poc""\xA4""o""\xAF";       //  ���. ����� �������
 txt_info29,              // 15 "Stop->PUSH Disp"; 
 txt_info30,              // 16 " "; 
 txt_test_all,            // 17 "Tec""\xA4"" ""\x97""cex pa""\x9C\xAA""e""\xA1""o""\x97";                 // ���� ���� ��������
 txt_test_all_exit1,      // 18 "\x82\xA0\xAF"" ""\x97\xAB""xo""\x99""a";                                 // ��� ������
 txt_test_all_exit2,      // 19 "\xA3""p""\x9D\x9F""oc""\xA2\x9D""c""\xAC"" ""\x9F"" ""\xAD\x9F""pa""\xA2""y";  // ���������� � ������
 txt_test_end,            // 20 "\x85""a""\x97""ep""\xA8\x9D\xA4\xAC";                                    // ���������
 txt_test_repeat,         // 21 "\x89""o""\x97\xA4""op""\x9D\xA4\xAC";                                    // ���������
 txt_error_connect1,      // 22 "O""\x8E\x86\x80""KA";                                                    // ������
 txt_error_connect2,      // 23 "\xA3""o""\x99\x9F\xA0\xAE\xA7""e""\xA2\x9D\xAF"" ""\x9F""a""\x96""e""\xA0\xAF"; //����������� ������
 txt_error_connect3,      // 24 "O""\xA8\x9D\x96""o""\x9F"" ""\xA2""e""\xA4";                             // ������ ���
 txt_error_connect4,      // 25 "O""\xA8\x9D\x96""o""\x9F"" -         ";                                  // ������  - 
 txt__connect1,           // 26 "O""\x96\xA2""apy""\x9B""e""\xA2"" ""\x9F""a""\x96""e""\xA0\xAC"" N1";    // ��������� ������ N1
 txt__connect2,           // 27 "O""\x96\xA2""apy""\x9B""e""\xA2"" ""\x9F""a""\x96""e""\xA0\xAC"" N2";    // ��������� ������ N2
 txt__connect3,           // 28 "O""\x96\xA2""apy""\x9B""e""\xA2"" ""\x9F""a""\x96""e""\xA0\xAC"" N3";    // ��������� ������ N3
 txt__connect4,           // 29 "O""\x96\xA2""apy""\x9B""e""\xA2"" ""\x9F""a""\x96""e""\xA0\xAC"" N4";    // ��������� ������ N4
 txt__test_end,           // 30 "TECT ""\x85""A""KOH""\x8D""EH";                                          // ���� ��������
 txt__panel,              // 31 "Tec""\xA4"" c""\x97""e""\xA4""o""\x99\x9D""o""\x99""o""\x97";            // ���� �����������
 txt__panel0,             // 32 "                          ";                                             // 
 txt__disp,               // 33 "Tec""\xA4"" MT""\x81"" ""\x99\x9D""c""\xA3""e""\xA4\xA7""epa";           // ���� ��� ����������
 txt__instr,              // 34 "Tec""\xA4"" MT""\x81"" ""\x9D\xA2""c""\xA4""py""\x9F\xA4""opa";          // ���� ��� �����������  
 txt__MTT,                // 35 "Tec""\xA4"" MTT";                                                        // ���� ���
 txt__disp_connect,       // 36 "Ka""\x96""e""\xA0\xAC"" ""\x99\x9D""c""\xA3"". ""\xA3""o""\x99\x9F\xA0"".";// ������ ����. �����.
 txt__disp_disconnect,    // 37 "Ka""\x96""e""\xA0\xAC"" ""\x99\x9D""c""\xA3"".o""\xA4\x9F\xA0"".";       // ������ ����.����.
 txt__instr_connect,      // 38 "Ka""\x96""e""\xA0\xAC"" ""\x9D\xA2""c""\xA4""p.""\xA3""o""\x99\x9F\xA0"".";// ������ �����.�����.
 txt__instr_disconnect,   // 39 "Ka""\x96""e""\xA0\xAC"" ""\x9D\xA2""c""\xA4""p.o""\xA4\x9F\xA0"".";      // ������ �����.����.
 txt__clear1,             // 40 " ";                                                                       //  
 txt__cont1_connect,      // 41 "Ko""\xA2\xA4"". N1 - O""\x9F";                                            // ����. N1 - ��
 txt__cont2_connect,      // 42 "Ko""\xA2\xA4"". N2 - O""\x9F";                                            // ����. N2 - ��
 txt__cont3_connect,      // 43 "Ko""\xA2\xA4"". N3 - O""\x9F";                                            // ����. N3 - ��
 txt__cont4_connect,      // 44 "Ko""\xA2\xA4"". N4 - O""\x9F";                                            // ����. N4 - ��
 txt__cont5_connect,      // 45 "Ko""\xA2\xA4"". N5 - O""\x9F";                                            // ����. N5 - ��
 txt__cont6_connect,      // 46 "Ko""\xA2\xA4"". N6 - O""\x9F";                                            // ����. N6 - ��
 txt__cont7_connect,      // 47 "Ko""\xA2\xA4"". N7 - O""\x9F";                                            // ����. N7 - ��
 txt__cont8_connect,      // 48 "Ko""\xA2\xA4"". N8 - O""\x9F";                                            // ����. N8 - ��
 txt__cont9_connect,      // 49 "Ko""\xA2\xA4"". N9 - O""\x9F";                                            // ����. N9 - ��
 txt__clear2,             // 50 " ";                                                                       //   
 txt__cont1_disconnect,   // 51 "Ko""\xA2\xA4"". N1 - He""\xA4""!";                                        // ����. N1 - ���!
 txt__cont2_disconnect,   // 52 "Ko""\xA2\xA4"". N2 - He""\xA4""!";                                        // ����. N2 - ���!
 txt__cont3_disconnect,   // 53 "Ko""\xA2\xA4"". N3 - He""\xA4""!";                                        // ����. N3 - ���! 
 txt__cont4_disconnect,   // 54 "Ko""\xA2\xA4"". N4 - He""\xA4""!";                                        // ����. N4 - ���! 
 txt__cont5_disconnect,   // 55 "Ko""\xA2\xA4"". N5 - He""\xA4""!";                                        // ����. N5 - ���! 
 txt__cont6_disconnect,   // 56 "Ko""\xA2\xA4"". N6 - He""\xA4""!";                                        // ����. N6 - ���! 
 txt__cont7_disconnect,   // 57 "Ko""\xA2\xA4"". N7 - He""\xA4""!";                                        // ����. N7 - ���! 
 txt__cont8_disconnect,   // 58 "Ko""\xA2\xA4"". N8 - He""\xA4""!";                                        // ����. N8 - ���! 
 txt__cont9_disconnect    // 59 "Ko""\xA2\xA4"". N9 - He""\xA4""!";                                        // ����. N9 - ���! 
};




 byte   temp_buffer[40] ;                                                                                                // ����� �������� ��������� ����������
 
 const byte connektN1_default[]    PROGMEM  = { 20,
      1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,                                                        // ������ �
	  1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,
	  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                                                        // ������ B
	  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1                                                         // 1- ���������� ����, 0- ���������� ��� 
 }; // 20 x 5 �����
 const byte connektN2_default[]    PROGMEM  = { 26,
       1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,                                     // ������ �
	   1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,                                     // ������ B
	   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	   1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1                                      // 1- ���������� ����, 0- ���������� ��� 
 }; // 26 x 5 �����
 const byte connektN3_default[]    PROGMEM  = { 37,
        1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,   // ������ �
	   19,18,17,16,15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,   // ������ B
 	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1    // 1- ���������� ����, 0- ���������� ��� 
 }; // 37 x 5 �����
 const byte connektN4_default[]    PROGMEM  = { 32,
       1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,                    // ������ �
	   1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,                    // ������ B
 	   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1                     // 1- ���������� ����, 0- ���������� ��� 
 }; // 32 x 2 �����






 //++++++++++++++++++ ������� � 1 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 // ����������� ������ ������������ � ���������  set_adr_EEPROM()
unsigned int adr_memN1_1 = 0;                       // ��������� ����� ������ ������� ������������ ��������� �������� �1�, �1�
unsigned int adr_memN1_2 = 0;                       // ��������� ����� ������ ������� ������������ ��������� �������� �2�, �2�
unsigned int adr_memN1_3 = 0;                       // ��������� ����� ������ ������� ������������ ��������� �������� �3�, �3�
unsigned int adr_memN1_4 = 0;                       // ��������� ����� ������ ������� ������������ ��������� �������� �4�, �4�
 //++++++++++++++++++ ������� � 2 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
unsigned int adr_memN2_1 = 0;                       // ��������� ����� ������ ������� ������������ ��������� �������� �1�, �1�
unsigned int adr_memN2_2 = 0;                       // ��������� ����� ������ ������� ������������ ��������� �������� �2�, �2�
unsigned int adr_memN2_3 = 0;                       // ��������� ����� ������ ������� ������������ ��������� �������� �3�, �3�
unsigned int adr_memN2_4 = 0;                       // ��������� ����� ������ ������� ������������ ��������� �������� �4�, �4�

//==========================================================================================================================



// ������ �� ���������� � �������� (1 sbi) ��� (0 cbi)
// ��������� ��� ������ � ��� (��������� ������� ������������)
// ���������� ��� ��������� � ������ ���� ��������

#define FASTADC 1
// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif



void dateTime(uint16_t* date, uint16_t* time)                                    // ��������� ������ ������� � ���� �����
{
  DateTime now = RTC.now();

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

void serial_print_date()                           // ������ ���� � �������    
{
	  DateTime now = RTC.now();
	  Serial.print(now.day(), DEC);
	  Serial.print('/');
	  Serial.print(now.month(), DEC);
	  Serial.print('/');
	  Serial.print(now.year(), DEC); 
	  Serial.print(' ');
	  Serial.print(now.hour(), DEC);
	  Serial.print(':');
	  Serial.print(now.minute(), DEC);
	  Serial.print(':');
	  Serial.print(now.second(), DEC);
	  Serial.print("  ");
	  Serial.println(str1[now.dayOfWeek()-1]);
}
void clock_read()
{
	DateTime now = RTC.now();
	second = now.second();       
	minute = now.minute();
	hour   = now.hour();
	dow    = now.dayOfWeek();
	day    = now.day();
	month  = now.month();
	year   = now.year();
}

void set_time()
{
	RTC.adjust(DateTime(__DATE__, __TIME__));
	DateTime now = RTC.now();
	second = now.second();       //Initialization time
	minute = now.minute();
	hour   = now.hour();
	day    = now.day();
	day++;
	if(day > 31)day = 1;
	month  = now.month();
	year   = now.year();
	DateTime set_time = DateTime(year, month, day, hour, minute, second); // ������� ������ � ������� � ������ "set_time"
	RTC.adjust(set_time);             
}
void i2c_eeprom_write_byte( int deviceaddress, unsigned int eeaddress, byte data )
{
	int rdata = data;
	Wire.beginTransmission(deviceaddress);
	Wire.write((int)(eeaddress >> 8)); // MSB
	Wire.write((int)(eeaddress & 0xFF)); // LSB
	Wire.write(rdata);
	Wire.endTransmission();
	delay(10);
}
byte i2c_eeprom_read_byte( int deviceaddress, unsigned int eeaddress ) {
	byte rdata = 0xFF;
	Wire.beginTransmission(deviceaddress);
	Wire.write((int)(eeaddress >> 8)); // MSB
	Wire.write((int)(eeaddress & 0xFF)); // LSB
	Wire.endTransmission();
	Wire.requestFrom(deviceaddress,1);
	if (Wire.available()) rdata = Wire.read();
	return rdata;
}
void i2c_eeprom_read_buffer( int deviceaddress, unsigned int eeaddress, byte *buffer, int length )
{
	
	Wire.beginTransmission(deviceaddress);
	Wire.write((int)(eeaddress >> 8)); // MSB
	Wire.write((int)(eeaddress & 0xFF)); // LSB
	Wire.endTransmission();
	Wire.requestFrom(deviceaddress,length);
	int c = 0;
	for ( c = 0; c < length; c++ )
	if (Wire.available()) buffer[c] = Wire.read();
	
}
void i2c_eeprom_write_page( int deviceaddress, unsigned int eeaddresspage, byte* data, byte length ) 
{
	
	Wire.beginTransmission(deviceaddress);
	Wire.write((int)(eeaddresspage >> 8)); // MSB
	Wire.write((int)(eeaddresspage & 0xFF)); // LSB
	byte c;
	for ( c = 0; c < length; c++)
	Wire.write(data[c]);
	Wire.endTransmission();
	
}

void drawDisplay()
{
  // Clear screen
  myGLCD.clrScr();
  
  // Draw Clockface
  myGLCD.setColor(0, 0, 255);
  myGLCD.setBackColor(0, 0, 0);
  for (int i=0; i<5; i++)
  {
	myGLCD.drawCircle(clockCenterX, clockCenterY, 119-i);
  }
  for (int i=0; i<5; i++)
  {
	myGLCD.drawCircle(clockCenterX, clockCenterY, i);
  }
  
  myGLCD.setColor(192, 192, 255);
  myGLCD.print("3", clockCenterX+92, clockCenterY-8);
  myGLCD.print("6", clockCenterX-8, clockCenterY+95);
  myGLCD.print("9", clockCenterX-109, clockCenterY-8);
  myGLCD.print("12", clockCenterX-16, clockCenterY-109);
  for (int i=0; i<12; i++)
  {
	if ((i % 3)!=0)
	  drawMark(i);
  }  
  clock_read();
  drawMin(minute);
  drawHour(hour, minute);
  drawSec(second);
  oldsec=second;

  // Draw calendar
  myGLCD.setColor(255, 255, 255);
  myGLCD.fillRoundRect(240, 0, 319, 85);
  myGLCD.setColor(0, 0, 0);
  for (int i=0; i<7; i++)
  {
	myGLCD.drawLine(249+(i*10), 0, 248+(i*10), 3);
	myGLCD.drawLine(250+(i*10), 0, 249+(i*10), 3);
	myGLCD.drawLine(251+(i*10), 0, 250+(i*10), 3);
  }

  // Draw SET button
  myGLCD.setColor(64, 64, 128);
  myGLCD.fillRoundRect(260, 200, 319, 239);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(260, 200, 319, 239);
  myGLCD.setBackColor(64, 64, 128);
  myGLCD.print("SET", 266, 212);
  myGLCD.setBackColor(0, 0, 0);
  
 /* myGLCD.setColor(64, 64, 128);
  myGLCD.fillRoundRect(260, 140, 319, 180);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(260, 140, 319, 180);
  myGLCD.setBackColor(64, 64, 128);
  myGLCD.print("RET", 266, 150);
  myGLCD.setBackColor(0, 0, 0);*/

}
void drawMark(int h)
{
  float x1, y1, x2, y2;
  
  h=h*30;
  h=h+270;
  
  x1=110*cos(h*0.0175);
  y1=110*sin(h*0.0175);
  x2=100*cos(h*0.0175);
  y2=100*sin(h*0.0175);
  
  myGLCD.drawLine(x1+clockCenterX, y1+clockCenterY, x2+clockCenterX, y2+clockCenterY);
}
void drawSec(int s)
{
  float x1, y1, x2, y2;
  int ps = s-1;
  
  myGLCD.setColor(0, 0, 0);
  if (ps==-1)
  ps=59;
  ps=ps*6;
  ps=ps+270;
  
  x1=95*cos(ps*0.0175);
  y1=95*sin(ps*0.0175);
  x2=80*cos(ps*0.0175);
  y2=80*sin(ps*0.0175);
  
  myGLCD.drawLine(x1+clockCenterX, y1+clockCenterY, x2+clockCenterX, y2+clockCenterY);

  myGLCD.setColor(255, 0, 0);
  s=s*6;
  s=s+270;
  
  x1=95*cos(s*0.0175);
  y1=95*sin(s*0.0175);
  x2=80*cos(s*0.0175);
  y2=80*sin(s*0.0175);
  
  myGLCD.drawLine(x1+clockCenterX, y1+clockCenterY, x2+clockCenterX, y2+clockCenterY);
}
void drawMin(int m)
{
  float x1, y1, x2, y2, x3, y3, x4, y4;
  int pm = m-1;
  
  myGLCD.setColor(0, 0, 0);
  if (pm==-1)
  pm=59;
  pm=pm*6;
  pm=pm+270;
  
  x1=80*cos(pm*0.0175);
  y1=80*sin(pm*0.0175);
  x2=5*cos(pm*0.0175);
  y2=5*sin(pm*0.0175);
  x3=30*cos((pm+4)*0.0175);
  y3=30*sin((pm+4)*0.0175);
  x4=30*cos((pm-4)*0.0175);
  y4=30*sin((pm-4)*0.0175);
  
  myGLCD.drawLine(x1+clockCenterX, y1+clockCenterY, x3+clockCenterX, y3+clockCenterY);
  myGLCD.drawLine(x3+clockCenterX, y3+clockCenterY, x2+clockCenterX, y2+clockCenterY);
  myGLCD.drawLine(x2+clockCenterX, y2+clockCenterY, x4+clockCenterX, y4+clockCenterY);
  myGLCD.drawLine(x4+clockCenterX, y4+clockCenterY, x1+clockCenterX, y1+clockCenterY);

  myGLCD.setColor(0, 255, 0);
  m=m*6;
  m=m+270;
  
  x1=80*cos(m*0.0175);
  y1=80*sin(m*0.0175);
  x2=5*cos(m*0.0175);
  y2=5*sin(m*0.0175);
  x3=30*cos((m+4)*0.0175);
  y3=30*sin((m+4)*0.0175);
  x4=30*cos((m-4)*0.0175);
  y4=30*sin((m-4)*0.0175);
  
  myGLCD.drawLine(x1+clockCenterX, y1+clockCenterY, x3+clockCenterX, y3+clockCenterY);
  myGLCD.drawLine(x3+clockCenterX, y3+clockCenterY, x2+clockCenterX, y2+clockCenterY);
  myGLCD.drawLine(x2+clockCenterX, y2+clockCenterY, x4+clockCenterX, y4+clockCenterY);
  myGLCD.drawLine(x4+clockCenterX, y4+clockCenterY, x1+clockCenterX, y1+clockCenterY);
}
void drawHour(int h, int m)
{
  float x1, y1, x2, y2, x3, y3, x4, y4;
  int ph = h;
  
  myGLCD.setColor(0, 0, 0);
  if (m==0)
  {
	ph=((ph-1)*30)+((m+59)/2);
  }
  else
  {
	ph=(ph*30)+((m-1)/2);
  }
  ph=ph+270;
  
  x1=60*cos(ph*0.0175);
  y1=60*sin(ph*0.0175);
  x2=5*cos(ph*0.0175);
  y2=5*sin(ph*0.0175);
  x3=20*cos((ph+5)*0.0175);
  y3=20*sin((ph+5)*0.0175);
  x4=20*cos((ph-5)*0.0175);
  y4=20*sin((ph-5)*0.0175);
  
  myGLCD.drawLine(x1+clockCenterX, y1+clockCenterY, x3+clockCenterX, y3+clockCenterY);
  myGLCD.drawLine(x3+clockCenterX, y3+clockCenterY, x2+clockCenterX, y2+clockCenterY);
  myGLCD.drawLine(x2+clockCenterX, y2+clockCenterY, x4+clockCenterX, y4+clockCenterY);
  myGLCD.drawLine(x4+clockCenterX, y4+clockCenterY, x1+clockCenterX, y1+clockCenterY);

  myGLCD.setColor(255, 255, 0);
  h=(h*30)+(m/2);
  h=h+270;
  
  x1=60*cos(h*0.0175);
  y1=60*sin(h*0.0175);
  x2=5*cos(h*0.0175);
  y2=5*sin(h*0.0175);
  x3=20*cos((h+5)*0.0175);
  y3=20*sin((h+5)*0.0175);
  x4=20*cos((h-5)*0.0175);
  y4=20*sin((h-5)*0.0175);
  
  myGLCD.drawLine(x1+clockCenterX, y1+clockCenterY, x3+clockCenterX, y3+clockCenterY);
  myGLCD.drawLine(x3+clockCenterX, y3+clockCenterY, x2+clockCenterX, y2+clockCenterY);
  myGLCD.drawLine(x2+clockCenterX, y2+clockCenterY, x4+clockCenterX, y4+clockCenterY);
  myGLCD.drawLine(x4+clockCenterX, y4+clockCenterY, x1+clockCenterX, y1+clockCenterY);
}
void printDate()
{
  clock_read();
  myGLCD.setFont(BigFont);
  myGLCD.setColor(0, 0, 0);
  myGLCD.setBackColor(255, 255, 255);
	
  myGLCD.print(str[dow-1], 256, 8);
  if (day<10)
	myGLCD.printNumI(day, 272, 28);
  else
	myGLCD.printNumI(day, 264, 28);

  myGLCD.print(str_mon[month-1], 256, 48);
  myGLCD.printNumI(year, 248, 65);
}
void clearDate()
{
  myGLCD.setColor(255, 255, 255);
  myGLCD.fillRect(248, 8, 312, 81);
}
void AnalogClock()
{
	int x, y;
	drawDisplay();
	printDate();
	while (true)
	  {
		if (oldsec != second)
		{
		  if ((second == 0) and (minute == 0) and (hour == 0))
		  {
			clearDate();
			printDate();
		  }
		  if (second==0)
		  {
			drawMin(minute);
			drawHour(hour, minute);
		  }
		  drawSec(second);
		  oldsec = second;
		  wait_time_Old =  millis();
		}

		if (myTouch.dataAvailable())
		{
		  myTouch.read();
		  x=myTouch.getX();
		  y=myTouch.getY();
		  if (((y>=200) && (y<=239)) && ((x>=260) && (x<=319))) //��������� �����
		  {
			myGLCD.setColor (255, 0, 0);
			myGLCD.drawRoundRect(260, 200, 319, 239);
			setClock();
		  }

		  if (((y>=1) && (y<=239)) && ((x>=1) && (x<=260))) //�������
		  {
			myGLCD.clrScr();
			myGLCD.setFont(BigFont);
			break;
		  }
		 if (((y>=1) && (y<=180)) && ((x>=260) && (x<=319))) //�������
		  {
			myGLCD.clrScr();
			myGLCD.setFont(BigFont);
			break;
		  }
		}
		delay(10);
		clock_read();
	  }
}

void flash_time()                                              // ��������� ���������� ���������� 
{ 
	// PORTB = B00000000; // ��� 12 ��������� � ��������� LOW
	slave.run();
	// PORTB = B01000000; // ��� 12 ��������� � ��������� HIGH
}
void serialEvent3()
{
	control_command();
}

void reset_klav()
{
		myGLCD.clrScr();
		myButtons.deleteAllButtons();
		but1 = myButtons.addButton( 10,  20, 250,  35, txt_menu5_1);
		but2 = myButtons.addButton( 10,  65, 250,  35, txt_menu5_2);
		but3 = myButtons.addButton( 10, 110, 250,  35, txt_menu5_3);
		but4 = myButtons.addButton( 10, 155, 250,  35, txt_menu5_4);
		butX = myButtons.addButton(279, 199,  40,  40, "W", BUTTON_SYMBOL); // ������ ���� 
		but_m1 = myButtons.addButton(  10, 199, 45,  40, "1");
		but_m2 = myButtons.addButton(  61, 199, 45,  40, "2");
		but_m3 = myButtons.addButton(  112, 199, 45,  40, "3");
		but_m4 = myButtons.addButton(  163, 199, 45,  40, "4");
		but_m5 = myButtons.addButton(  214, 199, 45,  40, "5");

}

void klav123() // ���� ������ � �������� ����������
{
	ret = 0;

	while (true)
	  {
		if (myTouch.dataAvailable())
		{
			  myTouch.read();
			  x=myTouch.getX();
			  y=myTouch.getY();
	  
		if ((y>=10) && (y<=60))  // Upper row
		  {
			if ((x>=10) && (x<=60))  // Button: 1
			  {
				  waitForIt(10, 10, 60, 60);
				  updateStr('1');
			  }
			if ((x>=70) && (x<=120))  // Button: 2
			  {
				  waitForIt(70, 10, 120, 60);
				  updateStr('2');
			  }
			if ((x>=130) && (x<=180))  // Button: 3
			  {
				  waitForIt(130, 10, 180, 60);
				  updateStr('3');
			  }
			if ((x>=190) && (x<=240))  // Button: 4
			  {
				  waitForIt(190, 10, 240, 60);
				  updateStr('4');
			  }
			if ((x>=250) && (x<=300))  // Button: 5
			  {
				  waitForIt(250, 10, 300, 60);
				  updateStr('5');
			  }
		  }

		 if ((y>=70) && (y<=120))  // Center row
		   {
			 if ((x>=10) && (x<=60))  // Button: 6
				{
				  waitForIt(10, 70, 60, 120);
				  updateStr('6');
				}
			 if ((x>=70) && (x<=120))  // Button: 7
				{
				  waitForIt(70, 70, 120, 120);
				  updateStr('7');
				}
			 if ((x>=130) && (x<=180))  // Button: 8
				{
				  waitForIt(130, 70, 180, 120);
				  updateStr('8');
				}
			 if ((x>=190) && (x<=240))  // Button: 9
				{
				  waitForIt(190, 70, 240, 120);
				  updateStr('9');
				}
			 if ((x>=250) && (x<=300))  // Button: 0
				{
				  waitForIt(250, 70, 300, 120);
				  updateStr('0');
				}
			}
		  if ((y>=130) && (y<=180))  // Upper row
			 {
			 if ((x>=10) && (x<=130))  // Button: Clear
				{
				  waitForIt(10, 130, 120, 180);
				  stCurrent[0]='\0';
				  stCurrentLen=0;
				  myGLCD.setColor(0, 0, 0);
				  myGLCD.fillRect(0, 224, 319, 239);
				}
			 if ((x>=250) && (x<=300))  // Button: Exit
				{
				  waitForIt(250, 130, 300, 180);
				  myGLCD.clrScr();
				  myGLCD.setBackColor(VGA_BLACK);
				  ret = 1;
				  stCurrent[0]='\0';
				  stCurrentLen=0;
				  break;
				}
			 if ((x>=130) && (x<=240))  // Button: Enter
				{
				  waitForIt(130, 130, 240, 180);
				 if (stCurrentLen>0)
				   {
				   for (x=0; x<stCurrentLen+1; x++)
					 {
						stLast[x]=stCurrent[x];
					 }
						stCurrent[0]='\0';
						stLast[stCurrentLen+1]='\0';
						//i2c_eeprom_write_byte(deviceaddress,adr_stCurrentLen1,stCurrentLen);
						stCurrentLen1 = stCurrentLen;
						stCurrentLen=0;
						myGLCD.setColor(0, 0, 0);
						myGLCD.fillRect(0, 208, 319, 239);
						myGLCD.setColor(0, 255, 0);
						myGLCD.print(stLast, LEFT, 208);
						break;
					}
				  else
					{
						myGLCD.setColor(255, 0, 0);
						myGLCD.print("\x80\x8A\x8B\x8B""EP \x89\x8A""CTO\x87!", CENTER, 192);//"������ ������!"
						delay(500);
						myGLCD.print("                ", CENTER, 192);
						delay(500);
						myGLCD.print("\x80\x8A\x8B\x8B""EP \x89\x8A""CTO\x87!", CENTER, 192);//"������ ������!"
						delay(500);
						myGLCD.print("                ", CENTER, 192);
						myGLCD.setColor(0, 255, 0);
					}
				 }
			  }
		  }
	   } 
} 
void drawButtons1() // ����������� �������� ����������
{
// Draw the upper row of buttons
  for (x=0; x<5; x++)
  {
	myGLCD.setColor(0, 0, 255);
	myGLCD.fillRoundRect (10+(x*60), 10, 60+(x*60), 60);
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawRoundRect (10+(x*60), 10, 60+(x*60), 60);
	myGLCD.printNumI(x+1, 27+(x*60), 27);
  }
// Draw the center row of buttons
  for (x=0; x<5; x++)
  {
	myGLCD.setColor(0, 0, 255);
	myGLCD.fillRoundRect (10+(x*60), 70, 60+(x*60), 120);
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawRoundRect (10+(x*60), 70, 60+(x*60), 120);
	if (x<4)
	myGLCD.printNumI(x+6, 27+(x*60), 87);
  }

  myGLCD.print("0", 267, 87);
// Draw the lower row of buttons
  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect (10, 130, 120, 180);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect (10, 130, 120, 180);
  strcpy_P(buffer, (char*)pgm_read_word(&(table_message[8]))); 
  myGLCD.print(buffer, 20, 147);                                   // "������"


  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect (130, 130, 240, 180);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect (130, 130, 240, 180);
  strcpy_P(buffer, (char*)pgm_read_word(&(table_message[9]))); 
  myGLCD.print(buffer, 155, 147);                                  // "����"
  

  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect (250, 130, 300, 180);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect (250, 130, 300, 180);
  strcpy_P(buffer, (char*)pgm_read_word(&(table_message[10]))); 
  myGLCD.print(buffer, 252, 147);                                  // ���
  myGLCD.setBackColor (0, 0, 0);
}
void updateStr(int val)
{
  if (stCurrentLen<20)
  {
	stCurrent[stCurrentLen]=val;
	stCurrent[stCurrentLen+1]='\0';
	stCurrentLen++;
	myGLCD.setColor(0, 255, 0);
	myGLCD.print(stCurrent, LEFT, 224);
  }
  else
  {   // ����� ������ "������������!"
	myGLCD.setColor(255, 0, 0);
	myGLCD.print("\x89""EPE""\x89O\x88HEH\x86""E!", CENTER, 224);// ������������!
	delay(500);
	myGLCD.print("              ", CENTER, 224);
	delay(500);
	myGLCD.print("\x89""EPE""\x89O\x88HEH\x86""E!", CENTER, 224);// ������������!
	delay(500);
	myGLCD.print("              ", CENTER, 224);
	myGLCD.setColor(0, 255, 0);
  }
}
void waitForIt(int x1, int y1, int x2, int y2)
{
  myGLCD.setColor(255, 0, 0);
  myGLCD.drawRoundRect (x1, y1, x2, y2);
  while (myTouch.dataAvailable())
  myTouch.read();
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect (x1, y1, x2, y2);
}

void control_command()
{
	/*
	��� ������ ������������ �������� ���������� �������� ����� �������� �� ������ adr_control_command (40120) 
	��� ��������
	0 -   ���������� ������� ��������
	1 -   ��������� �������� ������ �1
	2 -   ��������� �������� ������ �2
	3 -   ��������� �������� ������ �3
	4 -   ��������� �������� ������ �4
	5 -   ��������� �������� ������ ��������
	6 -   �������� ������� �������� �1 �� ���������
	7 -   �������� ������� �������� �2 �� ���������
	8 -   �������� ������� �������� �3 �� ���������
	9 -   �������� ������� �������� �4 �� ���������
	10 -  ���������� ������� ������� ���������� �1
	11 -  ���������� ������� ������� ���������� �2
	12 -  ������ ������ �� EEPROM ��� �������� � ��
	13 -  �������� ������� �� �K � �������� � EEPROM
	14 -  
	15 -  
	16 -                   
	17 -  
	18 -  
	19 -  
	20 -  
	21 -  
	22 -  
	23 -  
	24 - 
	25 - 
	26 - 
	27 - 
	28 - 
	29 - 
	30 - 

	*/


	int test_n = regBank.get(adr_control_command);   //�����  40000
	if (test_n != 0)
	{
		if(test_n != 0) Serial.println(test_n);	
		switch (test_n)
		{
			case 1:
				 test_cabel_N1();             // ��������� �������� ������ �1
				 break;
			case 2:	
				 test_cabel_N2();             // ��������� �������� ������ �2
				 break;
			case 3:
				 test_cabel_N3();             // ��������� �������� ������ �3
				 break;
			case 4:	
				 test_cabel_N4();             // ��������� �������� ������ �4
				 break;
			case 5:
				 test_panel_N1();             // ��������� �������� ������ ��������
				 break;
			case 6:	
				 save_default_pc();           // �������� ������� �������� � �� ���������
				 break;
			case 7:
				
				 break;
			case 8:	
				
				 break;
			case 9:
				 
				 break;
			case 10:
				 set_rezistor1();             // ���������� ������� ������� ���������� �1
				 break;
			case 11:
				 set_rezistor2();             // ���������� ������� ������� ���������� �1
				 break;
			case 12:
				 mem_byte_trans_readPC();     // ������ ������ �� EEPROM ��� �������� � ��
				 break;
			case 13:
				 mem_byte_trans_savePC();     // �������� ������� �� �K � �������� � EEPROM
				 break;
			case 14:
				 //
				 break;
			case 15:
				 //
				 break;
			case 16:
				 //                  
				 break;
			case 17:
				 //
				 break;
			case 18:
				 //
				 break;
			case 19:
				 //
				 break;
			case 20:                                         //  
				 //
				 break;
			case 21:                      		 		     //  
				//
				 break;
			case 22:                                         //  
				 //
				 break;
			case 23: 
				 //      
				 break;
			case 24: 
				 //    
				 break;
			case 25: 
				 //         
				 break;
			case 26: 
				 //    
				 break;
			case 27: 
				 //
				 break;
			case 28:
				 //
				 break;
			case 29:
				 //
				 break;
			case 30:  
				 //
				 break;

			default:
				 regBank.set(adr_control_command,0);        // ���������� ���������� �1,�2  ������� �������
				 break;
		 }

	}
	else
	{
	   regBank.set(adr_control_command,0);
	}
}

void draw_Glav_Menu()
{
  but1   = myButtons.addButton( 10,  20, 250,  35, txt_menu1_1);
  but2   = myButtons.addButton( 10,  65, 250,  35, txt_menu1_2);
  but3   = myButtons.addButton( 10, 110, 250,  35, txt_menu1_3);
  but4   = myButtons.addButton( 10, 155, 250,  35, txt_menu1_4);
  butX   = myButtons.addButton( 279, 199,  40,  40, "W", BUTTON_SYMBOL); // ������ ���� 
  but_m1 = myButtons.addButton(  10, 199, 45,  40, "1");
  but_m2 = myButtons.addButton(  61, 199, 45,  40, "2");
  but_m3 = myButtons.addButton(  112, 199, 45,  40, "3");
  but_m4 = myButtons.addButton(  163, 199, 45,  40, "4");
  but_m5 = myButtons.addButton(  214, 199, 45,  40, "5");
  myButtons.drawButtons(); // ������������ ������
  myGLCD.setColor(VGA_BLACK);
  myGLCD.setBackColor(VGA_WHITE);
  myGLCD.setColor(0, 255, 0);
  myGLCD.setBackColor(0, 0, 0);
  myGLCD.print("                      ", CENTER, 0); 

  switch (m2) 
				   {
					 case 1:
						  strcpy_P(buffer, (char*)pgm_read_word(&(table_message[2]))); 
					      myGLCD.print(buffer, CENTER, 0);                               // txt_info1
					      break;
					 case 2:
						  strcpy_P(buffer, (char*)pgm_read_word(&(table_message[3]))); 
                          myGLCD.print(buffer, CENTER, 0);                               // txt_info2
					      break;
					 case 3:
						  strcpy_P(buffer, (char*)pgm_read_word(&(table_message[4]))); 
					      myGLCD.print(buffer, CENTER, 0);                               // txt_info3
					      break;
					 case 4:
						  strcpy_P(buffer, (char*)pgm_read_word(&(table_message[5]))); 
					      myGLCD.print(buffer, CENTER, 0);                               // txt_info4
					      break;
					 case 5:
						  strcpy_P(buffer, (char*)pgm_read_word(&(table_message[6]))); 
					      myGLCD.print(buffer, CENTER, 0);                               // txt_info5
					      break;
					 }
}

void swichMenu() // ������ ���� � ������� "txt....."
{
	 m2=1;                                                         // ���������� ������ �������� ����
	 while(1) 
	   {
		  wait_time = millis();                                    // ��������� ������ ����� ��� ������� 
		  if (wait_time - wait_time_Old > 60000 * time_minute)
		  {
				wait_time_Old =  millis();
				AnalogClock();
				myGLCD.clrScr();
				myButtons.drawButtons();                           // ������������ ������
				print_up();                                        // ������������ ������� ������
		  }

		  myButtons.setTextFont(BigFont);                          // ���������� ������� ����� ������  

			if (myTouch.dataAvailable() == true)                   // ��������� ������� ������
			  {
			    pressed_button = myButtons.checkButtons();         // ���� ������ - ��������� ��� ������
				wait_time_Old =  millis();

					 if (pressed_button==butX)                     // ������ ����� ����
					      {  
							 AnalogClock();
							 myGLCD.clrScr();
							 myButtons.drawButtons();              // ������������ ������
							 print_up();                           // ������������ ������� ������
					      }
		 
					 if (pressed_button==but_m1)                   // ������ 1 �������� ����
						  {
							  myButtons.setButtonColors(VGA_WHITE, VGA_GRAY, VGA_WHITE, VGA_RED, VGA_BLUE); // ������� ��� ����
							  myButtons.drawButtons();             // ������������ ������
							  default_colors=true;
							  m2=1;                                // ���������� ������ �������� ����
							  myButtons.relabelButton(but1, txt_menu1_1, m2 == 1);
							  myButtons.relabelButton(but2, txt_menu1_2, m2 == 1);
							  myButtons.relabelButton(but3, txt_menu1_3, m2 == 1);
							  myButtons.relabelButton(but4, txt_menu1_4, m2 == 1);
							  myGLCD.setColor(0, 255, 0);
							  myGLCD.setBackColor(0, 0, 0);
							  myGLCD.print("                      ", CENTER, 0); 
							  strcpy_P(buffer, (char*)pgm_read_word(&(table_message[2]))); 
							  myGLCD.print(buffer, CENTER, 0);                               // txt_info1 "���� �������"
		
						  }
				    if (pressed_button==but_m2)
						  {
							  myButtons.setButtonColors(VGA_WHITE, VGA_RED, VGA_YELLOW, VGA_BLUE, VGA_TEAL);
							  myButtons.drawButtons();
							  default_colors=false;
							  m2=2;
							  myButtons.relabelButton(but1, txt_menu2_1 , m2 == 2);
							  myButtons.relabelButton(but2, txt_menu2_2 , m2 == 2);
							  myButtons.relabelButton(but3, txt_menu2_3 , m2 == 2);
							  myButtons.relabelButton(but4, txt_menu2_4 , m2 == 2);
							  myGLCD.setColor(0, 255, 0);
							  myGLCD.setBackColor(0, 0, 0);
							  myGLCD.print("                      ", CENTER, 0); 
							  strcpy_P(buffer, (char*)pgm_read_word(&(table_message[3]))); 
							  myGLCD.print(buffer, CENTER, 0);                              // txt_info2 ���� ����� ��������
						 }

				   if (pressed_button==but_m3)
						 {
							  myButtons.setButtonColors(VGA_WHITE, VGA_GRAY, VGA_WHITE, VGA_RED, VGA_GREEN);
							  myButtons.drawButtons();
							  default_colors=false;
							  m2=3;
							  myButtons.relabelButton(but1, txt_menu3_1 , m2 == 3);
							  myButtons.relabelButton(but2, txt_menu3_2 , m2 == 3);
							  myButtons.relabelButton(but3, txt_menu3_3 , m2 == 3);
							  myButtons.relabelButton(but4, txt_menu3_4 , m2 == 3);
							  myGLCD.setColor(0, 255, 0);
							  myGLCD.setBackColor(0, 0, 0);
							  myGLCD.print("                      ", CENTER, 0); 
							  strcpy_P(buffer, (char*)pgm_read_word(&(table_message[4]))); 
							  myGLCD.print(buffer, CENTER, 0);                              // txt_info3 ��������� �������
						}
				   if (pressed_button==but_m4)
						{
							  myButtons.setButtonColors(VGA_WHITE, VGA_GRAY, VGA_WHITE, VGA_RED, VGA_RED);
							  myButtons.drawButtons();
							  default_colors=false;
							  m2=4;
							  myButtons.relabelButton(but1, txt_menu4_1 , m2 == 4);
							  myButtons.relabelButton(but2, txt_menu4_2 , m2 == 4);
							  myButtons.relabelButton(but3, txt_menu4_3 , m2 == 4);
							  myButtons.relabelButton(but4, txt_menu4_4 , m2 == 4);
							  myGLCD.setColor(0, 255, 0);
							  myGLCD.setBackColor(0, 0, 0);
							  myGLCD.print("                      ", CENTER, 0); 
							  strcpy_P(buffer, (char*)pgm_read_word(&(table_message[5]))); 
							  myGLCD.print(buffer, CENTER, 0);                                // txt_info4 ��������� ��������
						}

				   if (pressed_button==but_m5)
						{
							  myButtons.setButtonColors(VGA_WHITE, VGA_GRAY, VGA_WHITE, VGA_RED, VGA_NAVY);
							  myButtons.drawButtons();
							  default_colors=false;
							  m2=5;
							  myButtons.relabelButton(but1, txt_osc_menu1 , m2 == 5);
							  myButtons.relabelButton(but2, txt_osc_menu2 , m2 == 5);
							  myButtons.relabelButton(but3, txt_osc_menu3 , m2 == 5);
							  myButtons.relabelButton(but4, txt_osc_menu4 , m2 == 5);
							  myGLCD.setColor(0, 255, 0);
							  myGLCD.setBackColor(0, 0, 0);
							  myGLCD.print("                      ", CENTER, 0); 
							  strcpy_P(buffer, (char*)pgm_read_word(&(table_message[6]))); 
							  myGLCD.print(buffer, CENTER, 0);                                // txt_info5  �����������   
						}
	
	               //*****************  ���� �1  **************

					if (pressed_button==but1 && m2 == 1)
						{
							// ���� ������ �1
							myGLCD.clrScr();   // �������� �����
							test_cabel_N1();
							myGLCD.clrScr();
							myButtons.drawButtons();
							print_up();
						}
	  
					if (pressed_button==but2 && m2 == 1)
						{
							// ���� ������ �2
							myGLCD.clrScr();   // �������� �����
							test_cabel_N2();
							myGLCD.clrScr();
							myButtons.drawButtons();
							print_up();
						}
	  
					if (pressed_button==but3 && m2 == 1)
					   {
							// ���� ������ �3
							myGLCD.clrScr();   // �������� �����
							test_cabel_N3();
			   				myGLCD.clrScr();
							myButtons.drawButtons();
							print_up();
					   }
		           if (pressed_button==but4 && m2 == 1)
					   {
							// ���� ������ �4
							myGLCD.clrScr();   // �������� �����
							test_cabel_N4();
			   				myGLCD.clrScr();
							myButtons.drawButtons();
							print_up();
					   }

		         //*****************  ���� �2  **************


		           if (pressed_button==but1 && m2 == 2)
					  {
							//���� ������ �������� 1
						    test_panel_N1();
	        				myGLCD.clrScr();
							myButtons.drawButtons();
							print_up();
				      }

				   if (pressed_button==but2 && m2 == 2)
				  	  {
 							//���� ������ �������� 2
						    test_panel_N2();
							myGLCD.clrScr();
							myButtons.drawButtons();
							print_up();
					  }
	  
				   if (pressed_button==but3 && m2 == 2)
					  {
						    test_panel_N3run();
							myGLCD.clrScr();
							myButtons.drawButtons();
							print_up();
					  }
				   if (pressed_button==but4 && m2 == 2)
					  {
							// ���� ������ ���� ��������
						    // myGLCD.print(txt_pass_ok, RIGHT, 208); 
							test_all_pin();
	     					myGLCD.clrScr();
							myButtons.drawButtons();
							print_up();
				      }
		
		        //*****************  ���� �3  **************
				   if (pressed_button==but1 && m2 == 3) // ������ ����� ���� 3
				 	  {
							myGLCD.clrScr();   // �������� �����
	/*						myGLCD.print(txt_pass_ok, RIGHT, 208); 
							delay (500);*/
							myButtons.drawButtons();
							print_up();
					  }

			 //--------------------------------------------------------------
		           if (pressed_button==but2 && m2 == 3)  // ������ ����� ���� 3
				      {
							myGLCD.clrScr();
							myGLCD.clrScr();
							myButtons.drawButtons();
							print_up();
					  }

			   //------------------------------------------------------------------

			       if (pressed_button==but3 && m2 == 3)  // ������ ����� ���� 3
					  { 
							myGLCD.clrScr();
		                    save_tab_def();	  
							myGLCD.clrScr();
							myButtons.drawButtons();
							print_up();
				      }

	 //------------------------------------------------------------------
				   if (pressed_button==but4 && m2 == 3)                 // ��������� ����� ���� 3
				      {
							myGLCD.clrScr();
							myGLCD.setFont(BigFont);
							myGLCD.setBackColor(0, 0, 255);
							myGLCD.clrScr();
							drawButtons1();                            // ���������� �������� ����������
							myGLCD.printNumI(time_minute, LEFT, 208);
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[14]))); 
							myGLCD.print(buffer, 35, 208);             // txt_time_wait
							klav123();                                 // ������� ���������� � ����������
							if (ret == 1)                              // ���� "�������" - ���������
								 {
									goto bailout41;                    // ������� �� ��������� ���������� ������ ����
								 }
							else                                       // ����� ��������� ����� ����
								 {
									 time_minute = atol(stLast);
								 }
						    bailout41:                                 // ������������ ������ ����
						    myGLCD.clrScr();
						    myButtons.drawButtons();
						    print_up();
				      }

                   //*****************  ���� �4  **************

                   if (pressed_button==but1 && m2 == 4) // 
					  {
			
							myGLCD.clrScr();   // �������� �����
						//	myGLCD.print(txt_pass_ok, RIGHT, 208); 
							delay (500);
							sine();
							//butA = myButtons.addButton(279, 20,  40,  35, "W", BUTTON_SYMBOL); // ���������
							//if (myButtons.buttonEnabled(butB)) myButtons.deleteButton(butB);
							//if (myButtons.buttonEnabled(butC)) myButtons.deleteButton(butC);
							//if (myButtons.buttonEnabled(butD)) myButtons.deleteButton(butD);
							myButtons.drawButtons();
							print_up();
							//
				   
					  }

				   if (pressed_button==but2 && m2 == 4)
					  {
					
							myGLCD.clrScr();
						//	myGLCD.print(txt_pass_ok, RIGHT, 208);
							delay (500);
							saw();
							//			butB = myButtons.addButton(279, 65, 40,  35, "W", BUTTON_SYMBOL); // �����������
							//if (myButtons.buttonEnabled(butA)) myButtons.deleteButton(butA);
							//if (myButtons.buttonEnabled(butC)) myButtons.deleteButton(butC);
							//if (myButtons.buttonEnabled(butD)) myButtons.deleteButton(butD);
							myButtons.drawButtons();
							print_up();
					  }

		           if (pressed_button==but3 && m2 == 4) // 
					  {
				
							myGLCD.clrScr();
						//	myGLCD.print(txt_pass_ok, RIGHT, 208);
							delay (500);
							triangle();
							//butC = myButtons.addButton(279, 110,  40,  35, "W", BUTTON_SYMBOL); // ������������
							//if (myButtons.buttonEnabled(butA)) myButtons.deleteButton(butA);
							//if (myButtons.buttonEnabled(butB)) myButtons.deleteButton(butB);
							//if (myButtons.buttonEnabled(butD)) myButtons.deleteButton(butD);
							myButtons.drawButtons();
							print_up();
					  }
				   if (pressed_button==but4 && m2 == 4) //
					  {
							myGLCD.clrScr();
						//	myGLCD.print(txt_pass_ok, RIGHT, 208);
							delay (500);
							pulse();
							//butD = myButtons.addButton(279, 155,  40,  35, "W", BUTTON_SYMBOL); // ������������� ������
							//if (myButtons.buttonEnabled(butB)) myButtons.deleteButton(butB);
							//if (myButtons.buttonEnabled(butC)) myButtons.deleteButton(butC);
							//if (myButtons.buttonEnabled(butA)) myButtons.deleteButton(butA);
							myButtons.drawButtons();
							print_up();
					  }
				    //*****************  ���� �5  **************

                   if (pressed_button==but1 && m2 == 5) // ����� ������
					  {
						    myGLCD.clrScr();
							oscilloscope();
							myGLCD.clrScr();
							myButtons.drawButtons();
							print_up();
					  }
				   if (pressed_button==but2 && m2 == 5)
					  {
						    myGLCD.clrScr();
						    logData();
							myGLCD.clrScr();   // �������� �����
							delay (500);
							myButtons.drawButtons();
							print_up();
					  }

				   if (pressed_button==but3 && m2 == 5) // ���� ������ ������������
					  {
							myGLCD.clrScr();   // �������� �����
							test_ADC();
							myGLCD.clrScr();   // �������� �����
							myButtons.drawButtons();
							print_up();
					  }

			       if (pressed_button==but4 && m2 == 5) // 
			          {
							myGLCD.clrScr();   // �������� �����
							//	myGLCD.print(txt_pass_ok, RIGHT, 208); 
							delay (500);

							myButtons.drawButtons();
							print_up();
				      }
			
		           if (pressed_button==-1) 
					  {
						//  myGLCD.print("HET", 220, 220);
					  }
				  } 
       }
}
void print_up() // ������ ������� ������� ��� ����
{
	myGLCD.setColor(0, 255, 0);
	myGLCD.setBackColor(0, 0, 0);
	myGLCD.print("                      ", CENTER, 0); 
	switch (m2) 
	{
	case 1:
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[2]))); 
			myGLCD.print(buffer, CENTER, 0);                                 // txt_info1
			break;
		case 2:
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[3]))); 
            myGLCD.print(buffer, CENTER, 0);                                 // txt_info2
			break;
		case 3:
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[4]))); 
			myGLCD.print(buffer, CENTER, 0);                                 // txt_info3
			break;
		case 4:
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[5]))); 
			myGLCD.print(buffer, CENTER, 0);                                 // txt_info4
			break;
        case 5:
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[6]))); 
			myGLCD.print(buffer, CENTER, 0);                                 // txt_info5
			break;
    }
}

void pulse()
{
	digitalWrite(kn1Nano,  LOW);                        // 
	digitalWrite(kn2Nano, HIGH);                        //
	digitalWrite(kn3Nano, HIGH);                        //
	digitalWrite(kn4Nano, HIGH);                         // 
}
void triangle()
{
	digitalWrite(kn1Nano, HIGH);                        // 
	digitalWrite(kn2Nano,  LOW);                        //
	digitalWrite(kn3Nano, HIGH);                        //
	digitalWrite(kn4Nano, HIGH);                         // 
}
void saw()
{
	digitalWrite(kn1Nano, HIGH);                        // 
	digitalWrite(kn2Nano, HIGH);                        //
	digitalWrite(kn3Nano, LOW);                        //
	digitalWrite(kn4Nano, HIGH);                         // 
}
void sine()
{
	digitalWrite(kn1Nano, HIGH);                        // 
	digitalWrite(kn2Nano, HIGH);                        //
	digitalWrite(kn3Nano, HIGH);                        //
	digitalWrite(kn4Nano, LOW);                         // 
}

void setup_resistor()
{ 
	Wire.beginTransmission(address_AD5252);        // transmit to device
	Wire.write(byte(control_word1));               // sends instruction byte  
	Wire.write(0);                                 // sends potentiometer value byte  
	Wire.endTransmission();                        // stop transmitting
	Wire.beginTransmission(address_AD5252);        // transmit to device
	Wire.write(byte(control_word2));               // sends instruction byte  
	Wire.write(0);                                 // sends potentiometer value byte  
	Wire.endTransmission();                        // stop transmitting
}
void resistor(int resist, int valresist)
{
	resistance = valresist;
	switch (resist)
	{
	case 1:
			Wire.beginTransmission(address_AD5252);     // transmit to device
			Wire.write(byte(control_word1));            // sends instruction byte  
			Wire.write(resistance);                     // sends potentiometer value byte  
			Wire.endTransmission();                     // stop transmitting
			break;
	case 2:				
			Wire.beginTransmission(address_AD5252);     // transmit to device
			Wire.write(byte(control_word2));            // sends instruction byte  
			Wire.write(resistance);                     // sends potentiometer value byte  
			Wire.endTransmission();                     // stop transmitting
			break;
	}
			//Wire.requestFrom(address_AD5252, 1, true);  // ������� ��������� ������ ��������� 
			//level_resist = Wire.read();                 // sends potentiometer value byte  
	// regBank.set(adr_control_command,0);
}
void set_rezistor1()
{
	int mwt1 = regBank.get(40003);             // ����� �������� �������� ������� ���������� � 1
	resistor(1, mwt1);
	regBank.set(adr_control_command,0);
}
void set_rezistor2()
{
	int mwt2 = regBank.get(40004);             // ����� �������� �������� ������� ���������� � 2
	resistor(2, mwt2);
	regBank.set(adr_control_command,0);
}

void save_tab_def()                        // ������ � EEPROM  ������ �������� �� ���������
{
	for (int i = 1;i<5;i++)
	{
		myGLCD.setColor(255, 255, 255);
		myGLCD.print("Save block N ", 25, 70);//
		myGLCD.printNumI(i, 230, 70);
		save_default(i);                 //������������ ������ � EEPROM ����� ������ �������� �� ���������

		for (int x = 10;x<65;x++)
		{
			myGLCD.setColor(0, 0, 255);
			myGLCD.fillRoundRect (30, 100, 30+ (x*4),110);
			myGLCD.setColor(255, 255, 255);
			myGLCD.drawRoundRect (28, 98, 32+ (x*4),112);
		}
		myGLCD.clrScr();
	}
}
void save_default(byte adrN_eeprom)                                               //������������ ������ � EEPROM ����� ������ �������� �� ���������
{
	byte _u_konnekt     = 0;                                                      // ��������� �������� ����������� ��������.
    byte _step_mem      = 0;                                                      // ����� ����� � �������
	int adr_memN        = 0;
	int connekt_default = 0;                                                      // ����� � ���������� ������
		switch (adrN_eeprom)
		   {
			case 1:
				 adr_memN   = adr_memN1_1;                                        // ����� ����� EEPROM � 1 
				 _step_mem  = (pgm_read_byte_near(connektN1_default));            // ����� ����� � �������
				 for (int i = 0; i < (_step_mem * 5)+1;i++)                       // �������� 5 ������ �������  
					{
					  _u_konnekt = pgm_read_byte_near(connektN1_default+i);
					  i2c_eeprom_write_byte(deviceaddress,adr_memN+i, _u_konnekt); 
					}
				 break;
			case 2:
				 adr_memN   = adr_memN1_2;                                        // ����� ����� EEPROM � 2 
				 _step_mem  = (pgm_read_byte_near(connektN2_default));            // ����� ����� � �������
				 for (int i = 0; i < (_step_mem * 5)+1;i++)                       // �������� 5 ������ �������  
					{
					  _u_konnekt = pgm_read_byte_near(connektN2_default+i);
					  i2c_eeprom_write_byte(deviceaddress,adr_memN+i, _u_konnekt); 
					}
				 break;
			case 3:
				 adr_memN   = adr_memN1_3;                                       // ����� ����� EEPROM � 3
				 _step_mem  = (pgm_read_byte_near(connektN3_default));           // ����� ����� � �������
				 for (int i = 0; i < (_step_mem * 5)+1;i++)                      // �������� 5 ������ �������  
					{
					  _u_konnekt = pgm_read_byte_near(connektN3_default+i);
					  i2c_eeprom_write_byte(deviceaddress,adr_memN+i, _u_konnekt); 
					}
				 break;
			case 4:
				 adr_memN   = adr_memN1_4;                                       // ����� ����� EEPROM � 4
				 _step_mem  = (pgm_read_byte_near(connektN4_default));           // ����� ����� � �������
				 for (int i = 0; i < (_step_mem * 5)+1;i++)                      // �������� 5 ������ �������   
					{
					  _u_konnekt = pgm_read_byte_near(connektN4_default+i);
					  i2c_eeprom_write_byte(deviceaddress,adr_memN+i, _u_konnekt); 
					}
				 break;
			default:
				 adr_memN   = adr_memN1_1;                                       // ����� ����� EEPROM � 1 
				 _step_mem  = (pgm_read_byte_near(connektN1_default));           // ����� ����� � �������
				 for (int i = 0; i < (_step_mem * 5)+1;i++)                      // �������� 5 ������ �������    
					{
					  _u_konnekt = pgm_read_byte_near(connektN1_default+i);
					  i2c_eeprom_write_byte(deviceaddress,adr_memN+i, _u_konnekt); 
					}
				 break;
		  }
}
void save_default_pc()                                                       // ������ ��������� ��������� ������� �������� �1
{
	int _step_mem       = 0;                                                 // ����� ����� � �������
	byte _u_konnekt     = 0;                                                 // ��������� �������� ����������� ��������.
    int adr_memN        = 0;
	int adrN_eeprom     = regBank.get(40008);                                // �������� ����� ������� �� ��������

		switch (adrN_eeprom)
		   {
			case 1:
				 adr_memN = adr_memN1_1;                                     // ����� ����� EEPROM � 1 
				 _step_mem = (pgm_read_byte_near(connektN1_default));        // ����� ����� � �������
				 for (int i = 1; i < (_step_mem * 2)+1;i++)                    
					{
					  _u_konnekt = pgm_read_byte_near(connektN1_default+i);
					  i2c_eeprom_write_byte(deviceaddress,adr_memN+i, _u_konnekt); 
					}
				 break;
			case 2:
				 adr_memN = adr_memN1_2;                                     // ����� ����� EEPROM � 2 
				 _step_mem = (pgm_read_byte_near(connektN2_default));        // ����� ����� � �������
				 for (int i = 1; i < (_step_mem * 2)+1;i++)                    
					{
					  _u_konnekt = pgm_read_byte_near(connektN2_default+i);
					  i2c_eeprom_write_byte(deviceaddress,adr_memN+i, _u_konnekt); 
					}
				 break;
			case 3:
				 adr_memN = adr_memN1_3;                                     // ����� ����� EEPROM � 3
				 _step_mem = (pgm_read_byte_near(connektN3_default));        // ����� ����� � �������
				 for (int i = 1; i < (_step_mem * 2)+1;i++)                    
					{
					  _u_konnekt = pgm_read_byte_near(connektN3_default+i);
					  i2c_eeprom_write_byte(deviceaddress,adr_memN+i, _u_konnekt); 
					}
				 break;
			case 4:
				 adr_memN = adr_memN1_4;                                     // ����� ����� EEPROM � 4
				 _step_mem = (pgm_read_byte_near(connektN4_default));        // ����� ����� � �������
				 for (int i = 1; i < (_step_mem * 2)+1;i++)                    
					{
					  _u_konnekt = pgm_read_byte_near(connektN4_default+i);
					  i2c_eeprom_write_byte(deviceaddress,adr_memN+i, _u_konnekt); 
					}
				 break;
			default:
				 adr_memN = adr_memN1_1;                                     // ����� ����� EEPROM � 1 
				 _step_mem = (pgm_read_byte_near(connektN1_default));        // ����� ����� � �������
			 	 for (int i = 1; i < (_step_mem * 2)+1;i++)                    
					{
					  _u_konnekt = pgm_read_byte_near(connektN1_default+i);
					  i2c_eeprom_write_byte(deviceaddress,adr_memN+i, _u_konnekt); 
					}
				 break;
		  }
	regBank.set(adr_control_command,0);                                      // ��������� ���������    
}

void set_komm_mcp(char chanal_a_b, int chanal_n, char chanal_in_out )   // ��������� ��������� ��������� ������
{
	/*
	int chanal_a_b  -  ������� ���� �������� � - 1 ��� � - 2
	int chanal_n    -  ������� � ������ (1-48)
	chanal_in_out   -  ������� ���������� ����� - 1 ��� ��������� ��������� ����� �����
	*/
	char _chanal_a_b     = chanal_a_b;                                // ����� ������ ������������  � - ����, B - �����.
	int _chanal_n        = chanal_n;                                  // � ������ (1- 48).
	int _chanal_in_out   = chanal_in_out;                             // ������� ������: 1 - ������,  2 - ���������� �� �����(���������).

	if (_chanal_a_b == 'A')                                           // ��������� ������� � 
	{
		if (_chanal_in_out == 'O')                                    // ���������  ��������� ������ �  �� ����/�����
		{
		    mcp_Out1.digitalWrite(8,  HIGH);                          // ����� ������ EN ���������� ��������� �����������  1E1  U13
			mcp_Out1.digitalWrite(9,  HIGH);                          // ����� ������ EN ���������� ��������� �����������  1E2  U17
			mcp_Out1.digitalWrite(10, HIGH);                          // ����� ������ EN ���������� ��������� �����������  1E3  U23
			if (_chanal_n <17)
			{
                set_mcp_byte_1a(_chanal_n-1);                         // ������������ ���� ������ ������ (0 - 15)
				mcp_Out1.digitalWrite(8, LOW);                        // ������� EN ���������� ��������� �����������  1E1  U13
			}
			else if(_chanal_n > 16 && _chanal_n < 33)
			{
				set_mcp_byte_1a(_chanal_n - 17);                      //  ������������ ���� ������ ������ (15 - 31)
				mcp_Out1.digitalWrite(9, LOW);                        // ������� EN ���������� ��������� �����������  1E2  U17
			}
			else if(_chanal_n > 32 && _chanal_n < 49)
			{
				set_mcp_byte_1a(_chanal_n - 33);                      // ������������ ���� ������ ������ (32 - 48)
				mcp_Out1.digitalWrite(10, LOW);                       // ������� EN ���������� ��������� �����������  1E3  U23
			}

		}
		if (_chanal_in_out == 'G')                                    // ��������� ����� � 
		{
		    mcp_Out1.digitalWrite(11, HIGH);                          // ����� ������ EN ���������� ��������� �����������  1E4  U14
			mcp_Out1.digitalWrite(12, HIGH);                          // ����� ������ EN ���������� ��������� �����������  1E5  U19 
			mcp_Out1.digitalWrite(13, HIGH);                          // ����� ������ EN ���������� ��������� �����������  1E6  U21 
			if (_chanal_n <17)
			{
				set_mcp_byte_1b(_chanal_n-1);                         // ������������ ���� ������ ������ (0 - 15)
				mcp_Out1.digitalWrite(11, LOW);                       // �������  EN ���������� ��������� �����������  1E4  U14
			}
			else if(_chanal_n > 16 && _chanal_n < 33)
			{
				set_mcp_byte_1b(_chanal_n - 17);                      // ������������ ���� ������ ������ (16 - 31)
				mcp_Out1.digitalWrite(12, LOW);                       // ������� EN ���������� ��������� �����������  1E5  U19 
			}
			else if(_chanal_n > 32 && _chanal_n < 49)
			{
				set_mcp_byte_1b(_chanal_n - 33);                      // ������������ ���� ������ ������ (32 - 48)
				mcp_Out1.digitalWrite(13, LOW);                       // �������  EN ���������� ��������� �����������  1E6  U21 
			}

		}
			//delay(10);
	}
	else if(_chanal_a_b == 'B')                                       // ��������� ������� � 
	{
		if (_chanal_in_out == 'O')                                    // ���������  ��������� ������ �  �� ����/�����
		{
		    mcp_Out2.digitalWrite(8,  HIGH);                          // ����� ������ EN ���������� ��������� �����������  2E1  U15
			mcp_Out2.digitalWrite(9,  HIGH);                          // ����� ������ EN ���������� ��������� �����������  2E2  U18 
			mcp_Out2.digitalWrite(10, HIGH);                          // ����� ������ EN ���������� ��������� �����������  2E3  U22
			if (_chanal_n <17)
			{
				set_mcp_byte_2a(_chanal_n-1);                         // ������������ ���� ������ ������ (0 - 15)
				mcp_Out2.digitalWrite(8, LOW);                        // ������� EN ���������� ��������� �����������  2E1  U15
			}
			else if(_chanal_n > 16 && _chanal_n < 33)
			{
				set_mcp_byte_2a(_chanal_n - 17);                      // ������������ ���� ������ ������ (16 - 31)
				mcp_Out2.digitalWrite(9, LOW);                        // ������� EN ���������� ��������� �����������  2E2  U18 
			}
			else if(_chanal_n > 32 && _chanal_n < 49)
			{
				set_mcp_byte_2a(_chanal_n - 33);                      // ������������ ���� ������ ������ (32 - 48)
				mcp_Out2.digitalWrite(10, LOW);                       // ������� EN ���������� ��������� �����������  2E3  U22
			}

		}
		if (_chanal_in_out == 'G')                                    // ��������� ����� B 
		{
		    mcp_Out2.digitalWrite(11, HIGH);                          // ����� ������ EN ���������� ��������� �����������  2E4  U16
			mcp_Out2.digitalWrite(12, HIGH);                          // ����� ������ EN ���������� ��������� �����������  2E5  U20 
			mcp_Out2.digitalWrite(13, HIGH);                          // ����� ������ EN ���������� ��������� �����������  2E6  U24
			if (_chanal_n <17)
			{
				set_mcp_byte_2b(_chanal_n-1);                         // ������������ ���� ������ ������ (0 - 15)
				mcp_Out2.digitalWrite(11, LOW);                       // ������� EN ���������� ��������� �����������  2E4  U16
			}
			else if(_chanal_n > 16 && _chanal_n < 33)
			{
				set_mcp_byte_2b(_chanal_n - 17);                      // ������������ ���� ������ ������ (16 - 31)
				mcp_Out2.digitalWrite(12, LOW);                       // ������� EN ���������� ��������� �����������  2E5  U20 
			}
			else if(_chanal_n > 32 && _chanal_n < 49)
			{
				set_mcp_byte_2b(_chanal_n - 33);                      // ������������ ���� ������ ������ (32 - 48)
				mcp_Out2.digitalWrite(13, LOW);                       // ������� EN ���������� ��������� �����������  2E6  U24
			}
		}
	}
	//delay(10);
}
void set_mcp_byte_1a(int set_byte)
{

	    int _chanal_n = set_byte;

		if(bitRead(_chanal_n, 0) == 1)      // ���������� ��� 0
		{
			mcp_Out1.digitalWrite(0, HIGH);
		}
		else
		{
            mcp_Out1.digitalWrite(0, LOW);
		}

		if(bitRead(_chanal_n, 1) == 1)      // ���������� ��� 1
		{
			mcp_Out1.digitalWrite(1, HIGH);
		}
		else
		{
            mcp_Out1.digitalWrite(1, LOW);
		}

		if(bitRead(_chanal_n, 2) == 1)      // ���������� ��� 2
		{
			mcp_Out1.digitalWrite(2, HIGH);
		}
		else
		{
            mcp_Out1.digitalWrite(2, LOW);
		}


		if(bitRead(_chanal_n, 3) == 1)      // ���������� ��� 3
		{
			mcp_Out1.digitalWrite(3, HIGH);
		}
		else
		{
            mcp_Out1.digitalWrite(3, LOW);
		}
}
void set_mcp_byte_1b(int set_byte)
{
	    int _chanal_n = set_byte;

		if(bitRead(_chanal_n, 0) == 1)      // ���������� ��� 0
		{
			mcp_Out1.digitalWrite(4, HIGH);
		}
		else
		{
            mcp_Out1.digitalWrite(4, LOW);
		}

		if(bitRead(_chanal_n, 1) == 1)      // ���������� ��� 1
		{
			mcp_Out1.digitalWrite(5, HIGH);
		}
		else
		{
            mcp_Out1.digitalWrite(5, LOW);
		}

		if(bitRead(_chanal_n, 2) == 1)      // ���������� ��� 2
		{
			mcp_Out1.digitalWrite(6, HIGH);
		}
		else
		{
            mcp_Out1.digitalWrite(6, LOW);
		}


		if(bitRead(_chanal_n, 3) == 1)      // ���������� ��� 3
		{
			mcp_Out1.digitalWrite(7, HIGH);
		}
		else
		{
            mcp_Out1.digitalWrite(7, LOW);
		}
}
void set_mcp_byte_2a(int set_byte)
{
	int _chanal_n = set_byte;

		if(bitRead(_chanal_n, 0) == 1)      // ���������� ��� 0
		{
			mcp_Out2.digitalWrite(0, HIGH);
		}
		else
		{
            mcp_Out2.digitalWrite(0, LOW);
		}

		if(bitRead(_chanal_n, 1) == 1)      // ���������� ��� 1
		{
			mcp_Out2.digitalWrite(1, HIGH);
		}
		else
		{
            mcp_Out2.digitalWrite(1, LOW);
		}

		if(bitRead(_chanal_n, 2) == 1)      // ���������� ��� 2
		{
			mcp_Out2.digitalWrite(2, HIGH);
		}
		else
		{
            mcp_Out2.digitalWrite(2, LOW);
		}


		if(bitRead(_chanal_n, 3) == 1)      // ���������� ��� 3
		{
			mcp_Out2.digitalWrite(3, HIGH);
		}
		else
		{
            mcp_Out2.digitalWrite(3, LOW);
		}
}
void set_mcp_byte_2b(int set_byte)
{
	int _chanal_n = set_byte;

		if(bitRead(_chanal_n, 0) == 1)      // ���������� ��� 0
		{
			mcp_Out2.digitalWrite(4, HIGH);
		}
		else
		{
            mcp_Out2.digitalWrite(4, LOW);
		}

		if(bitRead(_chanal_n, 1) == 1)      // ���������� ��� 1
		{
			mcp_Out2.digitalWrite(5, HIGH);
		}
		else
		{
            mcp_Out2.digitalWrite(5, LOW);
		}

		if(bitRead(_chanal_n, 2) == 1)      // ���������� ��� 2
		{
			mcp_Out2.digitalWrite(6, HIGH);
		}
		else
		{
            mcp_Out2.digitalWrite(6, LOW);
		}


		if(bitRead(_chanal_n, 3) == 1)      // ���������� ��� 3
		{
			mcp_Out2.digitalWrite(7, HIGH);
		}
		else
		{
            mcp_Out2.digitalWrite(7, LOW);
		}
}
void mem_byte_trans_readPC()                                      //  ������ ������ �� EEPROM ��� �������� � ��
{
	unsigned int _adr_reg = regBank.get(40005)+40000;             //  ����� ����� ��������� ��� �������� � �� ������.
	unsigned int _adr_mem = regBank.get(40006);                   //  ����� ����� ������ ��� �������� � �� ������.
	unsigned int _size_block = regBank.get(40007);                //  ����� ����� ����� ������

	for (unsigned int x_mem = 0;x_mem < _size_block;x_mem++)
	{
		regBank.set(_adr_reg+x_mem,i2c_eeprom_read_byte(deviceaddress,_adr_mem + x_mem));
	}
	regBank.set(adr_control_command,0);                           // ��������� ���������    
	delay(200);
}
void mem_byte_trans_savePC()                                      //  �������� ������� �� �K � �������� � EEPROM
{
	unsigned int _adr_reg = regBank.get(40005);                   //  ����� ����� ��������� ��� �������� � �� ������.
	unsigned int _adr_mem = regBank.get(40006);                   //  ����� ����� ������ ��� �������� � �� ������.
	unsigned int _size_block = regBank.get(40007);                //  ����� ����� ����� ������

	for (unsigned int x_mem = 0;x_mem < _size_block;x_mem++)
	{
		i2c_eeprom_write_byte(deviceaddress, _adr_mem + x_mem, regBank.get(_adr_reg+x_mem));
	}
	regBank.set(adr_control_command,0);                           // ��������� ���������    
	delay(200);
}

int search_cabel(int sc)
{
	pinMode(46, OUTPUT);                                                        // ���������� �� ����� ����� ������������ U13,U17,U23 (������� ����� � �� ������ ������)
	digitalWrite(46, LOW);                                                      // ���������� ����������� ������� �� �����������
	pinMode(47, INPUT);                                                         // ���������� �� ����  ����� ������������ U15,U18,U22 (������� ����� � �� �������� ������)
	digitalWrite(47, HIGH);                                                     // ���������� ������� ������� �� ������ 47
 	int n_connect = 0;

	switch (sc) 
	{
	case 1:
		set_komm_mcp('A', 1,'O');
		set_komm_mcp('B', 1,'O');
		if (digitalRead(47) == LOW ) 
		{
			n_connect = 2;
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[27]))); 
			myGLCD.print(buffer, CENTER, 20);                                  // txt__connect2
		}
		break;
	case 39:
		set_komm_mcp('A', 39,'O');
		set_komm_mcp('B', 19,'O');
		if (digitalRead(47) == LOW ) 
		{
			n_connect = 3;
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[28]))); 
			myGLCD.print(buffer, CENTER, 20);                                 // txt__connect3
		}
		break;
	case 40:
		set_komm_mcp('A', 40,'O');
		set_komm_mcp('B', 40,'O');
		if (digitalRead(47) == LOW ) 
		{
			n_connect = 1;
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[26]))); 
			myGLCD.print(buffer, CENTER, 20);                                 // txt__connect1
		}
		break;
	case 41:
		set_komm_mcp('A', 41,'O');
		set_komm_mcp('B', 41,'O');
		if (digitalRead(47) == LOW ) 
		{
			n_connect = 4;
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[29]))); 
			myGLCD.print(buffer, CENTER, 20);                                 // txt__connect4
		}
		break;
	}
	if(n_connect ==0) Serial.println("Connector is not detected");
	return n_connect;
}

void test_cabel_N1()
{
	myGLCD.clrScr();
	myGLCD.print(txt_menu1_1, CENTER, 1);                                      // "���� ������ N 1"
	myGLCD.setColor(255, 255, 255);                                            // ����� ���������
	myGLCD.drawRoundRect (5, 200, 155, 239);
 	myGLCD.drawRoundRect (160, 200, 315, 239);
	myGLCD.drawLine( 10, 60, 310, 60);
	myGLCD.setColor(0, 0, 255);
	myGLCD.fillRoundRect (6, 201, 154, 238);
	myGLCD.fillRoundRect (161, 201, 314, 238);
	myGLCD.setColor(255, 255, 255);  
	myGLCD.setBackColor( 0, 0, 255);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[21]))); 
	myGLCD.print(buffer, 10, 210);                                             //txt_test_repeat  ���������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[20]))); 
	myGLCD.print(buffer, 168, 210);                                            //txt_test_end ���������
	myGLCD.setBackColor( 0, 0, 0);                                             //  
	mcp_Out2.digitalWrite(14, LOW);                                            // ��������� ���� +12v
	if (search_cabel(40)== 1)                                                  // ������ ������ �1
	{
		test_cabel_N1_run();                                                   // ��������� ��������
		while (true)                                                           // �������� ��������� �������
			{

			if (myTouch.dataAvailable())
				{
				myTouch.read();
				x=myTouch.getX();
				y=myTouch.getY();
		
				if (((y>=200) && (y<=239)) && ((x>=5) && (x<=155)))           //������ ������ "��������� ��������"
					{
						waitForIt(5, 200, 155, 239);
						myGLCD.setFont(BigFont);
						test_cabel_N1_run();                                  // ��������� ��������� ��������
					}
				if (((y>=200) && (y<=239)) && ((x>=160) && (x<=315)))         //������ ������ "���������  ��������"
					{
						waitForIt(160, 200, 315, 239);
						myGLCD.setFont(BigFont);
						break;                                                // ����� �� ���������
					}
				}

			}
	 }
	else
	{
	  myGLCD.setColor(VGA_RED);  
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_message[22]))); 
	  myGLCD.print(buffer, CENTER, 80);                                       // txt_error_connect1 "������"
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_message[23]))); 
	  myGLCD.print(buffer, CENTER, 110);                                      // txt_error_connect2 "����������� ������" 
	  myGLCD.setColor(255, 255, 255);  
	  delay(3000);
	}
}
void test_cabel_N2()
{
	myGLCD.clrScr();
	myGLCD.print(txt_menu1_2, CENTER, 1); 
	myGLCD.setColor(255, 255, 255);                                             // ����� ���������
	myGLCD.drawRoundRect (5, 200, 155, 239);
 	myGLCD.drawRoundRect (160, 200, 315, 239);
	myGLCD.drawLine( 10, 60, 310, 60);
	myGLCD.setColor(0, 0, 255);
	myGLCD.fillRoundRect (6, 201, 154, 238);
	myGLCD.fillRoundRect (161, 201, 314, 238);
	myGLCD.setColor(255, 255, 255);  
	myGLCD.setBackColor( 0, 0, 255);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[21]))); 
	myGLCD.print(buffer, 10, 210);                                             //txt_test_repeat  ���������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[20]))); 
	myGLCD.print(buffer, 168, 210);                                            //txt_test_end ���������
	myGLCD.setBackColor( 0, 0, 0);
	mcp_Out2.digitalWrite(14, LOW);                                            // ��������� ���� +12v

	if (search_cabel(1)== 2)
	{
		test_cabel_N2_run();

		while (true)
			{

			if (myTouch.dataAvailable())
				{
				myTouch.read();
				x=myTouch.getX();
				y=myTouch.getY();
		
				if (((y>=200) && (y<=239)) && ((x>=5) && (x<=155)))                    //���������
					{
						waitForIt(5, 200, 155, 239);
						myGLCD.setFont(BigFont);
						test_cabel_N2_run();
					}
				if (((y>=200) && (y<=239)) && ((x>=160) && (x<=315)))                 //���������
					{
						waitForIt(160, 200, 315, 239);
						myGLCD.setFont(BigFont);
						break;
					}
				}
			}
	 }
	else
	{
	  myGLCD.setColor(VGA_RED);  
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_message[22]))); 
	  myGLCD.print(buffer, CENTER, 80);                                       // txt_error_connect1 "������"
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_message[23]))); 
	  myGLCD.print(buffer, CENTER, 110);                                      // txt_error_connect2 "����������� ������" 
	  myGLCD.setColor(255, 255, 255);  
	  delay(3000);
	}
}
void test_cabel_N3()
{
	myGLCD.clrScr();
	myGLCD.print(txt_menu1_3, CENTER, 1); 
	myGLCD.setColor(255, 255, 255);                                             // ����� ���������
	myGLCD.drawRoundRect (5, 200, 155, 239);
 	myGLCD.drawRoundRect (160, 200, 315, 239);
	myGLCD.drawLine( 10, 60, 310, 60);
	myGLCD.setColor(0, 0, 255);
	myGLCD.fillRoundRect (6, 201, 154, 238);
	myGLCD.fillRoundRect (161, 201, 314, 238);
	myGLCD.setColor(255, 255, 255);  
	myGLCD.setBackColor( 0, 0, 255);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[21]))); 
	myGLCD.print(buffer, 10, 210);                                             //txt_test_repeat  ���������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[20]))); 
	myGLCD.print(buffer, 168, 210);                                            //txt_test_end ���������
	myGLCD.setBackColor( 0, 0, 0);
	mcp_Out2.digitalWrite(14, LOW);                                            // ��������� ���� +12v

	if (search_cabel(39)== 3)
	{
		test_cabel_N3_run();

		while (true)
			{
			if (myTouch.dataAvailable())
				{
				myTouch.read();
				x=myTouch.getX();
				y=myTouch.getY();
		
				if (((y>=200) && (y<=239)) && ((x>=5) && (x<=155)))                    //���������
					{
						waitForIt(5, 200, 155, 239);
						myGLCD.setFont(BigFont);
						test_cabel_N3_run();
					}
				if (((y>=200) && (y<=239)) && ((x>=160) && (x<=315)))                 //���������
					{
						waitForIt(160, 200, 315, 239);
						myGLCD.setFont(BigFont);
						break;
					}
				}
			}
	 }
	else
	{
	  myGLCD.setColor(VGA_RED);  
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_message[22]))); 
	  myGLCD.print(buffer, CENTER, 80);                                       // txt_error_connect1 "������"
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_message[23]))); 
	  myGLCD.print(buffer, CENTER, 110);                                      // txt_error_connect2 "����������� ������" 
	  myGLCD.setColor(255, 255, 255);  
	  delay(3000);
	}
}
void test_cabel_N4()
{
	myGLCD.clrScr();
	myGLCD.print(txt_menu1_4, CENTER, 1); 
	myGLCD.setColor(255, 255, 255);                                             // ����� ���������
	myGLCD.drawRoundRect (5, 200, 155, 239);
 	myGLCD.drawRoundRect (160, 200, 315, 239);
	myGLCD.drawLine( 10, 60, 310, 60);
	myGLCD.setColor(0, 0, 255);
	myGLCD.fillRoundRect (6, 201, 154, 238);
	myGLCD.fillRoundRect (161, 201, 314, 238);
	myGLCD.setColor(255, 255, 255);  
	myGLCD.setBackColor( 0, 0, 255);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[21]))); 
	myGLCD.print(buffer, 10, 210);                                             //txt_test_repeat  ���������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[20]))); 
	myGLCD.print(buffer, 168, 210);                                            //txt_test_end ���������
	myGLCD.setBackColor( 0, 0, 0);
	mcp_Out2.digitalWrite(14, LOW);                                            // ��������� ���� +12v

	if (search_cabel(41)== 4)
	{
		test_cabel_N4_run();

		while (true)
			{

			if (myTouch.dataAvailable())
				{
				myTouch.read();
				x=myTouch.getX();
				y=myTouch.getY();
		
				if (((y>=200) && (y<=239)) && ((x>=5) && (x<=155)))                    //���������
					{
						waitForIt(5, 200, 155, 239);
						myGLCD.setFont(BigFont);
						test_cabel_N4_run();
					}
				if (((y>=200) && (y<=239)) && ((x>=160) && (x<=315)))                 //���������
					{
						waitForIt(160, 200, 315, 239);
						myGLCD.setFont(BigFont);
						break;
					}
				}

			}
	 }
	else
	{
	  myGLCD.setColor(VGA_RED);  
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_message[22]))); 
	  myGLCD.print(buffer, CENTER, 80);                                       // txt_error_connect1 "������"
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_message[23]))); 
	  myGLCD.print(buffer, CENTER, 110);                                      // txt_error_connect2 "����������� ������" 
	  myGLCD.setColor(255, 255, 255);  
	  delay(3000);
	}
}
void test_panel_N1()
{
	mcp_Out1.digitalWrite(8,  HIGH);                          // ����� ������ EN ���������� ��������� �����������  1E1  U13
	mcp_Out1.digitalWrite(9,  HIGH);                          // ����� ������ EN ���������� ��������� �����������  1E2  U17
	mcp_Out1.digitalWrite(10, HIGH);                          // ����� ������ EN ���������� ��������� �����������  1E3  U23
	mcp_Out1.digitalWrite(11, HIGH);                          // ����� ������ EN ���������� ��������� �����������  1E4  U14
	mcp_Out1.digitalWrite(12, HIGH);                          // ����� ������ EN ���������� ��������� �����������  1E5  U19 
	mcp_Out1.digitalWrite(13, HIGH);                          // ����� ������ EN ���������� ��������� �����������  1E6  U21 

	mcp_Out2.digitalWrite(8,  HIGH);                          // ����� ������ EN ���������� ��������� �����������  2E1  U15
	mcp_Out2.digitalWrite(9,  HIGH);                          // ����� ������ EN ���������� ��������� �����������  2E2  U18 
	mcp_Out2.digitalWrite(10, HIGH);                          // ����� ������ EN ���������� ��������� �����������  2E3  U22
	mcp_Out2.digitalWrite(11, HIGH);                          // ����� ������ EN ���������� ��������� �����������  2E4  U16
	mcp_Out2.digitalWrite(12, HIGH);                          // ����� ������ EN ���������� ��������� �����������  2E5  U20 
	mcp_Out2.digitalWrite(13, HIGH);                          // ����� ������ EN ���������� ��������� �����������  2E6  U24
	mcp_Out2.digitalWrite(14, LOW);                           // ��������� ���� +12v

	myGLCD.clrScr();
	myGLCD.print(txt_menu2_1, CENTER, 1);                            // "���� ������"
	myGLCD.setColor(255, 255, 255);                                  // ����� ���������
	myGLCD.drawRoundRect (5, 200, 155, 239);
 	myGLCD.drawRoundRect (160, 200, 315, 239);
	myGLCD.drawLine( 10, 60, 310, 60);
	myGLCD.setColor(0, 0, 255);
	myGLCD.fillRoundRect (6, 201, 154, 238);
	myGLCD.fillRoundRect (161, 201, 314, 238);
	myGLCD.setColor(255, 255, 255);  
	myGLCD.setBackColor( 0, 0, 255);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[21]))); 
	myGLCD.print(buffer, 10, 210);                                   //txt_test_repeat  ���������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[20]))); 
	myGLCD.print(buffer, 168, 210);                                  //txt_test_end ���������
	myGLCD.setBackColor( 0, 0, 0);                                             //  

		test_panel_N1run();                                                   // ��������� ��������
		while (true)                                                           // �������� ��������� �������
			{

			if (myTouch.dataAvailable())
				{
				myTouch.read();
				x=myTouch.getX();
				y=myTouch.getY();
		
				if (((y>=200) && (y<=239)) && ((x>=5) && (x<=155)))           //������ ������ "��������� ��������"
					{
						waitForIt(5, 200, 155, 239);
						myGLCD.setFont(BigFont);
						test_panel_N1run();                                   // ��������� ��������� ��������
					}
				if (((y>=200) && (y<=239)) && ((x>=160) && (x<=315)))         //������ ������ "���������  ��������"
					{
						waitForIt(160, 200, 315, 239);
						myGLCD.setFont(BigFont);
						break;                                                // ����� �� ���������
					}
				}

			}

	delay(1000);

	mcp_Out2.digitalWrite(14, LOW);                 // ��������� ����
}
void test_panel_N2()
{
	mcp_Out1.digitalWrite(8,  HIGH);                          // ����� ������ EN ���������� ��������� �����������  1E1  U13
	mcp_Out1.digitalWrite(9,  HIGH);                          // ����� ������ EN ���������� ��������� �����������  1E2  U17
	mcp_Out1.digitalWrite(10, HIGH);                          // ����� ������ EN ���������� ��������� �����������  1E3  U23
	mcp_Out1.digitalWrite(11, HIGH);                          // ����� ������ EN ���������� ��������� �����������  1E4  U14
	mcp_Out1.digitalWrite(12, HIGH);                          // ����� ������ EN ���������� ��������� �����������  1E5  U19 
	mcp_Out1.digitalWrite(13, HIGH);                          // ����� ������ EN ���������� ��������� �����������  1E6  U21 

	mcp_Out2.digitalWrite(8,  HIGH);                          // ����� ������ EN ���������� ��������� �����������  2E1  U15
	mcp_Out2.digitalWrite(9,  HIGH);                          // ����� ������ EN ���������� ��������� �����������  2E2  U18 
	mcp_Out2.digitalWrite(10, HIGH);                          // ����� ������ EN ���������� ��������� �����������  2E3  U22
	mcp_Out2.digitalWrite(11, HIGH);                          // ����� ������ EN ���������� ��������� �����������  2E4  U16
	mcp_Out2.digitalWrite(12, HIGH);                          // ����� ������ EN ���������� ��������� �����������  2E5  U20 
	mcp_Out2.digitalWrite(13, HIGH);                          // ����� ������ EN ���������� ��������� �����������  2E6  U24
    mcp_Out2.digitalWrite(14, LOW);                           // ��������� ���� +12v

	myGLCD.clrScr();
	myGLCD.print(txt_menu2_1, CENTER, 1);                            // "���� ������"
	myGLCD.setColor(255, 255, 255);                                  // ����� ���������
	myGLCD.drawRoundRect (5, 200, 155, 239);
 	myGLCD.drawRoundRect (160, 200, 315, 239);
	myGLCD.drawLine( 10, 60, 310, 60);
	myGLCD.setColor(0, 0, 255);
	myGLCD.fillRoundRect (6, 201, 154, 238);
	myGLCD.fillRoundRect (161, 201, 314, 238);
	myGLCD.setColor(255, 255, 255);  
	myGLCD.setBackColor( 0, 0, 255);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[21]))); 
	myGLCD.print(buffer, 10, 210);                                   //txt_test_repeat  ���������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[20]))); 
	myGLCD.print(buffer, 168, 210);                                  //txt_test_end ���������
	myGLCD.setBackColor( 0, 0, 0);                                             //  

		test_panel_N2run();                                                   // ��������� ��������
		while (true)                                                           // �������� ��������� �������
			{

			if (myTouch.dataAvailable())
				{
				myTouch.read();
				x=myTouch.getX();
				y=myTouch.getY();
		
				if (((y>=200) && (y<=239)) && ((x>=5) && (x<=155)))           //������ ������ "��������� ��������"
					{
						waitForIt(5, 200, 155, 239);
						myGLCD.setFont(BigFont);
						test_panel_N2run();                                   // ��������� ��������� ��������
					}
				if (((y>=200) && (y<=239)) && ((x>=160) && (x<=315)))         //������ ������ "���������  ��������"
					{
						waitForIt(160, 200, 315, 239);
						myGLCD.setFont(BigFont);
						break;                                                // ����� �� ���������
					}
				}

			}

	delay(1000);

	mcp_Out2.digitalWrite(14, LOW);                 // ��������� ����
}

void test_cabel_N1_run()
{
	byte  _size_block = i2c_eeprom_read_byte(deviceaddress,adr_memN1_1);         // �������� ���������� ������� ������������ ������� 
	pinMode(46, OUTPUT);                                                         // ���������� �� ����� ����� ������������ U13,U17,U23 (������� ����� � �� ������ ������)
	pinMode(47, INPUT);                                                          // ���������� �� ����  ����� ������������ U15,U18,U22 (������� ����� � �� �������� ������)
	digitalWrite(47, HIGH);                                                      // ���������� ������� ������� �� ������ 47
	myGLCD.print("                    ",1, 40);                                  // �������� ������� ����������� ��������
	byte canal_N     = 0;                                                        // ���������� �������� � ������ � ������
	unsigned int x_A = 1;                                                        // ���������� ������������ ������ �
	unsigned int x_B = 1;                                                        // ���������� ������������ ������ �
	int x_p          = 1;                                                        // ���������� ������ ������ ������ �� �
	int y_p          = 82;                                                       // ���������� ������ ������ ������ �� �
	int count_error  = 0;                                                        // ������� ���������� ������
	int ware_on      = 0;                                                        // �������� ������ �� ���� ���������
	for(int p = 0;p < 6;p++)                                                     // �������� ���� ������ �� �������
	{
		myGLCD.print("                    ", x_p, y_p);                          // �������� 6 �����
		y_p += 19;
	}
	y_p = 82;                                                                    // ������������ ������ ������ ������ �� �
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[24]))); 
	myGLCD.print(buffer, 50, 65);                                                // txt_error_connect3 "������ ���"       
	if (search_cabel(40)== 1)                                                    // ��������� ������������ ����������� ������ �1
	{
		digitalWrite(46, LOW);                                                   // ���������� ����������� ������� �� ������������ U13,U17,U23
		delay(10);                                                               // ����� �� ������������ ������ 46     
		for (x_A = 1;x_A < _size_block+1;x_A++)                                  // ���������������� ������ ��������� ��������.
		{
			canal_N = i2c_eeprom_read_byte(deviceaddress,adr_memN1_1 + x_A);     // �������� � ������ �� EEPROM
			ware_on = i2c_eeprom_read_byte(deviceaddress,adr_memN1_1 + x_A + (_size_block*4)); // �������� �� ������� ������� ����������.
			if (canal_N == 1)                                                    // 40 ����� ��� �������� ������ ������������ �������
			{
				set_komm_mcp('A', 40,'O');                                       // ���������� ���� ����������� �� ����������� 40 �����
			}
	    	else
			{
	    		set_komm_mcp('A', canal_N,'O');                                  // ���������� ������� ���� �����������
			}
		                                                                      	 // ��������������� ��������� ��� ������ ������� "�"
			                                                                     // ��������� ��� ������ ������� "�"
			for (x_B = 1;x_B < _size_block+1;x_B++)                              // ���������������� ������ ��������� �������� "�" .
			{
				canal_N = i2c_eeprom_read_byte(deviceaddress,adr_memN1_1 + x_B + _size_block); // �������� �� ������� ����� ����� �����������.

				if (canal_N == 1)                                                // 40 ����� ��� �������� ������ ������������ �������
				{
					set_komm_mcp('B', 40,'O');                                   // ���������� ����������� ���� �����������
				}
				else
				{
	    			set_komm_mcp('B', canal_N,'O');                              // ���������� ������� ���� �����������
				}
				// ++++++++++++++++++++++++ �������� �� ���������� � - � +++++++++++++++++++++++++++++++++++
				if (x_A == x_B)    
				{
					myGLCD.printNumI(x_A, 30, 40); 
					if(ware_on == 1)myGLCD.print("<->", 66, 40); 
					else myGLCD.print("<X>", 66, 40); 
					myGLCD.printNumI(canal_N, 130, 40); 
					if (digitalRead(47) == LOW && ware_on == 1)
					{
						myGLCD.print(" - Pass", 170, 40);
					}
					else
					{
						if (digitalRead(47) != LOW && ware_on == 0)                  // ������ ���� ��������
		                {
							myGLCD.print(" - Pass", 170, 40);
						}
						else
						{
							count_error++;
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[25]))); 
							myGLCD.print(buffer, 50, 65);                            // txt_error_connect4
							myGLCD.printNumI(count_error, 190, 65); 

							if ( ware_on == 1)
							{
								if(x_A < 10)
								{
									myGLCD.printNumI(x_A, x_p+13, y_p);              // ������������ ��������� ���������
									myGLCD.print("-", x_p+29, y_p); 
								}
								else
								{
									myGLCD.printNumI(x_A, x_p, y_p);                 // ������������ ��������� ���������
									myGLCD.print("-", x_p+29, y_p); 
								}
								if(canal_N < 10)
								{
									myGLCD.printNumI(canal_N, x_p+32+26, y_p);       // ������������ ��������� ���������
								}
								else
								{
									myGLCD.printNumI(canal_N, x_p+32+10, y_p);       // ������������ ��������� ���������
								}
							}
							else
							{
								if(x_A < 10)
								{
									myGLCD.printNumI(x_A, x_p+13, y_p);              // ������������ ��������� ���������
									myGLCD.print("+", x_p+29, y_p); 
								}
								else
								{
									myGLCD.printNumI(x_A, x_p, y_p);                 // ������������ ��������� ���������
									myGLCD.print("+", x_p+29, y_p); 
								}
								if(canal_N < 10)
								{
									myGLCD.printNumI(canal_N, x_p+32+26, y_p);       // ������������ ��������� ���������
								}
								else
								{
									myGLCD.printNumI(canal_N, x_p+32+10, y_p);       // ������������ ��������� ���������
								}
							}
							y_p += 19;
							if ( y_p > 190)                                          // ����� �� ����� ������� ������
							{
								myGLCD.drawLine( x_p+75, 85, x_p+75, 190);
								x_p +=80;
								y_p = 82;
							}
						}
					}
				}

				//------------------------ ����� �������� �� ���������� ---------------------------------------
			
				//++++++++++++++++++++++++ �������� ��������� �������� �� ��������� ---------------------------
				if (x_A != x_B)                                                      //����������� ������� �� �� ������ ���� ���������
				{
					if (digitalRead(47) == LOW)                                      // ��� ���� ��������
					{
						                                                             // �������� �������������� 3 �������, �������� ������ ����� ����������
						int canal_N_err = i2c_eeprom_read_byte(deviceaddress,adr_memN1_1 + x_A +(_size_block*2)); // �������� �� ������� ����� ����� �����������.
						if (x_B != canal_N_err)                                      // ����������� ���������� �� �������� � �������
						{
							                                                         // �������� �������������� 4 �������
							int canal_N_err = i2c_eeprom_read_byte(deviceaddress,adr_memN1_1 + x_A +(_size_block*3)); // �������� �� ������� ����� ����� �����������.
							if (x_B != canal_N_err)                                  // ����������� ���������� �� �������� � �������
							{
								count_error++;
								strcpy_P(buffer, (char*)pgm_read_word(&(table_message[25]))); 
								myGLCD.print(buffer, 50, 65);                        // txt_error_connect4
								myGLCD.printNumI(count_error, 190, 65); 
								if(x_A < 10)
								{
									myGLCD.printNumI(x_A, x_p+13, y_p);              // ������������ ��������� ���������
									myGLCD.print("+", x_p+29, y_p); 
								}
								else
								{
									myGLCD.printNumI(x_A, x_p, y_p);                 // ������������ ��������� ���������
									myGLCD.print("+", x_p+29, y_p); 
								}
								if(canal_N < 10)
								{
									myGLCD.printNumI(canal_N, x_p+32+26, y_p);       // ������������ ��������� ���������
								}
								else
								{
									myGLCD.printNumI(canal_N, x_p+32+10, y_p);       // ������������ ��������� ���������
								}
								y_p += 19;
								if ( y_p > 190)                                      // ����� �� ����� ������� ������
								{
									myGLCD.drawLine( x_p+75, 85, x_p+75, 190);
									x_p +=80;
									y_p = 82;
								}
							}
						}
					}
				} 	//----------------------- ����� �������� �� ��������� -----------------------------------------
			}
		}
    strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30]))); 
    if(count_error == 0) myGLCD.print(buffer, CENTER, 120);                   // txt__test_end  
	}
	else
	{
		myGLCD.setColor(VGA_RED);  
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[22]))); 
		myGLCD.print(buffer, CENTER, 82+19);                                  // txt_error_connect1 �������� ��� ������ �� ���������
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[23]))); 
		myGLCD.print(buffer, CENTER, 82+38);                                  // txt_error_connect2
		myGLCD.setColor(255, 255, 255);                                       // ������������ ����� �����
		delay(3000);
	}
}
void test_cabel_N2_run()
{
	byte  _size_block = i2c_eeprom_read_byte(deviceaddress,adr_memN1_2);         // �������� ���������� ������� ������������ ������� 
	pinMode(46, OUTPUT);                                                         // ���������� �� ����� ����� ������������ U13,U17,U23 (������� ����� � �� ������ ������)
	pinMode(47, INPUT);                                                          // ���������� �� ����  ����� ������������ U15,U18,U22 (������� ����� � �� �������� ������)
	digitalWrite(47, HIGH);                                                      // ���������� ������� ������� �� ������ 47
	myGLCD.print("                    ",1, 40);                                  // �������� ������� ����������� ��������
	byte canal_N     = 0;                                                        // ���������� �������� � ������ � ������
	unsigned int x_A = 1;                                                        // ���������� ������������ ������ �
	unsigned int x_B = 1;                                                        // ���������� ������������ ������ �
	int x_p          = 1;                                                        // ���������� ������ ������ ������ �� �
	int y_p          = 82;                                                       // ���������� ������ ������ ������ �� �
	int count_error  = 0;                                                        // ������� ���������� ������
	int ware_on      = 0;                                                        // �������� ������ �� ���� ���������
	for(int p = 0;p < 6;p++)                                                     // �������� ���� ������ �� �������
	{
		myGLCD.print("                    ", x_p, y_p);                          // �������� 6 �����
		y_p += 19;
	}
	y_p = 82;                                                                    // ������������ ������ ������ ������ �� �
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[24]))); 
	myGLCD.print(buffer, 50, 65);                                                // txt_error_connect3 "������ ���"    
	if (search_cabel(1)== 2)                                                     // ��������� ������������ ����������� ������ �1
	{
		digitalWrite(46, LOW);                                                   // ���������� ����������� ������� �� ������������ U13,U17,U23
		delay(10);                                                               // ����� �� ������������ ������ 46   
		                                                                         // ������ ��������
		for (x_A = 1;x_A < _size_block+1;x_A++)                                  // ���������������� ������ ��������� ��������.
		{
			canal_N = i2c_eeprom_read_byte(deviceaddress,adr_memN1_2 + x_A);     // �������� � ������ �� EEPROM
			ware_on = i2c_eeprom_read_byte(deviceaddress,adr_memN1_2 + x_A + (_size_block*4)); // �������� �� ������� ������� ����������.
			if (canal_N == 1)                                                    // 40 ����� ��� �������� ������ ������������ �������
			{
				set_komm_mcp('A', 1,'O');                                        // ���������� ���� ����������� �� ����������� 40 �����
			}
		else
			{
	    		set_komm_mcp('A', canal_N,'O');                                  // ���������� ������� ���� �����������
			}
		                                                                      	 // ��������������� ��������� ��� ������ ������� "�"
			                                                                     // ��������� ��� ������ ������� "�"
			for (x_B = 1;x_B < _size_block+1;x_B++)                              // ���������������� ������ ��������� �������� "�" .
			{
				canal_N = i2c_eeprom_read_byte(deviceaddress,adr_memN1_2 + x_B + _size_block); // �������� �� ������� ����� ����� �����������.
				if (canal_N == 1)                                                // 40 ����� ��� �������� ������ ������������ �������
				{
					set_komm_mcp('B', 1,'O');                                    // ���������� ����������� ���� �����������
				}
				else
				{
	    			set_komm_mcp('B', canal_N,'O');                              // ���������� ������� ���� �����������
				}
				// ++++++++++++++++++++++++ �������� �� ���������� � - � +++++++++++++++++++++++++++++++++++
				if (x_A == x_B)    
				{
					myGLCD.printNumI(x_A, 30, 40); 
					if(ware_on == 1)myGLCD.print("<->", 66, 40); 
					else myGLCD.print("<X>", 66, 40); 
					myGLCD.printNumI(canal_N, 130, 40); 
					if (digitalRead(47) == LOW && ware_on == 1)
					{
						myGLCD.print(" - Pass", 170, 40);
					}
					else
					{
						if (digitalRead(47) != LOW && ware_on == 0)                  // ������ ���� ��������
		                {
							myGLCD.print(" - Pass", 170, 40);
						}
						else
						{
							count_error++;
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[25]))); 
							myGLCD.print(buffer, 50, 65);                            // txt_error_connect4							myGLCD.printNumI(count_error, 190, 65); 
							myGLCD.printNumI(count_error, 190, 65); 
							if ( ware_on == 1)
							{
								if(x_A < 10)
								{
									myGLCD.printNumI(x_A, x_p+13, y_p);              // ������������ ��������� ���������
									myGLCD.print("-", x_p+29, y_p); 
								}
								else
								{
									myGLCD.printNumI(x_A, x_p, y_p);                 // ������������ ��������� ���������
									myGLCD.print("-", x_p+29, y_p); 
								}
								if(canal_N < 10)
								{
									myGLCD.printNumI(canal_N, x_p+32+26, y_p);       // ������������ ��������� ���������
								}
								else
								{
									myGLCD.printNumI(canal_N, x_p+32+10, y_p);       // ������������ ��������� ���������
								}
							}
							else
							{
								if(x_A < 10)
								{
									myGLCD.printNumI(x_A, x_p+13, y_p);              // ������������ ��������� ���������
									myGLCD.print("+", x_p+29, y_p); 
								}
								else
								{
									myGLCD.printNumI(x_A, x_p, y_p);                 // ������������ ��������� ���������
									myGLCD.print("+", x_p+29, y_p); 
								}
								if(canal_N < 10)
								{
									myGLCD.printNumI(canal_N, x_p+32+26, y_p);       // ������������ ��������� ���������
								}
								else
								{
									myGLCD.printNumI(canal_N, x_p+32+10, y_p);       // ������������ ��������� ���������
								}
							}
							y_p += 19;
							if ( y_p > 190)                                          // ����� �� ����� ������� ������
							{
								myGLCD.drawLine( x_p+75, 85, x_p+75, 190);
								x_p +=80;
								y_p = 82;
							}
						}
					}
				}

				//------------------------ ����� �������� �� ���������� ---------------------------------------

				
				//++++++++++++++++++++++++ �������� ��������� �������� �� ��������� ---------------------------
				if (x_A != x_B)                                                      //����������� ������� �� �� ������ ���� ���������
				{
					if (digitalRead(47) == LOW)                                      // ��� ���� ��������
					{
						                                                             // �������� �������������� 3 �������, �������� ������ ����� ����������
						int canal_N_err = i2c_eeprom_read_byte(deviceaddress,adr_memN1_2 + x_A +(_size_block*2)); // �������� �� ������� ����� ����� �����������.
						if (x_B != canal_N_err)                                      // ����������� ���������� �� �������� � �������
						{
							                                                         // �������� �������������� 4 �������
							int canal_N_err = i2c_eeprom_read_byte(deviceaddress,adr_memN1_2 + x_A +(_size_block*3)); // �������� �� ������� ����� ����� �����������.
							if (x_B != canal_N_err)                                  // ����������� ���������� �� �������� � �������
							{
								count_error++;
								strcpy_P(buffer, (char*)pgm_read_word(&(table_message[25]))); 
								myGLCD.print(buffer, 50, 65);                        // txt_error_connect4
								myGLCD.printNumI(count_error, 190, 65); 
								if(x_A < 10)
								{
									myGLCD.printNumI(x_A, x_p+13, y_p);              // ������������ ��������� ���������
									myGLCD.print("+", x_p+29, y_p); 
								}
								else
								{
									myGLCD.printNumI(x_A, x_p, y_p);                 // ������������ ��������� ���������
									myGLCD.print("+", x_p+29, y_p); 
								}
								if(canal_N < 10)
								{
									myGLCD.printNumI(canal_N, x_p+32+26, y_p);       // ������������ ��������� ���������
								}
								else
								{
									myGLCD.printNumI(canal_N, x_p+32+10, y_p);       // ������������ ��������� ���������
								}
								y_p += 19;
								if ( y_p > 190)                                      // ����� �� ����� ������� ������
								{
									myGLCD.drawLine( x_p+75, 85, x_p+75, 190);
									x_p +=80;
									y_p = 82;
								}
							}
						}
					}
				}
			//----------------------- ����� �������� �� ��������� -----------------------------------------
			}
		}
    strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30]))); 
    if(count_error == 0) myGLCD.print(buffer, CENTER, 120);                   // txt__test_end  
	}
	else
	{
		myGLCD.setColor(VGA_RED);  
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[22]))); 
		myGLCD.print(buffer, CENTER, 82+19);                                  // txt_error_connect1 �������� ��� ������ �� ���������
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[23]))); 
		myGLCD.print(buffer, CENTER, 82+38);                                  // txt_error_connect2
		myGLCD.setColor(255, 255, 255);                                       // ������������ ����� �����
		delay(3000);
	}
}
void test_cabel_N3_run()
{
 	byte  _size_block = i2c_eeprom_read_byte(deviceaddress,adr_memN1_3);         // �������� ���������� ������� ������������ ������� 
	pinMode(46, OUTPUT);                                                         // ���������� �� ����� ����� ������������ U13,U17,U23 (������� ����� � �� ������ ������)
	pinMode(47, INPUT);                                                          // ���������� �� ����  ����� ������������ U15,U18,U22 (������� ����� � �� �������� ������)
	digitalWrite(47, HIGH);                                                      // ���������� ������� ������� �� ������ 47
	myGLCD.print("                    ",1, 40);                                  // �������� ������� ����������� ��������
	byte canal_N     = 0;                                                        // ���������� �������� � ������ � ������
	unsigned int x_A = 1;                                                        // ���������� ������������ ������ �
	unsigned int x_B = 1;                                                        // ���������� ������������ ������ �
	int x_p          = 1;                                                        // ���������� ������ ������ ������ �� �
	int y_p          = 82;                                                       // ���������� ������ ������ ������ �� �
	int count_error  = 0;                                                        // ������� ���������� ������
	int ware_on      = 0;                                                        // �������� ������ �� ���� ���������
	for(int p = 0;p < 6;p++)                                                     // �������� ���� ������ �� �������
	{
		myGLCD.print("                    ", x_p, y_p);                          // �������� 6 �����
		y_p += 19;
	}
	y_p = 82;                                                                    // ������������ ������ ������ ������ �� �
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[24]))); 
	myGLCD.print(buffer, 50, 65);                                                // txt_error_connect3 "������ ���"    
	if (search_cabel(39)== 3)                                                    // ��������� ������������ ����������� ������ �1
	{
		digitalWrite(46, LOW);                                                   // ���������� ����������� ������� �� ������������ U13,U17,U23
		delay(10);                                                               // ����� �� ������������ ������ 46     
		for (x_A = 1;x_A < _size_block+1;x_A++)                                  // ���������������� ������ ��������� ��������.
		{
			canal_N = i2c_eeprom_read_byte(deviceaddress,adr_memN1_3 + x_A);     // �������� � ������ �� EEPROM
			ware_on = i2c_eeprom_read_byte(deviceaddress,adr_memN1_3 + x_A + (_size_block*4)); // �������� �� ������� ������� ����������.

			if (canal_N == 1)                                                    // 40 ����� ��� �������� ������ ������������ �������
			{
				set_komm_mcp('A', 39,'O');                                       // ���������� ���� ����������� �� ����������� 40 �����
			}
	    	else
			{
	    		set_komm_mcp('A', canal_N,'O');                                  // ���������� ������� ���� �����������
			}
		                                                                      	 // ��������������� ��������� ��� ������ ������� "�"
			                                                                     // ��������� ��� ������ ������� "�"
			for (x_B = 1;x_B < _size_block+1;x_B++)                              // ���������������� ������ ��������� �������� "�" .
			{
				canal_N = i2c_eeprom_read_byte(deviceaddress,adr_memN1_3 + x_B + _size_block); // �������� �� ������� ����� ����� �����������.

				if (canal_N == 1)                                                // 40 ����� ��� �������� ������ ������������ �������
				{
					set_komm_mcp('B', 39,'O');                                   // ���������� ����������� ���� �����������
				}
				else
				{
	    			set_komm_mcp('B', canal_N,'O');                              // ���������� ������� ���� �����������
				}
				// ++++++++++++++++++++++++ �������� �� ���������� � - � +++++++++++++++++++++++++++++++++++
				if (x_A == x_B)    
				{
					myGLCD.printNumI(x_A, 30, 40); 
					if(ware_on == 1)myGLCD.print("<->", 66, 40); 
					else myGLCD.print("<X>", 66, 40); 
					myGLCD.printNumI(canal_N, 130, 40); 
					if (digitalRead(47) == LOW && ware_on == 1)
					{
						myGLCD.print(" - Pass", 170, 40);
					}
					else
					{
						if (digitalRead(47) != LOW && ware_on == 0)                  // ������ ���� ��������
		                {
							myGLCD.print(" - Pass", 170, 40);
						}
						else
						{
							count_error++;
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[25]))); 
							myGLCD.print(buffer, 50, 65);                            // txt_error_connect4							myGLCD.printNumI(count_error, 190, 65); 
							myGLCD.printNumI(count_error, 190, 65); 
							if ( ware_on == 1)
							{
								if(x_A < 10)
								{
									myGLCD.printNumI(x_A, x_p+13, y_p);              // ������������ ��������� ���������
									myGLCD.print("-", x_p+29, y_p); 
								}
								else
								{
									myGLCD.printNumI(x_A, x_p, y_p);                 // ������������ ��������� ���������
									myGLCD.print("-", x_p+29, y_p); 
								}
								if(canal_N < 10)
								{
									myGLCD.printNumI(canal_N, x_p+32+26, y_p);       // ������������ ��������� ���������
								}
								else
								{
									myGLCD.printNumI(canal_N, x_p+32+10, y_p);       // ������������ ��������� ���������
								}
							}
							else
							{
								if(x_A < 10)
								{
									myGLCD.printNumI(x_A, x_p+13, y_p);              // ������������ ��������� ���������
									myGLCD.print("+", x_p+29, y_p); 
								}
								else
								{
									myGLCD.printNumI(x_A, x_p, y_p);                 // ������������ ��������� ���������
									myGLCD.print("+", x_p+29, y_p); 
								}
								if(canal_N < 10)
								{
									myGLCD.printNumI(canal_N, x_p+32+26, y_p);       // ������������ ��������� ���������
								}
								else
								{
									myGLCD.printNumI(canal_N, x_p+32+10, y_p);       // ������������ ��������� ���������
								}
							}
							y_p += 19;
							if ( y_p > 190)                                          // ����� �� ����� ������� ������
							{
								myGLCD.drawLine( x_p+75, 85, x_p+75, 190);
								x_p +=80;
								y_p = 82;
							}
						}
					}
				}

				//------------------------ ����� �������� �� ���������� ---------------------------------------
				
				//++++++++++++++++++++++++ �������� ��������� �������� �� ��������� ---------------------------
				if (x_A != x_B)                                                      //����������� ������� �� �� ������ ���� ���������
				{
					if (digitalRead(47) == LOW)                                      // ��� ���� ��������
					{
						                                                             // �������� �������������� 3 �������, �������� ������ ����� ����������
						int canal_N_err = i2c_eeprom_read_byte(deviceaddress,adr_memN1_3 + x_A +(_size_block*2)); // �������� �� ������� ����� ����� �����������.
						if (x_B != canal_N_err)                                      // ����������� ���������� �� �������� � �������
						{
							                                                         // �������� �������������� 4 �������
							int canal_N_err = i2c_eeprom_read_byte(deviceaddress,adr_memN1_3 + x_A +(_size_block*3)); // �������� �� ������� ����� ����� �����������.
							if (x_B != canal_N_err)                                  // ����������� ���������� �� �������� � �������
							{
								count_error++;
								strcpy_P(buffer, (char*)pgm_read_word(&(table_message[25]))); 
								myGLCD.print(buffer, 50, 65);                        // txt_error_connect4
								myGLCD.printNumI(count_error, 190, 65); 
								if(x_A < 10)
								{
									myGLCD.printNumI(x_A, x_p+13, y_p);              // ������������ ��������� ���������
									myGLCD.print("+", x_p+29, y_p); 
								}
								else
								{
									myGLCD.printNumI(x_A, x_p, y_p);                 // ������������ ��������� ���������
									myGLCD.print("+", x_p+29, y_p); 
								}
								if(canal_N < 10)
								{
									myGLCD.printNumI(canal_N, x_p+32+26, y_p);       // ������������ ��������� ���������
								}
								else
								{
									myGLCD.printNumI(canal_N, x_p+32+10, y_p);       // ������������ ��������� ���������
								}
								y_p += 19;
								if ( y_p > 190)                                      // ����� �� ����� ������� ������
								{
									myGLCD.drawLine( x_p+75, 85, x_p+75, 190);
									x_p +=80;
									y_p = 82;
								}
							}
						}
					}
				}	//----------------------- ����� �������� �� ��������� -----------------------------------------
			}
		}
    strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30]))); 
    if(count_error == 0) myGLCD.print(buffer, CENTER, 120);                   // txt__test_end  
	}
	else
	{
		myGLCD.setColor(VGA_RED);  
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[22]))); 
		myGLCD.print(buffer, CENTER, 82+19);                                  // txt_error_connect1 �������� ��� ������ �� ���������
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[23]))); 
		myGLCD.print(buffer, CENTER, 82+38);                                  // txt_error_connect2
		myGLCD.setColor(255, 255, 255);                                       // ������������ ����� �����
		delay(3000);
	}
}
void test_cabel_N4_run()
{
	byte  _size_block = i2c_eeprom_read_byte(deviceaddress,adr_memN1_4);         // �������� ���������� ������� ������������ ������� 
	pinMode(46, OUTPUT);                                                         // ���������� �� ����� ����� ������������ U13,U17,U23 (������� ����� � �� ������ ������)
	pinMode(47, INPUT);                                                          // ���������� �� ����  ����� ������������ U15,U18,U22 (������� ����� � �� �������� ������)
	digitalWrite(47, HIGH);                                                      // ���������� ������� ������� �� ������ 47
	myGLCD.print("                    ",1, 40);                                  // �������� ������� ����������� ��������
	byte canal_N     = 0;                                                        // ���������� �������� � ������ � ������
	unsigned int x_A = 1;                                                        // ���������� ������������ ������ �
	unsigned int x_B = 1;                                                        // ���������� ������������ ������ �
	int x_p          = 1;                                                        // ���������� ������ ������ ������ �� �
	int y_p          = 82;                                                       // ���������� ������ ������ ������ �� �
	int count_error  = 0;                                                        // ������� ���������� ������
	int ware_on      = 0;                                                        // �������� ������ �� ���� ���������
	for(int p = 0;p < 6;p++)                                                     // �������� ���� ������ �� �������
	{
		myGLCD.print("                    ", x_p, y_p);                          // �������� 6 �����
		y_p += 19;
	}
	y_p = 82;                                                                    // ������������ ������ ������ ������ �� �
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[24]))); 
	myGLCD.print(buffer, 50, 65);                                                // txt_error_connect3 "������ ���"    
	if (search_cabel(41)== 4)                                                    // ��������� ������������ ����������� ������ �1
	{
		digitalWrite(46, LOW);                                                   // ���������� ����������� ������� �� ������������ U13,U17,U23
		delay(10);                                                               // ����� �� ������������ ������ 46     
		for (x_A = 1;x_A < _size_block+1;x_A++)                                  // ���������������� ������ ��������� ��������.
		{
			canal_N = i2c_eeprom_read_byte(deviceaddress,adr_memN1_4 + x_A);     // �������� � ������ �� EEPROM
			ware_on = i2c_eeprom_read_byte(deviceaddress,adr_memN1_4 + x_A + (_size_block*4)); // �������� �� ������� ������� ����������.
			if (canal_N == 1)                                                    // 40 ����� ��� �������� ������ ������������ �������
			{
				set_komm_mcp('A', 41,'O');                                       // ���������� ���� ����������� �� ����������� 40 �����
			}
		    else
			{
	    		set_komm_mcp('A', canal_N,'O');                                  // ���������� ������� ���� �����������
			}
		                                                                      	 // ��������������� ��������� ��� ������ ������� "�"
			                                                                     // ��������� ��� ������ ������� "�"
			for (x_B = 1;x_B < _size_block+1;x_B++)                              // ���������������� ������ ��������� �������� "�" .
			{
				canal_N = i2c_eeprom_read_byte(deviceaddress,adr_memN1_4 + x_B + _size_block); // �������� �� ������� ����� ����� �����������.

				if (canal_N == 1)                                                // 40 ����� ��� �������� ������ ������������ �������
				{
					set_komm_mcp('B', 41,'O');                                   // ���������� ����������� ���� �����������
				}
				else
				{
	    			set_komm_mcp('B', canal_N,'O');                              // ���������� ������� ���� �����������
				}
				// ++++++++++++++++++++++++ �������� �� ���������� � - � +++++++++++++++++++++++++++++++++++
				if (x_A == x_B)    
				{
					myGLCD.printNumI(x_A, 30, 40); 
					if(ware_on == 1)myGLCD.print("<->", 66, 40); 
					else myGLCD.print("<X>", 66, 40); 
					myGLCD.printNumI(canal_N, 130, 40); 
					if (digitalRead(47) == LOW && ware_on == 1)
					{
						myGLCD.print(" - Pass", 170, 40);
					}
					else
					{
						if (digitalRead(47) != LOW && ware_on == 0)                  // ������ ���� ��������
		                {
							myGLCD.print(" - Pass", 170, 40);
						}
						else
						{
							count_error++;
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[25]))); 
							myGLCD.print(buffer, 50, 65);                            // txt_error_connect4
							myGLCD.printNumI(count_error, 190, 65); 

							if ( ware_on == 1)
							{
								if(x_A < 10)
								{
									myGLCD.printNumI(x_A, x_p+13, y_p);              // ������������ ��������� ���������
									myGLCD.print("-", x_p+29, y_p); 
								}
								else
								{
									myGLCD.printNumI(x_A, x_p, y_p);                 // ������������ ��������� ���������
									myGLCD.print("-", x_p+29, y_p); 
								}
								if(canal_N < 10)
								{
									myGLCD.printNumI(canal_N, x_p+32+26, y_p);       // ������������ ��������� ���������
								}
								else
								{
									myGLCD.printNumI(canal_N, x_p+32+10, y_p);       // ������������ ��������� ���������
								}
							}
							else
							{
								if(x_A < 10)
								{
									myGLCD.printNumI(x_A, x_p+13, y_p);              // ������������ ��������� ���������
									myGLCD.print("+", x_p+29, y_p); 
								}
								else
								{
									myGLCD.printNumI(x_A, x_p, y_p);                 // ������������ ��������� ���������
									myGLCD.print("+", x_p+29, y_p); 
								}
								if(canal_N < 10)
								{
									myGLCD.printNumI(canal_N, x_p+32+26, y_p);       // ������������ ��������� ���������
								}
								else
								{
									myGLCD.printNumI(canal_N, x_p+32+10, y_p);       // ������������ ��������� ���������
								}
							}
							y_p += 19;
							if ( y_p > 190)                                          // ����� �� ����� ������� ������
							{
								myGLCD.drawLine( x_p+75, 85, x_p+75, 190);
								x_p +=80;
								y_p = 82;
							}
						}
					}
				}

				//------------------------ ����� �������� �� ���������� ---------------------------------------

				//++++++++++++++++++++++++ �������� ��������� �������� �� ��������� ---------------------------
				if (x_A != x_B)                                                      //����������� ������� �� �� ������ ���� ���������
				{
					if (digitalRead(47) == LOW)                                      // ��� ���� ��������
					{
						                                                             // �������� �������������� 3 �������, �������� ������ ����� ����������
						int canal_N_err = i2c_eeprom_read_byte(deviceaddress,adr_memN1_4 + x_A +(_size_block*2)); // �������� �� ������� ����� ����� �����������.
						if (x_B != canal_N_err)                                      // ����������� ���������� �� �������� � �������
						{
							                                                         // �������� �������������� 4 �������
							int canal_N_err = i2c_eeprom_read_byte(deviceaddress,adr_memN1_4 + x_A +(_size_block*3)); // �������� �� ������� ����� ����� �����������.
							if (x_B != canal_N_err)                                  // ����������� ���������� �� �������� � �������
							{
								count_error++;
								strcpy_P(buffer, (char*)pgm_read_word(&(table_message[25]))); 
								myGLCD.print(buffer, 50, 65);                        // txt_error_connect4
								myGLCD.printNumI(count_error, 190, 65); 
								if(x_A < 10)
								{
									myGLCD.printNumI(x_A, x_p+13, y_p);              // ������������ ��������� ���������
									myGLCD.print("+", x_p+29, y_p); 
								}
								else
								{
									myGLCD.printNumI(x_A, x_p, y_p);                 // ������������ ��������� ���������
									myGLCD.print("+", x_p+29, y_p); 
								}
								if(canal_N < 10)
								{
									myGLCD.printNumI(canal_N, x_p+32+26, y_p);       // ������������ ��������� ���������
								}
								else
								{
									myGLCD.printNumI(canal_N, x_p+32+10, y_p);       // ������������ ��������� ���������
								}
								y_p += 19;
								if ( y_p > 190)                                      // ����� �� ����� ������� ������
								{
									myGLCD.drawLine( x_p+75, 85, x_p+75, 190);
									x_p +=80;
									y_p = 82;
								}
							}
						}
					}
				}  //----------------------- ����� �������� �� ��������� -----------------------------------------
			}
		}
    strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30]))); 
    if(count_error == 0) myGLCD.print(buffer, CENTER, 120);                   // txt__test_end  
	}
	else
	{
		myGLCD.setColor(VGA_RED);  
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[22]))); 
		myGLCD.print(buffer, CENTER, 82+19);                                  // txt_error_connect1 �������� ��� ������ �� ���������
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[23]))); 
		myGLCD.print(buffer, CENTER, 82+38);                                  // txt_error_connect2
		myGLCD.setColor(255, 255, 255);                                       // ������������ ����� �����
		delay(3000);
	}
}
void test_panel_N1run()
{
	mcp_Out1.digitalWrite(8,  HIGH);                                // ����� ������ EN ���������� ��������� �����������  1E1  U13
	mcp_Out1.digitalWrite(9,  HIGH);                                // ����� ������ EN ���������� ��������� �����������  1E2  U17
	mcp_Out1.digitalWrite(10, HIGH);                                // ����� ������ EN ���������� ��������� �����������  1E3  U23
	mcp_Out1.digitalWrite(11, HIGH);                                // ����� ������ EN ���������� ��������� �����������  1E4  U14
	mcp_Out1.digitalWrite(12, HIGH);                                // ����� ������ EN ���������� ��������� �����������  1E5  U19 
	mcp_Out1.digitalWrite(13, HIGH);                                // ����� ������ EN ���������� ��������� �����������  1E6  U21 

	mcp_Out2.digitalWrite(8,  HIGH);                                // ����� ������ EN ���������� ��������� �����������  2E1  U15
	mcp_Out2.digitalWrite(9,  HIGH);                                // ����� ������ EN ���������� ��������� �����������  2E2  U18 
	mcp_Out2.digitalWrite(10, HIGH);                                // ����� ������ EN ���������� ��������� �����������  2E3  U22
	mcp_Out2.digitalWrite(11, HIGH);                                // ����� ������ EN ���������� ��������� �����������  2E4  U16
	mcp_Out2.digitalWrite(12, HIGH);                                // ����� ������ EN ���������� ��������� �����������  2E5  U20 
	mcp_Out2.digitalWrite(13, HIGH);                                // ����� ������ EN ���������� ��������� �����������  2E6  U24
	digitalWrite(48,HIGH);                                          // ��������� ������ U11 ����� �1
	digitalWrite(49,HIGH);                                          // ��������� ������ U11 ����� �2
// ------------------------- �������� ����� �� ���������� ��������� ----------------------------------
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));   // 
	myGLCD.print(buffer, LEFT, 20); 
	myGLCD.print(buffer, LEFT, 38);   
	myGLCD.print(buffer, LEFT, 70);                                 // ����� 1
	myGLCD.print(buffer, LEFT, 85);                                 // ����� 2 
	myGLCD.print(buffer, LEFT, 100);                                // ����� 3  
	myGLCD.print(buffer, LEFT, 115);                                // ����� 4  
	myGLCD.print(buffer, LEFT, 130);                                // ����� 5  
	myGLCD.print(buffer, LEFT, 145);                                // ����� 6  
	myGLCD.print(buffer, LEFT, 160);                                // ����� 7  
	myGLCD.print(buffer, LEFT, 175);                                // ����� 8 
//-----------------------------------------------------------------------------------------------------------
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[31])));   // ����� ��������� "���� �����������"
	myGLCD.print(buffer, CENTER, 20);                               // ����� ��������� "���� �����������"
	myGLCD.setBackColor( 0, 0, 0);                                  //  
	pinMode(led_disp, OUTPUT);                                      //  
	pinMode(led_instr, OUTPUT);                                     //  
	digitalWrite(led_disp,HIGH);
	digitalWrite(led_instr,HIGH);
	mcp_Out2.digitalWrite(14, HIGH);                            // �������� ����. ������ +12� �� ����� 2 ������� J12(b2-12)  �1� �� �������� ������
//------------------------- �������� ����������� �� �������� ������ ����������/����������� ---------------------------------------------
		for (int i=0;i<4;i++)                                        
		{
			digitalWrite(led_disp,LOW);
			delay(250);
			digitalWrite(led_disp,HIGH);
			delay(250);
			digitalWrite(led_instr,LOW);
			delay(250);
			digitalWrite(led_instr,HIGH);
			delay(250);
		}
	pinMode(led_disp, INPUT);                                              //  ������� ������ � �������� ���������
	pinMode(led_instr, INPUT);                                             //  
	delay(1000);
//------------------------------------------------------------------------------------------------------------------------------

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));          // �������� �����
	myGLCD.print(buffer, CENTER, 20);                                      // 
	myGLCD.setBackColor( 0, 0, 0);        

	pinMode(46,INPUT);                                                     // ���������� ����� ����������� �� ����
	digitalWrite(46,HIGH);                                                 // ���������� �������� � �����
	pinMode(47,INPUT);                                                     // ���������� ����� ����������� �� ����
	digitalWrite(47,HIGH);                                                 // ���������� �������� � �����
	// �������� ��� ����������

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[33])));          // ����� ��������� "���� ����������"
	myGLCD.print(buffer, CENTER, 20);                                      // ����� ��������� "���� ����������"
	myGLCD.setBackColor( 0, 0, 0);                                         //  

	set_komm_mcp('A', 34,'O');                                             // ���������� ���������� � ������ 9 �������  
	delay(200);
	if(digitalRead(46)== LOW)                                              // ��������� ����������� �������
	{
      	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[36])));      // ����� ��������� "������ ����. �����."
		myGLCD.print(buffer, CENTER, 38);                                  // RIGHT

		//+++++++++++++++++++++ ��������� ������� �2 ����� ���� 70 +++++++++++++++++++++++++++++++++++++++++++ 
		set_komm_mcp('A', 32,'O');                                              // ���������� ���������� � ������  
		delay(200);
		if(digitalRead(46)== HIGH)                                              // ��������� ����������� �������
		{
			set_komm_mcp('B', 5,'G');                                           // ���������� ���������� � ������ 
			delay(200);
			if(digitalRead(46)== LOW)                                           // ��������� ����������� �������
			{
      			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[42])));  // ������ ����. �����.
				myGLCD.print(buffer, LEFT, 70);     
			}
			else
			{
      			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[52])));  // ������ ����. ����.
				myGLCD.print(buffer, LEFT, 70);      
			}

		}
		else
		{
			// ����� �������� �� �����
      		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[52])));     // ������ ����. ����.
			myGLCD.print(buffer, CENTER, 70);                          
		}

		//+++++++++++++++++++++ ��������� ������� �3 ����� ���� 85 +++++++++++++++++++++++++++++++++++++++++++ 
		set_komm_mcp('B', 40,'O');                                              // ���������� ���������� � ������  
		delay(200);
		if(digitalRead(47)== HIGH)                                              // ��������� ����������� �������
		{
			set_komm_mcp('A', 31,'G');                                           // ���������� ���������� � ������ 
			delay(200);
			if(digitalRead(47)== LOW)                                           // ��������� ����������� �������
			{
      			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[43])));  // ������ ����. �����.
				myGLCD.print(buffer, LEFT, 85);     
			}
			else
			{
      			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[53])));  // ������ ����. ����.
				myGLCD.print(buffer, LEFT, 85);      
			}

		}
		else
		{
			// ����� �������� �� �����
      		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[53])));     // ������ ����. ����.
			myGLCD.print(buffer, CENTER, 85);                          
		}


		/*
	 	set_komm_mcp('A', 31,'O');                                              // ���������� ���������� � ������  
		delay(200);
		if(digitalRead(46)== HIGH)                                              // ��������� ����������� �������
		{
			set_komm_mcp('B', 1,'G');                                           // ���������� ���������� � ������ 
			delay(200);
			if(digitalRead(46)== LOW)                                           // ��������� ����������� �������
			{
      			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[43])));  // ������ ����. �����.
				myGLCD.print(buffer, LEFT, 85);     
			}
			else
			{
      			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[53])));  // ������ ����. ����.
				myGLCD.print(buffer, LEFT, 85);      
			}

		}
		else
		{
			// ����� �������� �� �����
      		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[53])));     // ������ ����. ����.
			myGLCD.print(buffer, CENTER, 85);                          
		}
		*/
		//+++++++++++++++++++++ ��������� ������� �4 ����� ���� 100 +++++++++++++++++++++++++++++++++++++++++++ 
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[44])));        // ����� ��������� "������ ����. �����."
		myGLCD.print(buffer, LEFT, 100);           


		//+++++++++++++++++++++ ��������� ������� �6 ����� ���� 115 +++++++++++++++++++++++++++++++++++++++++++ 
		set_komm_mcp('A', 35,'O');                                              // ���������� ���������� � ������  
		delay(200);
		if(digitalRead(46)== HIGH)                                              // ��������� ����������� �������
		{
			set_komm_mcp('B', 6,'G');                                           // ���������� ���������� � ������ 
			delay(200);
			if(digitalRead(46)== LOW)                                           // ��������� ����������� �������
			{
      			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[46])));  // ������ ����. �����.
				myGLCD.print(buffer, LEFT, 115);     
			}
			else
			{
      			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[56])));  // ������ ����. ����.
				myGLCD.print(buffer, LEFT, 115);      
			}

		}
		else
		{
			// ����� �������� �� �����
      		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[56])));     // ������ ����. ����.
			myGLCD.print(buffer, CENTER, 115);                          
		}


	    //+++++++++++++++++++++ ��������� ������� �7 ����� ���� 130 +++++++++++++++++++++++++++++++++++++++++++ 
		set_komm_mcp('A', 33,'O');                                       // ���������� ���������� � ������  ���
		delay(200);
		if(digitalRead(46)== HIGH)                                              // ��������� ����������� �������
		{
			set_komm_mcp('B', 3,'G');                                           // ���������� ���������� � ������ 
			delay(200);
			if(digitalRead(46)== LOW)                                           // ��������� ����������� �������
			{
      			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[47])));  // ������ ����. �����.
				myGLCD.print(buffer, LEFT, 130);     
			}
			else
			{
      			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[57])));  // ������ ����. ����.
				myGLCD.print(buffer, LEFT, 130);      
			}

		}
		else
		{
			// ����� �������� �� �����
      		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[57])));     // ������ ����. ����.
			myGLCD.print(buffer, CENTER, 130);                          
		}

	    //+++++++++++++++++++++ ��������� ������� �8 ����� ���� 145 +++++++++++++++++++++++++++++++++++++++++++ 
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[48])));      // ����� ��������� "������ ����. �����."
		myGLCD.print(buffer, LEFT, 145);          
		
		//+++++++++++++++++++++ ��������� ������� �9 ����� ���� 160 +++++++++++++++++++++++++++++++++++++++++++ 

		set_komm_mcp('A', 44,'O');                                         // ���������� ���������� � ������ 7 ������� J40   
		delay(200);
		if(digitalRead(46)== LOW)                                          // ��������� ����������� �������
		{
      		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[49])));  // ������ ����. �����.
			myGLCD.print(buffer, LEFT, 160);     //RIGHT
		}
		else
		{
      		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[59])));  // ������ ����. ����.
			myGLCD.print(buffer, LEFT, 160);      
		}
       //----------------------------------------------------------------------

		//set_komm_mcp('B', 3,'G');                                        // ���������� ���������� � ������ 
		//if(digitalRead(46)== LOW)                                        // ��������� ����������� �������
		//{
  //    		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[36]))); // ������ ����. �����.
		//	myGLCD.print(buffer, LEFT, 145);     
		//}
		//else
		//{
  //    		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[37]))); // ������ ����. ����.
		//	myGLCD.print(buffer, LEFT, 145);      
		//}

		

	}
	else  // ��������� ��������. ������ �� ���������!!
	{
      	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[37])));         // ������ ����. ����.
		myGLCD.print(buffer, CENTER, 38);                                     //
	}

//-----------------------------------------------------------------------
	/*
		// �������� ��� �����������
	set_komm_mcp('A', 39,'O');                                        // ���������� ���������� � ������ 3 ������� J39   
	if(digitalRead(46)== LOW)                                         // ��������� ����������� �������
	{
      	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[38]))); // ������ �����. �����.
		myGLCD.print(buffer, LEFT, 85);     
	}
	else
	{
      	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[39]))); // ������ �����. ����.
		myGLCD.print(buffer, LEFT, 85);      
	}
	set_komm_mcp('A', 41,'O');                                       // ���������� ���������� � ������ 7 ������� J39   
	if(digitalRead(46)== LOW)                                        // ��������� ����������� �������
	{
      	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[38]))); // ������ �����. �����.
		myGLCD.print(buffer, LEFT, 115);     
	}
	else
	{
      	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[39]))); // ������ �����. ����.
		myGLCD.print(buffer, LEFT, 115);      
	}



	*/




	//delay(3000);
	digitalWrite(48,HIGH);
	digitalWrite(49,HIGH);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));   // 
	//myGLCD.print(buffer, CENTER, 25);                               // 
	myGLCD.setBackColor( 0, 0, 0);        


	delay(1000);

	mcp_Out2.digitalWrite(14, LOW);                 // ��������� ����
}
void test_panel_N2run()
{
	mcp_Out1.digitalWrite(8,  HIGH);                                // ����� ������ EN ���������� ��������� �����������  1E1  U13
	mcp_Out1.digitalWrite(9,  HIGH);                                // ����� ������ EN ���������� ��������� �����������  1E2  U17
	mcp_Out1.digitalWrite(10, HIGH);                                // ����� ������ EN ���������� ��������� �����������  1E3  U23
	mcp_Out1.digitalWrite(11, HIGH);                                // ����� ������ EN ���������� ��������� �����������  1E4  U14
	mcp_Out1.digitalWrite(12, HIGH);                                // ����� ������ EN ���������� ��������� �����������  1E5  U19 
	mcp_Out1.digitalWrite(13, HIGH);                                // ����� ������ EN ���������� ��������� �����������  1E6  U21 

	mcp_Out2.digitalWrite(8,  HIGH);                                // ����� ������ EN ���������� ��������� �����������  2E1  U15
	mcp_Out2.digitalWrite(9,  HIGH);                                // ����� ������ EN ���������� ��������� �����������  2E2  U18 
	mcp_Out2.digitalWrite(10, HIGH);                                // ����� ������ EN ���������� ��������� �����������  2E3  U22
	mcp_Out2.digitalWrite(11, HIGH);                                // ����� ������ EN ���������� ��������� �����������  2E4  U16
	mcp_Out2.digitalWrite(12, HIGH);                                // ����� ������ EN ���������� ��������� �����������  2E5  U20 
	mcp_Out2.digitalWrite(13, HIGH);                                // ����� ������ EN ���������� ��������� �����������  2E6  U24
	digitalWrite(48,HIGH);                                          // ��������� ������ U11 ����� �1
	digitalWrite(49,HIGH);                                          // ��������� ������ U11 ����� �2                                      // 
// ------------------------- �������� ����� �� ���������� ��������� ----------------------------------
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));   // 
	myGLCD.print(buffer, LEFT, 20); 
	myGLCD.print(buffer, LEFT, 38);   
	myGLCD.print(buffer, LEFT, 70);                                 // ����� 1
	myGLCD.print(buffer, LEFT, 85);                                 // ����� 2 
	myGLCD.print(buffer, LEFT, 100);                                // ����� 3  
	myGLCD.print(buffer, LEFT, 115);                                // ����� 4  
	myGLCD.print(buffer, LEFT, 130);                                // ����� 5  
	myGLCD.print(buffer, LEFT, 145);                                // ����� 6  
	myGLCD.print(buffer, LEFT, 160);                                // ����� 7  
	myGLCD.print(buffer, LEFT, 175);                                // ����� 8 
//-----------------------------------------------------------------------------------------------------------

	pinMode(led_disp, OUTPUT);                                      //  
	pinMode(led_instr, OUTPUT);                                     //  
	digitalWrite(led_disp,HIGH);
	digitalWrite(led_instr,HIGH);
	mcp_Out2.digitalWrite(14, HIGH);                            // �������� ����. ������ +12� �� ����� 2 ������� J12(b2-12)  �1� �� �������� ������

	delay(1000);
//------------------------------------------------------------------------------------------------------------------------------

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));          // �������� �����
	myGLCD.print(buffer, CENTER, 20);                                      // 
	myGLCD.setBackColor( 0, 0, 0);        

	pinMode(46,INPUT);                                                     // ���������� ����� ����������� �� ����
	//digitalWrite(46,HIGH);                                                 // ���������� �������� � �����
	pinMode(47,INPUT);                                                     // ���������� ����� ����������� �� ����
	//digitalWrite(47,HIGH);                                                 // ���������� �������� � �����
	// �������� ��� ����������

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[33])));          // ����� ��������� "���� ����������"
	myGLCD.print(buffer, CENTER, 20);                                      // ����� ��������� "���� ����������"
	myGLCD.setBackColor( 0, 0, 0);                                         //  

		set_komm_mcp('B', 40,'O');                                              // ���������� ���������� � ������  
		delay(200);
        Serial.println(analogRead(A9));
		if(analogRead(A9)> 512)                                              // ��������� ����������� �������
		{
			set_komm_mcp('A', 31,'G');                                           // ���������� ���������� � ������ 
			delay(2500);
			Serial.println(analogRead(A9));
			if(analogRead(A9)< 512)                                           // ��������� ����������� �������
			{
      			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[43])));  // ������ ����. �����.
				myGLCD.print(buffer, LEFT, 85);     
			}
			else
			{
      			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[53])));  // ������ ����. ����.
				myGLCD.print(buffer, LEFT, 85);      
			}

		}
		else
		{
			// ����� �������� �� �����
      		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[53])));     // ������ ����. ����.
			myGLCD.print(buffer, CENTER, 85);                          
		}
	//delay(3000);
	digitalWrite(48,HIGH);
	digitalWrite(49,HIGH);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));   // 
	//myGLCD.print(buffer, CENTER, 25);                               // 
	myGLCD.setBackColor( 0, 0, 0);        


	delay(1000);

//	mcp_Out2.digitalWrite(14, LOW);                 // ��������� ����
}
void test_panel_N3run()
 {
   mcp_Out2.digitalWrite(14, LOW);                 // ��������� ����
 }


void test_all_pin()
{
	myGLCD.clrScr();
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[17]))); 
	myGLCD.print(buffer, CENTER, 20);                                           // txt_test_all
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[18]))); 
	myGLCD.print(buffer, CENTER, 180);                                          // txt_test_all_exit1
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[19]))); 
	myGLCD.print(buffer, CENTER, 200);                                          // txt_test_all_exit2
	byte canal_N = 0;
	pinMode(47, INPUT);                                                         // ���������� �� ����  ����� ������������ U15,U18,U22 (������� ����� � �� �������� ������)
	pinMode(46, INPUT);                                                         // ���������� �� ����  ����� ������������ U13,U17,U23 (������� ����� � �� ������ ������)
	digitalWrite(47, HIGH);                                                     // ���������� ������� ������� �� ������ 47
	digitalWrite(46, HIGH);                                                     // ���������� ������� ������� �� ������ 46
	int i_step = 1;

	while (true)
	  {
		if (myTouch.dataAvailable())
		{
		  myTouch.read();
		  x=myTouch.getX();
		  y=myTouch.getY();
		
		if (((y>=1) && (y<=150)) && ((x>=10) && (x<=319))) //�������
		  {
			myGLCD.setFont(BigFont);
			break;
		  }
		}
	    set_komm_mcp('A', i_step,'O');                                          // ����������� ���������� �������� ����� "�" �� ����
		set_komm_mcp('B', i_step,'O');                                          // ����������� ���������� �������� ����� "�" �� ����
		delay(10);
		if (digitalRead(47) == LOW) 
			{
				myGLCD.print("A", CENTER, 80);
				myGLCD.print("  ", CENTER, 105);
				if (i_step == 39 ||i_step == 40 ||  i_step == 41)
				{
					myGLCD.print("1", CENTER, 105);
				}
				else
				{
					myGLCD.printNumI(i_step, CENTER, 105);
				}
			}
		else if (digitalRead(46) == LOW) 
			{
				myGLCD.print("B", CENTER, 80);
				myGLCD.print("  ", CENTER, 105);
				if (i_step == 39 ||i_step == 40 ||  i_step == 41)
				{
					myGLCD.print("1", CENTER, 105);
				}
				else
				{
					myGLCD.printNumI(i_step, CENTER, 105);
				}
			}
		  i_step++;
		  if (i_step == 42) i_step = 1;
	  }
}

//+++++++++++++++++++++ ����������� +++++++++++++++++++++++++++++


ISR(ADC_vect)  
{  
  // PORTB = B00000000; // ��� 12 ��������� � ��������� LOW
 // PORTB = B01000000; // ��� 12 ��������� � ��������� HIGH

   unsigned int	 analogValue = ADCL; // ��������� ������� ���� ���������� ���
    analogValue += ADCH << 8; // ��������� ������� ���� ���
	Sample_osc[i_osc][0] =  analogValue;
//	Sample_osc[i_osc][0] = (ADCL|ADCH << 8);   // ���������  ADC; 
    i_osc++; 

	 if(i_osc>=240)
	 {
      ADCSRA &= ~(1 << ADIE); //���������
 	  ADC_end = true;
	  i_osc=0;
	 }
   // PORTB = B01000000; // ��� 12 ��������� � ��������� HIGH


	/*
  // Read ADC data.
#if RECORD_EIGHT_BITS
  uint8_t d = ADCH;
#else  // RECORD_EIGHT_BITS
  // This will access ADCL first. 
  uint16_t d = ADC;
#endif  // RECORD_EIGHT_BITS

  if (isrBufNeeded && emptyHead == emptyTail) {
    // no buffers - count overrun
    if (isrOver < 0XFFFF) isrOver++;
    
    // Avoid missed timer error.
    timerFlag = false;
    return;
  }
  // Start ADC
  if (PIN_COUNT > 1) {
    ADMUX = adcmux[adcindex];
    ADCSRB = adcsrb[adcindex];
    ADCSRA = adcsra[adcindex];
    if (adcindex == 0) timerFlag = false;
    adcindex =  adcindex < (PIN_COUNT - 1) ? adcindex + 1 : 0;
  } else {
    timerFlag = false;
  }
  // Check for buffer needed.
  if (isrBufNeeded) {   
    // Remove buffer from empty queue.
    isrBuf = emptyQueue[emptyTail];
    emptyTail = queueNext(emptyTail);
    isrBuf->count = 0;
    isrBuf->overrun = isrOver;
    isrBufNeeded = false;    
  }
  // Store ADC data.
  isrBuf->data[isrBuf->count++] = d;

  // Check for buffer full.
  if (isrBuf->count >= PIN_COUNT*SAMPLES_PER_BLOCK) {
    // Put buffer isrIn full queue.  
    uint8_t tmp = fullHead;  // Avoid extra fetch of volatile fullHead.
    fullQueue[tmp] = (block_t*)isrBuf;
    fullHead = queueNext(tmp);
   
    // Set buffer needed and clear overruns.
    isrBufNeeded = true;
    isrOver = 0;
  }
  */

  // int  d  = (ADCL|ADCH << 8);      // ���������  ADC; 

 //  Sample_osc[i_osc][0] = (ADCL|ADCH << 8);   // ���������  ADC; 

 //   i_osc++; 



 /* if(i_osc==240)  
      { */
//        UART_SendByte(170); 
//        UART_SendByte(204); 
//        UART_SendByte(195); 
//        for (i=0; i<800; i++)  UART_SendByte(MyBuff[i]);  
//		  cli(); // ��������� ��������� ����������

	//	ADCSRA |=   (0 << ADIE);
/*        ADC_end = true;
        i_osc=0; 
      }   */     
}  

/*** ��������� ��� ***/
//ADCSRA |= (1 << ADEN) // ��������� ���
//             |(1 << ADPS1)|(1 << ADPS0);    // ������������ ��������������� �� 8
//ADMUX |= (0 << REFS1)|(0 << REFS0) // ������� ���
//            |(0 << MUX0)|(0 << MUX1)|(0 << MUX2)|(0 << MUX3); // ���� PC0
 
//ADCSRA |= (1 << ADSC);    // �������� ��������������
//while ((ADCSRA&(1 << ADIF))== 0); // ���� ����� ��������� ��������������    
//  
//u = (ADCL|ADCH << 8); // ���������  ADC
/*
  7     6     5    4     3    2   1    0
REFS1|REFS0|ADLAR|MUX4|MUX3|MUX2|MUX1|MUX0|

MUX4...0  | ���� ���   Mega
00000     | ADC0        A0
00001     | ADC1        A1
00010     | ADC2        A2
00011     | ADC3        A3
00100     | ADC4        A4
00101     | ADC5        A5
00110     | ADC6        A6
00111     | ADC7        A7

������������� ���� ���������� ������


010000    | ADC0  
010001    | ADC1  
010010    | ADC2  
010011    | ADC3 
010100    | ADC4 
010101    | ADC5 
010110    | ADC6 
010111    | ADC7  

ADCSRB � ADC Control and Status Register B
   Bit   7     6    5   4    3       2       1       0
(0x7B) | � | ACME | � | � | MUX5 | ADTS2 | ADTS1 | ADTS0       ADCSRB
Bit 3 � MUX5: Analog Channel and Gain Selection Bit

100000    | ADC8
100001    | ADC9
100010    | ADC10
100011    | ADC11
100100    | ADC12
100101    | ADC13
100110    | ADC14
100111    | ADC15

*/

//------------------------------------------------------------------------------
// timer1 interrupt to clear OCF1B
ISR(TIMER1_COMPB_vect) 
{
	digitalWrite(ledPin13, !digitalRead(ledPin13));      
  // Make sure ADC ISR responded to timer event.
  if (timerFlag) timerError = true;
  timerFlag = true;
}
//==============================================================================
// Error messages stored in flash.
#define error(msg) error_P(PSTR(msg))
//------------------------------------------------------------------------------
void error_P(const char* msg) 
{
//  sd.errorPrint_P(msg);
  fatalBlink();
}
//------------------------------------------------------------------------------
//
void fatalBlink() 
{
  while (true) {
    if (ERROR_LED_PIN >= 0) {
      digitalWrite(ERROR_LED_PIN, HIGH);
      delay(200);
      digitalWrite(ERROR_LED_PIN, LOW);
      delay(200);
    }
  }
}
//==============================================================================
#if ADPS0 != 0 || ADPS1 != 1 || ADPS2 != 2
#error unexpected ADC prescaler bits
#endif
//------------------------------------------------------------------------------
// initialize ADC and timer1
void adcInit(metadata_t* meta) 
{
  uint8_t adps;  // prescaler bits for ADCSRA
  uint32_t ticks = F_CPU*SAMPLE_INTERVAL + 0.5;  // Sample interval cpu cycles.

  if (ADC_REF & ~((1 << REFS0) | (1 << REFS1))) 
  {
    //error("Invalid ADC reference");
  }
#ifdef ADC_PRESCALER
  if (ADC_PRESCALER > 7 || ADC_PRESCALER < 2) 
  {
   // error("Invalid ADC prescaler");
  }
  adps = ADC_PRESCALER;
#else  // ADC_PRESCALER
  // Allow extra cpu cycles to change ADC settings if more than one pin.
  int32_t adcCycles = (ticks - ISR_TIMER0)/PIN_COUNT;
  - (PIN_COUNT > 1 ? ISR_SETUP_ADC : 0);

  for (adps = 7; adps > 0; adps--) 
  {
    if (adcCycles >= (MIN_ADC_CYCLES << adps)) 
	{
      break;
    }
  }
#endif  // ADC_PRESCALER
  meta->adcFrequency = F_CPU >> adps;
  if (meta->adcFrequency > (RECORD_EIGHT_BITS ? 2000000 : 1000000)) 
  {
   // error("Sample Rate Too High");
  }
#if ROUND_SAMPLE_INTERVAL
  // Round so interval is multiple of ADC clock.
  ticks += 1 << (adps - 1);
  ticks >>= adps;
  ticks <<= adps;
#endif  // ROUND_SAMPLE_INTERVAL

  if (PIN_COUNT > sizeof(meta->pinNumber)/sizeof(meta->pinNumber[0])) 
  {
   // error("Too many pins");
  }
  meta->pinCount = PIN_COUNT;
  meta->recordEightBits = RECORD_EIGHT_BITS;

  for (int i = 0; i < PIN_COUNT; i++) 
  {
	uint8_t pin = PIN_LIST[i];
	if (pin >= NUM_ANALOG_INPUTS) 
		{
		  //error("Invalid Analog pin number");
		}
		meta->pinNumber[i] = pin;

		// Set ADC reference and low three bits of analog pin number.
		adcmux[i] = (pin & 7) | ADC_REF;
		if (RECORD_EIGHT_BITS)
		{
		  adcmux[i] |= 1 << ADLAR;
		}

		// If this is the first pin, trigger on timer/counter 1 compare match B.
		adcsrb[i] = i == 0 ? (1 << ADTS2) | (1 << ADTS0) : 0;
	#ifdef MUX5
		if (pin > 7) 
		{
		  adcsrb[i] |= (1 << MUX5);
		}
	#endif  // MUX5
		adcsra[i] = (1 << ADEN) | (1 << ADIE) | adps;
		adcsra[i] |= i == 0 ? 1 << ADATE : 1 << ADSC;
  }

  // Setup timer1
  TCCR1A = 0;
  uint8_t tshift;
  if (ticks < 0X10000) 
  {
    // no prescale, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
    tshift = 0;
  }
  else if (ticks < 0X10000*8) 
  {
    // prescale 8, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    tshift = 3;
  } 
  else if (ticks < 0X10000*64) 
  {
    // prescale 64, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);
    tshift = 6;
  }
  else if (ticks < 0X10000*256) 
  {
    // prescale 256, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12);
    tshift = 8;
  } 
  else if (ticks < 0X10000*1024) 
  {
    // prescale 1024, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12) | (1 << CS10);
    tshift = 10;
  }
  else 
  {
   // error("Sample Rate Too Slow");
  }
  // divide by prescaler
  ticks >>= tshift;
  // set TOP for timer reset
  ICR1 = ticks - 1;
  // compare for ADC start
  OCR1B = 0;

  // multiply by prescaler
  ticks <<= tshift;

  // Sample interval in CPU clock ticks.
  meta->sampleInterval = ticks;
  meta->cpuFrequency = F_CPU;
  float sampleRate = (float)meta->cpuFrequency/meta->sampleInterval;
  Serial.print(F("Sample pins:"));
  for (int i = 0; i < meta->pinCount; i++) 
  {
    Serial.print(' ');
    Serial.print(meta->pinNumber[i], DEC);
  }
  Serial.println();
  Serial.print(F("ADC bits: "));
  Serial.println(meta->recordEightBits ? 8 : 10);
  Serial.print(F("ADC clock kHz: "));
  Serial.println(meta->adcFrequency/1000);
  Serial.print(F("Sample Rate: "));
  Serial.println(sampleRate);
  Serial.print(F("Sample interval usec: "));
  Serial.println(1000000.0/sampleRate, 4);
}
//------------------------------------------------------------------------------
// enable ADC and timer1 interrupts
void adcStart() 
{
 // initialize ISR
  isrBufNeeded = true;
  isrOver = 0;
  adcindex = 1;

  // Clear any pending interrupt.
  ADCSRA |= 1 << ADIF;

  // Setup for first pin.
  ADMUX = adcmux[0];
  ADCSRB = adcsrb[0];
  ADCSRA = adcsra[0];

  // Enable timer1 interrupts.
  timerError = false;
  timerFlag = false;
  TCNT1 = 0;
  TIFR1 = 1 << OCF1B;
  TIMSK1 = 1 << OCIE1B;
}
//------------------------------------------------------------------------------
void adcStop()
{
  TIMSK1 = 0;
  ADCSRA = 0;
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// log data
// max number of blocks to erase per erase call
uint32_t const ERASE_SIZE = 262144L;
void logData() 
{
	myGLCD.clrScr();
	myGLCD.setBackColor( 0, 0, 0);
	delay(500);
	myGLCD.clrScr();
	buttons_right();
	buttons_channel();
	myGLCD.setBackColor( 0, 0, 0);
	myGLCD.setFont( BigFont);
	myGLCD.setColor(VGA_LIME);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[15]))); 
	myGLCD.print(buffer,LEFT, 180);                                  // txt_info29
	mode1 = 2;
	int x_dTime;
	int xpos;
	int ypos1;
	int ypos2;

	int ypos_osc1_0;
	int ypos_osc1_1;
	int ypos_osc1_2;
	int ypos_osc1_3;

	int ypos_osc2_0;
	int ypos_osc2_1;
	int ypos_osc2_2;
	int ypos_osc2_3;

	ADC_end = false;

			ADCSRA |=(1 << ADEN);                                // ���������� ���
			ADCSRA |=(1 << ADATE);                               // ����������� ����� ������ ��� �
	//		ADCSRA |=(1 << ADIE);                                // ���������� ���������� �� ���
	//		ADCSRA |=(0 << ADIF);                                // ���� ���������� � "0"
			ADCSRA |=(1 << ADPS2)|(0 << ADPS1)|(0 << ADPS0);     // ������������ �� 
			ADCSRA |=(0 << ADSC) ;                               // ADC Start Conversion ���������

			ADMUX   =(0<<ADLAR)                                 // ��������� �������������� ������������� �� ������ ������� 
					|(0<<REFS1)|(1<<REFS0)                      // ���������� �������� �������� ����������  5�.   
					|(0 << MUX4) |(0 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0) ; //  ���������� ���� �14
			ADCSRB|= (1 << MUX5);                               //  ���������� ���� �14


	for( xpos = 0; xpos < 239;	xpos ++) // ������� ������ ������

		{
			OldSample_osc[xpos][0] = 0;
			OldSample_osc[xpos][1] = 0;
		}

	while(1) 
	{
		 DrawGrid();
		 if (myTouch.dataAvailable())
			{
				delay(10);
				myTouch.read();
				x_osc=myTouch.getX();
				y_osc=myTouch.getY();

				if ((x_osc>=2) && (x_osc<=240))  //  ������� ������
					{
						if ((y_osc>=1) && (y_osc<=160))  // Delay row
						{
							break;
						} 
					}

				myGLCD.setBackColor( 0, 0, 255);
				myGLCD.setFont( SmallFont);
				myGLCD.setColor (255, 255,255);
				myGLCD.drawRoundRect (250, 1, 318, 40);
				myGLCD.drawRoundRect (250, 45, 318, 85);
				myGLCD.drawRoundRect (250, 90, 318, 130);
				myGLCD.drawRoundRect (250, 135, 318, 175);

			if ((x_osc>=250) && (x_osc<=284))  // ������� ������
			  {
				  if ((y_osc>=1) && (y_osc<=40))  // ������  ������
				  {
					waitForIt(250, 1, 318, 40);
					mode -- ;
					if (mode < 0) mode = 0;   
					// Select delay times you can change values to suite your needs
					if (mode == 0) {dTime = 1;    x_dTime = 282;}
					if (mode == 1) {dTime = 10;   x_dTime = 278;}
					if (mode == 2) {dTime = 20;   x_dTime = 278;}
					if (mode == 3) {dTime = 50;   x_dTime = 278;}
					if (mode == 4) {dTime = 100;  x_dTime = 274;}
					if (mode == 5) {dTime = 200;  x_dTime = 274;}
					if (mode == 6) {dTime = 300;  x_dTime = 274;}
					if (mode == 7) {dTime = 500;  x_dTime = 274;}
					if (mode == 8) {dTime = 1000; x_dTime = 270;}
					if (mode == 9) {dTime = 5000; x_dTime = 270;}
					myGLCD.print("    ", 270, 22);
					myGLCD.printNumI(dTime, x_dTime, 22);
				  }

			 if ((y_osc>=45) && (y_osc<=85))  // ������ - �������
				 {
					waitForIt(250, 45, 318, 85);
					tmode --;
					if (tmode < 0)tmode = 0;
					if (tmode == 1){ Trigger = MinAnalog+10; myGLCD.print(" 0%  ", 268, 65);}
					if (tmode == 2){ Trigger = MaxAnalog/2;  myGLCD.print(" 50% ", 266, 65);}
					if (tmode == 3){ Trigger = MaxAnalog-10; myGLCD.print("100%", 270, 65);}
					if (tmode == 0)myGLCD.print(" Off ", 268, 65);
				 }
			 if ((y_osc>=90) && (y_osc<=130))  // ������ - ��������
				 {
					waitForIt(250, 90, 318, 130);
					mode1 -- ;
					myGLCD.setColor( 0, 0, 0);
					myGLCD.fillRoundRect (1, 1,239, 159);
					myGLCD.setColor (255, 255, 255);
					myGLCD.setBackColor( 0, 0, 255);
					myGLCD.setFont( SmallFont);
					if (mode1 < 0) mode1 = 0;   
					if (mode1 == 0){ koeff_h = 7.759*4; myGLCD.print(" 1  ", 275, 110);}
					if (mode1 == 1){ koeff_h = 3.879*4; myGLCD.print("0.5 ", 275, 110);}
					if (mode1 == 2){ koeff_h = 1.939*4; myGLCD.print("0.25", 275, 110);}
					if (mode1 == 3){ koeff_h = 0.969*4; myGLCD.print("0.1 ", 275, 110);}
				 }
			 if ((y_osc>=135) && (y_osc<=175))  // ��������� ����������
				 {

				 }
		   }
		
			if ((x_osc>=284) && (x_osc<=318))  // ������� ������
			  {
				  if ((y_osc>=1) && (y_osc<=40))  // ������  ������
				  {
					waitForIt(250, 1, 318, 40);
					mode ++ ;
					if (mode > 9) mode = 9;   
					if (mode == 0) {dTime = 1;    x_dTime = 282;}
					if (mode == 1) {dTime = 10;   x_dTime = 278;}
					if (mode == 2) {dTime = 20;   x_dTime = 278;}
					if (mode == 3) {dTime = 50;   x_dTime = 278;}
					if (mode == 4) {dTime = 100;  x_dTime = 274;}
					if (mode == 5) {dTime = 200;  x_dTime = 274;}
					if (mode == 6) {dTime = 300;  x_dTime = 274;}
					if (mode == 7) {dTime = 500;  x_dTime = 274;}
					if (mode == 8) {dTime = 1000; x_dTime = 270;}
					if (mode == 9) {dTime = 5000; x_dTime = 270;}
					myGLCD.print("    ", 270, 22);
					myGLCD.printNumI(dTime, x_dTime, 22);
				  }

			 if ((y_osc>=45) && (y_osc<=85))  // ������ - �������
				 {
					waitForIt(250, 45, 318, 85);
					tmode ++;
					if (tmode > 3)tmode = 3;
					if (tmode == 1){ Trigger = MinAnalog+10; myGLCD.print(" 0%  ", 268, 65);}
					if (tmode == 2){ Trigger = MaxAnalog/2;  myGLCD.print(" 50% ", 266, 65);}
					if (tmode == 3){ Trigger = MaxAnalog-10; myGLCD.print("100%", 270, 65);}
					if (tmode == 0)myGLCD.print(" Off ", 268, 65);
				 }
			 if ((y_osc>=90) && (y_osc<=130))  // ������ - ��������
				 {
					waitForIt(250, 90, 318, 130);
					mode1 ++ ;
					myGLCD.setColor( 0, 0, 0);
					myGLCD.fillRoundRect (1, 1,239, 159);
					myGLCD.setColor (255, 255, 255);
					myGLCD.setBackColor( 0, 0, 255);
					myGLCD.setFont( SmallFont);
					if (mode1 > 3) mode1 = 3;   
					if (mode1 == 0){ koeff_h = 7.759*4; myGLCD.print(" 1  ", 275, 110);}
					if (mode1 == 1){ koeff_h = 3.879*4; myGLCD.print("0.5 ", 275, 110);}
					if (mode1 == 2){ koeff_h = 1.939*4; myGLCD.print("0.25", 275, 110);}
					if (mode1 == 3){ koeff_h = 0.969*4; myGLCD.print("0.1 ", 275, 110);}
				 }
			 if ((y_osc>=135) && (y_osc<=175))  // ��������� ����������
				 {
					waitForIt(250, 135, 318, 175);
				 }
		   }

		if ((x_osc>=250) && (x_osc<=318))  

			{
			if ((y_osc>=200) && (y_osc<=239))  //   ������ ������  
				{
					waitForIt(250, 200, 318, 238);
					Channel_trig = 0;
					t_in_mode ++;
						if (t_in_mode > 3)
							{
								t_in_mode = 0;
							}
						switch_trig(t_in_mode);
						myGLCD.setBackColor( 0, 0, 255);
						myGLCD.setColor (255, 255,255);
						myGLCD.printNumI(t_in_mode, 282, 214);
				}
		  }

			 if ((y_osc>=205) && (y_osc<=239))  // ������ ������ ������������ ������
					{
						 touch_osc();
					}
		}

		// trig_min_max(t_in_mode);
		// if (tmode>0) trigger();
		//    trigger();
			//ADCSRA |=(1 << ADEN);                                // ���������� ���
			//ADCSRA |=(1 << ADATE);                               // ����������� ����� ������ ���
			ADCSRA |=(1 << ADIE);                                  // ���������� ���������� �� ���
		//	ADCSRA |=(0 << ADIF);                                  // ���� ���������� � "0"
		//    PORTB = B00000000; // ��� 12 ��������� � ��������� LOW
 // PORTB = B01000000; // ��� 12 ��������� � ��������� HIGH

			//ADCSRA |=(1 << ADPS2)|(0 << ADPS1)|(0 << ADPS0);     // ������������ �� 
			//ADCSRA  |=(1 << ADSC) ;                              // ADC Start Conversion

			//ADMUX   =(0<<ADLAR)                                 // ��������� �������������� ������������� �� ������ ������� 
			//		|(0<<REFS1)|(1<<REFS0)                      // ���������� �������� �������� ����������  5�.   
			//		|(0 << MUX4) |(0 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0) ; //  ���������� ���� �14
			//ADCSRB|= (1 << MUX5);                               //  ���������� ���� �14


		    ADCSRA  |=(1 << ADSC) ;  // ADC Start Conversion

		//	while(!ADIF){}
			//if (ADIF) PORTB = B01000000; // ��� 12 ��������� � ��������� HIGH
			while(ADC_end){}
	
			ADC_end = false;
		// �������� ���������� ������ � ���� ������
		StartSample = micros();


		EndSample = micros();
		DrawGrid();
 
		for( int xpos = 0; xpos < 238;	xpos ++)
			{
				//  ������� ���������� �����
				myGLCD.setColor( 0, 0, 0);
			
				if (Channel0 | osc_line_off0)
					{
						ypos_osc1_0 = 255-(OldSample_osc[ xpos + 1][0]/koeff_h) - hpos; 
						ypos_osc2_0 = 255-(OldSample_osc[ xpos + 2][0]/koeff_h) - hpos;
						if(ypos_osc1_0 < 0) ypos_osc1_0 = 0;
						if(ypos_osc2_0 < 0) ypos_osc2_0 = 0;
						if(ypos_osc1_0 > 220) ypos_osc1_0 = 220;
						if(ypos_osc2_0 > 220) ypos_osc2_0 = 220;
						myGLCD.drawLine (xpos + 1, ypos_osc1_0, xpos + 2, ypos_osc2_0);
						myGLCD.drawLine (xpos + 2, ypos_osc1_0+1, xpos + 3, ypos_osc2_0+1);

						if (xpos > 237 & Channel0 == false )
							{
								osc_line_off0 = false;
							}
					}
			
				if (Channel1|osc_line_off1)
					{
						ypos_osc1_1 = 255-(OldSample_osc[ xpos + 1][1]/koeff_h) - hpos; 
						ypos_osc2_1 = 255-(OldSample_osc[ xpos + 2][1]/koeff_h) - hpos;
						if(ypos_osc1_1 < 0) ypos_osc1_1 = 0;
						if(ypos_osc2_1 < 0) ypos_osc2_1 = 0;
						if(ypos_osc1_1 > 220) ypos_osc1_1 = 220;
						if(ypos_osc2_1 > 220) ypos_osc2_1 = 220;
						myGLCD.drawLine (xpos + 1, ypos_osc1_1, xpos + 2, ypos_osc2_1);
						myGLCD.drawLine (xpos + 2, ypos_osc1_1+1, xpos + 3, ypos_osc2_1+1);
						if (xpos > 237 & Channel1 == false )
							{
								osc_line_off1 = false;
							}
					}

					if (xpos == 0)
						{
							myGLCD.drawLine (xpos + 1, 1, xpos + 1, 220);
							myGLCD.drawLine (xpos + 2, 1, xpos + 2, 220);
						}
					
				if (Channel0)
					{
						myGLCD.setColor( 255, 255, 255);
						ypos_osc1_0 = 255-(Sample_osc[ xpos][0]/koeff_h) - hpos;
						ypos_osc2_0 = 255-(Sample_osc[ xpos + 1][0]/koeff_h)- hpos;
						if(ypos_osc1_0 < 0) ypos_osc1_0 = 0;
						if(ypos_osc2_0 < 0) ypos_osc2_0 = 0;
						if(ypos_osc1_0 > 220) ypos_osc1_0  = 220;
						if(ypos_osc2_0 > 220) ypos_osc2_0 = 220;
						myGLCD.drawLine (xpos, ypos_osc1_0, xpos + 1, ypos_osc2_0);
						myGLCD.drawLine (xpos+1, ypos_osc1_0+1, xpos + 2, ypos_osc2_0+1);
					}

				if (Channel1)
					{
						myGLCD.setColor( VGA_YELLOW);
						ypos_osc1_1 = 255-(Sample_osc[ xpos][1]/koeff_h) - hpos;
						ypos_osc2_1 = 255-(Sample_osc[ xpos + 1][1]/koeff_h)- hpos;
						if(ypos_osc1_1 < 0) ypos_osc1_1 = 0;
						if(ypos_osc2_1 < 0) ypos_osc2_1 = 0;
						if(ypos_osc1_1 > 220) ypos_osc1_1  = 220;
						if(ypos_osc2_1 > 220) ypos_osc2_1 = 220;
						myGLCD.drawLine (xpos, ypos_osc1_1, xpos + 1, ypos_osc2_1);
						myGLCD.drawLine (xpos+1, ypos_osc1_1+1, xpos + 2, ypos_osc2_1+1);
					}

					OldSample_osc[xpos][0] = Sample_osc[xpos][0];
					OldSample_osc[xpos][1] = Sample_osc[xpos][1];
			}

		    ADC_end = false;
	}
koeff_h = 7.759*4;
mode1 = 2;
Trigger = 0;
StartSample = millis();
myGLCD.setFont( BigFont);
while (myTouch.dataAvailable()){}
}
//------------------------------------------------------------------------------
void test_ADC()
{
	ADC_end = false;

	ADCSRA = (1 << ADEN)   // ���������� ���
	            |(1 << ADATE)  // ����������� ����� ������ ���
	            |(1 << ADPS2)|(0 << ADPS1)|(0 << ADPS0); // ������������ �� 64 (������� ��� 125kHz)

    ADMUX  =(0<<ADLAR)
		   |(0<<REFS1)|(1<<REFS0)
		   |(0 << MUX4) |(0 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0) ;       // ���������� �������� �������� ���������� � ��������� �������������� ������������� �� ����� ������� 
    ADCSRB|= (1 << MUX5);
	i_osc=0;

	ADCSRA  |=(1 << ADSC)   // ADC Start Conversion
	         |(1 << ADIE);  // �������� ����������  ���������� ���������� �� ���

	while (ADC_end){};

	Serial.print("i_osc - ");
	Serial.println(i_osc);
	i_osc = 0;
	for(int i = 0;i<239;i++)
	{
      // Serial.println(analogRead(14));
	   Serial.println(Sample_osc[i][0]);
	}
	ADC_end = false;
	//adcStart() ;
	//delay(4000);

 //   ADCSRA &= ~(1 << ADIE); //���������
}

void trigger()
{
	int tr = 0;

	int Input = 0;
	//Serial.println("trig");
	for(tr = 0; tr < 1000; tr++)
	{
		Input = analogRead(14);
		if (Input< 100) break;
	}
	//Serial.println("min");
		for(tr = 0; tr < 1000; tr++)
	{
		Input = analogRead(14);
		if (Input>500) break;
	}
    Serial.println(tr);
	Serial.println(Input);

	/*
	 ADC_CHER = Channel_trig;

	for(int tr = 0; tr < 1000; tr++)
	{
		ADC_CR = ADC_START ; 	// ��������� ��������������
		while (!(ADC_ISR_DRDY));
		switch (t_in_mode) 
			{
				case 1:
					Input = ADC->ADC_CDR[6];
					break;
				case 2:
					Input = ADC->ADC_CDR[5];
					break;
				case 3:
					Input = ADC->ADC_CDR[4];
					break;
				default: 
					Input = ADC->ADC_CDR[7];
			}
		// if (Input<Trigger) break;
		 if (Input< 15) break;
	}
	//delayMicroseconds(2);

	for(int tr = 0; tr < 1000; tr++)
	{
		 ADC_CR = ADC_START ; 	// ��������� ��������������
		 while (!(ADC_ISR_DRDY));
		 switch (t_in_mode) 
			{
				case 1:
					Input = ADC->ADC_CDR[6];
					break;
				case 2:
					Input = ADC->ADC_CDR[5];
					break;
				case 3:
					Input = ADC->ADC_CDR[4];
					break;
				default: 
					Input = ADC->ADC_CDR[7];
			}
	
		if (Input>Trigger) break;
		
	}
	*/
}
void oscilloscope()  // �������� � �������� ������� �� ������� ��������
{
	uint32_t bgnBlock, endBlock;
	block_t block[BUFFER_BLOCK_COUNT];
	myGLCD.clrScr();
	myGLCD.setBackColor( 0, 0, 0);
	delay(500);
	myGLCD.clrScr();
	buttons_right();
	buttons_channel();
	myGLCD.setBackColor( 0, 0, 0);
	myGLCD.setFont( BigFont);
	myGLCD.setColor(VGA_LIME);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[15]))); 
	myGLCD.print(buffer,LEFT, 180);                                  // txt_info29
	int x_dTime;
	int xpos;
	int ypos1;
	int ypos2;

	int ypos_osc1_0;
	int ypos_osc1_1;
	int ypos_osc1_2;
	int ypos_osc1_3;

	int ypos_osc2_0;
	int ypos_osc2_1;
	int ypos_osc2_2;
	int ypos_osc2_3;

	ADC_end = false;

	ADCSRA = (1 << ADEN)         // ���������� ���
	            |(1 << ADATE)    // ����������� ����� ������ ���
	            |(1 << ADPS2)|(0 << ADPS1)|(0 << ADPS0); // ������������ �� 

    ADMUX  =(0<<ADLAR)
		   |(0<<REFS1)|(1<<REFS0)
		   |(0 << MUX4) |(0 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0) ;       // ���������� �������� �������� ���������� � ��������� �������������� ������������� �� ����� ������� 
    ADCSRB|= (1 << MUX5);

	i_osc=0;


	for( xpos = 0; xpos < 239;	xpos ++) // ������� ������ ������

		{
			OldSample_osc[xpos][0] = 0;
			OldSample_osc[xpos][1] = 0;
		}

	while(1) 
	{
		 DrawGrid();
		 if (myTouch.dataAvailable())
			{
				delay(10);
				myTouch.read();
				x_osc=myTouch.getX();
				y_osc=myTouch.getY();

				if ((x_osc>=2) && (x_osc<=240))  //  ������� ������
					{
						if ((y_osc>=1) && (y_osc<=160))  // Delay row
						{
							break;
						} 
					}

				myGLCD.setBackColor( 0, 0, 255);
				myGLCD.setFont( SmallFont);
				myGLCD.setColor (255, 255,255);
				myGLCD.drawRoundRect (250, 1, 318, 40);
				myGLCD.drawRoundRect (250, 45, 318, 85);
				myGLCD.drawRoundRect (250, 90, 318, 130);
				myGLCD.drawRoundRect (250, 135, 318, 175);

			if ((x_osc>=250) && (x_osc<=284))  // ������� ������
			  {
				  if ((y_osc>=1) && (y_osc<=40))  // ������  ������
				  {
					waitForIt(250, 1, 318, 40);
					mode -- ;
					if (mode < 0) mode = 0;   
					// Select delay times you can change values to suite your needs
					if (mode == 0) {dTime = 1;    x_dTime = 282;}
					if (mode == 1) {dTime = 10;   x_dTime = 278;}
					if (mode == 2) {dTime = 20;   x_dTime = 278;}
					if (mode == 3) {dTime = 50;   x_dTime = 278;}
					if (mode == 4) {dTime = 100;  x_dTime = 274;}
					if (mode == 5) {dTime = 200;  x_dTime = 274;}
					if (mode == 6) {dTime = 300;  x_dTime = 274;}
					if (mode == 7) {dTime = 500;  x_dTime = 274;}
					if (mode == 8) {dTime = 1000; x_dTime = 270;}
					if (mode == 9) {dTime = 5000; x_dTime = 270;}
					myGLCD.print("    ", 270, 22);
					myGLCD.printNumI(dTime, x_dTime, 22);
				  }

			 if ((y_osc>=45) && (y_osc<=85))  // ������ - �������
				 {
					waitForIt(250, 45, 318, 85);
					tmode --;
					if (tmode < 0)tmode = 0;
					if (tmode == 1){ Trigger = MinAnalog+10; myGLCD.print(" 0%  ", 268, 65);}
					if (tmode == 2){ Trigger = MaxAnalog/2;  myGLCD.print(" 50% ", 266, 65);}
					if (tmode == 3){ Trigger = MaxAnalog-10; myGLCD.print("100%", 270, 65);}
					if (tmode == 0)myGLCD.print(" Off ", 268, 65);

				 }
			 if ((y_osc>=90) && (y_osc<=130))  // ������ - ��������
				 {
					waitForIt(250, 90, 318, 130);
					mode1 -- ;
					myGLCD.setColor( 0, 0, 0);
					myGLCD.fillRoundRect (1, 1,239, 159);
					myGLCD.setColor (255, 255, 255);
					myGLCD.setBackColor( 0, 0, 255);
					myGLCD.setFont( SmallFont);
					if (mode1 < 0) mode1 = 0;   
					if (mode1 == 0){ koeff_h = 7.759*4; myGLCD.print(" 1  ", 275, 110);}
					if (mode1 == 1){ koeff_h = 3.879*4; myGLCD.print("0.5 ", 275, 110);}
					if (mode1 == 2){ koeff_h = 1.939*4; myGLCD.print("0.25", 275, 110);}
					if (mode1 == 3){ koeff_h = 0.969*4; myGLCD.print("0.1 ", 275, 110);}
				 }
			 if ((y_osc>=135) && (y_osc<=175))  // ��������� ����������
				 {

				 }
		   }
		
			if ((x_osc>=284) && (x_osc<=318))  // ������� ������
			  {
				  if ((y_osc>=1) && (y_osc<=40))  // ������  ������
				  {
					waitForIt(250, 1, 318, 40);
					mode ++ ;
					if (mode > 9) mode = 9;   
					if (mode == 0) {dTime = 1;    x_dTime = 282;}
					if (mode == 1) {dTime = 10;   x_dTime = 278;}
					if (mode == 2) {dTime = 20;   x_dTime = 278;}
					if (mode == 3) {dTime = 50;   x_dTime = 278;}
					if (mode == 4) {dTime = 100;  x_dTime = 274;}
					if (mode == 5) {dTime = 200;  x_dTime = 274;}
					if (mode == 6) {dTime = 300;  x_dTime = 274;}
					if (mode == 7) {dTime = 500;  x_dTime = 274;}
					if (mode == 8) {dTime = 1000; x_dTime = 270;}
					if (mode == 9) {dTime = 5000; x_dTime = 270;}
					myGLCD.print("    ", 270, 22);
					myGLCD.printNumI(dTime, x_dTime, 22);
				  }

			 if ((y_osc>=45) && (y_osc<=85))  // ������ - �������
				 {
					waitForIt(250, 45, 318, 85);
					tmode ++;
					if (tmode > 3)tmode = 3;
					if (tmode == 1){ Trigger = MinAnalog+10; myGLCD.print(" 0%  ", 268, 65);}
					if (tmode == 2){ Trigger = MaxAnalog/2;  myGLCD.print(" 50% ", 266, 65);}
					if (tmode == 3){ Trigger = MaxAnalog-10; myGLCD.print("100%", 270, 65);}
					if (tmode == 0)myGLCD.print(" Off ", 268, 65);
				 }
			 if ((y_osc>=90) && (y_osc<=130))  // ������ - ��������
				 {
					waitForIt(250, 90, 318, 130);
					mode1 ++ ;
					myGLCD.setColor( 0, 0, 0);
					myGLCD.fillRoundRect (1, 1,239, 159);
					myGLCD.setColor (255, 255, 255);
					myGLCD.setBackColor( 0, 0, 255);
					myGLCD.setFont( SmallFont);
					if (mode1 > 3) mode1 = 3;   
					if (mode1 == 0){ koeff_h = 7.759*4; myGLCD.print(" 1  ", 275, 110);}
					if (mode1 == 1){ koeff_h = 3.879*4; myGLCD.print("0.5 ", 275, 110);}
					if (mode1 == 2){ koeff_h = 1.939*4; myGLCD.print("0.25", 275, 110);}
					if (mode1 == 3){ koeff_h = 0.969*4; myGLCD.print("0.1 ", 275, 110);}
				 }
			 if ((y_osc>=135) && (y_osc<=175))  // ��������� ����������
				 {
					waitForIt(250, 135, 318, 175);
				 }

		   }

		if ((x_osc>=250) && (x_osc<=318))  

			{
			if ((y_osc>=200) && (y_osc<=239))  //   ������ ������  
				{
					waitForIt(250, 200, 318, 238);
					Channel_trig = 0;
					t_in_mode ++;
						if (t_in_mode > 3)
							{
								t_in_mode = 0;
							}
						switch_trig(t_in_mode);
						myGLCD.setBackColor( 0, 0, 255);
						myGLCD.setColor (255, 255,255);
						myGLCD.printNumI(t_in_mode, 282, 214);
				}
		  }

			 if ((y_osc>=205) && (y_osc<=239))  // ������ ������ ������������ ������
					{
						 touch_osc();
					}
		}
		 trig_min_max(t_in_mode);
		// if (tmode>0) trigger();
	
		    ADCSRA  |=(1 << ADSC)   // ADC Start Conversion
	        |(1 << ADIE);   // �������� ����������  ���������� ���������� �� ���

			while(ADC_end){}
		// �������� ���������� ������ � ���� ������
		StartSample = micros();


		EndSample = micros();
		DrawGrid();
 
		for( int xpos = 0; xpos < 239;	xpos ++)
			{
				//  ������� ���������� �����
				myGLCD.setColor( 0, 0, 0);
			
				if (Channel0 | osc_line_off0)
					{
						ypos_osc1_0 = 255-(OldSample_osc[ xpos + 1][0]/koeff_h) - hpos; 
						ypos_osc2_0 = 255-(OldSample_osc[ xpos + 2][0]/koeff_h) - hpos;
						if(ypos_osc1_0 < 0) ypos_osc1_0 = 0;
						if(ypos_osc2_0 < 0) ypos_osc2_0 = 0;
						if(ypos_osc1_0 > 220) ypos_osc1_0 = 220;
						if(ypos_osc2_0 > 220) ypos_osc2_0 = 220;
						myGLCD.drawLine (xpos + 1, ypos_osc1_0, xpos + 2, ypos_osc2_0);
						myGLCD.drawLine (xpos + 2, ypos_osc1_0+1, xpos + 3, ypos_osc2_0+1);

						if (xpos > 237 & Channel0 == false )
							{
								osc_line_off0 = false;
							}
					}
			
				if (Channel1|osc_line_off1)
					{
						ypos_osc1_1 = 255-(OldSample_osc[ xpos + 1][1]/koeff_h) - hpos; 
						ypos_osc2_1 = 255-(OldSample_osc[ xpos + 2][1]/koeff_h) - hpos;
						if(ypos_osc1_1 < 0) ypos_osc1_1 = 0;
						if(ypos_osc2_1 < 0) ypos_osc2_1 = 0;
						if(ypos_osc1_1 > 220) ypos_osc1_1 = 220;
						if(ypos_osc2_1 > 220) ypos_osc2_1 = 220;
						myGLCD.drawLine (xpos + 1, ypos_osc1_1, xpos + 2, ypos_osc2_1);
						myGLCD.drawLine (xpos + 2, ypos_osc1_1+1, xpos + 3, ypos_osc2_1+1);
						if (xpos > 237 & Channel1 == false )
							{
								osc_line_off1 = false;
							}
					}

					if (xpos == 0)
						{
							myGLCD.drawLine (xpos + 1, 1, xpos + 1, 220);
							myGLCD.drawLine (xpos + 2, 1, xpos + 2, 220);
						}
					
				if (Channel0)
					{
						myGLCD.setColor( 255, 255, 255);
						ypos_osc1_0 = 255-(Sample_osc[ xpos][0]/koeff_h) - hpos;
						ypos_osc2_0 = 255-(Sample_osc[ xpos + 1][0]/koeff_h)- hpos;
						if(ypos_osc1_0 < 0) ypos_osc1_0 = 0;
						if(ypos_osc2_0 < 0) ypos_osc2_0 = 0;
						if(ypos_osc1_0 > 220) ypos_osc1_0  = 220;
						if(ypos_osc2_0 > 220) ypos_osc2_0 = 220;
						myGLCD.drawLine (xpos, ypos_osc1_0, xpos + 1, ypos_osc2_0);
						myGLCD.drawLine (xpos+1, ypos_osc1_0+1, xpos + 2, ypos_osc2_0+1);
					}

				if (Channel1)
					{
						myGLCD.setColor( VGA_YELLOW);
						ypos_osc1_1 = 255-(Sample_osc[ xpos][1]/koeff_h) - hpos;
						ypos_osc2_1 = 255-(Sample_osc[ xpos + 1][1]/koeff_h)- hpos;
						if(ypos_osc1_1 < 0) ypos_osc1_1 = 0;
						if(ypos_osc2_1 < 0) ypos_osc2_1 = 0;
						if(ypos_osc1_1 > 220) ypos_osc1_1  = 220;
						if(ypos_osc2_1 > 220) ypos_osc2_1 = 220;
						myGLCD.drawLine (xpos, ypos_osc1_1, xpos + 1, ypos_osc2_1);
						myGLCD.drawLine (xpos+1, ypos_osc1_1+1, xpos + 2, ypos_osc2_1+1);
					}

					OldSample_osc[xpos][0] = Sample_osc[xpos][0];
					OldSample_osc[xpos][1] = Sample_osc[xpos][1];
				
			}

		  ADC_end = false;
		  
    //ADCSRA  |=(1 << ADSC)   // ADC Start Conversion
	   //     |(1 << ADIE);   // �������� ����������  ���������� ���������� �� ���
		//  ADCSRA |= (1 << ADIE);  // �������� ����������
	}
koeff_h = 7.759*4;
mode1 = 2;
Trigger = 0;
StartSample = millis();
myGLCD.setFont( BigFont);
while (myTouch.dataAvailable()){}
}
void buttons_right()  //  ������ ������  oscilloscope
{
	
	myGLCD.setColor(0, 0, 255);
	myGLCD.fillRoundRect (250, 1, 318, 40);
	myGLCD.fillRoundRect (250, 45, 318, 85);
	myGLCD.fillRoundRect (250, 90, 318, 130);
	myGLCD.fillRoundRect (250, 135, 318, 175);
	myGLCD.fillRoundRect (250, 200, 318, 239);

	myGLCD.setBackColor( 0, 0, 255);
	myGLCD.setFont( SmallFont);
	myGLCD.setColor (255, 255,255);
	myGLCD.print("Delay", 265, 6);
	myGLCD.print("-      +", 255, 22);
	myGLCD.printNumI(dTime, 282, 22);
	myGLCD.print("Trig.", 270, 50);
	myGLCD.print("-      +", 255, 65);
	if (tmode == 0)myGLCD.print(" Off ", 268, 65);
	if (tmode == 1)myGLCD.print(" 0%  ", 268, 65);
	if (tmode == 2)myGLCD.print(" 50% ", 266, 65);
	if (tmode == 3)myGLCD.print(" 100%", 270, 65);

	myGLCD.print("V/del.", 265, 95);
	myGLCD.print("-      +", 255, 110);
	if (mode1 == 0){ koeff_h = 7.759*4; myGLCD.print(" 1  ", 275, 110);}
	if (mode1 == 1){ koeff_h = 3.879*4; myGLCD.print("0.5 ", 275, 110);}
	if (mode1 == 2){ koeff_h = 1.939*4; myGLCD.print("0.25", 275, 110);}
	if (mode1 == 3){ koeff_h = 0.969*4; myGLCD.print("0.1 ", 275, 110);}

	myGLCD.setBackColor( 0, 0, 255);
	myGLCD.setColor (255, 255,255);
	myGLCD.print("Synchro", 255, 202);
	switch_trig(t_in_mode);
	myGLCD.printNumI(t_in_mode, 282, 212);
	
}
void buttons_right_time()
{
	
	myGLCD.setColor(0, 0, 255);
	myGLCD.fillRoundRect (250, 1, 318, 40);
	myGLCD.fillRoundRect (250, 45, 318, 85);
	myGLCD.fillRoundRect (250, 90, 318, 130);
	myGLCD.fillRoundRect (250, 135, 318, 175);
	myGLCD.fillRoundRect (250, 200, 318, 239);

	myGLCD.setBackColor( 0, 0, 255);
	myGLCD.setFont( SmallFont);
	myGLCD.setColor (255, 255,255);
	myGLCD.print("C\xA0""e\x99", 270, 140);                       //
	if (sled == true) myGLCD.print("  B\x9F\xA0 ", 257, 155);     //
	if (sled == false) myGLCD.print("O\xA4\x9F\xA0", 270, 155);
	myGLCD.print(txt_info30, 260, 205);
	if (repeat == true & count_repeat == 0)
		{
			myGLCD.print("  B\x9F\xA0 ", 257, 220);
		}
	if (repeat == true & count_repeat > 0)
		{
			if (repeat == true) myGLCD.print("       ", 257, 220);
			if (repeat == true) myGLCD.printNumI(count_repeat, 270, 220);
		}
	if (repeat == false) myGLCD.print("O\xA4\x9F\xA0", 270, 220);    // 

	if(Set_x == true)
	{
	   myGLCD.print("V Max", 265, 50);
	   myGLCD.print(" /x  ", 265, 65);
	}
	else
	{
	   myGLCD.print("V Max", 265, 50);
	   myGLCD.print("     ", 265, 65);
	}

	myGLCD.print("V/del.", 260, 95);
	myGLCD.print("-     +", 260, 110);
	if (mode1 == 0){ koeff_h = 7.759*4; myGLCD.print(" 1  ", 275, 110);}
	if (mode1 == 1){ koeff_h = 3.879*4; myGLCD.print("0.5 ", 275, 110);}
	if (mode1 == 2){ koeff_h = 1.939*4; myGLCD.print("0.25", 275, 110);}
	if (mode1 == 3){ koeff_h = 0.969*4; myGLCD.print("0.1 ", 275, 110);}
	scale_time();   // ����� �������� �����
	
}
void scale_time()
{
	
	myGLCD.setBackColor( 0, 0, 255);
	myGLCD.setFont( SmallFont);
	myGLCD.setColor (255, 255, 255);
	myGLCD.print("Delay", 264, 5);
	myGLCD.print("-      +", 254, 20);
	if (mode == 0)myGLCD.print("1min", 269, 20);
	if (mode == 1)myGLCD.print("6min", 269, 20);
	if (mode == 2)myGLCD.print("12min", 266, 20);
	if (mode == 3)myGLCD.print("18min", 266, 20);
	myGLCD.setBackColor(0, 0, 0);
	myGLCD.print("0",3, 163);         // � ������ �����
	if (mode == 0)                    // ��������� �����
		{
			myGLCD.print("10", 35, 163);
			myGLCD.print("20", 75, 163);
			myGLCD.print("30", 115, 163);
			myGLCD.print("40", 155, 163);
			myGLCD.print("50", 195, 163);
			myGLCD.print("60", 230, 163);
		}
	if (mode == 1)
		{
			myGLCD.print(" 1 ", 32, 163);
			myGLCD.print(" 2 ", 72, 163);
			myGLCD.print(" 3 ", 112, 163);
			myGLCD.print(" 4 ", 152, 163);
			myGLCD.print(" 5 ", 192, 163);
			myGLCD.print(" 6", 230, 163);
		}
	if (mode == 2)
		{
			myGLCD.print(" 2 ", 32, 163);
			myGLCD.print(" 4 ", 72, 163);
			myGLCD.print(" 6 ", 112, 163);
			myGLCD.print(" 8 ", 152, 163);
			myGLCD.print("10", 195, 163);
			myGLCD.print("12", 230, 163);
		}
	if (mode == 3)
		{
			myGLCD.print(" 3 ", 32, 163);
			myGLCD.print(" 6 ", 72, 163);
			myGLCD.print(" 9 ", 112, 163);
			myGLCD.print("12", 155, 163);
			myGLCD.print("15", 195, 163);
			myGLCD.print("18", 230, 163);
		}
		
}
void buttons_channel()  // ������ ������ ������������ ������
{
	
	myGLCD.setFont( SmallFont);

				if (Channel0)
					{
						myGLCD.setColor( 255, 255, 255);
						myGLCD.fillRoundRect (10, 200, 60, 205);
						myGLCD.setColor(VGA_LIME);
						myGLCD.setBackColor( VGA_LIME);
						myGLCD.fillRoundRect (10, 210, 60, 239);
						myGLCD.setColor(0, 0, 0);
						myGLCD.print("0", 32, 212);
						myGLCD.print("BXOD", 20, 226);
						osc_line_off0 = true;
					}
				else
					{
						myGLCD.setColor(0,0,0);
						myGLCD.setBackColor( 0,0,0);
						myGLCD.fillRoundRect (10, 200, 60, 205);   // ��������� ����� �����
						myGLCD.fillRoundRect (10, 210, 60, 239);
						myGLCD.setColor(255, 255, 255);
						myGLCD.print("0", 32, 212);
						myGLCD.print("BXOD", 20, 226);
					}

				if (Channel1)
					{
						myGLCD.setColor(VGA_YELLOW);
						myGLCD.fillRoundRect (70, 200, 120, 205);
						myGLCD.setColor(VGA_LIME);
						myGLCD.setBackColor( VGA_LIME);
						myGLCD.fillRoundRect (70, 210, 120, 239);
						myGLCD.setColor(0, 0, 0);
						myGLCD.print("1", 92, 212);
						myGLCD.print("BXOD", 80, 226);
						osc_line_off1 = true;
					}
				else
					{
						myGLCD.setColor(0,0,0);
						myGLCD.setBackColor( 0,0,0);
						myGLCD.fillRoundRect (70, 200, 120, 205);   // ��������� ����� �����
						myGLCD.fillRoundRect (70, 210, 120, 239);
						myGLCD.setColor(255, 255, 255);
						myGLCD.print("1", 92, 212);
						myGLCD.print("BXOD", 80, 226);
					}


	myGLCD.setColor(255, 255, 255);
	myGLCD.drawRoundRect (10, 210, 60, 239);
	myGLCD.drawRoundRect (70, 210, 120, 239);
	myGLCD.drawRoundRect (130, 210, 180, 239);
	myGLCD.drawRoundRect (190, 210, 240, 239);
	
}
void chench_Channel()
{
	//���������� ������ ����������� �������, ���������� ������� � ���� ��������� ���
		   Channel_x = 0;
		//   ADC_CHER = Channel_x;
		   count_pin = 0;
	 
		if (Channel0 )
			{
				Channel_x|=0x80;
				count_pin++;
			}
		if (Channel1 )
			{
				Channel_x|=0x40;
				count_pin++;
			}
		
	
		// ADC_CHER = Channel_x;
		// SAMPLES_PER_BLOCK = DATA_DIM16/count_pin;
}
void DrawGrid()
{
	
  myGLCD.setColor( 0, 200, 0);
  for(  dgvh = 0; dgvh < 5; dgvh ++)
  {
	  myGLCD.drawLine( dgvh * 40, 0, dgvh * 40, 160);
	  myGLCD.drawLine(  0, dgvh * 40, 240 ,dgvh * 40);
  }
	myGLCD.drawLine( 200, 0, 200, 160);
	myGLCD.drawLine( 240, 0, 240, 160);
	myGLCD.setColor(255, 255, 255);           // ����� ���������
	myGLCD.drawRoundRect (250, 1, 318, 40);
	myGLCD.drawRoundRect (250, 45, 318, 85);
	myGLCD.drawRoundRect (250, 90, 318, 130);
	myGLCD.drawRoundRect (250, 135, 318, 175);

	myGLCD.drawRoundRect (10, 210, 60, 239);
	myGLCD.drawRoundRect (70, 210, 120, 239);
	myGLCD.drawRoundRect (130, 210, 180, 239);
	myGLCD.drawRoundRect (190, 210, 240, 239);
	myGLCD.drawRoundRect (250, 200, 318, 239);
	myGLCD.setBackColor( 0, 0, 0);
	myGLCD.setFont( SmallFont);
	if (mode1 == 0)
		{				
			myGLCD.print("4", 241, 0);
			myGLCD.print("3", 241, 34);
			myGLCD.print("2", 241, 74);
			myGLCD.print("1", 241, 114);
			myGLCD.print("0", 241, 152);
		}
	if (mode1 == 1)
		{
			myGLCD.print("2", 241, 0);
			myGLCD.print("1,5", 226, 34);
			myGLCD.print("1", 241, 74);
			myGLCD.print("0,5", 226, 114);
			myGLCD.print("0", 241, 152);
		}

	if (mode1 == 2)
		{
			myGLCD.print("1", 241, 0);
			myGLCD.print("0,75", 218, 34);
			myGLCD.print("0,5", 226, 74);
			myGLCD.print("0,25", 218, 114);
			myGLCD.print("0", 241, 152);
		}
	if (mode1 == 3)
		{
			myGLCD.print("0,4", 226, 0);
			myGLCD.print("0,3", 226, 34);
			myGLCD.print("0,2", 226, 74);
			myGLCD.print("0,1", 226, 114);
			myGLCD.print("0", 241, 152);
		}
	if (!strob_start) 
		{
			//myGLCD.setColor(VGA_RED);
			//myGLCD.fillCircle(227,12,10);
		}
	else
		{
			//myGLCD.setColor(255,255,255);
			//myGLCD.drawCircle(227,12,10);
		}
	myGLCD.setColor(255,255,255);
	
}
void DrawGrid1()
{
	
 myGLCD.setColor( 0, 200, 0);
  for(  dgvh = 0; dgvh < 5; dgvh ++)
  {
	  myGLCD.drawLine( dgvh * 40, 0, dgvh * 40, 160);
	  myGLCD.drawLine(  0, dgvh * 40, 240 ,dgvh * 40);
  }
	myGLCD.drawLine( 200, 0, 200, 160);
	myGLCD.drawLine( 240, 0, 240, 160);
	//myGLCD.setColor(255, 255, 255);           // ����� ���������

	//if (!strob_start) 
	//	{
	//		myGLCD.setColor(VGA_RED);
	//		//myGLCD.fillCircle(227,12,10);
	//	}
	//else
	//	{
	//		myGLCD.setColor(255,255,255);
	//		//myGLCD.drawCircle(227,12,10);
	//	}
	myGLCD.setColor(255,255,255);
	
}
void touch_osc()  //  ������ ���� ������������
{
	
	delay(10);
	myTouch.read();
	x_osc=myTouch.getX();
	y_osc=myTouch.getY();
	myGLCD.setFont( SmallFont);

	if ((y_osc>=210) && (y_osc<=239))                         //   ������ ������
	  {
		if ((x_osc>=10) && (x_osc<=60))                       //  ���� 0
			{
				waitForIt(10, 210, 60, 239);

				Channel0 = !Channel0;

				if (Channel0)
					{
						myGLCD.setColor( 255, 255, 255);
						myGLCD.fillRoundRect (10, 200, 60, 205);
						myGLCD.setColor(VGA_LIME);
						myGLCD.setBackColor( VGA_LIME);
						myGLCD.fillRoundRect (10, 210, 60, 239);
						myGLCD.setColor(0, 0, 0);
						myGLCD.print("0", 32, 212);
						myGLCD.print("BXOD", 20, 226);
						osc_line_off0 = true;
					}
				else
					{
						myGLCD.setColor(0,0,0);
						myGLCD.setBackColor( 0,0,0);
						myGLCD.fillRoundRect (10, 200, 60, 205);
						myGLCD.fillRoundRect (10, 210, 60, 239);
						myGLCD.setColor(255, 255, 255);
						myGLCD.drawRoundRect (10, 210, 60, 239);
						myGLCD.print("0", 32, 212);
						myGLCD.print("BXOD", 20, 226);
					}

				chench_Channel();
				MinAnalog0 = 1023;
				MaxAnalog0 = 0;
			}

		else if ((x_osc>=70) && (x_osc<=120))                    //  ���� 1
			{

				waitForIt(70, 210, 120, 239);

					Channel1 = !Channel1;

				if (Channel1)
					{
						myGLCD.setColor(VGA_YELLOW);
						myGLCD.fillRoundRect (70, 200, 120, 205);
						myGLCD.setColor(VGA_LIME);
						myGLCD.setBackColor( VGA_LIME);
						myGLCD.fillRoundRect (70, 210, 120, 239);
						myGLCD.setColor(0, 0, 0);
						myGLCD.print("1", 92, 212);
						myGLCD.print("BXOD", 80, 226);
						osc_line_off1 = true;
					}
				else
					{
						myGLCD.setColor(0,0,0);
						myGLCD.setBackColor( 0,0,0);
						myGLCD.fillRoundRect (70, 200, 120, 205);
						myGLCD.fillRoundRect (70, 210, 120, 239);
						myGLCD.setColor(255, 255, 255);
						myGLCD.drawRoundRect (70, 210, 120, 239);
						myGLCD.print("1", 92, 212);
						myGLCD.print("BXOD", 80, 226);
					}

				chench_Channel();
				MinAnalog1 = 4095;
				MaxAnalog1 = 0;
			}
		else if ((x_osc>=130) && (x_osc<=180))                    //  ���� 2
			{
				waitForIt(130, 210, 180, 239);


			}
		else if ((x_osc>=190) && (x_osc<=240))                     //  ���� 3
			{
				waitForIt(190, 210, 240, 239);

				
			}
	}
	
}
void switch_trig(int trig_x)
{
	
	switch (trig_x) 
					{
						case 1:
						 if (Channel1)
							{
								Channel_trig = 0x40;
								myGLCD.print(" ON ", 270, 226);
								MinAnalog = MinAnalog1 ;
								MaxAnalog = MaxAnalog1 ;
							}
						else
							{
								myGLCD.print(" OFF", 270, 226);
							}
						  break;

						default: 

						 if (Channel0)
							{
								Channel_trig = 0x80;
								myGLCD.print(" ON ", 270, 226);
								MinAnalog = MinAnalog0 ;
								MaxAnalog = MaxAnalog0 ;
							}
						else
							{
								myGLCD.print(" OFF", 270, 226);
							}
					}

}
void trig_min_max(int trig_x)
{
	switch (trig_x) 
					{
						case 1:
						 if (Channel1)
							{
								MinAnalog = MinAnalog1 ;
								MaxAnalog = MaxAnalog1 ;
							}
						  break;
						case 0:

						if (Channel0)
							{
								MinAnalog = MinAnalog0 ;
								MaxAnalog = MaxAnalog0 ;
							}
						  break;

						default: 

						 if (Channel0)
							{
								MinAnalog = MinAnalog0 ;
								MaxAnalog = MaxAnalog0 ;
							}

					}

}
//--------------------- ����� ��������� ������������ -------------

//----------------------------------------------------------------
void set_adr_EEPROM()
{
 adr_memN1_1 = 100;                       // ��������� ����� ������ ������� ������������ ��������� �������� �1�, �1�
 adr_memN1_2 = adr_memN1_1+sizeof(connektN1_default)+1;                       // ��������� ����� ������ ������� ������������ ��������� �������� �2�, �2�
 adr_memN1_3 = adr_memN1_2+sizeof(connektN2_default)+1;                       // ��������� ����� ������ ������� ������������ ��������� �������� �3�, �3�
 adr_memN1_4 = adr_memN1_3+sizeof(connektN3_default)+1;                       // ��������� ����� ������ ������� ������������ ��������� �������� �4�, �4�
 //++++++++++++++++++ ������� � 2 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 //adr_memN2_1 = adr_memN1_1+sizeof(connektN1_default)+1;                       // ��������� ����� ������ ������� ������������ ��������� �������� �1�, �1�
 //adr_memN2_2 = adr_memN1_1+sizeof(connektN1_default)+1;                       // ��������� ����� ������ ������� ������������ ��������� �������� �2�, �2�
 //adr_memN2_3 = adr_memN1_1+sizeof(connektN1_default)+1;                       // ��������� ����� ������ ������� ������������ ��������� �������� �3�, �3�
 //adr_memN2_4 = adr_memN1_1+sizeof(connektN1_default)+1;                       // ��������� ����� ������ ������� ������������ ��������� �������� �4�, �4�
 //
 }
void setup_pin()
{
	pinMode(led_Red, OUTPUT);                             //  
	pinMode(led_Green, OUTPUT);                           //
	digitalWrite(led_Red, HIGH);                          // 
	digitalWrite(led_Green, LOW);                         // 
	pinMode(48, OUTPUT);                                  // ��������� ������ ��������� ����� U11_1
	pinMode(49, OUTPUT);                                  // ��������� ������ ��������� ����� U11_2
	digitalWrite(48,HIGH);                                // ��������� ������ U11 ����� �1
	digitalWrite(49,HIGH);                                // ��������� ������ U11 ����� �2
	pinMode(46, INPUT);                                   // ����� ������������ ����� �
	pinMode(47, INPUT);                                   // ����� ������������ ����� �
	pinMode(led_disp, INPUT);                             // 
	pinMode(led_instr, INPUT);                            //  
}
void setup_mcp()
{
	// ��������� ����������� ������
 
  mcp_Out1.begin(1);                               //  ����� (4) �������  ����������� ������
  mcp_Out1.pinMode(0, OUTPUT);                     //  1A1
  mcp_Out1.pinMode(1, OUTPUT);                     //  1B1  
  mcp_Out1.pinMode(2, OUTPUT);                     //  1C1
  mcp_Out1.pinMode(3, OUTPUT);                     //  1D1  
  mcp_Out1.pinMode(4, OUTPUT);                     //  1A2
  mcp_Out1.pinMode(5, OUTPUT);                     //  1B2
  mcp_Out1.pinMode(6, OUTPUT);                     //  1C2
  mcp_Out1.pinMode(7, OUTPUT);                     //  1D2
  
  mcp_Out1.pinMode(8, OUTPUT);                     //  1E1   U13
  mcp_Out1.pinMode(9, OUTPUT);                     //  1E2   U17
  mcp_Out1.pinMode(10, OUTPUT);                    //  1E3   U23
  mcp_Out1.pinMode(11, OUTPUT);                    //  1E4   U14
  mcp_Out1.pinMode(12, OUTPUT);                    //  1E5   U19
  mcp_Out1.pinMode(13, OUTPUT);                    //  1E6   U21 
  mcp_Out1.pinMode(14, OUTPUT);                    //  1E7   ��������  
  mcp_Out1.pinMode(15, OUTPUT);                    //  1E8   ��������
	
  mcp_Out2.begin(2);                               //  
  mcp_Out2.pinMode(0, OUTPUT);                     //  2A1  
  mcp_Out2.pinMode(1, OUTPUT);                     //  2B1  
  mcp_Out2.pinMode(2, OUTPUT);                     //  2C1
  mcp_Out2.pinMode(3, OUTPUT);                     //  2D1  
  mcp_Out2.pinMode(4, OUTPUT);                     //  2A2
  mcp_Out2.pinMode(5, OUTPUT);                     //  2B2
  mcp_Out2.pinMode(6, OUTPUT);                     //  2C2
  mcp_Out2.pinMode(7, OUTPUT);                     //  2D2
  
  mcp_Out2.pinMode(8, OUTPUT);                     //  2E1   U15
  mcp_Out2.pinMode(9, OUTPUT);                     //  2E2   U18
  mcp_Out2.pinMode(10, OUTPUT);                    //  2E3   U22
  mcp_Out2.pinMode(11, OUTPUT);                    //  2E4   U16
  mcp_Out2.pinMode(12, OUTPUT);                    //  2E5   U20     
  mcp_Out2.pinMode(13, OUTPUT);                    //  2E6   U24       
  mcp_Out2.pinMode(14, OUTPUT);                    //  2E7   ���� �1, �2
  mcp_Out2.pinMode(15, OUTPUT);                    //  2E8   ��������
  for(int i=0;i<16;i++)
  {
	  mcp_Out1.digitalWrite(i, HIGH); 
	  mcp_Out2.digitalWrite(i, HIGH); 
  }
   mcp_Out2.digitalWrite(14, LOW);                 // ��������� ����
}
void setup_sound_port()
{
	//pinMode(ledPin13, OUTPUT);   
	//pinMode(ledPin12, OUTPUT);  
	//digitalWrite(ledPin12, LOW);                   // 
	//digitalWrite(ledPin13, LOW);                   // 

	pinMode(kn1Nano, OUTPUT);  
	pinMode(kn2Nano, OUTPUT);  
	pinMode(kn3Nano, OUTPUT);  
	pinMode(kn4Nano, OUTPUT);  
	pinMode(kn5Nano, OUTPUT);  
	pinMode(kn6Nano, OUTPUT);  

	digitalWrite(kn1Nano, HIGH);                        // 
	digitalWrite(kn2Nano, HIGH);                        //
	digitalWrite(kn3Nano, HIGH);                        //
	digitalWrite(kn4Nano, LOW);                         // 
	digitalWrite(kn5Nano, HIGH);                        // 
	digitalWrite(kn6Nano, HIGH);                        //
}
void setup_regModbus()
{
    regBank.setId(1);    // Slave ID 1

  	regBank.add(1);      //  
	regBank.add(2);      //  
	regBank.add(3);      //  
	regBank.add(4);      //  
	regBank.add(5);      //  
	regBank.add(6);      //  
	regBank.add(7);      //  
	regBank.add(8);      //  

	regBank.add(10001);  //  
	regBank.add(10002);  //  
	regBank.add(10003);  //  
	regBank.add(10004);  //  
	regBank.add(10005);  //  
	regBank.add(10006);  //  
	regBank.add(10007);  //  
	regBank.add(10008);  //  

	regBank.add(30001);  //  
	regBank.add(30002);  //  
	regBank.add(30003);  //  
	regBank.add(30004);  //  
	regBank.add(30005);  //  
	regBank.add(30006);  //  
	regBank.add(30007);  //  
	regBank.add(30008);  //  

	regBank.add(40001);  //  ����� �������� ������� �� ���������� 
	regBank.add(40002);  //  ����� �������� ���� ������
	regBank.add(40003);  //  ����� �������� �������� ������� ���������� � 1
	regBank.add(40004);  //  ����� �������� �������� ������� ���������� � 2
	regBank.add(40005);  //  ����� ����� ��������� ��� �������� � �� ������.
	regBank.add(40006);  //  ����� ����� ������ ��� �������� � �� ������.
	regBank.add(40007);  //  ����� ����� ����� ������
	regBank.add(40008);  //  ����� ����� ������ �� ���������
	regBank.add(40009);  //  

	regBank.add(40010);  //  �������� ���������� �������� ��� �������� �������
	regBank.add(40011);   
	regBank.add(40012);   
	regBank.add(40013);   
	regBank.add(40014);    
	regBank.add(40015);   
	regBank.add(40016);    
	regBank.add(40017);   
	regBank.add(40018);     
	regBank.add(40019);   

	regBank.add(40020);                            
	regBank.add(40021);   
	regBank.add(40022);   
	regBank.add(40023);   
	regBank.add(40024);    
	regBank.add(40025);   
	regBank.add(40026);    
	regBank.add(40027);   
	regBank.add(40028);     
	regBank.add(40029); 

	regBank.add(40030);                            
	regBank.add(40031);   
	regBank.add(40032);   
	regBank.add(40033);   
	regBank.add(40034);    
	regBank.add(40035);   
	regBank.add(40036);    
	regBank.add(40037);   
	regBank.add(40038);     
	regBank.add(40039); 

	regBank.add(40040);                            
	regBank.add(40041);   
	regBank.add(40042);   
	regBank.add(40043);   
	regBank.add(40044);    
	regBank.add(40045);   
	regBank.add(40046);    
	regBank.add(40047);   
	regBank.add(40048);     
	regBank.add(40049); 
						 // ������� ����� 
	regBank.add(40050);  // ����� ���� ������ ����� �����������
	regBank.add(40051);  // ����� ����� ������ ����� �����������
	regBank.add(40052);  // ����� ��� ������ ����� �����������
	regBank.add(40053);  // ����� ��� ������ ����� �����������
	regBank.add(40054);  // ����� ������ ������ ����� �����������
	regBank.add(40055);  // ����� ������� ������ ����� �����������
						 // ��������� ������� � �����������
	regBank.add(40056);  // ����� ����
	regBank.add(40057);  // ����� �����
	regBank.add(40058);  // ����� ���
	regBank.add(40059);  // ����� ���
	regBank.add(40060);  // ����� ������
	regBank.add(40061);  // 
	regBank.add(40062);  // 
	regBank.add(40063);  // 
	slave._device = &regBank;  
}

void setup()
{
	myGLCD.InitLCD();
	myGLCD.clrScr();
	myGLCD.setFont(BigFont);
	myTouch.InitTouch();
	delay(1000);
	//myTouch.setPrecision(PREC_MEDIUM);
	myTouch.setPrecision(PREC_HI);
	//myTouch.setPrecision(PREC_EXTREME);
	myButtons.setTextFont(BigFont);
	myButtons.setSymbolFont(Dingbats1_XL);
	Serial.begin(115200);                                    // ����������� � USB ��
	Serial1.begin(115200);                                 // ����������� � 
	slave.setSerial(3,57600);                              // ����������� � ��������� MODBUS ���������� Serial3 
	Serial2.begin(115200);                                 // ����������� � 
    setup_pin();
	Wire.begin();
	if (!RTC.begin())                                      // ��������� ����� 
		{
			Serial.println("RTC failed");
			while(1);
		};
	//DateTime set_time = DateTime(16, 3, 15, 10, 19, 0);  // ������� ������ � ������� � ������ "set_time" ���, �����, �����, �����...
	//RTC.adjust(set_time);                                // �������� ����
	Serial.println(" ");
	Serial.println(" ***** Start system  *****");
	Serial.println(" ");
	//set_time();
	serial_print_date();
	setup_sound_port();
	setup_mcp();                                          // ��������� ����� ����������  
	setup_resistor();                                     // ��������� ��������� ���������
	MsTimer2::set(300, flash_time);                       // 300ms ������ ������� ���������
	resistor(1, 200);                                     // ���������� ������� �������
	resistor(2, 200);                                     // ���������� ������� �������
	setup_regModbus();
	
	//myGLCD.setFont(BigFont);
	//myTouch.InitTouch();
	////myTouch.setPrecision(PREC_MEDIUM);
	//myTouch.setPrecision(PREC_HI);
	////myTouch.setPrecision(PREC_EXTREME);
	//myButtons.setTextFont(BigFont);
	//myButtons.setSymbolFont(Dingbats1_XL);
	// ++++++++++++++++++ ��������� ��� +++++++++++++++++++++++++++++++++++++++++++++++++++
	// set up the ADC
	 


  // Read the first sample pin to init the ADC.
 // analogRead(PIN_LIST[0]);

   

  
  Serial.print(F("FreeRam: "));
  Serial.println(FreeRam());

//	TCCR1B = TCCR1B & 0b11111000 | 1; 
 //
 //  

 //ADCSRA=(1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);     //ADC Control and Status Register A 

//ADCSRA=(1<<ADEN)|(1<<ADIE)|(1<<ADSC)|(1<<ADATE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);     //ADC Control and Status Register A 
//    ADMUX=(1<<ADLAR)|(1<<REFS1)|(1<<REFS0);  // ���������� �������� �������� ���������� � ��������� �������������� ������������� �� ����� ������� 

//16 MHz / 2 = 8 MHz
//16 MHz / 4 = 4 MHz
//16 MHz / 8 = 2 MHz
//16 MHz / 16 = 1 MHz
//16 MHz / 32 = 500 kHz
//16 MHz / 64 = 250 kHz
//16 MHz / 128 = 125 kHz

//TIMSK |= (1 << TOIE2); // ���������� ���������� �� �������2
//TCCR2 |= (1 << CS21);  // ������������ �� 8 
//// ��������� ���    
//ADCSRA |= (1 << ADEN) // ���������� ���
//        |(1 << ADSC) // ������ ��������������
//        |(1 << ADATE) // ����������� ����� ������ ���
//        |(1 << ADPS2)|(1 << ADPS1) // ������������ �� 64 (������� ��� 125kHz)
//        |(1 << ADIE); // ���������� ���������� �� ���
//ADMUX |= (1 << REFS1)|(1 << REFS0); // ���������� ��� 2,56V, ���� ADC0
//     
//
//
//	ADCSRA &= ~PS_128;  // remove bits set by Arduino library
//
//	// you can choose a prescaler from below.
//	// PS_16, PS_32, PS_64 or PS_128
//	ADCSRA |= PS_128;    // set our own prescaler  ���������� ���� ����������� ������������
//
//	sei(); // ��������� ��������� ����������

	//draw_Glav_Menu();

	wait_time_Old =  millis();
	digitalWrite(ledPin13, HIGH);                          // 
	digitalWrite(ledPin12, LOW);                           // 
	set_adr_EEPROM();
	Serial.println(" ");                                   //
	Serial.println("System initialization OK!.");          // ���������� � ���������� ���������
	//set_komm_mcp(2,44,2);
}

void loop()
{

	draw_Glav_Menu();
	swichMenu();
	//test_all_pin();
	// test_cabel_N1();
	//delay(100);
}
