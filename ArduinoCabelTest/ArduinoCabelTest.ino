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

MCP23017 mcp_Out1;                                 // ���������� ������ ���������� MCP23017  4 A - Out, B - Out
MCP23017 mcp_Out2;                                 // ���������� ������ ���������� MCP23017  6 A - Out, B - Out

// Define various ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

//+++++++++++++++++++ MODBUS ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

modbusDevice regBank;
modbusSlave slave;

//+++++++++++++++++++++++ ��������� ������������ ��������� +++++++++++++++++++++++++++++++++++++
  byte resistance        = 0x00;                     // ������������� 0x00..0xFF - 0��..100���

//+++++++++++++++++++++++++++++ ������� ������ +++++++++++++++++++++++++++++++++++++++
int deviceaddress        = 80;                     // ����� ���������� ������
unsigned int eeaddress   =  0;                     // ����� ������ ������
byte hi;                                           // ������� ���� ��� �������������� �����
byte low;                                          // ������� ���� ��� �������������� �����

//********************* ��������� �������� ***********************************

// Standard Arduino Mega/Due shield            : <display model>,38,39,40,41
//UTFT        myGLCD(ITDB32S,38,39,40,41);     // ������� 3.2"
UTFT        myGLCD(ITDB24E_8,38,39,40,41);     // ������� 2.4" !! ��������! �������� ��������� UTouchCD.h


// Standard Arduino Mega/Due shield            : 6,5,4,3,2
UTouch        myTouch(6,5,4,3,2);
// Finally we set up UTFT_Buttons :)
UTFT_Buttons  myButtons(&myGLCD, &myTouch);

boolean default_colors = true;
uint8_t menu_redraw_required = 0;

// Declare which fonts we will be using
extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t Dingbats1_XL[];
extern uint8_t SmallSymbolFont[];

//+++++++++++++++++++++++++++ ��������� ����� +++++++++++++++++++++++++++++++
uint8_t second = 0;                      //Initialization time
uint8_t minute = 10;
uint8_t hour   = 10;
uint8_t dow    = 2;
uint8_t day    = 15;
uint8_t month  = 3;
uint16_t year  = 16;
RTC_DS1307 RTC;                         // define the Real Time Clock object

int clockCenterX               = 119;
int clockCenterY               = 119;
int oldsec                     = 0;
const char* str[]              = {"MON","TUE","WED","THU","FRI","SAT","SUN"};
const char* str1[]             = {"Monday","Tuesday","Wednesday","Thursday","Friday","Saturday","Sunday"};
const char* str_mon[]          = {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
unsigned long wait_time        = 0;         // ����� ������� �������
unsigned long wait_time_Old    = 0;         // ����� ������� �������
int time_minute                = 5;         // ����� ������� �������
//------------------------------------------------------------------------------

const unsigned int adr_control_command    PROGMEM       = 40001; // ����� �������� ������� �� ���������� 
const unsigned int adr_reg_count_err      PROGMEM       = 40002; // ����� �������� ���� ������
//-------------------------------------------------------------------------------------------------------

//++++++++++++++++++++++++++++ ���������� ��� �������� ���������� +++++++++++++++++++++++++++++
int x, y, z;
char stCurrent[20]    ="";         // ���������� �������� ��������� ������ 
int stCurrentLen      =0;          // ���������� �������� ����� ��������� ������ 
int stCurrentLen1     =0;          // ���������� ���������� �������� ����� ��������� ������  
char stLast[20]       ="";         // ������ � ��������� ������ ������.
int ret               = 0;         // ������� ���������� ��������
//-------------------------------------------------------------------------------------------------


//++++++++++++++++++++++++++ ��������� ������������  +++++++++++++++++++++++++++++++++++++++++++++++++++

int dgvh;
int OldSample_osc[254][2];
int x_osc,y_osc;
int mode = 0;
int dTime = 1;
int tmode = 1;
int mode1 = 0;             //������������ ����������������
int Trigger = 0;
int MinAnalog = 500;
int MinAnalog0 = 500;
int MinAnalog1 = 500;
int MaxAnalog = 0;
int MaxAnalog0 = 0;
int MaxAnalog1 = 0;
float koeff_h = 7.759*4;
int Sample_osc[254][2];
float StartSample = 0; 
float EndSample = 0;
int t_in_mode = 0;
bool strob_start = true;
// Analog pin number list for a sample. 
int Channel_x = 0;
int Channel_trig = 0;
bool Channel0 = true;
bool Channel1 = false;
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
 char  txt_menu2_1[]            = "menu2_1";                                                                // 
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
 char  txt_pass_ok[]            = "Tec\xA4 Pass!";                                                          // ���� Pass!
 char  txt_pass_no[]            = "Tec\xA4 NO!";                                                            // ���� NO!
 char  txt_info1[]              = "Tec\xA4 ""\x9F""a\x96""e\xA0""e\x9E";                                    // ���� �������
 char  txt_info2[]              = "Tec\xA4 \x96\xA0o\x9F""a \x98""ap\xA2\x9D\xA4yp";                        // ���� ����� ��������
 char  txt_info3[]              = "Hac\xA4po\x9E\x9F""a c\x9D""c\xA4""e\xA1\xAB";                           // ��������� �������
 char  txt_info4[]              = "\x81""e\xA2""epa\xA4op c\x9D\x98\xA2""a\xA0o\x97";                       // ��������� ��������
 char  txt_info5[]              = "Oc\xA6\x9D\xA0\xA0o\x98pa\xA5";                                          // �����������
 char  txt_botton_clear[]       = "C\x96poc";                                                               // �����
 char  txt_botton_otmena[]      = "O""\xA4\xA1""e""\xA2""a";                                                // ������
 char  txt_system_clear1[]      = "B\xA2\x9D\xA1""a\xA2\x9D""e!";                                           // �������� !  
 char  txt_system_clear2[]      = "Bc\xAF \xA1\xA2\xA5op\xA1""a""\xA6\xA1\xAF \x96y\x99""e\xA4";            // ��� ���������� ����� 
 char  txt_system_clear3[]      = "\x8A\x82""A""\x88""EHA!";                                                // ������� 
 char  txt9[6]                  = "B\x97o\x99";                                                             // ����
 char  txt10[8]                 = "O""\xA4\xA1""e""\xA2""a";                                                // "������"
 char  txt_time_wait[]          = "\xA1\x9D\xA2"".""\x97""pe""\xA1\xAF"" ""\xA3""poc""\xA4""o""\xAF";       //  ���. ����� �������
 char  txt_osc_menu1[]          = "Oc\xA6\x9D\xA0\xA0o\x98pa\xA5";                                          //
 char  txt_osc_menu2[]          = "";                                    //
 char  txt_osc_menu3[]          = "";                                                  //
 char  txt_osc_menu4[]          = "";           
 char  txt_info29[]             = "Stop->PUSH Disp"; 
 char  txt_info30[]             = "\x89o\x97\xA4op."; 
 char  txt_test_all[]           = "Tec""\xA4"" ""\x97""cex pa""\x9C\xAA""e""\xA1""o""\x97";                 // ���� ���� ��������
 char  txt_test_all_exit1[]     = "\x82\xA0\xAF"" ""\x97\xAB""xo""\x99""a";                                 // ��� ������
 char  txt_test_all_exit2[]     = "\xA3""p""\x9D\x9F""oc""\xA2\x9D""c""\xAC"" ""\x9F"" ""\xAD\x9F""pa""\xA2""y";  // ���������� � ������
 char  txt_test_end[]           = "\x85""a""\x97""ep""\xA8\x9D\xA4\xAC";                                    // ���������
 char  txt_test_repeat[]        = "\x89""o""\x97\xA4""op""\x9D\xA4\xAC";                                    //���������
 char  txt_error_connect1[]     = "O""\x8E\x86\x80""KA";                                                    //������
 char  txt_error_connect2[]     = "\xA3""o""\x99\x9F\xA0\xAE\xA7""e""\xA2\x9D\xAF"" ""\x9F""a""\x96""e""\xA0\xAF";  //����������� ������
 char  txt__connect1[]          = "O""\x96\xA2""apy""\x9B""e""\xA2"" ""\x9F""a""\x96""e""\xA0\xAC"" N1";    //��������� ������ N1
 char  txt__connect2[]          = "O""\x96\xA2""apy""\x9B""e""\xA2"" ""\x9F""a""\x96""e""\xA0\xAC"" N2";    //��������� ������ N2
 char  txt__connect3[]          = "O""\x96\xA2""apy""\x9B""e""\xA2"" ""\x9F""a""\x96""e""\xA0\xAC"" N3";    //��������� ������ N3
 char  txt__connect4[]          = "O""\x96\xA2""apy""\x9B""e""\xA2"" ""\x9F""a""\x96""e""\xA0\xAC"" N4";    //��������� ������ N4


 byte   temp_buffer[40] ;                                                                                                // ����� �������� ��������� ����������
 
 const byte connektN1_default[]    PROGMEM  = { 20,
      1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,                                                                // ������ �
	  1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,
	  0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	  0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0                                                                  // ������ B
 }; // 20 x 4 �����
 const byte connektN2_default[]    PROGMEM  = { 26,
       1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,                                             // ������ �
	   1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,                                             // ������ B
	   0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	   0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
 }; // 26 x 2 �����
 const byte connektN3_default[]    PROGMEM  = { 37,
       1, 2, 3, 4, 5, 6, 7, 8, 9, 10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,   // ������ �
	   19,18,17,16,15,14,13,12,11,10,9, 8, 7, 6, 5, 4, 3, 2, 1, 37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,   // ������ B
 	   0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	   0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
 }; // 37 x 2 �����
 const byte connektN4_default[]    PROGMEM  = { 32,
       1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,                           // ������ �
	   1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,                           // ������ B
 	   0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	   0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
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
void txt_pass_no_all()
{
		myGLCD.clrScr();
		myGLCD.setColor(255, 255, 255);
		myGLCD.setBackColor(0, 0, 0);
		myGLCD.print(txt_pass_no, RIGHT, 208);
		delay (1000);
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
  myGLCD.print(txt_botton_clear, 25, 147);     //"�����"


  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect (130, 130, 240, 180);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect (130, 130, 240, 180);
  myGLCD.print("B\x97o\x99", 155, 147);       // "����"
  

  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect (250, 130, 300, 180);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect (250, 130, 300, 180);
  myGLCD.print("B""\xAB""x", 252, 147);       // ���
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
					      myGLCD.print(txt_info1, CENTER, 0);
					      break;
					 case 2:
                          myGLCD.print(txt_info2, CENTER, 0);
					      break;
					 case 3:
					      myGLCD.print(txt_info3, CENTER, 0);
					      break;
					 case 4:
					      myGLCD.print(txt_info4, CENTER, 0);
					      break;
					 case 5:
					      myGLCD.print(txt_info5, CENTER, 0);
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
							  myGLCD.print(txt_info1, CENTER, 0);   // "���� �������"
		
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
							  myGLCD.print(txt_info2, CENTER, 0);     // ���� ����� ��������
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
							  myGLCD.print(txt_info3, CENTER, 0);      // ��������� �������
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
							  myGLCD.print(txt_info4, CENTER, 0);     // ��������� ��������
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
							  myGLCD.print(txt_info5, CENTER, 0);     // �����������
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
							// ���� ����� ��������
	        				myGLCD.clrScr();
							myButtons.drawButtons();
							print_up();
				      }

				   if (pressed_button==but2 && m2 == 2)
				  	  {
 
							myGLCD.clrScr();
							myButtons.drawButtons();
							print_up();
					  }
	  
				   if (pressed_button==but3 && m2 == 2)
					  {
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
						//	myGLCD.print(txt_pass_ok, RIGHT, 208);
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
							myGLCD.print(txt_time_wait, 35, 208);      //
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
							oscilloscope();
							myGLCD.clrScr();
							myButtons.drawButtons();
							print_up();
					  }
				   if (pressed_button==but2 && m2 == 5)
					  {
							myGLCD.clrScr();   // �������� �����
							//  myGLCD.print(txt_pass_ok, RIGHT, 208); 
							delay (500);

							myButtons.drawButtons();
							print_up();
					  }

				   if (pressed_button==but3 && m2 == 5) // ���� ������ ������������
					  {
							myGLCD.clrScr();   // �������� �����
							//	  myGLCD.print(txt_pass_ok, RIGHT, 208); 

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
			myGLCD.print(txt_info1, CENTER, 0);
			break;
		case 2:
            myGLCD.print(txt_info2, CENTER, 0);
			break;
		case 3:
			myGLCD.print(txt_info3, CENTER, 0);
			break;
		case 4:
			myGLCD.print(txt_info4, CENTER, 0);
			break;
        case 5:
			myGLCD.print(txt_info5, CENTER, 0);
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
				 for (int i = 0; i < (_step_mem * 2)+1;i++)                    
					{
					  _u_konnekt = pgm_read_byte_near(connektN1_default+i);
					  i2c_eeprom_write_byte(deviceaddress,adr_memN+i, _u_konnekt); 
					}
				 break;
			case 2:
				 adr_memN   = adr_memN1_2;                                        // ����� ����� EEPROM � 2 
				 _step_mem  = (pgm_read_byte_near(connektN2_default));            // ����� ����� � �������
				 for (int i = 0; i < (_step_mem * 2)+1;i++)                    
					{
					  _u_konnekt = pgm_read_byte_near(connektN2_default+i);
					  i2c_eeprom_write_byte(deviceaddress,adr_memN+i, _u_konnekt); 
					}
				 break;
			case 3:
				 adr_memN   = adr_memN1_3;                                       // ����� ����� EEPROM � 3
				 _step_mem  = (pgm_read_byte_near(connektN3_default));           // ����� ����� � �������
				 for (int i = 0; i < (_step_mem * 2)+1;i++)                    
					{
					  _u_konnekt = pgm_read_byte_near(connektN3_default+i);
					  i2c_eeprom_write_byte(deviceaddress,adr_memN+i, _u_konnekt); 
					}
				 break;
			case 4:
				 adr_memN   = adr_memN1_4;                                       // ����� ����� EEPROM � 4
				 _step_mem  = (pgm_read_byte_near(connektN4_default));           // ����� ����� � �������
				 for (int i = 0; i < (_step_mem * 2)+1;i++)                    
					{
					  _u_konnekt = pgm_read_byte_near(connektN4_default+i);
					  i2c_eeprom_write_byte(deviceaddress,adr_memN+i, _u_konnekt); 
					}
				 break;
			default:
				 adr_memN   = adr_memN1_1;                                       // ����� ����� EEPROM � 1 
				 _step_mem  = (pgm_read_byte_near(connektN1_default));           // ����� ����� � �������
				 for (int i = 0; i < (_step_mem * 2)+1;i++)                    
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
			Serial.println("Connector N2");
			myGLCD.print(txt__connect2, CENTER, 20); 
		}
		break;
	case 39:
		set_komm_mcp('A', 39,'O');
		set_komm_mcp('B', 19,'O');
		if (digitalRead(47) == LOW ) 
		{
			n_connect = 3;
			Serial.println("Connector N3");
			myGLCD.print(txt__connect3, CENTER, 20); 
		}
		break;
	case 40:
		set_komm_mcp('A', 40,'O');
		set_komm_mcp('B', 40,'O');
		if (digitalRead(47) == LOW ) 
		{
			n_connect = 1;
			Serial.println("Connector N1");
			myGLCD.print(txt__connect1, CENTER, 20); 
		}
		break;
	case 41:
		set_komm_mcp('A', 41,'O');
		set_komm_mcp('B', 41,'O');
		if (digitalRead(47) == LOW ) 
		{
			n_connect = 4;
			Serial.println("Connector N4");
			myGLCD.print(txt__connect4, CENTER, 20); 
		}
		break;
	}
	if(n_connect ==0) Serial.println("Connector is not detected");
	return n_connect;
}
void test_cabel_N1()
{
	myGLCD.clrScr();
	myGLCD.print(txt_menu1_1, CENTER, 1); 
	myGLCD.setColor(255, 255, 255);                                             // ����� ���������
	myGLCD.drawRoundRect (5, 200, 155, 239);
 	myGLCD.drawRoundRect (160, 200, 315, 239);
	myGLCD.drawLine( 10, 60, 310, 60);
	myGLCD.print(txt_error_connect1, 40, 65); 
	myGLCD.setColor(0, 0, 255);
	myGLCD.fillRoundRect (6, 201, 154, 238);
	myGLCD.fillRoundRect (161, 201, 314, 238);
	myGLCD.setColor(255, 255, 255);  
	myGLCD.setBackColor( 0, 0, 255);
	myGLCD.print( txt_test_repeat, 10, 210);                                    // ���������
	myGLCD.print( txt_test_end, 168, 210);                                      // ���������
	myGLCD.setBackColor( 0, 0, 0);

	if (search_cabel(40)== 1)
	{
		test_cabel_N1_run();

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
						test_cabel_N1_run();
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
      Serial.println("Cable connection error");
	  myGLCD.setColor(VGA_RED);  
	  myGLCD.print(txt_error_connect1, CENTER, 80); 
	  myGLCD.print(txt_error_connect2, CENTER, 110); 
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
	myGLCD.print(txt_error_connect1, 40, 65); 
	myGLCD.setColor(0, 0, 255);
	myGLCD.fillRoundRect (6, 201, 154, 238);
	myGLCD.fillRoundRect (161, 201, 314, 238);
	myGLCD.setColor(255, 255, 255);  
	myGLCD.setBackColor( 0, 0, 255);
	myGLCD.print( txt_test_repeat, 10, 210);                                    // ���������
	myGLCD.print( txt_test_end, 168, 210);                                      // ���������
	myGLCD.setBackColor( 0, 0, 0);

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
      Serial.println("Cable connection error");
	  myGLCD.setColor(VGA_RED);  
	  myGLCD.print(txt_error_connect1, CENTER, 82+19); 
	  myGLCD.print(txt_error_connect2, CENTER, 82+38); 
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
	myGLCD.print(txt_error_connect1, 40, 65); 
	myGLCD.setColor(0, 0, 255);
	myGLCD.fillRoundRect (6, 201, 154, 238);
	myGLCD.fillRoundRect (161, 201, 314, 238);
	myGLCD.setColor(255, 255, 255);  
	myGLCD.setBackColor( 0, 0, 255);
	myGLCD.print( txt_test_repeat, 10, 210);                                    // ���������
	myGLCD.print( txt_test_end, 168, 210);                                      // ���������
	myGLCD.setBackColor( 0, 0, 0);

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
      Serial.println("Cable connection error");
	  myGLCD.setColor(VGA_RED);  
	  myGLCD.print(txt_error_connect1, CENTER, 82+19); 
	  myGLCD.print(txt_error_connect2, CENTER, 82+38); 
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
	myGLCD.print(txt_error_connect1, 40, 65); 
	myGLCD.setColor(0, 0, 255);
	myGLCD.fillRoundRect (6, 201, 154, 238);
	myGLCD.fillRoundRect (161, 201, 314, 238);
	myGLCD.setColor(255, 255, 255);  
	myGLCD.setBackColor( 0, 0, 255);
	myGLCD.print( txt_test_repeat, 10, 210);                                    // ���������
	myGLCD.print( txt_test_end, 168, 210);                                      // ���������
	myGLCD.setBackColor( 0, 0, 0);

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
      Serial.println("Cable connection error");
	  myGLCD.setColor(VGA_RED);  
	  myGLCD.print(txt_error_connect1, CENTER, 82+19); 
	  myGLCD.print(txt_error_connect2, CENTER, 82+38); 
	  myGLCD.setColor(255, 255, 255);  
	  delay(3000);
	}
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
	for(int p        = 0;p < 6;p++)                                              // �������� ���� ������
	{
		myGLCD.print("                    ", x_p, y_p); 
		y_p += 19;
	}
	y_p = 82;                                                                    // ������������ ������ �� �

	if (search_cabel(40)== 1)                                                    // ��������� ����������� ������ 
	{
		for (x_A = 1;x_A < _size_block+1;x_A++)                                  // ���������������� ������ ��������� ��������.
		{
			digitalWrite(46, LOW);                                               // ���������� ����������� ������� �� �����������
			delay(50);                                                           // ����� �� ������������ ��������� 
			canal_N = i2c_eeprom_read_byte(deviceaddress,adr_memN1_1 + x_A);     // �������� � ������ �� EEPROM
			if (canal_N == 1)                                                    // 40 ����� ��� �������� ������ ������������ �������
			{
				set_komm_mcp('A', 40,'O');                                       // ���������� ����������� ���� �����������
			}
		else
			{
	    		set_komm_mcp('A', canal_N,'O');                                  // ���������� ������� ���� �����������
			}
			// ��������������� ��������� ��� ������ ������� "�"
			for (x_B = 1;x_B < _size_block+1;x_B++)                              // ���������������� ������ ��������� ��������.
			{
				canal_N = i2c_eeprom_read_byte(deviceaddress,adr_memN1_1 + x_B + _size_block);

				if (canal_N == 1)                                                // 40 ����� ��� �������� ������ ������������ �������
				{
					set_komm_mcp('B', 40,'O');                                   // ���������� ����������� ���� �����������
				}
				else
				{
	    			set_komm_mcp('B', canal_N,'O');                              // ���������� ������� ���� �����������
				}

				if (digitalRead(47) == LOW && canal_N == x_B)                    // ��������� ���������� � - � �� ������
					{
						Serial.print(x_A);
						Serial.print(" - ");
						Serial.print(canal_N);
						Serial.println(" - Pass");
						myGLCD.printNumI(x_A, 30, 40); 
						myGLCD.print("<->", 66, 40); 
						myGLCD.printNumI(canal_N, 130, 40); 
						myGLCD.print(" - Pass", 170, 40);
						myGLCD.print("He""\xA4", 150, 65); 
					}

				if (x_A == canal_N)                                             // ����� ��������� � ������� �� ������                  
				{
				if (digitalRead(47) != LOW)                                     // ��������� �� ������ �����������
					{
						Serial.print(x_A);
						Serial.print(" - ");
						Serial.print(canal_N);
						Serial.println(" - False");
						myGLCD.print("    ", 150, 65); 

						myGLCD.printNumI(x_A, x_p, y_p);                       // ������������ ��������� ���������
						y_p += 19;
						if ( y_p > 190)                                        // ����� �� ����� ������� ������
						{
						x_p +=42;
						y_p = 82;
						}
					}
				}
				// ����� ��������� �� ��������� ��������� � �������
			}
		}
	}
	else
	{
		Serial.println("Cable connection error");               
		myGLCD.setColor(VGA_RED);  
		myGLCD.print(txt_error_connect1, CENTER, 82+19);                      // �������� ��� ������ �� ���������
		myGLCD.print(txt_error_connect2, CENTER, 82+38); 
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
	for(int p        = 0;p < 6;p++)                                              // �������� ���� ������
	{
		myGLCD.print("                    ", x_p, y_p); 
		y_p += 19;
	}
	y_p = 82;                                                                    // ������������ ������ �� �

	if (search_cabel(1)== 2)                                                     // ��������� ����������� ������ 
	{
		for (x_A = 1;x_A < _size_block+1;x_A++)                                  // ���������������� ������ ��������� ��������.
		{
			digitalWrite(46, LOW);                                               // ���������� ����������� ������� �� �����������
			delay(50);                                                           // ����� �� ������������ ��������� 
			canal_N = i2c_eeprom_read_byte(deviceaddress,adr_memN1_2 + x_A);     // �������� � ������ �� EEPROM
			if (canal_N == 1)                                                    // 40 ����� ��� �������� ������ ������������ �������
			{
				set_komm_mcp('A', 1,'O');                                        // ���������� ����������� ���� �����������
			}
		else
			{
	    		set_komm_mcp('A', canal_N,'O');                                  // ���������� ������� ���� �����������
			}
			// ��������������� ��������� ��� ������ ������� "�"
			for (x_B = 1;x_B < _size_block+1;x_B++)                              // ���������������� ������ ��������� ��������.
			{
				canal_N = i2c_eeprom_read_byte(deviceaddress,adr_memN1_2 + x_B + _size_block);

				if (canal_N == 1)                                                // 40 ����� ��� �������� ������ ������������ �������
				{
					set_komm_mcp('B', 1,'O');                                    // ���������� ����������� ���� �����������
				}
				else
				{
	    			set_komm_mcp('B', canal_N,'O');                              // ���������� ������� ���� �����������
				}

				if (digitalRead(47) == LOW && canal_N == x_B)                    // ��������� ���������� � - � �� ������
					{
						Serial.print(x_A);
						Serial.print(" - ");
						Serial.print(canal_N);
						Serial.println(" - Pass");
						myGLCD.printNumI(x_A, 30, 40); 
						myGLCD.print("<->", 66, 40); 
						myGLCD.printNumI(canal_N, 130, 40); 
						myGLCD.print(" - Pass", 170, 40);
						myGLCD.print("He""\xA4", 150, 65); 
					}

				if (x_A == canal_N)                                             // ����� ��������� � ������� �� ������                  
				{
				if (digitalRead(47) != LOW)                                     // ��������� �� ������ �����������
					{
						Serial.print(x_A);
						Serial.print(" - ");
						Serial.print(canal_N);
						Serial.println(" - False");
						myGLCD.print("    ", 150, 65); 

						myGLCD.printNumI(x_A, x_p, y_p);                       // ������������ ��������� ���������
						y_p += 19;
						if ( y_p > 190)                                        // ����� �� ����� ������� ������
						{
						x_p +=42;
						y_p = 82;
						}
					}
				}
				// ����� ��������� �� ��������� ��������� � �������
			}
		}
	}
	else
	{
		Serial.println("Cable connection error");               
		myGLCD.setColor(VGA_RED);  
		myGLCD.print(txt_error_connect1, CENTER, 82+19);                      // �������� ��� ������ �� ���������
		myGLCD.print(txt_error_connect2, CENTER, 82+38); 
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
	for(int p        = 0;p < 6;p++)                                              // �������� ���� ������
	{
		myGLCD.print("                    ", x_p, y_p); 
		y_p += 19;
	}
	y_p = 82;                                                                    // ������������ ������ �� �

	if (search_cabel(39)== 3)                                                    // ��������� ����������� ������ 
	{
		for (x_A = 1;x_A < _size_block+1;x_A++)                                  // ���������������� ������ ��������� ��������.
		{
			digitalWrite(46, LOW);                                               // ���������� ����������� ������� �� �����������
			delay(50);                                                           // ����� �� ������������ ��������� 
			canal_N = i2c_eeprom_read_byte(deviceaddress,adr_memN1_3 + x_A);     // �������� � ������ �� EEPROM
			if (canal_N == 1)                                                    // 40 ����� ��� �������� ������ ������������ �������
			{
				set_komm_mcp('A', 39,'O');                                       // ���������� ����������� ���� �����������
			}
		else
			{
	    		set_komm_mcp('A', canal_N,'O');                                  // ���������� ������� ���� �����������
			}
			// ��������������� ��������� ��� ������ ������� "�"
			for (x_B = 1;x_B < _size_block+1;x_B++)                              // ���������������� ������ ��������� ��������.
			{
				canal_N = i2c_eeprom_read_byte(deviceaddress,adr_memN1_3 + x_B + _size_block);

				if (canal_N == 1)                                                // 40 ����� ��� �������� ������ ������������ �������
				{
					set_komm_mcp('B', 39,'O');                                   // ���������� ����������� ���� �����������
				}
				else
				{
	    			set_komm_mcp('B', canal_N,'O');                              // ���������� ������� ���� �����������
				}

				Serial.print(x_A);
				Serial.print(" - ");
				Serial.print(x_B);
				Serial.print(" - ");
				Serial.println(canal_N);
				if (digitalRead(47) == LOW && x_A == x_B)                        // ��������� ���������� � - � �� ������
					{
						Serial.print(x_A);
						Serial.print(" - ");
						Serial.print(canal_N);
						Serial.println(" - Pass");
						myGLCD.printNumI(x_A, 30, 40); 
						myGLCD.print("<->", 66, 40); 
						myGLCD.printNumI(canal_N, 130, 40); 
						myGLCD.print(" - Pass", 170, 40);
						myGLCD.print("He""\xA4", 150, 65); 
					}

				//if (x_A != x_B)                                             // ����� ��������� � ������� �� ������                  
				//{
				if (digitalRead(47) != LOW && x_A == x_B)                                     // ��������� �� ������ �����������
					{
						Serial.print(x_A);
						Serial.print(" - ");
						Serial.print(canal_N);
						Serial.println(" - False");
						myGLCD.print("    ", 150, 65); 

						myGLCD.printNumI(x_A, x_p, y_p);                       // ������������ ��������� ���������
						y_p += 19;
						if ( y_p > 190)                                        // ����� �� ����� ������� ������
						{
						x_p +=42;
						y_p = 82;
						}
					}
				//}
				
				// ����� ��������� �� ��������� ��������� � �������
			}
		}
	}
	else
	{
		Serial.println("Cable connection error");               
		myGLCD.setColor(VGA_RED);  
		myGLCD.print(txt_error_connect1, CENTER, 82+19);                      // �������� ��� ������ �� ���������
		myGLCD.print(txt_error_connect2, CENTER, 82+38); 
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
	for(int p        = 0;p < 6;p++)                                              // �������� ���� ������
	{
		myGLCD.print("                    ", x_p, y_p); 
		y_p += 19;
	}
	y_p = 82;                                                                    // ������������ ������ �� �

	if (search_cabel(41)== 4)                                                    // ��������� ����������� ������ 
	{
		for (x_A = 1;x_A < _size_block+1;x_A++)                                  // ���������������� ������ ��������� ��������.
		{
			digitalWrite(46, LOW);                                               // ���������� ����������� ������� �� �����������
			delay(50);                                                           // ����� �� ������������ ��������� 
			canal_N = i2c_eeprom_read_byte(deviceaddress,adr_memN1_4 + x_A);     // �������� � ������ �� EEPROM
			if (canal_N == 1)                                                    // 40 ����� ��� �������� ������ ������������ �������
			{
				set_komm_mcp('A', 41,'O');                                       // ���������� ����������� ���� �����������
			}
		else
			{
	    		set_komm_mcp('A', canal_N,'O');                                  // ���������� ������� ���� �����������
			}
			// ��������������� ��������� ��� ������ ������� "�"
			for (x_B = 1;x_B < _size_block+1;x_B++)                              // ���������������� ������ ��������� ��������.
			{
				canal_N = i2c_eeprom_read_byte(deviceaddress,adr_memN1_4 + x_B + _size_block);

				if (canal_N == 1)                                                // 40 ����� ��� �������� ������ ������������ �������
				{
					set_komm_mcp('B', 41,'O');                                   // ���������� ����������� ���� �����������
				}
				else
				{
	    			set_komm_mcp('B', canal_N,'O');                              // ���������� ������� ���� �����������
				}

				if (digitalRead(47) == LOW && canal_N == x_B)                    // ��������� ���������� � - � �� ������
					{
						Serial.print(x_A);
						Serial.print(" - ");
						Serial.print(canal_N);
						Serial.println(" - Pass");
						myGLCD.printNumI(x_A, 30, 40); 
						myGLCD.print("<->", 66, 40); 
						myGLCD.printNumI(canal_N, 130, 40); 
						myGLCD.print(" - Pass", 170, 40);
						myGLCD.print("He""\xA4", 150, 65); 
					}

				if (x_A == canal_N)                                             // ����� ��������� � ������� �� ������                  
				{
				if (digitalRead(47) != LOW)                                     // ��������� �� ������ �����������
					{
						Serial.print(x_A);
						Serial.print(" - ");
						Serial.print(canal_N);
						Serial.println(" - False");
						myGLCD.print("    ", 150, 65); 

						myGLCD.printNumI(x_A, x_p, y_p);                       // ������������ ��������� ���������
						y_p += 19;
						if ( y_p > 190)                                        // ����� �� ����� ������� ������
						{
						x_p +=42;
						y_p = 82;
						}
					}
				}
				// ����� ��������� �� ��������� ��������� � �������
			}
		}
	}
	else
	{
		Serial.println("Cable connection error");               
		myGLCD.setColor(VGA_RED);  
		myGLCD.print(txt_error_connect1, CENTER, 82+19);                      // �������� ��� ������ �� ���������
		myGLCD.print(txt_error_connect2, CENTER, 82+38); 
		myGLCD.setColor(255, 255, 255);                                       // ������������ ����� �����
		delay(3000);
	}
}
void test_panel_N1()
{

}

void test_all_pin()
{
	myGLCD.clrScr();
	myGLCD.print(txt_test_all, CENTER, 20); 
	myGLCD.print( txt_test_all_exit1, CENTER, 180); 
	myGLCD.print( txt_test_all_exit2, CENTER, 200); 
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

void trigger()
{
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
//	block_t block[BUFFER_BLOCK_COUNT];
	myGLCD.clrScr();
	myGLCD.setBackColor( 0, 0, 0);
	delay(500);
	myGLCD.clrScr();
	buttons_right();
	buttons_channel();
	myGLCD.setBackColor( 0, 0, 0);
	myGLCD.setFont( BigFont);
	myGLCD.setColor(VGA_LIME);
	myGLCD.print(txt_info29,LEFT, 180);
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
		 if (tmode>0) trigger();
	
		// �������� ���������� ������ � ���� ������
		StartSample = micros();
			//ADC_CHER = Channel_x;    // this is (1<<7) | (1<<6) for adc 7= A0, 6=A1 , 5=A2, 4 = A3    
		for( xpos = 0;	xpos < 240; xpos ++) 
			{
				/*
			//	ADC_CHER = Channel_x;    // this is (1<<7) | (1<<6) for adc 7= A0, 6=A1 , 5=A2, 4 = A3    
				ADC_CR = ADC_START ; 	// ��������� ��������������
				 while (!(ADC_ISR_DRDY));
				if (Channel0)
					{
						Sample_osc[xpos][0] = ADC->ADC_CDR[7];
						MaxAnalog0 = max(MaxAnalog0, Sample_osc[xpos][0]);
						MinAnalog0 = min(MinAnalog0, Sample_osc[xpos][0]);
					}
				if (Channel1)
				   {
						Sample_osc[xpos][1] = ADC->ADC_CDR[6];
						MaxAnalog1 = max(MaxAnalog1, Sample_osc[xpos][1]);
						MinAnalog1 = min(MinAnalog1, Sample_osc[xpos][1]);
				   }
				if (Channel2)
					{
						Sample_osc[xpos][2] = ADC->ADC_CDR[5];
						MaxAnalog2 = max(MaxAnalog2, Sample_osc[xpos][2]);
						MinAnalog2 = min(MinAnalog2, Sample_osc[xpos][2]);
					}
				if (Channel3)
					{
						Sample_osc[xpos][3] = ADC->ADC_CDR[4];
						MaxAnalog3 = max(MaxAnalog3, Sample_osc[xpos][3]);
						MinAnalog3 = min(MinAnalog3, Sample_osc[xpos][3]);
					}
				delayMicroseconds(dTime); //dTime
				*/
			}
		EndSample = micros();
		DrawGrid();
  
		// 
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
	}
koeff_h = 7.759*4;
mode1 = 0;
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
			myGLCD.setColor(VGA_RED);
			myGLCD.fillCircle(227,12,10);
		}
	else
		{
			myGLCD.setColor(255,255,255);
			myGLCD.drawCircle(227,12,10);
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

}
void setup_pin()
{
	pinMode(ledPin12, OUTPUT);   
	pinMode(ledPin13, OUTPUT);   
	digitalWrite(ledPin12, HIGH);                          // 
	digitalWrite(ledPin13, LOW);                           // 
	pinMode(48, OUTPUT);   
	pinMode(49, OUTPUT);   
	digitalWrite(48, HIGH);                               // ��������� ������ ��������� ����� U11_1
	digitalWrite(49, HIGH);                               // ��������� ������ ��������� ����� U11_2
	pinMode(46, INPUT);                                   // ����� ������������ ����� �
	pinMode(47, INPUT);                                   // ����� ������������ ����� �
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
	pinMode(ledPin13, OUTPUT);   
	pinMode(ledPin12, OUTPUT);  
	digitalWrite(ledPin12, LOW);                   // 
	digitalWrite(ledPin13, LOW);                   // 

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
	Serial.begin(9600);                                    // ����������� � USB ��
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
	//myGLCD.InitLCD();
	//myGLCD.clrScr();
	//myGLCD.setFont(BigFont);
	//myTouch.InitTouch();
	////myTouch.setPrecision(PREC_MEDIUM);
	//myTouch.setPrecision(PREC_HI);
	////myTouch.setPrecision(PREC_EXTREME);
	//myButtons.setTextFont(BigFont);
	//myButtons.setSymbolFont(Dingbats1_XL);
	// ++++++++++++++++++ ��������� ��� +++++++++++++++++++++++++++++++++++++++++++++++++++
	// set up the ADC
	ADCSRA &= ~PS_128;  // remove bits set by Arduino library

	// you can choose a prescaler from below.
	// PS_16, PS_32, PS_64 or PS_128
	ADCSRA |= PS_128;    // set our own prescaler 

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
