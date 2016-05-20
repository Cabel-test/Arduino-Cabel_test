#include <SPI.h>
#include "AnalogBinLogger.h"
#include <UTFT.h>
#include <UTouch.h>
#include <UTFT_Buttons.h>
//#include <DueTimer.h>
#include "Wire.h"


extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t Dingbats1_XL[];
extern uint8_t SmallSymbolFont[];

// Настройка монитора

UTFT    myGLCD(ITDB32S,38,39,40,41);

UTouch        myTouch(6,5,4,3,2);
//
//// Finally we set up UTFT_Buttons :)
UTFT_Buttons  myButtons(&myGLCD, &myTouch);

boolean default_colors = true;
uint8_t menu_redraw_required = 0;


//******************Назначение переменных для хранения № опций меню (клавиш)****************************

 int but1, but2, but3, but4, but5, but6, but7, but8, but9, but10, butX, butY, but_m1, but_m2, but_m3, but_m4, but_m5, pressed_button;
 int m2 = 1; // Переменная номера меню
 
//=====================================================================================

	int x_kn = 30;  // Смещение кнопок по Х
	int dgvh;
	const int hpos = 95; //set 0v on horizontal  grid
	int port = 0;
	int x_osc,y_osc;
	int Input = 0;
	int Old_Input = 0;
	int Sample[254];
	int OldSample[254];
	int Sample_osc[254][4];
	int OldSample_osc[254][4];
	int PageSample_osc[240][10][4];
	unsigned long PageSample_Num[10];
	int Page_count = 0;
	int x_pos_count = 0;
	int YSample_osc[254][4];
	float VSample_osc[254][4];
	unsigned long LongFile = 0;
	float StartSample = 0; 
	float EndSample = 0;
	float koeff_h = 7.759*4;
	int MaxAnalog = 0;
	int MaxAnalog0 = 0;
	int MaxAnalog1 = 0;
	int MaxAnalog2 = 0;
	int MaxAnalog3 = 0;
	unsigned long SrednAnalog = 0;
	unsigned long SrednAnalog0 = 0;
	unsigned long SrednAnalog1 = 0;
	unsigned long SrednAnalog2 = 0;
	unsigned long SrednAnalog3 = 0;
	unsigned long SrednCount = 0;
	bool Set_x = false;
	bool osc_line_off0 = false;
	bool osc_line_off1 = false;
	bool osc_line_off2 = false;
	bool osc_line_off3 = false;
	bool repeat = false;
	int16_t count_repeat = 0;
	bool save_files = false;
	bool sled = false;
	int Set_ADC = 10;
	int MinAnalog = 500;
	int MinAnalog0 = 500;
	int MinAnalog1 = 500;
	int MinAnalog2 = 500;
	int MinAnalog3 = 500;
	int mode = 0;

	int mode1 = 0;             //Переключение чувствительности
	int dTime = 1;
	int tmode = 1;
	int t_in_mode = 0;
	int Trigger = 0;
	int SampleSize = 0;
	float SampleTime = 0;
	float v_const = 0.0008057;
	int x_measure = 0 ;              // Переменная для изменения частоты измерения источника питания
	bool strob_start = true;



 //***************** Назначение переменных для хранения текстов*****************************************************

char  txt_menu1_1[]          = "PE\x81\x86""CTPATOP";                                                       // "РЕГИСТРАТОР"
char  txt_menu1_2[]          = "CAMO\x89\x86""CE\x8C";                                                      // "САМОПИСЕЦ"
char  txt_menu1_3[]          = "PE\x81\x86""CT.+ CAMO\x89.";                                                // "РЕГИСТ. + САМОП."
char  txt_menu1_4[]          = "PA\x80OTA c SD";                                                            // "РАБОТА с SD"

char  txt_ADC_menu1[]        = "\x85""a\xA3\x9D""c\xAC \x99""a\xA2\xA2\xABx";                                                               //
char  txt_ADC_menu2[]        = "\x89poc\xA1o\xA4p \xA5""a\x9E\xA0""a";                                                                    //
char  txt_ADC_menu3[]        = "\x89""epe\x99""a\xA7""a \x97 KOM";                                                            //
char  txt_ADC_menu4[]        = "B\x91XO\x82";                                                                      //

char  txt_osc_menu1[]        = "Oc\xA6\x9D\xA0\xA0o\x98pa\xA5";                                                              //
char  txt_osc_menu2[]        = "Oc\xA6\x9D\xA0\xA0.1-18\xA1\x9D\xA2";                                                               //
char  txt_osc_menu3[]        = "O\xA8\x9d\x96\x9F\x9D";                                                                    //
char  txt_osc_menu4[]        = "B\x91XO\x82";           

char  txt_SD_menu1[]         = "\x89poc\xA1o\xA4p \xA5""a\x9E\xA0""a";                                                                 //
char  txt_SD_menu2[]         = "\x86\xA2\xA5o SD";                                                                   //
char  txt_SD_menu3[]         = "\x8Bop\xA1""a\xA4 SD";                                                                 //
char  txt_SD_menu4[]         = "B\x91XO\x82";           

char  txt_info6[]             = "Info: ";                                                                   //Info: 
char  txt_info7[]             = "Writing:"; 
char  txt_info8[]             = "%"; 
char  txt_info9[]             = "Done: "; 
char  txt_info10[]            = "Seconds"; 
char  txt_info11[]            = "ESC->PUSH Display"; 
char  txt_info12[]            = "CTAPT"; 
char  txt_info13[]            = "Deleting tmp file"; 
char  txt_info14[]            = "Erasing all data"; 
//char  txt_info15[]            = "Stop->PUSH Display"; 
char  txt_info16[]            = "File: "; 
char  txt_info17[]            = "Max block :"; 
char  txt_info18[]            = "Record time: "; 
char  txt_info19[]            = "Sam count:"; 
char  txt_info20[]            = "Samples/sec: "; 
char  txt_info21[]            = "O\xA8\x9D\x96\x9F\x9D:"; 
char  txt_info22[]            = "Sample pins:"; 
char  txt_info23[]            = "ADC bits:"; 
char  txt_info24[]            = "ADC clock kHz:"; 
char  txt_info25[]            = "Sample Rate:"; 
char  txt_info26[]            = "Sample interval:"; 
char  txt_info27[]            = "Creating new file"; 
char  txt_info28[]            = "\x85""a\xA3\x9D""c\xAC \x99""a\xA2\xA2\xABx"; 
char  txt_info29[]            = "Stop->PUSH Disp"; 
char  txt_info30[]            = "\x89o\x97\xA4op."; 


#define strob_pin A4    // Вход для запуска измерения

uint32_t ulChannel;

//------------------------------------------------------------------------------
// Analog pin number list for a sample. 

int Channel_x = 0;
int Channel_trig = 0;
bool Channel0 = true;
bool Channel1 = false;
bool Channel2 = false;
bool Channel3 = false;
int count_pin = 0;
int set_strob = 100;

//------------------------------------------------------------------------------
// Analog pin number list for a sample.  Pins may be in any order and pin
// numbers may be repeated.
const uint8_t PIN_LIST[] = {3};
//const uint8_t PIN_LIST[] = {0, 1, 2, 3};
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
#define FILE_BASE_NAME "ANALOG"

// Set RECORD_EIGHT_BITS non-zero to record only the high 8-bits of the ADC.
#define RECORD_EIGHT_BITS 0
//------------------------------------------------------------------------------
// Pin definitions.
// Digital pin to indicate an error, set to -1 if not used.
// The led blinks for fatal errors. The led goes on solid for SD write
// overrun errors and logging continues.
const int8_t ERROR_LED_PIN = 13;




// cache for SD block
//cache_t cache;

//// MBR information
//uint8_t partType;
//uint32_t relSector;
//uint32_t partSize;
//
//// Fake disk geometry
//uint8_t numberOfHeads;
//uint8_t sectorsPerTrack;
//
//// FAT parameters
//uint16_t reservedSectors;
//uint8_t sectorsPerCluster;
//uint32_t fatStart;
//uint32_t fatSize;
//uint32_t dataStart;
//
//// constants for file system structure
//uint16_t const BU16 = 128;
//uint16_t const BU32 = 8192;
//
////  strings needed in file system structures
//char noName[] = "NO NAME    ";
//char fat16str[] = "FAT16   ";
//char fat32str[] = "FAT32   ";

//------------------------------------------------------------------------------
// Buffer definitions.
//
// The logger will use SdFat's buffer plus BUFFER_BLOCK_COUNT additional 
// buffers.  QUEUE_DIM must be a power of two larger than
//(BUFFER_BLOCK_COUNT + 1).
//
//#if RAMEND < 0X8FF
//#error Too little SRAM
////
//#elif RAMEND < 0X10FF
//// Use total of two 512 byte buffers.
//const uint8_t BUFFER_BLOCK_COUNT = 1;
//// Dimension for queues of 512 byte SD blocks.
//const uint8_t QUEUE_DIM = 4;  // Must be a power of two!
////
//#elif RAMEND < 0X20FF
//// Use total of five 512 byte buffers.
//const uint8_t BUFFER_BLOCK_COUNT = 4;
//// Dimension for queues of 512 byte SD blocks.
//const uint8_t QUEUE_DIM = 8;  // Must be a power of two!
////
//#elif RAMEND < 0X40FF
//// Use total of 13 512 byte buffers.
//const uint8_t BUFFER_BLOCK_COUNT = 12;
//// Dimension for queues of 512 byte SD blocks.
//const uint8_t QUEUE_DIM = 16;  // Must be a power of two!
////
//#else  // RAMEND
//// Use total of 29 512 byte buffers.
//const uint8_t BUFFER_BLOCK_COUNT = 28;
//// Dimension for queues of 512 byte SD blocks.
//const uint8_t QUEUE_DIM = 32;  // Must be a power of two!
//#endif  // RAMEND

// Use total of five 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 4;
//const uint8_t BUFFER_BLOCK_COUNT = 1;
// Dimension for queues of 512 byte SD blocks.
//const uint8_t QUEUE_DIM = 4;  // Must be a power of two!
const uint8_t QUEUE_DIM = 8;  // Must be a power of two!

// Number of analog pins to log.
const uint8_t PIN_COUNT = sizeof(PIN_LIST)/sizeof(PIN_LIST[0]);

// Minimum ADC clock cycles per sample interval
const uint16_t MIN_ADC_CYCLES = 15;

// Extra cpu cycles to setup ADC with more than one pin per sample.
const uint16_t ISR_SETUP_ADC = 100;

// Maximum cycles for timer0 system interrupt, millis, micros.
const uint16_t ISR_TIMER0 = 160;

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
block_t* isrBuf;  //Указатель на текущий буфер

// Need new buffer if true.
volatile bool isrBufNeeded = true;

// overrun count
volatile uint16_t isrOver = 0;

// ADC configuration for each pin.
volatile uint8_t adcmux[PIN_COUNT];
volatile uint8_t adcsra[PIN_COUNT];
volatile uint8_t adcsrb[PIN_COUNT];
volatile uint8_t adcindex = 1;

// Insure no timer events are missed.
volatile bool timerError = false;
volatile bool timerFlag = false;
//------------------------------------------------------------------------------
// ADC done interrupt.
ISR(ADC_vect) 
{
  // Read ADC data.
#if RECORD_EIGHT_BITS
  uint8_t d = ADCH;
#else  // RECORD_EIGHT_BITS
  // This will access ADCL first. 
  uint16_t d = ADC;
#endif  // RECORD_EIGHT_BITS

    /* 	Serial.print("ADMUX - ");
		Serial.print(ADMUX);
		Serial.print("  adcindex - ");
		Serial.println(adcindex);
*/


  if (isrBufNeeded && emptyHead == emptyTail) 
	{
		// no buffers - count overrun нет буферов - рассчитывайте перерасход памяти
		if (isrOver < 0XFFFF) isrOver++;
		// Avoid missed timer error.
		timerFlag = false;
		return;
	}

  // Start ADC
  if (PIN_COUNT > 1) 
	{
		ADMUX  = adcmux[adcindex];
		ADCSRB = adcsrb[adcindex];
		ADCSRA = adcsra[adcindex];
		if (adcindex == 0) timerFlag = false;
		adcindex =  adcindex < (PIN_COUNT - 1) ? adcindex + 1 : 0;
	/*	Serial.print("ADMUX - ");
		Serial.print(ADMUX);
		Serial.print("  adcindex - ");
		Serial.println(adcindex);*/
	}
  else 
	{
		timerFlag = false;
	}
  // Check for buffer needed. Необходимо проверить буфер
  if (isrBufNeeded) 
	{   
		// Remove buffer from empty queue.  Удалить буфер из пустого очереди.
		isrBuf = emptyQueue[emptyTail];
		emptyTail = queueNext(emptyTail);
		isrBuf->count = 0;
		isrBuf->overrun = isrOver;
		isrBufNeeded = false;    
	}
  // Store ADC data.
  isrBuf->data[isrBuf->count++] = d;
  
  // Check for buffer full. Проверка, буфер полон?
  if (isrBuf->count >= PIN_COUNT*SAMPLES_PER_BLOCK) 
	{
		// Put buffer isrIn full queue.   Положите буфер isrIn полной очереди
		uint8_t tmp = fullHead;  // Avoid extra fetch of volatile fullHead.  Избежать дополнительных
		fullQueue[tmp] = (block_t*)isrBuf;
		fullHead = queueNext(tmp);
		// Set buffer needed and clear overruns. Установить необходимый буффер и очистить перерасход
		isrBufNeeded = true;
		isrOver = 0;
	}
}
//------------------------------------------------------------------------------
// timer1 interrupt to clear OCF1B
ISR(TIMER1_COMPB_vect) 
{
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
  fatalBlink();
}
//------------------------------------------------------------------------------
//
void fatalBlink() {
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
  myGLCD.setBackColor( 0, 0, 0);
  myGLCD.setColor(255, 255, 255);
  uint8_t adps;  // prescaler bits for ADCSRA 
  uint32_t ticks = F_CPU*SAMPLE_INTERVAL + 0.5;  // Sample interval cpu cycles.

  if (ADC_REF & ~((1 << REFS0) | (1 << REFS1))) 
	  {
		error("Invalid ADC reference");
	  }
#ifdef ADC_PRESCALER
  if (ADC_PRESCALER > 7 || ADC_PRESCALER < 2) 
	  {
		error("Invalid ADC prescaler");
	  }
  adps = ADC_PRESCALER;
#else  // ADC_PRESCALER
  // Allow extra cpu cycles to change ADC settings if more than one pin.
  int32_t adcCycles = (ticks - ISR_TIMER0)/PIN_COUNT;
//					  - (PIN_COUNT > 1 ? ISR_SETUP_ADC : 0); // Уточнить для чего
					  
  for (adps = 7; adps > 0; adps--) 
	  {
		 if (adcCycles >= (MIN_ADC_CYCLES << adps)) break;
	  }
#endif  // ADC_PRESCALER
  meta->adcFrequency = F_CPU >> adps;
  if (meta->adcFrequency > (RECORD_EIGHT_BITS ? 2000000 : 1000000)) 
  {
	error("Sample Rate Too High");
  }
#if ROUND_SAMPLE_INTERVAL
  // Round so interval is multiple of ADC clock.
  ticks += 1 << (adps - 1);
  ticks >>= adps;
  ticks <<= adps;
#endif  // ROUND_SAMPLE_INTERVAL

  if (PIN_COUNT > sizeof(meta->pinNumber)/sizeof(meta->pinNumber[0])) 
  {
	error("Too many pins");
  }
  meta->pinCount = PIN_COUNT;
  meta->recordEightBits = RECORD_EIGHT_BITS;
  
  for (int i = 0; i < PIN_COUNT; i++) 
  {
	uint8_t pin = PIN_LIST[i];
	if (pin >= NUM_ANALOG_INPUTS) error("Invalid Analog pin number");
	meta->pinNumber[i] = pin;
	
   // Set ADC reference and low three bits of analog pin number.   
	adcmux[i] = (pin & 7) | ADC_REF;
	if (RECORD_EIGHT_BITS) adcmux[i] |= 1 << ADLAR;
	
	// If this is the first pin, trigger on timer/counter 1 compare match B.
	adcsrb[i] = i == 0 ? (1 << ADTS2) | (1 << ADTS0) : 0;
#ifdef MUX5
	if (pin > 7) adcsrb[i] |= (1 << MUX5);
#endif  // MUX5
	adcsra[i] = (1 << ADEN) | (1 << ADIE) | adps;
	adcsra[i] |= i == 0 ? 1 << ADATE : 1 << ADSC;
  }
  // Serial.flush();
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
  } else if (ticks < 0X10000*64) {
	// prescale 64, CTC mode
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);
	tshift = 6;
  } else if (ticks < 0X10000*256) {
	// prescale 256, CTC mode
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12);
	tshift = 8;
  } else if (ticks < 0X10000*1024) {
	// prescale 1024, CTC mode
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12) | (1 << CS10);
	tshift = 10;
  } else {
	error("Sample Rate Too Slow");
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
  /*
  Serial.print(F("Sample pins:"));
  myGLCD.print(txt_info22,LEFT, 25);
  myGLCD.setColor(VGA_YELLOW);
  for (unsigned int i = 0; i < meta->pinCount; i++) 
  {
	Serial.print(' ');
	Serial.print(meta->pinNumber[i], DEC);
	myGLCD.printNumI(meta->pinNumber[i], 230+(20*i), 25);// 
  }
  */
 /*
  Serial.println(); 
  Serial.print(F("ADC bits: "));
  Serial.println(meta->recordEightBits ? 8 : 10);
  Serial.print(F("ADC clock kHz: "));
  Serial.println(meta->adcFrequency/1000);
  Serial.print(F("Sample Rate: "));
  Serial.println(sampleRate);  
  Serial.print(F("Sample interval usec: "));
  Serial.println(1000000.0/sampleRate, 4); 
  Serial.flush();
  myGLCD.setColor(255, 255, 255);
  myGLCD.print(txt_info23,LEFT, 45);
  myGLCD.setColor(VGA_YELLOW);
  myGLCD.printNumI(meta->recordEightBits ? 8 : 10, RIGHT, 45);// 
  myGLCD.setColor(255, 255, 255);
  myGLCD.print(txt_info24,LEFT, 65);
  myGLCD.setColor(VGA_YELLOW);
  myGLCD.printNumI(meta->adcFrequency/1000, RIGHT, 65);// 
  myGLCD.setColor(255, 255, 255);
  myGLCD.print(txt_info25,LEFT, 85);
  myGLCD.setColor(VGA_YELLOW);
  myGLCD.printNumI(sampleRate, RIGHT, 85);// 
  myGLCD.setColor(255, 255, 255);
  myGLCD.print(txt_info26,LEFT, 105);
  myGLCD.setColor(VGA_YELLOW);
  myGLCD.printNumI(1000000.0/sampleRate, RIGHT, 105);// 
  myGLCD.setColor(255, 255, 255);
  */
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
void adcStop() {
  TIMSK1 = 0;
  ADCSRA = 0;
}



//------------------------------------------------------------------------------
bool ledOn = false;
void firstHandler()
{
	// int time_start = micros();
	////ADC_CHER = Channel_x;    // this is (1<<7) | (1<<6) for adc 7= A0, 6=A1 , 5=A2, 4 = A3    
	//ADC_CR = ADC_START ; 	// Запустить преобразование

 // // while (!(ADC_ISR & ADC_ISR_DRDY));

 // if (isrBufNeeded && emptyHead == emptyTail)   //  Необходим новый  буфер, если true 
	//  {
	//	// no buffers - count overrun нет буферов - рассчитывайте перерасход
	//	if (isrOver < 0XFFFF) isrOver++;
	//
	//	// Avoid missed timer error. Избежать пропущенных ошибку таймера.
	//	timerFlag = false;
	//	return;
	//  }

 //// Check for buffer needed.  Проверьте буфера, необходимого.
 // if (isrBufNeeded) 
	//  {   
	//	// Remove buffer from empty queue. Удалить буфер из пустого очереди.
	//	isrBuf = emptyQueue[emptyTail];
	//	emptyTail = queueNext(emptyTail);
	//	isrBuf->count = 0;            // Счнтчик в 0
	//	isrBuf->overrun = isrOver;    // 
	//	isrBufNeeded = false;    
	//  }
 // // Store ADC data.
 //  //while (!(ADC_ISR & ADC_ISR_DRDY));
 // while (!(ADC_ISR_DRDY));
	//  //  int time_period = micros() - time_start;
	// //   isrBuf->data[isrBuf->count++] = time_period;
 // /*
	//	if (Channel0) isrBuf->data[isrBuf->count++] = ADC->ADC_CDR[7];
	//	if (Channel1) isrBuf->data[isrBuf->count++] = ADC->ADC_CDR[6];
	//	if (Channel2) isrBuf->data[isrBuf->count++] = ADC->ADC_CDR[5];
	//	if (Channel3) isrBuf->data[isrBuf->count++] = ADC->ADC_CDR[4];
 //*/
 // if (isrBuf->count >= count_pin*SAMPLES_PER_BLOCK)  //  SAMPLES_PER_BLOCK -количество памяти выделенное на  один вход 
	//							 // SAMPLES_PER_BLOCK = DATA_DIM16/PIN_COUNT; // 254 разделить на количество входов
	//  {
	//	// Put buffer isrIn full queue.  Положите буфер isrIn полной очереди.
	//	uint8_t tmp = fullHead;  // Avoid extra fetch of volatile fullHead.
	//	fullQueue[tmp] = (block_t*)isrBuf;
	//	fullHead = queueNext(tmp);
	//	// Set buffer needed and clear overruns.
	//	isrBufNeeded = true;
	//	isrOver = 0;
	//  }
 //
}
//==============================================================================

//==============================================================================
/*
void adcInit(metadata_t* meta) 
{
  uint8_t adps;  // prescaler bits for ADCSRA 
  uint32_t ticks = F_CPU*SAMPLE_INTERVAL + 0.5;  // Sample interval cpu cycles.

#ifdef ADC_PRESCALER
  if (ADC_PRESCALER > 7 || ADC_PRESCALER < 2) {
	error("Invalid ADC prescaler");
  }
  adps = ADC_PRESCALER;
#else  // ADC_PRESCALER
  // Allow extra cpu cycles to change ADC settings if more than one pin.
  int32_t adcCycles = (ticks - ISR_TIMER0)/count_pin;
					  - (count_pin > 1 ? ISR_SETUP_ADC : 0);
					  
  for (adps = 7; adps > 0; adps--) {
	 if (adcCycles >= (MIN_ADC_CYCLES << adps)) break;
  }
#endif  // ADC_PRESCALER
   meta->adcFrequency = F_CPU >> adps;
  if (meta->adcFrequency > (RECORD_EIGHT_BITS ? 2000000 : 1000000)) 
  {
//	error("Sample Rate Too High");
  }

  #if ROUND_SAMPLE_INTERVAL
  // Round so interval is multiple of ADC clock.
  ticks += 1 << (adps - 1);
  ticks >>= adps;
  ticks <<= adps;
#endif  // ROUND_SAMPLE_INTERVAL



	  meta->pinCount = count_pin;
	  meta->recordEightBits = RECORD_EIGHT_BITS;
	//  isrOver = 0;
  
		 int i = 0;
	
		if (Channel0 )
			{
				meta->pinNumber[i] = 0;
				i++;
			}
		if (Channel1 )
			{
				meta->pinNumber[i] = 1;
				i++;
			}
		
		if (Channel2 ) 
			{
				meta->pinNumber[i] = 2;
				i++;
			}

		if (Channel3 ) 
			{
			   meta->pinNumber[i] = 3;
			}

// разделить на предделителе
		 uint8_t tshift = 10;
 // divide by prescaler
  ticks >>= tshift;
  // set TOP for timer reset
 // ICR1 = ticks - 1;
  // compare for ADC start
 // OCR1B = 0;
  
  // multiply by prescaler
  ticks <<= tshift;


	  // Sample interval in CPU clock ticks.
	  meta->sampleInterval = ticks;

	  meta->cpuFrequency = F_CPU;
  
	  float sampleRate = (float)meta->cpuFrequency/meta->sampleInterval;
	//  Serial.print(F("Sample pins:"));
	///*  for (int i = 0; i < meta->pinCount; i++) 
	//  {
	//	Serial.print(' ');
	//	Serial.print(meta->pinNumber[i], DEC);
	//  }
 //
	//  Serial.println(); 
	//  Serial.println(F("ADC bits: 12 "));
	//  Serial.print(F("ADC interval usec: "));
	//  Serial.println(set_strob);
	  //Serial.print(F("Sample Rate: "));
	  //Serial.println(sampleRate);  
	  //Serial.print(F("Sample interval usec: "));
	  //Serial.println(1000000.0/sampleRate, 4); 
}
*/

/*
void adcStart() {
  // initialize ISR
  isrBufNeeded = true;
  isrOver = 0;
//  adcindex = 1;

  //// Clear any pending interrupt.
  //ADCSRA |= 1 << ADIF;
  //
  //// Setup for first pin.
  //ADMUX = adcmux[0];
  //ADCSRB = adcsrb[0];
  //ADCSRA = adcsra[0];

  // Enable timer1 interrupts.
  timerError = false;
  timerFlag = false;
 
}
*/
void draw_Glav_Menu()
{
  but1 = myButtons.addButton( 10,  20, 250,  35, txt_menu1_1);
  but2 = myButtons.addButton( 10,  65, 250,  35, txt_menu1_2);
  but3 = myButtons.addButton( 10, 110, 250,  35, txt_menu1_3);
  but4 = myButtons.addButton( 10, 155, 250,  35, txt_menu1_4);
  butX = myButtons.addButton( 279, 199,  40,  40, "W", BUTTON_SYMBOL); // кнопка Часы 
  myGLCD.setColor(VGA_BLACK);
  myGLCD.setBackColor(VGA_WHITE);
  myGLCD.setColor(0, 255, 0);
  myGLCD.setBackColor(0, 0, 0);
  myButtons.drawButtons();
}
void swichMenu() // Тексты меню в строках "txt....."
	
{
	 while(1) 
	   {
		 myButtons.setTextFont(BigFont);                      // Установить Большой шрифт кнопок  

			if (myTouch.dataAvailable() == true)              // Проверить нажатие кнопок
			  {
				pressed_button = myButtons.checkButtons();    // Если нажата - проверить что нажато
					 if (pressed_button==butX)                // Нажата вызов часы
						  {  
							 myGLCD.setFont( BigFont);

							 myGLCD.clrScr();
							 myButtons.drawButtons();         // Восстановить кнопки
						  }
	
				   //*****************  Меню №1  **************

				   if (pressed_button==but1)
					   {
							 Draw_menu_ADC1();
							 menu_ADC();
							 myGLCD.clrScr();
							 myButtons.drawButtons();;
					   }
	  
				   if (pressed_button==but2)
					   {
							Draw_menu_Osc();
							menu_Oscilloscope();
							myGLCD.clrScr();
							myButtons.drawButtons();
					   }
	  
				   if (pressed_button==but3)
					   {

							myGLCD.clrScr();
							myButtons.drawButtons();
					   }
				   if (pressed_button==but4)
					   {

							myGLCD.clrScr();
							myButtons.drawButtons();
					   }

			 } 
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
//++++++++++++++++++++++++++ Конец меню прибора ++++++++++++++++++++++++
void Draw_menu_Osc()
{
	myGLCD.clrScr();
	myGLCD.setFont( BigFont);
	myGLCD.setBackColor(0, 0, 255);
	for (int x=0; x<4; x++)
		{
			myGLCD.setColor(0, 0, 255);
			myGLCD.fillRoundRect (30, 20+(50*x), 290,60+(50*x));
			myGLCD.setColor(255, 255, 255);
			myGLCD.drawRoundRect (30, 20+(50*x), 290,60+(50*x));
		}
	myGLCD.print( txt_osc_menu1, CENTER, 30);     // 
	myGLCD.print( txt_osc_menu2, CENTER, 80);   
	myGLCD.print( txt_osc_menu3, CENTER, 130);   
	myGLCD.print( txt_osc_menu4, CENTER, 180);      
}
void menu_Oscilloscope()   // Меню "Осциллоскопа", вызывается из меню "Самописец"
{
	while (true)
		{
		delay(10);
		if (myTouch.dataAvailable())
			{
				myTouch.read();
				int	x=myTouch.getX();
				int	y=myTouch.getY();

				if ((x>=30) && (x<=290))       // 
					{
					if ((y>=20) && (y<=60))    // Button: 1  "Oscilloscope"
						{
							waitForIt(30, 20, 290, 60);
							myGLCD.clrScr();
							oscilloscope();
							Draw_menu_Osc();
						}
					if ((y>=70) && (y<=110))   // Button: 2 "Oscill_Time"
						{
							waitForIt(30, 70, 290, 110);
							myGLCD.clrScr();
						}
					if ((y>=120) && (y<=160))  // Button: 3 "checkOverrun"  Проверка ошибок
						{
							waitForIt(30, 120, 290, 160);
							myGLCD.clrScr();
						}
					if ((y>=170) && (y<=220))  // Button: 4 "EXIT" Выход
						{
							waitForIt(30, 170, 290, 210);
							break;
						}
				}
			}
	   }

}
void trigger()
{
//	 ADC_CHER = Channel_trig;
//
//	for(int tr = 0; tr < 1000; tr++)
//	{
//		ADC_CR = ADC_START ; 	// Запустить преобразование
//		while (!(ADC_ISR_DRDY));
//		switch (t_in_mode) 
//			{
//				case 1:
////					Input = ADC->ADC_CDR[6];
//					break;
//				case 2:
//	//				Input = ADC->ADC_CDR[5];
//					break;
//				case 3:
////					Input = ADC->ADC_CDR[4];
//					break;
//				//default: 
//				//	Input = ADC->ADC_CDR[7];
//			}
//		// if (Input<Trigger) break;
//		 if (Input< 15) break;
//	}
//	//delayMicroseconds(2);
//
//	for(int tr = 0; tr < 1000; tr++)
//	{
//		 ADC_CR = ADC_START ; 	// Запустить преобразование
//		 while (!(ADC_ISR_DRDY));
//		 switch (t_in_mode) 
//			{
//				case 1:
////					Input = ADC->ADC_CDR[6];
//					break;
//				case 2:
////					Input = ADC->ADC_CDR[5];
//					break;
//				case 3:
////					Input = ADC->ADC_CDR[4];
//					break;
//				//default: 
//				//	Input = ADC->ADC_CDR[7];
//			}
//	
//		if (Input>Trigger) break;
//		
//	}
//
}

void oscilloscope()  // просмотр в реальном времени на большой скорости
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

	uint8_t* cache = (uint8_t*)240;
	adcInit((metadata_t*) &block[0]);

		// Initialize queues.  Инициализация очереди.
	emptyHead = emptyTail = 0;
	fullHead = fullTail = 0;
  
	// Use SdFat buffer for one block.  Используйте SdFat буфер для одного блока
	emptyQueue[emptyHead] = (block_t*)cache;
	emptyHead = queueNext(emptyHead);

	  
	// Put rest of buffers in the empty queue. Поместите остальные буферов в пустую очередь.
	for (uint8_t i = 0; i < BUFFER_BLOCK_COUNT; i++) 
		{
			emptyQueue[emptyHead] = &block[i];
			emptyHead = queueNext(emptyHead);
		}




	for( xpos = 0; xpos < 239;	xpos ++) // Стереть старые данные

		{
			OldSample_osc[xpos][0] = 0;
			OldSample_osc[xpos][1] = 0;
			OldSample_osc[xpos][2] = 0;
			OldSample_osc[xpos][3] = 0;
		}

	uint32_t bn = 1;
	uint32_t t0 = millis();
	uint32_t t1 = t0;
	uint32_t overruns = 0;
	uint32_t count = 0;
	uint32_t maxLatency = 0;


	//adcStart();  // Start logging interrupts.


	while(1) 
	{
		 DrawGrid();
		 if (myTouch.dataAvailable())
			{
				delay(10);
				myTouch.read();
				x_osc=myTouch.getX();
				y_osc=myTouch.getY();

				if ((x_osc>=2) && (x_osc<=240))  //  Область экрана
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

			if ((x_osc>=250) && (x_osc<=284))  // Боковые кнопки
			  {
				  if ((y_osc>=1) && (y_osc<=40))  // Первая  период
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

			 if ((y_osc>=45) && (y_osc<=85))  // Вторая - триггер
				 {
					waitForIt(250, 45, 318, 85);
					tmode --;
					if (tmode < 0)tmode = 0;
					if (tmode == 1){ Trigger = MinAnalog+10; myGLCD.print(" 0%  ", 268, 65);}
					if (tmode == 2){ Trigger = MaxAnalog/2;  myGLCD.print(" 50% ", 266, 65);}
					if (tmode == 3){ Trigger = MaxAnalog-10; myGLCD.print("100%", 270, 65);}
					if (tmode == 0)myGLCD.print(" Off ", 268, 65);

				 }
			 if ((y_osc>=90) && (y_osc<=130))  // Третья - делитель
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
			 if ((y_osc>=135) && (y_osc<=175))  // Четвертая разрешение
				 {

				 }
		   }
		
			if ((x_osc>=284) && (x_osc<=318))  // Боковые кнопки
			  {
				  if ((y_osc>=1) && (y_osc<=40))  // Первая  период
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

			 if ((y_osc>=45) && (y_osc<=85))  // Вторая - триггер
				 {
					waitForIt(250, 45, 318, 85);
					tmode ++;
					if (tmode > 3)tmode = 3;
					if (tmode == 1){ Trigger = MinAnalog+10; myGLCD.print(" 0%  ", 268, 65);}
					if (tmode == 2){ Trigger = MaxAnalog/2;  myGLCD.print(" 50% ", 266, 65);}
					if (tmode == 3){ Trigger = MaxAnalog-10; myGLCD.print("100%", 270, 65);}
					if (tmode == 0)myGLCD.print(" Off ", 268, 65);
				 }
			 if ((y_osc>=90) && (y_osc<=130))  // Третья - делитель
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
			 if ((y_osc>=135) && (y_osc<=175))  // Четвертая разрешение
				 {
					waitForIt(250, 135, 318, 175);
				 }

		   }

		if ((x_osc>=250) && (x_osc<=318))  

			{
			if ((y_osc>=200) && (y_osc<=239))  //   Нижние кнопки  
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

			 if ((y_osc>=205) && (y_osc<=239))  // Нижние кнопки переключения входов
					{
						 touch_osc();
					}
		}
		 trig_min_max(t_in_mode);
		 if (tmode>0) trigger();



		 adcStart();  // Start logging interrupts.


		 if (fullHead != fullTail) 
			{
				// Get address of block to write.  Получить адрес блока, чтобы написать
				block_t* pBlock = fullQueue[fullTail];
						// Get address of block to write.  Получить адрес блока, чтобы написать
				uint32_t usec = micros();

				usec = micros() - usec;
				t1 = millis();
				if (usec > maxLatency) maxLatency = usec;
				count += pBlock->count;
		
				// Move block to empty queue.
				emptyQueue[emptyHead] = pBlock;
				emptyHead = queueNext(emptyHead);
				fullTail = queueNext(fullTail);
				bn++;
				if (bn == FILE_BLOCK_COUNT) 
				{
				// File full so stop ISR calls.
				adcStop();
				break;
				}
				for( xpos = 0;	xpos < 239; xpos ++) 
			        {

               		if (Channel0)
						{
							 Sample_osc[xpos][0] = pBlock->data[xpos];
	    				}
					if (Channel1)
					   {
							Sample_osc[xpos][1] = pBlock->data[xpos];
					   }
					if (Channel2)
						{
							Sample_osc[xpos][2] = pBlock->data[xpos];
						}
					if (Channel3)
						{
							Sample_osc[xpos][3] = pBlock->data[xpos];
						}
			        		//delayMicroseconds(dTime); //dTime
					}





		    }
/*
		// Записать аналоговый сигнал в блок памяти
		StartSample = micros();
	//!!	ADC_CHER = Channel_x;    // this is (1<<7) | (1<<6) for adc 7= A0, 6=A1 , 5=A2, 4 = A3    
		for( xpos = 0;	xpos < 240; xpos ++) 
			{
			//	ADC_CHER = Channel_x;    // this is (1<<7) | (1<<6) for adc 7= A0, 6=A1 , 5=A2, 4 = A3    
		//!!		ADC_CR = ADC_START ; 	// Запустить преобразование
		//!!		 while (!(ADC_ISR_DRDY));
				if (Channel0)
					{
//						Sample_osc[xpos][0] = ADC->ADC_CDR[7];
						MaxAnalog0 = max(MaxAnalog0, Sample_osc[xpos][0]);
						MinAnalog0 = min(MinAnalog0, Sample_osc[xpos][0]);
					}
				if (Channel1)
				   {
//						Sample_osc[xpos][1] = ADC->ADC_CDR[6];
						MaxAnalog1 = max(MaxAnalog1, Sample_osc[xpos][1]);
						MinAnalog1 = min(MinAnalog1, Sample_osc[xpos][1]);
				   }
				if (Channel2)
					{
	//					Sample_osc[xpos][2] = ADC->ADC_CDR[5];
						MaxAnalog2 = max(MaxAnalog2, Sample_osc[xpos][2]);
						MinAnalog2 = min(MinAnalog2, Sample_osc[xpos][2]);
					}
				if (Channel3)
					{
//						Sample_osc[xpos][3] = ADC->ADC_CDR[4];
						MaxAnalog3 = max(MaxAnalog3, Sample_osc[xpos][3]);
						MinAnalog3 = min(MinAnalog3, Sample_osc[xpos][3]);
					}
				delayMicroseconds(dTime); //dTime
			}

		*/
		 

		//8888888888888888888888888888888888888888888888
		EndSample = micros();
		DrawGrid();
  
		// 
		for( int xpos = 0; xpos < 239;	xpos ++)
			{
				//  Стереть предыдущий экран
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
			
				if (Channel2|osc_line_off2)
					{
						ypos_osc1_2 = 255-(OldSample_osc[ xpos + 1][2]/koeff_h) - hpos; 
						ypos_osc2_2 = 255-(OldSample_osc[ xpos + 2][2]/koeff_h) - hpos;
						if(ypos_osc1_2 < 0) ypos_osc1_2 = 0;
						if(ypos_osc2_2 < 0) ypos_osc2_2 = 0;
						if(ypos_osc1_2 > 220) ypos_osc1_2 = 220;
						if(ypos_osc2_2 > 220) ypos_osc2_2 = 220;
						myGLCD.setColor( 0, 0, 0);
						myGLCD.drawLine (xpos + 1, ypos_osc1_2, xpos + 2, ypos_osc2_2);
						myGLCD.drawLine (xpos + 2, ypos_osc1_2+1, xpos + 3, ypos_osc2_2+1);
						if (xpos > 237 & Channel2 == false )
							{
								osc_line_off2 = false;
							}
					}
			
				if (Channel3|osc_line_off3)
					{
						ypos_osc1_3 = 255-(OldSample_osc[ xpos + 1][3]/koeff_h) - hpos; 
						ypos_osc2_3 = 255-(OldSample_osc[ xpos + 2][3]/koeff_h) - hpos;
						if(ypos_osc1_3 < 0) ypos_osc1_3 = 0;
						if(ypos_osc2_3 < 0) ypos_osc2_3 = 0;
						if(ypos_osc1_3 > 220) ypos_osc1_3 = 220;
						if(ypos_osc2_3 > 220) ypos_osc2_3 = 220;
						myGLCD.drawLine (xpos + 1, ypos_osc1_3, xpos + 2, ypos_osc2_3);
						myGLCD.drawLine (xpos + 2, ypos_osc1_3+1, xpos + 3, ypos_osc2_3+1);
						if (xpos > 237 & Channel3 == false )
							{
								osc_line_off3 = false;
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
				
				if (Channel2)
					{
						//Draw the new data
						myGLCD.setColor( VGA_RED);
						ypos_osc1_2 = 255-(Sample_osc[ xpos][2]/koeff_h) - hpos;
						ypos_osc2_2 = 255-(Sample_osc[ xpos + 1][2]/koeff_h)- hpos;
						if(ypos_osc1_2 < 0) ypos_osc1_2 = 0;
						if(ypos_osc2_2 < 0) ypos_osc2_2 = 0;
						if(ypos_osc1_2 > 220) ypos_osc1_2  = 220;
						if(ypos_osc2_2 > 220) ypos_osc2_2 = 220;
						myGLCD.drawLine (xpos, ypos_osc1_2, xpos + 1, ypos_osc2_2);
						myGLCD.drawLine (xpos+1, ypos_osc1_2+1, xpos + 2, ypos_osc2_2+1);
					}
				
				if (Channel3)
					{
						myGLCD.setColor( VGA_BLUE);
						ypos_osc1_3 = 255-(Sample_osc[ xpos][3]/koeff_h) - hpos;
						ypos_osc2_3 = 255-(Sample_osc[ xpos + 1][3]/koeff_h)- hpos;
						if(ypos_osc1_3 < 0) ypos_osc1_3 = 0;
						if(ypos_osc2_3 < 0) ypos_osc2_3 = 0;
						if(ypos_osc1_3 > 220) ypos_osc1_3  = 220;
						if(ypos_osc2_3 > 220) ypos_osc2_3 = 220;
						myGLCD.drawLine (xpos, ypos_osc1_3, xpos + 1, ypos_osc2_3);
						myGLCD.drawLine (xpos+1, ypos_osc1_3+1, xpos + 2, ypos_osc2_3+1);
					}

					OldSample_osc[xpos][0] = Sample_osc[xpos][0];
					OldSample_osc[xpos][1] = Sample_osc[xpos][1];
					OldSample_osc[xpos][2] = Sample_osc[xpos][2];
					OldSample_osc[xpos][3] = Sample_osc[xpos][3];
			}
	}
koeff_h = 7.759*4;
mode1 = 0;
Trigger = 0;
StartSample = millis();
myGLCD.setFont( BigFont);
while (myTouch.dataAvailable()){}
}

void buttons_right()  //  Правые кнопки  oscilloscope
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
	scale_time();   // вывод цифровой шкалы
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
	myGLCD.print("0",3, 163);         // В начале шкалы
	if (mode == 0)                    // Остальная сетка
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
void buttons_channel()  // Нижние кнопки переключения входов
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
						myGLCD.fillRoundRect (10, 200, 60, 205);   // Индикатор цвета линии
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
						myGLCD.fillRoundRect (70, 200, 120, 205);   // Индикатор цвета линии
						myGLCD.fillRoundRect (70, 210, 120, 239);
						myGLCD.setColor(255, 255, 255);
						myGLCD.print("1", 92, 212);
						myGLCD.print("BXOD", 80, 226);
					}

				if (Channel2)
					{
						myGLCD.setColor(VGA_RED);
						myGLCD.fillRoundRect (130, 200, 180, 205);
						myGLCD.setColor(VGA_LIME);
						myGLCD.setBackColor( VGA_LIME);
						myGLCD.fillRoundRect (130, 210, 180, 239);
						myGLCD.setColor(0, 0,0);
						myGLCD.print("2", 152, 212);
						myGLCD.print("BXOD", 140, 226);
						osc_line_off2 = true;
					}
				else
					{
						myGLCD.setColor(0,0,0);
						myGLCD.setBackColor( 0,0,0);
						myGLCD.fillRoundRect (130, 210, 180, 239);
						myGLCD.fillRoundRect (130, 200, 180, 205);   // Индикатор цвета линии
						myGLCD.setColor(255, 255, 255);
						myGLCD.print("2", 152, 212);
						myGLCD.print("BXOD", 140, 226);
					}

				if (Channel3)
					{
						myGLCD.setColor(VGA_BLUE);
						myGLCD.fillRoundRect (190, 200, 240, 205);
						myGLCD.setColor(VGA_LIME);
						myGLCD.setBackColor( VGA_LIME);
						myGLCD.fillRoundRect (190, 210, 240, 239);
						myGLCD.setColor(0, 0, 0);
						myGLCD.print("3", 212, 212);
						myGLCD.print("BXOD", 200, 226);
						osc_line_off3 = true;
					}
				else
					{
						myGLCD.setColor(0,0,0);
						myGLCD.setBackColor( 0,0,0);
						myGLCD.fillRoundRect (190, 210, 240, 239);
						myGLCD.fillRoundRect (190, 200, 240, 205);   // Индикатор цвета линии
						myGLCD.setColor(255, 255, 255);
						myGLCD.print("3", 212, 212);
						myGLCD.print("BXOD", 200, 226);
					}
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawRoundRect (10, 210, 60, 239);
	myGLCD.drawRoundRect (70, 210, 120, 239);
	myGLCD.drawRoundRect (130, 210, 180, 239);
	myGLCD.drawRoundRect (190, 210, 240, 239);
}
void chench_Channel()
{
	//Подготовка номера аналогового сигнала, количества каналов и кода настройки АЦП
		   Channel_x = 0;
//!!		   ADC_CHER = Channel_x;
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
		
		if (Channel2 ) 
			{
				Channel_x|=0x20;
				count_pin++;
			}

		if (Channel3) 
			{
				Channel_x|=0x10;
				count_pin++;
			}
	//!!	 ADC_CHER = Channel_x;
//!!		 SAMPLES_PER_BLOCK = DATA_DIM16/count_pin;
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
	myGLCD.setColor(255, 255, 255);           // Белая окантовка
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
	//myGLCD.setColor(255, 255, 255);           // Белая окантовка

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

void touch_osc()  //  Нижнее меню осциллографа
{
	delay(10);
	myTouch.read();
	x_osc=myTouch.getX();
	y_osc=myTouch.getY();
	myGLCD.setFont( SmallFont);

	if ((y_osc>=210) && (y_osc<=239))                         //   Нижние кнопки
	  {
		if ((x_osc>=10) && (x_osc<=60))                       //  Вход 0
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
				MinAnalog0 = 4095;
				MaxAnalog0 = 0;
			}

		else if ((x_osc>=70) && (x_osc<=120))                    //  Вход 1
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
		else if ((x_osc>=130) && (x_osc<=180))                    //  Вход 2
			{
				waitForIt(130, 210, 180, 239);

					Channel2 = !Channel2;

				if (Channel2)
					{
						myGLCD.setColor(VGA_RED);
						myGLCD.fillRoundRect (130, 200, 180, 205);
						myGLCD.setColor(VGA_LIME);
						myGLCD.setBackColor( VGA_LIME);
						myGLCD.fillRoundRect (130, 210, 180, 239);
						myGLCD.setColor(0, 0,0);
						myGLCD.print("2", 152, 212);
						myGLCD.print("BXOD", 140, 226);
						osc_line_off2 = true;
					}
				else
					{
						myGLCD.setColor(0,0,0);
						myGLCD.setBackColor( 0,0,0);
						myGLCD.fillRoundRect (130, 210, 180, 239);
						myGLCD.fillRoundRect (130, 200, 180, 205);
						myGLCD.setColor(255, 255, 255);
						myGLCD.drawRoundRect (130, 210, 180, 239);
						myGLCD.print("2", 152, 212);
						myGLCD.print("BXOD", 140, 226);
					}
				chench_Channel();
				MinAnalog2 = 4095;
				MaxAnalog2 = 0;
			}
		else if ((x_osc>=190) && (x_osc<=240))                     //  Вход 3
			{
				waitForIt(190, 210, 240, 239);

					Channel3 = !Channel3;

				if (Channel3)
					{
						myGLCD.setColor(VGA_BLUE);
						myGLCD.fillRoundRect (190, 200, 240, 205);
						myGLCD.setColor(VGA_LIME);
						myGLCD.setBackColor( VGA_LIME);
						myGLCD.fillRoundRect (190, 210, 240, 239);
						myGLCD.setColor(0, 0, 0);
						myGLCD.print("3", 212, 212);
						myGLCD.print("BXOD", 200, 226);
						osc_line_off3 = true;
					}
				else
					{
						myGLCD.setColor(0,0,0);
						myGLCD.setBackColor( 0,0,0);
						myGLCD.fillRoundRect (190, 210, 240, 239);
						myGLCD.fillRoundRect (190, 200, 240, 205);
						myGLCD.setColor(255, 255, 255);
						myGLCD.drawRoundRect (190, 210, 240, 239);
						myGLCD.print("3", 212, 212);
						myGLCD.print("BXOD", 200, 226);
					}

				chench_Channel();
				MinAnalog3 = 4095;
				MaxAnalog3 = 0;
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
						case 2:

						if (Channel2)
							{
								Channel_trig = 0x20;
								myGLCD.print(" ON ", 270, 226);
								MinAnalog = MinAnalog2 ;
								MaxAnalog = MaxAnalog2 ;
							}
						else
							{
								myGLCD.print(" OFF", 270, 226);
							}
						  break;
						case 3:
						 if (Channel3)
							{
								Channel_trig = 0x10;
								myGLCD.print(" ON ", 270, 226);
								MinAnalog = MinAnalog3 ;
								MaxAnalog = MaxAnalog3 ;
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
						case 2:

						if (Channel2)
							{
								MinAnalog = MinAnalog2 ;
								MaxAnalog = MaxAnalog2 ;
							}
						  break;
						case 3:
						 if (Channel3)
							{
								MinAnalog = MinAnalog3 ;
								MaxAnalog = MaxAnalog3 ;
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

void Draw_menu_ADC1()
{
	myGLCD.clrScr();
	myGLCD.setFont( BigFont);
	myGLCD.setBackColor(0, 0, 255);
	for (int x=0; x<4; x++)
		{
			myGLCD.setColor(0, 0, 255);
			myGLCD.fillRoundRect (30, 20+(50*x), 290,60+(50*x));
			myGLCD.setColor(255, 255, 255);
			myGLCD.drawRoundRect (30, 20+(50*x), 290,60+(50*x));
		}
	myGLCD.print( txt_ADC_menu1, CENTER, 30);       // "Record data"
	myGLCD.print( txt_ADC_menu2, CENTER, 80);       // "List fales"
	myGLCD.print( txt_ADC_menu3, CENTER, 130);      // "Data to Serial"
	myGLCD.print( txt_ADC_menu4, CENTER, 180);      // "EXIT"
}
void menu_ADC()
{
	char c;

	while (true)
		{
		delay(10);
		if (myTouch.dataAvailable())
			{
				myTouch.read();
				int	x=myTouch.getX();
				int	y=myTouch.getY();

				if ((x>=30) && (x<=290))       // Upper row
					{
					if ((y>=20) && (y<=60))    // Button: 1
						{
							waitForIt(30, 20, 290, 60);

						}
					if ((y>=70) && (y<=110))   // Button: 2
						{
							waitForIt(30, 70, 290, 110);

						}
					if ((y>=120) && (y<=160))  // Button: 3
						{
							waitForIt(30, 120, 290, 160);

						}
					if ((y>=170) && (y<=220))  // Button: 4
						{
							waitForIt(30, 170, 290, 210);
							break;
						}
				}
			}
	   }
}

//------------------------------------------------------------------------------
void setup(void) 
{
	if (ERROR_LED_PIN >= 0) 
	{
		pinMode(ERROR_LED_PIN, OUTPUT);
	}

	Serial.begin(115200);

	myGLCD.InitLCD();
	myGLCD.clrScr();
	myGLCD.setFont(BigFont);
	myGLCD.setBackColor(0, 0, 255);

	myTouch.InitTouch();
	myTouch.setPrecision(PREC_MEDIUM);
	//myTouch.setPrecision(PREC_HI);
	myButtons.setTextFont(BigFont);
	myButtons.setSymbolFont(Dingbats1_XL);

//	ADC_MR |= 0x00000100 ; // ADC full speed
	// Read the first sample pin to init the ADC.
	  analogRead(PIN_LIST[3]);
 
	//chench_Channel();

	myGLCD.setBackColor(0, 0, 255);
	pinMode(strob_pin, INPUT);
	digitalWrite(strob_pin, HIGH);
	Serial.println(F("Setup Ok!"));
}
//------------------------------------------------------------------------------
void loop(void) 
{
	draw_Glav_Menu();
	swichMenu();
}
