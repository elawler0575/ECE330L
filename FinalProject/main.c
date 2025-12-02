/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c  LAB6
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"
#include "seg7.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { menuState, playState} GameState;
//GameState = menuState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_DIGITS 8


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
int DelayValue = 50;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM7_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
//void Play_Note(int note,int size,int tempo,int space);
//extern void Seven_Segment_Digit (unsigned char digit, unsigned char hex_char, unsigned char dot);
//extern void Seven_Segment(unsigned int HexValue);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char ramp = 0;
char RED_BRT = 0;
char GREEN_BRT = 0;
char BLUE_BRT = 0;
char RED_STEP = 1;
char GREEN_STEP = 2;
char BLUE_STEP = 3;
char DIM_Enable = 0;
char Music_ON = 0;
int TONE = 0;
int COUNT = 0;
int INDEX = 0;
int Note = 0;
int Save_Note = 0;
int Vibrato_Depth = 1;
int Vibrato_Rate = 40;
int Vibrato_Count = 0;
char Animate_On = 0;
char Message_Length = 0;
char *Message_Pointer;
char *Save_Pointer;
char MessageButtonLength = 0;
int Delay_msec = 0;
int Delay_counter = 0;


/* PRESS PC10 TO START */
char Message[] =
		{SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,
		CHAR_P,CHAR_R,CHAR_E,CHAR_S,CHAR_S,SPACE,CHAR_P,CHAR_C,CHAR_1,CHAR_0,SPACE,CHAR_T,CHAR_O, SPACE, CHAR_S,CHAR_T,CHAR_A,CHAR_R,CHAR_T,
		SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE};

/*GAME OVER               SCORE: 120383293819*/
//I THINK TO TURN THE INTEGER SCORE TO A CHARACTER, U CALL SEVEN SEGMENT DISPLAY FOR THIS MESSAGE AND THEN APPEND THE CHAR VERSION COVERED OF THE SCORE
char GAMEOVER[] =
		{SPACE,SPACE,SPACE,SPACE,SPACE,
		CHAR_G,CHAR_A,CHAR_M,CHAR_E,SPACE,CHAR_O,CHAR_V,CHAR_E,CHAR_R,SPACE,CHAR_T,CHAR_O, SPACE, SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,
		CHAR_S,CHAR_C,CHAR_O,CHAR_R,CHAR_E }; //APPEND THE SCORE TO THIS MESSAGE

unsigned char Message_Button[] = {SPACE,SPACE,SPACE,CHAR_P,CHAR_R,CHAR_E,CHAR_S,CHAR_S,CHAR_E,CHAR_D,SPACE,SPACE,SPACE};

/* Declare array for Song */
Music Song[100];


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_I2C1_Init();
  //MX_I2S3_Init();
  //MX_SPI1_Init();
  //MX_USB_HOST_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  /*** Configure GPIOs ***/
  GPIOD->MODER = 0x55555555; // set all Port D pins to outputs
  GPIOA->MODER |= 0x000000FF; // Port A mode register - make A0 to A3 analog pins
  GPIOE->MODER |= 0x55555555; // Port E mode register - make E8 to E15 outputs
  //GPIOC->MODER &= ~(3 << (10 * 2));// Port C mode register - all inputs
  GPIOE->ODR = 0xFFFF; // Set all Port E pins high

  /*** Configure ADC1 ***/
  RCC->APB2ENR |= 1<<8;  // Turn on ADC1 clock by forcing bit 8 to 1 while keeping other bits unchanged
  ADC1->SMPR2 |= 1; // 15 clock cycles per sample
  ADC1->CR2 |= 1;        // Turn on ADC1 by forcing bit 0 to 1 while keeping other bits unchanged

  //setting up pc10
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  GPIOC->MODER &= ~(3U << (10 * 2));
  GPIOC->PUPDR &= ~(3U << (10 * 2));
  GPIOC->PUPDR |= (1U << (10 *2));

  /*****************************************************************************************************
  These commands are handled as part of the MX_TIM7_Init() function and don't need to be enabled
  RCC->AHB1ENR |= 1<<5; // Enable clock for timer 7
  __enable_irq(); // Enable interrupts
  NVIC_EnableIRQ(TIM7_IRQn); // Enable Timer 7 Interrupt in the NVIC controller
  *******************************************************************************************************/

  TIM7->PSC = 199; //250Khz timer clock prescaler value, 250Khz = 50Mhz / 200
  TIM7->ARR = 1; // Count to 1 then generate interrupt (divide by 2), 125Khz interrupt rate to increment byte counter for 78Hz PWM
  TIM7->DIER |= 1; // Enable timer 7 interrupt
  TIM7->CR1 |= 1; // Enable timer counting

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* Don't Stop Believing Song */

  Song[0].note = rest;
     Song[0].size = half;
     Song[0].tempo = 1650;
     Song[0].space = 10;
     Song[0].end = 0;

     Song[1].note = Gs5_Ab5;
     Song[1].size = _8th;
     Song[1].tempo = 1650;
     Song[1].space = 10;
     Song[1].end = 0;

     Song[2].note = E5;
     Song[2].size = _8th;
     Song[2].tempo = 1650;
     Song[2].space = 10;
     Song[2].end = 0;

     Song[3].note = Fs5_Gb5;
     Song[3].size = _8th;
     Song[3].tempo = 1650;
     Song[3].space = 10;
     Song[3].end = 0;

     Song[4].note = Fs5_Gb5;
     Song[4].size = quarter;
     Song[4].tempo = 1650;
     Song[4].space = 10;
     Song[4].end = 0;

     Song[5].note = Gs5_Ab5;
     Song[5].size = quarter;
     Song[5].tempo = 1650;
     Song[5].space = 0;
     Song[5].end = 0;

     Song[6].note = Gs5_Ab5;
     Song[6].size = _8th;
     Song[6].tempo = 1650;
     Song[6].space = 10;
     Song[6].end = 0;

     Song[7].note = rest;
     Song[7].size = quarter;
     Song[7].tempo = 1650;
     Song[7].space = 0;
     Song[7].end = 0;

     Song[8].note = rest;
     Song[8].size = half;
     Song[8].tempo = 1650;
     Song[8].space = 10;
     Song[8].end = 0;

     Song[9].note = E5;
     Song[9].size = _8th;
     Song[9].tempo = 1650;
     Song[9].space = 10;
     Song[9].end = 0;

     Song[10].note = E5;
     Song[10].size = _8th;
     Song[10].tempo = 1650;
     Song[10].space = 10;
     Song[10].end = 0;

     Song[11].note = E5;
     Song[11].size = _8th;
     Song[11].tempo = 1650;
     Song[11].space = 10;
     Song[11].end = 0;

     Song[12].note = E5;
     Song[12].size = _8th;
     Song[12].tempo = 1650;
     Song[12].space = 10;
     Song[12].end = 0;

     Song[13].note = B5;
     Song[13].size = quarter;
     Song[13].tempo = 1650;
     Song[13].space = 10;
     Song[13].end = 0;

     Song[14].note = B5;
     Song[14].size = _8th;
     Song[14].tempo = 1650;
     Song[14].space = 0;
     Song[14].end = 0;

     Song[15].note = Gs5_Ab5;
     Song[15].size = quarter;
     Song[15].tempo = 1650;
     Song[15].space = 10;
     Song[15].end = 0;

     Song[16].note = Fs5_Gb5;
     Song[16].size = _8th;
     Song[16].tempo = 1650;
     Song[16].space = 0;
     Song[16].end = 0;

     Song[17].note = Fs5_Gb5;
     Song[17].size = quarter;
     Song[17].tempo = 1650;
     Song[17].space = 10;
     Song[17].end = 0;

     Song[18].note = rest;
     Song[18].size = half;
     Song[18].tempo = 1650;
     Song[18].space = 10;
     Song[18].end = 0;

     Song[19].note = rest;
     Song[19].size = _8th;
     Song[19].tempo = 1650;
     Song[19].space = 10;
     Song[19].end = 0;

     Song[20].note = B4;
     Song[20].size = _8th;
     Song[20].tempo = 1650;
     Song[20].space = 10;
     Song[20].end = 0;

     Song[21].note = Gs5_Ab5;
     Song[21].size = _8th;
     Song[21].tempo = 1650;
     Song[21].space = 10;
     Song[21].end = 0;

     Song[22].note = E5;
     Song[22].size = _8th;
     Song[22].tempo = 1650;
     Song[22].space = 10;
     Song[22].end = 0;

     Song[23].note = Fs5_Gb5;
     Song[23].size = _8th;
     Song[23].tempo = 1650;
     Song[23].space = 10;
     Song[23].end = 0;

     Song[24].note = Fs5_Gb5;
     Song[24].size = quarter;
     Song[24].tempo = 1650;
     Song[24].space = 10;
     Song[24].end = 0;

     Song[25].note = Gs5_Ab5;
     Song[25].size = quarter;
     Song[25].tempo = 1650;
     Song[25].space = 10;
     Song[25].end = 0;

     Song[26].note = Fs5_Gb5;
     Song[26].size = quarter;
     Song[26].tempo = 1650;
     Song[26].space = 10;
     Song[26].end = 0;

     Song[27].note = E5;
     Song[27].size = _8th;
     Song[27].tempo = 1650;
     Song[27].space = 10;
     Song[27].end = 0;

     Song[28].note = Fs5_Gb5;
     Song[28].size = quarter;
     Song[28].tempo = 1650;
     Song[28].space = 0;
     Song[28].end = 0;

     Song[29].note = Fs5_Gb5;
     Song[29].size = _8th;
     Song[29].tempo = 1650;
     Song[29].space = 10;
     Song[29].end = 0;

     Song[30].note = Gs5_Ab5;
     Song[30].size = _8th;
     Song[30].tempo = 1650;
     Song[30].space = 0;
     Song[30].end = 0;

     Song[31].note = Gs5_Ab5;
     Song[31].size = quarter;
     Song[31].tempo = 1650;
     Song[31].space = 10;
     Song[31].end = 0;

     Song[32].note = E5;
     Song[32].size = quarter;
     Song[32].tempo = 1650;
     Song[32].space = 00;
     Song[32].end = 0;

     Song[33].note = E5;
     Song[33].size = half;
     Song[33].tempo = 1650;
     Song[33].space = 100;
     Song[33].end = 0; //end of small town girl verse

     Song[34].note = rest;
     Song[34].size = whole;
     Song[34].tempo = 1650;
     Song[34].space = 10;
     Song[34].end = 0;

     Song[35].note = Gs5_Ab5;
     Song[35].size = _8th;
     Song[35].tempo = 1650;
     Song[35].space = 10;
     Song[35].end = 0;

     Song[36].note = E5;
     Song[36].size = _8th;
     Song[36].tempo = 1650;
     Song[36].space = 10;
     Song[36].end = 0;

     Song[37].note = Fs5_Gb5;
     Song[37].size = _8th;
     Song[37].tempo = 1650;
     Song[37].space = 10;
     Song[37].end = 0;

     Song[38].note = Fs5_Gb5;
     Song[38].size = quarter;
     Song[38].tempo = 1650;
     Song[38].space = 10;
     Song[38].end = 0;

     Song[39].note = Gs5_Ab5;
     Song[39].size = quarter;
     Song[39].tempo = 1650;
     Song[39].space = 0;
     Song[39].end = 0;

     Song[40].note = Gs5_Ab5;
     Song[40].size = _8th;
     Song[40].tempo = 1650;
     Song[40].space = 10;
     Song[40].end = 0;

     Song[41].note = rest;
     Song[41].size = quarter;
     Song[41].tempo = 1650;
     Song[41].space = 0;
     Song[41].end = 0;

     Song[42].note = rest;
     Song[42].size = half;
     Song[42].tempo = 1650;
     Song[42].space = 10;
     Song[42].end = 0;

     Song[43].note = E5;
     Song[43].size = _8th;
     Song[43].tempo = 1650;
     Song[43].space = 10;
     Song[43].end = 0;

     Song[44].note = E5;
     Song[44].size = _8th;
     Song[44].tempo = 1650;
     Song[44].space = 44;
     Song[44].end = 0;

     Song[45].note = E5;
     Song[45].size = _8th;
     Song[45].tempo = 1650;
     Song[45].space = 10;
     Song[45].end = 0;

     Song[46].note = E5;
     Song[46].size = _8th;
     Song[46].tempo = 1650;
     Song[46].space = 10;
     Song[46].end = 0;

     Song[47].note = B5;
     Song[47].size = quarter;
     Song[47].tempo = 1650;
     Song[47].space = 10;
     Song[47].end = 0;

     Song[48].note = B5;
     Song[48].size = _8th;
     Song[48].tempo = 1650;
     Song[48].space = 0;
     Song[48].end = 0;

     Song[49].note = Gs5_Ab5;
     Song[49].size = quarter;
     Song[49].tempo = 1650;
     Song[49].space = 10;
     Song[49].end = 0;

     Song[50].note = Fs5_Gb5;
     Song[50].size = _8th;
     Song[50].tempo = 1650;
     Song[50].space = 0;
     Song[50].end = 0;

     Song[51].note = Fs5_Gb5;
     Song[51].size = quarter;
     Song[51].tempo = 1650;
     Song[51].space = 10;
     Song[51].end = 0;

     Song[52].note = rest;
     Song[52].size = half;
     Song[52].tempo = 1650;
     Song[52].space = 10;
     Song[52].end = 0;

     Song[53].note = rest;
     Song[53].size = _8th;
     Song[53].tempo = 1650;
     Song[53].space = 10;
     Song[53].end = 0;

     Song[54].note = B4;
     Song[54].size = _8th;
     Song[54].tempo = 1650;
     Song[54].space = 10;
     Song[54].end = 0;

     Song[55].note = Gs5_Ab5;
     Song[55].size = _8th;
     Song[55].tempo = 1650;
     Song[55].space = 10;
     Song[55].end = 0;

     Song[56].note = E5;
     Song[56].size = _8th;
     Song[56].tempo = 1650;
     Song[56].space = 10;
     Song[56].end = 0;

     Song[57].note = Fs5_Gb5;
     Song[57].size = _8th;
     Song[57].tempo = 1650;
     Song[57].space = 10;
     Song[57].end = 0;

     Song[58].note = Fs5_Gb5;
     Song[58].size = quarter;
     Song[58].tempo = 1650;
     Song[58].space = 10;
     Song[58].end = 0;

     Song[59].note = Gs5_Ab5;
     Song[59].size = quarter;
     Song[59].tempo = 1650;
     Song[59].space = 10;
     Song[59].end = 0;

     Song[60].note = Fs5_Gb5;
     Song[60].size = quarter;
     Song[60].tempo = 1650;
     Song[60].space = 10;
     Song[60].end = 0;

     Song[61].note = E5;
     Song[61].size = _8th;
     Song[61].tempo = 1650;
     Song[61].space = 10;
     Song[61].end = 0;

     Song[62].note = Fs5_Gb5;
     Song[62].size = quarter;
     Song[62].tempo = 1650;
     Song[62].space = 0;
     Song[62].end = 0;

     Song[63].note = Fs5_Gb5;
     Song[63].size = _8th;
     Song[63].tempo = 1650;
     Song[63].space = 10;
     Song[63].end = 0;

     Song[64].note = Gs5_Ab5;
     Song[64].size = _8th;
     Song[64].tempo = 1650;
     Song[64].space = 0;
     Song[64].end = 0;

     Song[65].note = Gs5_Ab5;
     Song[65].size = quarter;
     Song[65].tempo = 1650;
     Song[65].space = 10;
     Song[65].end = 0;

     Song[66].note = E5;
     Song[66].size = quarter;
     Song[66].tempo = 1650;
     Song[66].space = 00;
     Song[66].end = 0;

     Song[67].note = E5;
     Song[67].size = half;
     Song[67].tempo = 1650;
     Song[67].space = 100;
     Song[67].end = 0;//city boy verse

     Song[68].note = rest;
     Song[68].size = whole;
     Song[68].tempo = 1650;
     Song[68].space = 10;
     Song[68].end = 0;

     Song[69].note = B4;
     Song[69].size = _8th;
     Song[69].tempo = 1650;
     Song[69].space = 10;
     Song[69].end = 0;

     Song[70].note = Fs5_Gb5;
     Song[70].size = _8th;
     Song[70].tempo = 1650;
     Song[70].space = 10;
     Song[70].end = 0;

     Song[71].note = Gs5_Ab5;
     Song[71].size = _8th;
     Song[71].tempo = 1650;
     Song[71].space = 10;
     Song[71].end = 0;

     Song[72].note = E5;
     Song[72].size = _8th;
     Song[72].tempo = 1650;
     Song[72].space = 10;
     Song[72].end = 0;

     Song[73].note = Fs5_Gb5;
     Song[73].size = _8th;
     Song[73].tempo = 1650;
     Song[73].space = 10;
     Song[73].end = 0;

     Song[74].note = Fs5_Gb5;
     Song[74].size = quarter;
     Song[74].tempo = 1650;
     Song[74].space = 10;
     Song[74].end = 0;

     Song[75].note = Gs5_Ab5;
     Song[75].size = quarter;
     Song[75].tempo = 1650;
     Song[75].space = 0;
     Song[75].end = 0;

     Song[76].note = Gs5_Ab5;
     Song[76].size = _8th;
     Song[76].tempo = 1650;
     Song[76].space = 10;
     Song[76].end = 0;

     Song[77].note = rest;
     Song[77].size = quarter;
     Song[77].tempo = 1650;
     Song[77].space = 0;
     Song[77].end = 0;

     Song[78].note = rest;
     Song[78].size = half;
     Song[78].tempo = 1650;
     Song[78].space = 10;
     Song[78].end = 0;

     Song[79].note = Cs4_Db4;
     Song[79].size = _8th;
     Song[79].tempo = 1650;
     Song[79].space = 10;
     Song[79].end = 0;

     Song[80].note = E5;
     Song[80].size = _8th;
     Song[80].tempo = 1650;
     Song[80].space = 10;
     Song[80].end = 0;

     Song[81].note = E5;
     Song[81].size = _8th;
     Song[81].tempo = 1650;
     Song[81].space = 10;
     Song[81].end = 0;

     Song[82].note = E5;
     Song[82].size = _8th;
     Song[82].tempo = 1650;
     Song[82].space = 10;
     Song[82].end = 0;

     Song[83].note = E5;
     Song[83].size = _8th;
     Song[83].tempo = 1650;
     Song[83].space = 10;
     Song[83].end = 0;

     Song[84].note = B5;
     Song[84].size = quarter;
     Song[84].tempo = 1650;
     Song[84].space = 10;
     Song[84].end = 0;

     Song[85].note = B5;
     Song[85].size = _8th;
     Song[85].tempo = 1650;
     Song[85].space = 0;
     Song[85].end = 0;

          Song[86].note = Gs5_Ab5;
          Song[86].size = quarter;
          Song[86].tempo = 1650;
          Song[86].space = 10;
          Song[86].end = 0;

          Song[87].note = Fs5_Gb5;
          Song[87].size = _8th;
          Song[87].tempo = 1650;
          Song[87].space = 0;
          Song[87].end = 0;

          Song[88].note = Fs5_Gb5;
          Song[88].size = quarter;
          Song[88].tempo = 1650;
          Song[88].space = 10;
          Song[88].end = 0;

          Song[89].note = rest;
          Song[89].size = quarter;
          Song[89].tempo = 1650;
          Song[89].space = 10;
          Song[89].end = 0;

          Song[90].note = Gs5_Ab5;
          Song[90].size = _8th;
          Song[90].tempo = 1650;
          Song[90].space = 10;
          Song[90].end = 0;

          Song[91].note = E5;
          Song[91].size = _8th;
          Song[91].tempo = 1650;
          Song[91].space = 10;
          Song[91].end = 0;

          Song[92].note = Gs5_Ab5;
          Song[92].size = quarter;
          Song[92].tempo = 1650;
          Song[92].space = 10;
          Song[92].end = 0;

          Song[93].note = Gs5_Ab5;
          Song[93].size = _8th;
          Song[93].tempo = 1650;
          Song[93].space = 10;
          Song[93].end = 0;

          Song[94].note = E5;
          Song[94].size = _8th;
          Song[94].tempo = 1650;
          Song[94].space = 10;
          Song[94].end = 0;

          Song[95].note = Fs5_Gb5;
          Song[95].size = _8th;
          Song[95].tempo = 1650;
          Song[95].space = 10;
          Song[95].end = 0;

          Song[96].note = Fs5_Gb5;
          Song[96].size = quarter;
          Song[96].tempo = 1650;
          Song[96].space = 10;
          Song[96].end = 0;

          Song[97].note = Gs5_Ab5;
          Song[97].size = quarter;
          Song[97].tempo = 1650;
          Song[97].space = 10;
          Song[97].end = 0;

          Song[98].note = Fs5_Gb5;
          Song[98].size = quarter;
          Song[98].tempo = 1650;
          Song[98].space = 10;
          Song[98].end = 0;

          Song[99].note = E5;
          Song[99].size = _8th;
          Song[99].tempo = 1650;
          Song[99].space = 0;
          Song[99].end = 0;

          Song[100].note = Fs5_Gb5;
          Song[100].size = quarter;
          Song[100].tempo = 1650;
          Song[100].space = 10;
          Song[100].end = 0;

          Song[101].note = Gs5_Ab5;
          Song[101].size = _8th;
          Song[101].tempo = 1650;
          Song[101].space = 10;
          Song[101].end = 0;

          Song[102].note = Fs5_Gb5;
          Song[102].size = quarter;
          Song[102].tempo = 1650;
          Song[102].space = 00;
          Song[102].end = 0;

          Song[103].note = Fs5_Gb5;
          Song[103].size = _8th;
          Song[103].tempo = 1650;
          Song[103].space = 10;
          Song[103].end = 0;

          Song[104].note = Gs5_Ab5;
          Song[104].size = _8th;
          Song[104].tempo = 1650;
          Song[104].space = 10;
          Song[104].end = 0;

          Song[105].note = Fs5_Gb5;
          Song[105].size = quarter;
          Song[105].tempo = 1650;
          Song[105].space = 00;
          Song[105].end = 0;

          Song[107].note = Fs5_Gb5;
          Song[107].size = _8th;
          Song[107].tempo = 1650;
          Song[107].space = 10;
          Song[107].end = 0;

          Song[108].note = Gs5_Ab5;
          Song[108].size = _8th;
          Song[108].tempo = 1650;
          Song[108].space = 10;
          Song[108].end = 0;

          Song[109].note = Gs5_Ab5;
          Song[109].size = _16th;
          Song[109].tempo = 1650;
          Song[109].space = 00;
          Song[109].end = 0;

          Song[110].note = Fs5_Gb5;
          Song[110].size = _16th;
          Song[110].tempo = 1650;
          Song[110].space = 0;
          Song[110].end = 0;

          Song[111].note = E5;
          Song[111].size = quarter;
          Song[111].tempo = 1650;
          Song[111].space = 100;
          Song[111].end = 1;
          //singer

   Save_Note = Song[0].note;  // Needed for vibrato effect
  INDEX = 0;

 /*Message_Pointer = &Message[0];
  Save_Pointer = &Message[0];
  Message_Length = sizeof(Message)/sizeof(Message[0]);
  Delay_msec = 200;*/
  Animate_On = 1;

  GameState gameState = menuState;
  GameState lastState = menuState; // track previous state


  uint32_t adc_val;
  uint8_t dot_digit;
  while (1) {
	// start ADC conversion
	  ADC1->SQR3=1;
	  ADC1 ->CR2 |= 1;
	  while(!(ADC1->SR & (1<<1))){}

	  adc_val = ADC1->DR;

	  int dot_index = (adc_val * (NUM_DIGITS-1)/4095);
	  if (dot_index >= NUM_DIGITS) dot_index = NUM_DIGITS -1;

	  for (int i = 0; i < NUM_DIGITS; i++){
		  GPIOE -> ODR |= 0xFF00;
		  uint8_t dot = (i == dot_index) ? 1 : 0;
		  Seven_Segment_Digit(i, SPACE, dot);
		  HAL_Delay(1);
	  }

	/*  GPIOE->ODR = 0x0000;
	  HAL_Delay(500);
	  GPIOE->ODR = 0xFFFF;
	  HAL_Delay(500);*/

/*

	  switch(gameState) {
	  case menuState:
		  if (lastState != menuState){
			  Message_Pointer = &Message[0];
			  Save_Pointer = &Message[0];
			  Message_Length = sizeof(Message)/sizeof(Message[0]);
			  Delay_msec = 200;
			  Animate_On = 1;
			  lastState = menuState;
		  }

		  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10)== GPIO_PIN_RESET) {
			  HAL_Delay(50); //debounce
			  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10)== GPIO_PIN_RESET){
				  gameState = playState;
			  }
		  }
		  break;
	  case playState:
		  if (lastState!= playState){
		  INDEX = 0;
		  Message_Pointer = &Message_Button[0];
		  Save_Pointer = &Message_Button[0];
		  MessageButtonLength = sizeof(Message_Button)/ sizeof(Message_Button[0]);
		  Delay_msec = 200;
		  Animate_On=1;
		  lastState = playState;
		  }
		  break;
	  }*/


	  Music_ON = 0;
}


	 /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */


/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);


  //
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
