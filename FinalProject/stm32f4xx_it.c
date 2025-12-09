/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
#include "game.h"
#include "seg7.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
GameState gameState = STATE_MENU;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int NUM_NOTES = 68;
int digit = 0;
volatile unsigned char displayBuffer[8] = {SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE};
int NUM_DIGITS = 8;
extern int lowestNote;
extern int highestNote;
extern int row;
extern int playerLane;
extern int score;
extern int miss;
int noteScored = 0;
int note_to_row(int note) {
	if (note==rest)return -1;
	int range = highestNote - lowestNote;
	    if (range <= 0) return 0;        // avoid divide by zero

	    row = (note - lowestNote) * (NUM_DIGITS - 1) / range;

	    if (row < 0) row = 0;
	    if (row >= NUM_DIGITS) row = NUM_DIGITS - 1;

	    return row;
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern HCD_HandleTypeDef hhcd_USB_OTG_FS;
extern TIM_HandleTypeDef htim7;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

COUNT++;  // Increment note duration counter
Vibrato_Count++; // Increment the note vibrato effect counter

/* This code applies vibrato to the current note that is playing  */
if (Vibrato_Count >= Vibrato_Rate)
{
	Vibrato_Count = 0;
	if (Song[INDEX].note > 0)
		{
			Song[INDEX].note += Vibrato_Depth;
			if (Song[INDEX].note > (Save_Note + Vibrato_Depth)) Song[INDEX].note = Save_Note - Vibrato_Depth;

		}
}

if (Animate_On > 0)
{
	Delay_counter++;
	if (Delay_counter > Delay_msec)
	{
		Delay_counter = 0;
		Seven_Segment_Digit(7,*(Message_Pointer),0);
		Seven_Segment_Digit(6,*(Message_Pointer+1),0);
		Seven_Segment_Digit(5,*(Message_Pointer+2),0);
		Seven_Segment_Digit(4,*(Message_Pointer+3),0);
		Seven_Segment_Digit(3,*(Message_Pointer+4),0);
		Seven_Segment_Digit(2,*(Message_Pointer+5),0);
		Seven_Segment_Digit(1,*(Message_Pointer+6),0);
		Seven_Segment_Digit(0,*(Message_Pointer+7),0);
		Message_Pointer++;
		if ((Message_Pointer - Save_Pointer) >= (Message_Length-5)) Message_Pointer = Save_Pointer;

	}
}
// Refresh one digit for multiplexing
 if (gameState == STATE_GAME){
	  Seven_Segment_Digit(digit, displayBuffer[digit], 0);
	  digit = (digit + 1) % 8;
 }

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

	/* Increment TONE counter and dimming ramp counter */
	TONE++;
	ramp++;

	/* This code plays the song from the song array structure */
	if ((Music_ON > 0) && (Song[INDEX].note > 0) && ((Song[INDEX].tempo/Song[INDEX].size - Song[INDEX].space) > COUNT))
	{

		if (Song[INDEX].note <= TONE)
		{
			GPIOD->ODR ^= 1;
			TONE = 0;
		}
	}
	else if ((Music_ON > 0) && Song[INDEX].tempo/Song[INDEX].size > COUNT)
	{
		TONE = 0;
	}
	else if ((Music_ON > 0)&& (INDEX >=0) &&(INDEX< NUM_NOTES) && Song[INDEX].tempo/Song[INDEX].size == COUNT)
	{
		// Only score once per note
		    if (!noteScored) {
		        if (Song[INDEX].note != rest) {
		            if (playerLane == row) {
		                score++;
		            } else {
		                miss++;
		            }
		        }
		        noteScored = 1; // mark this note processed
		    }

		COUNT = 0;
		TONE = 0;
		if (INDEX < NUM_NOTES -1)
				{
					INDEX++;
					Save_Note = Song[INDEX].note;
					noteScored = 0;
				}
		else {
			Music_ON = 0; //end the song
			noteScored = 1;
		}
	}
	else if (Music_ON == 0)
		{
			TONE = 0;
			COUNT = 0;
		}


	//Update 8-digit 7-segment display with d, under, etc
	if (gameState == STATE_GAME) {
		for (int i = 0; i < 8; i++) displayBuffer[i] = SPACE; // clear all

		// current note d
		int d = note_to_row(Song[INDEX].note); // get digit
		if (d >= 0 && d < 8) displayBuffer[d] = CHAR_D;

		// next notes
		char futureNotes[3] = {UNDER, DASH, OVER};
		for (int f = 1; f<=3;f++){
			if ((INDEX+f) < NUM_NOTES && Song[INDEX+f].note != rest) {
				int next = note_to_row(Song[INDEX+f].note);
				if (next >= 0 && next <8) {
					if (displayBuffer[next]!=CHAR_D) {
						displayBuffer[next]= futureNotes[f-1];
					}
				}
			}
		}
	}


  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_HCD_IRQHandler(&hhcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
