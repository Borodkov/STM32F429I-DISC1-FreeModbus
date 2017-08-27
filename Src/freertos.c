/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include <string.h>

#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"

#include "lcd_log.h"
#include "cpu_utils.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId adcTaskHandle;
osThreadId btnTaskHandle;

/* USER CODE BEGIN Variables */
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartADCTask(void const * argument);
void StartBTNTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of ADC */
  osThreadDef(ADC, StartADCTask, osPriorityNormal, 0, 128);
  adcTaskHandle = osThreadCreate(osThread(ADC), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(BTN, StartBTNTask, osPriorityBelowNormal, 0, 128);
  btnTaskHandle = osThreadCreate(osThread(BTN), NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartADCTask function */
void StartADCTask(void const * argument)
{
  /* USER CODE BEGIN StartADCTask */
  RTC_TimeTypeDef time;
  RTC_DateTypeDef date;
  char *weekDays[] = {"TMP","Mon","Tue","Wed","Thu","Fri","Sat","Sun"};
  char sToUART[40] = {0};
  
  #define  ADC_WINDOW  8
  uint16_t adcBuf[ADC_WINDOW];
  uint16_t adcBufMean;
  
  float t, Vsense = 0, V25 = 0.76, Avg_Slope = 0.0025;
  
  /***************************************************************************
  Set default date and time
  ***************************************************************************/
  date.Year    = (uint8_t)17;
  date.Month   = RTC_MONTH_AUGUST;
  date.Date    = ((uint8_t)26);
  date.WeekDay = RTC_WEEKDAY_SATURDAY;
  time.Hours   = (uint8_t)7;
  time.Minutes = (uint8_t)8;
  time.Seconds = (uint8_t)9;
  
  // set default date and time
  HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);
  HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
   
  /***************************************************************************
  Toggle LED3 every ADC conversion, and LED4 - when DMA complete (ADC_WINDOW conversions)
  ***************************************************************************/
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adcBuf, ADC_WINDOW);
  
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
    
    /***************************************************************************
    every second update RTC vars
    ***************************************************************************/
    HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
    
    /***************************************************************************
    Computation of ADC conversions raw data to physical values 
    ***************************************************************************/
    adcBufMean = 0;
    for (uint32_t i = 0; i < ADC_WINDOW; ++i) {
      adcBufMean += adcBuf[i];
    } adcBufMean >>= 3; // /8 or 'adcBufMean / ADC_WINDOW;'
    
    // formula from DS on stm32f42xx
    Vsense = 2.94 * ((float)adcBufMean / 4096);
    t = ( (Vsense - V25) / Avg_Slope ) + 25;
    
    /***************************************************************************
    Display the Temperature Value on the LCD 
    ***************************************************************************/
    sprintf(sToUART, "%s 20%02u/%02u/%02u %02u:%02u:%02u %+.1f [C]\n", 
            weekDays[date.WeekDay],
            date.Year,
            date.Month,
            date.Date,
            time.Hours,
            time.Minutes,
            time.Seconds,
            t);
    LCD_UsrLog(sToUART);
    
    strcat(sToUART,"\r");
    BSP_COM_Transmit(COM1, sToUART);
  }
  /* USER CODE END StartADCTask */
}

/* USER CODE BEGIN Application */
void StartBTNTask(void const * argument)
{
  uint32_t tikcs;
  char sFooter[30] = "Press BTN to pause | Load 00%";

  /* Infinite loop */
  for(;;)
  {
    osDelay(50);
    
    if (BSP_PB_GetState(BUTTON_KEY) == SET) {
      // w8 
      tikcs = 0;
      while (BSP_PB_GetState(BUTTON_KEY) == SET && tikcs <= 50) {
        osDelay(10);
        tikcs++;        
      }
      
      // simple press (less then 500 ms)
      if (tikcs < 50) {
        LCD_UsrLog("USER BTN: simple press\n");
        
        osThreadSuspend(adcTaskHandle);
      } else {
        // long press
        // ...
        LCD_UsrLog("USER BTN: long press\n");
        osThreadResume(adcTaskHandle);
        
        // w8 when button relesed
        while (BSP_PB_GetState(BUTTON_KEY) == SET) {
          osDelay(10);
        } osDelay(100);
      }
    }
    
    sprintf(sFooter,"Press BTN to pause | Load %02u%", osGetCPUUsage());
    sFooter[29] = 0;
    LCD_LOG_SetFooter((uint8_t *)sFooter);
  }
  /* USER CODE END StartADCTask */
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 if (GPIO_Pin == KEY_BUTTON_PIN)
 {
   ;
 }
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
