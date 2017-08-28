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
osThreadId uartTaskHandle;
osThreadId modbusTaskHandle;

/* USER CODE BEGIN Variables */
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;

char sDateTemp[40] = {0};
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartADCTask(void const * argument);
void StartBTNTask(void const * argument);
void StartUartTask(void const * argument);
void StartModBusTask(void const * argument);

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
  osThreadDef(BTN, StartBTNTask, osPriorityLow, 0, 128);
  btnTaskHandle = osThreadCreate(osThread(BTN), NULL);
  
  osThreadDef(UART, StartUartTask, osPriorityBelowNormal, 0, 128);
  uartTaskHandle = osThreadCreate(osThread(UART), NULL);
  
  osThreadDef(MODBUS, StartModBusTask, osPriorityHigh, 0, 512);
  modbusTaskHandle = osThreadCreate(osThread(MODBUS), NULL);
  osThreadSuspend(modbusTaskHandle);
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
    sprintf(sDateTemp, "%s 20%02u/%02u/%02u %02u:%02u:%02u %+.1f [C]\n", 
            weekDays[date.WeekDay],
            date.Year,
            date.Month,
            date.Date,
            time.Hours,
            time.Minutes,
            time.Seconds,
            t);
    LCD_UsrLog(sDateTemp);
  }
  /* USER CODE END StartADCTask */
}

/* USER CODE BEGIN Application */
void StartBTNTask(void const * argument)
{
  uint32_t tikcs;
  char sFooter[31] = "Press BTN to switch | Load 00%";

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
        LCD_DbgLog("USER BTN: simple press\n");
        LCD_DbgLog("*!* ModBus Mode *!*\n");
        
        osThreadSuspend(uartTaskHandle);
        osThreadResume(modbusTaskHandle);
      } else {
        // long press
        // ...
        LCD_UsrLog("USER BTN: long press\n");
        LCD_DbgLog("*!* UART Mode *!*\n");
        
        osThreadSuspend(modbusTaskHandle);
        osThreadResume(uartTaskHandle);
        
        // w8 when button relesed
        while (BSP_PB_GetState(BUTTON_KEY) == SET) {
          osDelay(10);
        } osDelay(100);
      }
    }
    
    sprintf(sFooter,"Press BTN to switch | Load %02u%", osGetCPUUsage());
    sFooter[30] = 0;
    LCD_LOG_SetFooter((uint8_t *)sFooter);
  }
  /* USER CODE END StartADCTask */
}

void StartUartTask(void const * argument)
{
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
    
    strcat(sDateTemp,"\r");
    BSP_COM_Transmit(COM1, sDateTemp);
  }
  /* USER CODE END StartADCTask */
}

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"

/* ----------------------- Defines ------------------------------------------*/
#define REG_INPUT_START   1000
#define REG_INPUT_NREGS   4
#define REG_HOLDING_START 1
#define REG_HOLDING_NREGS 40

/* ----------------------- Static variables ---------------------------------*/
static USHORT   usRegInputBuf[REG_INPUT_NREGS];
static USHORT   usRegHoldingStart = REG_HOLDING_START;
static USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];

void StartModBusTask(void const * argument)
{
  eMBErrorCode eStatus;
  
  portTickType xLastWakeTime;

  /* Select either ASCII or RTU Mode. */
  eStatus = eMBInit( MB_RTU, 0x01, 0, 115200, MB_PAR_NONE );

  /* Enable the Modbus Protocol Stack. */
  eStatus = eMBEnable();
  
  /* Infinite loop */
  for( ;; )
  {
    /* Call the main polling loop of the Modbus protocol stack. */
    ( void )eMBPoll(  );
    /* Application specific actions. Count the number of poll cycles. */
    usRegInputBuf[0]++;
    /* Hold the current FreeRTOS ticks. */
    xLastWakeTime = xTaskGetTickCount(  );
    usRegInputBuf[1] = ( unsigned portSHORT )( xLastWakeTime >> 16UL );
    usRegInputBuf[2] = ( unsigned portSHORT )( xLastWakeTime & 0xFFFFUL );
    /* The constant value. */
    usRegInputBuf[3] = 33;
    
    osDelay(20);
  }
}

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
  eMBErrorCode    eStatus = MB_ENOERR;
  int             iRegIndex;

  if( ( usAddress >= REG_HOLDING_START ) &&
      ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
  {
    iRegIndex = ( int )( usAddress - usRegHoldingStart );
    switch ( eMode )
    {
    /* Pass current register values to the protocol stack. */
    case MB_REG_READ:
      // copy string with RTC and Temperature to the holding buf
      memcpy(usRegHoldingBuf, sDateTemp, REG_HOLDING_NREGS);
      
      while( usNRegs > 0 )
      {
        *pucRegBuffer++ = ( unsigned char )( sDateTemp[iRegIndex++]);
        *pucRegBuffer++ = ( unsigned char )( sDateTemp[iRegIndex++]);
//        *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
//        *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
//        iRegIndex++;
        usNRegs--;
      }
      break;

    /* Update current register values with new values from the protocol stack. */
    case MB_REG_WRITE:
      while( usNRegs > 0 )
      {
        usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
        usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
        iRegIndex++;
        usNRegs--;
      }
    }
  }
  else
  {
    eStatus = MB_ENOREG;
  }
  
  return eStatus;
}

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    return MB_ENOREG;
}

eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{
    return MB_ENOREG;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
