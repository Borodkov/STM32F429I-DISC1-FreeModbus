/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "port.h"

/* ----------------------- Bard specific ----------------------------------*/
extern TIM_HandleTypeDef htim6;

/* ----------------------- Static variables ---------------------------------*/
static uint16_t timeout = 0;
static uint16_t counter = 0;

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timeout50us )
{  
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 79;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 50-1; // 50 us
  
  timeout = usTim1Timeout50us;
  
  return HAL_OK == HAL_TIM_Base_Init(&htim6) ? TRUE : FALSE;
}

void
vMBPortTimersEnable( void )
{
  /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
  counter = 0;
  
  HAL_TIM_Base_Start_IT(&htim6);
}

void
vMBPortTimersDisable( void )
{
  HAL_TIM_Base_Stop_IT(&htim6);
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
static void prvvTIMERExpiredISR( void )
{
    ( void )pxMBPortCBTimerExpired(  );
}

/**
* @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  __HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
  
  /* TIM Update event */
  if (++counter >= timeout)
     prvvTIMERExpiredISR();
}