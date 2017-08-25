/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"

extern TIM_HandleTypeDef htim6;

static uint16_t timeout = 0;
static uint16_t downcounter = 0;

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timeout50us )
{  
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 79;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 50-1;
  
  timeout = usTim1Timeout50us;
  
  return HAL_OK == HAL_TIM_Base_Init(&htim6) ? TRUE : FALSE;
}

void
vMBPortTimersEnable( void )
{
  /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
  downcounter = timeout;
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

void TIM6_DAC1_IRQHandler(void) {
//  /* TIM Update event */
//  if(__HAL_TIM_GET_FLAG(&htim, TIM_FLAG_UPDATE) != RESET && __HAL_TIM_GET_IT_SOURCE(&htim, TIM_IT_UPDATE) !=RESET) {
//    __HAL_TIM_CLEAR_IT(&htim, TIM_IT_UPDATE);
    if (!--downcounter)
            prvvTIMERExpiredISR();
//  }
}
