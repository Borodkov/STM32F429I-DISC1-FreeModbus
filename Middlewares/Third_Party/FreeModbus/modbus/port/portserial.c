/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"

/* ----------------------- Static variables ---------------------------------*/
UCHAR ucGIEWasEnabled = FALSE;
UCHAR ucCriticalNesting = 0x00;

/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
//    ENTER_CRITICAL_SECTION(  );
//    if( xRxEnable )
//    {
//        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
//    }
//    else
//    {
//       USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
//    }
//    if( xTxEnable )
//    {
//       USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
//    }
//    else
//    {
//       USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
//    }
//    EXIT_CRITICAL_SECTION(  );
}

BOOL
xMBPortSerialInit( UCHAR ucPort, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
	BOOL    bInitialized = TRUE;
//	GPIO_InitTypeDef GPIO_InitStruct; 
//	USART_InitTypeDef USART_InitStructure;
//
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD
//		| RCC_APB2Periph_AFIO, ENABLE);
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA,&GPIO_InitStruct);
//
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
//	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA,&GPIO_InitStruct);
//
//	USART_InitStructure.USART_BaudRate = ulBaudRate;
//	switch ( eParity )
//	{
//	case MB_PAR_NONE:
//		USART_InitStructure.USART_Parity = USART_Parity_No;
//		break;
//	case MB_PAR_ODD:
//		USART_InitStructure.USART_Parity = USART_Parity_Odd;
//		break;
//	case MB_PAR_EVEN:
//		USART_InitStructure.USART_Parity = USART_Parity_Even;
//		break;
//	}
//	switch ( ucDataBits )
//	{
//	case 8:
//		if(eParity==MB_PAR_NONE)
//		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//		else
//		USART_InitStructure.USART_WordLength = USART_WordLength_9b;
//		break;
//	case 7:
//		break;
//	default:
//		bInitialized = FALSE;
//	}
//	if( bInitialized )
//	{
//		ENTER_CRITICAL_SECTION(  );
//		USART_InitStructure.USART_StopBits = USART_StopBits_1;
//		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//		USART_Init(USART1, &USART_InitStructure);
//		USART_Cmd(USART1, ENABLE);
//		EXIT_CRITICAL_SECTION(  );
//	}
	return bInitialized;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
//	USART_SendData(USART1, ucByte);
//	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);  
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
//	USART_ClearFlag(USART1, USART_IT_RXNE) ;				 
////	while(USART_GetFlagStatus(USART1, USART_IT_RXNE) == 0);
//    *pucByte = (u8)USART_ReceiveData(USART1);
    return TRUE;
}

void UART1_IRQ(void)
{
//
//	if(USART_GetITStatus(USART1,USART_IT_TXE))
//	{
//	    pxMBFrameCBTransmitterEmpty(  );
//	}
//	else if(USART_GetITStatus(USART1,USART_IT_RXNE))
//	{
//		pxMBFrameCBByteReceived(  );
//	}
}
