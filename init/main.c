/* standard include */
#include <stdio.h>
#include <stdint.h>
/* stm32 include */
#include "stm32f10x.h"  
#include "stm32f10x_it.h"
/* freeRTOS include */
#include "FreeRTOS.h"  
#include "task.h" 
#include "queue.h" 
#include "timers.h"
#include "portable.h"  
#include "FreeRTOSConfig.h" 
/* timer exam */
#define mainOne_SHOT_TIMER_PERIOD pdMS_TO_TICKS( 3333 )
#define mainAUTO_RELOAD_TIMER_PERIOD pdMS_TO_TICKS( 500 )
/* queue exam */
QueueHandle_t xMsgQueue;

static void prvSetupHardware( void );
static void prvOneShotTimerCallback( TimerHandle_t xTimer );
static void prvAutoReloadTimerCallback( TimerHandle_t xTimer );

void vTask_A( void *pvParameters )
{
    int16_t sSendNum = 1;
    for (;;) {

        vTaskDelay( 2000/portTICK_PERIOD_MS);
        printf("Sending Num:%d\r\n", sSendNum);
        xQueueSend( xMsgQueue, (void *)&sSendNum, portMAX_DELAY);
        sSendNum++;
    }
}

void vTask_B( void *pvParameters )
{
    int16_t sReceiveNum = 0;
    for (;;) {

        if ( xQueueReceive( xMsgQueue, &sReceiveNum, portMAX_DELAY)) {
            printf("ReceiveNum:%d\r\n", sReceiveNum);
        }
    }
}

void  vLED_1_Task( void *pvParameters ) 
{  
    while( 1 ) {  
        GPIO_ResetBits( GPIOB, GPIO_Pin_0 );
        printf("LED1 ON\r\n");
        vTaskDelay( 1000 / portTICK_RATE_MS ); 
        GPIO_SetBits( GPIOB, GPIO_Pin_0);
        printf("LED1 OFF\r\n");
        vTaskDelay( 2000 / portTICK_RATE_MS ); 
    }  
}

void  vLED_2_Task( void *pvParameters ) 
{  
    while( 1 ) {  
        GPIO_SetBits( GPIOC, GPIO_Pin_0); 
        printf("LED2 OFF\r\n");        
        vTaskDelay( 1000 / portTICK_RATE_MS ); 
        GPIO_ResetBits( GPIOC, GPIO_Pin_0);   
        printf("LED2 ON\r\n");
        vTaskDelay( 1000 / portTICK_RATE_MS );  
    }  
} 

int main(void) 
{  
TimerHandle_t xAutoReloadTimer=NULL, xOneShotTimer=NULL;
BaseType_t xTimer1Started, xTimer2Started;
    /* 初使化硬件平台 */
    prvSetupHardware();

    /* 建立队列 */
    xMsgQueue = xQueueCreate( 5, sizeof(int16_t) );
    printf("befroe create,xone:%d,xAuto:%d",xOneShotTimer!=NULL,(int)xOneShotTimer);
    
    //xTaskCreate( vLED_1_Task, "LED1",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY+1 ,NULL);        
    //xTaskCreate( vLED_2_Task, "LED2",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY+2 ,NULL);
    xOneShotTimer = xTimerCreate(
                    "OneShot",
                    mainOne_SHOT_TIMER_PERIOD,
                    pdFALSE,
                    0,
                    prvOneShotTimerCallback );

    xAutoReloadTimer = xTimerCreate(
                    "AutoReload",
                    mainAUTO_RELOAD_TIMER_PERIOD,
                    pdTRUE,
                    0,
                    prvAutoReloadTimerCallback );

    if ( (xOneShotTimer != NULL) && (xAutoReloadTimer != NULL) )  {
        xTimer1Started = xTimerStart(xOneShotTimer, 0);
        xTimer2Started = xTimerStart(xAutoReloadTimer, 0);

        if ( (xTimer1Started == pdPASS) && (xTimer2Started == pdPASS) ) {
            /* Start the scheduler */
            vTaskStartScheduler();
        }
    }
    // xTaskCreate( vTask_A, "taskA", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL);
    // xTaskCreate( vTask_B, "taskB", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL);
    
    // vTaskStartScheduler();
    
    for( ;; ); 
}

static void prvOneShotTimerCallback( TimerHandle_t xTimer )
{
TickType_t  xTimerNow;
    
    xTimerNow = xTaskGetTickCount();

    printf( "One-short timer callback executing %d\r\n", xTimerNow );

}

static void prvAutoReloadTimerCallback( TimerHandle_t xTimer )
{
TickType_t  xTimerNow;
    
    xTimerNow = xTaskGetTickCount();

    printf( "Auto-Reload timer callback executing %d\r\n", xTimerNow );
    
}


int fputc(int ch, FILE *f)  
{  
    /* 写一个字节到USART1 */  
    USART_SendData(USART1, (uint8_t) ch);  
    /* 等待发送结束 */  
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)  
    {}  
    return ch;  
}

void LED_Config() 
{
    GPIO_InitTypeDef  GPIO_InitStructure;       
    RCC_APB2PeriphClockCmd(  RCC_APB2Periph_GPIOB
                           | RCC_APB2Periph_GPIOC,  
                             ENABLE
                           );      
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;     
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;    
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_Init( GPIOB, &GPIO_InitStructure );    
    GPIO_SetBits( GPIOB, GPIO_Pin_0 );
    
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_Init( GPIOC, &GPIO_InitStructure);      
    GPIO_SetBits( GPIOC, GPIO_Pin_0 ); 
}

void UART1Init() 
{
    GPIO_InitTypeDef GPIO_InitStructure;  
    USART_InitTypeDef USART_InitStructure;  
     
    /* 第1步：打开GPIO和USART时钟 */  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);  
     
    /* 第2步：将USART1 Tx@PA9的GPIO配置为推挽复用模式 */  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
     
    /* 第3步：将USART1 Rx@PA10的GPIO配置为浮空输入模式 */  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
     
    /* 第4步：配置USART1参数 */  
    USART_InitStructure.USART_BaudRate = 115200;  
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  
    USART_Init(USART1, &USART_InitStructure);
     
    /* 第5步：使能 USART1， 配置完毕 */
    USART_Cmd(USART1, ENABLE);
}

static void prvSetupHardware( void ) 
{
    /*
     * STM32中断优先级分组为4，即4bit都用来表示抢占优先级，范围为：0~15
     * 优先级分组只需要分组一次即可，以后如果有其他的任务需要用到中断，
     * 都统一用这个优先级分组，千万不要再分组，切忌。
     */
    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
    LED_Config();
    UART1Init();
}
