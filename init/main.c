#include "stm32f10x.h"  
#include "stm32f10x_it.h"
#include "FreeRTOS.h"  
#include "task.h" 
#include "queue.h" 
#include "list.h"  
#include "portable.h"  
#include "FreeRTOSConfig.h" 


void LED_config() {
    GPIO_InitTypeDef  GPIO_InitStructure;       
    RCC_APB2PeriphClockCmd(  RCC_APB2Periph_GPIOA
                           | RCC_APB2Periph_GPIOD,  
                             ENABLE
                           );      
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;     
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;    
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_Init( GPIOA, &GPIO_InitStructure );    
    GPIO_SetBits( GPIOA, GPIO_Pin_8 );
    
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_Init( GPIOD, &GPIO_InitStructure);      
    GPIO_SetBits( GPIOD, GPIO_Pin_2 ); 
}

void  vLED_1_Task( void *pvParameters ) {  
    while( 1 ) {  
        GPIO_ResetBits( GPIOA, GPIO_Pin_8 );  
        vTaskDelay( 1000 / portTICK_RATE_MS );   
        GPIO_SetBits( GPIOA, GPIO_Pin_8);    
        vTaskDelay( 2000 / portTICK_RATE_MS );  
    }  
}

void  vLED_2_Task( void *pvParameters ) {  
    while( 1 ) {  
        GPIO_SetBits( GPIOD, GPIO_Pin_2); 
        vTaskDelay( 1000 / portTICK_RATE_MS );   
        GPIO_ResetBits( GPIOD, GPIO_Pin_2);   
        vTaskDelay( 1000 / portTICK_RATE_MS );  
    }  
} 

int main(void) {  
    
    LED_config();  
    
    xTaskCreate( vLED_1_Task, "LED1",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY+1 ,NULL);        
    xTaskCreate( vLED_2_Task, "LED2",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY+2 ,NULL);
    
    vTaskStartScheduler();
    
    return 0;
    
}
