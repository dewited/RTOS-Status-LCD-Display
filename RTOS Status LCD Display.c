//
//
//   EECE 437 Spring 2014
//   Lab #5 Template File (FreeRTOS)
 //

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx.h"
#include <stdio.h>
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_ioe.h"
#include "stm32f429i_discovery_l3gd20.h"
#include <string.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#define ABS(x)  (x < 0) ? (-x) : x
#define y_max   320
#define led_delay_1     100
#define led_delay_2     25
#define led_delay_3     13

// Private globals
static __IO uint32_t TimingDelay = 0;
RCC_ClocksTypeDef RCC_Clocks;

void Display_Init(void);
void TP_Config(void);
void LCD_ClearSection(uint16_t, uint16_t);

// Timing methods
void Delay(uint32_t);
void TimingDelay_Decrement(void);

// FreeRTOS task entry points
void Task1(void*);
void Task2(void*);
void Task3(void*);
void DisplayTask(void*);

// FreeRTOS ISR methods
void xPortSysTickHandler();
void vPortSVCHandler();
void xPortPendSVHandler();

//own variables

static int runningTask=3;
  xTaskHandle test, display, task1, task2, task3;

int main(void)
{

  GPIO_InitTypeDef GPIO_InitStructure;
    
    /* Enable the BUTTON Clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* Configure Button pin as Output */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Force capacity to be charged quickly */
  GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
    
  /* Check whether the test mode should be started */
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO); 
    
  /* Initialize LEDs, User Button and LCD on STM32F429I-Disco */
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);
  

  // LCD initialization
  LCD_Init();

  // LCD Layer initialization
  LCD_LayerInit();

  // Enable the LTDC
  LTDC_Cmd(ENABLE);

  // Set LCD foreground layer
  LCD_SetLayer(LCD_FOREGROUND_LAYER);

 

  // Touch Panel configuration
  TP_Config();

  Display_Init();
  

  // Create some tasks
  xTaskCreate(&Task1, "Task1 ", 64, NULL, 4, &task1);
  xTaskCreate(&Task2, "Task2 ", 64, NULL, 3, &task2);
  xTaskCreate(&Task3, "Task3 ", 64, NULL, 2, &task3);
  xTaskCreate(&DisplayTask, "DisplayTask", 512, NULL, 10, &display);

  // Start the scheduler
  vTaskStartScheduler();
}

void Task1(void* x)
{
 int j;
 int i;
  while (1) {
  
    //Delay(100);                 // Busy wait....
    
    runningTask=1;//flag is changed by every function. lets the display function know which one is running
    j = 0;
    i = xTaskGetTickCount();//gets current clock tick
    STM_EVAL_LEDOn(LED4);//turns on LED 4
    //GPIO_ToggleBits(GPIOG, 13);
    while(j<700)//keeps the function going for a set time
    {
      j=xTaskGetTickCount()-i;
    }
    /*vTaskDelay(1000);
    STM_EVAL_LEDOff(LED4);
    vTaskDelay(1000);*/
    
    vTaskDelay(400);            // RTOS wait - to make task block


  }
  
  
  
}
void Task2(void* x)
{
 int j;
 int i;
 
  while (1) {
  
    //Delay(1);                 // Busy wait....
    
    runningTask=2;
   // STM_EVAL_LEDOff(LED4);
    j = 0;
    i = xTaskGetTickCount();
    while(j<600)
    {
      
      if(runningTask!=2)
      {
        i=xTaskGetTickCount()-j;
        runningTask=2;
      }
      
      j=xTaskGetTickCount()-i;
    }
    
    vTaskDelay(400);            // RTOS wait - to make task block
  
  
 } 
}
void Task3(void* x)
{
 int j;
 int i;
 
  while (1) {
  
    //Delay(1);                 // Busy wait....
    
    runningTask=3;
    
   /* STM_EVAL_LEDOn(LED3);
    vTaskDelay(1000);
    STM_EVAL_LEDOff(LED3);
    vTaskDelay(1000);*/
    j = 0;
    i = xTaskGetTickCount();
    while(j<200)
    {
      if(runningTask!=3)
      {
        i=xTaskGetTickCount()-j;
        runningTask=3;
      }     
      j=xTaskGetTickCount()-i;
    }
    vTaskDelay(800);            // RTOS wait - to make task block
    

  }
  
  
  
}

void DisplayTask(void* x)  {

  static TP_STATE* TP_State;
  static int y_index = 0;
  static int led = 0;

  while (1)   {
    led++;
    // Get touch status
    TP_State = IOE_TP_GetState();
    // Forcibly clear the IT pending bit
    IOE_ClearGITPending(IOE_GIT_TOUCH);
    // Check for touch events
    if(TP_State->TouchDetected)  {
     
          LCD_SetTextColor(LCD_COLOR_BLUE);
          LCD_DrawFullCircle(TP_State->X, TP_State->Y, 4);
      
    }
    
    if(runningTask==1)
    {
      if (led>=led_delay_1)
      {
        STM_EVAL_LEDToggle(LED4);
        led=0;
      }
      LCD_SetTextColor(LCD_COLOR_RED);
      LCD_DrawFullRect(172, y_index, 16, 2);
      
      if(eTaskGetState(task3)==eBlocked)
      {
       LCD_SetTextColor(LCD_COLOR_BLUE);
       LCD_DrawFullRect(58, y_index, 4, 1);
      }
      else
      {
       LCD_SetTextColor(LCD_COLOR_BLUE);
       LCD_DrawFullRect(56, y_index, 8, 2);
      }
      if(eTaskGetState(task2)==eBlocked)
      {
       LCD_SetTextColor(LCD_COLOR_GREEN);
       LCD_DrawFullRect(118, y_index, 4, 1);
      }
      else
      {
       LCD_SetTextColor(LCD_COLOR_GREEN);
       LCD_DrawFullRect(116, y_index, 8, 2);
      }
     }//end of if statement
    
    else if(runningTask==2)
    {
      if (led>=led_delay_2)
      {
        STM_EVAL_LEDToggle(LED4);;
        led=0;
      }
      LCD_SetTextColor(LCD_COLOR_GREEN);
      LCD_DrawFullRect(112, y_index, 16, 2);
      
      if(eTaskGetState(task3)==eReady)
      {
       LCD_SetTextColor(LCD_COLOR_BLUE);
       LCD_DrawFullRect(56, y_index, 8, 2);
      }
     else
      {
       LCD_SetTextColor(LCD_COLOR_BLUE);
       LCD_DrawFullRect(58, y_index, 4, 1);
      }
      if(eTaskGetState(task1)==eReady)
      {
       LCD_SetTextColor(LCD_COLOR_RED);
       LCD_DrawFullRect(176, y_index, 8, 2);
      }
      else
      {
       LCD_SetTextColor(LCD_COLOR_RED);
       LCD_DrawFullRect(178, y_index, 4, 1);
      }
     }//end of if statement
    else if(runningTask==3)
    {
      if (led>=led_delay_3)
      {
        STM_EVAL_LEDToggle(LED4);
        led=0;
      }
      LCD_SetTextColor(LCD_COLOR_BLUE);
      LCD_DrawFullRect(52, y_index, 16, 2);
      
      if(eTaskGetState(task2)==eReady)
      {
       LCD_SetTextColor(LCD_COLOR_GREEN);
       LCD_DrawFullRect(116, y_index, 8, 2);
      }
     else
      {
       LCD_SetTextColor(LCD_COLOR_GREEN);
       LCD_DrawFullRect(118, y_index, 4, 1);
      }
      if(eTaskGetState(task1)==eReady)
      {
       LCD_SetTextColor(LCD_COLOR_RED);
       LCD_DrawFullRect(176, y_index, 8, 2);
      }
      else
      {
       LCD_SetTextColor(LCD_COLOR_RED);
       LCD_DrawFullRect(178, y_index, 4, 1);
      }
     }//end of if statement
    
    
     LCD_SetTextColor(LCD_COLOR_BLACK);
     //LCD_DrawFullRect(118, y_index++, 4, 4);
   
   y_index=y_index+2;
   
    
    if(y_index > y_max)//resests display when the end is reached.
    {
      y_index = 0;
      Display_Init();
    }

  vTaskDelay(10);//change to change refresh rate of graph
  }
}



 void Display_Init(void)
{
    /* Clear the LCD */
    LCD_Clear(LCD_COLOR_WHITE);

    //LCD_SetFont(&Font16x24);
    //LCD_DisplayStringLine(LINE(1), (uint8_t*)"EECE 437 S14");
    LCD_DrawLine(60, 0, 320, LCD_DIR_VERTICAL);
    LCD_DrawLine(120, 0, 320, LCD_DIR_VERTICAL);
    LCD_DrawLine(180, 0, 320, LCD_DIR_VERTICAL);


    //LCD_SetTextColor(LCD_COLOR_RED);
    //LCD_DrawFullRect(176, 2, 8, 40);

    
 
}



// Supplied Functions - Do not change //////////////////////////////////////////

void SysTick_Handler()
{
  xPortSysTickHandler();
}

void SVC_Handler()
{
  vPortSVCHandler();
}

void PendSV_Handler()
{
  xPortPendSVHandler();
}

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
  while (1) asm("BKPT #0");
}

void LCD_ClearSection(uint16_t yMin, uint16_t yMax)
{
  // Changed to leverage the 2D DMA engine
  LCD_SetTextColor(LCD_COLOR_WHITE);
  LCD_DrawFullRect(0, (yMin < yMax) ? yMin : yMax, 240, ABS(yMax - yMin) + 1);
}

void TP_Config(void)
{
  /* Clear the LCD */
  LCD_Clear(LCD_COLOR_WHITE);

  /* Configure the IO Expander */
  if (IOE_Config() == IOE_OK)
  {
    LCD_SetFont(&Font16x24);
    LCD_DisplayStringLine(LINE(4), (uint8_t*)" EECE 437 S14");
    LCD_DisplayStringLine(LINE(5), (uint8_t*)"  Hit Button");
    LCD_DisplayStringLine(LINE(6), (uint8_t*)"   to Start");
  }
  else
  {
    LCD_Clear(LCD_COLOR_RED);
    LCD_SetTextColor(LCD_COLOR_BLACK);
    LCD_DisplayStringLine(LCD_LINE_6, (uint8_t*)"    IOE NOT OK     ");
    LCD_DisplayStringLine(LCD_LINE_7, (uint8_t*)"  Reset the board  ");
    LCD_DisplayStringLine(LCD_LINE_8, (uint8_t*)"   and try again   ");
  }
  
  /* Wait for User button to be pressed */
  while (STM_EVAL_PBGetState(BUTTON_USER) != Bit_SET)
  {}
  /* Wait for User button is released */
  while (STM_EVAL_PBGetState(BUTTON_USER) != Bit_RESET)
  {}
  
}



void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
    TimingDelay--;

}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
