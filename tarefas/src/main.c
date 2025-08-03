#include "system_tm4c1294.h" // CMSIS-Core
#include "driverleds.h" // device drivers
#include "cmsis_os2.h" // CMSIS-RTOS
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h" // driverlib
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "uartstdio.h"
#include "inc/tm4c1294ncpdt.h" // CMSIS-Core
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/debug.h"


osThreadId_t thread_calcFreq_id, thread_uart_id, thread_botao_id;
 
typedef struct {
  uint16_t freq; 
  uint16_t escala;
}data;

void timerInterrupHandler()
{
  TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  osThreadFlagsSet(thread_calcFreq_id, 0x0001);
}

void thread_calcFreq(void *arg)
{
  data *medicao = (data *) arg;
  volatile uint32_t med = 0;
  while(1)
  {
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
    TimerIntDisable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    medicao->freq = HWREG(TIMER0_BASE + TIMER_O_TAR);
    med++;
    if(med >= medicao->escala)
    {
      osThreadFlagsSet(thread_uart_id, 0x0010);
      med = 0;
    }
    HWREG(TIMER0_BASE + TIMER_O_TAV) = 0;
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  }
}

void thread_uart (void* arg)
{
  data *medicao = (data *) arg;
  while(1)
  {
    osThreadFlagsWait(0x0010, osFlagsWaitAny, osWaitForever);
    if (medicao->escala == 1)
      UARTprintf("%i Hz\n",medicao->freq);
    else
      UARTprintf("%i kHz\n",medicao->freq);
  }
}

void thread_botao (void* arg)
{
  data *medicao = (data *) arg;
  static uint32_t debounce;
  while(1){
    osDelay(1);
    debounce = (debounce << 1) | GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0);
    if(debounce == 0)
    {
      if(medicao->escala == 1000){
          TimerLoadSet(TIMER1_BASE, TIMER_A, 120000000);
        medicao->escala = 1;
      }else if(medicao->escala == 1){
          TimerLoadSet(TIMER1_BASE, TIMER_A, 120000);
          medicao->escala = 1000;
      }
    }
  }
}  



void main(void){
  data medicao;
  medicao.escala = 1;
  SystemInit();
  
  uint32_t ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                              SYSCTL_OSC_MAIN |
                                                SYSCTL_USE_PLL |
                                                  SYSCTL_CFG_VCO_480),
  120000000); // PLL em 24MHz
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0);
  GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); // Habilita GPIO J (push-button SW1 = PJ0)
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)); // Aguarda final da habilitação
  GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0); // push-button SW1 como entrada
  GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
  UARTStdioConfig(0, 115200, 16000000);
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));
  TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_COUNT_UP);
  TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
  TimerPrescaleSet(TIMER0_BASE, TIMER_A, 1);
  TimerUpdateMode(TIMER0_BASE, TIMER_A, TIMER_UP_LOAD_IMMEDIATE);
  TimerEnable(TIMER0_BASE, TIMER_A);
  
  //
  // Enable pin PL4 for TIMER0 T0CCP0
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL));
  GPIOPinConfigure(GPIO_PL4_T0CCP0);
  GPIOPinTypeTimer(GPIO_PORTL_BASE, GPIO_PIN_4);
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
  TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
  TimerPrescaleSet(TIMER1_BASE, TIMER_A, 0);
  TimerLoadSet(TIMER1_BASE, TIMER_A, 120000000);
  TimerEnable(TIMER1_BASE, TIMER_A);
  
  TimerIntRegister(TIMER1_BASE, TIMER_A, &timerInterrupHandler);
  TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  
  
  LEDInit(LED1|LED2|LED3|LED4);
  

  osKernelInitialize();

  thread_calcFreq_id = osThreadNew(thread_calcFreq, (void *) &medicao, NULL);
  thread_uart_id = osThreadNew(thread_uart, (void *) &medicao, NULL);
  thread_botao_id = osThreadNew(thread_botao, (void *) &medicao, NULL);

  if(osKernelGetState() == osKernelReady)
    osKernelStart();

  while(1);
  
} // main
