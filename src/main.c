#include <string.h>


/*-------------------------------------------------------------------------
                     FreeRTOS includes
-------------------------------------------------------------------------*/
#include "../dependencies/FreeRTOS/FreeRTOS.h"
#include "../dependencies/FreeRTOS/task.h"
#include "../dependencies/FreeRTOS/semphr.h"

#include "mcuio.h"
#include "miniprintf.h"

/*-------------------------------------------------------------------------
                     libopencm3 includes
-------------------------------------------------------------------------*/
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h> /* the interrupts functions */

#define USE_USB 0 /* set 1 to use a USB to I/O */

static TaskHandle_t h_task2 = 0, h_task3 = 0;
static SemaphoreHandle_t h_mutex;

static volatile unsigned
   rtc_isr_count = 0u, /* times the rtc_isr function is called */
   rtc_alarm_count = 0u, /* times alarm detected */
   rtc_overflow_count = 0u; /* times overflow ocurred */

static volatile unsigned
   days = 0,
   hours = 0, minutes = 0, seconds = 0,
   alarm = 0; /* alarm flag */

/*-------------------------------------------------------------------------
                     lock mutex
-------------------------------------------------------------------------*/
static void
mutex_lock(void) {
   xSemaphoreTake(h_mutex,portMAX_DELAY);
}

/*-------------------------------------------------------------------------
                  unlock mutex
-------------------------------------------------------------------------*/
static void
mutex_unlock(void) {
   xSemaphoreGive(h_mutex);
}

/*-------------------------------------------------------------------------
                  RTC Interrupt Service Routine
-------------------------------------------------------------------------*/
void
rtc_isr(void) {
   UBaseType_t intstatus;
   BaseType_t woken = pdFALSE;
   ++rtc_isr_count;

   if (rtc_check_flag(RTC_OW)) {
      /* time overflowed */
      ++rtc_overflow_count;
      rtc_clear_flag(RTC_OW);
   }

   if (rtc_check_flag(RTC_SEC)) {
      rtc_clear_flag(RTC_SEC);

      /* incrment the time */
      intstatus = taskENTER_CRITICAL_FROM_ISR();
      if (++seconds >= 60) {
         ++minutes;
         seconds -= 60;
      }
      if (minutes >= 60) {
         ++hours;
         minutes -= 60;
      }
      if (hours >= 24) {
         ++days;
         hours -= 24;
      }
      taskEXIT_CRITICAL_FROM_ISR(intstatus);

      /* wake the task2 if we can */
      vTaskNotifyGiveFromISR(h_task2,&woken);
      portYIELD_FROM_ISR(woken);
   }
}

/*-------------------------------------------------------------------------
                     RTC alarm ISR
-------------------------------------------------------------------------*/
void
rtc_alarm_isr(void) {
   BaseType_t woken = pdFALSE;

   ++rtc_alarm_count;
   exti_reset_request(EXTI17);
   rtc_clear_flag(RTC_ALR);

   vTaskNotifyGiveFromISR(h_task3,&woken);
   portYIELD_FROM_ISR(woken);
}

/*-------------------------------------------------------------------------
                  set an alarm n seconds into the future
-------------------------------------------------------------------------*/
static void
set_alarm(unsigned secs) {
   alarm = (rtc_get_counter_val() + secs) & 0xFFFFFFFF;

   rtc_disable_alarm();
   rtc_set_alarm_time(rtc_get_counter_val()+10);
   rtc_enable_alarm();
}

/*-------------------------------------------------------------------------
                  task3: alarm
-------------------------------------------------------------------------*/
static void
task3(void *args __attribute__((unused))) {

   while(1) {
      /* block execution until notified */
      ulTaskNotifyTake(pdTRUE,portMAX_DELAY);		

      mutex_lock();
      std_printf("*** ALARM *** at %3u days %02u:%02u:%02u\n",
      days,hours,minutes,seconds);
      mutex_unlock();
   }
}

/*-------------------------------------------------------------------------
                     task2: toggle a LED and report time
-------------------------------------------------------------------------*/
static void
task2(void *args __attribute__((unused))) {

   while(1) {
   /* block execution until notified */
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);		

    /* toggle LED */
   gpio_toggle(GPIOC,GPIO13);

   mutex_lock();
   std_printf("Time: %3u days %02u:%02u:%02u isr_count: %u, alarms: %u, overflows: %u\n",
      days,hours,minutes,seconds,
      rtc_isr_count,rtc_alarm_count,rtc_overflow_count);
      mutex_unlock();
   }
}

/*-------------------------------------------------------------------------
               initialize RTC for interrupt processing
-------------------------------------------------------------------------*/
static void
rtc_setup(void) {

   rcc_enable_rtc_clock();
   rtc_interrupt_disable(RTC_SEC);
   rtc_interrupt_disable(RTC_ALR);
   rtc_interrupt_disable(RTC_OW);

   /* RCC_HSE, RCC_LSE, RCC_LSI */
   rtc_awake_from_off(RCC_HSE);
   rtc_set_prescale_val(62500);

   /* initialize a 16 counts to overflow */
   rtc_set_counter_val(0xFFFFFFF0);

   exti_set_trigger(EXTI17,EXTI_TRIGGER_RISING);
   exti_enable_request(EXTI17);
   nvic_enable_irq(NVIC_RTC_ALARM_IRQ);
   nvic_enable_irq(NVIC_RTC_IRQ);

   cm_disable_interrupts();
   rtc_clear_flag(RTC_SEC);
   rtc_clear_flag(RTC_ALR);
   rtc_clear_flag(RTC_OW);
   rtc_interrupt_enable(RTC_SEC);
   rtc_interrupt_enable(RTC_ALR);
   rtc_interrupt_enable(RTC_OW);
   cm_enable_interrupts();
}

/*-------------------------------------------------------------------------
         wait until the user presses a key on the terminal program
-------------------------------------------------------------------------*/
static void
wait_terminal(void) {
   TickType_t ticks0, ticks;

   ticks0 = xTaskGetTickCount();

   while(1) {
      ticks = xTaskGetTickCount();
      if ( ticks - ticks0 > 2000 ) { /* every 2 seconds */
         std_printf("Press any key to start...\n");
         ticks0 = ticks;
      }
      if (std_peek() >= 1) { /* key data pending ??? */
         while (std_peek() >= 1) {
            std_getc(); /* eat pending chars */
         }
         return;
      }
      taskYIELD(); /* give a CPU */
   }
}

/*-------------------------------------------------------------------------
                  task1: the user console task
-------------------------------------------------------------------------*/
static void
task1(void *args __attribute__((unused))) {
   char ch;

   wait_terminal();
   std_printf("Started!\n\n");

   rtc_setup(); /* start a RTC interrupt */
   taskYIELD();

   while(1) {
      mutex_lock();
      std_printf("\nPress 'A' to set 10 second alarm,\n"
         "else any key to read time.\n\n");
      mutex_unlock();

      ch = std_getc();

      if (ch == 'a' || ch == 'A') {
         mutex_lock();
         std_printf("\nAlarm configured for 10 seconds from now.\n");
         mutex_unlock();
         set_alarm(10u);
      }
   }
}

/*-------------------------------------------------------------------------
                           main routine: startup
  -------------------------------------------------------------------------*/
int
main(void) {

   rcc_clock_setup_in_hse_8mhz_out_72mhz(); /* use this for the blue-pill */

   rcc_periph_clock_enable(RCC_GPIOC);
   gpio_set_mode(GPIOC,GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,GPIO13);

   h_mutex = xSemaphoreCreateMutex();
   xTaskCreate(task1,"task1",350,NULL,1,NULL);
   xTaskCreate(task2,"task2",400,NULL,3,&h_task2);
   xTaskCreate(task3,"task3",400,NULL,3,&h_task3);

   gpio_clear(GPIOC,GPIO13);

#if USE_USB
   usb_start(1,1);
   std_set_device(mcu_usb); /* use the USB for I/O */
#else
   rcc_periph_clock_enable(RCC_GPIOA) /* TX = A9, RX = A10, CTS = A11, RTS = A12 */
   rcc_periph_clock_enable(RCC_USART1);

   gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO9|GPIO11);
   gpio_set_mode(GPIOA,GPIO_MODE_INPUT,
      GPIO_CNF_INPUT_FLOAT,GPIO10|GPIO12);
   open_uart(1,115200,"8N1","rw",1,1) /* UART1 whith RTS/CTS flow control */
   // open_uart(1,9600,"8N1","rw",0,0) /* UART1@115200 baud rate with no flow control */
   std_set_device(mcu_uart1) /* use UART1 for std I/O */
#endif

   vTaskStartScheduler();
   while(1) {

   }
   return 0;
}
