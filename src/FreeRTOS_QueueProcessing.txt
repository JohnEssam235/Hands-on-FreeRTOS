/**

  *******************************************************************************/
#include <stdio.h>
#include <stdint.h>  // dy el library el feha el standard types bta3tna ...el hya uint8_t w uint32_t w hakza.............
#include <string.h>  // include it to use the memset() function..
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"  // this is the library which provides the usage of the Task creation API .........
#include "queue.h"
#include "timers.h"

/* struct for the commands....................*/
typedef struct command
{
	// number of command the user will send....
	uint8_t number;
	// lw fe ay associated arguments m3 el command ...htt7at f el array dy...
	uint8_t args[10];
}app_cmd;


/* prototypes...........*/
void print_msg(char * s);
void prvSetupHardware();
static void prvSetupUart();
static void prvSetupGPIO();
void rtos_delay(uint32_t delay_in_ms);


/* prototypes for task handlers...*/
void vTask1_MenuDisplay(void * para);
void vTask2_CmdHandling(void * para);
void vTask3_CmdProcessing(void * para);
void vTask4_UartWrite(void * para);



/* helper functions prototypes..*/
uint8_t getCmd(uint8_t * buffer);
void SetArgs(uint8_t * a);

//prototypes command helper functions
void make_led_on(void);
void make_led_off(void);
void led_toggle_start(uint32_t duration);
void led_toggle_stop(void);
void read_led_status(char *task_msg);
void read_rtc_info(char *task_msg);
void print_error_message(char *task_msg);

//Software timer callback function prototype
void led_toggle( TimerHandle_t xTimer );

//softwar timer handler
TimerHandle_t led_timer_handle = NULL;

/* variables for task_handle...*/
TaskHandle_t task1_handle = NULL;
TaskHandle_t task2_handle = NULL;
TaskHandle_t task3_handle = NULL;
TaskHandle_t task4_handle = NULL;


/* variables for queue handles............*/
QueueHandle_t Cmd_Queue = NULL;
QueueHandle_t Uart_Write_Queue = NULL;


/* global vars.....*/
char user_msg[250]= {0}; // to print messages with UART.......
uint8_t cmd_buffer[20]; // 34an a5zn feh el commands bta3ty fe el UART ISR ............w el size 20 eshta 3ady bra7tak ...shof enta htst2bel kam command msln.......
uint8_t len = 0; // da 34an el index bta3 el buffer..

//This is the menu........kol 7aga el user mmkn y3mlha bl command bta3ha..........
char menu[]={"\
\r\nLED_ON             ----> 1 \
\r\nLED_OFF            ----> 2 \
\r\nLED_TOGGLE         ----> 3 \
\r\nLED_TOGGLE_OFF     ----> 4 \
\r\nLED_READ_STATUS    ----> 5 \
\r\nRTC_PRINT_DATETIME ----> 6 \
\r\nEXIT_APP           ----> 0 \
\r\nType your option here : "};


// command numbers...................................

#define LED_ON_COMMAND 			1
#define LED_OFF_COMMAND 		2
#define LED_TOGGLE_COMMAND 		3
#define LED_TOGGLE_STOP_COMMAND 		4
#define LED_READ_STATUS_COMMAND 		5
#define RTC_READ_DATE_TIME_COMMAND		6



int main(void)
{



	/* 1) make PLL off*/
   RCC_DeInit(); /* this function makes PLL off ..... so the CPU clock freq. returns to default(16MHZ).....*/

   /* 2) update the system clock */
   SystemCoreClockUpdate(); /* this function updates the system clock....*/

   /* setup the hardware function....if i will use the uart not the semi hosting....*/
   prvSetupHardware();  // el function el bt initialize el hardware..



/* creating queues..........................................................................................*/
   // first we will create the command queue.........


   // we will create a queue of 10 elements and each element of size app_cmd defined structure ...which is a structure for each command....
   // kan mmkn a3mlha kda ...........xQueueCreate(10,sizeof(app_cmd)); ... kda el reserved memory 110 bytes(10*11)...la2n el struct b 11 bytes w homa 10 elements..
   // el a7san b2a eny a3ml 10 elements ...each element is a pointer to that structure.....
   // fa hya hya bzabt ana bs 3mlt kda 34an awfr memeory......
    Cmd_Queue = xQueueCreate(10,sizeof(app_cmd *));
   // fa el size bta3 el pointer by3tmd 3al architecture enta fahem.....
   // fa hena el arch. bta3 el " ARM " da 32-bit................................
   // fa kda el memory reserved b2et (4bytes*10) y3ny 40 bytes kolo..........badal ma kan 110 bytes.....
   // fa ana wafart fe el heap gedan ......w hya hya nafs el ma3na...3mlt reserve l struct aw 3mlt reserve l pointer to these structs.. ;).....


    // second we will create the write queue........
    // el write queue dy b5zen feha el commands ...y3ny uint8_t variables aw char variables y3ny...
    Uart_Write_Queue = xQueueCreate(10,sizeof(char *));


    // check the queue handle ...to know if there was a sufficient heap memory or not....
    if((Cmd_Queue == NULL) || (Uart_Write_Queue == NULL))
    {
    	sprintf(user_msg,"queue creation failed");
    	print_msg(user_msg); // bl UART.............
    }
    else
    {
    	// lw el queues at3mlha creation sa7 abda2 b2a a3ml el tasks w start scheduler........
    	// 34an m3mel4 el tasks mn 3'er ma el queues mtkon4 at3mlt aslun......

    	   /* creating tasks........................................................................................*/
    	// menu display task...
    	xTaskCreate(vTask1_MenuDisplay,"Task1",500,NULL,1,&task1_handle); // leh el menu task el priority 1 w el ba2y 2 ??? shof video 16 f noso keda l7d a5ro b2a....

    	// command handling task.....
    	xTaskCreate(vTask2_CmdHandling,"Task2",500,NULL,2,&task2_handle);

    	// command processing task.....
    	xTaskCreate(vTask3_CmdProcessing,"Task3",500,NULL,2,&task3_handle);

    	// Uart write task.....
    	xTaskCreate(vTask4_UartWrite,"Task4",500,NULL,2,&task4_handle);

    	/* 4)..............scheduler function*/
    	vTaskStartScheduler();

    }



   for(;;);

}



void vTask1_MenuDisplay(void * para)
 {

	char * p = menu; //hst3mlha b3den...........

    /* dy bt3ml eih 2olna ??
     * dy btb3at el menu el ana m3rfha fo2 dy ll uart_write_queue .... w b3d kda el uart_write_queue tb3at ll
     * uart_write_task ......el hya b2a btb3at ll UART.......
     *
     * fa howa el menu_display dy htb3at el menu b2a ll uart_write_queue.............
     * fa hst3ml el API el bt3ml send ll queue..............
     */
    while(1)
    {
         xQueueSend(Uart_Write_Queue,&p,portMAX_DELAY); // el portMAX_DELAY da m3nah eno hy-wait to the infinity l7d ma el queue yefda b2a....
         // fa ana ba3at el menu ll uart_write_queue..........

         /* b3d kda b2a h-wait l7d ma ygely notification.........
          * l7d ma el user y-send el command b2a........
          *
          * y3ny enta httb3lo el menu tamam.....howa hy5tar b2a ......
          * fa b3d lama howa y5tar el mafrod ttb3lo el menu tany .....34an y5tar command tany w hakza.....
          * tb ana b2a h3raf mnen eno da5al command 5las ??????
          * fa 34an kda hstna ygely notification mn el UART interrupt handler(ISR)
          * la2n howa lama byd5l data by7sl interrupt ....fa fe el ISR lazm b2a awl ma t4t3'l ...ab3at notification
          * ll MenuDisplay function dy 34an a2olha atb3y tany b2a howa 5las da5al command.....
          */
         xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
    }
}

void vTask2_CmdHandling(void * para)
  {

	 uint8_t cmd = 0;

	 app_cmd * new_cmd ; // pointer no3o nafs no3 el struct......w asmo new_cmd

      while(1)
      {
           /* el command handling task el mafrod el awl tb2a blocked .... w tstna ygelha notification 34an t3raf
            * en el user da5al command 5las f3lan..
            */
    	  xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
    	  /* fa awl ma tglha el notification tbda2 ta5od el command mn el buffer ely el UART interrupt handler
    	   * 7at el command feh......
    	   */

    	  // h3ml el awl create ll command dy ...w h3mlha el struct bta3tha ...create command as struct dynamically..
          new_cmd = (app_cmd *) pvPortMalloc(sizeof(app_cmd)); // 3mlt create tamam...

          /* Mohm awe....................................................................................
           * lazm a7afez 3al shared resource hena el homa el buffer w el array of struct pointers dol.....
           */

          taskENTER_CRITICAL();
          // hena b2a ha5od el command mn el buffer aho.....................
         cmd = getCmd(cmd_buffer); // getCmd dy helper function h3mlha implementation ana btgbly el cmd mn el buffer..


         new_cmd -> number = cmd; // access el number bl cmd...
         // w h3ml access bardo ll arguments b helper function lesa hn3mlha ......
         SetArgs(new_cmd->args);
         /* Kda ana 5lst accessing ll shared resources ... raga3 el interrupts b2a....*/

         taskEXIT_CRITICAL();

         /* b3d kda b2a el task dy htb3at el command ll queue command.............*/
         xQueueSend(Cmd_Queue,&new_cmd,portMAX_DELAY);



      }
  }

void vTask3_CmdProcessing(void * para)
{
	app_cmd * new_cmd = NULL;
	char task_msg[30];
	uint8_t toggle_duration = pdMS_TO_TICKS(500);

	/* dy function bt2ra el command ely el user e5tarha mn el queue command w btnfzha b2a....*/

	while(1)
	{
		// receive the command....
         xQueueReceive(Cmd_Queue,&new_cmd,portMAX_DELAY);
         // b3d ma a-receive el command b2a hnfzo ..............
         if(new_cmd->number == LED_ON_COMMAND)
         		{
         			make_led_on();
         		}
         		else if(new_cmd->number == LED_OFF_COMMAND)
         		{
         			make_led_off();
         		}
         		else if(new_cmd->number == LED_TOGGLE_COMMAND)
         		{
         			led_toggle_start(toggle_duration);
         		}
         		else if(new_cmd->number == LED_TOGGLE_STOP_COMMAND)
         		{
         			led_toggle_stop();
         		}
         		else if(new_cmd->number == LED_READ_STATUS_COMMAND)
         		{
         			read_led_status(task_msg);  // eih el task_msg ?? dy el hy7ot feha el status y3ny..
         		}
         		else if(new_cmd->number == RTC_READ_DATE_TIME_COMMAND )
         		{
         			read_rtc_info(task_msg);
         		}else
         		{
         			print_error_message(task_msg);
         		}

         /* a7na 5las 5lsna processing bs enta kont 3aml dynamic allocation bl malloc() function
          * w tb3an lazm b3d ma t5las tms7ha ...a7na aslun m4 most7ab t7ml dynamic allocation f el embedded
          * bs modtren y3ny ......fa lazm lw ht3ml kda t3ml free 3ltol....
          * fa ana 3mlt create ll command fe el task2 dlw2ty h3ml ll command processing.....
          *  fa b3d ma a5ls processing h3ml free b2a ll command el kont 3mltlo dynamic allocation ka struct f el heap
          *  w b3ml kda b API bardo...
          */

         vPortFree(new_cmd);

	}
}

void vTask4_UartWrite(void * para)
{
	char * data = NULL; // dy el ha5od beha el data ... el h3ml beha receive ll data y3ny.........

	// dy function bt2ra el menu mn el queue w tb3tha bl UART 34an el user y4ofha b2a..........
	while(1)
	{
		// fa el awl hsta2bel el 7aga mn el queue..............
        xQueueReceive(Uart_Write_Queue,&data,portMAX_DELAY);
        // b3d kda hb3tha 3ala el uart b2a....................
        print_msg(data);

	}
}


// dy function for setupping UART configurations and initializations ...........
static void prvSetupUart()
{

	GPIO_InitTypeDef gpio_uart_pins;  // struct for initializing the PINS...this struct is passed to the GPIO_init() function .....

	/* 1) enable UART2 peripheral clock and gpioA el feha el 2 pins bto3 el UART2 da fa lazm a7ot el clock 3ala el port da zay el TIVAC kda ..la2n UART2 hya el hst3mlha hena zay ma 2olt ... fa fe el MC da lazm a3ml enable ll clock bta3 el UART el awl .....*/
	// so there is a specific API for managing clock .........here it is....
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);  // APB da m3nah en el USART da mtwsl bl ABP processor m4 el AHB ......Tab 3rft mnen ?? mn el data sheet..w aft7 el data sheet w shof b2a el function dy bta5od eih w kda....
	RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* make struct elements = 0 .............memset is a function which do that .......*/
	memset(&gpio_uart_pins,0,sizeof(gpio_uart_pins));  // howa el struct el ana 3mltha dy local ... fa hya btb2a initialized b values f el awl f el stack ... fa ana 3ayz asfrha el awl w b3d kda a7ot el values el ana 3ayzha feha ..... 34an bs adman en mfy4 values 3'alat htt5zn 3andy f el struct..........

	/* 2) configuring PA2 and PA3 modes...........el struct gowaha elements m7tag a3mlhom config w feh elements mlhom4 lazma zay el speed msln m4 7war w m4 hst3mlo .....*/
	gpio_uart_pins.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; /*  tb3an bst3ml el macros bta3t el pins bta3t el gpio driver el mwgod hena ...zay ma enta 3mlt el macros dy bl zabt fe el port driver fe project el autosar  ;) fa hya 3obara 3an macros ana m3rfha w bst3mlha ...........*/
    gpio_uart_pins.GPIO_Mode = GPIO_Mode_AF;
    gpio_uart_pins.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init(GPIOA,&gpio_uart_pins); // hb3tlha base address of portA el feh el 2 pins el 3ayz a3mlhom config w hb3tlha address of the struct......

	/* 3) AF mode settings ...h5ly PA2/PA3  y4t3'lo m3 el UART(ka UART pins y3ny) .....asl ana 2olt fo2 enhm AF tab eih bzabt f el AF y3ny hy4t3'lo m3 el UART wla timer msln zay ma mwgod f el data sheet ....homa applicable for both...fa lazm a2olo la2 hy4t3'lo m3 el UART...*/
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);  //PA2..
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);  //PA3..

	/* 4) UART parameters.................*/
	USART_InitTypeDef uart2_init; // struct to set the UART parameters.............this struct is passed to the USART_Init() function...


    /* make struct elements = 0 .............memset is a function which do that .......*/
	memset(&uart2_init,0,sizeof(uart2_init));  // howa el struct el ana 3mltha dy local ... fa hya btb2a initialized b values f el awl f el stack ... fa ana 3ayz asfrha el awl w b3d kda a7ot el values el ana 3ayzha feha ..... 34an bs adman en mfy4 values 3'alat htt5zn 3andy f el struct..........


	uart2_init.USART_BaudRate = 115200; // tb3an hya arkam standard fe el baud rate ...shof tare2 ...hya arkam m3rofa kda w enta bt5tar benhom....
	uart2_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	uart2_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	uart2_init.USART_Parity = USART_Parity_No;
	uart2_init.USART_StopBits = USART_StopBits_1;
	uart2_init.USART_WordLength = USART_WordLength_8b; // el hya el frame y3ny ( 3adad el data bits ).......
	USART_Init(USART2,&uart2_init);


	// implement the UART interrupt handler......
	// feh API hya bt3ml configuration ll UART interrupt da............
	// el API mwgoda f el USART.C asmha " USART_ITConfig ".................
	// ro7 a2raha w shof el parameters el hya bta5odha w hya bt3ml eih aslun....
	// awl parameter da rakam el UART el enta btst3mlo..
	// tany parameter da no3 el interrupt ...y3ny 3ayz interrupt lama el user yb3at data ....wla lama el user yst2bel data...wla lama yekon feh parity error msln ....wla lama yekon feh idle line......w 7agat kter f45.....fa hena ana 3ayz interrupt lama el user yb3at data(y3ny ana k-UART) h-receive data.....
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);

	// lets set the priority in NVIC registers ..
	NVIC_SetPriority(USART2_IRQn,5);

	//Enable the IRQ....
	NVIC_EnableIRQ(USART2_IRQn);


	/* 5) enabling UART .... ana 3mlt init ll configs bs ...h3ml enable b2a 34an t4t3'l ..........zay el TIVAC kda b3ml enable ll UART b3d ma a5ls kol el configs*/
	USART_Cmd(USART2,ENABLE);
}


/* function to setup the button and the led ( GPIO ) in general.....................*/
static void prvSetupGPIO()
{

	// first of all ... put the clock on the 2 ports...........
	// search this function to know what this function takes as parameters..........
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	// put clock on the interrupt to register ...howa da el nzam f el STM32 family..
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);

	// Creating two variables of type "struct" of GPIO_InitTypeDef ..........
	GPIO_InitTypeDef led_pin , button_pin;
	// set the data in each struct for the LED and the Button............
	// start with the LED .............
	led_pin.GPIO_Mode = GPIO_Mode_OUT ; // output pin
	led_pin.GPIO_OType = GPIO_OType_PP; // hya dy 7aga asmha output type bs ana m4 fahm eih el output type da ?!
	led_pin.GPIO_Pin = GPIO_Pin_5 ;  // Pin number...
	led_pin.GPIO_Speed = GPIO_Low_Speed ; // m4 fahm bardo bsra7a eih el GPIO speed dy !!
	led_pin.GPIO_PuPd = GPIO_PuPd_NOPULL; // no pull ......wla pull up wla pull down...
	GPIO_Init(GPIOA,&led_pin);


	// Button pin
		button_pin.GPIO_Mode = GPIO_Mode_IN ; // input pin
		button_pin.GPIO_OType = GPIO_OType_PP; // hya dy 7aga asmha output type bs ana m4 fahm eih el output type da ?!
		button_pin.GPIO_Pin = GPIO_Pin_12 ;  // Pin number...
		button_pin.GPIO_Speed = GPIO_Low_Speed ; // m4 fahm bardo bsra7a eih el GPIO speed dy !!
		button_pin.GPIO_PuPd = GPIO_PuPd_NOPULL; // no pull ......wla pull up wla pull down...
		GPIO_Init(GPIOC,&button_pin);

	// configuring the button for acting as interrupt............
		// feh register asmo SYSCFG leh el API dy ..dy el b5tar beha a2rab EXTI ll pin w el port el 3lehom el interrupt bta3y....
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource13);
		// ana kda 7adedt el EXTI block el hst3mlo ll interrupt ... lazm a3ml configure ll block da b2a.....
		// ezay b2a h3ml configure ll EXTI block ???? bardo b struct w el struct da bb3to l init function ....
		EXTI_InitTypeDef exti_init;  // da el struct kda..
		// azbot el struct variables b2a.......3mltha lw7dy :D....
		exti_init.EXTI_Line = EXTI_Line13;
		exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
		exti_init.EXTI_Trigger = EXTI_Trigger_Falling; // 3mlo m3 el falling edge...
		exti_init.EXTI_LineCmd = ENABLE;

		// hb3at el struct ll init function b2a.............
		EXTI_Init(&exti_init);
		// fa el ana 3mlto eny 7adedt el EXTI block el hst3mlo w 3mltlo configuration............................


		// el ana h3mlo dlw2ty eny hzbat el NVIC registers zay el TIVAC bzabt....
		// SET PRIORITY..
		NVIC_SetPriority(EXTI15_10_IRQn,5);
		//Enable IRQ.....
		NVIC_EnableIRQ(EXTI15_10_IRQn);
		// we are done........................................................................
}


void print_msg(char * s)
{
	for(int i=0;i<strlen(s);i++)
	{
		/* w zay ma enta 3arf b2a fe el UART .... m2dar4 ab3at data tany ela lama at2kd en el data el 2ola atb3tt 5las
		* fa kont b3mel polling lw tftkr 3ala register l7d ma yb2a b 1(kda sending data is done).....................
		* hena bs el mwdo3 mo5tlf shwia ...hya function btrg3ly el flag w a4of howa b zero wla 1 ...fa hya function m4 register b3ml 3leh polling....
		*/
		while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == 0);

		USART_SendData(USART2,s[i]);  // howa hyb3at byte byte ... ana kont m7ded fe el UART configs. el number of data bits w kan b 8-bits on the single frame .... fa hb3at byte byte b2a ......

	}
}





void prvSetupHardware() // 2olna dy el function el bt3ml setup ll micro configurations ...fa hena msln h3ml configure ll UART....
{
	// setup the button and LED so i will setup the GPIO................
	prvSetupGPIO();
	// setup the UART...........
	prvSetupUart();
}


void rtos_delay(uint32_t delay_in_ms)
{
	uint32_t tick_count = xTaskGetTickCount(); // dy function btgeb el tick el f el systick dlw2ty....
	// fa ana 3andy el delay in ms ..... 3ayz a4of da yo3adel kam count fa h3ml el cross multiplication
	uint32_t delay_in_counts = (delay_in_ms * configTICK_RATE_HZ)/1000;
	while(xTaskGetTickCount() < tick_count+delay_in_counts); // kol mara el tick count byzed ...fa hstnah l7d ma y3ml el delay el ana 3ayzo....
}


void USART2_IRQHandler()
{
    BaseType_t x = pdFALSE;
	uint16_t data; // hst3mlo b3den......


	/* 5aly balak awe.....el ISR dy ... dy common ISR ....y3ny ay UART interrupt y7sal hy-call el ISR dy...
	 * sawa2 el interrupt 7asal bsbb receiving data aw sending data aw parity error aw idle line ...aw ayan kan
	 * fa howa 3ltol hy-call el ISR dy directly ..... tab ana 3ayz a3ml functionality mo3yna lama a-receive data bs
	 * lakn m4 ay 7aga mn el ba2y dol......fa h3ml hena checking .... howa el ISR dy 7slha call 34an 7asal
	 * receiveing data ? wla 34an 7aga tanya ?
	 * w b3mel el 7war da b API bardo...
	 */

	// fa de API bt4of el flag status .....
	// hb3tlha el flag el howa " USART_IT_RXNE " ....lw el interrupt 7sl bsbb en el uart received data f3lan
	// fa el flag da byb2a b one ....lakn lw bsbb 7aga tanya el flag da byfdal b zero....
	// afta7 w a2ra el API dy .................
	if(USART_GetFlagStatus(USART2,USART_IT_RXNE))
	{
		// ISR is executing due to data receiving f3lan ........
		// fa get the data b2a :D .............
		data = USART_ReceiveData(USART2);
		// note : data dy uint16_t 34an el function dy bt-return uint16_t ..............

		// a7na 2olna el ISR dy bta5od el command w t7fzha .....
		// fa h7otha f buffer 34an a7fzha.................................
        cmd_buffer[len++] = (data & 0xFF) ; // ana interested f awl 8 bits bs ...34an kda 3mlt el "&" dy .............


        // tab ana h3raf el user 5alas da5al kol 7aga wla eih ezay ?????
        // el mafrod lama yedos enter ... kda yb2a 5alas howa finished..
        if(data == '\r')
        {
        	// el \r dy el howa m3naha enter y3ny................el command 5lst 5las....
        	len = 0; // 3mlt index b zero el command 5lst 5las...
        	// el mafrod b2a el cmd_handling task te2ra mn el ISR dy el command el galha....
        	// fa ana h-notify el task dy en ana galy command 5las.........

        	xTaskNotifyFromISR(task2_handle,0,eNoAction,&x);
        	/* mohm gedan rakez..........lazm FromISR .... la2n enta bt3ml l FreeRTOS API mn ISR ....a7na wada7na el
        	* 7war da abl kda........w 2olna el FromISR dy bta5od one parameter zyada......*/


        	// w bardo h-notify el menu task 34an tekamel teba3a b2a w ttba3 tany .....w yd5al command tany w hakza..
        	xTaskNotifyFromISR(task1_handle,0,eNoAction,&x);
        }
	}

	if(x)
	{
		// el x ba2et b TRUE ... fa kda el API gat fe seketha task tanya with higher priority ...
		// a7na kona shr7na el 7war da..........
		// fa lazm a3ml task yielding kda b2a ...........
		taskYIELD();
	}



}


// helper functions...............................................

uint8_t getCmd(uint8_t * buffer)
{
	return buffer[0]-48; // to convert from ascii to number...
}


// we are not supporting this in our application .........................leave it empty..
void SetArgs(uint8_t * a)
{

}


// command helper functions..........................................................................
void make_led_on(void)
{
	GPIO_WriteBit(GPIOA,GPIO_Pin_5,Bit_SET);
}


void make_led_off(void)
{
	GPIO_WriteBit(GPIOA,GPIO_Pin_5,Bit_RESET);
}



void led_toggle(TimerHandle_t xTimer) // el argument da howa lazm ab3to kda la2n dy callback function ...shof el timer.h htla2y el 7war da ...by2olak el callback function lazm tbt3tlha el argument da kda ....
{
	GPIO_ToggleBits(GPIOA,GPIO_Pin_5);
}

void led_toggle_start(uint32_t duration)
{

	if(led_timer_handle == NULL) // howa el user mmkn y3ml command el toggle kaza mara ....fa b4of lw el timer created aslun fa 5las h3mlo start 3ltol ...lakn lw m4 created aslun h3mlo creation el awl..
	{
		//1. lets create the software timer
		/* el xTimerCreate bta5od 5 parameters
		 * awl wa7d da el timer name
		 * tany wa7ed el duration
		 * talet wa7ed da el reload flag ...y3ny enta 3ayzo b3d ma y5las counting y3ed el counting tany wla la2 ? .. da el ana 3ayzo 3mtn ...ana 3ayzo y-toggle kol 500ms...
		 * rabe3 wa7ed da el timer ID
		 * 5ames wa7ed da el call back function el b3d ma el xTimerCreate t5las ht3mlha call....el hya el led_toggle..
		 */
		led_timer_handle = xTimerCreate("LED-TIMER",duration,pdTRUE,NULL,led_toggle);

		//2. start the software timer
		// el xTimerStart dy f el implementation bta3ha btst3ml queues asmha timer queues
	    // 34an kda b2olo wait until its not empty .............
		xTimerStart(led_timer_handle,portMAX_DELAY);
	}
	else
	{
		//start the software timer

		xTimerStart(led_timer_handle,portMAX_DELAY);
	}
}


void led_toggle_stop(void)
{
	// stops the software timer......................
	// hya stops the timer bs lakn m4 btms7o ...m4 btms7 el task bta3to y3ny...
	 xTimerStop(led_timer_handle,portMAX_DELAY);
}


void read_led_status(char *task_msg)
{
	//h7ot el status f el task_msg el ana ba3etha dy..
	sprintf(task_msg , "\r\nLED status is : %d\r\n", GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_5));
	// el mafrod ab3tha ll uart_write_queue fa ywdeha ll uart_write task fa tb3tha ll user bl UART......
	xQueueSend(Uart_Write_Queue,&task_msg,portMAX_DELAY);
}


void read_rtc_info(char *task_msg)
{
	RTC_TimeTypeDef RTC_time;
	RTC_DateTypeDef RTC_date;
	//read time and date from RTC peripheral of the microcontroller
	RTC_GetTime(RTC_Format_BIN, &RTC_time);
	RTC_GetDate(RTC_Format_BIN, &RTC_date);

	sprintf(task_msg,"\r\nTime: %02d:%02d:%02d \r\n Date : %02d-%2d-%2d \r\n",RTC_time.RTC_Hours,RTC_time.RTC_Minutes,RTC_time.RTC_Seconds, \
									RTC_date.RTC_Date,RTC_date.RTC_Month,RTC_date.RTC_Year );
	xQueueSend(Uart_Write_Queue,&task_msg,portMAX_DELAY);



}


void print_error_message(char *task_msg)
{
	sprintf( task_msg,"\r\nInvalid command received\r\n");
	xQueueSend(Uart_Write_Queue,&task_msg,portMAX_DELAY);
}








