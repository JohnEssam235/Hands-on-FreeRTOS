/**

  *******************************************************************************/
#include <stdio.h>
#include <stdint.h>  // dy el library el feha el standard types bta3tna ...el hya uint8_t w uint32_t w hakza.............
#include <string.h>  // include it to use the memset() function..
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"  // this is the library which provides the usage of the Task creation API .........

/* prototypes...........*/
void print_msg(char * s);
void prvSetupHardware();
static void prvSetupUart();
static void prvSetupGPIO();

void Led_Task_Handler(void * para);
void Button_Task_Handler(void * para);

/* global vars.....*/
uint8_t flag = 0; // tb3an mfy4 7aga asmha zero w 1 ...lazm tb2a declared macros...bs da sho3'l sare3 kda...


int main(void)
{



	/* 1) make PLL off*/
   RCC_DeInit(); /* this function makes PLL off ..... so the CPU clock freq. returns to default(16MHZ).....*/

   /* 2) update the system clock */
   SystemCoreClockUpdate(); /* this function updates the system clock....*/

   /* setup the hardware function....if i will use the uart not the semi hosting....*/
   prvSetupHardware();  // el function el bt initialize el hardware..

// creating LED task.....
xTaskCreate(Led_Task_Handler,"LED_Task",configMINIMAL_STACK_SIZE,NULL,2,NULL);


/* creating Button task ...............*/
xTaskCreate(Button_Task_Handler,"Button_Task",configMINIMAL_STACK_SIZE,NULL,2,NULL);

/* 4)..............scheduler function*/
vTaskStartScheduler();


 for(;;);

}


/* task1 handler function ... tb3an esm el handler function howa el enta m3rfo f el creation..*/
// el handler function bta3t el LED m4 bt3ml 7aga 3'er enha t4of flag mo3yn lw b 1 kda yb2a el LED tnwr
// lakn lw b zero 5las sebha zay ma hya ...
void Led_Task_Handler(void * para) /* 2olna bta5od pointer wa7d bs .....w 2olna btrg3 void....aftkr shakl el handler function...*/
 {
    while(1)
    {
       if(flag == 1)
       {
    	   // turn LED on..
    	   // h3ml turn on ezay ??!! b API zay ma enta 3arf ......a3ml search 3la el API da w shof bya5od eih ..
    	   GPIO_WriteBit(GPIOA,GPIO_Pin_13,Bit_SET);
       }
       else
       {
    	   // turn LED off.......
    	   GPIO_WriteBit(GPIOC,GPIO_Pin_12,Bit_RESET);
       }


    }

}

 /* task2 handler function ... tb3an esm el handler function howa el enta m3rfo f el creation..*/
void Button_Task_Handler(void * para) /* 2olna bta5od pointer wa7d bs .....w 2olna btrg3 void....aftkr shakl el handler function...*/
  {
      while(1)
      {
    	  if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13) == 1)
    	  {
    		  flag = 1;
    	  }
    	  else
    		  flag = 0;
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



