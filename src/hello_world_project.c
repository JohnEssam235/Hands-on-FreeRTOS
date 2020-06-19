/**

  *******************************************************************************/
#include <stdio.h>
#include <stdint.h>  // dy el library el feha el standard types bta3tna ...el hya uint8_t w uint32_t w hakza.............
#include <string.h>  // include it to use the memset() function..
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"  // this is the library which provides the usage of the Task creation API .........

// names for the pointers pointing to the TCB of each task..................
TaskHandle_t handle1 = NULL; /* initialized to NULL...*/
TaskHandle_t handle2 = NULL;

/* task handlers prototypes*/
void vTask1_Handler(void * para);
void vTask2_Handler(void * para);

#ifdef USE_SEMI_HOSTING
/* used for semi hosting .......*/
extern void initialise_monitor_handles();
#endif


/* function for hardware(micro) configuration  ..... 34an msln ast3ml el UART aw ay 7aga ana 3ayzha fa a3mlha configuration ....*/
static void prvSetupHardware();
/* function for setup UART ....*/
static void prvSetupUart();
/* FUNCTION FOR PRINTING A STRING...*/
void print_msg(char * s);



/* THE MESSAGE i wanna print..*/

char message[]="hello..John";


/* flags....*/

uint8_t flag = 1;

int main(void)
{

#ifdef USE_SEMI_HOSTING
	/* calling the semi hosting function in the main()....*/
	initialise_monitor_handles();
#endif

	// el DWT register da register by5zn 3adad el clock cycles el at3mlt b3d ma 3mlt reset ll CPU(by3ml cycle counting y3ny)................
	DWT->CTRL |= (1 << 0);//Enable CYCCNT in DWT_CTRL.

	printf("Hello world ... first project");

	/* 1) make PLL off*/
   RCC_DeInit(); /* this function makes PLL off ..... so the CPU clock freq. returns to default(16MHZ).....*/

   /* 2) update the system clock */
   SystemCoreClockUpdate(); /* this function updates the system clock....*/

   /* setup the hardware function....if i will use the uart not the semi hosting....*/
   prvSetupHardware();  // el function el bt initialize el hardware..
   print_msg(message); // el function el bt3ml sending el string(data) by UART............

   /* two APIs to config and start event recording.......*/
   SEGGER_SYSVIEW_Conf();
   SEGGER_SYSVIEW_Start();

   /* 3) lets make the task ..*/
   /* task1 creation API ...............*/
xTaskCreate(vTask1_Handler,"Task_1",configMINIMAL_STACK_SIZE,NULL,2,&handle1);


/* task2 creation API ...............*/
xTaskCreate(vTask2_Handler,"Task_2",configMINIMAL_STACK_SIZE,NULL,2,&handle2);

/* 4)..............scheduler function*/
vTaskStartScheduler();
 for(;;);

}


/* task1 handler function ... tb3an esm el handler function howa el enta m3rfo f el creation..*/
void vTask1_Handler(void * para) /* 2olna bta5od pointer wa7d bs .....w 2olna btrg3 void....aftkr shakl el handler function...*/
 {
    while(1)
    {

#ifdef USE_SEMI_HOSTING
      	printf("Hello World: From Task_1");
#endif
      	/* lakn lw hst3ml el UART ...hst3ml el function el esmha print_msg().................... */
      	if(flag == 1)
      	{
         flag =0;  // hy5o4 task1 w y5ly flag =0 ...hynfz shwia mn el string w yro7 l task2............
      	 print_msg("Hello..from Task_1");
      	 flag = 1;
      	  taskYIELD();// 34an a5ly el task1 tseb el CPU lama t5ls ...34an my5o45 fe task1 tany w ytba3 task1 tany ...w yro7 l task2 b2a......
        }
 }
    // el mafrod b3d kol task bt5ls a3mlha delete .. 34an a4elha w ams7ha ...bs ana hena m4 3ayz a3mlha delete..
    // la2ny m4 3ayz ams7ha aslun ... la2n ana 3ayzha tfdal mwgoda w sh3'ala....

 }

 /* task2 handler function ... tb3an esm el handler function howa el enta m3rfo f el creation..*/
void vTask2_Handler(void * para) /* 2olna bta5od pointer wa7d bs .....w 2olna btrg3 void....aftkr shakl el handler function...*/
  {
      while(1)
      {
#ifdef USE_SEMI_HOSTING
      	printf("Hello World: From Task_2");
#endif
      	/* lakn lw hst3ml el UART ...hst3ml el function el esmha print_msg()....................*/
      	if(flag == 1)  // yegy hena yla2y el flag b zero ....la2n ana 3mlto b zero lama d5lt fe task1 .... fa lama el time slice y5las hyrg3 l task1 tany w ykml ba2y el string bta3tha w ttnfz sa7....5aly balak awe awe awe howa lama byrg3 l task1 byrg3 mn el makan el howa meshy mn 3ndo lama el time slice 5ls....y3ny hyrg3 ynfz el string 3ltol w ykmlha ....m4 lesa hy3ml check 3ala el if condition w kda .............
      	      	{
      	         flag =0;
      	      	 print_msg("Hello..from Task_2");
      	      	 flag = 1;
      	      	  taskYIELD();  // 34an a5ly el task1 tseb el CPU lama t5ls ...34an my5o45 fe task1 tany w ytba3 task1 tany ...w yro7 l task2 b2a......

      	        }

      }
  }


// dy function for setupping UART configurations and initializations ...........
static void prvSetupUart()
{

	GPIO_InitTypeDef gpio_uart_pins;  // struct for initializing the PINS...this struct is passed to the GPIO_init() function .....

	/* 1) enable UART2 peripheral clock and gpioA el feha el 2 pins bto3 el UART2 da fa lazm a7ot el clock 3ala el port da zay el TIVAC kda ..la2n UART2 hya el hst3mlha hena zay ma 2olt ... fa fe el MC da lazm a3ml enable ll clock bta3 el UART el awl .....*/
	// so there is a specific API for managing clock .........here it is....
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
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
static void prvSetupHardware() // 2olna dy el function el bt3ml setup ll micro configurations ...fa hena msln h3ml configure ll UART....
{
	// setup the UART...........
	prvSetupUart();
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



