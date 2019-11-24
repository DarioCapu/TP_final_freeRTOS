/* Copyright 2016, Eric Pernia
 * All rights reserved.
 *
 * This file is part of lpc1769_template.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** @brief Blinky using FreeRTOS.
 *
 *
 * NOTE: It's interesting to check behavior differences between standard and
 * tickless mode. Set @ref configUSE_TICKLESS_IDLE to 1, increment a counter
 * in @ref vApplicationTickHook and print the counter value every second
 * inside a task. In standard mode the counter will have a value around 1000.
 * In tickless mode, it will be around 25.
 *
 */

/** \addtogroup rtos_blink FreeRTOS blink example
 ** @{ */

/*==================[inclusions]=============================================*/

#include "../../TP_final_freeRTOS/inc/FreeRTOSConfig.h"
#include "../../TP_final_freeRTOS/inc/main.h"
#include "board.h"
#include "sapi.h"        // <= sAPI header
#include "sapi_i2c.h"   /* <= sAPI I2C header */
#include "sapi_adc.h"	// sAPI ADC header

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "strings.h"
#include "string.h"
#include <stdlib.h>


/*==================[macros and definitions]=================================*/

#define GY906_ADDR	0x00	// Dirección del sensor de temperatura
#define TAMB_ADDR	0x06	// Dirección de la temperatura de ambiente
#define TOBJ1_ADDR	0x07	// Dirección de la temperatura del objeto

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

typedef struct
{
	float tension;
	float temperatura;
	float alarma_baja;
	float alarma_alta;
}
medicion;

/**
 * C++ version 0.4 char* style "itoa":
 * Written by Lukás Chmela
 * Released under GPLv3.

 */

static void floatToString( float valor, char *dst, uint8_t pos ){
	uint16_t val;
	val = 100 * valor;

	dst[pos] = (val / 1000) + '0';
	pos++;
	dst[pos] = (val % 1000) / 100 + '0';
	pos++;
	dst[pos] = '.';
	pos++;
	dst[pos] = (val % 100) / 10 + '0';
	pos++;
	dst[pos] = (val % 10)  + '0';
	pos++;
	dst[pos] = '\0';

}

static void floatToString2( float valor, char *dst, uint8_t pos ){
	uint16_t val;
	val = 100 * valor;

	dst[pos] = (val / 10000) + '0';
	pos++;
	dst[pos] = (val % 10000)/1000 + '0';
	pos++;
	dst[pos] = (val % 1000) / 100 + '0';
	pos++;
	dst[pos] = '.';
	pos++;
	dst[pos] = (val % 100) / 10 + '0';
	pos++;
	dst[pos] = (val % 10)  + '0';
	pos++;
	dst[pos] = '\0';

}

char* itoa(int value, char* result, int base) {
   // check that the base if valid
   if (base < 2 || base > 36) { *result = '\0'; return result; }

   char* ptr = result, *ptr1 = result, tmp_char;
   int tmp_value;

   do {
      tmp_value = value;
      value /= base;
      *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
   } while ( value );

   // Apply negative sign
   if (tmp_value < 0) *ptr++ = '-';
   *ptr-- = '\0';
   while(ptr1 < ptr) {
      tmp_char = *ptr;
      *ptr--= *ptr1;
      *ptr1++ = tmp_char;
   }
   return result;
}

/*==================[internal functions definition]==========================*/

static void initHardware(void)
{
   /* Inicializar la placa */
   boardConfig();

   /* Inicializar Uart */
   uartConfig( UART_232, 9600 );
   //debugPrintConfigUart( UART_USB, 115200 );
   i2cConfig( I2C0, 100000 );	// Inicializa la comunicación I2C
   adcConfig( ADC_ENABLE );		// Inicializa el ADC

}


void vPrintString( char * string){
   uartWriteString( UART_232, string );
}


void vPrintNumber( int32_t number){
   uint8_t uartBuff[10];
   /* Conversión de number entero a ascii con base decimal */
   itoa( number, uartBuff, 10 ); /* 10 significa decimal */
   /* Enviar number */
   uartWriteString( UART_232, uartBuff );
}


void vPrintStringAndNumber( char * string, int32_t number){
   vPrintString( string );
   vPrintNumber( number );
   vPrintString( "\r\n" );
}

/*==================[external functions definition]==========================*/


/* The tasks to be created.  Two instances are created of the sender task while
only a single instance is created of the receiver task. */
static void vSenderTask( void *pvParameters );
static void vReceiverTask( void *pvParameters );

/*-----------------------------------------------------------*/

/* Declare a variable of type xQueueHandle.  This is used to store the queue
that is accessed by all three tasks. */
xQueueHandle xQueue;


int main( void )
{
	 initHardware();

    /* The queue is created to hold a maximum of 5 long values. */
    xQueue = xQueueCreate( 3, sizeof( medicion ) );

	if( xQueue != NULL )
	{
		/* Create two instances of the task that will write to the queue.  The
		parameter is used to pass the value that the task should write to the queue,
		so one task will continuously write 100 to the queue while the other task
		will continuously write 200 to the queue.  Both tasks are created at
		priority 1. */
		xTaskCreate( vSenderTask, "Sender1", 240, ( void * ) 100, 1, NULL );
//		xTaskCreate( vSenderTask, "Sender2", 240, ( void * ) 200, 1, NULL );

		/* Create the task that will read from the queue.  The task is created with
		priority 2, so above the priority of the sender tasks. */
		xTaskCreate( vReceiverTask, "Receiver", 240, NULL, 2, NULL );

		/* Start the scheduler so the created tasks start executing. */
		vTaskStartScheduler();
	}
	else
	{
		/* The queue could not be created. */
	}
    /* If all is well we will never reach here as the scheduler will now be
    running the tasks.  If we do reach here then it is likely that there was
    insufficient heap memory available for a resource to be created. */
	for( ;; ); // lo mismo que while(1);
	return 0;
}

/*-----------------------------------------------------------*/

static void vSenderTask( void *pvParameters )
{

uint16_t t_obj1=0;
uint8_t dato[3];
uint8_t dataToRead;
static medicion medicion_actual={0,0,0,100};
uint8_t alarma = 0;
char buffer_alarma_baja[64];
char buffer_alarma_alta[64];
portBASE_TYPE xStatus;
static int i=0;

	/* Two instances are created of this task so the value that is sent to the
	queue is passed in via the task parameter rather than be hard coded.  This way
	each instance can use a different value.  Cast the parameter to the required
	type. */
	//lValueToSend = ( long ) pvParameters;

	/* As per most tasks, this task is implemented within an infinite loop. */
	for( ;; )
	{
		/* The first parameter is the queue to which data is being sent.  The
		queue was created before the scheduler was started, so before this task
		started to execute.

		The second parameter is the address of the data to be sent.

		The third parameter is the Block time – the time the task should be kept
		in the Blocked state to wait for space to become available on the queue
		should the queue already be full.  In this case we don’t specify a block
		time because there should always be space in the queue. */

		medicion_actual.tension = adcRead( CH1 ) * 3.30 / 1024 ;

		/* Leo temperatura de objeto del sensor */
		dataToRead = TOBJ1_ADDR;
		i2cRead( I2C0,GY906_ADDR,&dataToRead,1,TRUE,&dato,3,TRUE );
		t_obj1 = dato[0];
		t_obj1 |= dato[1]<<8;
		medicion_actual.temperatura = 0.02 * t_obj1 - 273.15;


		// valores de prueba

		if(i>100)
		{
			i=0;
		}

		medicion_actual.temperatura = i;

		i++;

		if( uartReadByte( UART_232, &alarma ) ){

			if(alarma == 'B'){

				uartReadByte( UART_232, &alarma );

				int i=0;
				while(alarma != 'B')
				{
					buffer_alarma_baja[i]=alarma;

					i++;

					uartReadByte( UART_232, &alarma );

				}

				buffer_alarma_baja[i]='\0';

				medicion_actual.alarma_baja = atoi(buffer_alarma_baja);

			}

			if(alarma == 'A'){

				uartReadByte( UART_232, &alarma );

				int i=0;
				while(alarma != 'A')
				{
					buffer_alarma_alta[i]=alarma;

					i++;

					uartReadByte( UART_232, &alarma );

				}

				buffer_alarma_alta[i]='\0';

				medicion_actual.alarma_alta = atoi(buffer_alarma_alta);

			}

		}

		xStatus = xQueueSendToBack( xQueue, &medicion_actual, 0 );


		if( xStatus != pdPASS )
		{
			/* We could not write to the queue because it was full – this must
			be an error as the queue should never contain more than one item! */
			vPrintString( "Could not send to the queue.\r\n" );
		}

		/* Allow the other sender task to execute. */
		taskYIELD();
	}
}
/*-----------------------------------------------------------*/

static void vReceiverTask( void *pvParameters )
{

/* Declare the variable that will hold the values received from the queue. */
static medicion medicion_actual={0,0,0,100};
char buffout[64];		// Resultado en string
char buffer_alarma_baja[64];
char buffer_alarma_alta[64];
portBASE_TYPE xStatus;
const portTickType xTicksToWait = 100 / portTICK_RATE_MS;
static int tiempo=0;

	/* This task is also defined within an infinite loop. */
	for( ;; )
	{
		/* As this task unblocks immediately that data is written to the queue this
		call should always find the queue empty. */
		if( uxQueueMessagesWaiting( xQueue ) != 0 )
		{
			vPrintString( "Queue should have been empty!\r\n" );
		}

		/* The first parameter is the queue from which data is to be received.  The
		queue is created before the scheduler is started, and therefore before this
		task runs for the first time.

		The second parameter is the buffer into which the received data will be
		placed.  In this case the buffer is simply the address of a variable that
		has the required size to hold the received data.

		the last parameter is the block time – the maximum amount of time that the
		task should remain in the Blocked state to wait for data to be available should
		the queue already be empty. */
		xStatus = xQueueReceive( xQueue, &medicion_actual, xTicksToWait );

		if( xStatus == pdPASS )
		{
			/* Data was successfully received from the queue, print out the received value. */

			// tension actual

			vPrintString( "*V" );
			floatToString( medicion_actual.tension, buffout, 0 );
			vPrintString( buffout );
			vPrintString("\r\n");

			// temperatura actual

			vPrintString( "*T" );
			floatToString( medicion_actual.temperatura, buffout, 0 );
			vPrintString( buffout );
			vPrintString("\r\n");

			// envia la temperatura al monitor

			vPrintString( "*M" );
			vPrintString( "X" );
			itoa(tiempo,buffout,10);
			vPrintString( buffout );
			vPrintString( "Y" );
			floatToString( medicion_actual.temperatura, buffout, 0 );
			vPrintString( buffout );
			vPrintString("*\r\n");

			// alarma de temperatura baja
			vPrintString( "*B" );
			floatToString( medicion_actual.alarma_baja, buffer_alarma_baja, 0 );
			vPrintString( buffer_alarma_baja );
			vPrintString("\r\n");

			// alarma de temperatura alta

			vPrintString( "*A" );
			floatToString2( medicion_actual.alarma_alta, buffer_alarma_alta, 0 );
			vPrintString( buffer_alarma_alta );
			vPrintString("\r\n");

			if(medicion_actual.temperatura < medicion_actual.alarma_baja)
			{
				vPrintString( "*L" );
				vPrintString("\r\n");
			}
			if(medicion_actual.temperatura > medicion_actual.alarma_alta)
			{
				vPrintString( "*H" );
				vPrintString("\r\n");

			}

			tiempo++;

			if(tiempo>100)
			{
				tiempo=0;
			}

		}
		else
		{
			/* We did not receive anything from the queue even after waiting for 100ms.
			This must be an error as the sending tasks are free running and will be
			continuously writing to the queue. */
			vPrintString( "Could not receive from the queue.\r\n" );
		}



	}
}
/*-----------------------------------------------------------*/


/** @} doxygen end group definition */

/*==================[end of file]============================================*/
