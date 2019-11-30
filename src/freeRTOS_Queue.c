/*============== PROYECTO FINAL - MONITOR DE TEMPERATURA ====================*/


/*==================  ARCHIVOS DE CABECERA ==================================*/

#include "../../TP_final_freeRTOS/inc/FreeRTOSConfig.h"
#include "../../TP_final_freeRTOS/inc/main.h"
#include "board.h"
#include "sapi.h"         // LIBRERIA SAPI
#include "sapi_i2c.h"     // LIBRERIA I2C
#include "sapi_adc.h"	  // LIBRERIA ADC

#include "FreeRTOS.h"     // LIBRERIA FRERTOS
#include "task.h"
#include "queue.h"
#include "strings.h"
#include "string.h"
#include <stdlib.h>


/*==================  DEFINICION DE MACROS  =================================*/

#define GY906_ADDR	0x00	// DIRECCION DEL SENSOR DE TEMPERATURA
#define TAMB_ADDR	0x06	// DIRECCION DE LA TEMPERATURA DE AMBIENTE
#define TOBJ1_ADDR	0x07	// DIRECCION DE LA TEMPERATURA DEL OBJETO

/*==================  DEFINICION VARIABLES INTERNAS ========================*/

// DEFINE LA ESTRUCTURA DE MEDICION QUE GUARDA LOS VALORES DE TENSION,TEMPERATURA,ALARMA_BAJA Y ALARMA_ALTA

typedef struct
{
	float tension;
	float temperatura;
	float alarma_baja;
	float alarma_alta;
}
medicion;


// DEFINE LA VARIABLE TIPO COLA

xQueueHandle xQueue;


/*==================    FUNCIONES EXTERNAS    ===============================*/


// CONVIERTE  UN FLOAT A UN STRING CON 4 DIGITOS  CON EL FORMATO XX.XX

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

// CONVIERTE  UN FLOAT A UN STRING CON 5 DIGITOS  CON EL FORMATO XXX.XX

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

// CONVIERTE UN ENTERO A STRING

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

// IMPRIME UN STRING POR UART

void vPrintString( char * string){
   uartWriteString( UART_232, string );
}

/*================== DECLARACION DE FUNCIONES INTERNAS ==========================*/

// FUNCION QUE INICIALIZA LOS PROTOCOLOS DE COMUNICACION Y EL HARDWARE DE LA PLACA

static void initHardware(void)
{

   //INICIALIZA LA PLACA

   boardConfig();

   // INICIALIZA LOS PROTOCOLOS DE COMUNICACION

   uartConfig( UART_232, 9600 );  // INICIALIZA LA COMUNICACION UART
   i2cConfig( I2C0, 100000 );	 // INICIALIZA INICIALIZA LA COMUNICACION I2C
   adcConfig( ADC_ENABLE );		// INICIALIZA EL ADC

}


// FUNCION UTILIZADA PARA LEER LOS VALORES DE LAS ALARMAS BAJA Y ALTA PARA GUARDARLOS EN LA MEDICION ACTUAL

void lectura_alarmas(medicion* medicion_actual,uint8_t * alarma,char* buffer_alarma_baja,char* buffer_alarma_alta )
{


	if( uartReadByte( UART_232, alarma ) ){

		if(*alarma == 'B'){

			uartReadByte( UART_232, alarma );

			int i=0;
			while(*alarma != 'B')
			{
				buffer_alarma_baja[i]=*alarma;

				i++;

				uartReadByte( UART_232, alarma );

			}

			buffer_alarma_baja[i]='\0';

			medicion_actual->alarma_baja = atoi(buffer_alarma_baja);

		}

		if(*alarma == 'A'){

			uartReadByte( UART_232, alarma );

			int i=0;
			while(*alarma != 'A')
			{
				buffer_alarma_alta[i]=*alarma;

				i++;

				uartReadByte( UART_232, alarma );

			}

			buffer_alarma_alta[i]='\0';

			medicion_actual->alarma_alta = atoi(buffer_alarma_alta);

		}

	}

};

// GUARDA EL VALOR DE TEMPERATURA LEIDA EN MEDICION ACTUAL

void lectura_temperatura(medicion* medicion_actual)
{
	// VARIABLES DE TEMPERATURA

	uint16_t t_obj1=0;
	uint8_t dato[3];
	uint8_t dataToRead;

	// LECTURA DEL VALOR DE TENSION DE TEMPERATURA

	dataToRead = TOBJ1_ADDR;
	i2cRead( I2C0,GY906_ADDR,&dataToRead,1,TRUE,dato,3,TRUE);
	t_obj1 = dato[0];
	t_obj1 |= dato[1]<<8;
	medicion_actual->temperatura = 0.02 * t_obj1 - 273.15;

}
/*==================[TAREAS]==========================*/

// GUARDA EL VALOR DE TEMPERATURA LEIDA EN TENSION ACTUAL

void lectura_tension(medicion* medicion_actual)
{
	medicion_actual->tension = adcRead( CH1 ) * 10.475 / 1024 ;
}

// ENVIA LOS DATOS DE MEDICION POR PUERTO SERIE

void envia_medicion_uart(medicion* medicion_actual,size_t tiempo)
{

	// VARAIBLES UTILIZADAS PARA LAS ALARMAS

	char buffout[64];
	char buffer_alarma_baja[64];
	char buffer_alarma_alta[64];

	// ENVIA MEDICION AL PUERTO SERIE


	// tension actual

	vPrintString( "*V" );
	floatToString( medicion_actual->tension, buffout, 0 );
	vPrintString( buffout );
	vPrintString("\r\n");

	// temperatura actual

	vPrintString( "*T" );
	floatToString( medicion_actual->temperatura, buffout, 0 );
	vPrintString( buffout );
	vPrintString("\r\n");

	// envia la temperatura al monitor

	vPrintString( "*M" );
	vPrintString( "X" );
	itoa(tiempo,buffout,10);
	vPrintString( buffout );
	vPrintString( "Y" );
	floatToString( medicion_actual->temperatura, buffout, 0 );
	vPrintString( buffout );
	vPrintString("*\r\n");

	// alarma de temperatura baja
	vPrintString( "*B" );
	floatToString( medicion_actual->alarma_baja, buffer_alarma_baja, 0 );
	vPrintString( buffer_alarma_baja );
	vPrintString("\r\n");

	// alarma de temperatura alta

	vPrintString( "*A" );
	floatToString2( medicion_actual->alarma_alta, buffer_alarma_alta, 0 );
	vPrintString( buffer_alarma_alta );
	vPrintString("\r\n");

}

// TAREA QUE ENVIA DATOS A LA COLA

static void vSenderTask( void *pvParameters )
{
/*---------------------------------- 	VARIABLES		 -----------------------------------------*/

// VARIABLE DE LA ESTRUCTURA DE MEDICION

static medicion medicion_actual={0,0,0,100};

// VARIABLES USADAS PARA LA LECTURA DE ALARMAS BAJAS Y ALTAS

static int i=0;
uint8_t alarma = 0;
char buffer_alarma_baja[64];
char buffer_alarma_alta[64];

// VARIABLES DE LA TAREA

portBASE_TYPE xStatus; // VARIABLE USADA PARA GUARDAR EL ESTADO DE CARGA DE LA COLA

portTickType xLastExecutionTime; // VARIABLE USADA PARA LA INICILIZACION DE TIEMPO DE LA TAREA

xLastExecutionTime = xTaskGetTickCount(); // INICIALIZA EL TIEMPO DONDE COMENZO  LA TAREA



	for( ;; )
	{

		// ENCIENDE LA SALIDA DIGITAL PARA MEDIR LA TENSION

		gpioWrite(GPIO1,ON);

		// LECTURA DEL VALOR DE TENSION DE TEMPERATURA

		lectura_temperatura(&medicion_actual);

		// LECTURA DEL VALOR DE TENSION DE LA BATERIA

		lectura_tension(&medicion_actual);

		//APAGA LA SALIDA DIGITAL

		gpioWrite(GPIO1,OFF);

		// VALORES DE PRUEBA DE TEMPERATURA

		/*
		if(i>100)
		{
			i=0;
		}

		medicion_actual.temperatura = i;

		i++;
		*/

		// LESTURA DE LOS VALORES DE ALARMA BAJA Y ALARMA ALTA

		lectura_alarmas(&medicion_actual,&alarma,buffer_alarma_baja,buffer_alarma_alta);

		// ENVIA LOS DATOS DE MEDICION(TENSION,TEMPERATURA,ALARMA BAJA,ALARMA ALTA) A LA COLA

		xStatus = xQueueSendToBack( xQueue, &medicion_actual, 0 );


		// CORROBORA SI SE PUDO CARGAR EL DATO EN LA COLA

		if( xStatus != pdPASS )
		{
			// EN CASO DE QUE NO SE PUEDA ESCRIBIR EN LA COLA

			vPrintString( "NO SE PUEDE ESCRIBIR EN LA COLA PORQUE ESTA LLENA\r\n" );
		}

		/* BLOQUEA LA TAREA UN TIEMPO FIJO */

		vTaskDelayUntil(&xLastExecutionTime, 500);
	}
}

// TAREA QUE SACA DATOS DE LA COLA

static void vReceiverTask( void *pvParameters )
{

// VARIABLE TIEMPO USADA PARA EL MONITOR DE TEMPERATURA

static size_t  tiempo=0;

// VARIABLE DE LA ESTRUCTURA DE MEDICION

static medicion medicion_actual={0,0,0,100};

// VARIABLES DE LA TAREA

portBASE_TYPE xStatus; // VARIABLE USADA PARA GUARDAR EL ESTADO DE CARGA DE LA COLA

const portTickType xTicksToWait = 100 / portTICK_RATE_MS;

portTickType xLastExecutionTime; // VARIABLE USADA PARA LA INICILIZACION DE TIEMPO DE LA TAREA

xLastExecutionTime = xTaskGetTickCount(); // INICIALIZA EL TIEMPO DONDE COMENZO  LA TAREA


	for( ;; )
	{

		if( uxQueueMessagesWaiting( xQueue ) != 0 )
		{
			vPrintString( "LA COLA DEBERIA ESTAS VACIA\r\n" );
		}

		xStatus = xQueueReceive( xQueue, &medicion_actual, xTicksToWait );

		if( xStatus == pdPASS )
		{

			// ENVIA LA MEDICION POR PUERTO SERIE

			envia_medicion_uart(&medicion_actual,tiempo);

			// ACTIVA LA ALARMA SI LA TEMPERATURA ACTUAL ES MENOR A LA ALARMA BAJA

			if(medicion_actual.temperatura < medicion_actual.alarma_baja)
			{
				// ENVIA POR PUERTO SERIE LA CADENA DE CARACTERES QUE ACTIVA LA ALARMA BAJA
				vPrintString( "*L" );
				vPrintString("\r\n");
			}

			// ACTIVA LA ALARMA SI LA TEMPERATURA ACTUAL ES MAYOR A LA ALARMA ALTA

			if(medicion_actual.temperatura > medicion_actual.alarma_alta)
			{
				// ENVIA POR PUERTO SERIE LA CADENA DE CARACTERES QUE ACTIVA LA ALARMA ALTA
				vPrintString( "*H" );
				vPrintString("\r\n");

			}

			// AUMENTA LA VARIABLE TIEMPO

			tiempo++;

			/*
			if(tiempo>100)
			{
				tiempo=0;
			}
			*/
		}
		else
		{

			vPrintString( "NO SE PUEDE SACAR UN DATO DE LA COLA\r\n" );
		}

		/* BLOQUEA LA TAREA UN TIEMPO FIJO */

		vTaskDelayUntil(&xLastExecutionTime, 500);

	}
}


/*=============================			 MAIN 			=======================================*/

int main( void )
{

	// INICIALIZA EL HARDWARE

	 initHardware();

    // CREA UNA COLA DE 3 TRES ESTRUCTURAS DE MEDICION

    xQueue = xQueueCreate( 3, sizeof( medicion ) );

	if( xQueue != NULL )
	{

		// SE CREA LA TAREA QUE ENVIA DATOS DE LA COLA

		xTaskCreate( vSenderTask, "Sender1", 240, ( void * ) 100, 1, NULL );

		// SE CREA LA TAREA QUE SACA DATOS DE LA COLA

		xTaskCreate( vReceiverTask, "Receiver", 240, NULL, 2, NULL );

		// TOMA EL CONTROL EL MANEJADOR DE TAREAS

		vTaskStartScheduler();
	}
	else
	{
		// SI NO TIENE ESPACIO PARA CREAR LA COLA
	}

	for( ;; ); // CICLO DE INFINITO

	return 0;
}


/*==================		 TERMINA EL ARCHIVO		============================================*/
