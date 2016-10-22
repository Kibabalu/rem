/*------------------------------------------------------------------------------------------------*/
/*	Projekt 'rem'	remote switch for Mechanika IV coffee machine	                              */
/*					                                                                              */
/*					Hardware:  Arduino Pro Micro (AVR ATmega32U4)                                 */
/*								                                                                  */
/*								                                                       			  */
/*																							   	  */
/*	Frank Kirschbaum (frank.kirschbaum@me.com)                                 					  */
/*																								  */
/*																								  */
/*	Kibas Coding Standard and Style Guide:                                    					  */
/*  (frei nach http://www.freertos.org/FreeRTOS-Coding-Standard-and-Style-Guide.html)			  */
/*																								  */
/*	Namenskonventionen:																			  */
/*	Präfixes für Konstanten, Variablen, Funktionen und Methoden:								  */
/*	void/void*				   v/pv        void											         */
/*	int/int*				   i/pi		    integer												*/
/*	uint/uint*				   ui/pui	   unsigned integer								          */
/*	int8_t/int8_t*			   c/pc		   char (Byte)											  */
/*	uint8_t/uint8_t*		   uc/puc		unsigned char										 */
/*	int16_t/int16_t*		   s/ps		    short												 */
/*	uint16_t/uint16_t*         us/pus		unsigned short									       */
/*	int32_t/int32_t*		   l/pl		    long										        */
/*	uint32_t/uint32_t*		   ul/pul		unsigned long									*/
/*	char/unsigned char		   uc/puc		char (byte) für Zeichen							*/
/*	float/float*			   f/pf		     float									*/
/*	double/double*			    d/pd		double										*/
/*	BaseType_t/BaseType_t*		x/px		base type, optimal für Registerbreite				*/
/*	UBaseType_t/UBaseType_t*	ux/pux		unsigned base type, optimal für Registerbreite        */
/*	TickType_t/TickType_t*		x/px		16/32 Bit, abhängig von Registerbreite				*/
/*	size_t/size_t*				x/px																																*/
/*	TaskHandle_t				pv			Task-handle (Pointer) für die Referenzierung von Tasks	*/
/*	SemaphoreHandle_t			x																																		*/
/*	Postfix:																																											*/
/*	class member variables		XYZ_		Unterstrich am Ende jeder Member-Variablen									*/
/*																																																*/
/*	Lesbarkeit des Quelltextes:																																		*/
/*	Space nach ( und vor )																																				*/
/*	Space nach [ und vor ]																																				*/
/*	unter jeder Funktionsdeklaration etc. ----...---                              */
/*																																																*/
/*	'Der Unterschied zwischen Theorie und Praxis ist in der Praxis größer als in der Theorie'			*/
/*																																																*/
/*------------------------------------------------------------------------------------------------*/

#define DEBUG 1                     // enable/disable debug mode

#include <Arduino.h>
#include <DebugMacro.h>             // dprint(x) and dshow("Blablubb");
#include <RegisterBitsMacros.h>     // fast direct manipulation of registers
#include <SPI.h>                    // supporting SPI interface
#include <FreeRTOS_AVR.h>           // FreeRTOS - real time operating system
#include <DigitalIO.h>              // here: support for I2C communication

#define RXLED 17  // The RX LED has a defined Arduino pin
// The TX LED was not so lucky, we'll need to use pre-defined
// macros (TXLED1, TXLED0) to control that.
// (We could use the same macros for the RX LED too -- RXLED1,
//  and RXLED0.)

/*------------------------------------------------------------------------------------------------*/
/*
 * declarations
 */
uint8_t ucLedState1 = LOW;                          // state of LED1, toggles from time to time

TaskHandle_t pvTask1s;                              // handle for 1s task
TaskHandle_t pvTask100ms;                           // handle for 100ms task
TaskHandle_t pvTask10ms;                           // handle for 100ms task

const PROGMEM uint32_t ulBaud = 9600;                // Baud rate for serial communication

const PROGMEM uint16_t iIntervalTicks1s = 977;      // sampling time in units of 1024 usec
const PROGMEM uint16_t iIntervalTicks100ms = 98;    // sampling time in units of 1024 usec
const PROGMEM uint16_t iIntervalTicks10ms = 10;    // sampling time in units of 1024 usec

/*------------------------------------------------------------------------------------------------*/
/*
 * function prototypes
 */
static void vTask1s( void *arg );                   // 1s task
static void vTask100ms( void *arg );                // 100ms task
static void vTask10ms( void *arg );                 // 10ms task
void setup();                                       // setup function

/*------------------------------------------------------------------------------------------------*/
/*
 * 1s task
 */
static void vTask1s( void *arg )
{
    // initialise the ticks variable with the current time.
    TickType_t ticks = xTaskGetTickCount( );

    while ( 1 )
    {
        vTaskDelayUntil( &ticks, iIntervalTicks1s );
        // wait until time for next task run

        Serial.println("Hello world from 1s task");  // Print "Hello World" to the Serial Monitor
        Serial1.println("Hello from 1s task!");  // Print "Hello!" over hardware UART

        ucLedState1 = ucLedState1 == LOW ? HIGH : LOW; // toggle ucLedState1

        digitalWrite( RXLED, ucLedState1 ); // write ucLedState1 to output ucLedPin1

        if(ucLedState1)
        {
            TXLED1;         //TX LED is not tied to a normally controlled pin
        }
        else
        {
            TXLED0;         //TX LED is not tied to a normally controlled pin
        }
    }
}

/*------------------------------------------------------------------------------------------------*/
/*
 * 100ms task
 */
static void vTask100ms( void *arg )
{
    // initialise the ticks variable with the current time.
    TickType_t ticks = xTaskGetTickCount( );

    while ( 1 )
    {
        vTaskDelayUntil( &ticks, iIntervalTicks100ms );
        // wait until time for next task run

        Serial.println("Hello world from 100ms task");  // Print "Hello World" to the Serial Monitor
        Serial1.println("Hello from 100ms task!");  // Print "Hello!" over hardware UART
    }
}

/*------------------------------------------------------------------------------------------------*/
/*
 * 10ms task
 */
static void vTask10ms( void *arg )
{
    // initialise the ticks variable with the current time.
    TickType_t ticks = xTaskGetTickCount( );

    while ( 1 )
    {
        vTaskDelayUntil( &ticks, iIntervalTicks10ms );
        // wait until time for next task run

        //Serial.println("Hello world from 10ms task");  // Print "Hello World" to the Serial Monitor
        //Serial1.println("Hello from 10ms task!");  // Print "Hello!" over hardware UART
    }
}

/*------------------------------------------------------------------------------------------------*/
/*
 * setup function
 */
void setup()
{
    pinMode(RXLED, OUTPUT);  // Set RX LED as an output
    // TX LED is set as an output behind the scenes

    Serial.begin( ulBaud ); //This pipes to the serial monitor
    Serial1.begin( ulBaud ); //This is the UART, pipes to sensors attached to board

    // task creation status
    portBASE_TYPE s1, s2, s3;     // return variables for the RTOS task creation

    // setting up the RTOS tasks and starting the scheduler

    s1 = xTaskCreate( vTask1s, NULL, configMINIMAL_STACK_SIZE, NULL, 3, &pvTask1s ); // create 1s task at priority 3
    s2 = xTaskCreate( vTask100ms, NULL, configMINIMAL_STACK_SIZE, NULL, 2, &pvTask100ms ); // create 100ms task at priority 2
    s3 = xTaskCreate( vTask10ms, NULL, configMINIMAL_STACK_SIZE, NULL, 1, &pvTask10ms ); // create 10ms task at priority 2

    if ( s1 != pdPASS || s2 != pdPASS || s3 != pdPASS ) // check for creation errors
    {
        dshow( "Creation problem" );
        while( 1 );
    }

    // throw away serial input
    //while (Serial.read() >= 0);
    //Serial.println(F("Type any character to end"));

    // start scheduler
    dshow( "starting scheduler ..." );
    vTaskStartScheduler( );      // starting the scheduler
    dshow( "Insufficient RAM" );
    while( 1 );
}

/*------------------------------------------------------------------------------------------------*/
/*
 * main loop
 */
 void loop( )
 {
	 // Not used - idle loop has a very small, configMINIMAL_STACK_SIZE, stack
	 // loop must never block
 }
