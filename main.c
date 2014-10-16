#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_uart.h"
#include "inc/hw_sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/ssi.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
#include "utils/uartstdio.h"
#include "usb_structs.h"
#include "usbconfig.h"
#include "nrf24l01.h"
#include "spi.h"

// Define pin to LED color mapping.
#define LED_0	GPIO_PIN_0
#define LED_1	GPIO_PIN_1
#define LED_2	GPIO_PIN_2
#define LED_3	GPIO_PIN_3

// Function prototypes
void IRQInterruptHandler(void);
void RxDataHandler();

// Buffer for RF recieve data
uint8_t ui8RxBuffer[MAX_PLOAD];
volatile bool isConfigured;

// The error routine that is called if the driver library encounters an error.
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

// UART configuration for uartstdio library
void ConfigureUART(void)
{
    // Enable the GPIO Peripheral used by the UART.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 16000000);
}

// This is the main application entry function.
int main(void)
{
    // Set the clocking to run from the PLL at 50MHz
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Enable and configure the GPIO port for the LED operation.
    // EVK Board
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, LED_0 | LED_1 | LED_2 | LED_3 );

    // Recorder is initially not configured
    isConfigured = false;

    // Initialise UART for debug
    ConfigureUART();

    // Initialise USBCDC for VCP.
    USBInit();

    // Initialize RF module for TX initially to configure recorder
    RFInit(1);

    // Set up IRQ for handling interrupts
    ROM_GPIOPinTypeGPIOInput(IRQ_BASE, IRQ);
    ROM_GPIOPadConfigSet(IRQ_BASE, IRQ, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOIntRegister(IRQ_BASE, IRQInterruptHandler);
    ROM_GPIOIntTypeSet(IRQ_BASE, IRQ, GPIO_FALLING_EDGE);
    GPIOIntClear(IRQ_BASE, GPIO_INT_PIN_7);
    GPIOIntEnable(IRQ_BASE, GPIO_INT_PIN_7); // EVK, Launchpad Board


    ROM_IntPrioritySet(INT_USB0, 0x00); // high priority
    ROM_IntPrioritySet(INT_GPIOB, 0x10); // low priority



    while(1)
    {
    	 if(isConfigured == false)
    	 {
    		 ROM_GPIOPinWrite(GPIO_PORTB_BASE, LED_0, LED_0);
    		 ROM_SysCtlDelay(ROM_SysCtlClockGet()/12);
    		 ROM_GPIOPinWrite(GPIO_PORTB_BASE, LED_0, 0);
    		 ROM_SysCtlDelay(ROM_SysCtlClockGet()/12);
    	 }
    }
}

// Data handler for RX channel
void RxDataHandler()
{
	int i;
	uint32_t numbytes;
	uint8_t data[32];
	numbytes = USBBufferDataAvailable(&RxBuffer);
	USBBufferRead(&RxBuffer, data, numbytes);
	if(isConfigured == false) // recieving configuration data
	{
		UARTprintf("Bytes received: %d\n", numbytes);
		if(data[0] == 1)
		{
			UARTprintf("Number of channels: %d\n", ((data[3] << 8) | data[2]));
			for(i = 0 ; i < ((data[3] << 8) | data[2]) ; ++i)
			{
				UARTprintf("Channel: %d\n", data[i*2 + 4]);
			}
		}
		if(data[0] == 2)
		{
			UARTprintf("Buffer Size: %d\n", ((data[3] << 8) | data[2]));
		}
		if(data[0] == 3)
		{
			UARTprintf("Sampling Frequency: %d\n", ((data[3] << 8) | data[2]));
		}
		if(data[0] == 10)
		{
			isConfigured = true;
			UARTprintf("Configuration complete\n");
			RFWriteSendBuffer(data, numbytes);
			ROM_SysCtlDelay(ROM_SysCtlClockGet()/12);
			RFInit(0); // Enable RF for RX
			return;
		}
		RFWriteSendBuffer(data, numbytes);
	}

	if(isConfigured == true && data[0] == 13) // bad number reset system
	{
		isConfigured = false;
		UARTprintf("Resetting system\n");
		RFWriteSendBuffer(data, numbytes);
		ROM_SysCtlDelay(ROM_SysCtlClockGet()/12);
		RFInit(1); // Enable RF for TX
		return;
	}

	//USBBufferFlush(&RxBuffer);
	//USBBufferWrite(&TxBuffer, data, numbytes);
}

// RF module interrupt handler. Called whenever new data is recieved.
void IRQInterruptHandler(void)
{
	uint32_t ui32Bytes;
	GPIOIntClear(IRQ_BASE, GPIO_INT_PIN_7); // EVK, Launchpad: clear interrupt flag

	SPISetCELow(); // set CE low to cease all operation

	if(RFReadRegister(READ_REG + STATUSREG) & 0x40) // read operation
	{
		// --------------- RX operation  ------------- //
		ui32Bytes = RFReadRecieveBuffer(ui8RxBuffer);
		USBBufferWrite(&TxBuffer, ui8RxBuffer, ui32Bytes);

		// Flush RX buffer
		SPISetCSNLow();
		SPIDataWrite(FLUSH_RX);
		SPIDataRead();
		SPISetCSNHigh();

		RFWriteRegister(WRITE_REG + STATUSREG, 0x40); // Clear RX_DR flag
		// --------------- RX operation  ------------- //
	}
	else if(RFReadRegister(READ_REG + STATUSREG) & 0x20) // transmit successful
	{
		RFWriteRegister(WRITE_REG + STATUSREG, 0x20); // Clear RX_DR flag
	}
	else // failed transmission
	{
		//Flush TX buffer
		SPISetCSNLow();
		SPIDataWrite(FLUSH_TX);
		SPIDataRead();
		SPISetCSNHigh();

		RFWriteRegister(WRITE_REG + STATUSREG, 0x10); // Clear MAX_RT flag

	}
	SPISetCEHigh(); // set CE high again to start all operation
}

