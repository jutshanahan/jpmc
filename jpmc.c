// JPMC
// Jut Shanahan, Paul Karplus Motor Controller
// Copyright: 2012
//###########################################################################
// Description: Piccolo Control Stick with F28069 Processor
//
// Rev		Date		Edited By		Revision Notes
// 0.01		8/15/2012	Jut				Initial rev. Basic serial command functionality working. Delete, printf, and several other things implemented.
// 0.02		8/15/2012	PK				Moved dispatch_console command to the SCIA rx interrupt. Added a command line function that blinks the LED on the controlStick. Cleaned up and commented code.
// 0.03		8/16/2012	Jut				Added command to set led blink frequency
// 0.04		8/18/2012	Jut				Added command to get value at any data memory location.  Added UART color defines.
//###########################################################################
#define REV "0.04"

#include "DSP28x_Project.h"     	   // Device Headerfile and Examples Include File
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/cmdline.h"
#include "console.h"
#include "utils/uartstdio.h"
#include "uartcolors.h"


// Prototype statements for functions found within this file.
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void scia_rx_isr(void);					// sci receive interrupt function
void init(void);

// Initialize global variables for this file
float Freq = 1;   // frequency of timer0 interrupt in Hz


void main(void)
{
	init();			// Sets up the sci and GPIO registers

	// Print bootup message containing device title and revision number
	UARTprintf("\r\n");
	UARTprintf(BOLDCYAN);
	UARTprintf("JSPK Motor Controller\r\n");
	UARTprintf("Rev %s\r\n",REV);
	UARTprintf(restore);

//	UARTprintf("\033[36mJSPK Motor Controller\033[37m\r\n");
//	UARTprintf("\033[36mRev %s\033[37m\r\n",REV);
	CmdLineProcess("help");
	UARTprintf("> ");

	// Infinite loop.
   while(1){}
}

// Initial setup of all registers
void init(void)
{
	// Initialize System Control:
	// PLL, WatchDog, enable Peripheral Clocks
	InitSysCtrl();

	// Initialize Sci GPIO
	InitSciaGpio();

	// init pwn
	EALLOW;

	/* Disable internal pull-up for the selected output pins
	for reduced power consumption */
	// Pull-ups can be enabled or disabled by the user.
	// Comment out other unwanted lines.

	GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
	GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)

	/* Configure EPWM-1 pins using GPIO regs*/
	// This specifies which of the possible GPIO pins will be EPWM1 functional pins.
	// Comment out other unwanted lines.

	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B

	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;    // disable pwm clock

//	EPwm1Regs.TBPRD = 600; // Period = 8000 TBCLK counts = 10kHz
//	EPwm1Regs.CMPA.half.CMPA = 500; // Compare A = 350 TBCLK counts
//	EPwm1Regs.CMPB = 300; // Compare B = 0 TBCLK counts
//	EPwm1Regs.TBPHS.all = 0; // Set Phase register to zero
//	EPwm1Regs.TBCTR = 0; // clear TB counter
//	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
//	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Phase loading disabled
//	EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
//	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
//	EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; // TBCLK = SYSCLK
//	EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
//	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
//	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
//	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR = Zero
//	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR = Zero
//	EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;    // when TBCTR = 0, set ePWMxA high
//	EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;  // when TBCTR = CMPA, clear ePWMxA low
//	EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;    // when TBCTR = 0, set ePWMxB high
//	EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;  // when TBCTR = CMPB, clear ePWMxB low


	//=====================================================================
	// Configuration
	//=====================================================================
	// Initialization Time
	//========================// EPWM Module 1 config
	EPwm1Regs.TBPRD = 800; // Period = 800 TBCLK counts
	EPwm1Regs.TBPHS.half.TBPHS = 0; // Set Phase register to zero
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Master module
	EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // Sync down-stream module
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
	EPwm1Regs.AQCTLA.bit.CAU = AQ_SET; // set actions for EPWM1A
	EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
	EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active Hi complementary
	EPwm1Regs.DBFED = 0; // FED = 50 TBCLKs
	EPwm1Regs.DBRED = 0; // RED = 50 TBCLKs
	// EPWM Module 2 config
	EPwm2Regs.TBPRD = 800; // Period = 1600 TBCLK counts
	EPwm2Regs.TBPHS.half.TBPHS = 0; // Set Phase register to zero
	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
	EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Slave module
	EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
	EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
	EPwm2Regs.AQCTLA.bit.CAU = AQ_SET; // set actions for EPWM2A
	EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
	EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active Hi complementary
	EPwm2Regs.DBFED = 50; // FED = 50 TBCLKs
	EPwm2Regs.DBRED = 50; // RED = 50 TBCLKs
	// EPWM Module 3 config
	EPwm3Regs.TBPRD = 800; // Period = 1600 TBCLK counts
	EPwm3Regs.TBPHS.half.TBPHS = 0; // Set Phase register to zero
	EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
	EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Slave module
	EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
	EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
	EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
	EPwm3Regs.AQCTLA.bit.CAU = AQ_SET; // set actions for EPWM3A
	EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm3Regs.DBCTL.bit.IN_MODE = DB_FULL_ENABLE; // enable Dead-band module
	EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active Hi complementary
	EPwm3Regs.DBFED = 50; // FED = 50 TBCLKs
	EPwm3Regs.DBRED = 50; // RED = 50 TBCLKs
	// Run Time (Note: Example execution of one run-time instant)
	//=========================================================
	EPwm1Regs.CMPA.half.CMPA = 500; // adjust duty for output EPWM1A
	EPwm2Regs.CMPA.half.CMPA = 600; // adjust duty for output EPWM2A
	EPwm3Regs.CMPA.half.CMPA = 700; // adjust duty for output EPWM3A


	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;

	EDIS;

	// Clear all interrupts and initialize PIE vector table. Disable CPU interrupts
	DINT;

	// Disable CPU interrupts and clear all CPU interrupt flags:
	IER = 0x0000;
	IFR = 0x0000;

	// Initialize the PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.
	InitPieCtrl();

	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in F2806x_DefaultIsr.c.
	// This function is found in F2806x_PieVect.c.
	InitPieVectTable();

	// Interrupts that are used in this file are re-mapped to
	// ISR functions found within this file.
	EALLOW;  // This is needed to write to EALLOW protected registers
	PieVectTable.TINT0 = &cpu_timer0_isr;
	PieVectTable.TINT1 = &cpu_timer1_isr;
	PieVectTable.TINT2 = &cpu_timer2_isr;
	PieVectTable.SCIRXINTA = &scia_rx_isr;		// SciA rx interrupt
	EDIS;    // This is needed to disable write to EALLOW protected registers

	// Step 4. Initialize the Device Peripheral.
	InitCpuTimers();   // For this file, only initialize the Cpu Timers

	// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
	// 80MHz CPU Freq, 1 second Period (in uSeconds)
	ConfigCpuTimer(&CpuTimer0, 80, 1.0/Freq*1000000.0);
	ConfigCpuTimer(&CpuTimer1, 80, 1000000);
	ConfigCpuTimer(&CpuTimer2, 80, 1000000);

	// To ensure precise timing, use write-only instructions to write to the entire register. Therefore, if any
	// of the configuration bits are changed in ConfigCpuTimer and InitCpuTimers (in F2806x_CpuTimers.h), the
	// below settings must also be updated.
	CpuTimer0Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0 (starts timer) and TIE=1
	CpuTimer1Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0 (starts timer) and TIE=1
	CpuTimer2Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0 (starts timer) and TIE=1

	// Enable CPU int1 which is connected to CPU-Timer 0, CPU int 9 which is connected to SCIA_rx and SCIA_tx, CPU int13
	// which is connected to CPU-Timer 1, and CPU int 14, which is connected
	// to CPU-Timer 2:
	IER |= M_INT1;    // enable group 1 interrupts
	IER |= M_INT9;    // enable group 9 interrupts (scia rx and tx)
	IER |= M_INT13;   // enable group 13 interrupts
	IER |= M_INT14;   // enable group 14 interrupts

	// Enable individual PIE Table interrupts
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;		// Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;		// Enable SCIRXINTA in the PIE: Group 9 interrupt 1


	// Enable global Interrupts and higher priority real-time debug events:
	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM

	// Intialize UART(INT_SCIRXINTA, UARTStdioIntHandler);
	UARTStdioInitExpClk(0,9600);

	// Enable Sci tx and rx interupts within Sci control register
	SciaRegs.SCICTL2.bit.TXINTENA = 1;
	SciaRegs.SCICTL2.bit.RXBKINTENA = 1;

	// Setup LED on GPIO-34 for Sci blink command. LED is toggled by blink command is sent.
	EALLOW;
	GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;	// 0=GPIO,  1=COMP2OUT,  2=EMU1,  3=Resv
	GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;		// 1=OUTput,  0=INput
	GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;	// uncomment if --> Set Low initially
	//GpioDataRegs.GPBSET.bit.GPIO34 = 1;	// uncomment if --> Set High initially
	EDIS;
}

// Sci rx interrupt function.
__interrupt void scia_rx_isr(void)
{
	dispatch_console();							// Processes the received data.
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;		// Acknowledges the PIE group so that another interrupt can occur
	//SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;		// (PK) I tried commenting this out and it did not affect performance. So it must not be necessary.
	//SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;		// (PK) I tried commenting this out and it did not affect performance. So it must not be necessary.
}
__interrupt void cpu_timer0_isr(void)
{
   CpuTimer0.InterruptCount++;
//   UARTprintf("cpu_timer0_isr %u\r\n",CpuTimer0.InterruptCount);
   GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;



//   if(SciaRegs.SCIFFRX.bit.RXFFINT)
//	   UARTprintf("SciaRegs.SCIFFRX.bit.RXFFINT = 1\r\n");
//   else
//	   UARTprintf("SciaRegs.SCIFFRX.bit.RXFFINT = 0\r\n");
//
//
//   if(PieCtrlRegs.PIEIFR9.bit.INTx1)
//	   UARTprintf("PieCtrlRegs.PIEIFR9.bit.INTx1 = 1\r\n");
//   else
//	   UARTprintf("PieCtrlRegs.PIEIFR9.bit.INTx1 = 0\r\n");


   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void cpu_timer1_isr(void)
{
   CpuTimer1.InterruptCount++;
   // The CPU acknowledges the interrupt.
   EDIS;
}

__interrupt void cpu_timer2_isr(void)
{  EALLOW;
   CpuTimer2.InterruptCount++;
   // The CPU acknowledges the interrupt.
   EDIS;
}


//===========================================================================
//
//===========================================================================
