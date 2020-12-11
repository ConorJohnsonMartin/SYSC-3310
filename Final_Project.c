#include <msp.h>
#include <stdio.h>

#define DEBOUNCE_TIME 1500
#define LED_RED 0
#define LED_RGB 1
#define RGB_LED_MASK 0x07
#define RGB_LED_OFFSET 0

// Functions and ISR
static void state_incrementation(void);
static void state_decrementation(void);
void PORT1_IRQHandler(void);
void EUSCIA0_IRQHandler(void);

// Variables
static uint8_t state = 0;


// Initialize GPIO
static void initialize_gpio (void)
{
    // Buttons P1.1, P1.4 on board   
    P1->DIR &= ~((1<<1) | (1<<4)); // Direction set to input
    P1->REN |= ((1<<1) | (1<<4));  // Enable resistor
    P1->OUT |= ((1<<1) | (1<<4));  // Pull direction to up
    P1->IE &= ~((1<<1) | (1<<4));  // Disable interrupts
	
    // LEDs P1.0, P2.0
    P1->DIR |= (1<<0);    // Direction set to output
    P2->DIR |= ((1<<0));  // Direction set to output
    P1->DS &= ~(1<<0);    // Disable high drive strength
    P2->DS &= ~((1<<0));  // Disable high drive strength
    P1->OUT &= ~(1<<0);   // Driven low
    P2->OUT &= ~((1<<0)); // Driven low
    P1->IE &= ~((1<<0));  // Disable interrupts
    P2->IE &= ~((1<<0));  // Disable interrupts
}

//UART ECHO example, Author: William Goh for MSP432P401
static void initialize_uart (void)
{
    CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
            CS_CTL1_SELS_3 |                // SMCLK = DCO
            CS_CTL1_SELM_3;                 // MCLK = DCO
    CS->KEY = 0;                            // Lock CS module from unintended accesses

    // Configure UART pins
    P1->SEL0 |= BIT2 | BIT3;                // set 2-UART pin as secondary function

    // Configure UART
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset
            EUSCI_B_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
    // Baud Rate calculation
    // 12000000/(16*9600) = 78.125
    // Fractional portion = 0.125
    // User's Guide Table 21-4: UCBRSx = 0x10
    // UCBRFx = int ( (78.125-78)*16) = 2
    EUSCI_A0->BRW = 78;                     // 12000000/16/9600
    EUSCI_A0->MCTLW = (2 << EUSCI_A_MCTLW_BRF_OFS) |
            EUSCI_A_MCTLW_OS16;

    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A0 RX interrupt

    // Enable sleep on exit from ISR
    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;

    // Enable global interrupt
    __enable_irq();

    // Enable EUSCIA0 interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn) & 31);
	
}

int main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // Stop watchdog timer 
    initialize_gpio(); // Setup GPIO
		initialize_uart(); // Setup UART
    
    // Setup Interrupts
    P1->IES |= ((1<<1) | (1<<4));   // Trigger Falling Edge
    P1->IFG &= ~((1<<1) | (1<<4));  // Clear Flags
    P1->IE |= ((1<<1) | (1<<4));    // Enable Interrupts
    
    NVIC_SetPriority(PORT1_IRQn, 2);
    NVIC->ICPR[PORT1_IRQn > 31] |= ((PORT1_IRQn % 32) << 1);
    NVIC_EnableIRQ(PORT1_IRQn);
    
    // Wait For Interrupt
    for (;;) {
        __WFI();
    }
    
	return 0;
}

// Function that increments the state by toggling
void state_incrementation(void)
{
		    if (state == 0) {
            P2 -> OUT ^= ((1 << 0));
            state = 1;
        }
        else if (state == 1) {
            P1 -> OUT ^= ((1 << 0));
            P2 -> OUT ^= ((1 << 0));
            state = 2;
        }
        else if (state == 2) {
            P2 -> OUT ^= ((1 << 0));
            state = 3;
        }
        else if (state == 3) {
            P1 -> OUT ^= ((1 << 0));
            P2 -> OUT ^= ((1 << 0));
            state = 0;
        }
				EUSCI_A0->TXBUF = state;
}

// Function that decrements the state by toggling
void state_decrementation(void)
{
				if (state == 2) {
            P1->OUT ^= ((1 << 0));
            P2->OUT ^= ((1 << 0));
            state = 1;
        }
        else if (state == 3) {
            P2->OUT ^= ((1 << 0));
            state = 2;
        }
        else if (state == 0) {
            P1->OUT ^= ((1 << 0));
            P2->OUT ^= ((1 << 0));
            state = 3;
        }
        else if (state == 1) {
            P2->OUT ^= ((1 << 0));
            state = 0;
        }
				EUSCI_A0->TXBUF = state;
}


// Funtion for ISR
void PORT1_IRQHandler(void)
{
    if (P1->IFG & (1<<1)) {
        // Select LED Button
        for (uint32_t i = 0; i < DEBOUNCE_TIME; i++);
        if (!(P1->IN & (1<<1))) {
            state_incrementation();
        }
        P1->IFG &= ~(1<<1);
    }
    if (P1->IFG & (1<<4)) {
        // Select Mode Button
        for (uint32_t i = 0; i < DEBOUNCE_TIME; i++);
        if (!(P1->IN & (1<<4))) {
            state_decrementation();
        }
        P1->IFG &= ~(1<<4);
    }
}

void EUSCIA0_IRQHandler(void)
{
    if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG)
    {
          if ((EUSCI_A0->RXBUF == '1') || (EUSCI_A0->RXBUF == '2')){
               if(EUSCI_A0->RXBUF == '1'){
                     state_incrementation();
               }
               if(EUSCI_A0->RXBUF == '2'){
                     state_decrementation();
               }
          EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;
				 }
    }
}