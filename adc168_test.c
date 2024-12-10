/**
 * @file adc168_test.c
 * @brief This file contains the implementation of a bare minimum example for interfacing with the ADC168 using an MSP430.
 *
 * The code initializes the necessary peripherals (UART, SPI, GPIO) and provides functions to send and receive data
 * to/from the ADC168. It also includes a main loop that listens for UART commands to configure and read from the ADC.
 *
 * @details
 * - Setup function initializes the watchdog timer, GPIO, clock system, UART, and SPI.
 * - Send function sends and receives data over SPI, with optional control signals.
 * - Print function sends received data over UART.
 * - Main loop listens for UART commands to configure and read from the ADC.
 * - USCI_A0 ISR handles UART receive interrupts.
 *
 * @note
 * - The code assumes an 8MHz clock frequency.
 * - The UART baud rate is set to 9600.
 * - The SPI is configured as a 3-pin, 8-bit master.
 *
 * @command
 * - 'a': Set 4 x 2 pseudo-differential operation
 * - 'b': Set 2 x 2 fully-differential operation (default)
 * - 'c': Enable 2-bit counter
 * - 'd': Disable 2-bit counter (default)
 * - 'e': Disable channel information
 * - 'f': Enable channel information (default)
 * - 'g': Enable FIFO
 * - 'h': Disable FIFO (default)
 * - 'i': Enable special read mode
 * - 'j': Disable special read mode (default)
 * - 'k': Program SEQFIFO
 * - 'l': Read SEQFIFO
 * - 'm': Toggle M0 (Control analog input channel mode)
 * - 'n': Set REFDAC 1
 * - 'o': Set REFCM
 * - 'p': Toggle M1 (Control digital output mode)
 * - 'q': Reserved
 * - 'r': Software reset
 * - 's' to 'x': Reserved
 * - 'y': Read configuration
 * - 'z': Send empty bytes
 * - '0': Read channel 0  << Your best bet to read data. Spam this a few times
 * - '1': Read channel 1  << Your best bet to read data. Spam this a few times
 * - '2': Read channel 2  << Your best bet to read data. Spam this a few times
 * - '3': Read channel 3  << Your best bet to read data. Spam this a few times
 * - '4': Convert using SEQUENCER  << Not working
 * - '5': Read FIFO  << Not working
 * - '6': Custom read sequence  << Not working
 * - '7': Use in Mode II with M0 = '0' and M1 = '1'
 *
 * @interrupt
 * - USCI_A0_ISR: Handles UART receive interrupts and toggles LED2.
 */

#include <msp430fr5969.h>
#include <stdint.h>

volatile int rx = -1;

/**
 * @brief Setup function to initialize the microcontroller's peripherals.
 *
 * This function performs the following initializations:
 * - Stops the watchdog timer.
 * - Configures GPIO pins for UART and SPI operations.
 * - Disables the GPIO power-on default high-impedance mode.
 * - Sets up the XT1 oscillator and configures the clock system.
 * - Configures USCI_A0 for UART mode with a baud rate of 9600.
 * - Configures USCI_B0 for SPI operation.
 * - Sets up various GPIO pins for SPI and other control signals.
 * - Enables global interrupts.
 */
void Setup(void) {
    WDTCTL = WDTPW | WDTHOLD;  // Stop watchdog timer

    // Configure GPIO
    P2SEL1 |= BIT0 | BIT1;  // USCI_A0 UART operation
    P2SEL0 &= ~(BIT0 | BIT1);
    P1SEL1 |= BIT6 | BIT7;  // USCI_B0 SPI operation
    P2SEL1 |= BIT2;
    PJSEL0 |= BIT4 | BIT5;  // For XT1

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    // XT1 Setup
    CSCTL0_H = CSKEY >> 8;  // Unlock CS registers
    CSCTL1 = DCOFSEL_7;     // Set DCO to 8MHz
    CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;  // set all dividers
    CSCTL4 &= ~LFXTOFF;
    do {
        CSCTL5 &= ~LFXTOFFG;  // Clear XT1 fault flag
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);  // Test oscillator fault flag
    CSCTL0_H = 0;               // Lock CS registers

    // Configure USCI_A0 for UART mode
    UCA0CTLW0 = UCSWRST;         // Put eUSCI in reset
    UCA0CTLW0 |= UCSSEL__SMCLK;  // CLK = SMCLK
    // Baud Rate calculation
    // 8000000/(16*9600) = 52.083
    // Fractional portion = 0.083
    // User's Guide Table 21-4: UCBRSx = 0x04
    // UCBRFx = int ( (52.083-52)*16) = 1
    UCA0BR0 = 52;  // 8000000/16/9600
    UCA0BR1 = 0x00;
    UCA0MCTLW |= UCOS16 | UCBRF_1;
    UCA0CTLW0 &= ~UCSWRST;  // Initialize eUSCI
    UCA0IE |= UCRXIE;       // Enable USCI_A0 RX interrupt

    // Configure USCI_B0 for SPI operation
    UCB0CTLW0 = UCSWRST;  // **Put state machine in reset**
    // UCB0CTLW0 |= UCCKPH;                  // Clock phase select. UCCKPH = CAPTURED_ONFIRST_CHANGED_ON_NEXT
    UCB0CTLW0 |= UCMST | UCSYNC | UCMSB;  // 3-pin, 8-bit SPI master
    UCB0CTLW0 |= UCSSEL__SMCLK;           // SMCLK
    UCB0BR0 = 0x10;                       // /<divider value>
    UCB0BR1 = 0;                          //
    UCB0CTLW0 &= ~UCSWRST;                // **Initialize USCI state machine**

    // nCS (1.2), CONVST (1.3), BUSY (1.4), RD (1.5), Sclk (2.2), M0 (3.0), AFE nCs (3.4)
    P3DIR |= BIT4;                // Set nCS for other device sharing SPI bus as output
    P3OUT |= BIT4;                // Set nCS high for other device sharing SPI bus
    P3DIR |= BIT0;                // Set M0 as output
    P4DIR |= BIT3;                // Set M1 as output
    P1DIR |= BIT2 | BIT3 | BIT5;  // Set nCS, CONVST, and RD as output
    P1DIR &= ~(BIT4);             // BUSY is an input
    P1REN |= BIT4;                // Enable pullup/pulldown resistor
    P1OUT &= ~BIT4;               // Pulldown
    P1DIR |= BIT0;                // LED2
    __enable_interrupt();
}

/**
 * @brief Communicate with the ADC168 over SPI.
 *
 * @param txbytes Pointer to the array of bytes to be transmitted.
 * @param rxbytes Pointer to the array where received bytes will be stored.
 * @param n Number of bytes to be transmitted and received.
 * @param convst Boolean flag to control the CONVST signal.
 * @param read Boolean flag to control the read operation.
 * @param n_extra Number of extra clock cycles to be generated at the end.
 */
void Send(uint8_t *txbytes, uint8_t *rxbytes, int n, bool convst = false, bool read = true, int n_extra = 0) {
    P1OUT &= ~BIT2;  // nCs

    if (convst) {
        P1OUT |= BIT3;   // CONVST high
        P1OUT &= ~BIT3;  // CONVST low
    }

    // Start routine
    if (read) {
        P2DIR |= BIT2;    // Configure Sclk as GPIO
        P2SEL1 &= ~BIT2;  // Configure Sclk as GPIO
        P2OUT |= BIT2;    // Sclk high
        P1OUT |= BIT5;    // Rd high
        P2OUT &= ~BIT2;   // Sclk low. Rd sees falling sclk
        P1OUT &= ~BIT5;   // Rd low
        P2SEL1 |= BIT2;   // Configure Sclk back to SPI
    }
    // End of start routine

    for (uint8_t i = 0; i < n; i++) {
        UCB0TXBUF = txbytes[i];
        while (!(UCB0IFG & UCRXIFG))
            ;
        rxbytes[i] = UCB0RXBUF;
    }

    if (n_extra != 0) {
        P2DIR |= BIT2;    // Configure Sclk as GPIO
        P2SEL1 &= ~BIT2;  // Configure Sclk as GPIO
        for (uint8_t i = 0; i < n_extra; i++) {
            P2OUT |= BIT2;   // Sclk high
            P2OUT &= ~BIT2;  // Sclk low
        }
        P2SEL1 |= BIT2;  // Configure Sclk back to SPI
    }

    P1OUT |= BIT2;  // nCs
}

/**
 * @brief Sends data over UART.
 *
 * This function takes an array of bytes and sends them over the UART interface.
 * It waits for the UART transmit buffer to be ready before sending each byte.
 *
 * @param data Pointer to the array of bytes to be transmitted.
 * @param n Number of bytes to be transmitted.
 */
void Print(uint8_t *data, int n) {
    for (uint8_t i = 0; i < n; i++) {
        while (!(UCA0IFG & UCTXIFG))
            ;
        UCA0TXBUF = data[i];
    }
}

int main(void) {
    Setup();

#define OS_CONSTANT 0  // Compensates for start routine where MSb is sent
#define OS_C1 (15 + OS_CONSTANT)
#define OS_C0 (14 + OS_CONSTANT)
#define OS_R1 (13 + OS_CONSTANT)
#define OS_R0 (12 + OS_CONSTANT)
#define OS_PD1 (11 + OS_CONSTANT)
#define OS_PD0 (10 + OS_CONSTANT)
#define OS_FE (9 + OS_CONSTANT)
#define OS_SR (8 + OS_CONSTANT)
#define OS_FC (7 + OS_CONSTANT)
#define OS_PDE (6 + OS_CONSTANT)
#define OS_CID (5 + OS_CONSTANT)
#define OS_CE (4 + OS_CONSTANT)
#define OS_A3 (3 + OS_CONSTANT)
#define OS_A2 (2 + OS_CONSTANT)
#define OS_A1 (1 + OS_CONSTANT)
#define OS_A0 (0 + OS_CONSTANT)
    // Ex. To verify the DACs settings, an RD pulse must be generated when providing a control word containing R[1:0] =
    // '01' and A[3:0] = '0011' or '0110' to initialize the read access of the appropriate DAC register.
    const uint16_t initial_config =
        0 << OS_C1 |
        0 << OS_C0 |
        0 << OS_R1 |
        1 << OS_R0 |
        0 << OS_PD1 |
        0 << OS_PD0 |
        0 << OS_FE |
        0 << OS_SR |
        0 << OS_FC |
        0 << OS_PDE |
        0 << OS_CID |
        0 << OS_CE |
        0 << OS_A3 |
        0 << OS_A2 |
        0 << OS_A1 |
        0 << OS_A0;
    uint8_t config_bytes[3] = {(uint8_t)(initial_config >> 8), (uint8_t)initial_config, 0};
    uint8_t skip_bytes[32] = {0};
    uint8_t read_channel_bytes[3] = {0, 0, 0};
    uint8_t rxbytes[32] = {0};

    while (1) {
        if (rx != -1) {
            switch (rx) {
                case 'a':  // Set 4 x 2 pseudo-differential operation
                    config_bytes[1] |= 1 << OS_PDE;
                    Send(config_bytes, rxbytes, 3);
                    Send(skip_bytes, rxbytes, 3);
                    break;
                case 'b':  // Set 2 x 2 fully-differential operation (default)
                    config_bytes[1] &= ~(1 << OS_PDE);
                    Send(config_bytes, rxbytes, 3);
                    Send(skip_bytes, rxbytes, 3);
                    break;
                case 'c':  // 2-bit counter enable
                    config_bytes[1] |= 1 << OS_CE;
                    Send(config_bytes, rxbytes, 3);
                    Send(skip_bytes, rxbytes, 3);
                    break;
                case 'd':  // 2-bit counter disable (default)
                    config_bytes[1] &= ~(1 << OS_CE);
                    Send(config_bytes, rxbytes, 3);
                    Send(skip_bytes, rxbytes, 3);
                    break;
                case 'e':  // Channel information disable
                    config_bytes[1] |= 1 << OS_CID;
                    Send(config_bytes, rxbytes, 3);
                    Send(skip_bytes, rxbytes, 3);
                    break;
                case 'f':  // Channel information enable (default)
                    config_bytes[1] &= ~(1 << OS_CID);
                    Send(config_bytes, rxbytes, 3);
                    Send(skip_bytes, rxbytes, 3);
                    break;
                case 'g':  // FIFO enable
                    config_bytes[0] |= 1 << (OS_FE - 8);
                    Send(config_bytes, rxbytes, 3);
                    Send(skip_bytes, rxbytes, 3);
                    break;
                case 'h':  // FIFO disable (default)
                    config_bytes[0] &= ~(1 << (OS_FE - 8));
                    Send(config_bytes, rxbytes, 3);
                    Send(skip_bytes, rxbytes, 3);
                    break;
                case 'i':  // Special read mode enable
                    config_bytes[0] |= 1 << (OS_SR - 8);
                    Send(config_bytes, rxbytes, 3);
                    Send(skip_bytes, rxbytes, 3);
                    break;
                case 'j':  // Special read mode disable (default)
                    config_bytes[0] &= ~(1 << (OS_SR - 8));
                    Send(config_bytes, rxbytes, 3);
                    Send(skip_bytes, rxbytes, 3);
                    break;
                case 'k':  // Program SEQFIFO. The sequencer is used in pseudo-differential mode only;
                           // this register must be set before setting the REFCM register.
                {
#define OS_SEQ_S 14
#define OS_SEQ_SL 12
#define OS_SEQ_C1 10
#define OS_SEQ_C2 8
#define OS_SEQ_C3 6
#define OS_SEQ_C4 4
#define OS_SEQ_SP 2
#define OS_SEQ_FD 0
                    config_bytes[1] |= (1 << OS_A0 | 1 << OS_A3);  // Write SEQFIFO
                    Send(config_bytes, rxbytes, 3);
                    uint8_t seqbytes[3] = {
                        0b10 << (OS_SEQ_S - 8) |       // A single CONVST is required for the entire sequence with BUSY indicating each conversion (half-clock mode only).
                            0b11 << (OS_SEQ_SL - 8) |  // The sequencer length is set to 4.
                            0b11 << (OS_SEQ_C1 - 8) |  // The first conversion is on channel 0.
                            0b10 << (OS_SEQ_C2 - 8),   // The second conversion is on channel 1.
                        0b01 << (OS_SEQ_C3) |          // The third conversion is on channel 2.
                            0b00 << (OS_SEQ_C4) |      // The fourth conversion is on channel 3.
                            0b00 << (OS_SEQ_SP) |      // READ ONLY. Indicate the setting of the pseudo-differential input multiplexer in sequencer mode
                            0b00 << (OS_SEQ_FD),       // FIFO unused
                        0};
                    Send(seqbytes, rxbytes, 3);
                    config_bytes[1] |= (1 << OS_A1);  // Read SEQFIFO
                    Send(config_bytes, rxbytes, 3);
                    Send(skip_bytes, rxbytes, 3);
                    Print(rxbytes, 3);
                    config_bytes[1] &= ~(1 << OS_A0 | 1 << OS_A1 | 1 << OS_A3);
                    break;
                }
                case 'l':  // Read SEQFIFO
                    config_bytes[1] |= (1 << OS_A0 | 1 << OS_A1 | 1 << OS_A3);
                    Send(config_bytes, rxbytes, 3);
                    Send(skip_bytes, rxbytes, 3);
                    Print(rxbytes, 3);
                    config_bytes[1] &= ~(1 << OS_A0 | 1 << OS_A1 | 1 << OS_A3);
                    break;
                case 'm':           // Control analog input channel mode
                    P3OUT ^= BIT0;  // Toggle M0
                    break;
                case 'n':  // Set REFDAC 1. We don't use REFDAC 2
                {
                    config_bytes[1] |= (1 << OS_A1);  // Write REFDAC 1
                    Send(config_bytes, rxbytes, 3);
                    uint8_t refdac_bytes[2] = {0x03, 0xFF};
                    Send(refdac_bytes, rxbytes, 3);
                    config_bytes[1] |= (1 << OS_A0);  // Read REFDAC 1
                    Send(config_bytes, rxbytes, 3);
                    Send(skip_bytes, rxbytes, 3);
                    Print(rxbytes, 3);
                    config_bytes[1] &= ~(1 << OS_A0 | 1 << OS_A1);
                    break;
                }
                case 'o':  // Set REFCM. This register must be set after setting the SEQFIFO register
                {
                    config_bytes[1] |= (1 << OS_A2 | 1 << OS_A3);  // Write REFCM
                    Send(config_bytes, rxbytes, 3);
                    uint8_t refcm_bytes[2] = {0xFF, 0};
                    Send(refcm_bytes, rxbytes, 3);
                    config_bytes[1] |= (1 << OS_A1);  // Read REFCM
                    Send(config_bytes, rxbytes, 3);
                    Send(skip_bytes, rxbytes, 3);
                    Print(rxbytes, 3);
                    config_bytes[1] &= ~(1 << OS_A1 | 1 << OS_A2 | 1 << OS_A3);
                    break;
                }
                case 'p':           // Control digital output mode
                    P4OUT ^= BIT3;  // Toggle M1
                    break;
                case 'q':
                    break;
                case 'r':  // Software reset
                    config_bytes[0] = 1 << (OS_R0 - 8);
                    config_bytes[1] = 1 << OS_A2;
                    Send(config_bytes, rxbytes, 3);
                    Send(skip_bytes, rxbytes, 3);
                    config_bytes[1] = 0;
                    break;
                case 's':
                case 't':
                case 'u':
                case 'v':
                case 'w':
                case 'x':
                    break;
                case 'y':  // Read configuration
                    config_bytes[1] |= 1 << OS_A0;
                    Send(config_bytes, rxbytes, 3);
                    config_bytes[1] &= ~(1 << OS_A0);
                    Send(skip_bytes, rxbytes, 3);
                    Print(rxbytes, 3);
                    break;
                case 'z':  // Send empty bytes
                    Send(skip_bytes, rxbytes, 3);
                    Send(skip_bytes, rxbytes, 3);
                    break;
                case '0':  // Read channel 0
                    read_channel_bytes[0] &= ~(1 << (OS_C0 - 8) | 1 << (OS_C1 - 8));
                    Send(read_channel_bytes, rxbytes, 3, true);
                    Print(rxbytes, 3);
                    break;
                case '1':  // Read channel 1
                    read_channel_bytes[0] |= 1 << (OS_C0 - 8);
                    read_channel_bytes[0] &= ~(1 << (OS_C1 - 8));
                    Send(read_channel_bytes, rxbytes, 3, true);
                    Print(rxbytes, 3);
                    break;
                case '2':  // Read channel 2
                    read_channel_bytes[0] &= ~(1 << (OS_C0 - 8));
                    read_channel_bytes[0] |= 1 << (OS_C1 - 8);
                    Send(read_channel_bytes, rxbytes, 3, true);
                    Print(rxbytes, 3);
                    break;
                case '3':  // Read channel 3
                    read_channel_bytes[0] |= 1 << (OS_C0 - 8) | 1 << (OS_C1 - 8);
                    Send(read_channel_bytes, rxbytes, 3, true);
                    Print(rxbytes, 3);
                    break;
                // Convert using SEQUENCER. Use this when in SEQFIFO mode. Never worked
                case '4':
                    Send(skip_bytes, rxbytes, 12, true, false);  // len = 20 cycles * seq length ~ 3 * seq length
                    break;
                // Read FIFO. Use this when in SEQFIFO mode. Never worked
                case '5':
                    Send(skip_bytes, rxbytes, 8);  // len = 2 * seq length * FIFO length
                    Print(rxbytes, 8);
                    break;
                // Automatic mode read sequence. Never worked
                case '6': {
                    uint8_t byte_holder[8] = {0};
                    Send(skip_bytes, byte_holder, 2, true, true, 3);
                    Send(skip_bytes, byte_holder + 2, 2, false, true, 3);
                    Send(skip_bytes, byte_holder + 4, 2, false, true, 3);
                    Send(skip_bytes, byte_holder + 6, 2, false, true, 3);
                    Print(byte_holder, 8);
                    break;
                }
                // Use in Mode II with M0 = '0' and M1 = '1'
                case '7': {
                    const uint16_t kSamples = 4;
                    const uint16_t kChannels = 2;
                    const uint16_t kBytes = 3;
                    uint8_t byte_holder[(kSamples + 1) * kChannels * kBytes] = {0};
                    const uint8_t channels[4] = {0, 0b01110000, 0b10000000, 0b11110000};  // Figure 32
                    uint8_t read_channel[kBytes] = {channels[0]};
                    for (uint16_t i = 0; i < (kSamples + 1); ++i) {
                        Send(read_channel, byte_holder + (kBytes * (2 * i)), kBytes, true, true);
                        Send(skip_bytes, byte_holder + (kBytes * ((2 * i) + 1)), kBytes, true, true);
                        read_channel[0] = channels[i % 4];
                    }
                    Print(byte_holder + (kChannels * kBytes), kSamples * kChannels * kBytes);  // Ignore first sample
                    break;
                }
            }
            rx = -1;
        }
    }

    return 0;
}

void __attribute__((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR(void) {
    switch (__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG)) {
        case USCI_NONE:
            break;
        case USCI_UART_UCRXIFG:
            rx = UCA0RXBUF;
            P1OUT ^= BIT0;  // Toggle LED2
            break;
        case USCI_UART_UCTXIFG:
            break;
        case USCI_UART_UCSTTIFG:
            break;
        case USCI_UART_UCTXCPTIFG:
            break;
    }
}
