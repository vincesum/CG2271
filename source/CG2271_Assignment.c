/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    CG2271_Assignment.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

/* Include sensor modules */
#include "ldr_sensor.h"

/* Sensor definitions */
#define SOUND_PIN      6    // PTC6 digital output from sound sensor

// LED pin numbers (using your friend's configuration)
#define RED_PIN		5	// PTD5
#define GREEN_PIN	7	// PTD7
#define BLUE_PIN	6	// PTD6

// Buzzer pin numbers
#define BUZZER_PIN 30   // PTE30 (Note: your friend had PTE31, but PTE30 is TPM0_CH3)

typedef enum tl {
	RED, GREEN, BLUE
} TLED;

/* Function prototypes */
void initSoundSensor(void);
void initGPIO(void);
void setMCGIRClk(void);
void setTPMClock(void);
void initPWM(void);
void startPWM(void);
void stopPWM(void);
void setBuzzer(int percent, int frequency);
void ledOn(TLED led);
void ledOff(TLED led);
void PORTC_PORTD_IRQHandler(void);
void delay(uint32_t count);

/*
 * @brief   Application entry point.
 */

void initGPIO() {
    // LED setup (using your friend's configuration)
    // Set up the clock gating
    SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK);

    // Set up the pin PCR values for LEDs (all on PORTD)
    PORTD->PCR[RED_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[GREEN_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[BLUE_PIN] &= ~PORT_PCR_MUX_MASK;

    PORTD->PCR[RED_PIN] = PORT_PCR_MUX(1);    // Set to ALT1
    PORTD->PCR[GREEN_PIN] = PORT_PCR_MUX(1);  // Set to ALT1
    PORTD->PCR[BLUE_PIN] = PORT_PCR_MUX(1);   // Set to ALT1

    // Set as outputs
    GPIOD->PDDR |= (1 << RED_PIN);
    GPIOD->PDDR |= (1 << GREEN_PIN);
    GPIOD->PDDR |= (1 << BLUE_PIN);

    // All LEDs OFF at startup
    ledOff(RED);
    ledOff(GREEN);
    ledOff(BLUE);
}

// Configure the MCG Internal Reference Clock
void setMCGIRClk() {
    MCG->C1 &= ~MCG_C1_CLKS_MASK;
    // Choose MCG clock source of 01 for LIRC
    // and set IRCLKEN to 1 to enable LIRC
    MCG->C1 |= ((MCG_C1_CLKS(0b01) | MCG_C1_IRCLKEN_MASK));

    // Set IRCS to 1 to choose 8 MHz clock
    MCG->C2 |= MCG_C2_IRCS_MASK;

    // Choose FCRDIV of 0 for divisor of 1
    MCG->SC &= ~MCG_SC_FCRDIV_MASK;
    MCG->SC |= MCG_SC_FCRDIV(0b0);

    // Choose LIRC_DIV2 of 0 for divisor of 1
    MCG->MC &= ~MCG_MC_LIRC_DIV2_MASK;
    MCG->MC |= MCG_MC_LIRC_DIV2(0b0);
}

void setTPMClock(){
    // Set MCGIRCLK
    setMCGIRClk();

    // Choose MCGIRCLK (8 MHz)
    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0b11);
}

// Buzzer: PTE30 TPM0 CH 3 ALT3
void initPWM() {
    // Turn on clock gating to TPM0
    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

    // Turn on clock gating to Port E (already done in initGPIO, but keeping for completeness)
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    // Set the pin multiplexor for buzzer
    PORTE->PCR[BUZZER_PIN] &= ~PORT_PCR_MUX_MASK;
    // PWM (ALT3 for TPM0_CH3)
    PORTE->PCR[BUZZER_PIN] |= PORT_PCR_MUX(0b11);

    // Set pin to output
    GPIOE->PDDR |= (1 << BUZZER_PIN);

    // Set up TPM0
    // Turn off TPM0 and clear the prescalar field
    TPM0->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);

    // Set prescalar of 8 (0b011)
    TPM0->SC |= TPM_SC_PS(0b011);

    // Select centre-aligned PWM mode
    TPM0->SC |= TPM_SC_CPWMS_MASK;

    // Initialize count to 0
    TPM0->CNT = 0;

    // Default MOD value
    TPM0->MOD = 125;

    // Configure channel 3
    // MS=10, ELS=10 for PWM signal
    TPM0->CONTROLS[3].CnSC &= ~(TPM_CnSC_MSA_MASK | TPM_CnSC_ELSA_MASK);
    TPM0->CONTROLS[3].CnSC |= (TPM_CnSC_MSB(1) | TPM_CnSC_ELSB(1));
}

void startPWM() {
    TPM0->SC |= TPM_SC_CMOD(0b01);
}

void stopPWM() {
    TPM0->SC &= ~TPM_SC_CMOD_MASK;
}

void setBuzzer(int percent, int frequency) {
    int clock_freq = CLOCK_GetBusClkFreq();
    int mod = (int) ((clock_freq/(frequency*2*8)));
    TPM0->MOD = mod;

    int value = (int)((percent / 100.0) * (double) TPM0->MOD);
    TPM0->CONTROLS[3].CnV = value; // set duty cycle on channel 3
}

void initSoundSensor(void) {
    NVIC_DisableIRQ(PORTC_PORTD_IRQn);

    // Enable clock gating for PORTC
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

    // Configure PTC6 as GPIO input with rising edge interrupt
    PORTC->PCR[SOUND_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[SOUND_PIN] |= PORT_PCR_MUX(1); // GPIO

    // Rising edge interrupt: IRQC = 1001
    PORTC->PCR[SOUND_PIN] &= ~PORT_PCR_IRQC_MASK;
    PORTC->PCR[SOUND_PIN] |= PORT_PCR_IRQC(0b1001);

    GPIOC->PDDR &= ~(1 << SOUND_PIN); // input mode

    // Enable PORTC interrupt
    PORTC->ISFR = (1 << SOUND_PIN);
    NVIC_SetPriority(PORTC_PORTD_IRQn, 2);
    NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);
    NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}

void ledOn(TLED led) {
    switch(led) {
    case RED:
        GPIOD->PSOR |= (1 << RED_PIN);  // All LEDs now on PORTD
        break;
    case GREEN:
        GPIOD->PSOR |= (1 << GREEN_PIN);
        break;
    case BLUE:
        GPIOD->PSOR |= (1 << BLUE_PIN);
        break;
    }
}

void ledOff(TLED led) {
    switch(led) {
    case RED:
        GPIOD->PCOR |= (1 << RED_PIN);  // All LEDs now on PORTD
        break;
    case GREEN:
        GPIOD->PCOR |= (1 << GREEN_PIN);
        break;
    case BLUE:
        GPIOD->PCOR |= (1 << BLUE_PIN);
        break;
    }
}

void PORTC_PORTD_IRQHandler(void) {
    if (PORTC->ISFR & (1 << SOUND_PIN)) {
        PORTC->ISFR = (1 << SOUND_PIN);  // clear flag
        
        // Flash red LED (turn on, brief delay, turn off)
        ledOn(RED);
        delay(200000);  // Flash duration
        ledOff(RED);
        
        PRINTF("Sound detected! Red LED flash!\r\n");
    }
}

void delay(uint32_t count) {
    volatile uint32_t i = 0;
    for (i = 0; i < count; ++i) {
        __asm("NOP"); /* delay */
    }
}

int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    PRINTF("=== CG2271 Multi-Sensor System ===\r\n");
    PRINTF("LDR Sensor: PTE20 (ADC0_SE0)\r\n");
    PRINTF("Sound Sensor: PTC6 (Digital)\r\n");
    PRINTF("Buzzer: PTE30 (TPM0_CH3)\r\n");
    PRINTF("LEDs: PTD5(Red), PTD7(Green), PTD6(Blue)\r\n");
    
    /* Initialize GPIO and LEDs */
    initGPIO();
    
    /* Initialize PWM for buzzer */
    PRINTF("Initializing actuators...\r\n");
    setTPMClock();
    initPWM();
    PRINTF("Buzzer initialized\r\n");
    
    /* Initialize all sensor modules */
    PRINTF("Initializing sensors...\r\n");
    
    // Initialize LDR sensor (using modular approach)
    LDR_Init();
    
    // Initialize Sound sensor
    initSoundSensor();
    PRINTF("Sound sensor initialized\r\n");
    
    // Test LEDs briefly
    PRINTF("Testing LEDs...\r\n");
    ledOn(RED);
    delay(500000);
    ledOff(RED);
    ledOn(GREEN);
    delay(500000);
    ledOff(GREEN);
    ledOn(BLUE);
    delay(500000);
    ledOff(BLUE);
    
    PRINTF("All sensors and actuators ready!\r\n");
    PRINTF("Make some noise to test sound detection with buzzer beep!\r\n");

    /* Enter an infinite loop with controlled readings */
    while(1) {
        // Trigger a single ADC reading for LDR (using modular function)
        LDR_TriggerReading();
        
        // Wait for a reasonable interval between readings (about 1 second)
        delay(1000000); // Adjust delay as needed for your desired reading frequency
    }
    return 0 ;
}


