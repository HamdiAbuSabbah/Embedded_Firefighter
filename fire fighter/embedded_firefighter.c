// Global variables for sonar timing and distance measurement
unsigned int T1overflow;      // Counter for Timer1 overflow events
unsigned long T1counts;       // Timer1 counts combined with overflow
unsigned long T1time;         // Time measured by Timer1 in microseconds
unsigned long Distance;       // Calculated distance from sonar in cm
unsigned int Dcntr;           // General purpose delay counter
unsigned char cntr = 0;       // General counter

unsigned int myreading;       // Analog to Digital converted value
unsigned int myVoltage;       // Converted voltage from ADC reading

// Function declarations
void usDelay(unsigned int);         // Microsecond delay function
void Delay_ms(unsigned int);        // Millisecond delay function

void init_sonar(void);              // Initialize sonar sensor
void read_sonar(void);              // Trigger sonar sensor and read distance

void motor_right();                 // Move motors right
void motor_forward();               // Move motors forward
void motor_left();                  // Move motors left
void motor_stop();                  // Stop all motors

void right_sensor();                // Handle flame detected by right sensor (PORTD bit 0)
void middle_sensor();               // Handle flame detected by middle sensor (PORTD bit 1)
void left_sensor();                 // Handle flame detected by left sensor (PORTD bit 2)

void ATD_init(void);                // Initialize ADC (Analog to Digital Converter)
unsigned int ATD_read(void);        // Read ADC value

void pwm_init();                   // Initialize PWM for servo control
void set_servo_position1(int);    // Set servo 1 position (degrees)
void init();                      // Initialize ports and peripherals
void interrupt(void);             // Interrupt service routine
void main();                      // Main program loop

// ------------------- Function Definitions -------------------

// Initialize PWM for servo motors on CCP1 (RC2) and CCP2 (RC1)
void pwm_init() {
    TRISC = TRISC & 0xF9;     // Set RC2 and RC1 pins as output (clear bits 2 and 3)
    CCP1CON = 0x0C;           // Configure CCP1 module for PWM mode
    CCP2CON = 0x0C;           // Configure CCP2 module for PWM mode
    T2CON = T2CON | 0x07;     // Turn on Timer2 with prescaler 16
    PR2 = 249;                // Set period register for ~50Hz PWM frequency (for servos)
}

// Set servo 1 position in degrees (-90 to +90 mapped to PWM pulse width)
void set_servo_position1(int degrees) {
    int pulse_width = (degrees + 90) * 8 + 500;   // Calculate pulse width in timer counts
    CCPR1L = pulse_width >> 2;                     // Set high 8 bits of pulse width
    CCP1CON = (CCP1CON & 0xCF) | ((pulse_width & 0x03) << 4); // Set low 2 bits in CCP1CON
    Delay_ms(200);                                 // Wait 200ms for servo to reach position
}

// Set servo 2 position in degrees (-90 to +90 mapped to PWM pulse width)
void set_servo_position2(int degrees) {
    int pulse_width = (degrees + 90) * 8 + 500;   // Calculate pulse width in timer counts
    CCPR2L = pulse_width >> 2;                     // Set high 8 bits of pulse width
    CCP2CON = (CCP2CON & 0xCF) | ((pulse_width & 0x03) << 4); // Set low 2 bits in CCP2CON
    Delay_ms(200);                                 // Wait 200ms for servo to reach position
}

// Interrupt Service Routine handling multiple interrupts
void interrupt(void){
    if(INTCON & 0x04) {            // Timer0 overflow interrupt (every 1ms)
        TMR0 = 248;                // Reload Timer0 to overflow after ~1ms
        Dcntr++;                   // Increment delay counter
        if(Dcntr == 500) {         // Every 500 ms
            Dcntr = 0;
            read_sonar();          // Read sonar distance
            myreading = ATD_read(); // Read ADC value
            myVoltage = (unsigned int)(myreading * 500) / 1023;  // Convert ADC reading to voltage (0-500 scale)
        }
        INTCON = INTCON & 0xFB;    // Clear Timer0 overflow interrupt flag (T0IF)
    }
    if(PIR1 & 0x04) {              // CCP1 interrupt (not used here)
        PIR1 = PIR1 & 0xFB;        // Clear CCP1 interrupt flag
    }
    if(PIR1 & 0x01) {              // Timer1 overflow interrupt
        T1overflow++;              // Increment Timer1 overflow counter
        PIR1 = PIR1 & 0xFE;        // Clear Timer1 overflow flag
    }
    if(INTCON & 0x02) {            // External interrupt
        INTCON = INTCON & 0xFD;    // Clear external interrupt flag
    }
}

// Initialize ports and peripherals for motor and sensors
void init() {
    TRISB = 0x00;                 // Set PORTB as output (motor control)
    TRISD = 0b11101111;           // Set PORTD bit 4 as output, others inputs (flame sensors active low)
    TRISC = 0x00;                 // Set PORTC as output (for PWM)
    PORTB = 0x00;                 // Stop motors initially
    PORTD = 0xFF;                 // Set all flame sensor bits HIGH (no flame detected)
}

// Main program loop
void main() {
    ADCON1 = 0x06;           // Configure PORTA pins as digital (except RA0 for analog)
    TRISA = 0x00;            // Set PORTA as output (except RA0)

    init();                  // Initialize ports and devices
    PORTD &= 0b11101111;     // Clear bit 4 of PORTD (pump off)
    TMR0 = 248;              // Timer0 initial value

    CCP1CON = 0x00;          // Disable CCP1 module (prepare for capture)
    OPTION_REG = 0x87;       // Configure Timer0 prescaler and source for 1ms interrupt
    INTCON = 0xF0;           // Enable interrupts (TMR0, TMR1, External, peripheral)

    init_sonar();            // Initialize sonar sensor
    ATD_init();              // Initialize ADC
    pwm_init();              // Initialize PWM for servos

    while(1) {
        if (Distance < 20) {                 // If obstacle closer than 20 cm
            PORTE = 0x01 | PORTE;            // Turn off all motors (set RE0 high)
            motor_stop();                    // Stop motors
        } else {
            // Check flame sensors and react accordingly
            if (!(PORTD & 0b00000001)) {    // Right sensor flame detected (PORTD bit 0 low)
                right_sensor();
            }
            if (!(PORTD & 0b00000100)) {    // Left sensor flame detected (PORTD bit 2 low)
                left_sensor();
            }
            if (!(PORTD & 0b00000010)) {    // Middle sensor flame detected (PORTD bit 1 low)
                middle_sensor();
                if (myVoltage / 10 > 3) {   // If voltage > 30 (arbitrary threshold)
                    PORTD &= 0b11101111;    // Clear bit 4 of PORTD (pump off)
                } else {
                    int i;
                    motor_stop();            // Stop motors
                    PORTD |= 0x10;           // Set bit 4 of PORTD (pump on)
                    for(i = 0; i < 12; i++) {
                        set_servo_position1(0);     // Move servo 1 to 0 degrees
                        set_servo_position1(180);   // Move servo 1 to 180 degrees
                    }
                    PORTD &= 0b11101111;    // Clear bit 4 of PORTD (pump off)
                }
            }
        }
        motor_stop();    // Stop motors if no flame detected
    }
}

// Trigger ultrasonic sensor and measure echo time to calculate distance
void read_sonar(void) {
    T1overflow = 0;          // Reset overflow count
    TMR1H = 0;               // Clear Timer1 high byte
    TMR1L = 0;               // Clear Timer1 low byte

    PORTE = 0x04;            // Set RE2 high to trigger ultrasonic sensor
    usDelay(10);             // Wait 10 microseconds
    PORTE = 0x00;            // Clear RE2 trigger

    while (!(PORTE & 0x02)); // Wait for echo pin RE1 to go high (start timing)

    T1CON = 0x19;            // Start Timer1 with prescaler 1:2 (increment every 1us)
    while (PORTE & 0x02);    // Wait for echo pin to go low (end timing)
    T1CON = 0x18;            // Stop Timer1

    T1counts = ((TMR1H << 8) | TMR1L) + (T1overflow * 65536); // Calculate total counts

    T1time = T1counts;       // Time in microseconds
    Distance = ((T1time * 34) / 1000) / 2;  // Calculate distance in cm

    if (Distance > 100) {    // Limit max distance to 100 cm
        Distance = 100;
    }
}

// Initialize sonar sensor variables and Timer1 interrupts
void init_sonar(void) {
    T1overflow = 0;
    T1counts = 0;
    T1time = 0;
    Distance = 0;
    TMR1H = 0;
    TMR1L = 0;
    TRISE = 0x02;            // Set RE1 as input (echo), RE2 as output (trigger)
    PORTE = 0x00;            // Clear PORTE outputs
    INTCON = INTCON | 0xC0;  // Enable global and peripheral interrupts
    PIE1 = PIE1 | 0x01;      // Enable Timer1 overflow interrupt
    T1CON = 0x18;            // Timer1 off, prescaler 1:1
}

// Microsecond delay using NOP instructions (approximate)
void usDelay(unsigned int usCnt) {
    unsigned int us = 0;
    for (us = 0; us < usCnt; us++) {
        asm NOP;  // 0.5 us delay
        asm NOP;  // 0.5 us delay (total 1 us)
    }
}

// Millisecond delay
void Delay_ms(unsigned int ms) {
    unsigned int i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 1000; j++) {
            // Do nothing, just waste time
        }
    }
}

// Functions for flame sensors reacting to detected flame

void right_sensor() {
    motor_right();        // Turn motors right
    Delay_ms(100);        // Wait 100 ms to stabilize movement
}

void middle_sensor() {
    motor_forward();      // Move motors forward
    Delay_ms(100);        // Wait 100 ms
}

void left_sensor() {
    motor_left();         // Turn motors left
    Delay_ms(100);        // Wait 100 ms
}

// Motor control functions to control motor direction by setting PORTB bits

void motor_right() {
    PORTB = 0b00000101;    // Set PORTB bits for right turn motors
}

void motor_forward() {
    PORTB = 0b000000110;   // Set PORTB bits for forward motion motors
}

void motor_left() {
    PORTB = 0b00001010;    // Set PORTB bits for left turn motors
}

void motor_stop() {
    PORTB = 0b00000000;    // Clear PORTB bits to stop motors
}

// Initialize ADC module for analog input on RA0/AN0
void ATD_init(void) {
    ADCON0 = 0x41;        // Turn on ADC, select channel 0, Fosc/16
    ADCON1 = 0xCE;        // Configure ADC port (RA0 analog, others digital), right justified
    TRISA = 0x01;         // Set RA0 as input (analog)
}

// Read analog value from ADC channel 0
unsigned int ATD_read(void) {
    ADCON0 = ADCON0 | 0x04;   // Start ADC conversion (GO bit)
    while (ADCON0 & 0x04);    // Wait until conversion is done
    return ((ADRESH << 8) | ADRESL);  // Return 10-bit ADC result
}