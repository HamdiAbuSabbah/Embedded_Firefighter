
_pwm_init:

;embedded_firefighter.c,42 :: 		void pwm_init() {
;embedded_firefighter.c,43 :: 		TRISC = TRISC & 0xF9;     // Set RC2 and RC1 pins as output (clear bits 2 and 3)
	MOVLW      249
	ANDWF      TRISC+0, 1
;embedded_firefighter.c,44 :: 		CCP1CON = 0x0C;           // Configure CCP1 module for PWM mode
	MOVLW      12
	MOVWF      CCP1CON+0
;embedded_firefighter.c,45 :: 		CCP2CON = 0x0C;           // Configure CCP2 module for PWM mode
	MOVLW      12
	MOVWF      CCP2CON+0
;embedded_firefighter.c,46 :: 		T2CON = T2CON | 0x07;     // Turn on Timer2 with prescaler 16
	MOVLW      7
	IORWF      T2CON+0, 1
;embedded_firefighter.c,47 :: 		PR2 = 249;                // Set period register for ~50Hz PWM frequency (for servos)
	MOVLW      249
	MOVWF      PR2+0
;embedded_firefighter.c,48 :: 		}
L_end_pwm_init:
	RETURN
; end of _pwm_init

_set_servo_position1:

;embedded_firefighter.c,51 :: 		void set_servo_position1(int degrees) {
;embedded_firefighter.c,52 :: 		int pulse_width = (degrees + 90) * 8 + 500;   // Calculate pulse width in timer counts
	MOVLW      90
	ADDWF      FARG_set_servo_position1_degrees+0, 0
	MOVWF      R3+0
	MOVF       FARG_set_servo_position1_degrees+1, 0
	BTFSC      STATUS+0, 0
	ADDLW      1
	MOVWF      R3+1
	MOVLW      3
	MOVWF      R2+0
	MOVF       R3+0, 0
	MOVWF      R0+0
	MOVF       R3+1, 0
	MOVWF      R0+1
	MOVF       R2+0, 0
L__set_servo_position140:
	BTFSC      STATUS+0, 2
	GOTO       L__set_servo_position141
	RLF        R0+0, 1
	RLF        R0+1, 1
	BCF        R0+0, 0
	ADDLW      255
	GOTO       L__set_servo_position140
L__set_servo_position141:
	MOVLW      244
	ADDWF      R0+0, 0
	MOVWF      R3+0
	MOVF       R0+1, 0
	BTFSC      STATUS+0, 0
	ADDLW      1
	ADDLW      1
	MOVWF      R3+1
;embedded_firefighter.c,53 :: 		CCPR1L = pulse_width >> 2;                     // Set high 8 bits of pulse width
	MOVF       R3+0, 0
	MOVWF      R0+0
	MOVF       R3+1, 0
	MOVWF      R0+1
	RRF        R0+1, 1
	RRF        R0+0, 1
	BCF        R0+1, 7
	BTFSC      R0+1, 6
	BSF        R0+1, 7
	RRF        R0+1, 1
	RRF        R0+0, 1
	BCF        R0+1, 7
	BTFSC      R0+1, 6
	BSF        R0+1, 7
	MOVF       R0+0, 0
	MOVWF      CCPR1L+0
;embedded_firefighter.c,54 :: 		CCP1CON = (CCP1CON & 0xCF) | ((pulse_width & 0x03) << 4); // Set low 2 bits in CCP1CON
	MOVLW      207
	ANDWF      CCP1CON+0, 0
	MOVWF      R5+0
	MOVLW      3
	ANDWF      R3+0, 0
	MOVWF      R2+0
	MOVF       R2+0, 0
	MOVWF      R0+0
	RLF        R0+0, 1
	BCF        R0+0, 0
	RLF        R0+0, 1
	BCF        R0+0, 0
	RLF        R0+0, 1
	BCF        R0+0, 0
	RLF        R0+0, 1
	BCF        R0+0, 0
	MOVF       R0+0, 0
	IORWF      R5+0, 0
	MOVWF      CCP1CON+0
;embedded_firefighter.c,55 :: 		Delay_ms(200);                                 // Wait 200ms for servo to reach position
	MOVLW      3
	MOVWF      R11+0
	MOVLW      8
	MOVWF      R12+0
	MOVLW      119
	MOVWF      R13+0
L_set_servo_position10:
	DECFSZ     R13+0, 1
	GOTO       L_set_servo_position10
	DECFSZ     R12+0, 1
	GOTO       L_set_servo_position10
	DECFSZ     R11+0, 1
	GOTO       L_set_servo_position10
;embedded_firefighter.c,56 :: 		}
L_end_set_servo_position1:
	RETURN
; end of _set_servo_position1

_set_servo_position2:

;embedded_firefighter.c,59 :: 		void set_servo_position2(int degrees) {
;embedded_firefighter.c,60 :: 		int pulse_width = (degrees + 90) * 8 + 500;   // Calculate pulse width in timer counts
	MOVLW      90
	ADDWF      FARG_set_servo_position2_degrees+0, 0
	MOVWF      R3+0
	MOVF       FARG_set_servo_position2_degrees+1, 0
	BTFSC      STATUS+0, 0
	ADDLW      1
	MOVWF      R3+1
	MOVLW      3
	MOVWF      R2+0
	MOVF       R3+0, 0
	MOVWF      R0+0
	MOVF       R3+1, 0
	MOVWF      R0+1
	MOVF       R2+0, 0
L__set_servo_position243:
	BTFSC      STATUS+0, 2
	GOTO       L__set_servo_position244
	RLF        R0+0, 1
	RLF        R0+1, 1
	BCF        R0+0, 0
	ADDLW      255
	GOTO       L__set_servo_position243
L__set_servo_position244:
	MOVLW      244
	ADDWF      R0+0, 0
	MOVWF      R3+0
	MOVF       R0+1, 0
	BTFSC      STATUS+0, 0
	ADDLW      1
	ADDLW      1
	MOVWF      R3+1
;embedded_firefighter.c,61 :: 		CCPR2L = pulse_width >> 2;                     // Set high 8 bits of pulse width
	MOVF       R3+0, 0
	MOVWF      R0+0
	MOVF       R3+1, 0
	MOVWF      R0+1
	RRF        R0+1, 1
	RRF        R0+0, 1
	BCF        R0+1, 7
	BTFSC      R0+1, 6
	BSF        R0+1, 7
	RRF        R0+1, 1
	RRF        R0+0, 1
	BCF        R0+1, 7
	BTFSC      R0+1, 6
	BSF        R0+1, 7
	MOVF       R0+0, 0
	MOVWF      CCPR2L+0
;embedded_firefighter.c,62 :: 		CCP2CON = (CCP2CON & 0xCF) | ((pulse_width & 0x03) << 4); // Set low 2 bits in CCP2CON
	MOVLW      207
	ANDWF      CCP2CON+0, 0
	MOVWF      R5+0
	MOVLW      3
	ANDWF      R3+0, 0
	MOVWF      R2+0
	MOVF       R2+0, 0
	MOVWF      R0+0
	RLF        R0+0, 1
	BCF        R0+0, 0
	RLF        R0+0, 1
	BCF        R0+0, 0
	RLF        R0+0, 1
	BCF        R0+0, 0
	RLF        R0+0, 1
	BCF        R0+0, 0
	MOVF       R0+0, 0
	IORWF      R5+0, 0
	MOVWF      CCP2CON+0
;embedded_firefighter.c,63 :: 		Delay_ms(200);                                 // Wait 200ms for servo to reach position
	MOVLW      3
	MOVWF      R11+0
	MOVLW      8
	MOVWF      R12+0
	MOVLW      119
	MOVWF      R13+0
L_set_servo_position21:
	DECFSZ     R13+0, 1
	GOTO       L_set_servo_position21
	DECFSZ     R12+0, 1
	GOTO       L_set_servo_position21
	DECFSZ     R11+0, 1
	GOTO       L_set_servo_position21
;embedded_firefighter.c,64 :: 		}
L_end_set_servo_position2:
	RETURN
; end of _set_servo_position2

_interrupt:
	MOVWF      R15+0
	SWAPF      STATUS+0, 0
	CLRF       STATUS+0
	MOVWF      ___saveSTATUS+0
	MOVF       PCLATH+0, 0
	MOVWF      ___savePCLATH+0
	CLRF       PCLATH+0

;embedded_firefighter.c,67 :: 		void interrupt(void){
;embedded_firefighter.c,68 :: 		if(INTCON & 0x04) {            // Timer0 overflow interrupt (every 1ms)
	BTFSS      INTCON+0, 2
	GOTO       L_interrupt2
;embedded_firefighter.c,69 :: 		TMR0 = 248;                // Reload Timer0 to overflow after ~1ms
	MOVLW      248
	MOVWF      TMR0+0
;embedded_firefighter.c,70 :: 		Dcntr++;                   // Increment delay counter
	INCF       _Dcntr+0, 1
	BTFSC      STATUS+0, 2
	INCF       _Dcntr+1, 1
;embedded_firefighter.c,71 :: 		if(Dcntr == 500) {         // Every 500 ms
	MOVF       _Dcntr+1, 0
	XORLW      1
	BTFSS      STATUS+0, 2
	GOTO       L__interrupt47
	MOVLW      244
	XORWF      _Dcntr+0, 0
L__interrupt47:
	BTFSS      STATUS+0, 2
	GOTO       L_interrupt3
;embedded_firefighter.c,72 :: 		Dcntr = 0;
	CLRF       _Dcntr+0
	CLRF       _Dcntr+1
;embedded_firefighter.c,73 :: 		read_sonar();          // Read sonar distance
	CALL       _read_sonar+0
;embedded_firefighter.c,74 :: 		myreading = ATD_read(); // Read ADC value
	CALL       _ATD_read+0
	MOVF       R0+0, 0
	MOVWF      _myreading+0
	MOVF       R0+1, 0
	MOVWF      _myreading+1
;embedded_firefighter.c,75 :: 		myVoltage = (unsigned int)(myreading * 500) / 1023;  // Convert ADC reading to voltage (0-500 scale)
	MOVLW      244
	MOVWF      R4+0
	MOVLW      1
	MOVWF      R4+1
	CALL       _Mul_16X16_U+0
	MOVLW      255
	MOVWF      R4+0
	MOVLW      3
	MOVWF      R4+1
	CALL       _Div_16X16_U+0
	MOVF       R0+0, 0
	MOVWF      _myVoltage+0
	MOVF       R0+1, 0
	MOVWF      _myVoltage+1
;embedded_firefighter.c,76 :: 		}
L_interrupt3:
;embedded_firefighter.c,77 :: 		INTCON = INTCON & 0xFB;    // Clear Timer0 overflow interrupt flag (T0IF)
	MOVLW      251
	ANDWF      INTCON+0, 1
;embedded_firefighter.c,78 :: 		}
L_interrupt2:
;embedded_firefighter.c,79 :: 		if(PIR1 & 0x04) {              // CCP1 interrupt (not used here)
	BTFSS      PIR1+0, 2
	GOTO       L_interrupt4
;embedded_firefighter.c,80 :: 		PIR1 = PIR1 & 0xFB;        // Clear CCP1 interrupt flag
	MOVLW      251
	ANDWF      PIR1+0, 1
;embedded_firefighter.c,81 :: 		}
L_interrupt4:
;embedded_firefighter.c,82 :: 		if(PIR1 & 0x01) {              // Timer1 overflow interrupt
	BTFSS      PIR1+0, 0
	GOTO       L_interrupt5
;embedded_firefighter.c,83 :: 		T1overflow++;              // Increment Timer1 overflow counter
	INCF       _T1overflow+0, 1
	BTFSC      STATUS+0, 2
	INCF       _T1overflow+1, 1
;embedded_firefighter.c,84 :: 		PIR1 = PIR1 & 0xFE;        // Clear Timer1 overflow flag
	MOVLW      254
	ANDWF      PIR1+0, 1
;embedded_firefighter.c,85 :: 		}
L_interrupt5:
;embedded_firefighter.c,86 :: 		if(INTCON & 0x02) {            // External interrupt
	BTFSS      INTCON+0, 1
	GOTO       L_interrupt6
;embedded_firefighter.c,87 :: 		INTCON = INTCON & 0xFD;    // Clear external interrupt flag
	MOVLW      253
	ANDWF      INTCON+0, 1
;embedded_firefighter.c,88 :: 		}
L_interrupt6:
;embedded_firefighter.c,89 :: 		}
L_end_interrupt:
L__interrupt46:
	MOVF       ___savePCLATH+0, 0
	MOVWF      PCLATH+0
	SWAPF      ___saveSTATUS+0, 0
	MOVWF      STATUS+0
	SWAPF      R15+0, 1
	SWAPF      R15+0, 0
	RETFIE
; end of _interrupt

_init:

;embedded_firefighter.c,92 :: 		void init() {
;embedded_firefighter.c,93 :: 		TRISB = 0x00;                 // Set PORTB as output (motor control)
	CLRF       TRISB+0
;embedded_firefighter.c,94 :: 		TRISD = 0b11101111;           // Set PORTD bit 4 as output, others inputs (flame sensors active low)
	MOVLW      239
	MOVWF      TRISD+0
;embedded_firefighter.c,95 :: 		TRISC = 0x00;                 // Set PORTC as output (for PWM)
	CLRF       TRISC+0
;embedded_firefighter.c,96 :: 		PORTB = 0x00;                 // Stop motors initially
	CLRF       PORTB+0
;embedded_firefighter.c,97 :: 		PORTD = 0xFF;                 // Set all flame sensor bits HIGH (no flame detected)
	MOVLW      255
	MOVWF      PORTD+0
;embedded_firefighter.c,98 :: 		}
L_end_init:
	RETURN
; end of _init

_main:

;embedded_firefighter.c,101 :: 		void main() {
;embedded_firefighter.c,102 :: 		ADCON1 = 0x06;           // Configure PORTA pins as digital (except RA0 for analog)
	MOVLW      6
	MOVWF      ADCON1+0
;embedded_firefighter.c,103 :: 		TRISA = 0x00;            // Set PORTA as output (except RA0)
	CLRF       TRISA+0
;embedded_firefighter.c,105 :: 		init();                  // Initialize ports and devices
	CALL       _init+0
;embedded_firefighter.c,106 :: 		PORTD &= 0b11101111;     // Clear bit 4 of PORTD (pump off)
	MOVLW      239
	ANDWF      PORTD+0, 1
;embedded_firefighter.c,107 :: 		TMR0 = 248;              // Timer0 initial value
	MOVLW      248
	MOVWF      TMR0+0
;embedded_firefighter.c,109 :: 		CCP1CON = 0x00;          // Disable CCP1 module (prepare for capture)
	CLRF       CCP1CON+0
;embedded_firefighter.c,110 :: 		OPTION_REG = 0x87;       // Configure Timer0 prescaler and source for 1ms interrupt
	MOVLW      135
	MOVWF      OPTION_REG+0
;embedded_firefighter.c,111 :: 		INTCON = 0xF0;           // Enable interrupts (TMR0, TMR1, External, peripheral)
	MOVLW      240
	MOVWF      INTCON+0
;embedded_firefighter.c,113 :: 		init_sonar();            // Initialize sonar sensor
	CALL       _init_sonar+0
;embedded_firefighter.c,114 :: 		ATD_init();              // Initialize ADC
	CALL       _ATD_init+0
;embedded_firefighter.c,115 :: 		pwm_init();              // Initialize PWM for servos
	CALL       _pwm_init+0
;embedded_firefighter.c,117 :: 		while(1) {
L_main7:
;embedded_firefighter.c,118 :: 		if (Distance < 20) {                 // If obstacle closer than 20 cm
	MOVLW      0
	SUBWF      _Distance+3, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main50
	MOVLW      0
	SUBWF      _Distance+2, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main50
	MOVLW      0
	SUBWF      _Distance+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main50
	MOVLW      20
	SUBWF      _Distance+0, 0
L__main50:
	BTFSC      STATUS+0, 0
	GOTO       L_main9
;embedded_firefighter.c,119 :: 		PORTE = 0x01 | PORTE;            // Turn off all motors (set RE0 high)
	BSF        PORTE+0, 0
;embedded_firefighter.c,120 :: 		motor_stop();                    // Stop motors
	CALL       _motor_stop+0
;embedded_firefighter.c,121 :: 		} else {
	GOTO       L_main10
L_main9:
;embedded_firefighter.c,123 :: 		if (!(PORTD & 0b00000001)) {    // Right sensor flame detected (PORTD bit 0 low)
	BTFSC      PORTD+0, 0
	GOTO       L_main11
;embedded_firefighter.c,124 :: 		right_sensor();
	CALL       _right_sensor+0
;embedded_firefighter.c,125 :: 		}
L_main11:
;embedded_firefighter.c,126 :: 		if (!(PORTD & 0b00000100)) {    // Left sensor flame detected (PORTD bit 2 low)
	BTFSC      PORTD+0, 2
	GOTO       L_main12
;embedded_firefighter.c,127 :: 		left_sensor();
	CALL       _left_sensor+0
;embedded_firefighter.c,128 :: 		}
L_main12:
;embedded_firefighter.c,129 :: 		if (!(PORTD & 0b00000010)) {    // Middle sensor flame detected (PORTD bit 1 low)
	BTFSC      PORTD+0, 1
	GOTO       L_main13
;embedded_firefighter.c,130 :: 		middle_sensor();
	CALL       _middle_sensor+0
;embedded_firefighter.c,131 :: 		if (myVoltage / 10 > 3) {   // If voltage > 30 (arbitrary threshold)
	MOVLW      10
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	MOVF       _myVoltage+0, 0
	MOVWF      R0+0
	MOVF       _myVoltage+1, 0
	MOVWF      R0+1
	CALL       _Div_16X16_U+0
	MOVF       R0+1, 0
	SUBLW      0
	BTFSS      STATUS+0, 2
	GOTO       L__main51
	MOVF       R0+0, 0
	SUBLW      3
L__main51:
	BTFSC      STATUS+0, 0
	GOTO       L_main14
;embedded_firefighter.c,132 :: 		PORTD &= 0b11101111;    // Clear bit 4 of PORTD (pump off)
	MOVLW      239
	ANDWF      PORTD+0, 1
;embedded_firefighter.c,133 :: 		} else {
	GOTO       L_main15
L_main14:
;embedded_firefighter.c,135 :: 		motor_stop();            // Stop motors
	CALL       _motor_stop+0
;embedded_firefighter.c,136 :: 		PORTD |= 0x10;           // Set bit 4 of PORTD (pump on)
	BSF        PORTD+0, 4
;embedded_firefighter.c,137 :: 		for(i = 0; i < 12; i++) {
	CLRF       main_i_L4+0
	CLRF       main_i_L4+1
L_main16:
	MOVLW      128
	XORWF      main_i_L4+1, 0
	MOVWF      R0+0
	MOVLW      128
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main52
	MOVLW      12
	SUBWF      main_i_L4+0, 0
L__main52:
	BTFSC      STATUS+0, 0
	GOTO       L_main17
;embedded_firefighter.c,138 :: 		set_servo_position1(0);     // Move servo 1 to 0 degrees
	CLRF       FARG_set_servo_position1_degrees+0
	CLRF       FARG_set_servo_position1_degrees+1
	CALL       _set_servo_position1+0
;embedded_firefighter.c,139 :: 		set_servo_position1(180);   // Move servo 1 to 180 degrees
	MOVLW      180
	MOVWF      FARG_set_servo_position1_degrees+0
	CLRF       FARG_set_servo_position1_degrees+1
	CALL       _set_servo_position1+0
;embedded_firefighter.c,137 :: 		for(i = 0; i < 12; i++) {
	INCF       main_i_L4+0, 1
	BTFSC      STATUS+0, 2
	INCF       main_i_L4+1, 1
;embedded_firefighter.c,140 :: 		}
	GOTO       L_main16
L_main17:
;embedded_firefighter.c,141 :: 		PORTD &= 0b11101111;    // Clear bit 4 of PORTD (pump off)
	MOVLW      239
	ANDWF      PORTD+0, 1
;embedded_firefighter.c,142 :: 		}
L_main15:
;embedded_firefighter.c,143 :: 		}
L_main13:
;embedded_firefighter.c,144 :: 		}
L_main10:
;embedded_firefighter.c,145 :: 		motor_stop();    // Stop motors if no flame detected
	CALL       _motor_stop+0
;embedded_firefighter.c,146 :: 		}
	GOTO       L_main7
;embedded_firefighter.c,147 :: 		}
L_end_main:
	GOTO       $+0
; end of _main

_read_sonar:

;embedded_firefighter.c,150 :: 		void read_sonar(void) {
;embedded_firefighter.c,151 :: 		T1overflow = 0;          // Reset overflow count
	CLRF       _T1overflow+0
	CLRF       _T1overflow+1
;embedded_firefighter.c,152 :: 		TMR1H = 0;               // Clear Timer1 high byte
	CLRF       TMR1H+0
;embedded_firefighter.c,153 :: 		TMR1L = 0;               // Clear Timer1 low byte
	CLRF       TMR1L+0
;embedded_firefighter.c,155 :: 		PORTE = 0x04;            // Set RE2 high to trigger ultrasonic sensor
	MOVLW      4
	MOVWF      PORTE+0
;embedded_firefighter.c,156 :: 		usDelay(10);             // Wait 10 microseconds
	MOVLW      10
	MOVWF      FARG_usDelay+0
	MOVLW      0
	MOVWF      FARG_usDelay+1
	CALL       _usDelay+0
;embedded_firefighter.c,157 :: 		PORTE = 0x00;            // Clear RE2 trigger
	CLRF       PORTE+0
;embedded_firefighter.c,159 :: 		while (!(PORTE & 0x02)); // Wait for echo pin RE1 to go high (start timing)
L_read_sonar19:
	BTFSC      PORTE+0, 1
	GOTO       L_read_sonar20
	GOTO       L_read_sonar19
L_read_sonar20:
;embedded_firefighter.c,161 :: 		T1CON = 0x19;            // Start Timer1 with prescaler 1:2 (increment every 1us)
	MOVLW      25
	MOVWF      T1CON+0
;embedded_firefighter.c,162 :: 		while (PORTE & 0x02);    // Wait for echo pin to go low (end timing)
L_read_sonar21:
	BTFSS      PORTE+0, 1
	GOTO       L_read_sonar22
	GOTO       L_read_sonar21
L_read_sonar22:
;embedded_firefighter.c,163 :: 		T1CON = 0x18;            // Stop Timer1
	MOVLW      24
	MOVWF      T1CON+0
;embedded_firefighter.c,165 :: 		T1counts = ((TMR1H << 8) | TMR1L) + (T1overflow * 65536); // Calculate total counts
	MOVF       TMR1H+0, 0
	MOVWF      R0+1
	CLRF       R0+0
	MOVF       TMR1L+0, 0
	IORWF      R0+0, 0
	MOVWF      R8+0
	MOVF       R0+1, 0
	MOVWF      R8+1
	MOVLW      0
	IORWF      R8+1, 1
	MOVF       _T1overflow+1, 0
	MOVWF      R4+3
	MOVF       _T1overflow+0, 0
	MOVWF      R4+2
	CLRF       R4+0
	CLRF       R4+1
	MOVF       R8+0, 0
	MOVWF      R0+0
	MOVF       R8+1, 0
	MOVWF      R0+1
	CLRF       R0+2
	CLRF       R0+3
	MOVF       R4+0, 0
	ADDWF      R0+0, 1
	MOVF       R4+1, 0
	BTFSC      STATUS+0, 0
	INCFSZ     R4+1, 0
	ADDWF      R0+1, 1
	MOVF       R4+2, 0
	BTFSC      STATUS+0, 0
	INCFSZ     R4+2, 0
	ADDWF      R0+2, 1
	MOVF       R4+3, 0
	BTFSC      STATUS+0, 0
	INCFSZ     R4+3, 0
	ADDWF      R0+3, 1
	MOVF       R0+0, 0
	MOVWF      _T1counts+0
	MOVF       R0+1, 0
	MOVWF      _T1counts+1
	MOVF       R0+2, 0
	MOVWF      _T1counts+2
	MOVF       R0+3, 0
	MOVWF      _T1counts+3
;embedded_firefighter.c,167 :: 		T1time = T1counts;       // Time in microseconds
	MOVF       R0+0, 0
	MOVWF      _T1time+0
	MOVF       R0+1, 0
	MOVWF      _T1time+1
	MOVF       R0+2, 0
	MOVWF      _T1time+2
	MOVF       R0+3, 0
	MOVWF      _T1time+3
;embedded_firefighter.c,168 :: 		Distance = ((T1time * 34) / 1000) / 2;  // Calculate distance in cm
	MOVLW      34
	MOVWF      R4+0
	CLRF       R4+1
	CLRF       R4+2
	CLRF       R4+3
	CALL       _Mul_32x32_U+0
	MOVLW      232
	MOVWF      R4+0
	MOVLW      3
	MOVWF      R4+1
	CLRF       R4+2
	CLRF       R4+3
	CALL       _Div_32x32_U+0
	MOVF       R0+0, 0
	MOVWF      R4+0
	MOVF       R0+1, 0
	MOVWF      R4+1
	MOVF       R0+2, 0
	MOVWF      R4+2
	MOVF       R0+3, 0
	MOVWF      R4+3
	RRF        R4+3, 1
	RRF        R4+2, 1
	RRF        R4+1, 1
	RRF        R4+0, 1
	BCF        R4+3, 7
	MOVF       R4+0, 0
	MOVWF      _Distance+0
	MOVF       R4+1, 0
	MOVWF      _Distance+1
	MOVF       R4+2, 0
	MOVWF      _Distance+2
	MOVF       R4+3, 0
	MOVWF      _Distance+3
;embedded_firefighter.c,170 :: 		if (Distance > 100) {    // Limit max distance to 100 cm
	MOVF       R4+3, 0
	SUBLW      0
	BTFSS      STATUS+0, 2
	GOTO       L__read_sonar54
	MOVF       R4+2, 0
	SUBLW      0
	BTFSS      STATUS+0, 2
	GOTO       L__read_sonar54
	MOVF       R4+1, 0
	SUBLW      0
	BTFSS      STATUS+0, 2
	GOTO       L__read_sonar54
	MOVF       R4+0, 0
	SUBLW      100
L__read_sonar54:
	BTFSC      STATUS+0, 0
	GOTO       L_read_sonar23
;embedded_firefighter.c,171 :: 		Distance = 100;
	MOVLW      100
	MOVWF      _Distance+0
	CLRF       _Distance+1
	CLRF       _Distance+2
	CLRF       _Distance+3
;embedded_firefighter.c,172 :: 		}
L_read_sonar23:
;embedded_firefighter.c,173 :: 		}
L_end_read_sonar:
	RETURN
; end of _read_sonar

_init_sonar:

;embedded_firefighter.c,176 :: 		void init_sonar(void) {
;embedded_firefighter.c,177 :: 		T1overflow = 0;
	CLRF       _T1overflow+0
	CLRF       _T1overflow+1
;embedded_firefighter.c,178 :: 		T1counts = 0;
	CLRF       _T1counts+0
	CLRF       _T1counts+1
	CLRF       _T1counts+2
	CLRF       _T1counts+3
;embedded_firefighter.c,179 :: 		T1time = 0;
	CLRF       _T1time+0
	CLRF       _T1time+1
	CLRF       _T1time+2
	CLRF       _T1time+3
;embedded_firefighter.c,180 :: 		Distance = 0;
	CLRF       _Distance+0
	CLRF       _Distance+1
	CLRF       _Distance+2
	CLRF       _Distance+3
;embedded_firefighter.c,181 :: 		TMR1H = 0;
	CLRF       TMR1H+0
;embedded_firefighter.c,182 :: 		TMR1L = 0;
	CLRF       TMR1L+0
;embedded_firefighter.c,183 :: 		TRISE = 0x02;            // Set RE1 as input (echo), RE2 as output (trigger)
	MOVLW      2
	MOVWF      TRISE+0
;embedded_firefighter.c,184 :: 		PORTE = 0x00;            // Clear PORTE outputs
	CLRF       PORTE+0
;embedded_firefighter.c,185 :: 		INTCON = INTCON | 0xC0;  // Enable global and peripheral interrupts
	MOVLW      192
	IORWF      INTCON+0, 1
;embedded_firefighter.c,186 :: 		PIE1 = PIE1 | 0x01;      // Enable Timer1 overflow interrupt
	BSF        PIE1+0, 0
;embedded_firefighter.c,187 :: 		T1CON = 0x18;            // Timer1 off, prescaler 1:1
	MOVLW      24
	MOVWF      T1CON+0
;embedded_firefighter.c,188 :: 		}
L_end_init_sonar:
	RETURN
; end of _init_sonar

_usDelay:

;embedded_firefighter.c,191 :: 		void usDelay(unsigned int usCnt) {
;embedded_firefighter.c,192 :: 		unsigned int us = 0;
	CLRF       usDelay_us_L0+0
	CLRF       usDelay_us_L0+1
;embedded_firefighter.c,193 :: 		for (us = 0; us < usCnt; us++) {
	CLRF       usDelay_us_L0+0
	CLRF       usDelay_us_L0+1
L_usDelay24:
	MOVF       FARG_usDelay_usCnt+1, 0
	SUBWF      usDelay_us_L0+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__usDelay57
	MOVF       FARG_usDelay_usCnt+0, 0
	SUBWF      usDelay_us_L0+0, 0
L__usDelay57:
	BTFSC      STATUS+0, 0
	GOTO       L_usDelay25
;embedded_firefighter.c,194 :: 		asm NOP;  // 0.5 us delay
	NOP
;embedded_firefighter.c,195 :: 		asm NOP;  // 0.5 us delay (total 1 us)
	NOP
;embedded_firefighter.c,193 :: 		for (us = 0; us < usCnt; us++) {
	INCF       usDelay_us_L0+0, 1
	BTFSC      STATUS+0, 2
	INCF       usDelay_us_L0+1, 1
;embedded_firefighter.c,196 :: 		}
	GOTO       L_usDelay24
L_usDelay25:
;embedded_firefighter.c,197 :: 		}
L_end_usDelay:
	RETURN
; end of _usDelay

_Delay_ms:

;embedded_firefighter.c,200 :: 		void Delay_ms(unsigned int ms) {
;embedded_firefighter.c,202 :: 		for (i = 0; i < ms; i++) {
	CLRF       R1+0
	CLRF       R1+1
L_Delay_ms27:
	MOVF       FARG_Delay_ms_ms+1, 0
	SUBWF      R1+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Delay_ms59
	MOVF       FARG_Delay_ms_ms+0, 0
	SUBWF      R1+0, 0
L__Delay_ms59:
	BTFSC      STATUS+0, 0
	GOTO       L_Delay_ms28
;embedded_firefighter.c,203 :: 		for (j = 0; j < 1000; j++) {
	CLRF       R3+0
	CLRF       R3+1
L_Delay_ms30:
	MOVLW      3
	SUBWF      R3+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Delay_ms60
	MOVLW      232
	SUBWF      R3+0, 0
L__Delay_ms60:
	BTFSC      STATUS+0, 0
	GOTO       L_Delay_ms31
	INCF       R3+0, 1
	BTFSC      STATUS+0, 2
	INCF       R3+1, 1
;embedded_firefighter.c,205 :: 		}
	GOTO       L_Delay_ms30
L_Delay_ms31:
;embedded_firefighter.c,202 :: 		for (i = 0; i < ms; i++) {
	INCF       R1+0, 1
	BTFSC      STATUS+0, 2
	INCF       R1+1, 1
;embedded_firefighter.c,206 :: 		}
	GOTO       L_Delay_ms27
L_Delay_ms28:
;embedded_firefighter.c,207 :: 		}
L_end_Delay_ms:
	RETURN
; end of _Delay_ms

_right_sensor:

;embedded_firefighter.c,211 :: 		void right_sensor() {
;embedded_firefighter.c,212 :: 		motor_right();        // Turn motors right
	CALL       _motor_right+0
;embedded_firefighter.c,213 :: 		Delay_ms(100);        // Wait 100 ms to stabilize movement
	MOVLW      2
	MOVWF      R11+0
	MOVLW      4
	MOVWF      R12+0
	MOVLW      186
	MOVWF      R13+0
L_right_sensor33:
	DECFSZ     R13+0, 1
	GOTO       L_right_sensor33
	DECFSZ     R12+0, 1
	GOTO       L_right_sensor33
	DECFSZ     R11+0, 1
	GOTO       L_right_sensor33
	NOP
;embedded_firefighter.c,214 :: 		}
L_end_right_sensor:
	RETURN
; end of _right_sensor

_middle_sensor:

;embedded_firefighter.c,216 :: 		void middle_sensor() {
;embedded_firefighter.c,217 :: 		motor_forward();      // Move motors forward
	CALL       _motor_forward+0
;embedded_firefighter.c,218 :: 		Delay_ms(100);        // Wait 100 ms
	MOVLW      2
	MOVWF      R11+0
	MOVLW      4
	MOVWF      R12+0
	MOVLW      186
	MOVWF      R13+0
L_middle_sensor34:
	DECFSZ     R13+0, 1
	GOTO       L_middle_sensor34
	DECFSZ     R12+0, 1
	GOTO       L_middle_sensor34
	DECFSZ     R11+0, 1
	GOTO       L_middle_sensor34
	NOP
;embedded_firefighter.c,219 :: 		}
L_end_middle_sensor:
	RETURN
; end of _middle_sensor

_left_sensor:

;embedded_firefighter.c,221 :: 		void left_sensor() {
;embedded_firefighter.c,222 :: 		motor_left();         // Turn motors left
	CALL       _motor_left+0
;embedded_firefighter.c,223 :: 		Delay_ms(100);        // Wait 100 ms
	MOVLW      2
	MOVWF      R11+0
	MOVLW      4
	MOVWF      R12+0
	MOVLW      186
	MOVWF      R13+0
L_left_sensor35:
	DECFSZ     R13+0, 1
	GOTO       L_left_sensor35
	DECFSZ     R12+0, 1
	GOTO       L_left_sensor35
	DECFSZ     R11+0, 1
	GOTO       L_left_sensor35
	NOP
;embedded_firefighter.c,224 :: 		}
L_end_left_sensor:
	RETURN
; end of _left_sensor

_motor_right:

;embedded_firefighter.c,228 :: 		void motor_right() {
;embedded_firefighter.c,229 :: 		PORTB = 0b00000101;    // Set PORTB bits for right turn motors
	MOVLW      5
	MOVWF      PORTB+0
;embedded_firefighter.c,230 :: 		}
L_end_motor_right:
	RETURN
; end of _motor_right

_motor_forward:

;embedded_firefighter.c,232 :: 		void motor_forward() {
;embedded_firefighter.c,233 :: 		PORTB = 0b000000110;   // Set PORTB bits for forward motion motors
	MOVLW      6
	MOVWF      PORTB+0
;embedded_firefighter.c,234 :: 		}
L_end_motor_forward:
	RETURN
; end of _motor_forward

_motor_left:

;embedded_firefighter.c,236 :: 		void motor_left() {
;embedded_firefighter.c,237 :: 		PORTB = 0b00001010;    // Set PORTB bits for left turn motors
	MOVLW      10
	MOVWF      PORTB+0
;embedded_firefighter.c,238 :: 		}
L_end_motor_left:
	RETURN
; end of _motor_left

_motor_stop:

;embedded_firefighter.c,240 :: 		void motor_stop() {
;embedded_firefighter.c,241 :: 		PORTB = 0b00000000;    // Clear PORTB bits to stop motors
	CLRF       PORTB+0
;embedded_firefighter.c,242 :: 		}
L_end_motor_stop:
	RETURN
; end of _motor_stop

_ATD_init:

;embedded_firefighter.c,245 :: 		void ATD_init(void) {
;embedded_firefighter.c,246 :: 		ADCON0 = 0x41;        // Turn on ADC, select channel 0, Fosc/16
	MOVLW      65
	MOVWF      ADCON0+0
;embedded_firefighter.c,247 :: 		ADCON1 = 0xCE;        // Configure ADC port (RA0 analog, others digital), right justified
	MOVLW      206
	MOVWF      ADCON1+0
;embedded_firefighter.c,248 :: 		TRISA = 0x01;         // Set RA0 as input (analog)
	MOVLW      1
	MOVWF      TRISA+0
;embedded_firefighter.c,249 :: 		}
L_end_ATD_init:
	RETURN
; end of _ATD_init

_ATD_read:

;embedded_firefighter.c,252 :: 		unsigned int ATD_read(void) {
;embedded_firefighter.c,253 :: 		ADCON0 = ADCON0 | 0x04;   // Start ADC conversion (GO bit)
	BSF        ADCON0+0, 2
;embedded_firefighter.c,254 :: 		while (ADCON0 & 0x04);    // Wait until conversion is done
L_ATD_read36:
	BTFSS      ADCON0+0, 2
	GOTO       L_ATD_read37
	GOTO       L_ATD_read36
L_ATD_read37:
;embedded_firefighter.c,255 :: 		return ((ADRESH << 8) | ADRESL);  // Return 10-bit ADC result
	MOVF       ADRESH+0, 0
	MOVWF      R0+1
	CLRF       R0+0
	MOVF       ADRESL+0, 0
	IORWF      R0+0, 1
	MOVLW      0
	IORWF      R0+1, 1
;embedded_firefighter.c,256 :: 		}
L_end_ATD_read:
	RETURN
; end of _ATD_read
