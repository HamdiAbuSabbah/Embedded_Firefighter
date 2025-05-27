#line 1 "C:/Users/Public/Documents/Mikroelektronika/mikroC PRO for PIC/Examples/Development Systems/EasyPIC v8/Led Blinking/embedded_firefighter.c"

unsigned int T1overflow;
unsigned long T1counts;
unsigned long T1time;
unsigned long Distance;
unsigned int Dcntr;
unsigned char cntr = 0;

unsigned int myreading;
unsigned int myVoltage;


void usDelay(unsigned int);
void Delay_ms(unsigned int);

void init_sonar(void);
void read_sonar(void);

void motor_right();
void motor_forward();
void motor_left();
void motor_stop();

void right_sensor();
void middle_sensor();
void left_sensor();

void ATD_init(void);
unsigned int ATD_read(void);

void pwm_init();
void set_servo_position1(int);
void set_servo_position2(int);

void init();
void interrupt(void);
void main();




void pwm_init() {
 TRISC = TRISC & 0xF9;
 CCP1CON = 0x0C;
 CCP2CON = 0x0C;
 T2CON = T2CON | 0x07;
 PR2 = 249;
}


void set_servo_position1(int degrees) {
 int pulse_width = (degrees + 90) * 8 + 500;
 CCPR1L = pulse_width >> 2;
 CCP1CON = (CCP1CON & 0xCF) | ((pulse_width & 0x03) << 4);
 Delay_ms(200);
}


void set_servo_position2(int degrees) {
 int pulse_width = (degrees + 90) * 8 + 500;
 CCPR2L = pulse_width >> 2;
 CCP2CON = (CCP2CON & 0xCF) | ((pulse_width & 0x03) << 4);
 Delay_ms(200);
}


void interrupt(void){
 if(INTCON & 0x04) {
 TMR0 = 248;
 Dcntr++;
 if(Dcntr == 500) {
 Dcntr = 0;
 read_sonar();
 myreading = ATD_read();
 myVoltage = (unsigned int)(myreading * 500) / 1023;
 }
 INTCON = INTCON & 0xFB;
 }
 if(PIR1 & 0x04) {
 PIR1 = PIR1 & 0xFB;
 }
 if(PIR1 & 0x01) {
 T1overflow++;
 PIR1 = PIR1 & 0xFE;
 }
 if(INTCON & 0x02) {
 INTCON = INTCON & 0xFD;
 }
}


void init() {
 TRISB = 0x00;
 TRISD = 0b11101111;
 TRISC = 0x00;
 PORTB = 0x00;
 PORTD = 0xFF;
}


void main() {
 ADCON1 = 0x06;
 TRISA = 0x00;

 init();
 PORTD &= 0b11101111;
 TMR0 = 248;

 CCP1CON = 0x00;
 OPTION_REG = 0x87;
 INTCON = 0xF0;

 init_sonar();
 ATD_init();
 pwm_init();

 while(1) {
 if (Distance < 20) {
 PORTE = 0x01 | PORTE;
 motor_stop();
 } else {

 if (!(PORTD & 0b00000001)) {
 right_sensor();
 }
 if (!(PORTD & 0b00000100)) {
 left_sensor();
 }
 if (!(PORTD & 0b00000010)) {
 middle_sensor();
 if (myVoltage / 10 > 3) {
 PORTD &= 0b11101111;
 } else {
 int i;
 motor_stop();
 PORTD |= 0x10;
 for(i = 0; i < 12; i++) {
 set_servo_position1(0);
 set_servo_position1(180);
 }
 PORTD &= 0b11101111;
 }
 }
 }
 motor_stop();
 }
}


void read_sonar(void) {
 T1overflow = 0;
 TMR1H = 0;
 TMR1L = 0;

 PORTE = 0x04;
 usDelay(10);
 PORTE = 0x00;

 while (!(PORTE & 0x02));

 T1CON = 0x19;
 while (PORTE & 0x02);
 T1CON = 0x18;

 T1counts = ((TMR1H << 8) | TMR1L) + (T1overflow * 65536);

 T1time = T1counts;
 Distance = ((T1time * 34) / 1000) / 2;

 if (Distance > 100) {
 Distance = 100;
 }
}


void init_sonar(void) {
 T1overflow = 0;
 T1counts = 0;
 T1time = 0;
 Distance = 0;
 TMR1H = 0;
 TMR1L = 0;
 TRISE = 0x02;
 PORTE = 0x00;
 INTCON = INTCON | 0xC0;
 PIE1 = PIE1 | 0x01;
 T1CON = 0x18;
}


void usDelay(unsigned int usCnt) {
 unsigned int us = 0;
 for (us = 0; us < usCnt; us++) {
 asm NOP;
 asm NOP;
 }
}


void Delay_ms(unsigned int ms) {
 unsigned int i, j;
 for (i = 0; i < ms; i++) {
 for (j = 0; j < 1000; j++) {

 }
 }
}



void right_sensor() {
 motor_right();
 Delay_ms(100);
}

void middle_sensor() {
 motor_forward();
 Delay_ms(100);
}

void left_sensor() {
 motor_left();
 Delay_ms(100);
}



void motor_right() {
 PORTB = 0b00000101;
}

void motor_forward() {
 PORTB = 0b000000110;
}

void motor_left() {
 PORTB = 0b00001010;
}

void motor_stop() {
 PORTB = 0b00000000;
}


void ATD_init(void) {
 ADCON0 = 0x41;
 ADCON1 = 0xCE;
 TRISA = 0x01;
}


unsigned int ATD_read(void) {
 ADCON0 = ADCON0 | 0x04;
 while (ADCON0 & 0x04);
 return ((ADRESH << 8) | ADRESL);
}
