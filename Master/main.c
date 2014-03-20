/* GAIWR - an application for the Pololu Baby Orangutan B
 *
 * This application uses the Pololu AVR C/C++ Library.  For help, see:
 * -User's guide: http://www.pololu.com/docs/0J20
 * -Command reference: http://www.pololu.com/docs/0J18
 *
 * Created: 4/26/2012 10:44:19 AM
 *  Author: jleichty / nwiltsie/ skalouche
 */

#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <pololu/orangutan.h>

// adjust F_CPU and SCL_CLOCK in twimaster.c

// robot parameters
#define NUM_RC 4
#define NUM_MOTORS 2
#define SERIAL_BAUD_RATE 115200
#define FAST_PERIOD_TICKS 2500      // at 20MHz, ticks are 0.4us
#define SLOW_PERIOD_MS 50


// GAIWR defines
#define LA_1_ON 50 //Analog value for LA 1
#define LA_1_OFF 200 //Analog value for LA 1
#define LA_1_RELEASE 180 //Analog value for LA 1
#define LA_2_ON 50 //Analog value for LA 1
#define LA_2_OFF 200 //Analog value for LA 1
#define LA_2_RELEASE 180 //Analog value for LA 1

#define WRIST_1_DOWN -3 // Angle for wrist 1 touchdown
#define WRIST_1_UP 90 // Angle for wrist 1 liftoff
#define WRIST_2_DOWN -3 // Angle for wrist 1 touchdown
#define WRIST_2_UP 90 // Angle for wrist 1 liftoff

#define extend 1 // direction for rack and pinion
#define contract -1 // direction for rack and pinion
#define RAP_speed 100// motor power for maxon rack and pinion motor


#define POS_ERR_TOL 5
#define RACK_ERR_TOL 20
#define LA_SPEED 100

#define ON 1
#define OFF 0
#define CHECK 2

// serial
char receive_buffer[5];
char send_buffer[256];
volatile int command_changed;

// timekeeping
uint32_t prev_ticks;
uint32_t current_ticks;
uint32_t last_fast_period_ticks;
uint32_t lfp_actual;
uint32_t prev_ms;
uint32_t current_ms;
uint32_t last_slow_period_ms;

// encoder gray code storage
uint8_t enc_gray[NUM_MOTORS];
int8_t enc_lookup[16] = {0,-1, 1, 0, 1, 0, 0,-1,-1, 0, 0, 1, 0, 1,-1, 0};

// RC pulse data
int16_t RC[NUM_RC];
// Combined RC pulse data
int16_t RC_motor[NUM_MOTORS];
// RC pulse timestamp storage
uint32_t RC_pulse_timestamp[NUM_RC];
// port state storage for changed pin identification
uint8_t portB_storage;
uint8_t portD_storage;

// Servo commands
uint16_t servo_cmds[3];
// Linear actuator commands
uint8_t la_cmds[2];

// motor data layout
typedef struct {
    long  pos;               // current position
    long  pos_tgt;           // position target
    float    pos_tgt_acc;       // position target accumulator
    int32_t  pos_err;      // position error
    int32_t  int_err_acc;       // integral error accumulator
    int16_t  int_err;           // integral error / 256
    uint8_t  pwm;               // PWM level to drive motor
    int8_t   dir;               // direction to drive motor
    uint16_t Kp;                // controller position gain
    uint16_t Ki;                // controller integral gain
    uint16_t Kd;                // controller derivative gain
    uint16_t IL;                // controller integration limit
    uint8_t  OL;                // PWM output limit
    uint8_t  SRD;               // servo rate divisor for derivative gain
    uint8_t  DB;                // amplifier deadband compensation
} motor;
// motor array
motor M[NUM_MOTORS];

// interrupt on Port C input change, read encoders
ISR(PCINT1_vect) {
    uint8_t i;

    // store all the old encoder states by shifting left
    for (i = 0; i < NUM_MOTORS; i++) {
        enc_gray[i] <<= 2;
    }

    // after the below operations, the lower 4 bits of enc_gray[i] point to
    // the appropriate increment/decrement element in enc_lookup

    // encoder 1 (A = PC0, B = PC1)
    enc_gray[0] |= (PINC & 0b00001100) >> 2;
    // encoder 2 (A = PC2, B = PC3)
    enc_gray[1] |= (PINC & 0b00000011);

    // update all the encoder counters, index is lower 4 bits of enc_gray[i]
    for (i = 0; i < NUM_MOTORS; i++) {
        M[i].pos += enc_lookup[enc_gray[i] & 0x0F];
    }
}

// interrupt on Port B input change, read Port B receiver inputs
ISR(PCINT0_vect) {
    uint32_t timestamp;

    // record the time
    timestamp = get_ticks();

    // check if PB0 changed (not currently necessary)
    if ((PINB ^ portB_storage) & 0b00000001) {
        if (PINB & 0b00000001) {
            // record the rising edge time
            RC_pulse_timestamp[3] = timestamp;
        } else {
            // record the pulse length in ticks
            RC[3] = timestamp - RC_pulse_timestamp[3];
        }
    }

    // record Port B value for next time
    command_changed = 1;
    portB_storage = PINB;
}

// interrupt on Port D input change, read Port D receiver inputs
ISR(PCINT2_vect) {
    uint32_t timestamp;

    // record the time
    timestamp = get_ticks();

    // check if PD2 changed
    if ((PIND ^ portD_storage) & 0b00000100) {
        if (PIND & 0b00000100) {
            // record the rising edge time
            RC_pulse_timestamp[0] = timestamp;
        } else {
            // record the pulse length in ticks
            RC[0] = timestamp - RC_pulse_timestamp[0];
        }
    }
    // check if PD4 changed
    if ((PIND ^ portD_storage) & 0b00010000) {
        if (PIND & 0b00010000) {
            RC_pulse_timestamp[1] = timestamp;
        } else {
            RC[1] = timestamp - RC_pulse_timestamp[1];
        }
    }
    // check if PD7 changed
    if ((PIND ^ portD_storage) & 0b10000000) {
        if (PIND & 0b10000000) {
            RC_pulse_timestamp[2] = timestamp;
        } else {
            RC[2] = timestamp - RC_pulse_timestamp[2];
        }
    }

    // record Port D value for next time
    command_changed = 1;
    portD_storage = PIND;
}



void send_command(){
    // 111222333Ll or 090:-90:005:1:0 (Servo1 90, Servo2 -90, Servo3 5, LA1 on, LA2 off)
    snprintf(send_buffer, 12, "%03d%03d%03d%01d%01d", servo_cmds[0],
                                                      servo_cmds[1],
                                                      servo_cmds[2],
                                                      la_cmds[0],
                                                      la_cmds[1]);
    serial_send(send_buffer, 12);
    /*
    int16_t n;

    n = sprintf(send_buffer, "Ch 1: %ld\r\nCh 2: %ld\r\n\n", M[1].pos, M[1].pos_tgt);
    serial_send(send_buffer, n);
    */
}

void sync_input(void){
    int synced = 0;

    serial_receive(receive_buffer,1);

    while(!synced){
        if(serial_receive_buffer_full()){
            if (receive_buffer[0] == '.'){
                synced = 1;
                serial_receive(receive_buffer, 5);
            } else {
                serial_receive(receive_buffer, 1);
            }
        }
    }
}


/***************************
 * Low-level motor commands
 **************************/

// Move linear actuator 1 until analog feedback reports
// that it is at pos
void actuate_la_1(int pos){
    int current_pos;

    while (abs(current_pos - pos) > POS_ERR_TOL) {
        if (current_pos > pos){
            set_m1_speed(-1*LA_SPEED);
        } else {
            set_m1_speed(LA_SPEED);
        }
    }

    set_m1_speed(0);
}

void actuate_la_2(int pos){
    // Feedback loop until pos is reached
    // Reading from analog pin
int current_pos;

while (abs(current_pos - pos) > POS_ERR_TOL) {
        if (current_pos > pos){
            set_m2_speed(-1*LA_SPEED);
        } else {
            set_m2_speed(LA_SPEED);
        }
    }

    set_m2_speed(0);
}

// Changes command written to slave board to adjust wrist angle
// Delays for 20 ms for every degree of angle change
// Moves wrist 2 to arbitrary angle
void actuate_wrist_1(int angle) {
    int old_angle = servo_cmds[0];
    servo_cmds[0] = angle;

    //if (servo_cmds[0] > 90){
    //    servo_cmds[0] = 90;
    //} else if (servo_cmds[0] < -90){
    //    servo_cmds[0] = -90;
    //}
}

void increment_wrist_1(int delta_angle) {
    servo_cmds[0] = servo_cmds[0] + delta_angle;
    if (servo_cmds[0] > 90){
        servo_cmds[0] = 90;
    } else if (servo_cmds[0] < -90){
        servo_cmds[0] = -90;
    }
}

// Moves wrist 2 to arbitrary angle
void actuate_wrist_2(int angle) {
    int old_angle = servo_cmds[1];
    servo_cmds[1] = angle;

    if (servo_cmds[1] > 90){
        servo_cmds[1] = 90;
    } else if (servo_cmds[0] < -90){
        servo_cmds[1] = -90;
    }
}

void increment_wrist_2(int delta_angle) {
    servo_cmds[1] = servo_cmds[1] + delta_angle;

    if (servo_cmds[1] > 90){
        servo_cmds[1] = 90;
    } else if (servo_cmds[0] < -90){
        servo_cmds[1] = -90;
    }
}

// turns or rotates module 1 relative to module 2
void actuate_turn(int angle) {
    // set_servo_target takes microseconds, 600-2400
    // Translate an angle (0-180) into 600-2400
    int old_angle = (int)((get_servo_target(3) - 1500) / 10);

    int pwm_width = angle*10+1500;
    set_servo_target(3, pwm_width);

    // Delay for 20 ms for every degree swept
    delay_ms(20 * (abs(angle-old_angle)));
}

// uses microswitch feedback to determine if modules are in contact with surface correctly
int is_module_1_down(void){
	//reads microswitch signals
	int RL, RR;
    RL=is_digital_input_high(IO_B4);
    RR=is_digital_input_high(IO_B5);

    if (RL==1 && RR==1) {
	    return 1; }
    else {
	    return 0;}
}

int is_module_2_down(void){
	//reads microswitch signals
	int FL, FR;
    FL=is_digital_input_high(IO_B1);
    FR=is_digital_input_high(IO_B2);

    // determines if both microswitches have been triggered
    if (FL==1 && FR==1) {
	    return 1; }
    else {
	    return 0;}
}

void spin_RAP(int dir){
    if (dir > 0){
        set_m2_speed(100);
    } else if (dir < 0){
        set_m2_speed(-100);
    } else {
        set_m2_speed(0);
    }
}

// run rack and pinion (RAP) w/ inputs of direction and distance
void actuate_RAP (long dist) {
    M[1].pos_tgt = M[1].pos + dist;
}



/****************************
 * Mid-level action commands
 ***************************/

void climb_raise_mod_1(void){
	 while(is_module_1_down() ){
        //current_pos = current_pos + 5;
        servo_cmds[0] = servo_cmds[0] + 5;
        actuate_wrist_1(servo_cmds[0]);
    }
    actuate_wrist_1(WRIST_1_UP);
}


void climb_lower_mod_1(void){
    //  Choose one of next two line, rest stays the same
    int current_pos = (int)(get_servo_target(1)-1500 / 10);
    //int current_pos = WRIST_1_DOWN;
    // or could use define WRIST_1_DOWN
    actuate_wrist_1(current_pos);

    while(! is_module_1_down() ){
        current_pos = current_pos - 5;
        actuate_wrist_1(current_pos);
    }
}



void climb_raise_mod_2(void){
	 while(is_module_2_down() ){
        int current_pos = 0;
        current_pos = current_pos + 5;
        actuate_wrist_2(current_pos);
    }
    actuate_wrist_2(WRIST_2_UP);
}


void climb_lower_mod_2(void){
    //  Choose one of next two line, rest stays the same
    int current_pos = (int)(get_servo_target(1)-1500 / 10);
    //int current_pos = WRIST_1_DOWN;
    // or could use define WRIST_1_DOWN
    actuate_wrist_2(current_pos);

    while(! is_module_2_down() ){
        current_pos = current_pos - 5;
        actuate_wrist_2(current_pos);
    }
}



void stick_mod_1(void){
    climb_lower_mod_1();
    actuate_la_1(LA_1_ON);
}



void release_mod_1(void){
    actuate_la_1(LA_1_RELEASE);
    climb_raise_mod_1();
}


void stick_mod_2(void){
    climb_lower_mod_2();
    actuate_la_2(LA_2_ON);}


void release_mod_2(void){
    actuate_la_2(LA_1_RELEASE);
    climb_raise_mod_2();}

/*****************************
 * High level manuevers here
 * **************************/

int gecko_pad(int mod_number, int toggle){
	if (mod_number==1 && toggle==1) {
		stick_mod_1();}
	else if (mod_number==1 && toggle==0) {
		release_mod_1();}
	else if (mod_number==2 && toggle==1){
		stick_mod_2();}
	else if (mod_number==2 && toggle==0) {
		release_mod_2();}
	else if (mod_number==1 && toggle==2){
		int x1;
		x1=is_module_1_down();}
	else if (mod_number==2 && toggle==2){
		int x2;
		x2=is_module_2_down();
    }

}


void stationary_adhere(float turn_angle, int IW_length){
    int check_value;

    gecko_pad(2,ON);
    check_value=gecko_pad(1,CHECK);
    if (check_value==0){
        ; }
	    //check RAP motor encoder
	    //check turn_angle
	    //
    //actuate_wrist_1(wrist_angle);
    //actuate_turn(turn_angle);
    //stick_mod_1();

}

void step(){;}

void fast_loop(void) {
    // Inputs:
    // q-45.
    // w020.
    // e090.
    // a000.
    // s001.
	// d-50.
    // {q, w, e} are servos 1 2 3, followed by a 3-digit angle, followed by a period
    // {a, s} are linear actuators, followed by 2 0's, followed by {0,1}, followed by period
    // {d} is the rack and pinion, followed by 2 0's, then a direction.

    if (serial_receive_buffer_full()){
        // Check that we're properly synchronized
        if (receive_buffer[4] == '.') {
            char num_str[4];
            if (receive_buffer[0] == 'q'){
                memcpy(num_str, receive_buffer+1, 3);
                servo_cmds[0] = atoi(num_str);
            } else if (receive_buffer[0] == 'w'){
                memcpy(num_str, receive_buffer+1, 3);
                servo_cmds[1] = atoi(num_str);
            } else if (receive_buffer[0] == 'e'){
                memcpy(num_str, receive_buffer+1, 3);
                servo_cmds[2] = atoi(num_str);
            } else if (receive_buffer[0] == 'd'){
                memcpy(num_str, receive_buffer+1, 3);
                actuate_RAP((long)1000 * (long)atoi(num_str));

		// linear actuators
            } else if (receive_buffer[0] == 'a'){
				memcpy(num_str, receive_buffer+1, 3);
                la_cmds[0] = atoi(num_str);
             } else if (receive_buffer[0] == 's'){
                memcpy(num_str, receive_buffer+1, 3);
                la_cmds[1] = atoi(num_str);
             }
            // Listen for another command
            serial_receive(receive_buffer, 5);
        // Otherwise re-sync
        } else {
            sync_input();
        }
    }

	if (abs(M[1].pos_tgt - M[1].pos) > RACK_ERR_TOL) {
        if (M[1].pos > M[1].pos_tgt){
	        set_m2_speed(RAP_speed);
        } else {
            set_m2_speed(-1*RAP_speed);
        }
	} else {
        set_m2_speed(0);
    }
}

void slow_loop(void) {
    send_command();
}

void setup() {
    uint8_t i;

    // start up OrangutanTime
    prev_ticks = get_ticks();
    prev_ms = get_ms();

    // encoder pins (PC0-PC3) default to inputs
    // RC receiver pins (PD2,PD4,PD7,PB0) default to inputs

    // initialize enc_gray (set bits 2 and 3)
    // encoder 1 (A = PC0, B = PC1)
    enc_gray[0] |= (PINC & 0b00000011) << 2;
    // encoder 2 (A = PC2, B = PC3)

    // set up pin change interrupts (PB,PC,PD)
    PCICR |= 0b00000111;
    // set up pin change interrupt masks for encoder and receiver pins
    PCMSK0 = 0b00000001;
    PCMSK1 = 0b00001111;
    PCMSK2 = 0b10010100;

    // set up the motor parameters
    // Useful sets, (Kp, Kd, Ki): (50,0,0)
    for (i = 0; i < NUM_MOTORS; i++) {
        M[i].pos = 0;
        M[i].pos_tgt = 0;
        M[i].pos_tgt_acc = 0;
        M[i].Kp = 50;
        M[i].Ki = 0;
        M[i].Kd = 0;
        M[i].IL = 10000;
        M[i].OL = 255;
        M[i].SRD = 5;
        M[i].DB = 0;
    }

    // enable global interrupts (for encoders and servo pulse measurement)
    sei();

    // set up the serial port
    serial_set_baud_rate(SERIAL_BAUD_RATE);
    serial_receive(receive_buffer,5);

    command_changed = 0;

    // SERVOS INITIALIZATION
    // Servos are nominally at neutral positions
    servo_cmds[0] = 0;
    servo_cmds[1] = 0;
    servo_cmds[2] = 0;

    // Linear actuator initialization
    la_cmds[0] = 1;
    la_cmds[1] = 1;

    // Analog pin initialization
    set_analog_mode(MODE_10_BIT);
}

int main() {
    uint32_t t;

    setup();

    // wait for receiver to come up
    delay_ms(1000);

    // do the loop scheduling
    while (1) {
        current_ticks = get_ticks();
        current_ms = get_ms();
        t = current_ticks - prev_ticks;
        if (t >= FAST_PERIOD_TICKS) {
            last_fast_period_ticks = t;
            fast_loop();
            lfp_actual = get_ticks() - current_ticks;
            prev_ticks = current_ticks;
        }
        t = current_ms - prev_ms;
        if (t >= SLOW_PERIOD_MS) {
            last_slow_period_ms = t;
            slow_loop();
            prev_ms = current_ms;
        }
    }
}
