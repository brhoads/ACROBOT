/* GAIWR - an application for the Pololu Baby Orangutan B
 *
 * This application uses the Pololu AVR C/C++ Library.  For help, see:
 * -User's guide: http://www.pololu.com/docs/0J20
 * -Command reference: http://www.pololu.com/docs/0J18
 *
 * Created: 4/26/2012 10:44:19 AM
 *  Author: jleichty / nwiltsie
 * 
 * Modified: rhoadsb 18 Mar 14
 */

#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <pololu/orangutan.h>
#include <string.h>

// adjust F_CPU and SCL_CLOCK in twimaster.c

//DEBUG mode
// Define DEBUG when BO is wired to PuTTY via FTDI Cable
// In PuTTY set the baud to 115200
// The slave wants the null byte (ctrl+shift+2) before anything
// Type info just like the master would
// 09009000511 to set servos to 90, 90, 5 degrees and both LAs to 1
// Terminate info with null byte
//#define DEBUG 1

// robot parameters
#define NUM_MOTORS 1
#define SERIAL_BAUD_RATE 115200
#define FAST_PERIOD_TICKS 2500      // at 20MHz, ticks are 0.4us
#define SLOW_PERIOD_MS 1000
#define LA_MOVE_MS 750
#define LA_SPEED 100

// serial
char receive_buffer[13];
char send_buffer[256];

// timekeeping
uint32_t prev_ticks = 0;
uint32_t current_ticks = 0;
uint32_t last_fast_period_ticks = 0;
uint32_t lfp_actual = 0;
uint32_t prev_ms = 0;
uint32_t current_ms = 0;
uint32_t last_slow_period_ms = 0;
uint32_t last_la_move_ms[2];

// encoder gray code storage
uint8_t enc_gray[NUM_MOTORS];

// Combined RC pulse data
int16_t RC_motor[NUM_MOTORS];
// port state storage for changed pin identification
uint8_t portB_storage;
uint8_t portD_storage;

int16_t servo_cmds[3];
int8_t la_cmds[2];
int8_t la_state[2];

// motor data layout
typedef struct {
    int32_t  pos;               // current position
    int32_t  pos_tgt;           // position target
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

void sync_input(void){
    int synced = 0;

    serial_receive(receive_buffer,1);

    while(!synced){
        if(serial_receive_buffer_full()){
            if (receive_buffer[0] == '\0'){
                synced = 1;
                serial_receive(receive_buffer, 12);
            } else {
                serial_receive(receive_buffer, 1);
            }
        }
    }
}

void receive_command(void){
	int16_t n;
	
    if (serial_receive_buffer_full()){
#ifdef DEBUG
		n = sprintf(send_buffer, "Serial Full\n %s\n", receive_buffer);
		serial_send(send_buffer, n);
#endif
        if (receive_buffer[12] == '\0') {
            set_digital_output(IO_B2, HIGH);
            char servo1_str[4];
            char servo2_str[4];
            char servo3_str[4];
            char la1_str[2];
            char la2_str[2];

            memcpy(servo1_str, receive_buffer + 0, 3);
            servo1_str[3] = '\0';
            memcpy(servo2_str, receive_buffer + 3, 3);
            servo2_str[3] = '\0';
            memcpy(servo3_str, receive_buffer + 6, 3);
            servo3_str[3] = '\0';

            memcpy(la1_str, receive_buffer + 9, 1);
            la1_str[1] = '\0';
            memcpy(la2_str, receive_buffer + 10, 1);
            la2_str[1] = '\0';

            servo_cmds[0] = atoi(servo1_str);
            servo_cmds[1] = atoi(servo2_str);
            servo_cmds[2] = atoi(servo3_str);

            la_cmds[0] = atoi(la1_str);
            la_cmds[1] = atoi(la2_str);
#ifdef DEBUG
			n = sprintf(send_buffer, "LA %i %i\n", la_cmds[0],la_cmds[1]);
			serial_send(send_buffer, n);
#endif
            serial_receive(receive_buffer, 12);
            set_digital_output(IO_B2, LOW);			
			
			// run the motors
			set_servos();
			set_las();
        } else {
            sync_input();
        }
    }
}

void set_servos(){
    int i;
    for (i=0; i<3; i++){
        set_servo_target(i, servo_cmds[i] * 10 + 1500);
    }
}

void set_mx_speed(int x, int speed){
	int16_t n;
    if (x==1){
        set_m1_speed(speed);
#ifdef DEBUG
		n = sprintf(send_buffer, "Set Motor 1 %i\n",speed);
		serial_send(send_buffer, n);
#endif
	}
    else if (x==2){
        set_m2_speed(speed);
    }
}

void set_las(){
	int16_t n;

    int i;
    uint32_t la_ms = get_ms();
    for (i=0; i<2; i++){
        if (la_state[i] != la_cmds[i]){
            last_la_move_ms[i] = la_ms;
            if (la_cmds[i] == 1){
                set_mx_speed(i, LA_SPEED);
            } else if (la_cmds[i] == 0){
                set_mx_speed(i, -1*LA_SPEED);
            }
            la_state[i] = la_cmds[i];
        } else if ((la_ms - last_la_move_ms[i]) > LA_MOVE_MS){
            set_mx_speed(i, 0);
        }
    }
	//n = sprintf(send_buffer, "Set LAs %i %i", la_state[0],la_state[1]);
	//serial_send(send_buffer, n);
}

void fast_loop(void) {
    receive_command();
}

void slow_loop(void) {
    int16_t n;
    // debug messages
    n = sprintf(send_buffer, " CH1: %d\t P1: %ld\t PT1: %ld\t LFP: %ld\r\n", RC_motor[0], M[0].pos, M[0].pos_tgt, lfp_actual);
    //n = sprintf(send_buffer, "CH1: %d\r\nCH2: %d\r\nCH3: %d\r\nCH4: %d\r\n    ", RC[0], RC[1], RC[2], RC[3]);
    serial_send(send_buffer, n);
}

void setup() {
    uint8_t i;
	int16_t n;

    // start up OrangutanTime
    prev_ticks = get_ticks();
    prev_ms = get_ms();

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

    // Initialize the servos
    servos_start((unsigned char[]) {IO_C0, IO_C1, IO_C2}, 3);
    // Start the servos at 1500 ms = 90 degrees
    servo_cmds[0] = 0;
    servo_cmds[1] = 0;
    servo_cmds[2] = 0;
    la_cmds[0] = 0;
    la_cmds[0] = 0;
    la_state[0] = 0;
    la_state[1] = 0;
    last_la_move_ms[0] = prev_ms;
    last_la_move_ms[1] = prev_ms;
    set_servo_target(0, 1500);
    set_servo_target(1, 1500);
    set_servo_target(2, 1500);

    // set up the serial port
    serial_set_baud_rate(SERIAL_BAUD_RATE);
#ifdef DEBUG
	// debug messages
	n = sprintf(send_buffer, "Serial Init\n"); 
	serial_send(send_buffer, n);
#endif
    sync_input();
#ifdef DEBUG
	n = sprintf(send_buffer, "Serial Synced\n");
	serial_send(send_buffer, n);
#endif
}

int main()
{
    uint32_t t;
	int16_t n;

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
            //slow_loop();
            prev_ms = current_ms;
        }
    }
}
