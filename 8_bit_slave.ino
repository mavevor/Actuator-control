#include "8_bit_comms.h"
#include "calculate_pid.h"

uint8_t KEY_LIST[3] = {0b0000001, 0b00000010, 0b00000011};
uint8_t ON_STATE[3] = {0};

#define RESET_KEY 0

int key_index = -1;
int state_index = -1;

uint8_t in_byte = 0;
uint8_t out_byte = 0;

double timer_1 = 0;

int index(uint8_t arr[], uint8_t c, int len = 3) {
  for (int i = 0; i < len; ++i)
  {
    if (arr[i] == c) return i;
  }
  return -1;
}

int sensor_val = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  setup_comms();
  for (int i = 0; i < 3; i++) {
    analogWrite(ACTUATOR[i], dist[i]);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  int temp_counter = input_buffer_counter;
  if (ext_error_val != 0) {
    write_data(RESET_KEY);
    for (int i = 0; i < 3; i++) {
      dist[i] = 0;
      ON_STATE[i] = 0;
      PID_STATE[i] = 0;
      PID_ON_STATE[i] = 0;
    }
    delayMicroseconds(1000);
  }

  measure_force();
  calculate_pid();
  if (temp_counter == 0 && (micros() - force_timer > 100000)) {
    write_data(force_data_out);
    force_timer = micros();
  }

  double t_1 = micros();
  double t_2 = 0;
  if (temp_counter > 1) {
    in_byte = read_data();
  }

  if (temp_counter == 1) {

    in_byte = read_data();
    key_index = index(KEY_LIST, in_byte);
    state_index = index(ON_STATE, 1, 3);
    pid_state_index = index(PID_STATE, 1, 3);
    pid_key_index = index(PID_KEY_LIST, in_byte);

    if (in_byte == RESET_KEY) {
      for (int i = 0; i < 3; i++) {
        dist[i] = 0;
        ON_STATE[i] = 0;
        PID_STATE[i] = 0;
        PID_ON_STATE[i] = 0;
      }
    }
    
    else if ((key_index + 1) && !(state_index + 1)) {                  // key_index + 1 non zero if in_byte in KEY_LIST
      out_byte = KEY_LIST[key_index] - 1;                         // state_index + 1 zero if all states are 0
      write_data((byte) out_byte);
      ON_STATE[key_index] = 1;
    }
    
    else if ((state_index + 1) && !(pid_state_index + 1)) {
      dist[state_index] = in_byte;
      out_byte = dist[state_index] + 1;
      write_data((byte) out_byte);

      ON_STATE[state_index] = 0;
      PID_STATE[state_index] = 1;
    }
    
    else if ((pid_key_index + 1) && (pid_state_index + 1)) {
      out_byte = PID_KEY_LIST[pid_key_index];
      write_data((byte) out_byte);
      PID_STATE[pid_state_index] = 0;
      PID_ON_STATE[pid_state_index] = pid_key_index;
    }
    
    if (pid_state_index + 1) {
      analogWrite(ACTUATOR[pid_state_index], dist[pid_state_index]);
      max_dist[pid_state_index] = 1.1 * dist[pid_state_index];          //pid controller acts over distances +- 10% of set value
      if (max_dist[pid_state_index] > 255) {
        max_dist[pid_state_index] = 255;
      }
      min_dist[pid_state_index] = 0.9 * dist[pid_state_index];
      delayMicroseconds(1000000);                                       //wait for actuator to get to position
      loop_timer = micros();
    }
  }
}
