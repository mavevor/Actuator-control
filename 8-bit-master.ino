#include "8_bit_comms.h"

#define RESET_KEY 0

uint8_t KEY_LIST[3] = {1, 2, 3};
uint8_t ON_STATE[3] = {0};
uint8_t dist[3] = {0};
uint8_t PID_KEY_LIST[2] = {254, 255};
uint8_t PID_STATE[3] = {0};
bool PID_ON_STATE[3] = {0};
uint8_t ACTIVE_STATE[3] = {0};

uint8_t in_byte = 0;
uint8_t out_byte = 0;

bool force_feedback_data[3] = {0};

int key_index = -1;
int state_index = -1;
int pid_state_index = -1;
int pid_key_index = -1;
int active_state_index = -1;
bool pid_ack = 0;
bool key_ack = 0;
bool dist_ack = 0;
bool program_counter = 1;

bool key_sent = 0;

double timer_1;
double timer_2;
double timer_3;

int counter_1;

String ready_statement[3] = {"Actuator 1 ready! ", "Actuator 2 ready! ", "Actuator 3 ready! "};
String dist_statement[3] = {"Actuator 1 distance: ", "Actuator 2 distance: ", "Actuator 3 distance: "};
String pid_enable_statement[2] = {". PID disabled.", ". PID enabled."};
String invalid_key_statement = "Invalid Actuator ID! Try again: ";
String invalid_pid_key_statement = "Invalid PID Enable ID! Try again: ";
String key_statement = "Input valid Actuator ID";
String dist_input_statement = "Input distance: ";
String pid_input_statement = "Input PID Enable ID";

int index(uint8_t arr[], uint8_t c, int len = 3) {
  for (int i = 0; i < len; ++i)
  {
    if (arr[i] == c) return i;
  }
  return -1;
}

void setup() {
  Serial.begin(57600);
  Serial.setTimeout(0.5);
  setup_comms();
  write_data(RESET_KEY);
}

void loop() {
  if (Serial.available() > 0) {
    out_byte = Serial.parseInt();

    state_index = index(ON_STATE, 1, 3);
    key_index = index(KEY_LIST, out_byte);
    pid_state_index = index(PID_STATE, 1, 3);
    pid_key_index = index(PID_KEY_LIST, out_byte, 2);

    if (out_byte == RESET_KEY) {
      write_data(RESET_KEY);
      for (int i = 0; i < 3; i++) {
        ACTIVE_STATE[i] = 0;
        ON_STATE[i] = 0;
        PID_STATE[i] = 0;
        PID_ON_STATE[i] = 0;
        dist[i] = 0;
      }
      dist_ack = 0;
      program_counter = 1;
      key_sent = 0;

      Serial.print('\n');
      Serial.println(" Slave Device Resetting...");
      timer_3 = micros();
    }
    else if (state_index + 1) {
      write_data(out_byte);
      key_sent = 1;
    }
    else if (key_index + 1 && !dist_ack) {
      write_data(out_byte);
      key_sent = 1;
    }
    else if ((pid_key_index + 1) && dist_ack) {
      write_data(out_byte);
      key_sent = 1;
    }
    else if (key_sent == 0 && !dist_ack) {
      Serial.println(invalid_key_statement);
    }
    else if (key_sent == 0 && dist_ack) {
      Serial.println(invalid_pid_key_statement);
    }

    if (key_sent == 1) {
      program_counter = 0;
      delayMicroseconds(1000);
      timer_2 = micros();
      timer_3 = micros();
    }
  }

  if (ext_error_val != 0) {
    if (program_counter || key_sent == 1) {
      Serial.print('\n');
      Serial.println("Check connection and try again.");
      key_sent = 0;
      program_counter = 0;

    }
    dist_ack = 0;
    for (int i = 0; i < 3; i++) {
      ACTIVE_STATE[i] = 0;
      ON_STATE[i] = 0;
      PID_STATE[i] = 0;
      PID_ON_STATE[i] = 0;
      dist[i] = 0;
    }
    write_data(RESET_KEY);
    delayMicroseconds(1000);
  }

  if (input_buffer_counter == 0 && key_sent == 1 && out_byte != RESET_KEY) {

    if ((micros() - timer_2) > 15000) {
      ext_error_val = 5;
      dist_ack = 0;
      key_sent = 0;
      Serial.print('\n');
      Serial.println("Timeout Error! Try again!");
    }
  }

  while (input_buffer_counter > 1) {
    uint8_t temp = read_data();
  }
  if (input_buffer_counter == 1 && key_sent == 0) {
    in_byte = read_data();

    if (in_byte >= 128 && in_byte <= 135) {
      double temp_3 = micros() - timer_3;
      if (temp_3 < 200000) {

        for (int i = 0; i < 3; i++) {
          force_feedback_data[(2 - i)] = (in_byte >> i) & 0x01;
        }
        for (int i = 0; i < 3; i++) {
          if (ACTIVE_STATE[i] != 0) {
            Serial.print("Force Sensor ");
            Serial.print(i + 1);
            if (force_feedback_data[i] == 1) {
              Serial.println(" reached threshold.");
            }
            if (force_feedback_data[i] == 0) {
              Serial.println(" yet to reach threshold.");
            }
          }
        }
      }
      timer_3 = micros();
    }
  }

  if (((micros() - timer_3) > 300000) && (program_counter || key_sent == 1)) {
    ext_error_val = 6;
    dist_ack = 0;
    key_sent = 0;
  }

  if (input_buffer_counter == 1 && key_sent == 1 && ext_error_val == 0) {
    key_sent = 0;
    program_counter = 0;
    in_byte = read_data();
    key_index = index(KEY_LIST, (in_byte + 1) % 256);
    key_ack = (out_byte % 256 == (in_byte + 1) % 256);
    dist_ack = (out_byte % 256 == (in_byte + 255) % 256);
    state_index = index(ON_STATE, 1, 3);
    pid_state_index = index(PID_STATE, 1, 3);
    pid_key_index = index(PID_KEY_LIST, in_byte, 2);
    pid_ack = out_byte == in_byte;

    if ((key_index + 1) && key_ack && !(state_index + 1)) {
      program_counter = 1;
      ON_STATE[key_index] = 1;
      PID_STATE[key_index] = 0;
      Serial.print(ready_statement[key_index]);
      Serial.print(dist_input_statement);
    }
    else if (dist_ack  && (state_index + 1) && !(pid_state_index + 1)) {
      program_counter = 1;
      ON_STATE[state_index] = 0;
      PID_STATE[state_index] = 1;
      dist[state_index] = out_byte;
      Serial.println(out_byte);
      Serial.println(pid_input_statement);
    }
    else if (pid_ack && (pid_key_index + 1) && (pid_state_index + 1)) {
      program_counter = 1;
      PID_STATE[pid_state_index] = 0;
      PID_ON_STATE[pid_state_index] = pid_key_index;
      ACTIVE_STATE[pid_state_index] = 1;
      for (int i = 0; i < 3; i++) {
        Serial.print(dist_statement[i]);
        Serial.print(dist[i]);
        Serial.println(pid_enable_statement[PID_ON_STATE[i]]);
      }
      delayMicroseconds(1000000);

      Serial.println(key_statement);
    }
    else if (program_counter == 0) {
      program_counter = 1;
      Serial.println("Try again!");
    }
    timer_3 = micros();
  }
}
