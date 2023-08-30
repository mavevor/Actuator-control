#include "8_bit_comms.h"

//uint8_t KEY_LIST[3] = {0b0000001, 0b0000010, 0b00000011};

#define RESET_KEY 0

uint8_t KEY_LIST[3] = {1,2,3};
uint8_t ON_STATE[3] = {0};
uint8_t dist[3] = {0};
uint8_t PID_KEY_LIST[2] = {254,255};
uint8_t PID_STATE[3] = {0};
bool PID_ON_STATE[3] = {0}; 

uint8_t in_byte = -1;
uint8_t out_byte = -1;

bool force_feedback_data[3] = {0};

int key_index = -1;
int state_index = -1;
int pid_state_index = -1;
int pid_key_index = -1;
bool pid_ack = 0;
bool key_ack = 0;
bool dist_ack = 0;
bool program_counter = 0;

bool key_sent;

String ready_statement[3] = {"Actuator 1 ready! ","Actuator 2 ready! ","Actuator 3 ready! "};
String dist_statement[3] = {"Actuator 1 distance: ","Actuator 2 distance: ","Actuator 3 distance: "};
String pid_enable_statement[2] = {". PID disabled.",". PID enabled."};
String invalid_key_statement = "Invalid Actuator ID! Try again: ";
String invalid_pid_key_statement = "Invalid PID Enable ID! Try again: ";
String key_statement = "Input valid Actuator ID";
String dist_input_statement = "Input distance: ";
String pid_input_statement = "Input PID Enable ID";

int index(uint8_t arr[], uint8_t c, int len = 3){
  for (int i = 0; i < len; ++i)
    {
        if (arr[i] == c) return i;
    }
    return -1;
}

/*void comms_wait(void){
  double start_time = millis();
  while(input_buffer_counter==0 && (millis() - start_time)< 100){}
  while(input_buffer_counter){
    uint8_t temp_1 = read_data();
    }
}*/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  setup_comms();
  Serial.println(key_statement);
  write_data(RESET_KEY);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available() > 0 && ext_error_val == 0){
      out_byte = Serial.parseInt();
      
      state_index = index(ON_STATE, 1, 3); 
      key_index = index(KEY_LIST, out_byte);
      pid_state_index = index(PID_STATE, 1, 3); 
      pid_key_index = index(PID_KEY_LIST, out_byte, 2);  
      if (out_byte == RESET_KEY){
          write_data(RESET_KEY);
          key_sent = 1;
          Serial.println(" Slave Device Resetting...");
      }
      if (state_index + 1){
        write_data(out_byte);
        key_sent = 1;
      }
      else if (key_index +1){
        write_data(out_byte);
        key_sent = 1;
      }
      else if ((pid_key_index + 1) && dist_ack){
        write_data(out_byte);
        key_sent = 1;
      }
      else if (key_sent == 0 && !dist_ack){
        Serial.println(invalid_key_statement);
      }
      else if(key_sent == 0 && dist_ack){
        Serial.println(invalid_pid_key_statement);
      }

    }    
  while (ext_error_val != 0){
    write_data(RESET_KEY);
  }
  double timer = micros();
  while (input_buffer_counter == 0 && key_sent == 1 && ext_error_val == 0 && out_byte != RESET_KEY){
    
    if ((micros() - timer) > 5000){
      Serial.print('\n');
      Serial.println("Timeout Error! Try again!");
      key_sent = 0;
      if (state_index + 1){
        ON_STATE[state_index] = 0;   
      }
      dist_ack = 0;
      if (pid_state_index +1){
        PID_STATE[pid_state_index] = 0;
      }
    }
  }
  while (input_buffer_counter > 1){                              
    read_data();
  }
  while (input_buffer_counter ==1 && key_sent == 0){
    in_byte = read_data();      
    if (in_byte == 255){    
      Serial.print('\n');
      Serial.print("Error Code ");      
      Serial.print(in_byte);
      Serial.println(":  Slave Device Has Restarted! ");
      for (int i = 0; i < 3; i++){
        dist[i] = 0;
        ON_STATE[i] = 0;
        PID_STATE[i] = 0;
        Serial.print(dist_statement[i]);
        Serial.println(dist[i]);        
      }
      Serial.println(key_statement);
    }

    if (in_byte >= 128 && in_byte <= 135){
      for (int i = 0; i < 3; i++){
        force_feedback_data[(2-i)] = (in_byte >> i) && 0x01;     
      }
      for (int i = 0; i < 3; i++){
        if (dist[i] != 0){
          Serial.print("Force Sensor ");
          Serial.print(i+1);
          if (force_feedback_data[i] == 1){
            Serial.println(" reached threshold.");   
          }
          if (force_feedback_data[i] == 0){
            Serial.println(" yet to reach threshold.");
          }
        }
      }
      
    }
  }
  if (input_buffer_counter>0 && key_sent == 1){
    if( ext_error_val == 0){
      
      program_counter = 0;   
      in_byte = read_data();
      
      key_index = index(KEY_LIST, (in_byte + 1)%256);
      key_ack = (out_byte%256 == (in_byte + 1)%256);
      dist_ack = (out_byte%256 == (in_byte + 255)%256);
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
  
      else if (dist_ack  && (state_index + 1) && !(pid_state_index +1)){
        program_counter = 1;
        ON_STATE[state_index] = 0;
        PID_STATE[state_index] = 1;
        dist[state_index] = out_byte;
        Serial.println(out_byte);
        Serial.println(pid_input_statement);
        /*for (int i = 0; i < 3; i++){
          Serial.print(dist_statement[i]);
          Serial.print(dist[i]);
          Serial.println(pid_enable_statement[pid_state_index]);
        }*/
      }
      else if (pid_ack && (pid_key_index + 1) && (pid_state_index + 1)){
        program_counter = 1;
        PID_STATE[pid_state_index] = 0;
        PID_ON_STATE[pid_state_index] = pid_key_index;
        for (int i = 0; i < 3; i++){
          Serial.print(dist_statement[i]);
          Serial.print(dist[i]);
          Serial.println(pid_enable_statement[PID_ON_STATE[i]]); 
          }    
          Serial.println(key_statement);
      } 
      else if (program_counter == 0){
        Serial.println("Try again!");   
      }
      key_sent = 0;
    }

  }

}
