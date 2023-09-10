//Half Duplex 8-bit communication

uint8_t outputs_8_bit[8] = {0,1,2,3,5,6,7,8};           //MSB to LSB
uint8_t inputs_8_bit[8] = {14,15,16,17,18,19,21,22};    // MSB to LSB

uint8_t check_byte = 0;
uint8_t ack_byte = 0;
uint8_t start_byte = 0;

const uint8_t buffer_len = 10;
uint8_t input_data_buffer[buffer_len] = {0};
uint8_t output_data_buffer[buffer_len] = {0};

int input_buffer_counter = 0;
int output_buffer_counter = 0;

uint8_t prev_out_byte = 0;
uint8_t prev_in_byte = 0;

uint32_t t_start = 0;
uint32_t t_output = 0;

bool is_first_write = 1;

uint8_t data_sent = 0;

uint8_t error_val = 0;
uint8_t error_counter = 0;

//Function Declarations

void send_byte(uint8_t data_out);
void send_data(void);
void write_data(uint8_t data_out);
uint8_t receive_byte(void);
void receive_data(void);
uint8_t read_data(void);
bool send_receive_ack(struct repeating_timer *t);

//Setup

void setup_comms(void){
  for (int i = 0; i < 8; i++){
    pinMode(outputs_8_bit[i], OUTPUT);
    pinMode(inputs_8_bit[i], INPUT_PULLUP);
  }
  prev_in_byte = receive_byte(); 
  check_byte = ~prev_in_byte;
  send_byte(check_byte);  
  static struct repeating_timer timer;
  add_repeating_timer_us(100, send_receive_ack, NULL, &timer);
}

//Write Functions

void send_byte(uint8_t data_out){
  for (int i = 7; i >= 0; i--){
    digitalWrite(outputs_8_bit[i], !(data_out & 0b00000001));
    data_out = data_out >> 1;   
  }
  return;
}

void send_data(void){

  while(( micros() - t_output < 5000) && data_sent == 0){
    uint8_t out_data = output_data_buffer[(output_buffer_counter-1)%buffer_len]; 
    uint8_t temp_1 = prev_out_byte;
    
    if (error_counter == 1){
      out_data = 0;
      send_byte(out_data);                              // could also set predetermined value on lost connection (zero)
      prev_out_byte = out_data;
      start_byte = ~prev_out_byte;
      check_byte = prev_out_byte; 
      output_buffer_counter = 0;
      error_val = 1;
    }
    if (error_counter == 0){
      start_byte = receive_byte();
      prev_out_byte = ~start_byte;
      check_byte = prev_out_byte; 
      uint8_t temp_2 = temp_1 & start_byte;
      if ((receive_byte() != temp_2) || start_byte == 0){
        send_byte(start_byte);
        uint32_t t_temp = micros();
        while (micros() - t_temp < 100);
        error_counter = 1; 
      }
      else if (receive_byte() == temp_2){
        out_data = 0;
        send_byte(out_data);
        prev_out_byte = out_data;
        start_byte = ~prev_out_byte;
        check_byte = prev_out_byte; 
        output_buffer_counter = 0;
        error_val = 4;   
      }
    }

    
    if (error_val == 0){
      uint32_t temp = micros();   
      while ((micros() - temp < 100) && is_first_write == 1){};
    
      temp = micros();  
      
      while ((micros() - temp < 300) && receive_byte() != prev_out_byte){}
    
      if (receive_byte() == prev_out_byte){
        send_byte(out_data); 
        prev_out_byte = out_data;
        start_byte = ~prev_out_byte;
        check_byte = prev_out_byte; 
        temp = micros();
        while((micros() - temp < 300) && (receive_byte()!= start_byte)){};
           
        if (receive_byte() == start_byte){
          data_sent = 1;
          is_first_write = 0;
          error_val = 0;
          error_counter = 0;
          output_buffer_counter--;
        }
        else{
          error_val = 2; 
        }
      }
      else{
        error_val = 3;
      }
    }
    is_first_write = 0;
  }
  if(error_val == 0 && data_sent == 0){
    error_val = 4;
    output_buffer_counter = 0;        
  }
}

void write_data(uint8_t data_out){
  error_val = 0;
  error_counter = 0;
  data_sent = 0;
  if (output_buffer_counter == 0){
    t_output = micros();
  }
  output_data_buffer[(output_buffer_counter)%buffer_len] = data_out;
  output_buffer_counter ++;
}

//Read Functions

uint8_t receive_byte(void){
  uint8_t data = 0;
  for (int i = 0; i < 8; i++){
     data = data << 1;
     data += !digitalRead(inputs_8_bit[i]);
  }
  return data;
}

 void receive_data(void){

  ack_byte = receive_byte();
  t_start = micros();
  
  send_byte(prev_in_byte);  
  while (ack_byte == check_byte && micros() - t_start < 300){
    ack_byte = receive_byte();
  }

  uint32_t t_temp = micros();
  while (micros() - t_temp < 200); 
  
  input_data_buffer[input_buffer_counter%buffer_len] = receive_byte();

  check_byte = ~receive_byte();
  send_byte(check_byte);
  prev_in_byte = receive_byte();
  start_byte = prev_in_byte; 
  prev_out_byte = check_byte;
  
  input_buffer_counter ++;
  return;
}

uint8_t read_data(void){                          //FIFO buffer
  uint8_t temp_3 = 0;
  uint8_t temp_4 = 0;
  if (input_buffer_counter > 0){
    if (input_buffer_counter > 10){
      temp_3 = (input_buffer_counter+1)%buffer_len;   
    }
    
    temp_4 = input_data_buffer[temp_3];
    
    for (int i = 0; i < buffer_len; i++){
      input_data_buffer[(i+temp_3)%buffer_len] = input_data_buffer[(i+temp_3+1)%buffer_len];
    }
    
    input_buffer_counter--;

  } 
  return temp_4;
}

//Interrupt Service Routine

bool send_receive_ack(struct repeating_timer *t){
  
  //cancel_repeating_timer (&timer);
  if (error_val == 0){
    if (output_buffer_counter >0){
      send_data();
      prev_in_byte = receive_byte();
    }
    else if((receive_byte() == check_byte)){
      receive_data();
    }
  }      
  if (error_val != 0){
      check_byte = ~receive_byte();
      send_byte(check_byte);
      prev_in_byte = receive_byte();
      start_byte = prev_in_byte; 
      prev_out_byte = check_byte;
    }  
  //add_repeating_timer_us(-100, send_receive_ack, NULL, &timer);  
  return true;
}
