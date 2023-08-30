uint8_t PID_KEY_LIST[3] = {254,255};
uint8_t PID_STATE[3] = {0};
uint8_t PID_ON_STATE[3] = {0};
int pid_state_index = -1;
int pid_key_index = -1;

int force_threshold[3] = {600, 600, 600};
float max_dist[3] = {0};
float min_dist[3] = {0};
const int force_buffer_len = 10;
float force_buffer_0[force_buffer_len] = {0};
float force_buffer_1[force_buffer_len] = {0};
float force_buffer_2[force_buffer_len] = {0};

float av_force[3] = {0};

float p_error[3] = {0};
float i_error[3] = {0};
float d_error[3] = {0};
float p_error_prev[3] = {0};

float K_p = 0.00002;
float K_i = 0.0000002;
float K_d = 0.02;

int dist[3] = {0};

int ACTUATOR[3] = {8,9,10};
int force_sensor[3] = {11,12,13};

double loop_timer = 0;
double cycle_time = 4000;
int force_counter = 0;

uint8_t force_data_out = 0;

void measure_force(void){ 
  av_force[0] = 0;
  av_force[1] = 0;
  av_force[2] = 0;
  force_counter = 0;
  force_data_out = 32;

  while ((micros() - loop_timer < cycle_time)){
    if (force_counter < force_buffer_len){
      force_buffer_0[force_counter%force_buffer_len] = analogRead(force_sensor[0]);
      force_buffer_1[force_counter%force_buffer_len] = analogRead(force_sensor[1]);
      force_buffer_2[force_counter%force_buffer_len] = analogRead(force_sensor[2]);
      av_force[0] += force_buffer_0[force_counter%force_buffer_len]/(force_counter+1);
      av_force[1] += force_buffer_1[force_counter%force_buffer_len]/(force_counter+1);
      av_force[2] += force_buffer_2[force_counter%force_buffer_len]/(force_counter+1);
      force_counter ++;
    }
    if (force_counter >= force_buffer_len){
      av_force[0] -= force_buffer_0[force_counter%force_buffer_len]/force_buffer_len;
      av_force[1] -= force_buffer_1[force_counter%force_buffer_len]/force_buffer_len;
      av_force[2] -= force_buffer_2[force_counter%force_buffer_len]/force_buffer_len;
      force_buffer_0[force_counter%force_buffer_len] = analogRead(force_sensor[0]);
      force_buffer_1[force_counter%force_buffer_len] = analogRead(force_sensor[1]);
      force_buffer_2[force_counter%force_buffer_len] = analogRead(force_sensor[2]);
      av_force[0] += force_buffer_0[force_counter%force_buffer_len]/force_buffer_len;
      av_force[1] += force_buffer_1[force_counter%force_buffer_len]/force_buffer_len;
      av_force[2] += force_buffer_2[force_counter%force_buffer_len]/force_buffer_len;
    }
  }
  for(int i = 0; i < 3; i ++){
    force_data_out += (av_force[i] > force_threshold[i]);
    if(i < 2){force_data_out = force_data_out << 1;} 
  }
  loop_timer = micros();
}

void calculate_pid(void){

  for (int i = 0 ; i < 3; i++){
    if (PID_ON_STATE[i] == 1){
      p_error[i] = force_threshold[i] - av_force[i];
      i_error[i] += p_error[i];
      d_error[i] = p_error[i] - p_error_prev[i];
      p_error_prev[i] = p_error[i];

      if (K_i * i_error[i] > max_dist[i]){i_error[i] = max_dist[i] / K_i;}
      if (K_i * i_error[i] < min_dist[i]){i_error[i] = min_dist[i] / K_i;}

      dist[i] = K_p * p_error[i] + K_i * i_error[i] + K_d * d_error[i];

      if (dist[i] > max_dist[i]){dist[i] = max_dist[i];}
      if (dist[i] < max_dist[i]){dist[i] = max_dist[i];}
    }
    analogWrite(ACTUATOR[i], dist[i]);
  }
}
