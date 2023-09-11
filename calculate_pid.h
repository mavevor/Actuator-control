uint8_t PID_KEY_LIST[3] = {254, 255};
uint8_t PID_STATE[3] = {0};
uint8_t PID_ON_STATE[3] = {0};
int pid_state_index = -1;
int pid_key_index = -1;

//Values to change
int force_threshold[3] = {900, 900, 900};
float K_p[3] = {0.001, 0.001, 0.001};
float K_i[3] = {0.0001, 0.0001, 0.0001};
float K_d[3] = {1, 1, 1};

float max_dist[3] = {255, 255, 255};
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


uint8_t dist[3] = {0};
float corrected_dist[3] = {0};

int ACTUATOR[3] = {10, 12, 13};
int force_sensor[3] = {26, 27, 28};

uint32_t loop_timer = 0;
uint32_t cycle_time = 4000;                                                   //time between force updates
int force_counter = 0;

uint8_t force_data_out = 0;

uint32_t force_timer = 0;

void measure_force(void) {

  force_counter = 0;
  force_data_out = 32;                                                        // shifted left twice to give 0b10000000

  while ((micros() - loop_timer < cycle_time)) {
    if (force_counter < force_buffer_len) {
      force_buffer_0[force_counter % force_buffer_len] = analogRead(force_sensor[0]);
      force_buffer_1[force_counter % force_buffer_len] = analogRead(force_sensor[1]);
      force_buffer_2[force_counter % force_buffer_len] = analogRead(force_sensor[2]);
      av_force[0] = 0;
      av_force[1] = 0;
      av_force[2] = 0;
      for (int i = 0; i <= force_counter; i++) {
        av_force[0] += force_buffer_0[i] / (force_counter + 1);
        av_force[1] += force_buffer_1[i] / (force_counter + 1);
        av_force[2] += force_buffer_2[i] / (force_counter + 1);
      }
      force_counter ++;
    }
    if (force_counter >= force_buffer_len) {
      av_force[0] -= force_buffer_0[force_counter % force_buffer_len] / force_buffer_len;
      av_force[1] -= force_buffer_1[force_counter % force_buffer_len] / force_buffer_len;
      av_force[2] -= force_buffer_2[force_counter % force_buffer_len] / force_buffer_len;
      force_buffer_0[force_counter % force_buffer_len] = analogRead(force_sensor[0]);
      force_buffer_1[force_counter % force_buffer_len] = analogRead(force_sensor[1]);
      force_buffer_2[force_counter % force_buffer_len] = analogRead(force_sensor[2]);
      av_force[0] += force_buffer_0[force_counter % force_buffer_len] / force_buffer_len;
      av_force[1] += force_buffer_1[force_counter % force_buffer_len] / force_buffer_len;
      av_force[2] += force_buffer_2[force_counter % force_buffer_len] / force_buffer_len;
      force_counter ++;
    }
  }
  for (int i = 0; i < 3; i ++) {
    force_data_out += ((av_force[i] > 0.96 * force_threshold[i]));                        //cutoff slightly lower than PID threshold for smoother data
    if (i < 2) {
      force_data_out = force_data_out << 1;
    }
  }
  loop_timer = micros();

}

void calculate_pid(void) {                                                                  // cycle time is constant.

  for (int i = 0 ; i < 3; i++) {
    if (PID_ON_STATE[i] == 1) {
      p_error[i] = force_threshold[i] - av_force[i];
      i_error[i] += p_error[i];
      d_error[i] = p_error[i] - p_error_prev[i];
      p_error_prev[i] = p_error[i];

      if ((K_i[i] * i_error[i] > max_dist[i]) && K_i[i] != 0) {
        i_error[i] = max_dist[i] / K_i[i];
      }
      if ((K_i[i] * i_error[i] < min_dist[i]) && K_i[i] != 0) {
        i_error[i] = min_dist[i] / K_i[i];
      }

      corrected_dist[i] = K_p[i] * p_error[i] + K_i[i] * i_error[i] + K_d[i] * d_error[i];
      if (corrected_dist[i] > max_dist[i]) {
        corrected_dist[i] = max_dist[i];
      }
      if (corrected_dist[i] < min_dist[i]) {
        corrected_dist[i] = min_dist[i];
      }
    }
    else {
      corrected_dist[i] = dist[i];
    }
    analogWrite(ACTUATOR[i], corrected_dist[i]);
  }
  /*Serial.print(av_force[0]);
    Serial.print("  ");
    Serial.print(corrected_dist[0]);
    Serial.print("   ");
    Serial.print(av_force[1]);
    Serial.print("  ");
    Serial.print(corrected_dist[1]);
    Serial.print("   ");
    Serial.print(av_force[2]);
    Serial.print("  ");
    Serial.println(corrected_dist[2]);*/
}
