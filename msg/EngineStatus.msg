uint64 timestamp					# time since system start (microseconds)
uint64 timestamp_sample

uint8[6]  engine_state
uint8[6]  ecode
uint16[6] rpm
uint32[6] rpm_timestamp

uint16[4] pwm_input
int16[4] pos_cmd
int16[4] pos_sensor
uint32[4] servo_timestamp

uint8 oil_pos_bt
uint8 oil_pos_jm
uint8 data_id

bool start_over_flag

