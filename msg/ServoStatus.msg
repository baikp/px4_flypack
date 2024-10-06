uint64 timestamp          # time since system start (microseconds)
# uint64 timestamp_sample

uint8 servo_id  #Range: 0-17, support up to 18 Servos
uint16 pwm_input #Range: PWM command: 700-2300, 1 LSB: us, CAN command: 0-1000
int16 pos_cmd #Range: -18000-18000,1 LSB: 0.01 degree
int16 pos_sensor #Range: -18000-18000,1 LSB: 0.01 degree
uint16 voltage #Range: 0-4095,1 LSB: 0.01 V
uint16 current #Range: 0-1023,1 LSB: 0.01 A
uint16 pcb_temp #Range: 0-1023,1 LSB: 0.2 degree(Celsius); temp=dat*0.2-40
uint16 motor_temp #Range: 0-1023,1 LSB: 0.2 degree(Celsius); temp=dat*0.2-40
uint8 statusinfo

