uint64 timestamp					# time since system start (microseconds)
uint64 timestamp_sample

float32[6] throttle_ref
float32[6] rpm_fb
float32[6] output

float32[6] output_p
float32[6] output_i
float32[6] output_d
float32[6] output_ff
float32[6] error

bool start_over_flag

