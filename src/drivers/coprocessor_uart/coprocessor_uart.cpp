#include "coprocessor_uart.hpp"

// discrete::JetInterp<float,JETENGINE_DATA_LEN> sw431_interp_tw(discrete::sw_all_throttle,discrete::sw013_wrpm);


CoprocessorUART::CoprocessorUART() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::ttyS3)
{
	_command_ack_pub.advertise();
	_data_parser.cmd_send.CMD = 0x00;
	_data_parser.cmd_send.engine_num = 0x00;
}

CoprocessorUART::~CoprocessorUART()
{
	perf_free(_loop_perf);
	perf_free(_bad_send_perf);
	perf_free(_bad_read_perf);
}

bool CoprocessorUART::init()
{

	do { // create a scope to handle exit conditions using break

		// open fd
		_fd = ::open(COPROCESSOR_UART_DEFAULT_PORT, O_RDWR | O_NOCTTY);

		if (_fd < 0) {
			PX4_ERR("Error opening fd");
			return false;
		}

		// baudrate 576000, 8 bits, no parity, 1 stop bit
		unsigned speed = B1000000;
		termios uart_config{};
		int termios_state{};

		tcgetattr(_fd, &uart_config);

		// clear ONLCR flag (which appends a CR for every LF)
		uart_config.c_oflag &= ~ONLCR;

		// set baud rate
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
			return false;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD\n", termios_state);
			return false;
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
			return false;
		}

		uart_config.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls
		uart_config.c_cflag &= ~CSIZE;
		uart_config.c_cflag |= CS8;			// 8-bit characters
		uart_config.c_cflag &= ~PARENB;			// no parity bit
		uart_config.c_cflag &= ~CSTOPB;			// only need 1 stop bit
		uart_config.c_cflag &= ~CRTSCTS;		// no hardware flowcontrol

		// setup for non-canonical mode
		uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		uart_config.c_oflag &= ~OPOST;

		// fetch bytes as they become available
		uart_config.c_cc[VMIN] = 1;
		uart_config.c_cc[VTIME] = 1;

		if (_fd < 0) {
			PX4_ERR("FAIL: laser fd");
			return false;
		}
	} while (0);

	// close the fd
	::close(_fd);
	_fd = -1;

	// alternatively, Run on fixed interval
	ScheduleOnInterval(9000_us); // 9000 us interval
	_data_parser.parser_num = 0;
	return true;
}

void CoprocessorUART::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	int ret = 0;

	// fds initialized?
	if (_fd < 0) {
		// open fd
		_fd = ::open(COPROCESSOR_UART_DEFAULT_PORT, O_RDWR | O_NOCTTY);
	}

	// Check the number of bytes available in the buffer
	int bytes_available = 0;
	// char parser_status = 0;
	::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);

	#ifdef SIM_JET_FEEDBACK
	if(_actuator_out_sub.update(&actuators_out)) { //debug
		const hrt_abstime now = hrt_absolute_time();
		const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);
		float throttle_output = 0.0f;

		for(int i = 0;i < 6;i++){
			my_discrete_plant_[i].update( (actuators_out.output[i] - 1000.f)/1000.f,throttle_output,dt);
			sw_interp_tw[i].getValue(throttle_output,throttle_output);
			_engine_status.rpm[i] = (throttle_output*1000.0f);
		}

		_engine_status.timestamp = now;
		_engine_status.timestamp_sample = (now - _last_run);
		_command_ack_pub.publish(_engine_status);
		_last_run = now;
	}
	return;
	#else
	if (!bytes_available) return; //return if no data
	#endif

	bytes_available = math::min(bytes_available,MAX_RECV_BUF_LEN);
	// bool data_parse_flag = false;
	_data_parser.data_parser_flag = false;
	do {
		perf_begin(_loop_perf);

		// read from the sensor (uart buffer)
		uint16_t readbuf_index = 0;
		ret = ::read(_fd, &(_data_parser._data[0]), bytes_available);

		if (ret < 0) {
			PX4_ERR("read err: %d", ret);
			perf_end(_loop_perf);
			perf_count(_bad_read_perf);
		}

		// #ifdef COPROCESSOR_UART_DEBUG
		// if (::write(_fd, &(_data_parser._data[0]), bytes_available) < 0) {
		// 	perf_count(_bad_send_perf);
		// }
		// #endif

		// data_parse_flag = _data_parser.protocolParser( &(_data_parser._data[0]),bytes_available);
		for (readbuf_index = 0; readbuf_index < bytes_available; ) {
			_data_parser.parser_status = _data_parser.protocol_parser(&(_data_parser._data[0]), readbuf_index, bytes_available);
		}
		#ifdef COPROCESSOR_UART_DEBUG
		for(int i = 0;i < 100;i++){
			printf("%x ",_data_parser.parse_buf[i]);
			// printf("%x ",_data_parser._data[i]);
		}
		printf("\n");
		#endif
		// bytes left to parse
		bytes_available -= ret;
		perf_end(_loop_perf);
	} while (bytes_available > 0);

	// _engine_status.msg_length = (uint32_t)_data_parser.parse_buf_index;
	// if(data_parse_flag){
	_engine_status.timestamp_sample = (hrt_absolute_time() - _engine_status.timestamp > 1e5) ? (0) : (hrt_absolute_time() - _engine_status.timestamp);
	_engine_status.timestamp = hrt_absolute_time();
	_engine_status.oil_pos_bt = _data_parser.protocol_s.pd.oil_pos1;
	_engine_status.oil_pos_jm = _data_parser.protocol_s.pd.oil_pos2;
	_engine_status.data_id = _data_parser.protocol_s.pd.data_num;
	_engine_status.start_over_flag = (_data_parser.protocol_s.pd.ECU_RPM[2] > 3000) ? true : false;

	memcpy(&(_engine_status.rpm[0]), &(_data_parser.protocol_s.pd.ECU_RPM[0]), sizeof(_data_parser.protocol_s.pd.ECU_RPM));
	memcpy(&(_engine_status.rpm_timestamp[0]), &(_data_parser.protocol_s.pd.rpm_timestamp[0]), sizeof(_data_parser.protocol_s.pd.rpm_timestamp));
	memcpy(&(_engine_status.ecode[0]), &(_data_parser.protocol_s.pd.Ecode[0]), sizeof(_data_parser.protocol_s.pd.Ecode));
	memcpy(&(_engine_status.engine_state[0]), &(_data_parser.protocol_s.pd.EngineState[0]), sizeof(_data_parser.protocol_s.pd.EngineState));
	memcpy(&(_engine_status.pwm_input[0]), &(_data_parser.protocol_s.pd.Pwm_Input[0]), sizeof(_data_parser.protocol_s.pd.Pwm_Input));
	memcpy(&(_engine_status.pos_cmd[0]), &(_data_parser.protocol_s.pd.Pos_cmd[0]), sizeof(_data_parser.protocol_s.pd.Pos_cmd));
	memcpy(&(_engine_status.pos_sensor[0]), &(_data_parser.protocol_s.pd.POS_Sensor[0]), sizeof(_data_parser.protocol_s.pd.POS_Sensor));
	memcpy(&(_engine_status.servo_timestamp[0]), &(_data_parser.protocol_s.pd.Time_MG[0]), sizeof(_data_parser.protocol_s.pd.Time_MG));

	_command_ack_pub.publish(_engine_status);

}

int CoprocessorUART::task_spawn(int argc, char *argv[])
{
	CoprocessorUART *instance = new CoprocessorUART();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int CoprocessorUART::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_bad_send_perf);
	perf_print_counter(_bad_read_perf);

	printf("parser_num: %ld\n",_data_parser.parser_num);
	printf("cmd: %x\n",_data_parser.protocol_s.cmd);
	printf("data_length: %d\n",_data_parser.protocol_s.length);
	printf("error_code: %d\n",_data_parser.parser_status);
	printf("parser_index: %d\n",_data_parser.parse_buf_index);
	printf("parser_state: %d\n",_data_parser.data_parser_flag);
	// printf("Throttle: %d %d %d %d %d %d\n",_data_parser.protocol_s.pd.Temp[0],
	// 				       _data_parser.protocol_s.pd.Throttle[1],
	// 				       _data_parser.protocol_s.pd.Throttle[2],
	// 				       _data_parser.protocol_s.pd.Throttle[3],
	// 				       _data_parser.protocol_s.pd.Throttle[4],
	// 				       _data_parser.protocol_s.pd.Throttle[5]);
	// printf("engine state: %d %d %d %d %d %d\n",_data_parser.protocol_s.pd.EngineState[0],
	// 				       _data_parser.protocol_s.pd.EngineState[1],
	// 				       _data_parser.protocol_s.pd.EngineState[2],
	// 				       _data_parser.protocol_s.pd.EngineState[3],
	// 				       _data_parser.protocol_s.pd.EngineState[4],
	// 				       _data_parser.protocol_s.pd.EngineState[5]);
	for(int i = 0;i < MAX_RECV_BUF_LEN;i++){
		printf("%x ",_data_parser._data[i]);
	}
	printf("\n");
	return 0;
}

int CoprocessorUART::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CoprocessorUART::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
coprocessor module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("CoprocessorUART", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int coprocessor_uart_main(int argc, char *argv[])
{
	return CoprocessorUART::main(argc, argv);
}

