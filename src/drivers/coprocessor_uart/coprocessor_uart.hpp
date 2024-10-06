/**
 * @file coprocessor_uart.hpp
 *
 * Driver for the extern UART bus to another mcu.
 *
 * @author   bkp	<baikp.2011@gmail.com>
 */


#pragma once

#include <termios.h>
#include <poll.h>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

#include "mixer_module/functions/FunctionJets.hpp"
#include "mixer_module/functions/JetsController/discreteController.hpp"
#include "mixer_module/functions/JetsController/jetEngineInterp.hpp"
#include "mixer_module/functions/JetsController/discreteTransferFun.hpp"
#include <uORB/topics/engine_status.h>
#include <uORB/topics/actuator_outputs.h>


#include <uORB/topics/engine_status.h>

#include "protocol.hpp"

#define COPROCESSOR_UART_DEFAULT_PORT	"/dev/ttyS3"
#define SIM_JET_FEEDBACK 1u

using namespace time_literals;

class CoprocessorUART : public ModuleBase<CoprocessorUART>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	CoprocessorUART();
	~CoprocessorUART() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	int _fd{-1};

	// char readbuf[5] {0,1,2,3,5};
	uint32_t last_recv_timestep[6] {0,0,0,0,0,0};
	protocol::dataParser _data_parser;
	engine_status_s _engine_status{};

	uORB::Publication<engine_status_s> _command_ack_pub{ORB_ID(engine_status)};

	// test bkp
	DiscreteFirstOrderInertialLink my_discrete_plant_[6] = {DiscreteFirstOrderInertialLink(0.2f),
								DiscreteFirstOrderInertialLink(0.25f),
								DiscreteFirstOrderInertialLink(0.3f),
								DiscreteFirstOrderInertialLink(0.35f),
								DiscreteFirstOrderInertialLink(0.45f),
								DiscreteFirstOrderInertialLink(0.5f)};
	hrt_abstime _last_run{0};
	actuator_outputs_s actuators_out{};
	uORB::Subscription _actuator_out_sub{ORB_ID(actuator_outputs)};


	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_bad_read_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad read")};
	perf_counter_t	_bad_send_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad send")};

};
