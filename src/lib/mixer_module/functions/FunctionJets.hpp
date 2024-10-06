/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include "FunctionProviderBase.hpp"
#include "JetsController/discreteController.hpp"
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/engine_status.h>
#include <uORB/topics/jet_controller.h>
#include <uORB/Publication.hpp>


using namespace discrete;

/**
 * Functions: Jet1 ... JetMax
 */
class FunctionJets : public FunctionProviderBase
{
public:
	static_assert(actuator_motors_s::NUM_CONTROLS == (int)OutputFunction::JetMax - (int)OutputFunction::Jet1 + 1,
		      "Unexpected num motors");

	static_assert(actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1 == (int)OutputFunction::Jet1 - 12, "Unexpected Jet idx");

	FunctionJets(const Context &context) :
		_topic(&context.work_item, ORB_ID(actuator_motors)),
		_thrust_factor(context.thrust_factor)
	{
		for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
			_data.control[i] = NAN;
		}
		_data.reversible_flags = 0;

		throttle_cmd.setZero();
		_controller_pub.advertise();
	}

	static FunctionProviderBase *allocate(const Context &context) { return new FunctionJets(context); }

	void update() override
	{
		const hrt_abstime now = hrt_absolute_time();
		const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);

		if (_topic.update(&_data) && _engine_status_sub.update(&_engine_status_data) ) {
			_pid_data.timestamp = now;
			_pid_data.timestamp_sample = (now - _last_run);
			updateValues(_engine_status_data.rpm, _data.control, flypack_2v6u_jet_num, dt);
			_last_run = now;
			_controller_pub.publish(_pid_data);
		}
	}

	float value(OutputFunction func) override { return throttle_cmd( (int)func - (int)OutputFunction::Jet1 ); }

	bool allowPrearmControl() const override { return false; }

	uORB::SubscriptionCallbackWorkItem *subscriptionCallback() override { return &_topic; }

	bool getLatestSampleTimestamp(hrt_abstime &t) const override { t = _data.timestamp_sample; return t != 0; }

	inline void updateValues(uint16_t *rpm, float *values, int num_values, float dt)
	{
		if(loop_en_){
			for (int i = 0; i < num_values; ++i) {
				sw_interp_wt[i].getValue(rpm[i]/1000.0f,throttle_fb[i]);

				controller_[i].update(values[i]*1000.0f,throttle_fb[i]*1000.0f, dt,throttle_cmd(i));
				controller_[i].getSeparateOut(_pid_data.output_p[i], _pid_data.output_i[i], _pid_data.output_d[i], _pid_data.output_ff[i], _pid_data.error[i]);

				_pid_data.output[i] = throttle_cmd(i);
				_pid_data.throttle_ref[i] = values[i]*1000.0f;
				_pid_data.rpm_fb[i] = throttle_fb[i]*1000.0f;

				throttle_cmd(i) = throttle_cmd(i)/1000.0f * 2.f - 1.f;
			}
		}else{
			for (int i = 0; i < num_values; ++i) {
				throttle_cmd(i) = values[i] * 2.f - 1.f;
				sw_interp_wt[i].getValue(rpm[i]/1000.0f,throttle_fb[i]);
				controller_[i].resetControllerState(values[i]*1000.0f,throttle_fb[i]*1000.0f);
			}
		}

	}

	bool reversible(OutputFunction func) const override { return _data.reversible_flags & (1u << ((int)func - (int)OutputFunction::Jet1)); }

	void setPIDGain(float kp,float ki,float kd, float Tt, bool loop_en) override{
		for (int i = 0; i < flypack_2v6u_jet_num; ++i) {
			controller_[i].setGain( kp, ki, kd, Tt);
		}
		loop_en_ = loop_en;
	}
private:
	const uint8_t flypack_2v6u_jet_num = 6;
	bool loop_en_ = false;

	uORB::Subscription _engine_status_sub{ORB_ID(engine_status)};
	uORB::Publication<jet_controller_s> _controller_pub{ORB_ID(jet_controller)};
	hrt_abstime _last_run{0};

	uORB::SubscriptionCallbackWorkItem _topic;
	actuator_motors_s _data{};
	jet_controller_s _pid_data{};
	engine_status_s _engine_status_data{};
	const float &_thrust_factor;

	// JetInterp<float,JETENGINE_DATA_LEN> sw_interp_wt[8] = { JetInterp<float,JETENGINE_DATA_LEN>(sw431_wrpm,sw_all_throttle),
	// 							JetInterp<float,JETENGINE_DATA_LEN>(sw430_wrpm,sw_all_throttle),
	// 							JetInterp<float,JETENGINE_DATA_LEN>(sw429_wrpm,sw_all_throttle),
	// 							JetInterp<float,JETENGINE_DATA_LEN>(sw426_wrpm,sw_all_throttle),
	// 							JetInterp<float,JETENGINE_DATA_LEN>(sw015_wrpm,sw_all_throttle),
	// 							JetInterp<float,JETENGINE_DATA_LEN>(sw013_wrpm,sw_all_throttle),
	// 							JetInterp<float,JETENGINE_DATA_LEN>(sw428_wrpm,sw_all_throttle),
	// 							JetInterp<float,JETENGINE_DATA_LEN>(sw427_wrpm,sw_all_throttle)};
	discrete2DofPID controller_[8];

	matrix::Vector<float,6> throttle_cmd;
	float throttle_fb[8] = {};

	typedef enum {
		SW431 = 0,
		SW430,
		SW429,
		SW426,
		SW015,
		SW013,
		SW428,
		SW427
	} JET_TAG;

};


