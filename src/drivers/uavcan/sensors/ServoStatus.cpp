/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "ServoStatus.hpp"

#include <drivers/drv_hrt.h>

const char *const UavcanServoStatusBridge::NAME = "ServoStatus";

UavcanServoStatusBridge::UavcanServoStatusBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_servo_status", ORB_ID(servo_status)),
	_sub_servo_status(node)
{
}

int
UavcanServoStatusBridge::init()
{
	int res = _sub_servo_status.start(ServoStatusCbBinder(this, &UavcanServoStatusBridge::servo_status_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void UavcanServoStatusBridge::servo_status_sub_cb(const uavcan::ReceivedDataStructure<flypack::ServoStatus> &msg)
{
	servo_status_s _servo_status{};
	_servo_status.timestamp = hrt_absolute_time();
	_servo_status.servo_id = msg.Servo_ID;
	_servo_status.pwm_input = msg.PWM_INPUT;
	_servo_status.pos_cmd = msg.POS_CMD;
	_servo_status.pos_sensor = msg.POS_SENSOR;
	_servo_status.voltage = msg.VOLTAGE;
	_servo_status.current = msg.CURRENT;
	_servo_status.pcb_temp = msg.PCB_Temp;
	_servo_status.motor_temp = msg.MOTOR_Temp;
	_servo_status.statusinfo = msg.StatusInfo;

	publish(msg.getSrcNodeID().get(), &_servo_status);
}
