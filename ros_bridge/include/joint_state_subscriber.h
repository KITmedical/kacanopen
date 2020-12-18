/*
 * Copyright (c) 2015-2016, Thomas Keh
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "device.h"
#include "subscriber.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include <string>
#include <cmath>

namespace kaco
{

	/// This class provides a Subscriber implementation for
	/// use with kaco::Bridge and a CiA 402 motor device.
	/// It listens for JointState messages from ROS
	/// common_messages package and updates the motor device
	/// accordingly.
	///
	/// Currenty, only the motor angle is regarded.
	/// You have to initialize the motor on your own.
	/// The motor is expected to be in position mode and
	/// operational state.
	class JointStateSubscriber : public Subscriber
	{

	public:
		/// Constructor
		/// \param device a CiA 402 compliant motor device object
		/// \param position_0_degree The motor position (dictionary
		/// entry "Position actual value") which represents a
		/// 0 degree angle.
		/// \param position_360_degree Like position_0_degree for
		/// 360 degree state.
		/// \param topic_name Custom topic name. Leave out for default.
		/// \throws std::runtime_error if device is not CiA 402 compliant and in position_mode.
		JointStateSubscriber(Device &device,
							 int32_t position_0_degree,
							 int32_t position_360_degree,
							 int32_t velocity_nominator,
							 int32_t velocity_denominator,
							 std::string topic_name = "");

		/// \see interface Subscriber
		void advertise() override;

	private:
		static const bool debug = false;

		// TODO: let the user change this?
		static const unsigned queue_size = 1;

		/// Callback "received ROS JointState message"
		void receive(const sensor_msgs::JointState &msg);

		/// converts revolutions to "Target position" value from CanOpen using m_position_0_degree and m_position_360_degree
		int32_t rev_to_pos(double pos) const;

		/// converts revolutions per second to "velocity actual value" from CanOpen using m_velocity_nominator and m_velocity_denominator
		int32_t rev_p_s_to_vel(double rad) const;

		/// converts effort to "torque actual value" from CanOpen
		int32_t eff_to_tor(double eff) const;

		/// constant PI
		static constexpr double pi() { return std::acos(-1); }

		Device &m_device;
		int32_t m_position_0_degree;
		int32_t m_position_360_degree;
		int32_t m_velocity_nominator;
		int32_t m_velocity_denominator;
		std::string m_topic_name;
		bool m_initialized;

		ros::Subscriber m_subscriber;
	};

} // end namespace kaco