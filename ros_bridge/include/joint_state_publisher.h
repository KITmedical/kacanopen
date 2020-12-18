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
#include "publisher.h"
#include "ros/ros.h"

#include <string>
#include <cmath>

namespace kaco
{

	/// This class provides a Publisher implementation for
	/// use with kaco::Bridge and a CiA 402 motor device.
	/// It publishes the motor state as JointState messages
	/// from the ROS common_messages package.
	///
	/// Currenty, only the motor angle is published.
	/// You have to initialize the motor on your own.
	/// The motor is expected to be in position mode.
	class JointStatePublisher : public Publisher
	{

	public:
		/// Constructor
		/// \param device a CiA 402 compliant motor device object
		/// \param position_0_degree The motor position (dictionary
		/// entry "Position actual value") which represents a
		/// 0 degree angle.
		/// \param position_360_degree Like position_0_degree for
		/// 360 degree state.
		/// \param position_actual_field Name of the dictionary entry to use.
		/// \param topic_name Custom topic name. Leave out for default.
		/// \throws std::runtime_error if device is not CiA 402 compliant and in position_mode.
		JointStatePublisher(Device &device,
							int32_t position_0_degree,
							int32_t position_360_degree,
							int32_t velocity_nominator,
							int32_t velocity_denominator,
							const std::string &position_actual_field = "Position actual value",
							const std::string &velocity_actual_field = "Velocity actual value",
							const std::string& torque_actual_field = "Torque actual value",
							const std::string &topic_name = "");

		/// \see interface Publisher
		void advertise() override;

		/// \see interface Publisher
		void publish() override;

	private:
		static const bool debug = false;

		// TODO: let the user change this?
		static const unsigned queue_size = 100;

		/// converts "position actual value" from CanOpen to revolutions using m_position_0_degree and m_position_360_degree
		double pos_to_rev(int32_t pos) const;

		/// converts "velocity actual value" from CanOpen to revolutions per second using m_velocity_nominator and m_velocity_denominator
		double vel_to_rev_p_s(int32_t vel) const;

		/// converts "torque actual value" from CanOpen to effort
		double tor_to_eff(int32_t tor) const;


		/// constant PI
		static constexpr double pi() { return std::acos(-1); }

		Device &m_device;
		int32_t m_position_0_degree;
		int32_t m_position_360_degree;
		int32_t m_velocity_nominator;
		int32_t m_velocity_denominator;
		std::string m_position_actual_field;
		std::string m_velocity_actual_field;
		std::string m_torque_actual_field;
		std::string m_topic_name;
		bool m_initialized;

		ros::Publisher m_publisher;
	};

} // end namespace kaco
