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

#include "joint_state_subscriber.h"
#include "utils.h"
#include "logger.h"
#include "profiles.h"
#include "sdo_error.h"

#include "ros/ros.h"

#include <string>

namespace kaco
{

	JointStateSubscriber::JointStateSubscriber(Device &device,
											   int32_t position_0_degree,
											   int32_t position_360_degree,
											   int32_t velocity_nominator,
											   int32_t velocity_denominator,
											   std::string topic_name)
		: m_device(device),
		  m_position_0_degree(position_0_degree),
		  m_position_360_degree(position_360_degree),
		  m_velocity_nominator(velocity_nominator),
		  m_velocity_denominator(velocity_denominator),
		  m_topic_name(topic_name),
		  m_initialized(false)
	{

		const uint16_t profile = device.get_device_profile_number();

		if (profile != 402)
		{
			throw std::runtime_error("JointStatePublisher can only be used with a CiA 402 device."
									 " You passed a device with profile number " +
									 std::to_string(profile));
		}

		const Value operation_mode = device.get_entry("Modes of operation display");

		// TODO: look into INTERPOLATED_POSITION_MODE
		if (operation_mode != Profiles::constants.at(402).at("profile_position_mode") && operation_mode != Profiles::constants.at(402).at("interpolated_position_mode") && operation_mode != Profiles::constants.at(402).at("profile_velocity_mode") && operation_mode != Profiles::constants.at(402).at("torque_mode"))
		{
			throw std::runtime_error("[JointStatePublisher] Only profile_position/profile_velocity/torque mode supported yet."
									 " Try device.set_entry(\"modes_of_operation\", device.get_constant(\"profile_position_mode\"));");
		}

		if (m_topic_name.empty())
		{
			uint8_t node_id = device.get_node_id();
			m_topic_name = "device" + std::to_string(node_id) + "/set_joint_state";
		}
	}

	void JointStateSubscriber::advertise()
	{

		assert(!m_topic_name.empty());
		DEBUG_LOG("Advertising " << m_topic_name);
		ros::NodeHandle nh;
		m_subscriber = nh.subscribe(m_topic_name, queue_size, &JointStateSubscriber::receive, this);
		m_initialized = true;
	}

	void JointStateSubscriber::receive(const sensor_msgs::JointState &msg)
	{

		try
		{

			// Read position value from message
			assert(msg.position.size() > 0);
			const int32_t pos = rev_to_pos(msg.position[0]);

			// Read velocity value from message
			assert(msg.velocity.size() > 0);
			const int32_t vel = rev_p_s_to_vel(msg.velocity[0]);

			// Read torque value from message
			assert(msg.effort.size() > 0);
			const int32_t tor = eff_to_tor(msg.velocity[0]);

			DEBUG_LOG("Received JointState message");
			DEBUG_LOG("Position:");
			DEBUG_DUMP(pos);
			DEBUG_DUMP(msg.position[0]);
			DEBUG_LOG("Velocity:");
			DEBUG_DUMP(vel);
			DEBUG_DUMP(msg.velocity[0]);
			DEBUG_LOG("Torque:");
			DEBUG_DUMP(tor);
			DEBUG_DUMP(msg.effort[0]);

			const Value operation_mode = m_device.get_entry("Modes of operation display");

			if (operation_mode == Profiles::constants.at(402).at("profile_position_mode"))
			{
				DEBUG_LOG("Executing target position");
				m_device.execute("set_target_position", pos);
			}
			else if (operation_mode == Profiles::constants.at(402).at("profile_velocity_mode"))
			{
				DEBUG_LOG("Executing target velocity");
				m_device.execute("set_target_velocity", vel);
			}
			else if (operation_mode == Profiles::constants.at(402).at("torque_mode"))
			{
				DEBUG_LOG("Executing target torque");
				m_device.execute("set_target_torque", tor);
			}
			else
			{
				ERROR("[JointStatePublisher] unkown/unsupported mode in execution");
			}
		}
		catch (const sdo_error &error)
		{
			// TODO: only catch timeouts?
			ERROR("Exception in JointStateSubscriber::receive(): " << error.what());
		}
	}

	int32_t JointStateSubscriber::rev_to_pos(double rad) const
	{
		const double p = rad;
		const double min = m_position_0_degree;
		const double max = m_position_360_degree;
		const double dist = max - min;
		const double result = (p * dist) + min;
		return (int32_t)result;
	}

	int32_t JointStateSubscriber::rev_p_s_to_vel(double rad) const
	{
		const double v = rad;
		const double nom = m_velocity_nominator;
		const double denom = m_velocity_denominator;
		const double result = v / (nom / denom);
		return (int32_t)result;
	}

	int32_t JointStateSubscriber::eff_to_tor(double eff) const
	{
		return (int32_t)eff * 1000;
	}

} // end namespace kaco
