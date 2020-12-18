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

#include "joint_state_publisher.h"
#include "utils.h"
#include "logger.h"
#include "profiles.h"
#include "sdo_error.h"

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include <string>
#include <stdexcept>

namespace kaco
{

	JointStatePublisher::JointStatePublisher(Device &device,
											 int32_t position_0_degree,
											 int32_t position_360_degree,
											 int32_t velocity_nominator,
											 int32_t velocity_denominator,
											 const std::string &position_actual_field,
											 const std::string &velocity_actual_field,
											 const std::string &torque_actual_field,
											 const std::string &topic_name)
		: m_device(device),
		  m_position_0_degree(position_0_degree),
		  m_position_360_degree(position_360_degree),
		  m_velocity_nominator(velocity_nominator),
		  m_velocity_denominator(velocity_denominator),
		  m_position_actual_field(position_actual_field),
		  m_velocity_actual_field(velocity_actual_field),
		  m_torque_actual_field(torque_actual_field),
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

		if (m_topic_name.empty())
		{
			uint8_t node_id = device.get_node_id();
			m_topic_name = "device" + std::to_string(node_id) + "/get_joint_state";
		}
	}

	void JointStatePublisher::advertise()
	{

		assert(!m_topic_name.empty());
		DEBUG_LOG("Advertising " << m_topic_name);
		ros::NodeHandle nh;
		m_publisher = nh.advertise<sensor_msgs::JointState>(m_topic_name, queue_size);
		m_initialized = true;
	}

	void JointStatePublisher::publish()
	{

		try
		{

			if (!m_initialized)
			{
				throw std::runtime_error("[JointStatePublisher] publish() called before advertise().");
			}

			sensor_msgs::JointState js;

			// only position and velocity supported yet.

			js.name.resize(1);
			js.position.resize(1);
			js.velocity.resize(1);
			js.effort.resize(1);

			const int32_t pos = m_device.get_entry(m_position_actual_field);
			const int32_t vel = m_device.get_entry(m_velocity_actual_field);
			const int32_t tor = m_device.get_entry(m_torque_actual_field);

			js.name[0] = m_topic_name;

			js.header.stamp = ros::Time::now();
			js.position[0] = pos_to_rev(pos);
			js.velocity[0] = vel_to_rev_p_s(vel);
			js.effort[0] = tor_to_eff(tor);

			DEBUG_LOG("Sending JointState message");
			DEBUG_LOG("Position:");
			DEBUG_DUMP(pos);
			DEBUG_DUMP(js.position[0]);
			DEBUG_LOG("Velocity:");
			DEBUG_DUMP(vel);
			DEBUG_DUMP(js.velocity[0]);
			DEBUG_LOG("Effort:");
			DEBUG_DUMP(tor);
			DEBUG_DUMP(js.effort[0]);

			m_publisher.publish(js);
		}
		catch (const sdo_error &error)
		{
			// TODO: only catch timeouts?
			ERROR("Exception in JointStatePublisher::publish(): " << error.what());
		}
	}

	double JointStatePublisher::pos_to_rev(int32_t pos) const
	{
		const double p = pos;
		const double min = m_position_0_degree;
		const double max = m_position_360_degree;
		const double dist = max - min;
		const double result = ((p - min) / dist);
		return result;
	}

	double JointStatePublisher::vel_to_rev_p_s(int32_t vel) const
	{
		const double v = vel;
		const double nom = m_velocity_nominator;
		const double denom = m_velocity_denominator;
		const double result = v * (nom / denom);
		return result;
	}

	double JointStatePublisher::tor_to_eff(int32_t tor) const
	{
		return (double)((double)tor) / 1000.0;
	}

} // end namespace kaco
