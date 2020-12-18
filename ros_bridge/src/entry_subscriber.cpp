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

#include "entry_subscriber.h"
#include "utils.h"
#include "logger.h"
#include "ros/ros.h"
#include "sdo_error.h"

#include <string>

namespace kaco
{

	EntrySubscriber::EntrySubscriber(Device &device, const std::string &entry_name, const WriteAccessMethod access_method)
		: m_device(device), m_entry_name(entry_name), m_access_method(access_method)
	{

		uint8_t node_id = device.get_node_id();
		m_device_prefix = "device" + std::to_string(node_id) + "/";
		// no spaces and '-' allowed in ros names
		m_name = Utils::escape(entry_name);
		m_type = device.get_entry_type(entry_name);
	}

	void EntrySubscriber::advertise()
	{

		std::string topic = m_device_prefix + "set_" + m_name;
		DEBUG_LOG("Advertising " << topic);
		ros::NodeHandle nh;

		switch (m_type)
		{
		case Type::uint8:
			m_subscriber = nh.subscribe(topic, queue_size, &EntrySubscriber::receive_uint8, this);
			PRINT("Type is uint8");
			break;
		case Type::uint16:
			m_subscriber = nh.subscribe(topic, queue_size, &EntrySubscriber::receive_uint16, this);
			PRINT("Type is unit16");
			break;
		case Type::uint32:
			m_subscriber = nh.subscribe(topic, queue_size, &EntrySubscriber::receive_uint32, this);
			PRINT("Type is uint32");
			break;
		case Type::int8:
			m_subscriber = nh.subscribe(topic, queue_size, &EntrySubscriber::receive_int8, this);
			PRINT("Type is int8");
			break;
		case Type::int16:
			PRINT("Type is int16");
			m_subscriber = nh.subscribe(topic, queue_size, &EntrySubscriber::receive_int16, this);
			break;
		case Type::int32:
			m_subscriber = nh.subscribe(topic, queue_size, &EntrySubscriber::receive_int32, this);
			PRINT("Type is int32");
			break;
		case Type::boolean:
			m_subscriber = nh.subscribe(topic, queue_size, &EntrySubscriber::receive_boolean, this);
			PRINT("Type is bool");
			break;
		case Type::string:
			m_subscriber = nh.subscribe(topic, queue_size, &EntrySubscriber::receive_string, this);
			PRINT("Type is string");
			break;
		default:
			ERROR("[EntryPublisher::advertise] Invalid entry type.")
		}
	}

	void EntrySubscriber::receive_uint8(const std_msgs::UInt8 &msg)
	{
		try
		{
			DEBUG_LOG("Recieved msg: " << msg.data);
			m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
		}
		catch (const sdo_error &error)
		{
			// TODO: only catch timeouts?
			ERROR("Exception in EntrySubscriber::receive_uint8(): " << error.what());
		}
	}

	void EntrySubscriber::receive_uint16(const std_msgs::UInt16 &msg)
	{
		try
		{
			DEBUG_LOG("Recieved msg: " << msg.data);
			m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
		}
		catch (const sdo_error &error)
		{
			// TODO: only catch timeouts?
			ERROR("Exception in EntrySubscriber::receive_uint16(): " << error.what());
		}
	}

	void EntrySubscriber::receive_uint32(const std_msgs::UInt32 &msg)
	{
		try
		{
			DEBUG_LOG("Recieved msg: " << msg.data);
			m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
		}
		catch (const sdo_error &error)
		{
			// TODO: only catch timeouts?
			ERROR("Exception in EntrySubscriber::receive_uint32(): " << error.what());
		}
	}

	void EntrySubscriber::receive_int8(const std_msgs::Int8 &msg)
	{
		try
		{
			DEBUG_LOG("Recieved msg: " << msg.data);
			m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
		}
		catch (const sdo_error &error)
		{
			// TODO: only catch timeouts?
			ERROR("Exception in EntrySubscriber::receive_int8(): " << error.what());
		}
	}

	void EntrySubscriber::receive_int16(const std_msgs::Int16 &msg)
	{
		try
		{
			DEBUG_LOG("Recieved msg: " << msg.data);
			m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
		}
		catch (const sdo_error &error)
		{
			// TODO: only catch timeouts?
			ERROR("Exception in EntrySubscriber::receive_int16(): " << error.what());
		}
	}

	void EntrySubscriber::receive_int32(const std_msgs::Int32 &msg)
	{
		try
		{
			DEBUG_LOG("Recieved msg: " << msg.data);
			m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
		}
		catch (const sdo_error &error)
		{
			// TODO: only catch timeouts?
			ERROR("Exception in EntrySubscriber::receive_int32(): " << error.what());
		}
	}

	void EntrySubscriber::receive_boolean(const std_msgs::Bool &msg)
	{
		try
		{
			DEBUG_LOG("Recieved msg: " << msg.data);
			m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
		}
		catch (const sdo_error &error)
		{
			// TODO: only catch timeouts?
			ERROR("Exception in EntrySubscriber::receive_boolean(): " << error.what());
		}
	}

	void EntrySubscriber::receive_string(const std_msgs::String &msg)
	{
		try
		{
			DEBUG_LOG("Recieved msg: " << msg.data);
			m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
		}
		catch (const sdo_error &error)
		{
			// TODO: only catch timeouts?
			ERROR("Exception in EntrySubscriber::receive_string(): " << error.what());
		}
	}

} // end namespace kaco
