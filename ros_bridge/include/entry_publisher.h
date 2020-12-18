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

namespace kaco
{

	/// This class provides a Publisher implementation for
	/// use with kaco::Bridge. It publishes a value from
	/// a device's dictionary.
	class EntryPublisher : public Publisher
	{

	public:
		/// Constructor
		/// \param device The CanOpen device
		/// \param entry_name The name of the entry. See device profile.
		/// \param access_method You can choose default/sdo/pdo method. See kaco::Device docs.
		EntryPublisher(Device &device, const std::string &entry_name, const ReadAccessMethod access_method = ReadAccessMethod::use_default);

		/// \see interface Publisher
		void advertise() override;

		/// \see interface Publisher
		void publish() override;

	private:
		static const bool debug = false;

		// TODO: let the user change this?
		static const unsigned queue_size = 100;

		ros::Publisher m_publisher;
		std::string m_device_prefix;
		std::string m_name;

		Device &m_device;
		std::string m_entry_name;
		ReadAccessMethod m_access_method;
		Type m_type;
	};

} // end namespace kaco