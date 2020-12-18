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

#include "master.h"
#include "publisher.h"
#include "subscriber.h"

#include <string>
#include <vector>
#include <memory>
#include <forward_list>
#include <mutex>


namespace kaco {

	/// This class is a bridge between a ROS network and a CanOpen network.
	class Bridge {

	public:

		/// Runs the ROS spinner and blocks until shutdown (e.g. via Ctrl+c).
		void run();

		/// Adds a Publisher, advertises it and publishes messages repeatedly.
		/// \param publisher The publisher. It's a smart pointer because references
		///   to a publisher may never change after advertising it to ROS.
		/// \param loop_rate Publishing rate in hertz. Default is 10 Hz.
		void add_publisher(std::shared_ptr<Publisher> publisher, double loop_rate = 10);

		/// Adds a Subscriber, which can advertise itself and receive
		/// messages on its own.
		/// \param subscriber The subscriber. It's a smart pointer because references
		///   to a subscriber may never change after advertising it to ROS.
		void add_subscriber(std::shared_ptr<Subscriber> subscriber);

	private:

		static const bool debug = false;

		std::vector<std::shared_ptr<Publisher>> m_publishers;
		std::vector<std::shared_ptr<Subscriber>> m_subscribers;
		std::forward_list<std::future<void>> m_futures;

	};

} // end namespace kaco