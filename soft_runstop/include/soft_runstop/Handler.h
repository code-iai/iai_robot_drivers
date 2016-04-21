/*
 * Copyright (c) 2010, Ingo Kresse <kresse@in.tum.de>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Technical University Munich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <ros/ros.h>

#include <std_msgs/Bool.h>

using namespace ros;

using namespace std_msgs;

namespace soft_runstop
{
	class Handler
	{
		private: NodeHandle nodeHandle;
		private: Duration guardTime;
		private: Subscriber softRunstopSubscriber;

		private: Time time;
		private: bool state;

		public: Handler(Duration guardTime)
		{
			this->guardTime = guardTime;
			this->softRunstopSubscriber = nodeHandle.subscribe("/soft_runstop", 1, &Handler::softRunstopCallback, this);

			this->time = Time::now();
			this->state = true;
		}

		public: bool getState()
		{
			if (Time::now() - time > guardTime) return true;

			return state;
		}

		private: void softRunstopCallback(const BoolConstPtr message)
		{
			time = Time::now();
			state = message->data;
		}
	};
};
