// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*! 
 * @file XMLProfilesExampleSubscriber.cpp
 * This file contains the implementation of the subscriber functions.
 *
 * This file was generated by the tool fastcdrgen.
 */

#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/attributes/SubscriberAttributes.h>

#include <fastrtps/Domain.h>

#include "XMLProfilesExampleSubscriber.h"


XMLProfilesExampleSubscriber::XMLProfilesExampleSubscriber() : mp_participant(nullptr), mp_subscriber(nullptr) {}

XMLProfilesExampleSubscriber::~XMLProfilesExampleSubscriber() {	Domain::removeParticipant(mp_participant);}

bool XMLProfilesExampleSubscriber::init()
{
	// Create RTPSParticipant
	std::string participant_profile_name = "participant_profile";
	mp_participant = Domain::createParticipant(participant_profile_name);
	if(mp_participant == nullptr)
		return false;

	// Register the type
	Domain::registerType(mp_participant,(TopicDataType*) &myType);

	// Create Subscriber
	std::string subscriber_profile_name = "subscriber_profile";
	mp_subscriber = Domain::createSubscriber(mp_participant,subscriber_profile_name,(SubscriberListener*)&m_listener);
	if(mp_subscriber == nullptr)
		return false;

	return true;
}

void XMLProfilesExampleSubscriber::SubListener::onSubscriptionMatched(Subscriber* sub,MatchingInfo& info)
{
	if (info.status == MATCHED_MATCHING)
	{
		n_matched++;
		std::cout << "Subscriber matched" << std::endl;
	}
	else
	{
		n_matched--;
		std::cout << "Subscriber unmatched" << std::endl;
	}
}

void XMLProfilesExampleSubscriber::SubListener::onNewDataMessage(Subscriber* sub)
{
		// Take data
		XMLProfilesExample st;

		if(sub->takeNextData(&st, &m_info))
		{
			if(m_info.sampleKind == ALIVE)
			{
				// Print your structure data here.
				++n_msg;
				std::cout << "Sample received, count=" << n_msg << std::endl;
			}
		}
}

void XMLProfilesExampleSubscriber::run()
{
	std::cout << "Waiting for Data, press Enter to stop the Subscriber. "<<std::endl;
	std::cin.ignore();
	std::cout << "Shutting down the Subscriber." << std::endl;
}

