/*
 * Copyright (c) 2009-2010, Martin Rosekeit
 * Copyright (c) 2009-2011, Fabian Greif
 * Copyright (c) 2009, 2011, Georgi Grinshpun
 * Copyright (c) 2012-2015, Niklas Hauser
 * Copyright (c) 2013, Sascha Schade
 * Copyright (c) 2015, Kevin Läufer
 * Copyright (c) 2016, Julia Gutheil
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef	XPCC_DISPATCHER_HPP
#define	XPCC_DISPATCHER_HPP

#include <modm/processing/timer.hpp>
#include <modm/container/linked_list.hpp>

#include "backend/backend_interface.hpp"
#include "postman/postman.hpp"

#include "response_callback.hpp"

namespace xpcc
{
	/**
	 * \brief
	 *
	 * \todo	Documentation
	 *
	 * \author	Georgi Grinshpun
	 * \ingroup	modm_communication_xpcc
	 */
	class Dispatcher
	{
	public:
		static constexpr std::chrono::milliseconds acknowledgeTimeout{ 100 };
		static constexpr std::chrono::milliseconds responseTimeout{ 200 };

	public:
		Dispatcher(BackendInterface *backend, Postman* postman);

		void
		update();

	private:
		/// Does not handle requests which are not acknowledge.
		bool
		handlePacket(const Header& header, const modm::SmartPointer& payload);

		/// Sends messages which are waiting in the list.
		void
		handleWaitingMessages();

		/**
		 * \brief 	This class holds information about a Message being send.
		 * 			This is the superclass of all entries.
		 */
		class Entry
		{
		public:
			enum class Type
			{
				Default,
				Callback,
			};

			/// Communication info, state of sending and retrieving messages and acks.
			enum State
			{
				TransmissionPending,
				WaitForACK,
				WaitForResponse,
			};

		public:
			/**
			 * \brief 	Creates one Entry with given header.
			 * 			this->next is initially set Null
			 *
			 * Creates one Entry with given header. this->next is initially
			 * set Null. The const member this->typeInfo is set to typeInfo
			 * and never else changed. this->typeInfo replaces runtime
			 * information needed by handling of messages.
			 */
			Entry(Type type, const Header& inHeader, modm::SmartPointer& inPayload) :
				type(type),
				header(inHeader), payload(inPayload)
			{
			}

			Entry(const Header& inHeader, modm::SmartPointer& inPayload) :
				header(inHeader), payload(inPayload)
			{
			}

			Entry(const Header& inHeader) :
				header(inHeader)
			{
			}

			Entry(const Header& inHeader,
					modm::SmartPointer& inPayload, ResponseCallback& callback_) :
				type(Type::Callback),
				header(inHeader), payload(inPayload),
				callback(callback_)
			{
			}

			/**
			 * \brief 	Checks if a Response or Acknowledge fits to the
			 * 			Message represented by this Entry.
			 */
			bool
			headerFits(const Header& header) const;

			inline void
			callbackResponse(const Header& header, const modm::SmartPointer &payload) const
			{
				this->callback.call(header, payload);
			}

			const Type type = Type::Default;
			const Header header;
			const modm::SmartPointer payload;
			State state = State::TransmissionPending;
			modm::ShortTimeout time;
			uint8_t tries = 0;
		private:
			ResponseCallback callback;
		};

		void
		addMessage(const Header& header, modm::SmartPointer& smartPayload);

		void
		addMessage(const Header& header, modm::SmartPointer& smartPayload,
				ResponseCallback& responseCallback);

		void
		addResponse(const Header& header, modm::SmartPointer& smartPayload);

		inline void
		handleActionCall(const Header& header, const modm::SmartPointer& payload);

		void
		sendAcknowledge(const Header& header);

		using EntryList = modm::LinkedList<Entry>;
		using EntryIterator = EntryList::iterator;

		EntryIterator
		sendMessageToInnerComponent(EntryIterator entry);

		BackendInterface * const backend;
		Postman * const postman;

		EntryList entries;

	private:
		friend class Communicator;
	};
}

#endif // XPCC_DISPATCHER_HPP