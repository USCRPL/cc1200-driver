/*
 * Copyright (c) 2019-2023 USC Rocket Propulsion Lab
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CC1200_CC1200MORSE_H
#define CC1200_CC1200MORSE_H

#include "CC1200.h"

/**
 * @brief Class allowing you to transmit Morse code using the CC1200 radio
 */
class CC1200Morse
{
	CC1200 & radio;

	// time units at the start and end of the morse message.
	const size_t spaceBefore = 3;
	const size_t spaceAfter = 3;

public:

	CC1200Morse(CC1200 & _radio):
	radio(_radio){}

	/**
	 * Configure the CC1200 to transmit morse code.
	 *
	 * @param band Radio band containing the frequency.
	 * @param radioFrequency Frequency to transmit on.
	 * @param morseTimePeriod Time unit to use when transmitting.
	 * @param transmitPower Power in dBm to transmit at.  Must be in the CC1200 allowed range.
	 *
	 * Dots are one time unit, dashes are three.
	 * 50ms is about the fastest a human can understand, while 125ms is a more reasonable speed.
	 * Specifics are here: http://www.codebug.org.uk/learn/step/541/morse-code-timing-rules/#:~:text=The%20space%20between%20symbols%20(dots,words%20is%207%20time%20units.
	 */
	void configure(CC1200::Band band, float radioFrequency, std::chrono::milliseconds morseTimePeriod, float transmitPower);

	struct EncodedMorse
	{
		/// Whether this morse data is valid. If false, it couldn't be encoded due to an error.
		bool valid;

		/// Buffer storing morse data.  Uses memory passed to #CC1200Morse::convertToMorse()
		uint8_t const * buffer;

		/// Number of complete bytes and bits used in the data
		size_t byteLen;
		uint8_t bitLen;

		/// Number of bytes in the buffer that are at least partially filled.
		size_t totalLength;
	};

	/**
	 * Convert ASCII text into characters suitable to be sent over the radio.
	 * Some ASCII characters do not have a morse equivalent, these will be removed.
	 *
	 * @param string ASCII string to convert.
	 * @param outputBuffer Buffer to write morse into.  Will be zeroed.
	 * @return Encoded morse data, if there was insufficient space valid will be false.
	 */
	EncodedMorse convertToMorse(const char* string, uint8_t * outputBuffer, size_t bufferLen);

	/**
	 * Queue this morse code to be transmitted over the radio.
	 * Packets with a total length larger than 128 bytes cannot be transmitted (though this limit
	 * could be worked around).
	 *
	 * The packet will be transmitted as soon as the radio is switched to TX state.
	 * If it's already in TX then it will be transmitted immediately.
	 * @param morse
	 */
	void transmit(EncodedMorse const & morse);
};


#endif //CC1200_CC1200MORSE_H