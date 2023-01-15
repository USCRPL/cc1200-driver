#include "CC1200Morse.h"

#include <cinttypes>

// Morse code tables.
// Covers ASCII ranges 0x41-0x5A and 0x61-0x7A
char const * const alphaMorse[] = {".-","-...","-.-.","-..",".","..-.","--.","....","..",".---",
					"-.-",".-..","--","-.","---",".--.","--.-",".-.","...","-","..-",
					"...-",".--","-..-","-.--","--.."};

// Covers ASCII range 0x30-0x39
char const * const numMorse[] = {"-----",".----","..---","...--","....-",".....","-....","--...","---..","----."};

// covers ASCII range 0x21-0x2F
char const * const punctuation1Morse[] = {"-.-.--", ".-..-.", nullptr, "...-..-", nullptr, ".-...", ".----.", "-.--.",
										  "-.--.-", nullptr, ".-.-.", "--..--", nullptr, ".-.-.-", "-..-."};

// covers ASCII range 0x3A-0x40
char const * const punctuation2Morse[] = {"---...", "-.-.-.", nullptr, "-...-", nullptr, "..--..", ".--.-."};

void CC1200Morse::configure(CC1200::Band band, float radioFrequency, float morseTimePeriod, float transmitPower)
{
	radio.setPacketMode(CC1200::PacketMode::FIXED_LENGTH);
	radio.setCRCEnabled(false);

	// set frequency
	radio.setSymbolRate(1/morseTimePeriod);
	radio.setRadioFrequency(band, radioFrequency);

	// disable anything getting sent before the data
	radio.configureSyncWord(0x0, CC1200::SyncMode::SYNC_NONE, 8);
	radio.configurePreamble(0, 0);

	// configure OOK modulation
	radio.setModulationFormat(CC1200::ModFormat::ASK);
	radio.disablePARamping();
	radio.setASKPowers(transmitPower, CC1200::ASK_MIN_POWER_OFF);
}

// helper function for convertToMorse
static bool appendBits(uint8_t *outputBuffer, size_t bufferLen, uint8_t & nextBit, size_t & currByte, uint8_t toAppend, size_t count)
{
	//printf("appendBits(%" PRIu8 ", %zu)\n", toAppend, count);
	for(size_t counter = 0; counter < count; ++counter)
	{
		outputBuffer[currByte] |= toAppend << nextBit;

		if(nextBit == 0)
		{
			nextBit = 7;
			currByte += 1;
			if(currByte >= bufferLen)
			{
				// out of space
				return false;
			}
		}
		else
		{
			nextBit--;
		}
	}

	return true;
}


CC1200Morse::EncodedMorse CC1200Morse::convertToMorse(const char *string, uint8_t *outputBuffer, size_t bufferLen)
{
	memset(outputBuffer, 0, bufferLen);

	// place in the output buffer where next items will be written
	uint8_t nextBit = 7;
	size_t currByte = 0;

	EncodedMorse encoded;
	encoded.buffer = outputBuffer;
	encoded.valid = false;

	if(!appendBits(outputBuffer, bufferLen, nextBit, currByte, 0, spaceBefore))
	{
		return encoded;
	}

	size_t stringLength = strlen(string);
	for(size_t charIndex = 0; charIndex < stringLength; ++charIndex)
	{
		char currChar = string[charIndex];
		char const * morseToAppend = nullptr;
		if((currChar >= 'A' && currChar <= 'Z'))
		{
			morseToAppend = alphaMorse[currChar - 'A'];
		}
		else if(currChar >= 'a' && currChar <= 'z')
		{
			morseToAppend = alphaMorse[currChar - 'a'];
		}
		else if(currChar >= '0' && currChar <= '9')
		{
			morseToAppend = numMorse[currChar - '0'];
		}
		else if(currChar >= '!' && currChar <= '/')
		{
			morseToAppend = punctuation1Morse[currChar - '!'];
		}
		else if(currChar >= ':' && currChar <= '@')
		{
			morseToAppend = punctuation2Morse[currChar - ':'];
		}
		else if(currChar == '_') // underscore is off by itself in the ASCII chart
		{
			morseToAppend = "..--.-";
		}

		// append bit timings
		//printf("currChar = '%c'\n", currChar);
		if(currChar == ' ')
		{
			// space between words is 7 time units
			if(!appendBits(outputBuffer, bufferLen, nextBit, currByte, 0, 7))
			{
				return encoded;
			}
		}
		else if(morseToAppend != nullptr)
		{
			size_t morseLength = strlen(morseToAppend);
			for(size_t morseIndex = 0; morseIndex < morseLength; ++morseIndex)
			{
				// dot is 1 time unit, dash is 3 time units
				if(!appendBits(outputBuffer, bufferLen, nextBit, currByte, 1, morseToAppend[morseIndex] == '-' ? 3 : 1))
				{
					return encoded;
				}

				// space between symbols is 1 time unit
				if(!appendBits(outputBuffer, bufferLen, nextBit, currByte, 0, 1))
				{
					return encoded;
				}
			}

			// extra space between letters is 2 time units
			if(!appendBits(outputBuffer, bufferLen, nextBit, currByte, 0, 2))
			{
				return encoded;
			}
		}
	}

	if(!appendBits(outputBuffer, bufferLen, nextBit, currByte, 0, spaceAfter))
	{
		return encoded;
	}

	encoded.valid = true;
	encoded.byteLen = currByte;
	encoded.bitLen = 7 - nextBit;
	encoded.totalLength = currByte + (encoded.bitLen > 0 ? 1 : 0);

	return encoded;
}

void CC1200Morse::transmit(const CC1200Morse::EncodedMorse &morse)
{
	if(morse.totalLength > 128)
	{
		// too large to send in one packet
		return;
	}
	radio.setPacketLength(morse.byteLen, morse.bitLen);
	radio.enqueuePacket(reinterpret_cast<const char *>(morse.buffer), morse.totalLength);
}