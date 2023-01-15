#pragma clang diagnostic push
#pragma ide diagnostic ignored "readability-magic-numbers"
//
// Created by jamie on 3/27/2020.
//

#include "CC1200.h"
#include "CC1200Bits.h"

#include <cinttypes>
#include <cmath>
#include <array>

// change to 1 to print debug info
#define CC1200_DEBUG 1

// change to 1 to print register read/write level debug info
#define CC1200_REGISTER_LEVEL_DEBUG 0

// miscellaneous constants
#define CC1200_READ (1 << 7) // SPI initial byte flag indicating read
#define CC1200_WRITE 0 // SPI initial byte flag indicating write
#define CC1200_BURST (1 << 6) // SPI initial byte flag indicating burst access

// SPI commands to access data buffers.  Can be used with CC1200_BURST.
#define CC1200_ENQUEUE_TX_FIFO 0x3F
#define CC1200_DEQUEUE_RX_FIFO 0xBF

// SPI command to access FIFO memory (or several other areas depending on mode)
#define CC1200_MEM_ACCESS 0x3E

#define CC1200_RX_FIFO (1 << 7) // address flag to access RX FIFO
#define CC1200_TX_FIFO 0 // address flag to access TX FIFO


#define CC1200_PART_NUMBER ((uint8_t)0x20) // part number we expect the chip to read
#define CC1201_PART_NUMBER ((uint8_t)0x21)
#define CC1200_EXT_ADDR 0x2F // SPI initial byte address indicating extended register space

#define SPI_MODE 0
#define SPI_FREQ 5000000 // hz
// NOTE: the chip supports a higher frequency for most operations but reads to extended registers require a lower frequency

// frequency of the chip's crystal oscillator
#define CC1200_OSC_FREQ 40000000 // hz
#define CC1200_OSC_FREQ_LOG2 25.253496f // log2 of above number

// length of the TX and RX FIFOS
#define CC1200_FIFO_SIZE 128

// maximum length of the packets we can send, including the length byte which we add.
// Since the TX and RX FIFOs are 128 bytes, supporting packet lengths longer than 128 bytes
// requires streaming bytes in during the transmission, which would make things complicated.
#define MAX_PACKET_LENGTH 128

// Length of the status bytes that can be appended to packets
#define PACKET_STATUS_LEN 2U

// utility function: compile-time power calculator.
// Works on all signed and unsigned integer types for T.
// from: http://prosepoetrycode.potterpcs.net/2015/07/a-simple-constexpr-power-function-c/
template <typename T>
constexpr T constexpr_pow(T num, unsigned int pow)
{
	return pow == 0 ? 1 : num * constexpr_pow(num, pow-1);
}

// power of two constants
const float twoToThe16 = constexpr_pow(2.0f, 16);
const float twoToThe20 = constexpr_pow(2.0f, 20);
const float twoToThe21 = constexpr_pow(2.0f, 21);
const float twoToThe22 = constexpr_pow(2.0f, 22);
const float twoToThe38 = constexpr_pow(2.0f, 38);
const float twoToThe39 = constexpr_pow(2.0f, 39);

// binary value size constants
const size_t maxValue3Bits = constexpr_pow(2, 3) - 1;
const size_t maxValue4Bits = constexpr_pow(2, 4) - 1;
const size_t maxValue8Bits = constexpr_pow(2, 8) - 1;
const size_t maxValue20Bits = constexpr_pow(2, 20) - 1;
const size_t maxValue24Bits = constexpr_pow(2, 24) - 1;

CC1200::CC1200(PinName mosiPin, PinName misoPin, PinName sclkPin, PinName csPin, PinName rstPin, Stream * _debugStream, bool _isCC1201):
spi(mosiPin, misoPin, sclkPin, csPin, use_gpio_ssel),
rst(rstPin, 1),
debugStream(_debugStream),
isCC1201(_isCC1201)
{
	spi.format(8, SPI_MODE);
	spi.frequency(SPI_FREQ);
}

bool CC1200::begin()
{
	chipReady = false;

	// reset
	rst.write(0);
	wait_us(100);
	rst.write(1);

	const auto resetTimeout = 10ms;
	Timer timeoutTimer;
	timeoutTimer.start();

	while(!chipReady)
	{
		// datasheet specifies 240us reset time
		wait_us(250);
		updateState();

		if(timeoutTimer.elapsed_time() > resetTimeout)
		{
			debugStream->printf("Timeout waiting for ready response from CC1200\n");
			break;
		}
	}

	// read ID register
	uint8_t partNumber = readRegister(ExtRegister::PARTNUMBER);
	uint8_t partVersion = readRegister(ExtRegister::PARTVERSION);

	uint8_t expectedPartNumber = isCC1201 ? CC1201_PART_NUMBER : CC1200_PART_NUMBER;
	if(partNumber != expectedPartNumber)
	{
		debugStream->printf("Read incorrect part number 0x%" PRIx8 " from CC1200, expected 0x%" PRIx8 "\n", partNumber, expectedPartNumber);
		return false;
	}

#if CC1200_DEBUG
	debugStream->printf("Detected CC1200, Part Number 0x%" PRIx8 ", Hardware Version %" PRIx8 "\n", partNumber, partVersion);
#endif


	// Set packet format settings for this driver
	// ------------------------------------------------------------------------

	// enable CRC but disable status bytes
	writeRegister(Register::PKT_CFG1, (0b01 << PKT_CFG1_CRC_CFG));

	return true;
}

size_t CC1200::getTXFIFOLen()
{
	return readRegister(ExtRegister::NUM_TXBYTES);
}

size_t CC1200::getRXFIFOLen()
{
	return readRegister(ExtRegister::NUM_RXBYTES);
}

bool CC1200::enqueuePacket(char const * data, size_t len)
{
	uint8_t totalLength = len + 1; // add one byte for length byte

	if(totalLength > MAX_PACKET_LENGTH)
	{
		// packet too big
		return false;
	}

	uint8_t txFreeBytes = CC1200_FIFO_SIZE - getTXFIFOLen();
	if(totalLength > txFreeBytes)
	{
		// packet doesn't fit in TX FIFO
		return false;
	}

	// burst write to TX FIFO
	spi.select();
	loadStatusByte(spi.write(CC1200_ENQUEUE_TX_FIFO | CC1200_BURST));
	if(_packetMode == PacketMode::VARIABLE_LENGTH)
	{
		spi.write(len);
	}
	for(size_t byteIndex = 0; byteIndex < len; ++byteIndex)
	{
		spi.write(data[byteIndex]);
	}
	spi.deselect();

#if CC1200_REGISTER_LEVEL_DEBUG
	debugStream->printf("Wrote packet of data length %zu:", len);
	if(_packetMode == PacketMode::VARIABLE_LENGTH)
	{
		debugStream->printf(" %02" PRIx8, static_cast<uint8_t>(len));
	}
	for(size_t byteIndex = 0; byteIndex < len; ++byteIndex)
	{
		debugStream->printf(" %02" PRIx8, data[byteIndex]);
	}
	debugStream->printf("\n");
#endif
	return true;
}

bool CC1200::hasReceivedPacket()
{
	size_t bytesReceived = getRXFIFOLen();

	if(bytesReceived < 1)
	{
		// no bytes at all, can't check the length
		return false;
	}

	if(_packetMode == PacketMode::FIXED_LENGTH)
	{
		return bytesReceived >= _packetTotalLength + (appendStatusEnabled ? PACKET_STATUS_LEN : 0);
	}
	else if(_packetMode == PacketMode::INFINITE_LENGTH)
	{
		// Any amount of bytes constitutes a packet.
		return bytesReceived > 0;
	}
	else // _packetMode == PacketMode::VARIABLE_LENGTH
	{
		// get value of first byte of the packet, which is the length.

		// The datasheet is wrong about this!  It says that the first byte in the RX FIFO can
		// be found by accessing address RXFIRST via direct fifo access.
		// However, in my own testing, RXFIRST points to the second entry in the FIFO, and
		// the first entry must be accessed through RXFIFO_PRE_BUF.
		uint8_t packetLen = readRegister(ExtRegister::RXFIFO_PRE_BUF);

		// if we have received a full packet's worth of bytes, then we have received a full packet.
		return bytesReceived >= static_cast<size_t>(packetLen + 1 /* Add one because length field does not include itself */) + (appendStatusEnabled ? PACKET_STATUS_LEN : 0);
	}
}

size_t CC1200::receivePacket(char *buffer, size_t bufferLen)
{
	// burst read from RX FIFO
	spi.select();
	loadStatusByte(spi.write(CC1200_DEQUEUE_RX_FIFO | CC1200_BURST));

	uint8_t dataLen;
	if(_packetMode == PacketMode::VARIABLE_LENGTH)
	{
		// first read length byte
		dataLen = spi.write(0);
	}
	else // _packetMode == PacketMode::FIXED_LENGTH)
	{
		dataLen = _packetTotalLength;
	}

	for(size_t byteIndex = 0; byteIndex < dataLen; ++byteIndex)
	{
		uint8_t currByte = spi.write(0);
		if(byteIndex < bufferLen)
		{
			buffer[byteIndex] = currByte;
		}
	}

	if(appendStatusEnabled)
	{
		uint8_t statusBytes[PACKET_STATUS_LEN];
		for(size_t byteIndex = 0; byteIndex < PACKET_STATUS_LEN; ++byteIndex)
		{
			statusBytes[byteIndex] = spi.write(0);
		}

		lastRSSI = statusBytes[0];
		lastLQI = statusBytes[1] & 0b01111111;
	}

	spi.deselect();

#if CC1200_REGISTER_LEVEL_DEBUG
	debugStream->printf("Read packet of data length %" PRIu8 ": %" PRIx8, dataLen, static_cast<uint8_t>(dataLen));
	for(size_t byteIndex = 0; byteIndex < dataLen; ++byteIndex)
	{
		debugStream->printf(" %" PRIx8, buffer[byteIndex]);
	}
	debugStream->printf("\n");
#endif

	return dataLen;
}

size_t CC1200::writeStream(const char *buffer, size_t count)
{
	size_t freeBytes = CC1200_FIFO_SIZE - getTXFIFOLen();

	/*if(state == State::TX)
	{
		if(freeBytes > 0)
		{
			freeBytes--;
		}
	}*/

	size_t bytesToWrite = std::min(freeBytes, count);

	if(bytesToWrite == 0)
	{
		return 0;
	}

	spi.select();
	loadStatusByte(spi.write(CC1200_ENQUEUE_TX_FIFO | CC1200_BURST));
	for(size_t byteIndex = 0; byteIndex < bytesToWrite; ++byteIndex)
	{
		spi.write(buffer[byteIndex]);
	}
	spi.deselect();

#if CC1200_REGISTER_LEVEL_DEBUG
	debugStream->printf("%zu bytes were free, wrote stream of data length %zu:", freeBytes, bytesToWrite);
	for(size_t byteIndex = 0; byteIndex < bytesToWrite; ++byteIndex)
	{
		debugStream->printf(" %02" PRIx8, buffer[byteIndex]);
	}
	debugStream->printf("\n");
#endif

	return bytesToWrite;
}

bool CC1200::writeStreamBlocking(const char *buffer, size_t count)
{
	//size_t origCount = count;
	size_t bufferOffset = 0;
	while(state == State::TX && count > 0)
	{
		size_t bytesWritten = writeStream(buffer + bufferOffset, count);
		count -= bytesWritten;
		bufferOffset += bytesWritten;
	}

	//debugStream->printf("Read stream of data length %zu\n:", origCount);

	return count == 0;
}

size_t CC1200::readStream(char *buffer, size_t maxLen)
{
	size_t bytesToRead = std::min(maxLen, getRXFIFOLen());
	if(bytesToRead == 0)
	{
		return 0;
	}

	// burst read from RX FIFO
	spi.select();
	loadStatusByte(spi.write(CC1200_DEQUEUE_RX_FIFO | CC1200_BURST));
	for(size_t byteIndex = 0; byteIndex < bytesToRead; ++byteIndex)
	{
		buffer[byteIndex] = spi.write(0);
	}
	spi.deselect();

#if CC1200_REGISTER_LEVEL_DEBUG
	debugStream->printf("Read stream of data length %zu:", bytesToRead);
	for(size_t byteIndex = 0; byteIndex < bytesToRead; ++byteIndex)
	{
		debugStream->printf(" %" PRIx8, buffer[byteIndex]);
	}
	debugStream->printf("\n");
#endif

	return bytesToRead;
}

bool CC1200::readStreamBlocking(char *buffer, size_t count, std::chrono::microseconds timeout)
{
	//size_t origCount = count;
	Timer timeoutTimer;

	if(timeout > 0us)
	{
		timeoutTimer.start();
	}

	size_t bufferOffset = 0;
	while((timeoutTimer.elapsed_time() < timeout || timeout == 0us) && state == State::RX && count > 0)
	{
		size_t bytesRead = readStream(buffer + bufferOffset, count);
		count -= bytesRead;
		bufferOffset += bytesRead;
	}

	//debugStream->printf("Read stream of data length %zu, first %" PRIx8 " last %" PRIx8 "\n:", origCount,
	//		buffer[0], buffer[origCount - 1]);

	return count == 0;
}


// helper function: convert a state to the bits for RXOFF_MODE and TXOFF_MODE
inline uint8_t getOffModeBits(CC1200::State state)
{
	uint8_t offBits = 0b0;
	if(state == CC1200::State::IDLE)
	{
		offBits = 0b0;
	}
	else if(state == CC1200::State::FAST_ON)
	{
		offBits = 0b1;
	}
	else if(state == CC1200::State::TX)
	{
		offBits = 0b10;
	}
	else if(state == CC1200::State::RX)
	{
		offBits = 0b11;
	}
	return offBits;
}

void CC1200::setOnReceiveState(CC1200::State goodPacket, CC1200::State badPacket)
{
	// configure good packet action via RXOFF_MODE
	uint8_t rfendCfg1 = readRegister(Register::RFEND_CFG1);
	rfendCfg1 &= ~(0b11 << RFEND_CFG1_RXOFF_MODE);
	rfendCfg1 |= getOffModeBits(goodPacket) << RFEND_CFG1_RXOFF_MODE;
	writeRegister(Register::RFEND_CFG1, rfendCfg1);

	// configure bad packet action via TERM_ON_BAD_PACKET_EN
	uint8_t rfendCfg0 = readRegister(Register::RFEND_CFG0);
	if(badPacket == State::RX)
	{
		rfendCfg0 &= ~(1 << RFEND_CFG0_TERM_ON_BAD_PACKET_EN);
	}
	else
	{
		rfendCfg0 |= 1 << RFEND_CFG1_RXOFF_MODE;
	}
	writeRegister(Register::RFEND_CFG0, rfendCfg0);
}

void CC1200::setOnTransmitState(CC1200::State txState)
{
	uint8_t rfendCfg0 = readRegister(Register::RFEND_CFG0);
	rfendCfg0 &= ~(0b11 << RFEND_CFG0_TXOFF_MODE);
	rfendCfg0 |= getOffModeBits(txState) << RFEND_CFG0_TXOFF_MODE;
	writeRegister(Register::RFEND_CFG0, rfendCfg0);
}

void CC1200::setFSCalMode(FSCalMode mode)
{
	uint8_t settlingCfg = readRegister(Register::SETTLING_CFG);
	settlingCfg &= ~(0b11 << SETTLING_CFG_FS_AUTOCAL);
	settlingCfg |= static_cast<uint8_t>(mode) << SETTLING_CFG_FS_AUTOCAL;
	writeRegister(Register::SETTLING_CFG, settlingCfg);
}

void CC1200::configureGPIO(uint8_t gpioNumber, CC1200::GPIOMode mode, bool outputInvert)
{
	// gpio 3 is the first register, then it goes down to 0
	Register gpioReg = static_cast<Register>(static_cast<uint8_t>(Register::IOCFG3) + (3 - gpioNumber));

	uint8_t gpioCfgVal = static_cast<uint8_t>(mode);
	if(outputInvert)
	{
		gpioCfgVal |= (1 << GPIO_INV);
	}
	writeRegister(gpioReg, gpioCfgVal);
}

void CC1200::configureFIFOMode()
{
	// configure packet format
	uint8_t pktCfg2 = readRegister(Register::PKT_CFG2);
	pktCfg2 &= ~(0b11 << PKT_CFG2_PKT_FORMAT);
	writeRegister(Register::PKT_CFG2, pktCfg2);

	// enable fifo
	uint8_t mdmCfg1 = readRegister(Register::MDMCFG1);
	mdmCfg1 |= 1 << MDMCFG1_FIFO_EN;
	writeRegister(Register::MDMCFG1, mdmCfg1);

	// make sure transparent mode is disabled
	uint8_t mdmCfg0 = readRegister(Register::MDMCFG0);
	mdmCfg0 &= ~(1 << MDMCFG0_TRANSPARENT_MODE_EN);
	writeRegister(Register::MDMCFG0, mdmCfg0);
}

void CC1200::setPacketMode(PacketMode mode, bool appendStatus)
{
	_packetMode = mode;

	uint8_t pktCfg0 = readRegister(Register::PKT_CFG0);
	// set length config field
	pktCfg0 &= ~(0b11 << PKT_CFG0_LENGTH_CONFIG);
	pktCfg0 |= static_cast<uint8_t>(mode) << PKT_CFG0_LENGTH_CONFIG;
	writeRegister(Register::PKT_CFG0, pktCfg0);

	if(mode == PacketMode::VARIABLE_LENGTH)
	{
		// disable packet length limit
		writeRegister(Register::PKT_LEN, MAX_PACKET_LENGTH);
	}
	else if(mode == PacketMode::FIXED_LENGTH)
	{
		// reset to selected fixed lengths
		setPacketLength(_packetByteLength, _packetBitLength);
	}
	else
	{
		// Infinite length packets, PKT_LEN register is a don't care.
	}

	// set append status
	appendStatusEnabled = appendStatus;
	uint8_t pktCfg1 = readRegister(Register::PKT_CFG1);
	if(appendStatus)
	{
		pktCfg1 |= 1 << PKT_CFG1_APPEND_STATUS;
	}
	else
	{
		pktCfg1 &= ~(1 << PKT_CFG1_APPEND_STATUS);
	}
	writeRegister(Register::PKT_CFG1, pktCfg1);
}

void CC1200::setPacketLength(uint16_t length, uint8_t bitLength)
{
	_packetByteLength = length;
	_packetBitLength = bitLength;
	_packetTotalLength = _packetByteLength;

	if(bitLength > 0)
	{
		// tell the driver to read the extra bits into another byte
		_packetTotalLength++;
	}

	if(_packetTotalLength == 256)
	{
		// Length byte of 0 indicates 256 bytes
		writeRegister(Register::PKT_LEN, 0);
	}
	else
	{
		writeRegister(Register::PKT_LEN, _packetByteLength);
	}

	uint8_t pktCfg0 = readRegister(Register::PKT_CFG0);
	pktCfg0 &= ~(0b111 << PKT_CFG0_PKT_BIT_LEN);
	pktCfg0 |= _packetBitLength << PKT_CFG0_PKT_BIT_LEN;
	writeRegister(Register::PKT_CFG0, pktCfg0);

#if CC1200_DEBUG
	debugStream->printf("Set total length to %zu, byte length = %zu, bit length = %" PRIu8 "\n", _packetTotalLength, _packetByteLength, _packetBitLength);
#endif
}

void CC1200::setCRCEnabled(bool enabled)
{
	uint8_t pktCfg1 = readRegister(Register::PKT_CFG1);
	pktCfg1 &= ~(0b11 << PKT_CFG1_CRC_CFG);
	pktCfg1 |= (enabled ? 0b01 : 0b00) << PKT_CFG1_CRC_CFG;
	writeRegister(Register::PKT_CFG1, pktCfg1);
}

void CC1200::setModulationFormat(CC1200::ModFormat format)
{
	uint8_t modcfgDevE = readRegister(Register::MODCFG_DEV_E);
	modcfgDevE &= ~(0b111 << MODCFG_DEV_E_MOD_FORMAT);
	modcfgDevE |= static_cast<uint8_t>(format) << MODCFG_DEV_E_MOD_FORMAT;
	writeRegister(Register::MODCFG_DEV_E, modcfgDevE);
}

void CC1200::setFSKDeviation(float deviation)
{
	// Deviation is set as two values, an exponent register from 0-3 and a mantissa register from 0-256.
	// See user guide page 81 for the original equation, this function was worked out from that

	// First assume mantissa is zero and calculate the needed exponent
	float exactExponent = std::log2(deviation) - CC1200_OSC_FREQ_LOG2 + 14;
	uint8_t actualExponent = static_cast<uint8_t>(std::min(static_cast<float>(maxValue3Bits), exactExponent));
	// note: 14 comes from log2(2^22) - log2(256)

	float exactMantissa;
	if(actualExponent >= 1)
	{
		exactMantissa = (std::pow(2.0f, static_cast<float>(22 - actualExponent)) * deviation)
						/ CC1200_OSC_FREQ - 256;
	}
	else
	{
		// use alternate high-resolution formula for case where exponent = 0
		exactMantissa = deviation * twoToThe21 / CC1200_OSC_FREQ;
	}

	// now calculate closest mantissa
	uint8_t actualMantissa = static_cast<uint8_t>(std::min(static_cast<float>(maxValue8Bits), exactMantissa));

	// set exponent and mantissa
	writeRegister(Register::DEVIATION_M, actualMantissa);

	uint8_t modcfgDevE = readRegister(Register::MODCFG_DEV_E);
	modcfgDevE &= ~(0b111 << MODCFG_DEV_E_DEV_E);
	modcfgDevE |= actualExponent << MODCFG_DEV_E_DEV_E;
	writeRegister(Register::MODCFG_DEV_E, modcfgDevE);

#if CC1200_DEBUG
	debugStream->printf("Setting FSK deviation, requested +-%.00f Hz, setting DEV_E = 0x%" PRIx8 " DEV_M = 0x%" PRIx8 "\n",
			deviation, actualExponent, actualMantissa);

	float actualDeviation;
	if(actualExponent == 0)
	{
		actualDeviation = CC1200_OSC_FREQ * actualMantissa / twoToThe21;
	}
	else
	{
		actualDeviation = (CC1200_OSC_FREQ / twoToThe22) * (256.0f + static_cast<float>(actualMantissa)) * pow(2.0f, static_cast<float>(actualExponent));
	}
	// sanity check: calculate actual deviation
	debugStream->printf("This yields an actual deviation of +-%.00f Hz\n", actualDeviation);
#endif
}

void CC1200::setSymbolRate(float symbolRateHz)
{
	// Datasheet says that the cc1200 works in ksps, but testing with SmartRF studio
	// shows that it actually is sps.

	symbolRateSps = symbolRateHz;

	// Note: these equations are given on page 29 of the user guide

	float exactExponent = std::log2(symbolRateSps) - CC1200_OSC_FREQ_LOG2 + 19;
	// note: 19 comes from log2(2^39) - 20
	uint8_t actualExponent = static_cast<uint8_t>(std::min(static_cast<float>(maxValue4Bits), exactExponent));

	float exactMantissa;
	if(actualExponent >= 1)
	{
		exactMantissa = (std::pow(2.0f, static_cast<float>(39 - actualExponent)) * symbolRateSps)
		  / CC1200_OSC_FREQ - twoToThe20;
	}
	else
	{
		// use alternate high-resolution formula for case where exponent = 0
		exactMantissa = symbolRateSps * twoToThe38 / CC1200_OSC_FREQ;
	}

	// mantissa is a 20 bit number, so restrict it to that domain
	uint32_t actualMantissa = static_cast<uint32_t>(std::min(static_cast<float>(maxValue20Bits), exactMantissa));

	// program symbol rate registers
	std::array<uint8_t, 3> symbolRateRegisters ={
			static_cast<uint8_t>((actualExponent << SYMBOL_RATE2_SRATE_E) | (static_cast<uint8_t>((actualMantissa >> 16) & 0xFF) << SYMBOL_RATE2_SRATE_M_19_16)),
			static_cast<uint8_t>((actualMantissa >> 8) & 0xFF),
			static_cast<uint8_t>((actualMantissa & 0xFF))
	};
	writeRegisters(Register::SYMBOL_RATE2, symbolRateRegisters);

	// Calculate upsampler value according to the formula in its register description
	float upsamplingFactor = CC1200_OSC_FREQ / (64 * symbolRateSps); // 2^upsamplerPVal needs to be less than this value
	uint8_t upsamplerPVal = std::floor(std::log2(upsamplingFactor));

	// prevent low sampling rates from choosing a nonexistent upsampling
	upsamplerPVal = std::min<uint8_t>(upsamplerPVal, 0b110);

	uint8_t mdmcfg2 = readRegister(ExtRegister::MDMCFG2);
	mdmcfg2 &= ~(0b111 << MDMCFG2_UPSAMPLER_P);
	mdmcfg2 |= (upsamplerPVal << MDMCFG2_UPSAMPLER_P);

	writeRegister(ExtRegister::MDMCFG2, mdmcfg2);

#if CC1200_DEBUG
	debugStream->printf("Setting symbol rate, requested %.03f Hz, setting SRATE_E = 0x%" PRIx8 " SRATE_M = 0x%" PRIx32 "\n",
						symbolRateHz, actualExponent, actualMantissa);

	// sanity check: calculate actual symbol rate
	float actualSymbolRateSps;
	if(actualExponent == 0)
	{
		actualSymbolRateSps = (static_cast<float>(actualMantissa) * CC1200_OSC_FREQ) / twoToThe38;
	}
	else
	{

		actualSymbolRateSps = ((static_cast<float>(actualMantissa) + twoToThe20) *
							   pow(2.0f,static_cast<float>(actualExponent)) * CC1200_OSC_FREQ)
							  / twoToThe39;
	}

	debugStream->printf("This yields an actual symbol rate of %.02f Hz\n", actualSymbolRateSps);

	uint8_t actualUpsampling = static_cast<uint8_t>(pow(2.0f, static_cast<float>(upsamplerPVal)));
	debugStream->printf("Also setting upsampling factor to %" PRIu8 " via UPSAMPLER_P = %" PRIx8 "\n", actualUpsampling, upsamplerPVal);
#endif
}

// helper function for power setting
inline uint8_t dBPowerToRegValue(float powerDB)
{
	const float minOutputPower = -16.0f;

	// note: datasheet says this is 14.5, but that must be a mistake: 14.5 would produce a number
	// too large to fit into 6 bits.
	const float maxOutputPower = 14.0f;

	// clamp output power into correct range
	powerDB = std::min(powerDB, maxOutputPower);
	powerDB = std::max(powerDB, minOutputPower);

	// this equation derived from user guide section 7.1
	float exactPowerRamp = (2 * powerDB) + 35;
	return static_cast<uint8_t>(exactPowerRamp); // round to nearest
}

void CC1200::setOutputPower(float outPower)
{
	uint8_t actualPowerRamp = dBPowerToRegValue(outPower);

	uint8_t paCfg1 = readRegister(Register::PA_CFG1);
	paCfg1 &= ~(0b111111 << PA_CFG1_PA_POWER_RAMP);
	paCfg1 |= actualPowerRamp << PA_CFG1_PA_POWER_RAMP;
	writeRegister(Register::PA_CFG1, paCfg1);

#if CC1200_DEBUG
	debugStream->printf("Output power set to %.01f dBm\n", outPower);
#endif
}

const float CC1200::ASK_MIN_POWER_OFF = -17.5f;

void CC1200::setASKPowers(float maxPower, float minPower)
{
	uint8_t maxPowerValue = dBPowerToRegValue(maxPower);

	minPower = std::min(minPower, maxPower);
	minPower = std::max(minPower, -17.5f);

	// calculate min power using formula derived from manual
	uint8_t minPowerValue = static_cast<uint8_t>((maxPower - minPower) * 2);

	// write registers
	uint8_t paCfg1 = readRegister(Register::PA_CFG1);
	paCfg1 &= ~(0b111111 << PA_CFG1_PA_POWER_RAMP);
	paCfg1 |= maxPowerValue << PA_CFG1_PA_POWER_RAMP;
	writeRegister(Register::PA_CFG1, paCfg1);

	uint8_t askCfg = readRegister(Register::ASK_CFG);
	askCfg &= ~(0b111111 << ASK_CFG_ASK_DEPTH);
	askCfg |= minPowerValue << ASK_CFG_ASK_DEPTH;
	writeRegister(Register::ASK_CFG, askCfg);
}

void CC1200::setRadioFrequency(CC1200::Band band, float frequencyHz)
{
	// Frequency synthesizer configuration.  This is completely opaque and it is unknown what these bits do --
	// they are only generated by SmartRF studio.
	// I manually deduplicated them since only a few change based on frequency.
	writeRegister(ExtRegister::IF_ADC1,0xEE);
	writeRegister(ExtRegister::IF_ADC0,0x10);
	writeRegister(ExtRegister::FS_DIG1,0x04);
	writeRegister(ExtRegister::FS_CAL1,0x40);
	writeRegister(ExtRegister::FS_CAL0,0x0E);
	writeRegister(ExtRegister::FS_DIVTWO,0x03);
	writeRegister(ExtRegister::FS_DSM0,0x33);
	writeRegister(ExtRegister::FS_DVC1,0xF7);
	writeRegister(ExtRegister::FS_PFD,0x00);
	writeRegister(ExtRegister::FS_PRE,0x6E);
	writeRegister(ExtRegister::FS_REG_DIV_CML,0x1C);
	writeRegister(ExtRegister::FS_SPARE,0xAC);
	writeRegister(ExtRegister::FS_VCO0,0xB5);
	writeRegister(ExtRegister::XOSC5,0x0E);
	writeRegister(ExtRegister::XOSC1,0x03);
	if(band == Band::BAND_820_960MHz)
	{
		writeRegister(ExtRegister::FS_DIG0,0x55);
		writeRegister(ExtRegister::FS_DVC0,0x17);
		writeRegister(ExtRegister::IFAMP,0x09);
	}
	else if(band == Band::BAND_410_480MHz)
	{
		writeRegister(ExtRegister::FS_DIG0,0xA3);
		writeRegister(ExtRegister::FS_DVC0,0x0F);
		writeRegister(ExtRegister::IFAMP,0x0D);
	}
	else if(band == Band::BAND_164_192MHz)
	{
		writeRegister(ExtRegister::FS_DIG0,0x50);
		writeRegister(ExtRegister::FS_DVC0,0x0F);
		writeRegister(ExtRegister::IFAMP,0x0D);
	}
	else
	{
		// TI doesn't make settings public for the other radio bands.
		// Let's take a guess and use the 164-192MHz values.
		writeRegister(ExtRegister::FS_DIG0,0x50);
		writeRegister(ExtRegister::FS_DVC0,0x0F);
		writeRegister(ExtRegister::IFAMP,0x0D);
	}

	// convert band to LO Divider value.
	// Most of the bands just multiply the register value by 2, but nooo, not BAND_136_160MHz.
	uint8_t loDividerValue;
	if(band == Band::BAND_136_160MHz)
	{
		loDividerValue = 24;
	}
	else
	{
		loDividerValue = static_cast<uint8_t>(band) * 2;
	}

	// program band (also enable FS out of lock detector, which is useful for testing)
	writeRegister(Register::FS_CFG, (1 << FS_CFG_FS_LOCK_EN) | (static_cast<uint8_t>(band) << FS_CFG_FSD_BANDSELECT));

	// equation derived from user guide section 9.12
	float exactFreqRegValue = (twoToThe16 * frequencyHz * static_cast<float>(loDividerValue)) / CC1200_OSC_FREQ;
	uint32_t actualFreqRegValue = static_cast<uint32_t>(std::min(static_cast<float>(maxValue24Bits), exactFreqRegValue));

	// program frequency registers
	std::array<uint8_t, 3> freqRegisters ={
			static_cast<uint8_t>((actualFreqRegValue >> 16) & 0xFF),
			static_cast<uint8_t>((actualFreqRegValue >> 8) & 0xFF),
			static_cast<uint8_t>((actualFreqRegValue & 0xFF))
	};
	writeRegisters(ExtRegister::FREQ2, freqRegisters);

	// sanity check: calculate actual frequency
	radioFreqHz = (static_cast<float>(actualFreqRegValue) * CC1200_OSC_FREQ) / (twoToThe16 * static_cast<float>(loDividerValue));

#if CC1200_DEBUG
	debugStream->printf("Setting radio frequency, requested %.00f Hz, setting FREQ = 0x%" PRIx32 "\n",
						frequencyHz, actualFreqRegValue);
	debugStream->printf("This yields an actual frequency of %.00f Hz\n", radioFreqHz);
#endif
}

// helper function for setRXFilterBandwidth:
// calculate actual receive bandwidth from the given decimations.
float calcReceiveBandwidth(uint8_t adcDecimation, uint8_t cicDecimation)
{
	return CC1200_OSC_FREQ / (static_cast<float>(adcDecimation) * static_cast<float>(cicDecimation) * 2);
}

void CC1200::setRXFilterBandwidth(float bandwidthHz, bool preferHigherCICDec)
{
	// settings that the chip supports
	const uint8_t possibleADCDecimations[] = {12, 24, 48}; // indexes in this array represent the register value
	const size_t numADCDecimations = sizeof(possibleADCDecimations) / sizeof(uint8_t);
	const uint8_t minBBDecimation = 1;

	// maximum supported BB decimation based on ADC decimation varies based on chip
	const uint8_t maxBBDecimations1201[] = {33, 16, 8};
	const uint8_t maxBBDecimations1200[] = {44, 44, 44};
	uint8_t const * maxBBDecimations = isCC1201 ? maxBBDecimations1201 : maxBBDecimations1200;

	// the datasheet suggests to use the highest possible ADC decimation factor that will work for the requested frequency.
	// So, we compute the closest we can get with each ADC decimation, and if there's a tie, we choose the one with the higher ADC bandwidth

	uint8_t actualBBDecimations[numADCDecimations];

	for(size_t adcDecimationIndex = 0; adcDecimationIndex < numADCDecimations; ++adcDecimationIndex)
	{
		uint8_t adcDecimation = possibleADCDecimations[adcDecimationIndex];

		// calculate BB decimation closest to the requested frequency
		// derived from formula in section 6.1
		float exactBBDecimation = CC1200_OSC_FREQ / (2 * bandwidthHz * static_cast<float>(adcDecimation));
		exactBBDecimation = std::max(exactBBDecimation, static_cast<float>(minBBDecimation));
		exactBBDecimation = std::min(exactBBDecimation, static_cast<float>(maxBBDecimations[adcDecimationIndex]));

		actualBBDecimations[adcDecimationIndex] = static_cast<uint8_t>(exactBBDecimation);
	}

	// now, choose the best of the ones we calculated
	uint8_t bestDecimationIndex = 0;
	float bestDecimationError = std::abs(bandwidthHz - calcReceiveBandwidth(possibleADCDecimations[0], actualBBDecimations[0]));

	for(size_t adcDecimationIndex = 1; adcDecimationIndex < numADCDecimations; ++adcDecimationIndex)
	{
		float thisDecimationError = std::abs(bandwidthHz -
				calcReceiveBandwidth(possibleADCDecimations[adcDecimationIndex], actualBBDecimations[adcDecimationIndex]));
		if((preferHigherCICDec && thisDecimationError <= bestDecimationError) || (!preferHigherCICDec && thisDecimationError < bestDecimationError))
		{
			bestDecimationError = thisDecimationError;
			bestDecimationIndex = adcDecimationIndex;
		}
	}

	// now use the best value!
	uint8_t chanBwValue = (bestDecimationIndex << CHAN_BW_ADC_CIC_DECFACT) | (actualBBDecimations[bestDecimationIndex] << CHAN_BW_BB_CIC_DECFACT);
	writeRegister(Register::CHAN_BW, chanBwValue);

	// also set DVGA_GAIN, which depends on bandwidth
	uint8_t mdmCfg1Value = readRegister(Register::MDMCFG1);

	mdmCfg1Value &= ~(0b11 << MDMCFG1_DVGA_GAIN);
	if(bandwidthHz >= 100000)
	{
		mdmCfg1Value |= 1 << MDMCFG1_DVGA_GAIN;
	}

	writeRegister(Register::MDMCFG1, mdmCfg1Value);

	// also set MDMCFG0.DATA_FILTER_EN, which should be 0b11 iff bandwidth / symbol rate > 10
	uint8_t mdmCfg0Val = readRegister(Register::MDMCFG0);

	if(bandwidthHz / symbolRateSps > 10.0f)
	{
		mdmCfg0Val |= 0b11 << MDMCFG0_DATA_FILTER_EN;
	}
	else
	{
		mdmCfg0Val &= ~(0b11 << MDMCFG0_DATA_FILTER_EN);
	}

	writeRegister(Register::MDMCFG0, mdmCfg0Val);

	// finally, we need to set RX_CONFIG_LIMITATION.  It's not exactly clear what this does, but its setting changes
	// based on the filter BW.
	uint8_t syncCfg0Value = readRegister(Register::SYNC_CFG0);

	if(symbolRateSps < bandwidthHz / 2 || bandwidthHz > 1500000)
	{
		// clear RX_CONFIG_LIMITATION
		syncCfg0Value &= ~(1 << SYNC_CFG0_RX_CONFIG_LIMITATION);
	}
	else
	{
		// set RX_CONFIG_LIMITATION
		syncCfg0Value |= (1 << SYNC_CFG0_RX_CONFIG_LIMITATION);
	}

	writeRegister(Register::SYNC_CFG0, syncCfg0Value);

	adcCicDecimation = possibleADCDecimations[bestDecimationIndex];
	currentRXFilterBW = calcReceiveBandwidth(possibleADCDecimations[bestDecimationIndex], actualBBDecimations[bestDecimationIndex]);

#if CC1200_DEBUG
	debugStream->printf("Setting BB decimation to %" PRIu8 " and ADC decimation to %" PRIu8 "\n",
			actualBBDecimations[bestDecimationIndex], possibleADCDecimations[bestDecimationIndex]);
	debugStream->printf("This yields an actual RX filter BW of %.00f\n", currentRXFilterBW);
#endif
}

void CC1200::configureDCFilter(bool enableAutoFilter, uint8_t settlingCfg, uint8_t cutoffCfg)
{
	uint8_t dcfiltCfg = 0;

	if(!enableAutoFilter)
	{
		// set "freeze coeff" bit
		dcfiltCfg |= (1 << DCFILT_CFG_DCFILT_FREEZE_COEFF);
	}

	dcfiltCfg |= settlingCfg << DCFILT_CFG_DCFILT_BW_SETTLE;
	dcfiltCfg |= cutoffCfg << DCFILT_CFG_DCFILT_BW;

	writeRegister(Register::DCFILT_CFG, dcfiltCfg);
}

void CC1200::configureSyncWord(uint32_t syncWord, CC1200::SyncMode mode, uint8_t syncThreshold)
{
	// program sync word registers
	std::array<uint8_t, 4> syncWordRegisters ={
			static_cast<uint8_t>((syncWord >> 24) & 0xFF),
			static_cast<uint8_t>((syncWord >> 16) & 0xFF),
			static_cast<uint8_t>((syncWord >> 8) & 0xFF),
			static_cast<uint8_t>(syncWord & 0xFF)
	};
	writeRegisters(Register::SYNC3, syncWordRegisters);

	// program sync word cfg
	writeRegister(Register::SYNC_CFG1, (static_cast<uint8_t>(mode) << SYNC_CFG1_SYNC_MODE) | (syncThreshold << SYNC_CFG1_SYNC_THR));
}

bool CC1200::isFSLocked()
{
	return readRegister(ExtRegister::FSCAL_CTRL) & (1 << FSCAL_CTRL_LOCK);
}

void CC1200::configurePreamble(uint8_t preambleLengthCfg, uint8_t preambleFormatCfg)
{
	uint8_t preambleCfg1 = 0;
	preambleCfg1 |= preambleLengthCfg << PREAMBLE_CFG1_NUM_PREAMBLE;
	preambleCfg1 |= preambleFormatCfg << PREAMBLE_CFG1_PREAMBLE_WORD;
	writeRegister(Register::PREAMBLE_CFG1, preambleCfg1);

#if CC1200_DEBUG
	debugStream->printf("Preamble length CFG set to 0x%" PRIx8 "\n", preambleLengthCfg);
#endif
}

void CC1200::setPARampRate(uint8_t firstRampLevel, uint8_t secondRampLevel, CC1200::RampTime rampTime)
{
	// enable PA ramping
	uint8_t paCfg1Val = readRegister(Register::PA_CFG1);
	paCfg1Val |= 1 << PA_CFG1_PA_RAMP_SHAPE_EN;
	writeRegister(Register::PA_CFG1, paCfg1Val);

	// configure properties
	uint8_t paCfg0Val = 0;
	paCfg0Val |= (firstRampLevel << PA_CFG0_FIRST_IPL);
	paCfg0Val |= (secondRampLevel << PA_CFG0_SECOND_IPL);
	paCfg0Val |= (static_cast<uint8_t>(rampTime) << PA_CFG0_RAMP_SHAPE);

	writeRegister(Register::PA_CFG0, paCfg0Val);
}

void CC1200::disablePARamping()
{
	uint8_t paCfg1Val = readRegister(Register::PA_CFG1);
	paCfg1Val &= ~(1 << PA_CFG1_PA_RAMP_SHAPE_EN);
	writeRegister(Register::PA_CFG1, paCfg1Val);
}

void CC1200::setAGCReferenceLevel(uint8_t level)
{
	writeRegister(Register::AGC_REF, level);
}

void CC1200::setAGCSyncBehavior(CC1200::SyncBehavior behavior)
{
	uint8_t agcCfg3Val = readRegister(Register::AGC_CFG3);
	agcCfg3Val &= ~(0b111 << AGC_CFG3_AGC_SYNC_BEHAVIOUR);
	agcCfg3Val |= static_cast<uint8_t>(behavior) << AGC_CFG3_AGC_SYNC_BEHAVIOUR;
	writeRegister(Register::AGC_CFG3, agcCfg3Val);
}

void CC1200::setAGCGainTable(CC1200::GainTable table, uint8_t minGainIndex, uint8_t maxGainIndex)
{
	uint8_t agcCfg3Val = readRegister(Register::AGC_CFG3);
	uint8_t agcCfg2Val = readRegister(Register::AGC_CFG2);

	agcCfg3Val &= ~(0b11111 << AGC_CFG3_AGC_MIN_GAIN);
	agcCfg2Val &= ~(0b11 << AGC_CFG2_FE_PERFORMANCE_MODE);
	agcCfg2Val &= ~(0b11111 << AGC_CFG2_AGC_MAX_GAIN);

	agcCfg3Val |= minGainIndex << AGC_CFG3_AGC_MIN_GAIN;
	agcCfg2Val |= static_cast<uint8_t>(table) << AGC_CFG2_FE_PERFORMANCE_MODE;
	agcCfg2Val |= maxGainIndex << AGC_CFG2_AGC_MAX_GAIN;

	writeRegister(Register::AGC_CFG3, agcCfg3Val);
	writeRegister(Register::AGC_CFG2, agcCfg2Val);
}

void CC1200::setAGCHysteresis(uint8_t hysteresisCfg)
{
	uint8_t agcCfg0Val = readRegister(Register::AGC_CFG0);
	agcCfg0Val &= ~(0b11 << AGC_CFG0_AGC_HYST_LEVEL);
	agcCfg0Val |= hysteresisCfg << AGC_CFG0_AGC_HYST_LEVEL;
	writeRegister(Register::AGC_CFG0, agcCfg0Val);
}

void CC1200::setAGCSlewRate(uint8_t slewrateCfg)
{
	uint8_t agcCfg0Val = readRegister(Register::AGC_CFG0);
	agcCfg0Val &= ~(0b11 << AGC_CFG0_AGC_SLEWRATE_LIMIT);
	agcCfg0Val |= slewrateCfg << AGC_CFG0_AGC_SLEWRATE_LIMIT;
	writeRegister(Register::AGC_CFG0, agcCfg0Val);
}

void CC1200::setAGCSettleWait(uint8_t settleWaitCfg)
{
	uint8_t agcCfg1Val = readRegister(Register::AGC_CFG1);
	agcCfg1Val &= ~(0b111 << AGC_CFG1_AGC_SETTLE_WAIT);
	agcCfg1Val |= (settleWaitCfg << AGC_CFG1_AGC_SETTLE_WAIT);
	writeRegister(Register::AGC_CFG1, agcCfg1Val);
}

float CC1200::getRSSIRegister()
{
	uint8_t rssi1Val = readRegister(ExtRegister::RSSI1);
	uint8_t rssi0Val = readRegister(ExtRegister::RSSI0);

	if(!(rssi0Val & (1 << RSSI0_RSSI_VALID)))
	{
		// no valid measurement
		return NAN;
	}

	// first convert to two's compliment number
	int16_t rssiInt = 0;
	rssiInt |= (rssi0Val >> RSSI0_RSSI_3_0) & 0b1111;
	rssiInt |= rssi1Val << 4;

	if(rssi1Val & 0x80)
	{
		// negative number, sign extend from 12 to 16 bits
		rssiInt |= (0b1111 << 12);
	}

	//debugStream->printf("Approx RSSI: %" PRIi8 ", exact RSSI: %f\n", static_cast<int8_t>(rssi1Val), static_cast<float>(rssiInt) * 0.0625f);

	return static_cast<float>(rssiInt) * 0.0625f; // conversion factor given in datasheet
}

void CC1200::setRSSIOffset(int8_t adjust)
{
	writeRegister(Register::AGC_GAIN_ADJUST, static_cast<uint8_t>(adjust));
}

uint8_t CC1200::getLQIRegister()
{
	return readRegister(ExtRegister::LQI_VAL) & 0b1111111;
}

void CC1200::setIFCfg(IFCfg value, bool enableIQIC)
{
	// make sure prerequisites have been run
	MBED_ASSERT(radioFreqHz > 0);
	MBED_ASSERT(adcCicDecimation > 0 && currentRXFilterBW > 0);

	uint8_t ifMixCfg = readRegister(ExtRegister::IF_MIX_CFG);
	ifMixCfg &= ~(0b111 << IF_MIX_CFG_CMIX_CFG);
	ifMixCfg |= (static_cast<uint8_t>(value) << IF_MIX_CFG_CMIX_CFG);
	writeRegister(ExtRegister::IF_MIX_CFG, ifMixCfg);

	float effectiveIF;

	// calculate effective IF value
	if(value == IFCfg::ZERO)
	{
		effectiveIF = radioFreqHz;
	}
	else
	{
		int32_t dividerValue = 0;
		switch(value)
		{
			case IFCfg::NEGATIVE_DIV_4: dividerValue = -4; break;
			case IFCfg::NEGATIVE_DIV_6: dividerValue = -6; break;
			case IFCfg::NEGATIVE_DIV_8: dividerValue = -8; break;
			case IFCfg::POSITIVE_DIV_4: dividerValue = 4; break;
			case IFCfg::POSITIVE_DIV_6: dividerValue = 6; break;
			case IFCfg::POSITIVE_DIV_8: dividerValue = 8; break;
			default: break;
		}

		// formula from IF_MIX_CFG register description
		effectiveIF = (CC1200_OSC_FREQ / static_cast<float>(adcCicDecimation * dividerValue)) * 1000;
	}

	uint8_t iqicValue = readRegister(Register::IQIC);
	if(enableIQIC && effectiveIF > currentRXFilterBW)
	{
		iqicValue |= (1 << IQIC_IQIC_EN);
	}
	else
	{
		iqicValue &= ~(1 << IQIC_IQIC_EN);
	}
	writeRegister(Register::IQIC, iqicValue);

#if CC1200_DEBUG
	debugStream->printf("Setting IF Mix Cfg to 0x%" PRIx8 " and IQIC_EN to %d\n", static_cast<uint8_t>(value),
					 !!(iqicValue & (1 << IQIC_IQIC_EN))); // note: double ! used to convert boolean to either 1 or 0.
	debugStream->printf("This yields an actual IF of %.00f\n", effectiveIF);
#endif
}

uint8_t CC1200::readRegister(CC1200::Register reg)
{
	spi.select();
	loadStatusByte(spi.write(CC1200_READ | static_cast<uint8_t>(reg)));
	uint8_t regValue = spi.write(0);
	spi.deselect();

#if CC1200_REGISTER_LEVEL_DEBUG
	debugStream->printf("Read register 0x%" PRIx8 " -> 0x%" PRIx8 "\n", static_cast<uint8_t>(reg), regValue);
#endif

	return regValue;
}

void CC1200::writeRegister(Register reg, uint8_t value)
{
	spi.select();
	loadStatusByte(spi.write(CC1200_WRITE | static_cast<uint8_t>(reg)));
	spi.write(value);
	spi.deselect();

#if CC1200_REGISTER_LEVEL_DEBUG
	debugStream->printf("Wrote register 0x%" PRIx8 " <- 0x%" PRIx8 "\n", static_cast<uint8_t>(reg), value);
#endif
}

void CC1200::writeRegisters(CC1200::Register startReg, uint8_t const *values, size_t numRegisters)
{
	spi.select();
	loadStatusByte(spi.write(CC1200_WRITE | CC1200_BURST | static_cast<uint8_t>(startReg)));

	for(size_t byteIndex = 0; byteIndex < numRegisters; ++byteIndex)
	{
		spi.write(values[byteIndex]);
	}
	spi.deselect();

#if CC1200_REGISTER_LEVEL_DEBUG
	for(size_t byteIndex = 0; byteIndex < numRegisters; ++byteIndex)
	{
		debugStream->printf("Wrote register 0x%" PRIx8 " <- 0x%" PRIx8 "\n",
				static_cast<uint8_t>(static_cast<uint8_t>(startReg) + byteIndex),
				values[byteIndex]);
	}
#endif
}

void CC1200::loadStatusByte(uint8_t status)
{
	chipReady = !(status >> 7);
	state = static_cast<State>((status >> 4) & 0x7);

#if CC1200_REGISTER_LEVEL_DEBUG
	debugStream->printf("Updated status, state = 0x%" PRIx8 " ready = %s\n", static_cast<uint8_t>(state), chipReady ? "true" : "false");
#endif
}

uint8_t CC1200::readRegister(CC1200::ExtRegister reg)
{
	spi.select();
	loadStatusByte(spi.write(CC1200_READ | CC1200_EXT_ADDR));
	spi.write(static_cast<uint8_t>(reg));
	uint8_t regValue = spi.write(0);
	spi.deselect();

#if CC1200_REGISTER_LEVEL_DEBUG
	debugStream->printf("Read ext register 0x%" PRIx8 " -> 0x%" PRIx8 "\n", static_cast<uint8_t>(reg), regValue);
#endif

	return regValue;
}

void CC1200::writeRegister(CC1200::ExtRegister reg, uint8_t value)
{
	spi.select();
	loadStatusByte(spi.write(CC1200_WRITE | CC1200_EXT_ADDR));
	spi.write(static_cast<uint8_t>(reg));
	spi.write(value);
	spi.deselect();

#if CC1200_REGISTER_LEVEL_DEBUG
	debugStream->printf("Wrote ext register 0x%" PRIx8 " <- 0x%" PRIx8 "\n", static_cast<uint8_t>(reg), value);
#endif
}

void CC1200::writeRegisters(CC1200::ExtRegister startReg, uint8_t const *values, size_t numRegisters)
{
	spi.select();
	loadStatusByte(spi.write(CC1200_WRITE | CC1200_BURST | CC1200_EXT_ADDR));
	spi.write(static_cast<uint8_t>(startReg));

	for(size_t byteIndex = 0; byteIndex < numRegisters; ++byteIndex)
	{
		spi.write(values[byteIndex]);
	}
	spi.deselect();

#if CC1200_REGISTER_LEVEL_DEBUG
	for(size_t byteIndex = 0; byteIndex < numRegisters; ++byteIndex)
	{
		debugStream->printf("Wrote extended register 0x%" PRIx8 " <- 0x%" PRIx8 "\n",
							static_cast<uint8_t>(static_cast<uint8_t>(startReg) + byteIndex),
							values[byteIndex]);
	}
#endif
}

void CC1200::sendCommand(CC1200::Command command)
{
	spi.select();
	loadStatusByte(spi.write(static_cast<uint8_t>(command)));
	spi.deselect();

#if CC1200_REGISTER_LEVEL_DEBUG
	debugStream->printf("Sent SPI command 0x%" PRIx8 "\n", static_cast<uint8_t>(command));
#endif
}

uint8_t CC1200::readRXFIFOByte(uint8_t address)
{
	spi.select();
	loadStatusByte(spi.write(CC1200_READ | CC1200_MEM_ACCESS));
	spi.write(CC1200_RX_FIFO | address);
	int value = spi.write(0);
	spi.deselect();

#if CC1200_REGISTER_LEVEL_DEBUG
	debugStream->printf("Read RX FIFO[0x%" PRIx8 "]: 0x%x 0x%x -> 0x%" PRIx8 "\n", static_cast<uint8_t>(address), CC1200_READ | CC1200_MEM_ACCESS, CC1200_RX_FIFO | address, value);
#endif

	return value;
}
