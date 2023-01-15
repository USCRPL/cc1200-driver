//
// Created by jamie on 3/27/2020.
//

#ifndef LIGHTSPEEDRANGEFINDER_CC1200_H
#define LIGHTSPEEDRANGEFINDER_CC1200_H

#include <mbed.h>
#include <Stream.h>

#include <cstdint>

/**
 *  Base driver for the CC1200 radio communications IC.
 *  This class provides basic functions and register level IO with the chip.
 */
class CC1200
{
	// connections to chip
	SPI spi;
	DigitalOut rst;

	// Output to print debug messages to
	Stream * debugStream;

public:

	// register definitions
	enum class Register : uint8_t
	{
		IOCFG3 = 0x00,
		IOCFG2 = 0x01,
		IOCFG1 = 0x02,
		IOCFG0 = 0x03,
		SYNC3 = 0x4,
		SYNC2 = 0x5,
		SYNC1 = 0x6,
		SYNC0 = 0x7,
		SYNC_CFG1 = 0x8,
		SYNC_CFG0 = 0x9,
		DEVIATION_M = 0xA,
		MODCFG_DEV_E = 0xB,
		DCFILT_CFG  = 0xC,
		PREAMBLE_CFG1 = 0xD,
		PREAMBLE_CFG0 = 0xE,
		IQIC = 0xF,
		CHAN_BW = 0x10,
		MDMCFG1 = 0x11,
		MDMCFG0 = 0x12,
		SYMBOL_RATE2 = 0x13,
		SYMBOL_RATE1 = 0x14,
		SYMBOL_RATE0 = 0x15,
		AGC_REF = 0x16,
		AGC_CS_THR = 0x17,
		AGC_GAIN_ADJUST = 0x18,
		AGC_CFG3 = 0x19,
		AGC_CFG2 = 0x1A,
		AGC_CFG1 = 0x1B,
		AGC_CFG0 = 0x1C,
		FIFO_CFG = 0x1D,
		DEV_ADDR = 0x1E,
		SETTLING_CFG = 0x1F,
		FS_CFG = 0x20,
		WOR_CFG1 = 0x21,
		WOR_CFG0 = 0x22,
		WOR_EVENT0_MSB = 0x23,
		WOR_EVENT0_LSB = 0x24,
		RXDCM_TIME = 0x25,
		PKT_CFG2 = 0x26,
		PKT_CFG1 = 0x27,
		PKT_CFG0 = 0x28,
		RFEND_CFG1 = 0x29,
		RFEND_CFG0 = 0x2A,
		PA_CFG1 = 0x2B,
		PA_CFG0 = 0x2C,
		ASK_CFG = 0x2D,
		PKT_LEN = 0x2E
	};

	// extended register definitions
	enum class ExtRegister : uint8_t
	{
		IF_MIX_CFG = 0x0,
		FREQOFF_CFG = 0x1,
		TOC_CFG = 0x2,
		//...
		MDMCFG2 = 0x5,
		//...
		FREQOFF1 = 0xA,
		FREQOFF2 = 0xB,
		FREQ2 = 0xC,
		FREQ1 = 0xD,
		FREQ0 = 0xE,
		IF_ADC2 = 0xF,
		IF_ADC1 = 0x10,
		IF_ADC0 = 0x11,
		FS_DIG1 = 0x12,
		FS_DIG0 = 0x13,
		//...
		FS_CAL1 = 0x16,
		FS_CAL0 = 0x17,
		FS_CHP = 0x18,
		FS_DIVTWO = 0x19,
		FS_DSM1 = 0x1A,
		FS_DSM0 = 0x1B,
		FS_DVC1 = 0x1C,
		FS_DVC0 = 0x1D,
		FS_LBI = 0x1E,
		FS_PFD = 0x1F,
		FS_PRE = 0x20,
		FS_REG_DIV_CML = 0x21,
		FS_SPARE = 0x22,
		FS_VCO4 = 0x23,
		FS_VCO3 = 0x24,
		FS_VCO2 = 0x25,
		FS_VCO1 = 0x26,
		FS_VCO0 = 0x27,
		//...
		IFAMP = 0x2F,
		//..
		XOSC5 = 0x32,
		XOSC4 = 0x33,
		XOSC3 = 0x34,
		XOSC2 = 0x35,
		XOSC1 = 0x36,
		XOSC0 = 0x37,
		//...
		RSSI1 = 0x71,
		RSSI0 = 0x72,
		MARCSTATE = 0x73,
		LQI_VAL = 0x74,
		//...
		FREQOFF_EST1 = 0x77,
		FREQOFF_EST2 = 0x78,
		//...
		FSCAL_CTRL = 0x8D,
		PARTNUMBER = 0x8F,
		PARTVERSION = 0x90,
		//...
		RXFIRST = 0xD2,
		TXFIRST = 0xD3,
		RXLAST = 0xD4,
		TXLAST = 0xD5,
		NUM_TXBYTES = 0xD6,
		NUM_RXBYTES = 0xD7,
		//...
		RXFIFO_PRE_BUF = 0xDA
	};

	// Command strobe definitions.  See user guide section 3.2.2
	enum class Command : uint8_t
	{
		SOFT_RESET = 0x30,
		FAST_TX_ON = 0x31,
		OSC_OFF = 0x32,
		CAL_FREQ_SYNTH = 0x33,
		RX = 0x34,
		TX = 0x35,
		IDLE = 0x36,
		AUTO_FREQ_COMP = 0x37,
		WAKE_ON_RADIO = 0x38,
		SLEEP = 0x39,
		FLUSH_RX = 0x3A,
		FLUSH_TX = 0x3B,
		WOR_RESET = 0x3C,
		NOP = 0x3D
	};

	// State of the radio chip.  See user guide Figure 2.
	enum class State : uint8_t
	{
		IDLE = 0x0,
		RX = 0x1,
		TX = 0x2,
		FAST_ON = 0x3,
		CALIBRATE = 0x4,
		SETTLING = 0x5,
		RX_FIFO_ERROR = 0x6,
		TX_FIFO_ERROR = 0x7
	};

private:
	// chip data variables
	bool chipReady = false;
	State state = State::IDLE;
	bool isCC1201;

	// current state variables

	// current symbol rate of the radio
	float symbolRateSps = 0;

	// current RX filter params
	uint8_t adcCicDecimation = 0;
	float currentRXFilterBW = 0;

	// current RF params
	float radioFreqHz;


public:

	/**
	 * Construct a CC1200 radio driver from the given set of pins.
	 *
	 * @param misoPin
	 * @param mosiPin
	 * @param sclkPin
	 * @param csPin
	 * @param rstPin
	 * @param _debugStream Stream to print error/debug information on.
	 * @param isCC1201 True if the chip is a CC1201, false if it is a CC1200.  The CC1201 is a cheaper option that lacks low bandwidth settings but is otherwise identical.
	 */
	CC1200(PinName mosiPin, PinName misoPin, PinName sclkPin, PinName csPin, PinName rstPin, Stream * _debugStream, bool _isCC1201 = false);

	/**
	 * Reset the chip and attempt to connect to it.
	 * Returns whether the chip could be contacted.
	 * @return
	 */
	bool begin();

	/**
	 * Get the radio's most recently known state.
	 * State is updated whenever registers are read or commands are sent, or when you call updateState.
	 * @return
	 */
	State getState() { return state; }

	// Data tx & rx functions
	// ------------------------------------------------------------------------------

	/**
	 * Get the number of bytes currently in the TX FIFO
	 * @return
	 */
	size_t getTXFIFOLen();

	/**
	 * Get the number of bytes currently in the RX FIFO
	 * @return
	 */
	size_t getRXFIFOLen();

	/**
	 * Enqueue a packet to be sent over the radio.  It will be sent the next time the radio is in
	 * transmit state.
	 *
	 * In variable length mode, the length of a packet is variable, from 1 byte to 127 bytes.
	 * The length will be transmitted along with the packet data.
	 *
	 * In fixed length mode, the length should be the fixed packet length.
	 *
	 * The function is not for use with infinite-length mode, use writeStream() instead.
	 *
	 * Also reads the radio's state.
	 *
	 * @param data
	 * @param len
	 *
	 * @return Whether the packet was enqueued.  Could return false if there was not enough FIFO
	 * space to enqueue the packet, or if the packet is too long.
	 */
	bool enqueuePacket(char const * data, size_t len);

	/**
	 * Check whether there is at least one complete packet in the RX FIFO.
	 * In infinite length mode, this returns true if any data is present at all.
	 * NOTE: An alternate way to do this using hardware is to configure one
	 * of the CC1200's GPIOs as PKT_SYNC_RXTX, then set a falling edge interrupt to receive a packet.
	 * @return
	 */
	bool hasReceivedPacket();

	/**
	 * Receive a packet from the radio.  Only packets that pass CRC check are received into the FIFO;
	 * those which do not pass checksum will be discarded.
	 *
	 * This function assumes that there is a packet in the buffer.  You should only call it after
	 * hasReceivedPacket() is true or a PKT_SYNC_RXTX pulse is received.  If there is not a packet
	 * in the FIFO, *undefined behavior* can occur.  An arbitrary amount of data will be read from
	 * the FIFO and garbage may be returned.
	 *
	 * The function is not for use with infinite-length mode, use readStream() instead.
	 *
	 * NOTE: A null terminator is NOT added unless it was present in the transmitted data.
	 * Be careful when treating the returned data as a string!
	 *
	 * @param buffer Buffer to store received bytes in.
	 * @param bufferLen Length of the buffer supplied.  If the packet is longer than this buffer, then
	 *  the full packet will be read from the FIFO but only a buffer's worth will be stored.
	 * @return Number of bytes actually received.
	 */
	size_t receivePacket(char * buffer, size_t bufferLen);

	// Infinite length tx & rx functions
	// ------------------------------------------------------------------------------

	/*
	 * How to use infinite length from the TX side:
	 * 1. Place up to 128 bytes of data in the TX FIFO for transmission using writeStream().
	 * 2. Enable TX mode using startTX()
	 * 3. Keep streaming in data using writeStream() writeStreamBlocking() or at least as fast as it is transmitted.
	 * 4. Disable TX mode when done using idle().
	 *
	 * Take care that the TX fifo never runs completely out of data, or the chip will go into TX FIFO ERROR state.
	 */

	/*
	 * How to use infinite length from the RX side:
	 * 1. Enable RX mode using startRX().  I'm pretty sure that if sync words are enabled, you must start RX before the transmitter starts transmitting.
	 * 2. Wait for data to arrive using hasReceivedPacket().
	 * 3. Start streaming out data using readStream() or readStreamBlocking().
	 * 4. Disable RX mode when done using idle().
	 *
	 * Take care that the RX fifo doesn't become full, or the chip will go into RX FIFO ERROR state.
	 */

	/**
	 * Write a stream of data to the chip.  This function is only for use in infinite-length mode.
	 * As many bytes from the given buffer will be written as can fit in the chip's FIFO.
	 * If the FIFO is full, this function does nothing.
	 *
	 * @param buffer
	 * @param count
	 * @return Number of bytes written.
	 */
	size_t writeStream(const char* buffer, size_t count);

	/**
	 * Write a stream of data to the chip.  This function is only for use in infinite-length mode.
	 * Will block until all bytes in the given buffer have been written to the TX FIFO, or an error has been
	 * detected (the radio switches to any state other than TX).
	 *
	 * @param buffer
	 * @param count
	 * @return True if successful, false if there was an error.
	 */
	bool writeStreamBlocking(const char* buffer, size_t count);

	/**
	 * Read up to maxLen bytes into buffer.
	 * @param buffer
	 * @param maxLen
	 * @return How many bytes were actually read.  0 if the RX FIFO was empty.
	 */
	size_t readStream(char* buffer, size_t maxLen);

	/**
	 * Read a stream of data from the chip.  This function is only for use in infinite-length mode.
	 * Will block until the buffer was filled, an error has been
	 * detected (the radio switches to any state other than RX), or the timeout expires.
	 *
	 * If false is returned, some data may have been written to the buffer, and the rest will not have been modified.
	 *
	 * Note: if using a zero timeout, this function could cause a hang if the transmitter stops transmitting.
	 *
	 * @param buffer
	 * @param count
	 * @param timeout Timeout, or 0us to disable timeout.
	 * @return True iff the buffer was completely filled.
	 */
	bool readStreamBlocking(char* buffer, size_t count, std::chrono::microseconds timeout=0us);

	// State transition configuration
	// ------------------------------------------------------------------------------

	/**
	 * Set what state the radio will enter when a packet is received.
	 * @param goodPacket State when a good (CRC pass) packet is received.
	 * Accepts State::TX, State::IDLE, State::FAST_TX_ON, and State::RX.
	 * @param badPacket State when a bad (CRC fail) packet is received.
	 * Accepts State::RX and State::IDLE
	 */
	void setOnReceiveState(State goodPacket, State badPacket);

	/**
	 * Set what state the radio will enter when a packet is sent.
	 * @param txState State when a packet is transmitted.
	 * Accepts State::TX, State::IDLE, State::FAST_TX_ON, and State::RX.
	 */
	void setOnTransmitState(State txState);

	enum class FSCalMode : uint8_t
	{
		NONE = 0b00, // never calibrate the FS automatically
		FROM_IDLE = 0b01, // calibrate the FS when going from idle to TX, RX, or fast TX on
		TO_IDLE = 0b10, // calibrate the FS when going from TX, RX, or fast TX on to idle
		TO_IDLE_1_4 = 0b11 // calibrate the FS 1/4 of the time when going from TX, RX, or fast TX on to idle
	};

	/**
	 * Set when the radio calibrates its frequency synthesizer.
	 *
	 * Per https://e2e.ti.com/support/wireless-connectivity/sub-1-ghz/f/156/t/375189
	 * it looks like the FS can drift with changes in supply voltage and/or temperature,
	 * so it is good to continually calibrate it in case these change.
	 */
	void setFSCalMode(FSCalMode mode);

	// GPIO configuration
	// ------------------------------------------------------------------------------

	/**
	 * Enum for all possible GPIO modes.
	 * Note: Some modes do different things depending on which GPIOs they're assigned to.
	 * Duplicate enum values have been intentionally defined for this.
	 */
	enum class GPIOMode : uint8_t
	{
		RXFIFO_THR_PKT = 1,
		PKT_SYNC_RXTX = 6,
		RSSI_UPDATE = 14, // GPIO3 and GPIO2
		AGC_HOLD = 14, // GPIO1
		AGC_UPDATE = 14, // GPIO0
		SYNC_EVENT = 41, // GPIO2
		HIGHZ = 48,
		HW0 = 51
	};

	/**
	 * Configure a CC1200 GPIO pin.
	 * @param gpioNumber Pin number, from 0-3.
	 * @param mode Mode to set the pin to.
	 * @param outputInvert Whether to invert the output of the pin.
	 */
	void configureGPIO(uint8_t gpioNumber, GPIOMode mode, bool outputInvert = false);

	// RF configuration
	// ------------------------------------------------------------------------------

	/**
	 * Set up the radio for FIFO mode.
	 */
	void configureFIFOMode();

	enum class PacketMode : uint8_t
	{
		/// Use infinite length transmissions (streaming mode).
		INFINITE_LENGTH = 0b10,
		/// Use fixed length packets.
		VARIABLE_LENGTH = 0b1,
		/// Use variable length packets, the length is encoded in the first byte of the packet.
		FIXED_LENGTH = 0b0
	};

	/**
	 * Set the packet mode that the system will use.
	 * @param mode
	 * @param appendStatus Have the radio append a status byte to each packet received.  This takes up
	 * 2 bytes per packet in the RX FIFO, but provides status information about each packet.
	 */
	void setPacketMode(PacketMode mode, bool appendStatus = false);

	/**
	 * Set the packet length when in fixed length packet mode.
	 * The bit length parameter can be used to send only the x most significant bits of the final byte.
	 * For example, if your packets are 20 bits long, you should set length to 2 bytes and bitLength to
	 * 4 bytes, so that 2 complete bytes + 4 extra bits are transmitted.  Buffers used for sending and receiving packets
	 * should then be 3 bytes long.
	 *
	 * When appendStatus is disabled, the max length is 256 bytes.  When it is enabled, the max length is 254 bytes.
	 * @param length
	 */
	void setPacketLength(uint16_t length, uint8_t bitLength = 0);

private:
	// current packet mode
	PacketMode _packetMode;

	// current packet length when in fixed length packet mode
	uint16_t _packetTotalLength = 0; // length in bytes including final partial byte
	uint16_t _packetByteLength = 0; // length in whole bytes
	uint8_t _packetBitLength = 0; // extra bit length at end

	// Whether the two status bytes are included in each received packet
	bool appendStatusEnabled = true;

public:

	/**
	 * Set whether the CRC check is enabled.  This driver enables it by default.
	 * Enabling CRC will cause a 16 bit checksum to be transmitted along with the packet.
	 * It will be automatically checked by the receiving CC1200, and the packet will be discarded if the CRC
	 * doesn't match.
	 *
	 * NOTE: it is not recommended to disable the CRC when using variable length mode.
	 * If the length byte is corrupted and the CRC doesn't check this, then the driver could read
	 * too little or too much from the chip's FIFO and cause the chip to enter the FIFO underflow state.
	 * @param enabled
	 */
	void setCRCEnabled(bool enabled);

	enum class ModFormat : uint8_t
	{
		FSK_2 = 0x0,
		GFSK_2 = 0x1,
		ASK = 0x3,
		FSK_4 = 0x4,
		GFSK_4 = 0x5
	};

	/**
	 * Set the modulation format of the radio.
	 * @param format
	 */
	void setModulationFormat(ModFormat format);

	/**
	 * Set the frequency deviation from the center frequency in Hz.
	 * See user guide section 5.2.1 for details, and cc1200 datasheet section 4.10.2 for example values.
	 */
	void setFSKDeviation(float deviation);

	/**
	 * Set the RF symbol rate in Hz.  If this radio is to be used in receive mode you must call
	 * setRXFilterBandwidth() after calling this function.
	 * @param symbolRateHz
	 */
	void setSymbolRate(float symbolRateHz);

	/**
	 * Set the approximate output power in dBm.
	 * Must be between -16dBm and +14dBm.
	 * @param outPower
	 */
	void setOutputPower(float outPower);

	// min power to use to turn the radio completely off
	const static float ASK_MIN_POWER_OFF;

	/**
	 * Set the high and low output powers when transmitting in ASK mode.
	 * Overrides the setOutputPower() power setting.
	 * @param maxPower High output power.  Must be between -16dBm and +14dBm.
	 * @param minPower Low output power.  Must be between maxPower and -17.5dBm.  -17.5dBm gives completely off,
	 *     so OOK modulation instead of ASK.
	 */
	void setASKPowers(float maxPower, float minPower);

	// Radio band for the chip to operate on.
	// See user guide description for FS_CFG register.
	enum class Band : uint8_t
	{
		BAND_820_960MHz = 0x2,
		BAND_410_480MHz = 0x4,
		BAND_273_320MHz = 0x6,
		BAND_205_240MHz = 0x8,
		BAND_164_192MHz = 0xA,
		BAND_136_160MHz = 0xB
	};

	/**
	 * Set the radio band and specific frequency.  See user guide section 9.12 for details.
	 * Note: Frequency offsets are not currently implemented, so the frequency can't be
	 * set at the finest resolution.  However, the resolution should be fine for most applications.
	 * (at 900MHz this function has a resolution of 152.5Hz)
	 * @param band
	 * @param frequencyHz
	 */
	void setRadioFrequency(Band band, float frequencyHz);

	/**
	 * Set the the RX filter bandwidth.  You must call this AFTER setting the symbol rate.
	 * See user guide section 6.1 for details.
	 *
	 * NOTE: The symbol rate and the RX filter bandwidth must be compatible with each other.
	 * See the user guide for details.
	 *
	 * A number of different registers must be configured in order to properly configure the radio for a given bandwidth.
	 * This call currently sets the following register fields:
	 * - CHAN_BW.ADC_CIC_DECFACT
	 * - CHAN_BW.BB_CIC_DECFACT
	 * - MDMCFG1.DVGA_GAIN
	 * - MDMCFG0.DATA_FILTER_EN
	 * - SYNC_CFG0.RX_CONFIG_LIMITATION
	 *
	 * @param bandwidthHz the bandwidth in Hz
	 * @param preferHigherCICDec If there are multiple register value choices, prefer the one with higher CIC decimation
	 * and lower BB decimation.  This is the recommendation of the datasheet but it actually causes transmission to fail in some cases.
	 */
	void setRXFilterBandwidth(float bandwidthHz, bool preferHigherCICDec = true);

	/**
	 * Get the ADC CIC decimation that was calculated by the most recent setRXFilterBandwidth() call.
	 * This is used for certain other calculations such as the DC offset.
	 * @return
	 */
	uint8_t getADCCICDecimation() { return adcCicDecimation; }

	/**
	 * Configure the radio's automatic DC offset removal algorithm as enabled.
	 * DC offset correction must be enabled when using zero IF mode, and in my testing
	 * it seems to be important when staying in TX mode for a long time at
	 * higher sample rates.
	 *
	 * See the datasheet register description for DCFILT_CFG for explanations of what these values do.
	 * Maybe you'll actually be able to make some sense out of what it says... I sure couldn't.
	 *
	 * @param enableAutoFilter Whether automatic filtering is enabled.
	 * @param settlingCfg Settling time configuration bits.
	 * @param cutoffCfg Cutoff frequency configuration bits.
	 */
	void configureDCFilter(bool enableAutoFilter, uint8_t settlingCfg, uint8_t cutoffCfg);

	/**
	 * Possible intermediate frequency values.
	 * For right now it seems like you have to get these from SmartRF.
	 * See the user guide section on IF_MIX_CFG.CMIX_CFG for details.
	 */
	enum class IFCfg : uint8_t
	{
		ZERO = 0, // Zero IF.  From what I can find, this means samples are taken at the radio frequency.
		NEGATIVE_DIV_4 = 0b001,
		NEGATIVE_DIV_6 = 0b010,
		NEGATIVE_DIV_8 = 0b011,
		POSITIVE_DIV_4 = 0b101,
		POSITIVE_DIV_6 = 0b110,
		POSITIVE_DIV_8 = 0b111
	};

	/**
	 * Set the receiver IF mixing configuration.
	 * See the user guide section on IF_MIX_CFG.CMIX_CFG for details.
	 *
	 * You must call *both* setRXFilterBandwidth() and setRadioFrequency() before calling this function.
	 *
	 * @param value Divider value to use, or zero-IF
	 * @param enableIQIC Whether to enable the ImageExtinct IQ mismatch compensation when supported
	 *   (when if IF > RX filter bandwidth).
	 */
	void setIFCfg(IFCfg value, bool enableIQIC);

	/**
	 * Mode describing the size and setup of the sync word.
	 * See user guide register description for SYNC_CFG1
	 */
	enum class SyncMode : uint8_t
	{
		SYNC_NONE = 0,
		SYNC_11_BITS = 0b1,
		SYNC_16_BITS = 0b10,
		SYNC_18_BITS = 0b11,
		SYNC_24_BITS = 0b100,
		SYNC_32_BITS = 0b101,
		SYNC_16_BITS_HIGH_BYTE = 0b110,
		SYNC_16_BITS_DUAL = 0b111
	};

	/**
	 * Configure the sync word settings of the radio. The sync word is the bit string sent before each packet -- the
	 * radio knows to switch into receive mode when it detects it.  Specific values with low autocorrelation should
	 * be used for the sync word.
	 *
	 * @param syncWord Sync word value.
	 * @param mode Sync word mode.  Configures how many bits of the value are significant.
	 * @param syncThreshold Correspondance threshold before the radio switches into receive mode.
	 */
	void configureSyncWord(uint32_t syncWord, SyncMode mode, uint8_t syncThreshold);

	/**
	 * Check whether the frequency synthesizer is locked on to the correct frequency.
	 * If not, then the correct RF frequency is not being used.
	 * If the FS is not locking then check that the correct black box FS registers are applied
	 * and that the FS has been calibrated.
	 * @return
	 */
	bool isFSLocked();

	/**
	 * Configure the preamble that the radio is configured to send/receive.  The main purpose of the preamble is to
	 * provide receiving radios with a chance to calibrate their RX gain.  However, you can also require that receiving
	 * radios see a valid preamble before they can detect the sync word (this is not on by default).
	 *
	 * @param preambleLengthCfg Bits that determine the length of the preamble. See the PREAMBLE_CFG1 register description for details. Set to 0 disable transmitting a preamble.
	 * @param preambleFormatCfg Bits that determine the format of the preamble.  See the PREAMBLE_CFG1 register description for details.
	 */
	void configurePreamble(uint8_t preambleLengthCfg, uint8_t preambleFormatCfg);

	/**
	 * Enum for different PA ramp times.
	 */
	enum class RampTime : uint8_t
	{
		RAMP_3_8_SYMBOL = 0b0,
		RAMP_3_2_SYMBOL = 0b1,
		RAMP_3_SYMBOL = 0b10,
		RAMP_6_SYMBOL = 0b11
	};

	/**
	 * Enable the the power amplifier ramp-up curve and set its shape and time.
	 * See section 7.1 for details.
	 * This is also used to set the ASK ramping between different power levels.
	 *
	 * The PA will gradually ramp from off to full amplitude in rampTime relative to the
	 * symbol rate.  At 1/3 of rampTime it will have ramped to (firstRampLevel / 16) * full amplitude,
	 * and at 2/3 of rampTime it will have ramped to ((secondRampLevel + 7) / 16) * full amplitude.
	 */
	void setPARampRate(uint8_t firstRampLevel, uint8_t secondRampLevel, RampTime rampTime);

	/**
	 * Disable the power amplifier ramp-up curve.
	 */
	void disablePARamping();

	// Automatic Gain Control (AGC) Config
	// ------------------------------------------------------------------------------

	/**
	 * Set the AGC reference level which is the internal target power level that
	 * the AGC tries to adjust to.
	 *
	 * The user manual section 6.4 gives a rough formula to calculate this, but I've just used the SmartRF values.
	 *
	 * @param level Internal power level in dB.
	 */
	void setAGCReferenceLevel(uint8_t level);

	/**
	 * Enum for possible AGC actions after is a sync word detection.
	 * See AGC_CFG3 register description for more info.
	 */
	enum class SyncBehavior : uint8_t
	{
		FREEZE_NONE = 0b000,
		FREEZE_GAIN = 0b001,
		AGC_SLOWMODE = 0b010,
		FREEZE_BOTH = 0b011
	};

	/**
	 * Set the AGC behavior after a sync word is detected.
	 * @param behavior
	 */
	void setAGCSyncBehavior(SyncBehavior behavior);

	/**
	 * Enum for possible gain tables to use.
	 * See AGC_CFG2 register description for more info.
	 */
	enum class GainTable : uint8_t
	{
		OPTIMIZED_LINEARITY = 0b00,
		NORMAL = 0b01,
		LOW_POWER = 0b10,
		ZERO_IF = 0b11
	};

	/**
	 * Set the gain table and min and max values within that table to use.
	 * Min and max values are indexes into the current selected table.
	 */
	void setAGCGainTable(GainTable table, uint8_t minGainIndex, uint8_t maxGainIndex);

	/**
	 * Configure the change in input signal power that must be sensed before the AGC starts to adjust itself.
	 * See the register description for AGC_CFG0.AGC_HYST_LEVEL
	 * @param hysteresisCfg
	 */
	void setAGCHysteresis(uint8_t hysteresisCfg);

	/**
	 * Configure the rate that the AGC changes the receive gain.
	 * See the register description for AGC_CFG0.AGC_SLEWRATE_LIMIT
	 * @param slewrateCfg
	 */
	void setAGCSlewRate(uint8_t slewrateCfg);

	/**
	 * Configure the time that the AGC takes to settle.
	 * See the register description for AGC_CFG1.AGC_SETTLE_WAIT
	 * @param settleWaitCfg bytes to write to AGC_CFG1.AGC_SETTLE_WAIT
	 */
	 void setAGCSettleWait(uint8_t settleWaitCfg);

	// Received Signal Strength Indicator (RSSI) and Link Quality Indicator (LQI) functions
	// ------------------------------------------------------------------------------

private:
	// data from packet status bytes
	int8_t lastRSSI;
	uint8_t lastLQI;

public:

	/**
	 * Get the RSSI as of the last packet received.
	 * Only provides valid data if appendStatus is enabled.
	 * @return RSSI in dBm, or -128 if no valid RSSI measurement exists.
	 */
	int8_t getLastRSSI() {return lastRSSI;}

	/**
	 * Get the current RSSI from the RSSI register.  Note: I think
	 * this might only work while the radio is actively receiving.
	 *
	 * @return RSSI in dBm, or NaN if no valid RSSI measurement exists.
	 */
	float getRSSIRegister();

	/**
	 * Set the RSSI gain adjustment.  This value is added to the reported RSSI, and also used
	 * in the calculation of the Carrier Sense (CS) line.
	 * You have to calibrate this in a lab by feeding in a known amplitude signal,
	 * see the user manual section 6.9 for details.
	 */
	void setRSSIOffset(int8_t adjust);

	/**
	 * Get the LQI from the LQI_VAL register.
	 * This is a qualitative estimate from 1-128 of how easily a packet can be demodulated.
	 * Lower is better, but 0 indicates invalid.
	 * @return
	 */
	uint8_t getLQIRegister();

	/**
	 * Get the LQI as of the last packet received.
	 * Only provides valid data if appendStatus is enabled.
	 * This is a qualitative estimate from 1-128 of how easily a packet can be demodulated.
	 * Lower is better, but 0 indicates invalid.
	 */
	uint8_t getLastLQI() {return lastLQI;}

	// Register level functions
	// ------------------------------------------------------------------------------

	/**
	 * Read a register and return the byte value.  Also reads the radio's state.
	 */
	uint8_t readRegister(Register reg);

	/**
	* Write a register with a byte value. Also reads the radio's state.
	*/
	void writeRegister(Register reg, uint8_t value);

	/**
	* Write a series of consecutive registers with byte values. Also reads the radio's state.
	*/
	void writeRegisters(CC1200::Register startReg, uint8_t const *values, size_t numRegisters);

	/**
	* Write a series of consecutive registers with byte values. Also reads the radio's state.
	* Template version that takes an std::array.
	*/
	template<size_t numRegisters>
	void writeRegisters(CC1200::Register startReg, std::array<uint8_t, numRegisters> const & values)
	{
		writeRegisters(startReg, values.data(), values.size());
	}

	/**
	* Read an extended register and return the byte value. Also reads the radio's state.
	*/
	uint8_t readRegister(ExtRegister reg);

	/**
	* Write an extended register with a byte value. Also reads the radio's state.
	*/
	void writeRegister(ExtRegister reg, uint8_t value);

	/**
	* Write a series of consecutive extended registers with byte values. Also reads the radio's state.
	*/
	void writeRegisters(CC1200::ExtRegister startReg, uint8_t const *values, size_t numRegisters);

	/**
	* Write a series of consecutive registers with byte values. Also reads the radio's state.
	* Template version that takes an std::array.
	*/
	template<size_t numRegisters>
	void writeRegisters(CC1200::ExtRegister startReg, std::array<uint8_t, numRegisters> const & values)
	{
		writeRegisters(startReg, values.data(), values.size());
	}


	/**
	 * Send a command. Also reads the radio's state.
	 * @param command
	 */
	void sendCommand(Command command);

	/**
	 * Update the current known state of the radio.
	 */
	void updateState() { sendCommand(Command::NOP); }

	/**
	 * Get a byte from the RX FIFO.
	 * @param address The byte address, from 0-127.
	 */
	uint8_t readRXFIFOByte(uint8_t address);

	// State change functions
	// ------------------------------------------------------------------------------

	/**
	 * Send the STX strobe to change the radio into TX state.
	 * Valid when the radio is in IDLE, FAST_TX_ON, and RX.
	 * A calibration will be performed if needed.
	 *
	 * The radio will stay in TX state until it is commanded to a different state, or a packet is
	 * transmitted and it is configured to change states when this happens, or a FIFO error occurs (which
	 * shouldn't be possible with the current configuration).
	 */
	void startTX() { sendCommand(Command::TX); }

	/**
	 * Send the SRX strobe to change the radio into TX state.
	 * Valid when the radio is in IDLE, FAST_TX_ON, and TX.
	 * A calibration will be performed if needed.
	 *
	 * The radio will stay in RX state until it is commanded to a different state, or a packet is
	 * received and it configured to change states when this happens, or a FIFO overflow occurs
	 * (because the host is not reading data out fast enough).
	 */
	void startRX() { sendCommand(Command::RX); }

	/**
	 * Send the radio into idle mode.  Stops a currently running tx or rx.
	 */
	void idle() { sendCommand(Command::IDLE); }

private:

	/**
	 * Called whenever we get a status byte from another operation.  Saves the info from it to member variables.
	 * @param status
	 */
	void loadStatusByte(uint8_t status);
};


#endif //LIGHTSPEEDRANGEFINDER_CC1200_H
