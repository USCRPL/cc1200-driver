# Mbed OS CC1200 Radio Driver

![CC1200 Dev Kit](https://www.ti.com/content/dam/ticom/images/products/ic/processors/evm-boards/cc1200emk-868-930-angled.png:large)

This driver can be used in Mbed OS projects to control the CC1200 (and CC1201) radio ICs.  Most features of the IC are implemented, and you aren't limited to premade configurations!  This code implements the calculations needed to configure each part of the chip from raw frequency and data rate inputs.

As an additional utility, this driver includes the CC1200Morse class, which can be used to dynamically reconfigure the CC1200 to send Morse code.  Great for ham radio applications!

Written by Jamie Smith @ USC Rocket Propulsion Lab.

Migrated from the original Mbed repository [here](https://os.mbed.com/users/MultipleMonomials/code/CC1200/shortlog/).

## About the CC1200
The [CC1200](https://www.ti.com/product/CC1200) is a digital radio transceiver supporting a large variety of different bands, modulations, and packet formats. It lets you use a single part (with different circuitry) to transmit on a couple different bands, including the 430MHz European ISM band and the 900MHz American ISM band. It can be configured for a variety of different packet formats, including OOK, ASK, and a number of variants of FSK. Data rates can be as high as 1Mbps (using 4-FSK at 500kbps), or as low as a few hundred bytes per second depending on your bandwidth needs and distance requirements. All in all, this is an extremely capable radio chip can be adapted for almost any 100-900MHz digital radio application.

## Features

- Automatic calculation of correct register values for:
  - RF frequency
  - FSK deviation
  - Symbol rate
  - Output power
  - RX filter bandwidth (this one's harder than it looks!)
- Easy handling of data packets
- GPIO configuration
- Preamble and sync word configuration
- RTOS compatible (always locks SPI bus during transactions)
- Two debug levels available
- RSSI and LQI support

### Not Supported:

- Transparent mode
- FM mode
- ASK parameter configuration
- Frequency offsets


## Examples
See the [cc1200-demo](https://github.com/mbed-ce/cc1200-demo) project for examples of how to use the driver.

## Note: Radio Settings
For configuring radio settings, TI provides a number of configurations for you in their SmartRF application. The MBed OS driver lets you use these, but you can also enter your own settings if you need something different than what SmartRF provides. I will say, in my experience, the CC1200 does tend to be a bit of a house of cards - changing even one value to be incorrect (out of the 10-15 values that you need to configure) can easily cause the chip to stop functioning entirely. So, I recommend you stick to the provided configurations if possible, and only change things if you know what you're doing and are sure that you need a different value.

## Changelog

### Version 2.0 Jan 14 2023
- Migrate to CMake build system
- Switch the debug stream to be a FILE * instead of the deprecated Stream class
- Bring in CC1200Morse project
- Switch time unit in CC1200Morse to std::chrono

### Version 1.2 May 3 2021

- Added unfinished infinite length packet support via the readStream() and writeStream() functions. The API is complete and basic usage works but there's still a bug I haven't been able to track down yet where incorrect data is transmitted at the end of a stream. Use with caution!
- Added preferHigherCICDec parameter to setRXFilterBandwidth
- Removed setIFMixCFG() (which takes a byte parameter) and replaced it with setIFCfg(), which takes documented enum class values.
- Added setAGCSettleWait(), which per my testing is needed for correct 430MHz operation.
- Added support for reading RSSI and LQI values, both from packet appended status bytes and from the registers.
- Update 430MHz black box registers based on SmartRF values
- Removed setIQMismatchCompensationEnabled(). This call has been replaced by the new 2nd parameter to setIFCfg().

### Version 1.1 Aug 28 2020

- Add fixed length packet support and other features needed for Morse support.
- Fix bug causing weird behavior with low sample rates (<1ksps).

NOTE: you must now call setPacketMode() when configuring the radio.

### Version 1.0 Aug 10 2020

Initial Release