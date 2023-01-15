# Mbed OS CC1200 Radio Driver

This driver can be used in Mbed OS projects to control the CC1200 radio IC.  Most features of the IC are implemented, and you aren't limited to premade configurations!  This code implements the calculations needed to configure each part of the chip from raw frequency and data rate inputs.

As an additional utility, this driver includes the CC1200Morse class, which can be used to dynamically reconfigure the CC1200 to send Morse code.  Great for ham radio applications!

Migrated from the original Mbed repository [here](https://os.mbed.com/users/MultipleMonomials/code/CC1200/shortlog/).

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
See the cc1200-demo project for examples of how to use the driver.


## Changelog

### Version 2.0 Jan 2023
- Migrate to CMake build system
- Removed usage of deprecated Stream for console prints
- Bring in CC1200Morse project

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