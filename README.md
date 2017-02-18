VPTools
-------

VPTools is a Davis(TM) weather station compatible transceiver library. It is a work in progress. It can receive from multiple RF transmitters (ISS, ATK, etc.) It tries to serve as a basis for similar hobby projects. Example sketches provide simple, parseable and RAW packet output among others.

* The ISSRx sketch is a receiver with a simple interface configurable in source code.
* The AnemometerTX sketch is a simple anemometer transmitter for Davis anemometers. It's a work in progress and needs optimisations for low power operation, but can serve as a base for doing so. It can easily be adapted to emulate any Davis transmitter.
* The WxReceiver sketch is a receiver compatible with the [WeeWx driver](https://github.com/matthewwall/weewx-meteostick) written by Matthew Wall et al for the Meteostick.

Originally based on [code by DeKay](https://github.com/dekay/DavisRFM69) - for technical details on packet formats, etc. see his (now outdated) [wiki](https://github.com/dekay/DavisRFM69/wiki) and the source code of this repo.

The RelayRxTx sketch now accepts a limited set of serial commands:
 >t0.000000    - Sets the timer calibration variable
 >o[0|1|2]     - Controls the serial logging output
                 0 = Off, not serial data logged
                 1 = On, all data is output
                 2 = Limited, only the weather data is output
 >f11111111    - Filter the serial output by transmitter id
                 8x positions for transmitter ids 1-8
                 You only need to enter the transmitter up to the last one you want to change.
                 E.g. f110 will disable output of data from transitter id 3
 >q000         - Sets the frequency offset used by the RFM69 module. Accepts corrections in
                 the range -800 to 800
 >?            - Dumps the current configuration values to the serial port

The configuration values are stored to non-volatile memory so will survive a Moteino power cycle.
Using these commands you can tweak the configuration on the fly to get the best reception on the Davis VP2 console.
