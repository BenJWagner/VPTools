RelayRxTx.ino
--------------

This sketch is a relay device for Davis VP2 transmitters

It receives the transmissions from two or more Davis VP2 transmitters, and rebroadcasts the data as if all the sensors were connected to a single ISS.
Obviously it can only be used with the array of sensor types that can legitimately be connected to an ISS:
* Wind speed/direction
* Temperature/humidity
* Rain bucket
* Solar
* UV

My use-case is to attach my Solar and UV sensors to my Wind transmitter (id=1). This relay combines the wind/solar/UV data from the wind transmitter with
the rain/temperature/humidity from the ISS (id=2), and retransmits the data as if there was a single ISS (id=3) for the console to listen to.

The sketch implements a number of serial commands to configure some aspects of behaviour:

  tn.nnnnn
    (where 1.2 < n.nn > 0.8) - Sets the timer adjustment factor to compenstate for clock differences

  on
    (where n = 0, 1, 2, 3) - Switches serial data output
    0 = off
    1 = full stats + data
    2 = full data only
    3 = wind data only (speed,direction)

  fnnnnnnnn
    (where each n = 0 or 1) - Switches output from a particular transmitter id 1-8 off or on
    You can send only the ids up to last one you want to set, eg: only turn transmitter 3 off = f110

  r
    Shows the current radio stats for each transmitter id

  ?
    Shows the current configuration values

The configuration values are stored to EEPROM so will survive a power cycle.

The sketch requires either a Moteino or any similar Arduino-compatible device equipped with:
* a Hope RF RFM69 series transceiver for the proper band for your country

Mark Crossley 2017
