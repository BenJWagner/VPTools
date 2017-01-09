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

The sketch requires either a Moteino or any similar Arduino-compatible device equipped with:
* a Hope RF RFM69 series transceiver for the proper band for your country

Mark Crossley 2017
