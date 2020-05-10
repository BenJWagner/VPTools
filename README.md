# RelayRxTx.ino

This sketch is a relay device for Davis VP2 transmitters.
This repository has been optimised for the RelayRxTx sketch, I do not recommend using for the other sketches.
The RelayRxTx example folder contains a customised version of the DavisRFM69 module and an updated TimerOne module, use these rather than the versions in the main repo.

It receives the transmissions from two or more Davis VP2 transmitters, and rebroadcasts the data as if all the sensors were connected to a single ISS.
Obviously it can only be used with the array of sensor types that can legitimately be connected to an ISS:
* Wind speed/direction
* Temperature/humidity
* Rain bucket
* Solar
* UV

My use-case is to attach my Solar and UV sensors to my Wind transmitter (id=1). This relay combines the wind/solar/UV data from the wind transmitter with
the rain/temperature/humidity from the ISS (id=2), and retransmits the data as if there was a single ISS (id=3) for the console to listen to.

## Configuration
You will need to configure a number of items in the sketch to customise it to your installation...


In the RelayRxTx.ino file:

     21: #define TX_ID 2 // 0..7, Davis transmitter ID

     24: #define NUM_RX_STATIONS 2   // Number of stations we are going to listen for

    131: // station id associated with each payload
    132: // Zero relative = Davis ID -1
    133: static PayloadStation payloadStations = {
    134:   1, 1, 1, 0, 1, 1   // wind, uv, rain, solar, temp, hum
    135: };

    137: // Stations to receive from
    138: // id, type, active
    139: static Station stations[NUM_RX_STATIONS] = {
    140:   {0, STYPE_ISS, true},   // Anemometer txmr in my case
    141:   {1, STYPE_ISS, true}    // The 'real' ISS
    142: };

    153:   radio.initialize(FREQ_BAND_EU);  // FREQ_BAND_US|AU|EU|NZ

That should be enough to get you going, if your board does not have an LEDs then disable them as well.

## Command Line

The sketch implements a number of serial commands to configure some aspects of behaviour:

- tn.nnnnn

    (where 1.2 < n.nn < 0.8) - Sets the timer adjustment factor to compensate for clock differences
- on

    (where n = 0, 1, 2, 3) - Switches serial data output

    ```
    0 = off
    1 = full stats + data
    2 = full data only
    3 = wind data only (speed,direction)
    4 = radio packets only
    ```
- fnnnnnnnn

    (where each n = 0 or 1) - Switches logging output from a particular transmitter id 1-8 off or on.

    You can send only the ids up to last one you want to set, eg: only turn transmitter 3 off = f110

- r

    Shows the current radio stats for each transmitter id

- qnnnn

    (where -1000 < nnn < 1000) Sets the frequency offset

- ?

    Shows the current configuration values

The configuration values are stored to EEPROM so will survive a power cycle.

The sketch requires either a Moteino or any similar Arduino-compatible device equipped with:
* a Hope RF RFM69 series transceiver for the proper band for your country

## Hints

* Initially make sure that you have full output details on the serial port.

* Check the "fei" values on the received frames, adjust using the "qnnnn" command to bring them down to low single digits (positive or negative). nnnn = 100 adjusts fei by approximately -20. Once you have done this then check field 5 on the VP2 console Reception Diagnostics screen, it should bring that down into the range +/-5. Tweak the q value if required to get it close to zero, but watch your fei numbers do not grown too large. Field 2 on that screen should show numbers between +/-2.

* Check the timing, field 1 on the VP2 console shows the last eight bits of the interval timer. The sequential jumps for each transmitter number should be approximately...

  ````
  1=192, 2=128, 3=64, 4=1, (untested values: 5=60, 6=132, 7=71, 8=8)
  ````

  Adjust the timing using the "tn.nnnn" command so that your intervals are as close to the value above as you can get. You will probably only want to adjust by 1% at a time at most, so start with something like t0.999 or t1.001

Mark Crossley 2020
