//
// Mark Crossley 2017
//

#include <Arduino.h>

#include <SPI.h>
#include <EEPROM.h>

#include <Wire.h>

#include "DavisRFM69.h"
#include "PacketFifo.h"


#define SERIAL_BAUD 115200
// Moteino #1
//#define TX_ID 2
// Moteino #2
#define TX_ID 3 // 0..7, Davis transmitter ID, set to a different value than all other transmitters
                // IMPORTANT: set it ONE LESS than you'd set it on the ISS via the DIP switch; 1 here is 2 on the ISS/Davis console
#define TX_PERIOD (41 + TX_ID) * 1000000 / 16 * DAVIS_INTV_CORR // TX_PERIOD is a function of the ID and some constants, in micros
                                                            // starts at 2.5625 and increments by 0.625 up to 3.0 for every increment in TX_ID


// Payload data structure for transmitting data
// Unconnected data contents are...
// uv:      40-00-00-ff-c5-00
// rainsec: 50-00-00-ff-71-00
// solar:   60-00-00-ff-c5-00
// temp:    80-00-00-ff-c5-00
// gust:    90-00-00-00-05-00
// rh:      a0-00-00-00-05-00
// rain:    e0-00-00-80-03-00
// unknown: c0-00-00-00-01-00
// for an unconnected wind sensor wind speed and direction are both 0
typedef struct __attribute__((packed)) Payload {
  byte wind[3];
  byte uv[6];
  byte rainsecs[6];
  byte solar[6];
  byte temp[6];
  byte windgust[6];
  byte hum[6];
  byte rain[6];
};

typedef struct __attribute__((packed)) PayloadStation {
  byte wind;
  byte uv;
  byte rain;
  byte solar;
  byte temp;
  byte hum;
};

// Observed sequence of transmitted ISS value types.
// The upper nibble is important, the lower nibble is the transmitter ID + battery flag.
// Type values for a standard VP2 ISS:
//     0x80 0xe0 0x40 0xa0 0x60 0x50 0x90 [0xc0]
//     temp rain uv   rh   sol  rsec gust [unkown]
// Wind speed and direction is transmitted in every packet at byte offsets 1 and 2.
static const byte txseq[20] = {
  0x8, 0xe, 0x5, 0x4,
  0x8, 0xe, 0x5, 0x9,
  0x8, 0xe, 0x5, 0xa,
  0x8, 0xe, 0x5, 0xa,
  0x8, 0xe, 0x5, 0x6
};


// DavisRFM69 radio base class;
DavisRFM69 radio(SPI_CS, RF69_IRQ_PIN, false, RF69_IRQ_NUM, true, true);

uint32_t lastTx;       // last time a radio transmission started
byte seqIndex;         // current packet type index into txseq array
byte channel;          // next transmission channel
byte digits1 = 1;      // used for print formatting
byte digits2 = 1;      // used for print formatting



// initialise the sensor payloads with 'sensor not connected' data
Payload payloads = {
//{0x03, 0xA8, 0x03},                    // dummy test wind data
  {0, 0, 0},                             // wind
  {0x40, 0x00, 0x00, 0xff, 0xc5, 0x00},  // uv
  {0x50, 0x00, 0x00, 0xff, 0x71, 0x00},  // rainsecs
  {0x60, 0x00, 0x00, 0xff, 0xc5, 0x00},  // solar
  {0x80, 0x00, 0x00, 0xff, 0xc5, 0x00},  // temp
  {0x90, 0x00, 0x00, 0x00, 0x05, 0x00},  // gust
  {0xa0, 0x00, 0x00, 0x00, 0x05, 0x00},  // humidity
  {0xe0, 0x00, 0x00, 0x80, 0x03, 0x00}   // rain
};

// station id associated with each payload
PayloadStation payloadStations = {
  255, 255, 255, 255, 255, 255   // wind, uv, rain, solar, temp, hum
};

// Stations to receive from
// id, type, active
#define NUM_RX_STATIONS 2   // Number of stations we are going to listen for
Station stations[NUM_RX_STATIONS] = {
  {0, STYPE_ISS, true},   // Anemometer txmr in my case
  {1, STYPE_ISS, true}    // The 'real' ISS
};

byte battery[NUM_RX_STATIONS];  // Array to hold battery status from each transmitter



void setup() {
  Serial.begin(SERIAL_BAUD);

  radio.setStations(stations, NUM_RX_STATIONS);
  radio.initialize(FREQ_BAND_US);
  radio.setBandwidth(RF69_DAVIS_BW_NARROW);

  // set the payload transmitter ids to match our transmitter id
  payloads.uv[0]       = payloads.uv[0]       + TX_ID;
  payloads.rainsecs[0] = payloads.rainsecs[0] + TX_ID;
  payloads.solar[0]    = payloads.solar[0]    + TX_ID;
  payloads.temp[0]     = payloads.temp[0]     + TX_ID;
  payloads.windgust[0] = payloads.windgust[0] + TX_ID;
  payloads.hum[0]      = payloads.hum[0]      + TX_ID;
  payloads.rain[0]     = payloads.rain[0]     + TX_ID;

  lastTx = micros();
  seqIndex = 0;
  channel = 0;

  for (byte i=0; i < NUM_RX_STATIONS; i++) {
    battery[i] = 0;
  }

  Serial.println("Setup done.");
  // print a header for the serial data log
  Serial.println("time\tRx/Tx\traw_packet_data              \tid\tpacket_counts\tchan\trssi\tfei\tdelta_t\tbatt\twind\tdir\tsensor\tvalue");
}


// Main loop
void loop() {
  if (micros() - lastTx < TX_PERIOD) {
    // Not time to transmit yet, check if we have received any data
    while (radio.fifo.hasElements()) {
      decode_packet(radio.fifo.dequeue());
    }
  } else {
    sendNextPacket();
  }
}



void decode_packet(RadioData* rd) {
  int val;
  byte* packet = rd->packet;
  byte *ptr;
  bool copyData = false;
  byte id = packet[0] & 7;

  // TODO: Need to detect loss of reception from a transimitter and then send 'not connected' packets for the sensors associated
  //       with that transmitter id until contact is regained. At present the code 'flat-lines' with the last received values

  // save wind data from every packet type - if valid
  if (packet[2] != 0 || id == payloadStations.wind) {
    payloads.wind[0] = packet[1];
    payloads.wind[1] = packet[2];
    payloads.wind[2] = packet[4];
    if (id != payloadStations.wind) {
      payloadStations.wind = id;
    }
  }

  // save battery status
  battery[id] = packet[0] & 8;


  switch (packet[0] >> 4) {
    case VP2P_UV:
      val = word(packet[3], packet[4]) >> 6;
      if (val < 0x3ff || id == payloadStations.uv) {
        copyData = true;
        ptr = payloads.uv;
        if (id != payloadStations.uv) {
          payloadStations.uv = id;
        }
     }
     break;

    case VP2P_SOLAR:
      val = word(packet[3], packet[4]) >> 6;
      if (val < 0x3fe || id == payloadStations.solar) {
        copyData = true;
        ptr = payloads.solar;
        if (id != payloadStations.solar) {
          payloadStations.solar = id;
        }
      }
      break;

    case VP2P_RAIN:
      if (packet[3] != 0x80 || id == payloadStations.rain) {
        copyData = true;
        ptr = payloads.rain;
        if (id != payloadStations.rain) {
          payloadStations.rain = id;
        }
      }
      break;

    case VP2P_RAINSECS:
      // PROBLEM - We need to store the data from the station when it resets to the 'not connected' value if it isn't raining,
      //         - but NOT from any other stations that may be reporting 'not connected' because they aren't rain stations!
      val = (packet[4] & 0x30) << 4 | packet[3];
      // is the packet from the station that previuously reported rain, OR do we have some rain data
      if (val != 0x3ff || id == payloadStations.rain) {
        // we have some rain data
        copyData = true;
        ptr = payloads.rainsecs;
        if (id != payloadStations.rain) {
          payloadStations.rain = id;
        }
      }
      break;

    case VP2P_TEMP:
      if (packet[3] != 0xff || id == payloadStations.temp) {
        copyData = true;
        ptr = payloads.temp;
        if (id != payloadStations.temp) {
          payloadStations.temp = id;
        }
      }
      break;

    case VP2P_HUMIDITY:
      val = ((packet[4] >> 4) << 8 | packet[3]) / 10; // 0 -> no sensor
      if (val > 0 || id == payloadStations.hum) {
        copyData = true;
        ptr = payloads.hum;
        if (id != payloadStations.hum) {
          payloadStations.hum = id;
        }
      }
      break;

    case VP2P_WINDGUST:
      if (packet[2] > 0 || id == payloadStations.wind) {  // 0 -> no sensor
        copyData = true;
        ptr = payloads.windgust;
        if (id != payloadStations.wind) {
          payloadStations.wind = id;
        }
      }
      break;

    case VP2P_SOIL_LEAF:
      break;
    case VUEP_VCAP:
      break;
    case VUEP_VSOLAR:
      break;
  }

  if (copyData) {
    memcpy(ptr + 1, (byte *)packet + 1, 5);
  }

  // dump it to the serial port
  printIt(rd->packet, 'R', rd->channel, radio.packets, radio.lostPackets, rd->rssi, rd->fei, rd->delta);

}

void printHex(volatile byte* packet, byte len) {
  for (byte i = 0; i < len; i++) {
    if (!(packet[i] & 0xf0)) Serial.print('0');
    Serial.print(packet[i], HEX);
    if (i < len - 1) Serial.print('-');
  }
}


void sendNextPacket() {
  byte *ptr;
  uint32_t now, delta;

  switch (txseq[seqIndex]) {
    case VP2P_UV:
      ptr = payloads.uv;
      break;
    case VP2P_SOLAR:
      ptr = payloads.solar;
      break;
    case VP2P_RAIN:
      ptr = payloads.rain;
      break;
    case VP2P_RAINSECS:
      ptr = payloads.rainsecs;
      break;
    case VP2P_TEMP:
      ptr = payloads.temp;
      break;
    case VP2P_HUMIDITY:
      ptr = payloads.hum;
      break;
    case VP2P_WINDGUST:
      ptr = payloads.windgust;
      break;
  }

  noInterrupts();

  // set the battery status, if any are set, then set it in relayed data
  // if any station has the flag set, then re-transmit it
  for (byte i = 0; i < NUM_RX_STATIONS; i++) {
    ptr[0] = ptr[0] | battery[i];
  }

  // Add in the latest wind data
  ptr[1] = payloads.wind[0];
  ptr[2] = payloads.wind[1];
  //ptr[4] = (ptr[4] & 0xfd) | (payloads.wind[2] & 0x02);

  // clear the flag if wind data valid
  if (ptr[1]) {
    bitClear(ptr[4], 2);
  }

  // Get current time
  now = micros();

  // Send packet
  radio.send(ptr, channel);

  // re-enable interrrupts
  interrupts();

  // dump it to the serial port
  printIt((byte*)radio.DATA, 'T', channel, 0, 0, 0, 0, now - lastTx);

  // record last txmt time
  lastTx = now;

  // move things on for the next transmission...
  channel = radio.nextChannel(channel);
  seqIndex = nextPktType(seqIndex);

}


// Calculate the next packet type to transmit
byte nextPktType(byte packetIndex) {
  return ++packetIndex % sizeof(txseq);
}


// Dump data to serial port
void printIt(byte *packet, char rxTx, byte channel, uint32_t packets, uint32_t lostPackets, byte rssi, int16_t fei, uint32_t delta) {
  int val;
  byte *ptr;
  byte i;

  Serial.print(micros() / 1000);
  Serial.print('\t');
  Serial.print(rxTx);
  Serial.print('\t');
  printHex(packet, 10);
  Serial.print('\t');

  byte id = packet[0] & 7;
  Serial.print(id + 1);
  Serial.print('\t');

  if (rxTx == 'R') {
    digits1 = Serial.print(packets);
    Serial.print('/');
    digits2 = Serial.print(lostPackets);
    Serial.print('/');
    Serial.print((float)(packets * 100.0 / (packets + lostPackets)), 1);
  } else {
    for (i=0; i < digits1; i++) {
      Serial.print('-');
    }
    Serial.print('/');
    for (i=0; i < digits2; i++) {
      Serial.print('-');
    }
    Serial.print("/----");
  }
  Serial.print('\t');

  Serial.print(channel);
  Serial.print('\t');
  if (rxTx == 'R') {
    Serial.print(-rssi);
  } else {
    Serial.print("--");
  }
  Serial.print('\t');
  if (rxTx == 'R') {
    Serial.print(round(fei * RF69_FSTEP / 1000));
  } else {
    Serial.print("--");
  }
  Serial.print('\t');
  Serial.print(delta);
  Serial.print('\t');

  Serial.print((char*)(packet[0] & 0x8 ? "bad" : "ok"));
  Serial.print('\t');

  // All packet payload values are printed unconditionally, either properly
  // calculated or flagged with a special "missing sensor" value '-'.

  int stIx = rxTx == 'R' ? radio.findStation(id) : TX_ID;

  // wind data is present in every packet, windd == 0 (packet[2] == 0) means there's no anemometer
  if (packet[2] != 0) {
    if (stations[stIx].type == STYPE_VUE) {
      val = (packet[2] << 1) | (packet[4] & 2) >> 1;
      val = round(val * 360 / 512);
    } else {
      val = 9 + round((packet[2] - 1) * 342.0 / 255.0);
      // subject to processing in the console for direction compensation
    }
    Serial.print(packet[1]);
    Serial.print('\t');
    Serial.print(val);
    Serial.print('\t');
  } else {
    Serial.print("-\t-\t");
  }

  switch (packet[0] >> 4) {

    case VP2P_UV:
      Serial.print("uv\t");
      val = word(packet[3], packet[4]) >> 6;
      if (val < 0x3ff) {
        Serial.print((float)(val / 50.0), 1);
     } else {
        Serial.print('-');
      }
      break;

    case VP2P_SOLAR:
      Serial.print("solar\t");
      val = word(packet[3], packet[4]) >> 6;
      if (val < 0x3fe) {
        if (val > 3) {
          val = (val * 1.757936);
        } else {
          val = 0;
        }
        Serial.print(val);
      } else {
        Serial.print('-');
      }
      break;

    case VP2P_RAIN:
      Serial.print("rain\t");
      if (packet[3] == 0x80) {
        Serial.print('-');
      } else {
        Serial.print(packet[3]);
      }
      break;

    case VP2P_RAINSECS:
      Serial.print("r_secs\t");
      // light rain:  byte4[5:4] as value[9:8] and byte3[7:0] as value[7:0] - 10 bits total
      // strong rain: byte4[5:4] as value[5:4] and byte3[7:4] as value[3:0] - 6 bits total
      val = (packet[4] & 0x30) << 4 | packet[3];
      if (val == 0x3ff) {
        Serial.print('-');
      } else {
        // packet[4] bit 6: strong == 0, light == 1
        if ((packet[4] & 0x40) == 0) {
          val >>= 4;
        }
        Serial.print(val);
      }
      break;

    case VP2P_TEMP:
      Serial.print("temp\t");
      if (packet[3] == 0xff) {
        Serial.print('-');
      } else {
        val = (int)packet[3] << 4 | packet[4] >> 4;
        Serial.print((float)(val / 10.0), 1);
        Serial.print(" (");
        Serial.print((float)(((val / 10.0) - 32) * (5.0 / 9.0)), 1);
        Serial.print(')');
      }
      break;

    case VP2P_HUMIDITY:
      Serial.print("hum\t");
      val = ((packet[4] >> 4) << 8 | packet[3]) / 10; // 0 -> no sensor
      if (val == 0) {
        Serial.print('-');
      } else {
        Serial.print(val + 1);
      }
      break;

    case VP2P_WINDGUST:
      Serial.print("gust\t");
      if (packet[2] == 0) {
        Serial.print('-');
      } else {
        Serial.print(packet[3]);
        if (packet[3] != 0) {
          Serial.print("\tgustref:");
          Serial.print(packet[5] & 0xf0 >> 4);
        }
      }
      break;

    case VP2P_SOIL_LEAF:
      // Not implemented, see...
      // https://github.com/cmatteri/CC1101-Weather-Receiver/wiki/Soil-Moisture-Station-Protocol
      /*
      byte subType;
      byte port;
      subType = packet[1] & 0x03;
      port = (packet[1] >> 5) & 0x07;
      Serial.print("soilleaf\t");
      if (subType == 1) {
	      val = (packet[2] << 2) + (packet[4] >> 6);    // Soil Moisture
	      Serial.print((String)(32+port) + ": " + val + " ");
	      val = (packet[3] << 2) + (packet[5] >> 6);    // Soil Temp
	      Serial.print((String)(36+port) + ": " + val);
      } else {
	      Serial.print((String)(40+port) + ": -");
      }
      */
      break;

    case VUEP_VCAP:
      val = (packet[3] << 2) | (packet[4] & 0xc0) >> 6;
      Serial.print("vcap\t");
      Serial.print((float)(val / 300.0));
      break;

    case VUEP_VSOLAR:
      val = (packet[3] << 2) | (packet[4] & 0xc0) >> 6;
      Serial.print("vsolar\t");
      Serial.print((float)(val / 300.0));
      break;

    default:
      Serial.print("unknown");
  }

  Serial.println();

}

