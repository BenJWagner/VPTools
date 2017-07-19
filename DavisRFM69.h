// Driver definition for HopeRF RFM69W/RFM69HW, Semtech SX1231/1231H used for
// compatibility with the frequency hopped, spread spectrum signals from a Davis Instrument
// wireless Integrated Sensor Suite (ISS)
//
// This is part of the DavisRFM69 library from https://github.com/dekay/DavisRFM69
// (C) DeKay 2014 dekaymail@gmail.com
//
// As I consider this to be a derived work from the RFM69W library from LowPowerLab,
// it is released under the same Creative Commons Attrib Share-Alike License
// You are free to use/extend this library but please abide with the CC-BY-SA license:
// http://creativecommons.org/licenses/by-sa/3.0/
//
// In accordance with the CC-BY-SA, many modifications by GitHub user "kobuki".

#ifndef DAVISRFM69_h
#define DAVISRFM69_h

#include <Arduino.h>            //assumes Arduino IDE v1.0 or greater

#include "PacketFifo.h"

#define DAVIS_PACKET_LEN     10 // ISS has fixed packet lengths of eight bytes, including CRC and trailing repeater info
#define SPI_CS               SS // SS is the SPI slave select pin, for instance D10 on atmega328
//#define RF69_IRQ_PIN          2 // INT0 on AVRs should be connected to DIO0 (ex on Atmega328 it's D2)
// INT0 on AVRs should be connected to RFM69's DIO0 (ex on ATmega328 it's D2, on ATmega644/1284 it's D2)
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega88) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega88__)
  #define RF69_IRQ_PIN          2
  #define RF69_IRQ_NUM          0
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
  #define RF69_IRQ_PIN          2
  #define RF69_IRQ_NUM          2
#elif defined(__AVR_ATmega32U4__)
  #define RF69_IRQ_PIN          3
  #define RF69_IRQ_NUM          0
#else
  #define RF69_IRQ_PIN          2
  #define RF69_IRQ_NUM          0
#endif

#define CSMA_LIMIT          -85 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP       0 // XTAL OFF
#define RF69_MODE_STANDBY     1 // XTAL ON
#define RF69_MODE_SYNTH       2 // PLL ON
#define RF69_MODE_RX          3 // RX MODE
#define RF69_MODE_TX          4 // TX MODE

#define RF69_DAVIS_BW_NARROW  1
#define RF69_DAVIS_BW_WIDE    2

#define null                  0
#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_FSTEP 61.03515625 	// == FXOSC/2^19 = 32mhz/2^19 (p13 in DS)

#define RESYNC_THRESHOLD 30       // max. number of lost packets from a station before rediscovery
#define LATE_PACKET_THRESH 5000L  // packet is considered missing after this many micros
#define POST_RX_WAIT 2000         // RX "settle" delay
#define MAX_STATIONS 8            // max. stations this code is able to handle

#define DISCOVERY_MINGAP 70000L       // Not used?
#define DISCOVERY_GUARD  20000L        // time before expected transmission to switch on receiver
#define DISCOVERY_STEP   60000000L   // time before moving the discovery channel on - to avoid interference?

#define FREQ_CORR 0         // Frequency correction factor for RFM69 = (Required correction Hz) / 61
                            // E.g. 20kHz = 20000/61 = 328, should use RF_FSTEP, but 61 is close enough
                            // My Moteino's #1 = -450, #2 = -340

#define DAVIS_INTV_CORR 1   // Used in the formula to calculate the station transmission interval - (41 + id) / 16
                            // Nominally = 1, but I found my Arduino was measuring longer intervals on the microsec clock for my transmitters
                            // My Moteino's #1 = 0.99811, #2 = 0.99770

// Station data structure for managing radio reception
typedef struct __attribute__((packed)) Station {
  byte id;                	// station ID (set with the DIP switch on original equipment)
                          	// set it ONE LESS than advertised station id, eg. 0 for station 1 (default) etc.
  byte type;              	// STYPE_XXX station type, eg. ISS, standalone anemometer transmitter, etc.
  bool active;            	// true when the station is actively listened to but ignored
  byte repeaterId;        	// repeater id when packet is coming via a repeater, otherwise 0
                          	// repeater IDs A..H are stored as 0x8..0xf here
  byte channel;           	// rx channel the next packet of the station is expected on
  uint32_t lastRx;   	 	// last time a packet is seen or should have been seen when missed
  uint32_t lastSeen; 	 	// last factual reception time
  uint32_t interval;    	// packet transmit interval for the station: (41 + id) / 16 * 1M microsecs
  uint32_t numResyncs;  	// number of times discovery of this station started because of packet loss
  uint32_t packets; 		// total number of received packets after (re)restart
  uint32_t missedPackets;	// total number of misssed packets after (re)restart
  byte lostPackets;         // missed packets since a packet was last seen from this station
};

class DavisRFM69 {
  public:
    static volatile byte DATA[DAVIS_PACKET_LEN];  // recv/xmit buf, including header, CRC, and RSSI value
    static volatile byte _mode; //should be protected?
    static volatile bool _packetReceived;
    static volatile byte CHANNEL;
    static volatile int RSSI;
    static volatile int16_t FEI;
	  static volatile byte band;

  	static volatile uint32_t packets;
  	static volatile uint32_t lostPackets;
  	static volatile uint32_t numResyncs;
  	static volatile uint32_t lostStations;
  	static volatile byte stationsFound;
  	static volatile byte curStation;
  	static volatile byte numStations;
    static volatile byte hopIndex;
    static volatile byte discChannel;
	  static volatile uint32_t lastDiscStep;

  	static PacketFifo fifo;
  	static Station *stations;

    DavisRFM69(byte slaveSelectPin=SPI_CS, byte interruptPin=RF69_IRQ_PIN, bool isRFM69HW=false, byte interruptNum=RF69_IRQ_NUM) {
      _slaveSelectPin = slaveSelectPin;
      _interruptPin = interruptPin;
      _interruptNum = interruptNum;
      _mode = RF69_MODE_STANDBY;
      _packetReceived = false;
      _powerLevel = 31;
      _isRFM69HW = isRFM69HW;
      _timerCalibration = DAVIS_INTV_CORR;
      _freqCalibration = FREQ_CORR;
    }

    void setChannel(byte channel);
    void hop();
    static uint16_t crc16_ccitt(volatile byte *buf, byte len, uint16_t initCrc = 0);

    void initialize(byte freqBand);
    bool canSend();
    void send(const void* buffer, byte channel=255);
    bool receiveDone();
    void setFrequency(uint32_t FRF);
    void setCS(byte newSPISlaveSelect);
    int readRSSI(bool forceTrigger=false);
    void setHighPower(bool onOFF=true); //have to call it after initialize for RFM69HW
    void setPowerLevel(byte level); //reduce/increase transmit power level
    void sleep();
    byte readTemperature(byte calFactor=0); //get CMOS temperature (8bit)
    void rcCalibration(); //calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section [4.3.5. RC Timer Accuracy]

    // allow hacking registers by making these public
    byte readReg(byte addr);
    void writeReg(byte addr, byte val);
    void readAllRegs();
    void setTxMode(bool txMode);
  	void setBand(byte newBand);
  	void setBandwidth(byte bw);
  	byte getBandTabLength();

  	byte nextChannel(byte channel);
  	int findStation(byte id);
  	void handleRadioInt();
  	void initStations();
  	void nextStation();

  	static void handleTimerInt();
  	static void setStations(Station *_stations, byte n);
  	void stopReceiver();
  	void setRssiThreshold(int rssiThreshold);
  	void setRssiThresholdRaw(int rssiThresholdRaw);
    float getTimerCalibation();
    bool  setTimerCalibation(float timerCalibration);
    int   getFrequencyCalibation();
    bool  setFrequencyCalibation(int freqCalibration);
    void  setMode(byte mode);

  protected:
    static volatile bool txMode;
    void (*userInterrupt)();

    void virtual interruptHandler();
    void sendFrame(const void* buffer);
    byte reverseBits(byte b);

    static void isr0();

    static DavisRFM69* selfPointer;
    byte _slaveSelectPin;
    byte _interruptPin;
    byte _interruptNum;
    byte _powerLevel;
    bool _isRFM69HW;

    float _timerCalibration;
    int   _freqCalibration;

    void receiveBegin();
    void setHighPowerRegs(bool onOff);
    void select();
    void unselect();
};

// FRF_MSB, FRF_MID, and FRF_LSB for the 51 North American, Australian, New Zealander & 5 European channels
// used by Davis in frequency hopping

#define FREQ_TABLE_LENGTH_US 51
#define FREQ_TABLE_LENGTH_AU 51
#define FREQ_TABLE_LENGTH_EU 5
#define FREQ_TABLE_LENGTH_NZ 51

#define FREQ_BAND_US 0
#define FREQ_BAND_AU 1
#define FREQ_BAND_EU 2
#define FREQ_BAND_NZ 3

typedef uint8_t FRF_ITEM[3];

static const __attribute__((progmem)) FRF_ITEM FRF_US[FREQ_TABLE_LENGTH_US] =
{
  {0xE3, 0xDA, 0x7C}, // 911413818
  {0xE1, 0x98, 0x71}, // 902381897
  {0xE3, 0xFA, 0x92}, // 911915161
  {0xE6, 0xBD, 0x01}, // 922953186
  {0xE4, 0xBB, 0x4D}, // 914926575
  {0xE2, 0x99, 0x56}, // 906395874
  {0xE7, 0x7D, 0xBC}, // 925964600
  {0xE5, 0x9C, 0x0E}, // 918438354
  {0xE3, 0x39, 0xE6}, // 908904663
  {0xE6, 0x1C, 0x81}, // 920445374
  {0xE4, 0x5A, 0xE8}, // 913420410
  {0xE1, 0xF8, 0xD6}, // 903888062
  {0xE5, 0x3B, 0xBF}, // 916933533
  {0xE7, 0x1D, 0x5F}, // 924458923
  {0xE3, 0x9A, 0x3C}, // 910409912
  {0xE2, 0x39, 0x00}, // 904890625
  {0xE4, 0xFB, 0x77}, // 915929138
  {0xE6, 0x5C, 0xB2}, // 921448364
  {0xE2, 0xD9, 0x90}, // 907399414
  {0xE7, 0xBD, 0xEE}, // 926967651
  {0xE4, 0x3A, 0xD2}, // 912919067
  {0xE1, 0xD8, 0xAA}, // 903385376
  {0xE5, 0x5B, 0xCD}, // 917434387
  {0xE6, 0xDD, 0x34}, // 923456299
  {0xE3, 0x5A, 0x0A}, // 909406860
  {0xE7, 0x9D, 0xD9}, // 926466370
  {0xE2, 0x79, 0x41}, // 905894592
  {0xE4, 0x9B, 0x28}, // 914424316
  {0xE5, 0xDC, 0x40}, // 919441406
  {0xE7, 0x3D, 0x74}, // 924960205
  {0xE1, 0xB8, 0x9C}, // 902884521
  {0xE3, 0xBA, 0x60}, // 910912109
  {0xE6, 0x7C, 0xC8}, // 921949707
  {0xE4, 0xDB, 0x62}, // 915427856
  {0xE2, 0xB9, 0x7A}, // 906898071
  {0xE5, 0x7B, 0xE2}, // 917935669
  {0xE7, 0xDE, 0x12}, // 927469849
  {0xE6, 0x3C, 0x9D}, // 920947083
  {0xE3, 0x19, 0xC9}, // 908402893
  {0xE4, 0x1A, 0xB6}, // 912417358
  {0xE5, 0xBC, 0x2B}, // 918940125
  {0xE2, 0x18, 0xEB}, // 904389343
  {0xE6, 0xFD, 0x42}, // 923957153
  {0xE5, 0x1B, 0xA3}, // 916431824
  {0xE3, 0x7A, 0x2E}, // 909909058
  {0xE5, 0xFC, 0x64}, // 919943604
  {0xE2, 0x59, 0x16}, // 905391968
  {0xE6, 0x9C, 0xEC}, // 922451904
  {0xE2, 0xF9, 0xAC}, // 907901123
  {0xE4, 0x7B, 0x0C}, // 913922607
  {0xE7, 0x5D, 0x98}  // 925462402
};

static const __attribute__((progmem)) FRF_ITEM FRF_AU[FREQ_TABLE_LENGTH_AU] =
{
  {0xE5, 0x84, 0xDD}, // 918075989
  {0xE6, 0x43, 0x43}, // 921050964
  {0xE7, 0x1F, 0xCE}, // 924496948
  {0xE6, 0x7F, 0x7C}, // 921991943
  {0xE5, 0xD5, 0x0E}, // 919328979
  {0xE7, 0x5B, 0xF7}, // 925436951
  {0xE6, 0xC5, 0x81}, // 923085999
  {0xE6, 0x07, 0x2B}, // 920112000
  {0xE6, 0xED, 0xA1}, // 923712952
  {0xE6, 0x61, 0x58}, // 921520996
  {0xE5, 0xA3, 0x02}, // 918546997
  {0xE6, 0xA7, 0x8D}, // 922617981
  {0xE7, 0x3D, 0xB2}, // 924963989
  {0xE6, 0x25, 0x3F}, // 920581970
  {0xE5, 0xB7, 0x0A}, // 918859985
  {0xE6, 0x93, 0x85}, // 922304993
  {0xE7, 0x01, 0xDB}, // 924028992
  {0xE5, 0xE9, 0x26}, // 919642944
  {0xE7, 0x70, 0x00}, // 925750000
  {0xE6, 0x57, 0x6C}, // 921365967
  {0xE5, 0x98, 0xF5}, // 918389954
  {0xE6, 0xB1, 0x99}, // 922774963
  {0xE7, 0x29, 0xDB}, // 924653992
  {0xE6, 0x11, 0x37}, // 920268982
  {0xE7, 0x65, 0xE3}, // 925591980
  {0xE5, 0xCB, 0x33}, // 919174988
  {0xE6, 0x75, 0x60}, // 921833984
  {0xE6, 0xD9, 0xA9}, // 923400940
  {0xE7, 0x47, 0xDF}, // 925122986
  {0xE5, 0x8E, 0xF9}, // 918233948
  {0xE6, 0x2F, 0x4B}, // 920738953
  {0xE7, 0x0B, 0xB6}, // 924182983
  {0xE6, 0x89, 0x68}, // 922146973
  {0xE5, 0xDF, 0x2B}, // 919487000
  {0xE6, 0xBB, 0xA5}, // 922931946
  {0xE7, 0x79, 0xFB}, // 925905945
  {0xE6, 0xF7, 0xAE}, // 923869995
  {0xE5, 0xFD, 0x2F}, // 919955994
  {0xE6, 0x4D, 0x4F}, // 921207947
  {0xE6, 0xCF, 0x8D}, // 923242981
  {0xE5, 0xAD, 0x0E}, // 918703979
  {0xE7, 0x33, 0xD7}, // 924809998
  {0xE6, 0x9D, 0x91}, // 922461975
  {0xE6, 0x1B, 0x33}, // 920424988
  {0xE6, 0xE3, 0xA5}, // 923556946
  {0xE5, 0xC1, 0x16}, // 919016968
  {0xE7, 0x15, 0xC2}, // 924339966
  {0xE5, 0xF3, 0x33}, // 919799988
  {0xE6, 0x6B, 0x64}, // 921677979
  {0xE7, 0x51, 0xDB}, // 925278992
  {0xE6, 0x39, 0x58}  // 920895996
};

static const __attribute__((progmem)) FRF_ITEM FRF_EU[FREQ_TABLE_LENGTH_EU] =
{
  {0xD9, 0x04, 0x45}, // 868066711
  {0xD9, 0x13, 0x04}, // 868297119
  {0xD9, 0x21, 0xC2}, // 868527466
  {0xD9, 0x0B, 0xA4}, // 868181885
  {0xD9, 0x1A, 0x63}  // 868412292
};

static const __attribute__((progmem)) FRF_ITEM FRF_NZ[FREQ_TABLE_LENGTH_NZ] =
{
  {0xE6, 0x45, 0x0E}, // 921078979
  {0xE7, 0x43, 0xC7}, // 925059021
  {0xE6, 0xF4, 0xAC}, // 923822998
  {0xE6, 0x9C, 0xEE}, // 922452026
  {0xE7, 0xA4, 0x7B}, // 926570007
  {0xE6, 0xC0, 0x31}, // 923002991
  {0xE7, 0xD0, 0x52}, // 927255005
  {0xE7, 0x20, 0x93}, // 924508972
  {0xE6, 0x68, 0x31}, // 921627991
  {0xE7, 0x67, 0x0A}, // 925609985
  {0xE6, 0xDA, 0x5E}, // 923411987
  {0xE7, 0xE1, 0xEC}, // 927530029
  {0xE7, 0x8A, 0x0C}, // 926156982
  {0xE6, 0x82, 0xA0}, // 922041016
  {0xE7, 0x0F, 0x1B}, // 924236023
  {0xE7, 0xBE, 0xE9}, // 926982971
  {0xE6, 0xB7, 0x3B}, // 922862976
  {0xE7, 0x4C, 0x6A}, // 925193970
  {0xE7, 0xFC, 0x5A}, // 927942993
  {0xE6, 0x4D, 0xF4}, // 921218018
  {0xE7, 0x92, 0xD1}, // 926294006
  {0xE6, 0xEB, 0xF8}, // 923687012
  {0xE6, 0x94, 0x39}, // 922315979
  {0xE7, 0xEA, 0xC1}, // 927668030
  {0xE7, 0x29, 0x79}, // 924648010
  {0xE6, 0x5F, 0x7D}, // 921492004
  {0xE7, 0x5E, 0x35}, // 925471985
  {0xE6, 0xC8, 0xC5}, // 923137024
  {0xE7, 0xB6, 0x25}, // 926846008
  {0xE6, 0xA5, 0xB2}, // 922588989
  {0xE6, 0xFD, 0x81}, // 923960999
  {0xE7, 0x6F, 0xCF}, // 925747009
  {0xE6, 0x79, 0xCB}, // 921903015
  {0xE7, 0x9B, 0xB6}, // 926432983
  {0xE7, 0x32, 0x2D}, // 924783997
  {0xE7, 0xC7, 0x7D}, // 927117004
  {0xE6, 0x8B, 0x54}, // 922177002
  {0xE7, 0x81, 0x37}, // 926018982
  {0xE6, 0xD1, 0x89}, // 923273987
  {0xE7, 0x55, 0x60}, // 925333984
  {0xE7, 0xD9, 0x17}, // 927392029
  {0xE6, 0x56, 0xA8}, // 921354004
  {0xE7, 0x06, 0x35}, // 924096985
  {0xE7, 0xAD, 0x2F}, // 926705994
  {0xE6, 0xAE, 0x77}, // 922726013
  {0xE7, 0x3B, 0x12}, // 924922974
  {0xE7, 0xF3, 0x85}, // 927804993
  {0xE6, 0x71, 0x06}, // 921765991
  {0xE7, 0x17, 0xCF}, // 924372009
  {0xE6, 0xE3, 0x12}, // 923547974
  {0xE7, 0x78, 0xA4}  // 925885010
};

static const FRF_ITEM *bandTab[4] = {
  FRF_US,
  FRF_AU,
  FRF_EU,
  FRF_NZ
};

static const uint8_t bandTabLengths[4] = {
  FREQ_TABLE_LENGTH_US,
  FREQ_TABLE_LENGTH_AU,
  FREQ_TABLE_LENGTH_EU,
  FREQ_TABLE_LENGTH_NZ
};

// added these here because upstream removed them
#define REG_TESTAFC        0x71

#define RF_FDEVMSB_9900    0x00
#define RF_FDEVLSB_9900    0xa1

#define RF_AFCLOWBETA_ON   0x20
#define RF_AFCLOWBETA_OFF  0x00 // Default

// Davis VP2 standalone station types
#define STYPE_ISS         0x0 // ISS
#define STYPE_TEMP_ONLY   0x1 // Temperature Only Station
#define STYPE_HUM_ONLY    0x2 // Humidity Only Station
#define STYPE_TEMP_HUM    0x3 // Temperature/Humidity Station
#define STYPE_WLESS_ANEMO 0x4 // Wireless Anemometer Station
#define STYPE_RAIN        0x5 // Rain Station
#define STYPE_LEAF        0x6 // Leaf Station
#define STYPE_SOIL        0x7 // Soil Station
#define STYPE_SOIL_LEAF   0x8 // Soil/Leaf Station
#define STYPE_SENSORLINK  0x9 // SensorLink Station (not supported for the VP2)
#define STYPE_OFF         0xA // No station � OFF
#define STYPE_VUE         0x10 // pseudo station type for the Vue ISS
                               // since the Vue also has a type of 0x0

// Below are known, publicly documented packet types for the VP2 and the Vue.

// VP2 packet types
#define VP2P_UV           0x4 // UV index
#define VP2P_RAINSECS     0x5 // seconds between rain bucket tips
#define VP2P_SOLAR        0x6 // solar irradiation
#define VP2P_TEMP         0x8 // outside temperature
#define VP2P_WINDGUST     0x9 // 10-minute wind gust
#define VP2P_HUMIDITY     0xA // outside humidity
#define VP2P_UNKNOWN      0xC // who knows!
#define VP2P_RAIN         0xE // rain bucket tips counter
#define VP2P_SOIL_LEAF    0xF // soil/leaf station

// Vue packet types
#define VUEP_VCAP         0x2 // supercap voltage
#define VUEP_VSOLAR       0x7 // solar panel voltage

#endif  // DAVISRFM_h
