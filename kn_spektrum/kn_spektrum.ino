#include <TinyPinChange.h>
#include <SoftSerial.h>
#include <util/atomic.h>
#include <avr/io.h>
#include "iface_nrf24l01.h"

// ############ Wiring ################
//SPI Comm.pins with nRF24L01
#define MOSI_pin  1
#define SCK_pin   3
#define CS_pin    2
#define MISO_pin  4
#define CE_pin    5

// SPI outputs
#define MOSI_on PORTB |= 0b00000010
#define MOSI_off PORTB &= ~0b00000010
#define SCK_on PORTB |= 0b00001000
#define SCK_off PORTB &= ~0b00001000
#define CS_on PORTB |= 0b00000100
#define CS_off PORTB &= ~0b00000100
//#define CE_on PORTB |= 0b00100000
//#define CE_off PORTB &= ~0b00100000
#define CE_on asm volatile ("nop\n\t")  //CE not used, tied high
#define CE_off asm volatile ("nop\n\t")
// SPI input
#define  MISO_on (PINB & 0b00010000)

#define RF_POWER TX_POWER_5mW //TX_POWER_80mW

#define S_A 0x00
#define S_E 0x08
#define S_T 0x10
#define S_R 0x18
#define S_G 0x20
#define S_AUX1 0x28
#define S_AUX2 0x30
#define S_AUX3 0x38
#define S_AUX4 0x40
#define S_AUX5 0x48
#define S_AUX6 0x50
#define S_AUX7 0x58

bool bind = false, use1mbps = true, started = false;
byte hopping_channels[4], data[16], spektrum[16];
byte tx_addr[5];
byte ch_index = 2, ch_indexl = 1;
unsigned long timing = 0;
int t, a, e, r, deadline = 13;
byte tt, ta, te, tr, sw, th, id, dr, td;
byte ttl = 0, tal = 0, tel = 0, trl = 0, tdl = 2;
unsigned int plost = 0;
int tempval = 0, tempch = 0;

SoftSerial mySerial(-1, 0);

void cleanArray(byte *arr, byte num) {
  for (byte i = 0; i < num; i++) {
    arr[i] = 0;
  }
}

void setup() {
  mySerial.begin(115200);
  delay(500);

  pinMode(MOSI_pin, OUTPUT);
  pinMode(SCK_pin, OUTPUT);
  pinMode(CS_pin, OUTPUT);
  //pinMode(CE_pin, OUTPUT);
  pinMode(MISO_pin, INPUT);
  MOSI_on;
  SCK_on;
  CS_on;
  //CE_on;
  delay(1);
  MOSI_off;
  SCK_off;
  //CS_off;
  delay(250);

  NRF24L01_Reset();

  cleanArray(tx_addr, 5);
  cleanArray(hopping_channels, 4);

  for (byte i = 0; i < 2; i++) {
    NRF24L01_Activate(0x73);
    NRF24L01_Reset();
    NRF24L01_Initialize();

    NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP)); //enable Crc / crc endcoding 2bytes / power up

    NRF24L01_SetTxRxMode(RX_EN);  //reset and switch to rx mode

    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);    //disable auto acknowledgement on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01); //enable rx on data pipe 0
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);  //rx-tx address width of 5bytes
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0); //auto-retransmit delay 250us
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);  //clear interupts (rx, tx and max number of retransmit)
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, 0x20);  //32bytes rx payload in data pipe 0

    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 1);            //enable dynamic payload length on data pipe 0
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, _BV(NRF2401_1D_EN_DPL)); //enable dynamic payload length

    NRF24L01_SetPower(RF_POWER);          //set rf power                    //

    NRF24L01_SetBitrate(NRF24L01_BR_1M);
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, (const byte*)"KNDZK", 5);
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, 83);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x40);
  }

  while (!bind) {
    cleanArray(data, 16);
    if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & 0x40) {
      NRF24L01_ReadPayload(data, 16);
      NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x40);
      if (data[0] == 'K' && data[1] == 'N' && data[2] == 'D' && data[3] == 'Z') {
        bind = true;
        tx_addr[0] = data[4];
        tx_addr[1] = data[5];
        tx_addr[2] = data[6];
        tx_addr[3] = data[7];
        tx_addr[4] = 'K';
        hopping_channels[0] = data[8];
        hopping_channels[1] = data[9];
        hopping_channels[2] = data[10];
        hopping_channels[3] = data[11];
        if (data[15] == 1) {
          use1mbps = true;
        } else {
          use1mbps = false;
        }
      }
    }
  }

  if (use1mbps) {
    NRF24L01_SetBitrate(NRF24L01_BR_1M);
  } else {
    NRF24L01_SetBitrate(NRF24L01_BR_250K);
  }
  NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, tx_addr, 5);
  cleanArray(data, 16);
}

void loop() {
  unsigned long m = millis();
  if (!started) {
    timing = m;
  }

  if ((m - timing) > deadline) {
    ch_index++;
    timing = m;
    plost++;
    if (deadline == 13) {
      deadline = 10;
    }
  }

  if (ch_index >= 4) {
    ch_index = 0;
  }

  if (ch_index != ch_indexl) {
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, hopping_channels[ch_index]);
  }
  ch_indexl = ch_index;

  if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & 0x40) {
    NRF24L01_ReadPayload(data, 16);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x40);

    t = data[0] << 8;
    t += data[1];
    a = data[2] << 8;
    a += data[3];
    e = data[4] << 8;
    e += data[5];
    r = data[6] << 8;
    r += data[7];
    tt = data[8];
    ta = data[9];
    te = data[10];
    tr = data[11];
    sw = data[12];
    th = bitRead(sw, 1);
    id = bitRead(sw, 2);
    dr = bitRead(sw, 0);
    td = bitRead(sw, 6);
    toSpektrum();

    ch_index++;
    timing = millis();
    started = true;
    deadline = 13;
  }
}

void toSpektrum() {
  int plostc = plost >> 1;
  spektrum[0] = plostc >> 8; spektrum[1] = plostc & 0xff;
  spektrum[2] = S_T | t >> 7; spektrum[3] = (t << 1) & 0xff;
  spektrum[4] = S_A | a >> 7; spektrum[5] = (a << 1) & 0xff;
  spektrum[6] = S_E | e >> 7; spektrum[7] = (e << 1) & 0xff;
  spektrum[8] = S_R | r >> 7; spektrum[9] = (r << 1) & 0xff;
  spektrum[10] = S_G | (0x07 * th); spektrum[11] = 0xff * th;
  spektrum[12] = S_AUX1 | (0x07 * id); spektrum[13] = 0xff * id;

  if (td != tdl) {
    tempch = S_AUX3; tempval = 2047 * td;
    tdl = td;
  } else if (tt != ttl) {
    tempval = trimToSpektrum(tt);
    tempch = S_AUX4;
    ttl = tt;
  } else if (ta != tal) {
    tempval = trimToSpektrum(ta);
    tempch = S_AUX5;
    tal = ta;
  } else if (te != tel) {
    tempval = trimToSpektrum(te);
    tempch = S_AUX6;
    tel = te;
  } else if (tr != trl) {
    tempval = trimToSpektrum(tr);
    tempch = S_AUX7;
    trl = tr;
  } else {
    tempch = S_AUX2; tempval = 2047 * dr;
    ttl = 0; tal = 0; tel = 0; trl = 0; tdl = 2;
  }

  spektrum[14] = tempch | tempval >> 8; spektrum[15] = tempval & 0xff;
  mySerial.write(spektrum, 16);
}

int trimToSpektrum(byte rawTrim) {
  int rt = (rawTrim * 201) - 19076;
  rt = (rt > 2047) ? 2047 : rt;
  rt = (rt < 0) ? 0 : rt;
  return rt;
}
