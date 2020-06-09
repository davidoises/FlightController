#include "CC1125.h"

bool CC1125::begin(void)
{
  vspi = new SPIClass(VSPI); // ESP32
  vspi->begin();
  pinMode(SS, OUTPUT); //VSPI SS = 5

  // Reset all CC1125
  commandStrobe(SRES);
  
  // General status info
  commandStrobe(SNOP);
  uint8_t partnum = spiRead(PARTNUM, EXTD_REGISTER); 
  uint8_t version = spiRead(VERSION, EXTD_REGISTER);
  Serial.print("Status:");
  Serial.println(STATUS);
  Serial.print("Partnumber:");
  Serial.println(partnum);
  Serial.print("Version:");
  Serial.println(version);

  
  // Flushing all Tx and Rx fifos
  commandStrobe(SFRX);
  commandStrobe(SFTX);
  
  
  // Start register configuration. Based on SmartRF studio
  spiWrite(IOCFG3, 0xB0); // Analog pad, non inverted, HIGHZ
  spiWrite(IOCFG2, 0x06); // Digital pad, non inverted, PKT_SYNC_RXTX = Asserted when sync word has been received and de-asserted at the end of the packet.
  spiWrite(IOCFG1, 0xB0); // Analog pad, non inverted, HIGHZ
  spiWrite(IOCFG0, 0x40); // 0x40 Digital pad, inverted, RXFIFO_THR = Asserted when the RX FIFO is filled above FIFO_CFG.FIFO_THR. --> changed to 0x27 = Digital pad, non inverted,RX FIFO OVERFLOW/underflow
  spiWrite(SYNC_CFG1, 0x08); // Original smart rf studio value was 0x08 /////////// modified to 0x02
  spiWrite(DEVIATION_M, 0x47);
  spiWrite(MODCFG_DEV_E, 0x0D);
  spiWrite(DCFILT_CFG, 0x15);
  spiWrite(PREAMBLE_CFG1, 0x18);
  spiWrite(FREQ_IF_CFG, 0x2E);
  spiWrite(IQIC, 0x00);
  spiWrite(CHAN_BW, 0x02);
  spiWrite(MDMCFG0, 0x05);
  spiWrite(SYMBOL_RATE2, 0x94);
  spiWrite(SYMBOL_RATE1, 0x7A);
  spiWrite(SYMBOL_RATE0, 0xE1);
  spiWrite(AGC_REF, 0x3C);
  spiWrite(AGC_CS_THR, 0xEF);
  spiWrite(AGC_CFG1, 0xA9);
  spiWrite(AGC_CFG0, 0xC0);
  spiWrite(FIFO_CFG, 0x00); // FIFO Threshold is 0, interrupt asserted at 1 byte in RX FIFO
  spiWrite(SETTLING_CFG, 0x03); // this is mine ///////////////////////// this is to avoid automatic calibration and may be optional
  spiWrite(FS_CFG, 0x12);
  spiWrite(PKT_CFG0, 0x20); // Variable packet length mode. Length received in the first byte after sync word
  spiWrite(RFEND_CFG1, 0x3F); // this is mine ///////////////////////// This allows automatic RX mode activation afet message reception
  spiWrite(PA_CFG0, 0x79);
  spiWrite(PKT_LEN, 0xFF); // Maximum allowed packet length

  spiWrite(IF_MIX_CFG, 0x00, EXTD_REGISTER);
  spiWrite(TOC_CFG, 0x0A, EXTD_REGISTER);
  /* 868MHz 
  spiWrite(FREQ2, 0x56, EXTD_REGISTER);
  spiWrite(FREQ1, 0xCC, EXTD_REGISTER);
  spiWrite(FREQ0, 0xCC, EXTD_REGISTER);
  */
  /* 915MHz */
  spiWrite(FREQ2, 0x5B, EXTD_REGISTER);
  spiWrite(FREQ1, 0x80, EXTD_REGISTER);
  spiWrite(FREQ0, 0x00, EXTD_REGISTER);
  
  spiWrite(IF_ADC0, 0x05, EXTD_REGISTER);
  spiWrite(FS_DIG1, 0x00, EXTD_REGISTER);
  spiWrite(FS_DIG0, 0x5F, EXTD_REGISTER);
  spiWrite(FS_CAL0, 0x0E, EXTD_REGISTER);
  spiWrite(FS_DIVTWO, 0x03, EXTD_REGISTER);
  spiWrite(FS_DSM0, 0x33, EXTD_REGISTER);
  spiWrite(FS_DVC0, 0x17, EXTD_REGISTER);
  spiWrite(FS_PFD, 0x50, EXTD_REGISTER);
  spiWrite(FS_PRE, 0x6E, EXTD_REGISTER);
  spiWrite(FS_REG_DIV_CML, 0x14, EXTD_REGISTER);
  spiWrite(FS_SPARE, 0xAC, EXTD_REGISTER);
  spiWrite(XOSC5, 0x0E, EXTD_REGISTER);
  spiWrite(XOSC3, 0xC7, EXTD_REGISTER);
  spiWrite(XOSC1, 0x07, EXTD_REGISTER);
}

#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX 1
#define FS_CHP_INDEX 2
void CC1125::manualCalibration(void)
{
  uint8_t original_fs_cal2;
  uint8_t calResults_for_vcdac_start_high[3];
  uint8_t calResults_for_vcdac_start_mid[3];
  uint8_t marcstate;
  uint8_t writeByte;

  // 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
  spiWrite(FS_VCO2, 0x00, EXTD_REGISTER);

  // 2) Start with high VCDAC (original VCDAC_START + 2):
  original_fs_cal2 = spiRead(FS_CAL2, EXTD_REGISTER);
  spiWrite(FS_CAL2, original_fs_cal2 + VCDAC_START_OFFSET, EXTD_REGISTER);

  // 3) Calibrate and wait for calibration to be done
  //   (radio back in IDLE state)
  commandStrobe(SCAL);

  do {
      marcstate = spiRead(MARCSTATE, EXTD_REGISTER);
  } while (marcstate != 0x41);

  // 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with
  //    high VCDAC_START value
  calResults_for_vcdac_start_high[FS_VCO2_INDEX] = spiRead(FS_VCO2, EXTD_REGISTER);
  calResults_for_vcdac_start_high[FS_VCO4_INDEX] = spiRead(FS_VCO4, EXTD_REGISTER);
  calResults_for_vcdac_start_high[FS_CHP_INDEX] = spiRead(FS_CHP, EXTD_REGISTER);

  // 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
  spiWrite(FS_VCO2, 0x00, EXTD_REGISTER);

  // 6) Continue with mid VCDAC (original VCDAC_START):
  spiWrite(FS_CAL2, original_fs_cal2, EXTD_REGISTER);

  // 7) Calibrate and wait for calibration to be done
  //   (radio back in IDLE state)
  commandStrobe(SCAL);
  
  do {
      marcstate = spiRead(MARCSTATE, EXTD_REGISTER);
  } while (marcstate != 0x41);

  // 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with
  //    mis VCDAC_START value
  calResults_for_vcdac_start_mid[FS_VCO2_INDEX] = spiRead(FS_VCO2, EXTD_REGISTER);
  calResults_for_vcdac_start_mid[FS_VCO4_INDEX] = spiRead(FS_VCO4, EXTD_REGISTER);
  calResults_for_vcdac_start_mid[FS_CHP_INDEX] = spiRead(FS_CHP, EXTD_REGISTER);

  // 9) Write back highest FS_VCO2 and corresponding FS_VCO
  //    and FS_CHP result
  if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] > calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) 
  {
      spiWrite(FS_VCO2, calResults_for_vcdac_start_high[FS_VCO2_INDEX], EXTD_REGISTER);
      spiWrite(FS_VCO4, calResults_for_vcdac_start_high[FS_VCO4_INDEX], EXTD_REGISTER);
      spiWrite(FS_CHP, calResults_for_vcdac_start_high[FS_CHP_INDEX], EXTD_REGISTER);
  } else {
      spiWrite(FS_VCO2, calResults_for_vcdac_start_mid[FS_VCO2_INDEX], EXTD_REGISTER);
      spiWrite(FS_VCO4, calResults_for_vcdac_start_mid[FS_VCO4_INDEX], EXTD_REGISTER);
      spiWrite(FS_CHP, calResults_for_vcdac_start_mid[FS_CHP_INDEX], EXTD_REGISTER);
  }
}

void CC1125::set_idle(void)
{
  uint8_t marcstate = 0xFF;                //set unknown/dummy state value
  commandStrobe(SIDLE);                   //sets to idle first. must be in
  do
  {
    marcstate = (spiRead(MARCSTATE, EXTD_REGISTER) & 0x1F);   //read out state of cc112x to be sure in IDDLE
  }while(marcstate != 0x01); //0x0D = RX, 0x01 = IDLE, 0x13 = TX
}

//---------------------------[Transmit mode]-------------------------------------
void CC1125::transmit(void)
{
  uint8_t marcstate = 0xFF;             //set unknown/dummy state value
  set_idle();
  commandStrobe(STX); 
  do
  {
    marcstate = (spiRead(MARCSTATE, EXTD_REGISTER) & 0x1F);   //read out state of cc112x to be sure in IDDLE
  }while(marcstate != 0x01); //0x0D = RX, 0x01 = IDLE, 0x13 = TX

  commandStrobe(SFTX); //flush the Tx_fifo conten
  //delayMicroseconds(100);
}

void CC1125::tx_payload(uint8_t *txbuffer, uint8_t len)
{
  for(int i = 0; i < len; i++)
  {
    spiWrite(STANDARD_FIFO, txbuffer[i]);
  }
}

void CC1125::sendPacket(uint8_t *txbuffer, uint8_t len)
{
  
  tx_payload(txbuffer, len);
  transmit();
  receive();
}

//---------------------------[receive mode]-------------------------------------
void CC1125::receive(void)
{
  uint8_t marcstate = 0xFF;             //set unknown/dummy state value
  set_idle();
  commandStrobe(SFRX);
  commandStrobe(SRX);
  do
  {
    marcstate = (spiRead(MARCSTATE, EXTD_REGISTER) & 0x1F);   //read out state of cc112x to be sure in RX
  }while(marcstate != 0x0D); //0x0D = RX
}

bool CC1125::get_packet(uint8_t rxbuffer[], uint8_t &pktlen)
{
  uint8_t num_bytes = spiRead(NUM_RXBYTES, EXTD_REGISTER);
  pktlen = 0;
  if(num_bytes != 0)
  {
    
    pktlen = spiRead(STANDARD_FIFO | READ_SINGLE_BYTE) - 1; // size ignoring the first byte (destination address)
    uint8_t address = spiRead(STANDARD_FIFO | READ_SINGLE_BYTE);

    /*
    Serial.print("FIFO Bytes: ");
    Serial.println(num_bytes);
    Serial.print("Packet Length: ");
    Serial.println(pktlen);
    Serial.print("RD destination ddress: ");
    Serial.println(address, HEX);*/
    
    if(address == 0xAA)
    {
      uint8_t temp_buffer[128] = {0};
      // store the full packet in a temporal array
      for(int i = 0; i < pktlen; i++)
      {
        temp_buffer[i] = spiRead(STANDARD_FIFO | READ_SINGLE_BYTE);
      }

      // Get status bytes, which cotain CRC
      uint8_t status_bytes[2] = {0};
      status_bytes[0] = spiRead(STANDARD_FIFO | READ_SINGLE_BYTE);
      status_bytes[1] = spiRead(STANDARD_FIFO | READ_SINGLE_BYTE);

      // Flush extra bytes to avoid overflows
      uint8_t extra_bytes = spiRead(NUM_RXBYTES, EXTD_REGISTER);
      for(int i = 0; i < extra_bytes; i++)
      {
        spiRead(STANDARD_FIFO | READ_SINGLE_BYTE);
      }

      // Check CRC
      if(!(status_bytes[1] & 0x80)) // This means CRC is ok
      {
        //Serial.println("Bad CRC");
        pktlen = 0;
        return false;
      }

      // if the CRC check was succesfull copy the packet into the correct array
      for(int i = 0; i < pktlen; i++)
      {
        rxbuffer[i] = temp_buffer[i];
      }

      return true;
    }

    pktlen = 0;
    return false;
  }
  
  return false;
}

void CC1125::commandStrobe(uint8_t instr)
{
  digitalWrite(SS_PIN, LOW);
  vspi->beginTransaction(s);
  STATUS = vspi->transfer(instr | WRITE_SINGLE_BYTE);
  while(digitalRead(MISO_PIN) == 1);
  vspi->endTransaction();
  digitalWrite(SS_PIN, HIGH);
  delayMicroseconds(100);
}

void CC1125::spiWrite(uint8_t reg, uint8_t data, uint8_t prefix)
{
  digitalWrite(SS_PIN, LOW);
  while(digitalRead(MISO_PIN) == 1);
  vspi->beginTransaction(s);
  if ( prefix ) {
    vspi->transfer(prefix | WRITE_SINGLE_BYTE);
    vspi->transfer(reg);
  } else
  {
    vspi->transfer(reg | WRITE_SINGLE_BYTE);
  }
  vspi->transfer(data);
  vspi->endTransaction();
  digitalWrite(SS_PIN, HIGH);
}

uint8_t CC1125::spiRead(uint8_t reg, uint8_t prefix)
{
  digitalWrite(SS_PIN, LOW);
  while(digitalRead(MISO_PIN) == 1);
  vspi->beginTransaction(s);

  if ( prefix ) {
    vspi->transfer(prefix | READ_SINGLE_BYTE);
    vspi->transfer(reg);
  } else
  {
    vspi->transfer(reg | READ_SINGLE_BYTE);
  }
  uint8_t spi_instr = vspi->transfer(0xFF);
  vspi->endTransaction();
  digitalWrite(SS_PIN, HIGH);
  return spi_instr;
}
