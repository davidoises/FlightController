#ifndef CC1125_H
#define CC1125_H
// based on https://github.com/Evancw/CC112x/blob/master/examples/Rx_demo.ino
#include <Arduino.h>
#include <SPI.h>


//**************************** pins ******************************************//
#define SS_PIN                      SS//2//SS
#define MISO_PIN                    MISO//19//MISO
#define MISO_PIN                    MOSI//23//MOSI

/*---------------------------[CC11XX - R/W and burst bits]---------------------------*/
#define WRITE_SINGLE_BYTE       0x00
#define WRITE_BURST             0x40
#define READ_SINGLE_BYTE        0x80
#define READ_BURST              0xC0

/*----------------------[CC11XX - config register]----------------------------*/
#define IOCFG3                  0x00    // GDO3 output pin config
#define IOCFG2                  0x01    // GDO2 output pin config
#define IOCFG1                  0x02    // GDO1 output pin config
#define IOCFG0                  0x03    // GDO0 output pin config
#define SYNC3                   0x04    // Sync word configuration
#define SYNC2                   0x05
#define SYNC1                   0x06
#define SYNC0                   0x07
#define SYNC_CFG1               0x08    // Sync word detection
#define SYNC_CFG0               0x09
#define DEVIATION_M             0x0A    // Frequency Deviation
#define MODCFG_DEV_E            0x0B    // Modulation format and frequency deviation
#define DCFILT_CFG              0x0C    // Digital DC Removal
#define PREAMBLE_CFG1           0x0D    // Preamble length config
#define PREAMBLE_CFG0           0x0E
#define FREQ_IF_CFG             0x0F    // RX Mixer frequency
#define IQIC                    0x10    // Digital image channel compensation
#define CHAN_BW                 0x11    // Frequency control word, low byte
#define MDMCFG1                 0x12    // Modem configuration
#define MDMCFG0                 0x13    // Modem configuration
#define SYMBOL_RATE2            0x14    // Modem configuration
#define SYMBOL_RATE1            0x15    // Modem configuration
#define SYMBOL_RATE0            0x16    // Modem configuration
#define AGC_REF                 0x17    // Modem deviation setting
#define AGC_CS_THR              0x18    // Main Radio Cntrl State Machine config
#define AGC_GAIN_ADJUST         0x19    // Main Radio Cntrl State Machine config
#define AGC_CFG3                0x1A    // Main Radio Cntrl State Machine config
#define AGC_CFG2                0x1B    // Frequency Offset Compensation config
#define AGC_CFG1                0x1C    // Bit Synchronization configuration
#define AGC_CFG0                0x1D    // AGC control
#define FIFO_CFG                0x1E    // AGC control
#define DEV_ADDR                0x1F    // AGC control
#define SETTLING_CFG            0x20    // High byte Event 0 timeout
#define FS_CFG                  0x21    // Low byte Event 0 timeout
#define WOR_CFG1                0x22    // Wake On Radio control
#define WOR_CFG0                0x23    // Front end RX configuration
#define WOR_EVENT0_MSB          0x24    // Front end TX configuration
#define WOR_EVENT0_LSB          0x25    // Frequency synthesizer calibration
#define PKT_CFG2                0x26    // Frequency synthesizer calibration
#define PKT_CFG1                0x27    // Frequency synthesizer calibration
#define PKT_CFG0                0x28    // Frequency synthesizer calibration
#define RFEND_CFG1              0x29    // RC oscillator configuration
#define RFEND_CFG0              0x2A    // RC oscillator configuration
#define PA_CFG2                 0x2B    // Frequency synthesizer cal control
#define PA_CFG1                 0x2C    // Production test
#define PA_CFG0                 0x2D    // AGC test
#define PKT_LEN                 0x2E    // Various test settings

/*----------------------[CC11XX - misc]---------------------------------------*/
#define EXTD_REGISTER               0x2F  //Extended register space

/*------------------------[CC11XX-command strobes]----------------------------*/
#define SRES                    0x30        // Reset chip
#define SFSTXON                 0x31        // Enable/calibrate freq synthesizer
#define SXOFF                   0x32        // Turn off crystal oscillator.
#define SCAL                    0x33        // Calibrate freq synthesizer & disable
#define SRX                     0x34        // Enable RX.
#define STX                     0x35        // Enable TX.
#define SIDLE                   0x36        // Exit RX / TX
#define SAFC                    0x37        // AFC adjustment of freq synthesizer
#define SWOR                    0x38        // Start automatic RX polling sequence
#define SPWD                    0x39        // Enter pwr down mode when CSn goes hi
#define SFRX                    0x3A        // Flush the RX FIFO buffer.
#define SFTX                    0x3B        // Flush the TX FIFO buffer.
#define SWORRST                 0x3C        // Reset real time clock.
#define SNOP                    0x3D        // No operation.

/*----------------------[CC11XX - extended registers]----------------------------*/
#define IF_MIX_CFG              0x00
#define TOC_CFG                 0x02
#define FREQ2                   0x0C
#define FREQ1                   0x0D
#define FREQ0                   0x0E
#define IF_ADC0                 0x11
#define FS_DIG1                 0x12
#define FS_DIG0                 0x13
#define FS_CAL2 0x15 // used in manual calibration
#define FS_CAL0                 0x17
#define FS_CHP 0x18 // used in manual calibration
#define FS_DIVTWO               0x19
#define FS_DSM0                 0x1B
#define FS_DVC0                 0x1D
#define FS_PFD                  0x1F
#define FS_PRE                  0x20
#define FS_REG_DIV_CML          0x21
#define FS_SPARE                0x22
#define FS_VCO4 0x23 // used in manual calibration
#define FS_VCO2 0x25 // used in manual calibration
#define XOSC5                   0x32
#define XOSC3                   0x34
#define XOSC1                   0x36
#define MARCSTATE               0x73    // Control state machine state
#define PARTNUM                 0x8F    // Part number
#define VERSION                 0x90    // Current version number
#define NUM_RXBYTES 0xD7
#define FIFO_NUM_RX             0xD9    // Number of available bytes in RXFIFO

/*------------------------[CC11XX - FIFO commands]----------------------------*/
#define STANDARD_FIFO          0x3F    //write single only

class CC1125
{
  private:
    //void commandStrobe(uint8_t instr) {STATUS = spiRead(instr);};
    
    uint8_t STATUS;
    //SPISettings s = SPISettings(4000000, MSBFIRST, SPI_MODE0);
    SPISettings s = SPISettings(8000000, MSBFIRST, SPI_MODE0);
    
  public:
    void commandStrobe(uint8_t instr);
    //uninitalised pointers to SPI objects
    SPIClass * vspi = NULL;
    bool begin(void);
    void set_idle(void);
    void receive(void);
    void transmit(void);
    void tx_payload(uint8_t *txbuffer, uint8_t len);
    void sendPacket(uint8_t *txbuffer, uint8_t len);
    bool get_packet(uint8_t rxbuffer[], uint8_t &pktlen);
    void manualCalibration(void);
    void spiWrite(uint8_t reg, uint8_t data, uint8_t prefix=0);
    uint8_t spiRead(uint8_t reg, uint8_t prefix=0);
};

#endif
