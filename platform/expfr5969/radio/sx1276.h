#include <stdint.h>
#include <stdbool.h>

#ifndef SX1276_H
#define SX1276_H

#define XTAL_FREQ                                   32000000
#define FREQ_STEP                                   61.03515625

#define RX_BUFFER_SIZE                              256

#define RSSI_OFFSET_LF                              -164
#define RSSI_OFFSET_HF                              -157

/*---------------------------------------------------------------------------*/
typedef struct {
    void (*TxDone)(void);
    void (*TxTimeout)(void);
    void (*RxDone)(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
    void (*RxTimeout)(void);
    void (*RxError)(void);
    void (*FhssChangeChannel)(uint8_t currentChannel);
    void (*CadDone) (int8_t channelActivityDetected);
} RadioEvents_t;

/*---------------------------------------------------------------------------*/
typedef enum
{
    RF_IDLE = 0,
    RF_RX_RUNNING,
    RF_TX_RUNNING,
    RF_CAD,
} radio_state_t;

/*---------------------------------------------------------------------------*/
typedef enum {
    MODEM_FSK,
    MODEM_LORA,
    MODEM_OOK
} radio_modem_t;

/*---------------------------------------------------------------------------*/
typedef struct
{
    int8_t   Power;
    uint32_t Fdev;
    uint32_t Bandwidth;
    uint32_t BandwidthAfc;
    uint32_t Datarate;
    uint16_t PreambleLen;
    bool     FixLen;
    uint8_t  PayloadLen;
    bool     CrcOn;
    bool     IqInverted;
    bool     RxContinuous;
    uint32_t TxTimeout;
} radio_fsk_settings_t;
/*---------------------------------------------------------------------------*/
typedef struct
{
    uint8_t  PreambleDetected;
    uint8_t  SyncWordDetected;
    int8_t   RssiValue;
    int32_t  AfcValue;
    uint8_t  RxGain;
    uint16_t Size;
    uint16_t NbBytes;
    uint8_t  FifoThresh;
    uint8_t  ChunkSize;
} radio_fsk_packet_handler_t;

/*---------------------------------------------------------------------------*/
typedef struct
{
    int8_t   Power;
    uint32_t Fdev;
    uint32_t Bandwidth;
    uint32_t BandwidthAfc;
    uint32_t Datarate;
    uint16_t PreambleLen;
    bool     FixLen;
    uint8_t  PayloadLen;
    bool     CrcOn;
    bool     IqInverted;
    bool     RxContinuous;
    uint32_t TxTimeout;
} radio_ook_settings_t;
/*---------------------------------------------------------------------------*/
typedef struct
{
    uint16_t Size;
    uint16_t NbBytes;
    uint8_t  FifoThresh;
    uint8_t  ChunkSize;
} radio_ook_packet_handler_t;
/*---------------------------------------------------------------------------*/

typedef struct {
    int8_t   Power;
    uint32_t Bandwidth;
    uint32_t Datarate;
    bool     LowDatarateOptimize;
    uint8_t  Coderate;
    uint16_t PreambleLen;
    bool     FixLen;
    uint8_t  PayloadLen;
    bool     CrcOn;
    bool     FreqHopOn;
    uint8_t  HopPeriod;
    bool     IqInverted;
    bool     RxContinuous;
    uint32_t TxTimeout;
} radio_lora_settings_t;

/*---------------------------------------------------------------------------*/
typedef struct
{
    int8_t SnrValue;
    int16_t RssiValue;
    uint8_t Size;
} radio_lora_packet_handler_t;

/*---------------------------------------------------------------------------*/
typedef struct {
    radio_state_t                 State;
    radio_modem_t                 Modem;
    uint32_t                      Channel;
    radio_fsk_settings_t          Fsk;
    radio_ook_settings_t          Ook;
    radio_fsk_packet_handler_t    FskPacketHandler;
    radio_ook_packet_handler_t    OokPacketHandler;
    radio_lora_settings_t         LoRa;
    radio_lora_packet_handler_t   LoRaPacketHandler;
} radio_settings_t;

/*---------------------------------------------------------------------------*/
typedef struct sx1276_struct {
    /*
    Gpio_t        Reset;
    Gpio_t        DIO0;
    Gpio_t        DIO1;
    Gpio_t        DIO2;
    Gpio_t        DIO3;
    Gpio_t        DIO4;
    Gpio_t        DIO5;
    Spi_t         Spi;
    */
    radio_settings_t Settings;
} sx1276_t;

/*---------------------------------------------------------------------------*/
extern sx1276_t sx1276;
/*---------------------------------------------------------------------------*/

/*!
 * ============================================================================
 * Public functions prototypes
 * ============================================================================
 */

void sx1276_init(RadioEvents_t *events ); 
void sx1276_reset();
void sx1276_rxchain_calibration();

void sx1276_set_rxconfig(radio_modem_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous);

void sx1276_set_txconfig(radio_modem_t modem, int8_t power, uint32_t fdev,
                         uint32_t bandwidth, uint32_t datarate,
                         uint8_t coderate, uint16_t preambleLen,
                         bool fixLen, bool crcOn, bool freqHopOn,
                         uint8_t hopPeriod, bool iqInverted, uint32_t timeout);

uint32_t sx1276_get_timeonair(radio_modem_t modem, uint8_t pktLen);

void sx1276_set_channel(uint32_t freq);
void sx1276_set_modem(radio_modem_t modem);
void sx1276_set_opmode(uint8_t opmode);
void sx1276_set_rx(uint32_t timeout);
void sx1276_set_tx(uint32_t timeout);

void sx1276_send(uint8_t *buffer, uint8_t size);
void sx1276_set_sleep(void);

void sx1276_write(uint8_t addr, uint8_t data);
void sx1276_write_buffer(uint8_t addr, uint8_t* buffer, uint8_t len);
void sx1276_write_fifo(uint8_t *data, uint8_t len);

uint8_t sx1276_read(uint8_t addr);
void sx1276_read_buffer(uint8_t addr, uint8_t* buffer, uint8_t len);
void sx1276_read_fifo(uint8_t *data, uint8_t len);
void sx1276_on_dio0irq();
void sx1276_disable_sync_word(void);
/*---------------------------------------------------------------------------*/

#endif
