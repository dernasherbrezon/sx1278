// Copyright 2022 Andrey Rodionov <dernasherbrezon@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//         http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef sx127x_h
#define sx127x_h

#ifdef __cplusplus
extern "C" {
#endif

// optional standard esp-idf configuration
#ifdef IDF_VER
#include "sdkconfig.h"
#endif
#include <stdbool.h>
#include <stdint.h>

#define MAX_PACKET_SIZE 255
#define MAX_PACKET_SIZE_FSK_FIXED 2047
#define MAX_NUMBER_OF_REGISTERS 0x71

#define SX127X_OK 0                      /*!< esp_err_t value indicating success (no error) */
#define SX127X_ERR_INVALID_ARG 0x102     /*!< Invalid argument */
#define SX127X_ERR_INVALID_STATE 0x103   /*!< Invalid state. Most likely function is not applicable for the selected modem */
#define SX127X_ERR_NOT_FOUND 0x105       /*!< Requested resource not found */
#define SX127X_ERR_INVALID_VERSION 0x10A /*!< Version was invalid */

#ifndef CONFIG_SX127X_MAX_PACKET_SIZE
#define CONFIG_SX127X_MAX_PACKET_SIZE MAX_PACKET_SIZE_FSK_FIXED
#endif

/*
 * This structure used to change mode
 */
typedef enum {
  SX127x_MODE_SLEEP = 0b00000000,      // SLEEP
  SX127x_MODE_STANDBY = 0b00000001,    // STDBY
  SX127x_MODE_FSTX = 0b00000010,       // Frequency synthesis TX
  SX127x_MODE_TX = 0b00000011,         // Transmit
  SX127x_MODE_FSRX = 0b00000100,       // Frequency synthesis RX
  SX127x_MODE_RX_CONT = 0b00000101,    // Receive continuous
  SX127x_MODE_RX_SINGLE = 0b00000110,  // Receive single
  SX127x_MODE_CAD = 0b00000111         // Channel activity detection
} sx127x_mode_t;

typedef enum {
  SX127x_MODULATION_LORA = 0b10000000,
  SX127x_MODULATION_FSK = 0b00000000, // default
  SX127x_MODULATION_OOK = 0b00100000
} sx127x_modulation_t;

/**
 * @brief Size of each decrement of the RSSI threshold in the OOK demodulator
 *
 */
typedef enum {
  SX127X_0_5_DB = 0b00000000,  // 0.5db (default)
  SX127X_1_0_DB = 0b00000001,  // 1.0db
  SX127X_1_5_DB = 0b00000010,  // 1.5db
  SX127X_2_0_DB = 0b00000011,  // 2.0db
  SX127X_3_0_DB = 0b00000100,  // 3.0db
  SX127X_4_0_DB = 0b00000101,  // 4.0db
  SX127X_5_0_DB = 0b00000110,  // 5.0db
  SX127X_6_0_DB = 0b00000111   // 6.0db
} sx127x_ook_peak_thresh_step_t;

/**
 * @brief Static offset added to the threshold in average mode in order to reduce glitching activity (OOK only)
 *
 */
typedef enum {
  SX127X_0_DB = 0b00000000,
  SX127X_2_DB = 0b00000100,
  SX127X_4_DB = 0b00001000,
  SX127X_6_DB = 0b00001100
} sx127x_ook_avg_offset_t;

/**
 * @brief Filter coefficients in average mode of the OOK demodulator
 *
 */
typedef enum {
  SX127X_32_PI = 0b00000000,  // chip rate / 32.π
  SX127X_8_PI = 0b00000001,   // chip rate / 8.π
  SX127X_4_PI = 0b00000010,   // chip rate / 4.π (default)
  SX127X_2_PI = 0b00000011    // chip rate / 2.π
} sx127x_ook_avg_thresh_t;

/**
 * @brief Period of decrement of the RSSI threshold in the OOK demodulator
 *
 */
typedef enum {
  SX127X_1_1_CHIP = 0b00000000,  // once per chip (default)
  SX127X_1_2_CHIP = 0b00100000,  // once every 2 chips
  SX127X_1_4_CHIP = 0b01000000,  // once every 4 chips
  SX127X_1_8_CHIP = 0b01100000,  // once every 8 chips
  SX127X_2_1_CHIP = 0b10000000,  // twice in each chip
  SX127X_4_1_CHIP = 0b10100000,  // 4 times in each chip
  SX127X_8_1_CHIP = 0b11000000,  // 8 times in each chip
  SX127X_16_1_CHIP = 0b11100000  // 16 times in each chip
} sx127x_ook_peak_thresh_dec_t;

typedef enum {
  SX127X_RX_TRIGGER_NONE = 0b00000000,
  SX127X_RX_TRIGGER_RSSI = 0b00000001,
  SX127X_RX_TRIGGER_PREAMBLE = 0b00000110,  // default
  SX127X_RX_TRIGGER_RSSI_PREAMBLE = 0b00000111
} sx127x_rx_trigger_t;

typedef enum {
  SX127X_PREAMBLE_55 = 0b00100000,
  SX127X_PREAMBLE_AA = 0b00000000
} sx127x_preamble_type_t;

typedef enum {
  SX127X_2 = 0b00000000,
  SX127X_4 = 0b00000001,
  SX127X_8 = 0b00000010,
  SX127X_16 = 0b00000011,
  SX127X_32 = 0b00000100,
  SX127X_64 = 0b00000101,
  SX127X_128 = 0b00000110,
  SX127X_256 = 0b00000111
} sx127x_rssi_smoothing_t;

typedef enum {
  SX127X_NRZ = 0b00000000,
  SX127X_MANCHESTER = 0b00100000,
  SX127X_SCRAMBLED = 0b01000000  // LFSR Polynomial =X9 + X5 + 1
} sx127x_packet_encoding_t;

typedef enum {
  SX127X_CRC_NONE = 0b00001000,   // CrcOff + Do not clear FIFO
  SX127X_CRC_CCITT = 0b00011000,  // CrcOn + CrcWhiteningType. Polynomial X16 + X12 + X5 + 1 Seed Value 0x1D0F
  SX127X_CRC_IBM = 0b00011001     // CrcOn + CrcWhiteningType. Polynomial X16 + X15 + X2 + 1 Seed Value 0xFFFF
} sx127x_crc_type_t;

typedef enum {
  SX127X_FIXED = 0b00000000,
  SX127X_VARIABLE = 0b10000000
} sx127x_packet_format_t;

typedef enum {
  SX127X_FILTER_NONE = 0b00000000,
  SX127X_FILTER_NODE_ADDRESS = 0b00000010,
  SX127X_FILTER_NODE_AND_BROADCAST = 0b00000100
} sx127x_address_filtering_t;

typedef enum {
  SX127x_LNA_GAIN_G1 = 0b00100000,  // Maximum gain
  SX127x_LNA_GAIN_G2 = 0b01000000,
  SX127x_LNA_GAIN_G3 = 0b01100000,
  SX127x_LNA_GAIN_G4 = 0b10000000,
  SX127x_LNA_GAIN_G5 = 0b10100000,
  SX127x_LNA_GAIN_G6 = 0b11000000,   // Minimum gain
  SX127x_LNA_GAIN_AUTO = 0b00000000  // Automatic. See 5.5.3. for details
} sx127x_gain_t;

typedef enum {
  SX127X_FSK_SHAPING_NONE = 0b00000000,  // no shaping
  SX127X_BT_1_0 = 0b00100000,            // Gaussian filter BT = 1.0
  SX127X_BT_0_5 = 0b01000000,            // Gaussian filter BT = 0.5
  SX127X_BT_0_3 = 0b01100000             // Gaussian filter BT = 0.3
} sx127x_fsk_data_shaping_t;

typedef enum {
  SX127X_OOK_SHAPING_NONE = 0b00000000,  // no shaping (default)
  SX127X_1_BIT_RATE = 0b00100000,        // filtering with fcutoff = bit_rate
  SX127X_2_BIT_RATE = 0b01000000         // filtering with fcutoff = 2*bit_rate (for bit_rate < 125 kb/s)
} sx127x_ook_data_shaping_t;

typedef enum {
  SX127X_PA_RAMP_1 = 0b00000000,   // 3.4 ms
  SX127X_PA_RAMP_2 = 0b00000001,   // 2 ms
  SX127X_PA_RAMP_3 = 0b00000010,   // 1 ms
  SX127X_PA_RAMP_4 = 0b00000011,   // 500 us
  SX127X_PA_RAMP_5 = 0b00000100,   // 250 us
  SX127X_PA_RAMP_6 = 0b00000101,   // 125 us
  SX127X_PA_RAMP_7 = 0b00000110,   // 100 us
  SX127X_PA_RAMP_8 = 0b00000111,   // 62 us
  SX127X_PA_RAMP_9 = 0b00001000,   // 50 us
  SX127X_PA_RAMP_10 = 0b00001001,  // Default. 40 us
  SX127X_PA_RAMP_11 = 0b00001010,  // 31 us
  SX127X_PA_RAMP_12 = 0b00001011,  // 25 us
  SX127X_PA_RAMP_13 = 0b00001100,  // 20 us
  SX127X_PA_RAMP_14 = 0b00001101,  // 15 us
  SX127X_PA_RAMP_15 = 0b00001110,  // 12 us
  SX127X_PA_RAMP_16 = 0b00001111   // 10 us
} sx127x_pa_ramp_t;

/**
 * @brief Signal bandwidth.
 *
 * @note In the lower band (169MHz), signal bandwidths 8&9 (250k and 500k) are not supported
 *
 */
typedef enum {
  SX127x_BW_7800 = 0b00000000,
  SX127x_BW_10400 = 0b00010000,
  SX127x_BW_15600 = 0b00100000,
  SX127x_BW_20800 = 0b00110000,
  SX127x_BW_31250 = 0b01000000,
  SX127x_BW_41700 = 0b01010000,
  SX127x_BW_62500 = 0b01100000,
  SX127x_BW_125000 = 0b01110000, // default
  SX127x_BW_250000 = 0b10000000,
  SX127x_BW_500000 = 0b10010000
} sx127x_bw_t;

typedef enum {
  SX127x_CR_4_5 = 0b00000010, // default
  SX127x_CR_4_6 = 0b00000100,
  SX127x_CR_4_7 = 0b00000110,
  SX127x_CR_4_8 = 0b00001000
} sx127x_cr_t;

/**
 * @brief SF rate (expressed as a base-2 logarithm)
 *
 */
typedef enum {
  SX127x_SF_6 = 0b01100000,   // 64 chips / symbol
  SX127x_SF_7 = 0b01110000,   // 128 chips / symbol
  SX127x_SF_8 = 0b10000000,   // 256 chips / symbol
  SX127x_SF_9 = 0b10010000,   // 512 chips / symbol
  SX127x_SF_10 = 0b10100000,  // 1024 chips / symbol
  SX127x_SF_11 = 0b10110000,  // 2048 chips / symbol
  SX127x_SF_12 = 0b11000000   // 4096 chips / symbol
} sx127x_sf_t;

/**
 * @brief Imlicit header for TX or RX.
 *
 */
typedef struct {
  uint8_t length;           // payload length. Cannot be more than 256 bytes.
  bool enable_crc;          // Enable or disable CRC.
  sx127x_cr_t coding_rate;  // Coding rate
} sx127x_implicit_header_t;

typedef struct {
  bool enable_crc;
  sx127x_cr_t coding_rate;
} sx127x_tx_header_t;

/**
 * @brief Type of interrupt. Same interrupts can happen on different digital pins.
 *
 */
typedef enum {
  SX127x_DIO0_RX_DONE = 0b00000000,              // Packet reception complete
  SX127x_DIO0_TX_DONE = 0b01000000,              // FIFO Payload transmission complete
  SX127x_DIO0_CAD_DONE = 0b10000000,             // CAD complete
  SX127x_DIO1_RXTIMEOUT = 0b00000000,            // RX timeout interrupt. Used in RX single mode
  SX127x_DIO1_FHSS_CHANGE_CHANNEL = 0b00010000,  // FHSS change channel
  SX127x_DIO1_CAD_DETECTED = 0b00100000,         // Valid Lora signal detected during CAD operation
  SX127x_DIO2_FHSS_CHANGE_CHANNEL = 0b00000000,  // FHSS change channel on digital pin 2
  SX127x_DIO3_CAD_DONE = 0b00000000,             // CAD complete on digital pin 3
  SX127x_DIO3_VALID_HEADER = 0b00000001,         // Valid header received in Rx
  SX127x_DIO3_PAYLOAD_CRC_ERROR = 0b00000010,    // Payload CRC error
} sx127x_dio_mapping1_t;

typedef enum {
  SX127x_FSK_DIO0_PAYLOAD_READY = 0b00000000,
  SX127x_FSK_DIO0_PACKET_SENT = 0b00000000,
  SX127x_FSK_DIO0_CRC_OK = 0b01000000,
  SX127x_FSK_DIO1_FIFO_LEVEL = 0b00000000,
  SX127x_FSK_DIO1_FIFO_EMPTY = 0b00010000,
  SX127x_FSK_DIO1_FIFO_FULL = 0b00100000,
  SX127x_FSK_DIO2_FIFO_FULL = 0b00000000,
  SX127x_FSK_DIO2_SYNCADDRESS = 0b00001100,
  SX127x_FSK_DIO3_FIFO_EMPTY = 0b00000000
} sx127x_fsk_ook_dio_mapping1_t;

/**
 * @brief Type of interrupt. Same interrupts can happen on different digital pins.
 *
 */
typedef enum {
  SX127x_DIO4_CAD_DETECTED = 0b00000000,  // Valid Lora signal detected during CAD operation
  SX127x_DIO4_PLL_LOCK = 0b01000000,      // PLL lock
  SX127x_DIO5_MODE_READY = 0b00000000,    // Mode ready
  SX127x_DIO5_CLK_OUT = 0b01000000        // clock out
} sx127x_dio_mapping2_t;

typedef enum {
  SX127x_FSK_DIO4_TEMP_CHANGE = 0b00000000,
  SX127x_FSK_DIO4_PLL_LOCK = 0b01000000,
  SX127x_FSK_DIO4_TIMEOUT = 0b10000000,
  SX127x_FSK_DIO4_PREAMBLE_DETECT = 0b11000000,
  SX127x_FSK_DIO5_CLK_OUT = 0b00000000,
  SX127x_FSK_DIO5_PLL_LOCK = 0b00010000,
  SX127x_FSK_DIO5_DATA = 0b00100000,
  SX127x_FSK_DIO5_MODE_READY = 0b00110000
} sx127x_fsk_ook_dio_mapping2_t;

/**
 * @brief sx127x chip has 2 pins for TX. Based on the pin below chip can produce different max power.
 *
 */
typedef enum {
  SX127x_PA_PIN_RFO = 0b00000000,   // RFO pin. Output power is limited to +14 dBm.
  SX127x_PA_PIN_BOOST = 0b10000000  // PA_BOOST pin. Output power is limited to +20 dBm
} sx127x_pa_pin_t;

/**
 * @brief Wrapper around abstract spi device.
 */
typedef struct {
  void *spi_device;
#ifndef CONFIG_SX127X_DISABLE_SPI_CACHE
  uint8_t shadow_registers[MAX_NUMBER_OF_REGISTERS];
  uint8_t shadow_registers_sync[MAX_NUMBER_OF_REGISTERS];
#endif
} shadow_spi_device_t;

/**
 * @brief Device handle
 *
 */
typedef struct sx127x_t sx127x;

struct sx127x_t {
  shadow_spi_device_t spi_device;

  bool use_implicit_header;

  void* user_context;

  void (*rx_callback)(sx127x *, uint8_t *, uint16_t);

  void (*tx_callback)(sx127x *);

  void (*cad_callback)(sx127x *, int);

  uint8_t packet[CONFIG_SX127X_MAX_PACKET_SIZE];
  uint16_t expected_packet_length;
  uint16_t fsk_ook_packet_sent_received;
  bool fsk_rssi_available;
  int16_t fsk_rssi;

  sx127x_modulation_t active_modem;
  sx127x_mode_t opmod;
  sx127x_packet_format_t fsk_ook_format;
  sx127x_crc_type_t fsk_crc_type;

  uint64_t *frequencies;
  uint8_t frequencies_length;
  uint8_t current_frequency;
};

/**
 * @brief Create device handle and attach to SPI bus.
 *
 * @param spi_device spi device
 * @param result Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG      if parameter is invalid
 *         - SX127X_ERR_NOT_FOUND        if host doesn't have any free CS slots
 *         - SX127X_ERR_NO_MEM           if no not enough memory
 *         - SX127X_ERR_INVALID_VERSION  if device attached to SPI bus is invalid or chip is not sx127x.
 *         - SX127X_OK                   on success
 */
int sx127x_create(void *spi_device, sx127x *result);

/**
 * @brief Set user context
 *
 * @param user_context Callback context that can be retrieved in callbacks
 * @param device Pointer to variable to hold the device handle
 */
void sx127x_set_user_context(void *user_context, sx127x *device);

/**
 * @brief Get user context
 *
 * @param device Pointer to variable to hold the device handle
 */
void* sx127x_get_user_context(sx127x *device);

/**
 * @brief Set operating mode.
 *
 * @param mode Sleep, standby, rx or tx. See @ref sx127x_mode_t for details.
 * @param modulation LoRa, FSK, OOK. See @ref sx127x_modulation_t for details.
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_set_opmod(sx127x_mode_t mode, sx127x_modulation_t modulation, sx127x *device);

/**
 * @brief Set frequency for RX or TX.
 *
 * @param frequency Frequency in hz.
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_set_frequency(uint64_t frequency, sx127x *device);

/**
 * @brief Get frequency for RX or TX
 * @param device Pointer to variable to hold the device handle
 * @param frequency Result frequency in hz.
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_get_frequency(sx127x *device, uint64_t *frequency);

/**
 * @brief Reset chip's memory pointers to 0. Both RX and TX.
 *
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_lora_reset_fifo(sx127x *device);

/**
 * @brief Set signal bandwidth
 *
 * Enable low datarate optimization if the symbol length exceeds 16ms.
 *
 * @note In the lower band (169MHz), signal bandwidths 8&9 (250k and 500k) are not supported.
 * @param bandwidth Selected bandwidth
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_lora_set_bandwidth(sx127x_bw_t bandwidth, sx127x *device);

/**
 * @brief Get signal bandwidth in hz.
 *
 * @param device Pointer to variable to hold the device handle
 * @param bandwidth Signal bandwidth in hz. For example: 125000.
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_lora_get_bandwidth(sx127x *device, uint32_t *bandwidth);

/**
 * @brief Set speading factor (SF rate). See section 4.1.1.2. for more details.
 *
 * Enable low datarate optimization if the symbol length exceeds 16ms.
 *
 * @param spreading_factor SF rate
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_lora_set_modem_config_2(sx127x_sf_t spreading_factor, sx127x *device);

/**
 * @brief Enable or disable low datarate optimization.
 *
 * It should have the same value on transmitter and receiver. If unsure, don't change.
 *
 * @param value Enable or disable.
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_lora_set_low_datarate_optimization(bool value, sx127x *device);

/**
 * @brief Set syncword.
 *
 * @note Value 0x34 is reserved for LoRaWAN networks.
 * @param value Syncword.
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_lora_set_syncword(uint8_t value, sx127x *device);

/**
 * @brief Set preamble length. See 4.1.1 for more details.
 *
 * @param value Preamble length in symbols for LoRa and bytes for FSK/OOK. Used during TX only
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_set_preamble_length(uint16_t value, sx127x *device);

/**
 * @brief Set implicit header.
 *
 * sx127x can send packets for explicit header or without it (implicit). In implicit mode receiver should be configured with pre-defined values using this function.
 * In explicit mode, all information is sent in the header. Thus no configuration needed.
 *
 * @param header Pre-defined packet information. If NULL, then assume explicit header in RX mode. For TX explicit mode please use sx127x_set_tx_explcit_header function.
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_lora_set_implicit_header(sx127x_implicit_header_t *header, sx127x *device);

/**
 * @brief Configure frequency hopping for TX and RX. Frequency hopping spread spectrum (FHSS) is typically employed when the duration of a single packet could exceed
regulatory requirements relating to the maximum permissible channel dwell time.
 *
 * @param period Symbol periods between frequency hops.
 * @param frequencies Set of predefined frequencies. Should be the same on RX and TX
 * @param frequencies_length Size of predefined frequencies
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_lora_set_frequency_hopping(uint8_t period, uint64_t *frequencies, uint8_t frequencies_length, sx127x *device);

/**
 * @brief Output internal registers
 *
 * Output all internal registers in the current mode. Can be useful for debugging and troubleshooting.
 *
 * @param output Pre-allocated array where the values will be written. Should be at least MAX_NUMBER_OF_REGISTERS length.
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_dump_registers(uint8_t *output, sx127x *device);

/**
 * @brief Handle interrupt from DIOx pins.
 *
 * @note This function SHOULD NOT be called from ISR. Use separate ISR-safe function
 *
 * @param device Pointer to variable to hold the device handle
 */
void sx127x_handle_interrupt(sx127x *device);

/**
 * @brief Set RX gain. Can be manual or automatic.
 *
 * @param gain G1 is maximum, G6 is minimum.
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_rx_set_lna_gain(sx127x_gain_t gain, sx127x *device);

/**
 * @brief Boost LNA current in high frequency mode (over 525Mhz).
 *
 * @param value Enable or disable
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_rx_set_lna_boost_hf(bool value, sx127x *device);

/**
 * @brief Set callback function for rxdone interrupt.
 *
 * @param rx_callback Callback function. Should accept pointer to variable to hold the device handle.
 * @param device Pointer to variable to hold the device handle
 */
void sx127x_rx_set_callback(void (*rx_callback)(sx127x *, uint8_t *, uint16_t), sx127x *device);

/**
 * @brief RSSI of the latest packet received (dBm)
 *
 * @param device Pointer to variable to hold the device handle
 * @param rssi RSSI
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_ERR_NOT_FOUND     In FSK/OOK mode RSSI is calculated during PreambleDetect interrupt. If this interrupt was not configured or received anyhow, then RSSI is empty.
 *         - SX127X_OK                on success
 */
int sx127x_rx_get_packet_rssi(sx127x *device, int16_t *rssi);

/**
 * @brief Estimation of SNR on last packet received
 *
 * @param device Pointer to variable to hold the device handle
 * @param snr SNR
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_lora_rx_get_packet_snr(sx127x *device, float *snr);

/**
 * @brief Compensate reference oscillator drift. The frequency error should be calculated as follows: send the LoRa message at the known stable frequency, get the frequency error on receiver. Please note the frequency should be manually adjusted and configured using sx127x_set_frequency function
 * @param frequency_error The frequency error as measured at the receiver. Just make sure transmitter has stable oscillator before the measurement.
 * @param device Pointer to variable to hold the device handle
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 * @return
 */
int sx127x_lora_set_ppm_offset(int32_t frequency_error, sx127x *device);

/**
 * @brief Read estimated frequency error from modem.
 *
 * @param device Pointer to variable to hold the device handle
 * @param frequency_error Output frequency error
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_rx_get_frequency_error(sx127x *device, int32_t *frequency_error);

// TX-related functions
/**
 * @brief Set output power for transmittion.
 *
 * Set reasonable overload current protection (OCP) based on table 6 from section 2.5.1:
 * 20dbm -> 120mA.
 * 2 ~ 17dbm -> 87mA.
 * 7 ~ 15dbm -> 29dbm.
 * -4 ~ 7dbm -> 20mA.
 *
 * @param pin Which pin to use. sx127x supports: RFO and PA_BOOST.
 * @param power Output power in dbm. When using RFO pin supported range is from -4 to 15dbm. When using PA_BOOST range is from 2 to 17dbm. And also 20dbm for high power output. See section 5.4.3.
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_tx_set_pa_config(sx127x_pa_pin_t pin, int power, sx127x *device);

/**
 * @brief Configure overload current protection (OCP) for PA.
 *
 * @param onoff Enable or disable OCP
 * @param milliamps Maximum current in milliamps
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_tx_set_ocp(bool enable, uint8_t milliamps, sx127x *device);

/**
 * @brief Set explicit header during TX.
 *
 * @param header Transmitter will populate packet's header with this configuration
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_lora_tx_set_explicit_header(sx127x_tx_header_t *header, sx127x *device);

/**
 * @brief Set callback function for txdone interrupt.
 *
 * @param tx_callback Callback function. Should accept pointer to variable to hold the device handle.
 * @param device Pointer to variable to hold the device handle
 */
void sx127x_tx_set_callback(void (*tx_callback)(sx127x *), sx127x *device);

/**
 * @brief Write packet into sx127x's FIFO for transmittion. Once packet is written, set opmod to TX.
 *
 * @param data Packet
 * @param data_length Packet length. Cannot be more than 256 bytes or 0.
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_lora_tx_set_for_transmission(const uint8_t *data, uint8_t data_length, sx127x *device);

/**
 * @brief Write packet into sx127x's FIFO for transmittion. Once packet is written, set opmod to TX.
 *
 * @param data Packet
 * @param data_length Packet length. Maximum length depend on packet format (sx127x_packet_format_t). VARIABLE format is limited by 255 bytes. FIXED format - 2047 bytes
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_fsk_ook_tx_set_for_transmission(const uint8_t *data, uint16_t data_length, sx127x *device);

/**
 * @brief Write packet into sx127x's FIFO for transmittion. Once packet is written, set opmod to TX.
 *
 * @param data Packet
 * @param data_length Packet length. Maximum length depend on packet format (sx127x_packet_format_t). VARIABLE format is limited by 254 bytes. FIXED format - 2046 bytes
 * @param address_to Address to send to. Can be Node address or broadcast address
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_fsk_ook_tx_set_for_transmission_with_address(const uint8_t *data, uint16_t data_length, uint8_t address_to, sx127x *device);

/**
 * @brief Start transmitting periodic beacon using FSK/OOK modulation. Packet format must be configured as SX127X_FIXED.
 * 
 * @param data Data to be transmitted periodically. Can only be changed after beacon stopped.
 * @param data_length Length of data to be transmitted. Cannot exceed 64 bytes.
 * @param interval_ms Period for beacon transmittion in milliseconds. Cannot exceed 255 * 2 * 262 ms = 133620 ms = ~ 2.2 min.
 * @param device Pointer to variable to hold the device handle
 * @return int 
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_ERR_INVALID_STATE if configured packet format is not SX127X_FIXED or selected modem is not FSK/OOK
 *         - SX127X_OK                on success
 */
int sx127x_fsk_ook_tx_start_beacon(const uint8_t *data, uint8_t data_length, uint32_t interval_ms, sx127x *device);

/**
 * @brief Stop transmitting periodic beacon.
 * 
 * @param device Pointer to variable to hold the device handle
 * @return int 
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_fsk_ook_tx_stop_beacon(sx127x *device);

/**
 * @brief Set callback function for caddone interrupt. int argument is 0 when no CAD detected.
 *
 * @param cad_callback Callback function. Should accept pointer to variable to hold the device handle.
 * @param device Pointer to variable to hold the device handle
 */
void sx127x_lora_cad_set_callback(void (*cad_callback)(sx127x *, int), sx127x *device);

/**
 * @brief Set the bit rate for the modulation. Bit rate is the FXOSC / bitrate. FSK modulation utilize fractional bits to make this ratio more precise.
 *
 * @param bitrate Default: 4.8 kb/s
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_fsk_ook_set_bitrate(float bitrate, sx127x *device);

/**
 * @brief Set frequency deviation for FSK modulation. It is most efficient when the modulation index of the signal is greater than 0.5 and below 10.
 *
 * @param frequency_deviation Minimum 600 hz, maximum - 200 khz. Default: 5 kHz
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_fsk_set_fdev(float frequency_deviation, sx127x *device);

/**
 * @brief Configure sync word. Sync word can be used to separate several different networks.
 *
 * @param syncword Array of sync word bytes. Any sync word byte cannot be zero (0x00).
 * @param syncword_length Length of array of sync word bytes. Maximum is 8. Minimum - 1. Default: 3
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_fsk_ook_set_syncword(const uint8_t *syncword, uint8_t syncword_length, sx127x *device);

/**
 * @brief Set data whitening or scrambling is widely used for randomizing the user data before radio transmission. Scrambling can improve bit synchronizer accuracy.
 *
 * @param encoding Can be NRZ (none) (default), MANCHESTER or SCRAMBLED. Scrambled data is passed through proper LFSR polynomial
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_fsk_ook_set_packet_encoding(sx127x_packet_encoding_t encoding, sx127x *device);

/**
 * @brief Set checksum generation for TX or validation for RX.
 *
 * @param crc_type Can be one of: NONE, CCITT (default), IBM.
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_fsk_ook_set_crc(sx127x_crc_type_t crc_type, sx127x *device);

/**
 * @brief Set the packet format.
 *
 * @param format Packet format can be FIXED or VARIABLE (default) (Unlimit is not supported). FIXED should have fixed length known to RX.
 * @param max_payload_length Maximum 2047 for FIXED type. Maximum 255 for VARIABLE. If specified 2047 for VARIABLE type, then payload length check is disabled.
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_fsk_ook_set_packet_format(sx127x_packet_format_t format, uint16_t max_payload_length, sx127x *device);

/**
 * @brief Configure address filtering. It adds another level of filtering. Each packet's first byte must be an address. If address do not match, then rx_callback won't be called. Can be useful for hardware-based filtering, which is fast and consume less power.
 *
 * @param type Type of filtering. Can be NONE (default) - when no filtering applied, NODE_ADDRESS - when expecting point-to-point message or NODE_AND_BROADCAST - when expecting point-to-point and broadcast messages
 * @param node_address Address of this node. Ignored when no filtering is requested.
 * @param broadcast_address Broadcast address of this node. Ignored when no filtering is requested.
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_fsk_ook_set_address_filtering(sx127x_address_filtering_t type, uint8_t node_address, uint8_t broadcast_address, sx127x *device);

/**
 * @brief Set FSK modulation shaping. Used to improve the narrow band response of the transmitter.
 *
 * @param data_shaping Modulation shaping. Can be NONE (default) or Gaussian filtered with BT 0.3, 0.5 or 1.0
 * @param pa_ramp Rise/Fall time of ramp up/down in FSK. Default: 40 us
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_fsk_set_data_shaping(sx127x_fsk_data_shaping_t data_shaping, sx127x_pa_ramp_t pa_ramp, sx127x *device);

/**
 * @brief Set OOK modulation shaping. Used to improve the narrow band response of the transmitter.
 *
 * @param data_shaping Modulation shaping. Can be NONE (default) or filtered with fcutoff = bit_rate or fcutoff = 2*bit_rate
 * @param pa_ramp Rise/Fall time of ramp up/down in FSK. Default: 40 us
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_ook_set_data_shaping(sx127x_ook_data_shaping_t data_shaping, sx127x_pa_ramp_t pa_ramp, sx127x *device);

/**
 * @brief Sets the polarity of the Preamble to be 0xAA (default) or 0x55
 *
 * @param type Preamble Polarity 0xAA (default) or 0x55
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_fsk_ook_set_preamble_type(sx127x_preamble_type_t type, sx127x *device);

/**
 * @brief The OOK demodulator performs a comparison of the RSSI output and a threshold value. This functions selects PEAK mode and configure its parameters.
 *
 * @param step Size of each decrement of the RSSI threshold in the OOK demodulator. Default: 0.5 dB
 * @param floor_threshold Floor threshold for the Data Slicer. Default: 0x0C
 * @param decrement Period of decrement of the RSSI threshold. Default: once per chip (SX127X_1_1_CHIP)
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_ook_rx_set_peak_mode(sx127x_ook_peak_thresh_step_t step, uint8_t floor_threshold, sx127x_ook_peak_thresh_dec_t decrement, sx127x *device);

/**
 * @brief The OOK demodulator performs a comparison of the RSSI output and a threshold value. This functions selects FIXED mode and configure its parameters.
 *
 * @param fixed_threshold Fixed threshold for the Data Slicer in OOK mode. Default: 0x0C
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_ook_rx_set_fixed_mode(uint8_t fixed_threshold, sx127x *device);

/**
 * @brief The OOK demodulator performs a comparison of the RSSI output and a threshold value. This functions selects AVERAGE mode and configure its parameters.
 *
 * @param avg_offset Static offset added to the threshold in average mode in order to reduce glitching activity. Default: 0 dB
 * @param avg_thresh Filter coefficients in average mode of the OOK demodulator. Default: fC ≈ chip rate / 4.π
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_ook_rx_set_avg_mode(sx127x_ook_avg_offset_t avg_offset, sx127x_ook_avg_thresh_t avg_thresh, sx127x *device);

/**
 * @brief Enable AFC on each receiver startup
 *
 * @param afc_auto ON - AFC is performed at each receiver startup. OFF (default) No AFC performed at receiver startup
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_fsk_ook_rx_set_afc_auto(bool afc_auto, sx127x *device);

/**
 * @brief Configure alternate receiver bandwidth during the AFC phase. This allow the accommodation of larger frequency errors. In a typical receiver application the, once the AFC is performed, the radio will revert to the receiver communication or channel bandwidth (RegRxBw) for the ensuing communication phase.
 *
 * @param bandwidth The single-side channel filter bandwidth. Defined by 2 numbers: mantissa and exponent. Thus approximate requested bandwidth. Minimum is 2600 hz, maximum - 250000 hz. Default: 50000 hz.
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_fsk_ook_rx_set_afc_bandwidth(float bandwidth, sx127x *device);

/**
 * @brief Configure bandwidth of channel filter. The role of the channel filter is to reject noise and interference outside of the wanted channel. Channel filtering is implemented with a 16-tap finite impulse response (FIR) filter. To respect sampling criterion in the decimation chain of the receiver, the communication bit rate cannot be set at a higher than twice the single side receiver bandwidth (BitRate < 2 x RxBw)
 *
 * @param bandwidth The single-side channel filter bandwidth. Defined by 2 numbers: mantissa and exponent. Thus approximate requested bandwidth. Minimum is 2600 hz, maximum - 250000 hz. Default: 10400 hz.
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_fsk_ook_rx_set_bandwidth(float bandwidth, sx127x *device);

/**
 * @brief Configure RSSI calculation
 *
 * @param smoothing The number of samples taken to average the RSSI result
 * @param offset Signed RSSI offset, to compensate for the possible losses/gains in the front-end (LNA, SAW filter...). 1dB / LSB, 2’s complement format
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_fsk_ook_rx_set_rssi_config(sx127x_rssi_smoothing_t smoothing, int8_t offset, sx127x *device);

/**
 * @brief Turns on the mechanism restarting the receiver automatically if it gets saturated or a packet collision is detected. Collisions are detected by a sudden rise in received signal strength, detected by the RSSI. This functionality can be useful in network configurations where many asynchronous slaves attempt periodic communication with a single a master node.
 *
 * @param enable Enable or disable. Default: disabled
 * @param threshold Sensitivity of the system in 1 dB steps that detect sudden change in RSSI. Default: 10dB
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_fsk_ook_rx_set_collision_restart(bool enable, uint8_t threshold, sx127x *device);

/**
 * @brief Configure trigger that will start receiver.
 *
 * @param trigger Interrupt that will start receiver. Can be either NONE, Rssi, PreambleDetect (default) or BOTH
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_fsk_ook_rx_set_trigger(sx127x_rx_trigger_t trigger, sx127x *device);

/**
 * @brief Enables Preamble detector when set to 1. The AGC settings supersede this bit during the startup / AGC phase. Used in the receiver only.
 *
 * @param enabled Enable or disable. Default: disabled
 * @param detector_size Number of Preamble bytes to detect to trigger an interrupt. Maximum 3 bytes. Default: 2 bytes
 * @param detector_tolerance Number or chip errors tolerated over detector_size. Default: 0x0A
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_fsk_ook_rx_set_preamble_detector(bool enable, uint8_t detector_size, uint8_t detector_tolerance, sx127x *device);

/**
 * @brief Perform manual image and RSSI calibration. Can be performed only in the standby mode.
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_STATE if current state is not STANDBY
 *         - SX127X_OK                on success
 */
int sx127x_fsk_ook_rx_calibrate(sx127x *device);

/**
 * @brief Control temperature monitoring. By default enabled. Should be used to measure the temperature in any mode except Sleep and Standby
 * @param enable - Default: enabled
 * @param device Pointer to variable to hold the device handle
 * @return int
 *         - SX127X_ERR_INVALID_STATE if current state is not STANDBY
 *         - SX127X_OK                on success
 */
int sx127x_fsk_ook_set_temp_monitor(bool enable, sx127x *device);

/**
 * @brief Higher precision requires a calibration procedure at a known temperature.
 * @param device Pointer to variable to hold the device handle
 * @param raw_temperature raw sensor temperature. Due to process variations, the absolute accuracy of the result is +/- 10 °C.
 * @return int
 *         - SX127X_ERR_INVALID_STATE if current state is not STANDBY
 *         - SX127X_OK                on success
 */
int sx127x_fsk_ook_get_raw_temperature(sx127x *device, int8_t *raw_temperature);

/**
 * Low-level API to read single register. No validation is performed
 * @param reg - the register id defined in sx127x_registers.h
 * @param spi_device - wrapper around spi_device. Use device->spi_device.
 * @param result - the result
 * @return int
 *         - SX127X_ERR_INVALID_ARG   on any SPI transfer errors
 *         - SX127X_OK                on success
 */
int sx127x_read_register(int reg, shadow_spi_device_t *spi_device, uint8_t *result);

/**
 * Low-level API to write single register. No validation is performed
 * @param reg - the register id defined in sx127x_registers.h
 * @param value - value to write
 * @param spi_device - wrapper around spi_device. Use device->spi_device.
 * @return int
 *         - SX127X_ERR_INVALID_ARG   on any SPI transfer errors
 *         - SX127X_OK                on success
 */
int sx127x_write_register(int reg, uint8_t value, shadow_spi_device_t *spi_device);

#ifdef __cplusplus
}
#endif
#endif