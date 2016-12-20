/*
 * This is an Arduino library wrapper around the PebbleSerial library.
 */

#include "ArduinoPebbleSerial.h"
#include "utility/board.h"

static bool s_is_hardware;
static uint8_t *s_buffer;
static size_t s_buffer_length;
static uint8_t s_pin;

static void prv_cmd_cb(SmartstrapCmd cmd, uint32_t arg) {
  switch (cmd) {
  case SmartstrapCmdSetBaudRate:
    #ifdef AVR
    if (arg == 57600) {
      // The Arduino library intentionally uses bad prescalers for a baud rate of exactly 57600 so
      // we just increase it by 1 to prevent it from doing that.
      arg++;
    }
    #endif
    BOARD_SERIAL.begin(arg);
    board_begin();
    break;
  case SmartstrapCmdSetTxEnabled:
    if (!arg)
      BOARD_SERIAL.flush();
    board_set_tx_enabled(arg);
    break;
  case SmartstrapCmdWriteByte:
    BOARD_SERIAL.write((uint8_t)arg);
    break;
  case SmartstrapCmdWriteBreak:
    // need to flush before changing parity
    BOARD_SERIAL.flush();
    board_set_even_parity(true);
    BOARD_SERIAL.write((uint8_t) 0);
    // need to flush before changing parity
    BOARD_SERIAL.flush();
    board_set_even_parity(false);
    break;
  default:
    break;
  }
}

static void prv_begin(uint8_t *buffer, size_t length, Baud baud,
                      const uint16_t *services, uint8_t num_services) {
  s_buffer = buffer;
  s_buffer_length = length;

  pebble_init(prv_cmd_cb, (PebbleBaud)baud, services, num_services);
  pebble_prepare_for_read(s_buffer, s_buffer_length);
}

void ArduinoPebbleSerial::begin_hardware(uint8_t *buffer, size_t length, Baud baud,
                                         const uint16_t *services, uint8_t num_services) {
  s_is_hardware = true;
  prv_begin(buffer, length, baud, services, num_services);
}

static int prv_available_bytes(void) {
  return BOARD_SERIAL.available();
}

static uint8_t prv_read_byte(void) {
  return (uint8_t)BOARD_SERIAL.read();
}

bool ArduinoPebbleSerial::feed(uint16_t *service_id, uint16_t *attribute_id, size_t *length,
                               RequestType *type) {
  SmartstrapRequestType request_type;
  bool did_feed = false;
  while (prv_available_bytes()) {
    did_feed = true;
    if (pebble_handle_byte(prv_read_byte(), service_id, attribute_id, length, &request_type,
                           millis())) {
      // we have a full frame
      pebble_prepare_for_read(s_buffer, s_buffer_length);
      switch (request_type) {
      case SmartstrapRequestTypeRead:
        *type = RequestTypeRead;
        break;
      case SmartstrapRequestTypeWrite:
        *type = RequestTypeWrite;
        break;
      case SmartstrapRequestTypeWriteRead:
        *type = RequestTypeWriteRead;
        break;
      default:
        break;
      }
      return true;
    }
  }

  if (!did_feed) {
    // allow the pebble code to dicsonnect if we haven't gotten any messages recently
    pebble_is_connected(millis());
  }
  return false;
}

bool ArduinoPebbleSerial::write(bool success, const uint8_t *payload, size_t length) {
  return pebble_write(success, payload, length);
}

void ArduinoPebbleSerial::notify(uint16_t service_id, uint16_t attribute_id) {
  pebble_notify(service_id, attribute_id);
}

bool ArduinoPebbleSerial::is_connected(void) {
  return pebble_is_connected(millis());
}
