#include "BLESerial.h"

//#define BLE_SERIAL_DEBUG

BLESerial* BLESerial::_instance = NULL;

BLESerial::BLESerial(unsigned char req, unsigned char rdy, unsigned char rst) :
  BLEPeripheral(req, rdy, rst)
{
  this->_txCount = 0;
  this->_rxHead = this->_rxTail = 0;
  //sds this->_flushed = 0;
  BLESerial::_instance = this;

  addAttribute(this->_uartService);
  addAttribute(this->_uartNameDescriptor);
  setAdvertisedServiceUuid(this->_uartService.uuid());
  addAttribute(this->_rxCharacteristic);
  addAttribute(this->_rxNameDescriptor);
  this->_rxCharacteristic.setEventHandler(BLEWritten, BLESerial::_received);
  addAttribute(this->_txCharacteristic);
  addAttribute(this->_txNameDescriptor);
}

void BLESerial::begin(...) {
  BLEPeripheral::begin();
  #ifdef BLE_SERIAL_DEBUG
    Serial.println(F("BLESerial::begin()"));
  #endif
}

void BLESerial::poll() {
  // sds: begrijp niet waarom voortdurend wordt geflusht??
  BLEPeripheral::poll();
  /*
  if (millis() < this->_flushed + 100) { 
    BLEPeripheral::poll();
  } else {
    flush();
  }
  */
} // poll

void BLESerial::end() {
  this->_rxCharacteristic.setEventHandler(BLEWritten, NULL);
  this->_rxHead = this->_rxTail = 0;
  flush();
  //sds for test BLEPeripheral::disconnect();
  BLEPeripheral::end();
}

int BLESerial::available(void) {
  BLEPeripheral::poll();
  int retval = (this->_rxHead - this->_rxTail + sizeof(this->_rxBuffer)) % sizeof(this->_rxBuffer);
  #ifdef BLE_SERIAL_DEBUG
    Serial.print(F("BLESerial::available() = "));
    Serial.println(retval);
  #endif
  return retval;
}

int BLESerial::peek(void) {
  BLEPeripheral::poll();
  if (this->_rxTail == this->_rxHead) return -1;
  uint8_t byte = this->_rxBuffer[this->_rxTail];
  #ifdef BLE_SERIAL_DEBUG
    Serial.print(F("BLESerial::peek() = "));
    Serial.print((char) byte);
    Serial.print(F(" 0x"));
    Serial.println(byte, HEX);
  #endif
  return byte;
}

int BLESerial::read(void) {
  BLEPeripheral::poll();
  if (this->_rxTail == this->_rxHead) return -1;
  this->_rxTail = (this->_rxTail + 1) % sizeof(this->_rxBuffer);
  uint8_t byte = this->_rxBuffer[this->_rxTail];
  #ifdef BLE_SERIAL_DEBUG
    Serial.print(F("BLESerial::read() = "));
    Serial.print((char) byte);
    Serial.print(F(" 0x"));
    Serial.println(byte, HEX);
  #endif
  return byte;
}

void BLESerial::flush(void) {
  bool success;

  #ifdef BLE_SERIAL_DEBUG
    Serial.println(F("BLESerial::flush()"));
  #endif

  if (this->_txCount == 0) return;
  //sds 
  while (this->_txCharacteristic.canNotify() == false) {
    bool retval = BLEPeripheral::connected();
    // BLEPeripheral::poll(); // ::connected() does a poll()
    if (!retval) {
      // we've been disconnected -> get out
      #ifdef BLE_SERIAL_DEBUG
        Serial.println(F("BLESerial::flush -> get out!"));
      #endif
      return;
    }
  }

  success = this->_txCharacteristic.setValue(this->_txBuffer, this->_txCount);
  if (success) {
    //sds this->_flushed = millis();
    this->_txCount = 0;
  }
  else {
    // with the busy wait above, this doesn't happen anymore
  }

  BLEPeripheral::poll();
} // flush

size_t BLESerial::write(uint8_t byte) {
  size_t txBufSize = sizeof(this->_txBuffer);
  size_t bytesWritten = 0;
  BLEPeripheral::poll();
  if (this->_txCharacteristic.subscribed() == false) return 0;
  if (this->_txCount < txBufSize-1) {
    this->_txBuffer[this->_txCount++] = byte;
    bytesWritten = 1; // byte successfully written to txBuffer
  }
  if (this->_txCount == sizeof(this->_txBuffer)) flush();
  #ifdef BLE_SERIAL_DEBUG
    Serial.print(F("BLESerial::write("));
    Serial.print((char) byte);
    Serial.print(F(" 0x"));
    Serial.print(byte, HEX);
    Serial.println(F(") = 1"));
  #endif
  return bytesWritten;
} // write

BLESerial::operator bool() {
  bool retval = BLEPeripheral::connected();
  #ifdef BLE_SERIAL_DEBUG
    Serial.print(F("BLESerial::operator bool() = "));
    Serial.println(retval);
  #endif
  return retval;
}

void BLESerial::_received(const uint8_t* data, size_t size) {
  for (unsigned int i = 0; i < size; i++) {
    this->_rxHead = (this->_rxHead + 1) % sizeof(this->_rxBuffer);
    this->_rxBuffer[this->_rxHead] = data[i];
  }
  #ifdef BLE_SERIAL_DEBUG
    Serial.print(F("BLESerial::received("));
    for (int i = 0; i < size; i++) Serial.print((char) data[i]);
    Serial.println(F(")"));
  #endif
}

void BLESerial::_received(BLECentral& /*central*/, BLECharacteristic& rxCharacteristic) {
  BLESerial::_instance->_received(rxCharacteristic.value(), rxCharacteristic.valueLength());
}
