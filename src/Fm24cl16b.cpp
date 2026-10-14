#include "Fm24cl16b.h"

void Fm24cl16b::begin(TwoWire& wire, uint32_t clockHz) {
  _wire = &wire;
  if (clockHz > 0) {
    _wire->setClock(clockHz);
  }
  _ok = probe();
}

bool Fm24cl16b::probe() {
  if (!_wire) return false;
  _wire->beginTransmission(kBaseAddr);
  const uint8_t err = _wire->endTransmission();
  return (err == 0);
}

bool Fm24cl16b::readU8(uint16_t addr, uint8_t& out) {
  return read(addr, &out, 1);
}

bool Fm24cl16b::writeU8(uint16_t addr, uint8_t v) {
  return write(addr, &v, 1);
}

bool Fm24cl16b::read(uint16_t addr, uint8_t* buf, size_t len) {
  if (!_wire || !buf || len == 0) return false;
  if ((uint32_t)addr + (uint32_t)len > kSizeBytes) return false;

  // Chunked: Wire-RX-Puffer ist begrenzt; Adresse je Chunk neu setzen.
  static constexpr size_t kChunk = 32;
  size_t done = 0;
  while (done < len) {
    const uint16_t a = (uint16_t)(addr + done);
    const size_t n = (len - done > kChunk) ? kChunk : (len - done);

    _wire->beginTransmission(slaveAddrFor(a));
    _wire->write(wordAddr(a));
    if (_wire->endTransmission(false) != 0) { // repeated start
      _ok = false;
      return false;
    }

    const size_t got = _wire->requestFrom((int)slaveAddrFor(a), (int)n);
    if (got != n) {
      _ok = false;
      return false;
    }
    for (size_t i = 0; i < n; i++) {
      buf[done + i] = (uint8_t)_wire->read();
    }
    done += n;
  }
  return true;
}

bool Fm24cl16b::write(uint16_t addr, const uint8_t* buf, size_t len) {
  if (!_wire || !buf || len == 0) return false;
  if ((uint32_t)addr + (uint32_t)len > kSizeBytes) return false;

  // Chunked Write (Wire-TX-Puffer). FRAM braucht kein ACK-Polling.
  static constexpr size_t kChunk = 30; // +1 Word-Addr im Buffer
  size_t done = 0;
  while (done < len) {
    const uint16_t a = (uint16_t)(addr + done);
    const size_t n = (len - done > kChunk) ? kChunk : (len - done);

    _wire->beginTransmission(slaveAddrFor(a));
    _wire->write(wordAddr(a));
    _wire->write(buf + done, n);
    if (_wire->endTransmission() != 0) {
      _ok = false;
      return false;
    }
    done += n;
  }
  return true;
}
