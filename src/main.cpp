// Sensor probe: a line-oriented command shell over USB serial for identifying
// and reading unknown lidar / radar modules from an Arduino Nano 33 BLE.
//
// Wiring (3.3 V logic only — the nRF52840 pins are NOT 5 V tolerant):
//   UART  : module TX -> D0 (RX),  module RX <- D1 (TX)
//   I2C   : SDA = A4, SCL = A5
//   GPIO  : D2..D12, A0..A3, A6, A7 for pin / adc / pulse commands
//
// Every command answers with zero or more text lines and then a final line
// that is exactly "ok" or starts with "err:", so a host script knows when the
// reply is complete.

#include <Arduino.h>
#include <Wire.h>

#define FW_VERSION "sensor-probe 0.1 (Nano 33 BLE)"

static char g_line[256];
static size_t g_lineLen = 0;

static uint32_t g_baud = 115200;
static bool g_uartOpen = false;

static uint8_t g_buf[2048];

// ---------------------------------------------------------------- helpers

static void ok() { Serial.println("ok"); }
static void err(const char* msg) {
  Serial.print("err: ");
  Serial.println(msg);
}

static void hexByte(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

static void hexDump(const uint8_t* p, size_t n) {
  for (size_t i = 0; i < n; i++) {
    if (i) Serial.print(' ');
    hexByte(p[i]);
  }
}

static bool parseNum(const char* s, uint32_t& out) {
  if (!s || !*s) return false;
  char* end;
  out = strtoul(s, &end, 0);  // 0x.. hex, otherwise decimal
  return *end == 0;
}

static bool parseHexByte(const char* s, uint8_t& out) {
  if (!s || !*s) return false;
  char* end;
  unsigned long v = strtoul(s, &end, 16);
  if (*end != 0 || v > 0xFF) return false;
  out = (uint8_t)v;
  return true;
}

// "D5", "5", "A3" -> Arduino pin number, or -1.
static int parsePin(const char* s) {
  if (!s || !*s) return -1;
  uint32_t n;
  if ((s[0] == 'A' || s[0] == 'a') && parseNum(s + 1, n) && n <= 7) return A0 + n;
  if ((s[0] == 'D' || s[0] == 'd') && parseNum(s + 1, n) && n <= 13) return n;
  if (parseNum(s, n) && n <= 13) return n;
  return -1;
}

// True if the host sent anything (used to abort long-running commands).
static bool hostInterrupt() {
  if (!Serial.available()) return false;
  while (Serial.available()) Serial.read();
  return true;
}

// ---------------------------------------------------------------- UART

// The mbed core only maps a fixed list of baud rates, and 256000 (the HLK
// LD24xx radar default) is not on it. So open at a rate it knows and then
// write the nRF BAUDRATE register directly, which accepts any rate.
static uint32_t baudReg(uint32_t baud) {
  uint64_t v = ((uint64_t)baud << 32) / 16000000ULL;
  return (uint32_t)((v + 0x800) & 0xFFFFF000ULL);
}

static void uartSetBaud(uint32_t baud) {
  if (g_uartOpen) Serial1.end();
  Serial1.begin(115200);
  g_uartOpen = true;
  NRF_UARTE_Type* inst[] = {NRF_UARTE0, NRF_UARTE1};
  for (NRF_UARTE_Type* u : inst) {
    if (u->ENABLE) u->BAUDRATE = baudReg(baud);
  }
  g_baud = baud;
  delay(3);
  while (Serial1.available()) Serial1.read();
}

static void uartEnsure() {
  if (!g_uartOpen) uartSetBaud(g_baud);
}

// Collect up to cap bytes for ms milliseconds.
static size_t uartCapture(uint8_t* buf, size_t cap, uint32_t ms) {
  size_t n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < ms && n < cap) {
    while (Serial1.available() && n < cap) buf[n++] = (uint8_t)Serial1.read();
  }
  return n;
}

static size_t countPattern(const uint8_t* buf, size_t n, const uint8_t* pat, size_t len) {
  size_t hits = 0;
  if (n < len) return 0;
  for (size_t i = 0; i + len <= n; i++) {
    if (memcmp(buf + i, pat, len) == 0) {
      hits++;
      i += len - 1;
    }
  }
  return hits;
}

struct Signature {
  uint8_t pat[4];
  uint8_t len;
  const char* name;
};

static const Signature SIGS[] = {
    {{0xF4, 0xF3, 0xF2, 0xF1}, 4, "HLK LD2410/2411/2412 radar data frame"},
    {{0xFD, 0xFC, 0xFB, 0xFA}, 4, "HLK LD24xx radar command/ack frame"},
    {{0xAA, 0xFF, 0x03, 0x00}, 4, "HLK LD2450 radar target frame"},
    {{0x54, 0x2C, 0, 0}, 2, "LDROBOT LD06/LD19/STL-19 lidar packet"},
    {{0x53, 0x59, 0, 0}, 2, "Seeed/MicRadar MR24/MR60 radar frame"},
    {{0xAA, 0x55, 0, 0}, 2, "YDLIDAR packet header"},
    {{0xA5, 0x5A, 0, 0}, 2, "RPLIDAR response descriptor"},
    {{0x57, 0x00, 0xFF, 0}, 3, "Nooploop TOFSense frame"},
    {{0x24, 0x47, 0, 0}, 2, "NMEA ($G...)"},
};

// Benewake TFmini / TF-Luna / TF02: 59 59 distL distH strL strH tL tH checksum.
static size_t benewakeFrames(const uint8_t* buf, size_t n, uint16_t& lastDist, uint16_t& lastStrength) {
  size_t good = 0;
  for (size_t i = 0; i + 9 <= n; i++) {
    if (buf[i] != 0x59 || buf[i + 1] != 0x59) continue;
    uint8_t sum = 0;
    for (int k = 0; k < 8; k++) sum += buf[i + k];
    if (sum != buf[i + 8]) continue;
    good++;
    lastDist = buf[i + 2] | (buf[i + 3] << 8);
    lastStrength = buf[i + 4] | (buf[i + 5] << 8);
    i += 8;
  }
  return good;
}

// DYP A02 / JSN-SR04T-style ultrasonic: FF distH distL checksum.
static size_t ffFrames(const uint8_t* buf, size_t n, uint16_t& lastMm) {
  size_t good = 0;
  for (size_t i = 0; i + 4 <= n; i++) {
    if (buf[i] != 0xFF) continue;
    if ((uint8_t)(buf[i] + buf[i + 1] + buf[i + 2]) != buf[i + 3]) continue;
    if (buf[i + 1] == 0xFF) continue;  // all-FF garbage also "checksums"
    good++;
    lastMm = (buf[i + 1] << 8) | buf[i + 2];
    i += 3;
  }
  return good;
}

static int asciiPercent(const uint8_t* buf, size_t n) {
  if (!n) return 0;
  size_t a = 0;
  for (size_t i = 0; i < n; i++) {
    uint8_t c = buf[i];
    if ((c >= 0x20 && c < 0x7F) || c == '\r' || c == '\n' || c == '\t') a++;
  }
  return (int)(a * 100 / n);
}

// Print what a captured buffer looks like. Returns true if anything matched.
static bool describeCapture(const uint8_t* buf, size_t n) {
  bool any = false;
  uint16_t d = 0, s = 0;
  size_t bw = benewakeFrames(buf, n, d, s);
  if (bw) {
    Serial.print("    MATCH Benewake TFmini/TF-Luna/TF02: ");
    Serial.print(bw);
    Serial.print(" checksum-valid frames, last dist=");
    Serial.print(d);
    Serial.print(" strength=");
    Serial.println(s);
    any = true;
  }
  uint16_t mm = 0;
  size_t ff = ffFrames(buf, n, mm);
  if (ff >= 2) {
    Serial.print("    MATCH FF-framed ultrasonic (DYP A02 style): ");
    Serial.print(ff);
    Serial.print(" valid frames, last=");
    Serial.print(mm);
    Serial.println(" mm");
    any = true;
  }
  for (const Signature& sig : SIGS) {
    size_t hits = countPattern(buf, n, sig.pat, sig.len);
    // Two-byte patterns turn up by chance in noise; ask for a few.
    size_t need = sig.len >= 3 ? 1 : 3;
    if (hits >= need) {
      Serial.print("    MATCH ");
      Serial.print(sig.name);
      Serial.print(" x");
      Serial.println(hits);
      any = true;
    }
  }
  int ap = asciiPercent(buf, n);
  if (n >= 8 && ap >= 90) {
    Serial.print("    MATCH ASCII text: \"");
    for (size_t i = 0; i < n && i < 96; i++) {
      char c = (char)buf[i];
      if (c == '\r') Serial.print("\\r");
      else if (c == '\n') Serial.print("\\n");
      else if (c >= 0x20 && c < 0x7F) Serial.print(c);
      else Serial.print('.');
    }
    Serial.println("\"");
    any = true;
  }
  return any;
}

// Is anything driving the RX line? A wired, idle UART transmitter holds it
// HIGH; an unconnected pin reads LOW through the pull-down.
static void cmdUartLevel() {
  if (g_uartOpen) {
    Serial1.end();
    g_uartOpen = false;
  }
  pinMode(0, INPUT_PULLDOWN);
  delay(2);
  uint32_t highs = 0, total = 0, edges = 0;
  int last = digitalRead(0);
  uint32_t t0 = millis();
  while (millis() - t0 < 500) {
    int v = digitalRead(0);
    if (v != last) edges++;
    last = v;
    highs += v;
    total++;
  }
  Serial.print("RX (D0) over 500 ms: high ");
  Serial.print(total ? highs * 100 / total : 0);
  Serial.print("% of samples, ");
  Serial.print(edges);
  Serial.println(" edges seen");
  if (edges > 4) Serial.println("  -> something is transmitting");
  else if (highs * 100 > total * 90) Serial.println("  -> held HIGH: a transmitter is connected but silent (may need a command)");
  else Serial.println("  -> LOW: nothing connected, module unpowered, or TX/RX swapped");
  pinMode(0, INPUT);
  ok();
}

static void cmdUartSweep(char* arg) {
  static const uint32_t BAUDS[] = {9600,   19200,  38400,  57600,  115200, 128000, 230400,
                                   250000, 256000, 420000, 460800, 500000, 921600, 1000000};
  uint32_t ms = 500;
  if (arg && !parseNum(arg, ms)) return err("usage: uart sweep [ms per baud]");
  uint32_t keep = g_baud;
  for (uint32_t baud : BAUDS) {
    uartSetBaud(baud);
    size_t n = uartCapture(g_buf, sizeof(g_buf), ms);
    Serial.print("  ");
    Serial.print(baud);
    Serial.print(": ");
    Serial.print(n);
    Serial.print(" bytes");
    if (n) {
      Serial.print(", ascii ");
      Serial.print(asciiPercent(g_buf, n));
      Serial.print("%, head: ");
      hexDump(g_buf, n < 20 ? n : 20);
    }
    Serial.println();
    if (n) describeCapture(g_buf, n);
  }
  uartSetBaud(keep);
  Serial.println("(no MATCH lines = nothing recognised; the baud with the most structured-looking head is the best guess)");
  ok();
}

// Stream received bytes as hex, one line per burst (a 3 ms gap ends a line),
// each prefixed with the time in ms. Stops after the duration or on any
// byte from the host.
static void uartMonitor(uint32_t ms) {
  uint32_t t0 = millis();
  uint32_t lastByte = 0;
  size_t inLine = 0;
  size_t total = 0;
  while (millis() - t0 < ms) {
    if (hostInterrupt()) break;
    if (Serial1.available()) {
      uint8_t b = (uint8_t)Serial1.read();
      uint32_t now = millis();
      if (inLine && (now - lastByte > 3 || inLine >= 48)) {
        Serial.println();
        inLine = 0;
      }
      if (!inLine) {
        Serial.print(now - t0);
        Serial.print(": ");
      } else {
        Serial.print(' ');
      }
      hexByte(b);
      inLine++;
      total++;
      lastByte = now;
    }
  }
  if (inLine) Serial.println();
  Serial.print("(");
  Serial.print(total);
  Serial.print(" bytes at ");
  Serial.print(g_baud);
  Serial.println(" baud)");
}

static void cmdUart(char* sub) {
  if (!sub) return err("usage: uart baud|sweep|mon|send|text|level|decode");
  if (!strcmp(sub, "level")) return cmdUartLevel();
  if (!strcmp(sub, "sweep")) return cmdUartSweep(strtok(nullptr, " "));

  if (!strcmp(sub, "baud")) {
    char* a = strtok(nullptr, " ");
    uint32_t baud;
    if (a) {
      if (!parseNum(a, baud) || baud < 300 || baud > 1000000) return err("baud must be 300..1000000");
      uartSetBaud(baud);
    } else {
      uartEnsure();
    }
    Serial.print("baud ");
    Serial.print(g_baud);
    Serial.print(" (register 0x");
    Serial.print(baudReg(g_baud), HEX);
    Serial.println(")");
    return ok();
  }

  if (!strcmp(sub, "mon")) {
    uint32_t ms = 2000;
    char* a = strtok(nullptr, " ");
    if (a && !parseNum(a, ms)) return err("usage: uart mon [ms]");
    uartEnsure();
    uartMonitor(ms);
    return ok();
  }

  if (!strcmp(sub, "decode")) {
    uint32_t ms = 1000;
    char* a = strtok(nullptr, " ");
    if (a && !parseNum(a, ms)) return err("usage: uart decode [ms]");
    uartEnsure();
    size_t n = uartCapture(g_buf, sizeof(g_buf), ms);
    Serial.print(n);
    Serial.print(" bytes at ");
    Serial.println(g_baud);
    if (!describeCapture(g_buf, n)) Serial.println("    nothing recognised");
    return ok();
  }

  // uart send <hex bytes...>   then listens 300 ms for a reply
  if (!strcmp(sub, "send")) {
    uint8_t out[64];
    size_t n = 0;
    char* a;
    while ((a = strtok(nullptr, " ")) != nullptr) {
      if (n >= sizeof(out) || !parseHexByte(a, out[n])) return err("usage: uart send <hex byte> ... (max 64)");
      n++;
    }
    if (!n) return err("usage: uart send <hex byte> ...");
    uartEnsure();
    while (Serial1.available()) Serial1.read();
    Serial1.write(out, n);
    Serial1.flush();
    uartMonitor(300);
    return ok();
  }

  // uart text <string>   sends the rest of the line plus CRLF, then listens
  if (!strcmp(sub, "text")) {
    char* rest = strtok(nullptr, "");
    uartEnsure();
    while (Serial1.available()) Serial1.read();
    if (rest) Serial1.print(rest);
    Serial1.print("\r\n");
    Serial1.flush();
    uartMonitor(300);
    return ok();
  }

  err("unknown uart subcommand");
}

// ---------------------------------------------------------------- I2C

static bool g_wireUp = false;
static bool g_extSda = false, g_extScl = false;  // sampled at boot

struct KnownAddr {
  uint8_t addr;
  const char* what;
};

static const KnownAddr KNOWN[] = {
    {0x08, "Nooploop TOFSense (I2C mode)"},
    {0x0D, "QMC5883L compass"},
    {0x10, "Benewake TF-Luna / TFmini-S / TFmini-Plus (I2C mode)"},
    {0x13, "VCNL4010 proximity"},
    {0x1E, "HMC5883L / LIS3MDL compass"},
    {0x29, "ST VL53L0X / VL53L1X / VL53L4CD / VL6180X ToF (or TSL2591, BNO055)"},
    {0x30, "VL53L0X/L1X moved address, MMC5603"},
    {0x39, "APDS-9960 gesture/proximity"},
    {0x3C, "SSD1306 OLED"},
    {0x40, "INA219 / HTU21 / PCA9685"},
    {0x41, "ams TMF8801/TMF882x ToF"},
    {0x48, "ADS1115 / TMP102"},
    {0x52, "Acconeer XM125 radar (I2C), EEPROM"},
    {0x53, "ADXL345, LTR-390"},
    {0x57, "MAX3010x, EEPROM"},
    {0x62, "Garmin LIDAR-Lite v3/v4"},
    {0x66, "LightWare SF/LW lidar"},
    {0x68, "MPU-6050 / ICM-20x / DS3231"},
    {0x69, "MPU/ICM alt address"},
    {0x70, "MaxBotix I2CXL sonar / SRF0x / TCA9548 mux"},
    {0x74, "TeraRanger / PAA5100?"},
    {0x76, "BMP280 / BME280 / MS5611"},
    {0x77, "BMP280 / BMP388 / MS5611"},
};

// A pull-up of roughly 4.7k or stronger wins against the nRF's ~13k internal
// pull-down. Only valid before the I2C driver has ever touched the pins:
// Wire.end() does not hand them back, so this is sampled once in setup().
// (Releasing a driven-low pin and reading it back does not work either: a
// floating pin here drifts high on its own.)
// The Nano 33 BLE has no pull-ups of its own on A4/A5.
static bool hasPullup(int pin) {
  pinMode(pin, INPUT_PULLDOWN);
  delay(2);
  return digitalRead(pin) == HIGH;
}

static void forceInternalPullups() {
  // A4 = P0.31 (SDA), A5 = P0.02 (SCL)
  const uint8_t pins[] = {31, 2};
  for (uint8_t p : pins) {
    uint32_t cnf = NRF_P0->PIN_CNF[p];
    cnf &= ~GPIO_PIN_CNF_PULL_Msk;
    cnf |= (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
    NRF_P0->PIN_CNF[p] = cnf;
  }
}

// Returns false if the bus cannot work at all.
static bool wireStart(bool verbose) {
  if (g_wireUp) return true;
  bool sda = g_extSda, scl = g_extScl;
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  delay(2);
  bool sdaHigh = digitalRead(A4), sclHigh = digitalRead(A5);
  if (verbose) {
    Serial.print("external pull-ups (as seen at power-up): SDA(A4) ");
    Serial.print(sda ? "yes" : "NO");
    Serial.print(", SCL(A5) ");
    Serial.println(scl ? "yes" : "NO");
  }
  if (!sdaHigh || !sclHigh) {
    Serial.println("a bus line is held LOW even with the internal pull-up: wiring fault or unpowered module");
    return false;
  }
  if (verbose && (!sda || !scl)) {
    Serial.println("  using the nRF's weak internal pull-ups (~13k): fine for short wires at 100 kHz");
  }
  Wire.begin();
  Wire.setClock(100000);
  Wire.beginTransmission(0x7F);  // dummy transaction so the driver finishes configuring pins
  Wire.endTransmission();
  forceInternalPullups();
  g_wireUp = true;
  return true;
}

static bool i2cRead8(uint8_t addr, uint8_t reg, uint8_t* out, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n) != n) return false;
  for (size_t i = 0; i < n; i++) out[i] = Wire.read();
  return true;
}

static bool i2cRead16(uint8_t addr, uint16_t reg, uint8_t* out, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg >> 8);
  Wire.write(reg & 0xFF);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n) != n) return false;
  for (size_t i = 0; i < n; i++) out[i] = Wire.read();
  return true;
}

// The ST ToF family all sit at 0x29; their ID registers tell them apart.
static void identify0x29(uint8_t addr) {
  uint8_t b[2];
  if (i2cRead16(addr, 0x010F, b, 2)) {
    uint16_t id = (b[0] << 8) | b[1];
    const char* name = nullptr;
    if (id == 0xEACC) name = "VL53L1X";
    else if (id == 0xEBAA) name = "VL53L4CD";
    else if (id == 0xEAAA) name = "VL53L3CX";
    else if (id == 0xF0AA || id == 0xF002) name = "VL53L5CX/L7CX family";
    if (name) {
      Serial.print("      ID reg 0x010F = 0x");
      Serial.print(id, HEX);
      Serial.print(" -> ");
      Serial.println(name);
      return;
    }
  }
  if (i2cRead8(addr, 0xC0, b, 1) && b[0] == 0xEE) {
    Serial.println("      ID reg 0xC0 = 0xEE -> VL53L0X");
    return;
  }
  if (i2cRead16(addr, 0x0000, b, 1) && b[0] == 0xB4) {
    Serial.println("      ID reg 0x0000 = 0xB4 -> VL6180X");
    return;
  }
  if (i2cRead8(addr, 0x00, b, 1) && b[0] == 0xA0) {
    Serial.println("      chip ID 0xA0 -> BNO055");
    return;
  }
  Serial.println("      no ST ToF ID matched");
}

static void cmdI2cScan() {
  g_wireUp = false;
  Wire.end();
  if (!wireStart(true)) return err("bus not usable");
  int found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() != 0) continue;
    found++;
    Serial.print("  0x");
    hexByte(a);
    bool named = false;
    for (const KnownAddr& k : KNOWN) {
      if (k.addr == a) {
        Serial.print("  maybe: ");
        Serial.print(k.what);
        named = true;
      }
    }
    if (!named) Serial.print("  (not in my table)");
    Serial.println();
    if (a == 0x29 || a == 0x30) identify0x29(a);
  }
  Serial.print(found);
  Serial.println(" device(s) answered");
  if (found > 60) Serial.println("  (nearly every address answered: SDA is probably stuck or shorted)");
  ok();
}

static void cmdI2c(char* sub) {
  if (!sub) return err("usage: i2c scan|r|r16|raw|w");
  if (!strcmp(sub, "scan")) return cmdI2cScan();

  uint32_t addr, reg = 0, n = 1;
  char* a = strtok(nullptr, " ");
  if (!parseNum(a, addr) || addr > 0x7F) return err("bad address (7-bit, e.g. 0x29)");
  if (!wireStart(false)) return err("bus not usable");

  if (!strcmp(sub, "r") || !strcmp(sub, "r16")) {
    bool wide = sub[1] == '1';
    if (!parseNum(strtok(nullptr, " "), reg)) return err("usage: i2c r|r16 <addr> <reg> [n]");
    char* c = strtok(nullptr, " ");
    if (c && (!parseNum(c, n) || n < 1 || n > 64)) return err("n must be 1..64");
    uint8_t out[64];
    bool good = wide ? i2cRead16(addr, reg, out, n) : i2cRead8(addr, reg, out, n);
    if (!good) return err("no ACK / short read");
    hexDump(out, n);
    Serial.println();
    return ok();
  }

  if (!strcmp(sub, "raw")) {
    char* c = strtok(nullptr, " ");
    if (c && (!parseNum(c, n) || n < 1 || n > 64)) return err("n must be 1..64");
    if (Wire.requestFrom((uint8_t)addr, (size_t)n) != n) return err("no ACK / short read");
    for (uint32_t i = 0; i < n; i++) {
      if (i) Serial.print(' ');
      hexByte(Wire.read());
    }
    Serial.println();
    return ok();
  }

  if (!strcmp(sub, "w")) {
    Wire.beginTransmission((uint8_t)addr);
    int count = 0;
    while ((a = strtok(nullptr, " ")) != nullptr) {
      uint8_t b;
      if (!parseHexByte(a, b)) return err("usage: i2c w <addr> <hex byte> ...");
      Wire.write(b);
      count++;
    }
    if (Wire.endTransmission() != 0) return err("no ACK");
    Serial.print("wrote ");
    Serial.print(count);
    Serial.println(" byte(s)");
    return ok();
  }

  err("unknown i2c subcommand");
}

// ---------------------------------------------------------------- GPIO

static void cmdPin(char* name) {
  int pin = parsePin(name);
  if (pin < 0) return err("usage: pin <D2..D13|A0..A7> [in|up|down|0|1]");
  char* mode = strtok(nullptr, " ");
  if (mode) {
    if (!strcmp(mode, "in")) pinMode(pin, INPUT);
    else if (!strcmp(mode, "up")) pinMode(pin, INPUT_PULLUP);
    else if (!strcmp(mode, "down")) pinMode(pin, INPUT_PULLDOWN);
    else if (!strcmp(mode, "0") || !strcmp(mode, "1")) {
      pinMode(pin, OUTPUT);
      digitalWrite(pin, mode[0] == '1');
    } else return err("mode must be in|up|down|0|1");
    delay(1);
  }
  Serial.print(name);
  Serial.print(" = ");
  Serial.println(digitalRead(pin));
  ok();
}

static void cmdAdc(char* name) {
  int pin = parsePin(name);
  if (pin < A0) return err("usage: adc <A0..A7> [samples]");
  uint32_t n = 1;
  char* c = strtok(nullptr, " ");
  if (c && (!parseNum(c, n) || n < 1 || n > 200)) return err("samples must be 1..200");
  for (uint32_t i = 0; i < n; i++) {
    uint32_t raw = analogRead(pin);
    Serial.print(raw * 3300 / 4095);
    Serial.println(" mV");
    if (n > 1) delay(10);
  }
  ok();
}

// Measure pulse widths, for modules with a PWM distance output.
static void cmdPulse(char* name) {
  int pin = parsePin(name);
  if (pin < 0) return err("usage: pulse <pin> [hi|lo] [count]");
  int level = HIGH;
  uint32_t n = 5;
  char* c = strtok(nullptr, " ");
  if (c) {
    if (!strcmp(c, "lo")) level = LOW;
    else if (strcmp(c, "hi")) return err("level must be hi or lo");
    c = strtok(nullptr, " ");
    if (c && (!parseNum(c, n) || n < 1 || n > 50)) return err("count must be 1..50");
  }
  pinMode(pin, INPUT);
  for (uint32_t i = 0; i < n; i++) {
    unsigned long us = pulseIn(pin, level, 500000UL);
    if (us) {
      Serial.print(us);
      Serial.println(" us");
    } else {
      Serial.println("timeout (no pulse in 500 ms)");
    }
  }
  ok();
}

// ---------------------------------------------------------------- shell

static void cmdHelp() {
  Serial.println(FW_VERSION);
  Serial.println("3.3 V LOGIC ONLY. UART: module TX->D0, module RX<-D1. I2C: SDA=A4, SCL=A5.");
  Serial.println("numbers: decimal or 0x hex. byte lists (send / w): bare hex, e.g. 5A 05 00 01 60");
  Serial.println("  info                         version and state");
  Serial.println("  uart level                   is anything wired to / talking on RX?");
  Serial.println("  uart sweep [ms]              listen at 14 baud rates, match known framing");
  Serial.println("  uart baud [n]                set / show baud (any rate 300..1000000)");
  Serial.println("  uart mon [ms]                hex dump, one line per burst (default 2000)");
  Serial.println("  uart decode [ms]             capture and match known framing at this baud");
  Serial.println("  uart send <hex>...           send bytes, show 300 ms of reply");
  Serial.println("  uart text <string>           send string + CRLF, show 300 ms of reply");
  Serial.println("  i2c scan                     scan, name known addresses, ID ST ToF chips");
  Serial.println("  i2c r <addr> <reg> [n]       read n bytes from 8-bit register");
  Serial.println("  i2c r16 <addr> <reg> [n]     read n bytes from 16-bit register");
  Serial.println("  i2c raw <addr> [n]           read n bytes with no register write");
  Serial.println("  i2c w <addr> <hex>...        write bytes");
  Serial.println("  pin <pin> [in|up|down|0|1]   set mode / drive, then read");
  Serial.println("  adc <A0..A7> [samples]       read voltage in mV (0..3300)");
  Serial.println("  pulse <pin> [hi|lo] [count]  measure pulse widths in us");
  ok();
}

static void cmdInfo() {
  Serial.println(FW_VERSION);
  Serial.print("uart: ");
  Serial.print(g_uartOpen ? "open" : "closed");
  Serial.print(", baud ");
  Serial.println(g_baud);
  Serial.print("i2c: ");
  Serial.println(g_wireUp ? "started" : "not started");
  Serial.print("uptime ms: ");
  Serial.println(millis());
  ok();
}

static void dispatch(char* line) {
  char* cmd = strtok(line, " ");
  if (!cmd) return;
  if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) return cmdHelp();
  if (!strcmp(cmd, "info")) return cmdInfo();
  if (!strcmp(cmd, "uart")) return cmdUart(strtok(nullptr, " "));
  if (!strcmp(cmd, "i2c")) return cmdI2c(strtok(nullptr, " "));
  if (!strcmp(cmd, "pin")) return cmdPin(strtok(nullptr, " "));
  if (!strcmp(cmd, "adc")) return cmdAdc(strtok(nullptr, " "));
  if (!strcmp(cmd, "pulse")) return cmdPulse(strtok(nullptr, " "));
  err("unknown command (try: help)");
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(LED_BUILTIN, OUTPUT);
  g_extSda = hasPullup(A4);
  g_extScl = hasPullup(A5);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, (millis() / 500) & 1);  // heartbeat
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      g_line[g_lineLen] = 0;
      g_lineLen = 0;
      dispatch(g_line);
    } else if (g_lineLen < sizeof(g_line) - 1) {
      g_line[g_lineLen++] = c;
    }
  }
}
