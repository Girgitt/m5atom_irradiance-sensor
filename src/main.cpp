#include <M5Atom.h>
#include <Wire.h>
#include <Adafruit_VEML7700.h>

#define I2C_SDA 25
#define I2C_SCL 21

Adafruit_VEML7700 veml;

/* ---------- LED helpers ------------------------------------------------ */
static inline uint8_t xyIdx(uint8_t x, uint8_t y) { return y * 5 + x; }

/* two‑row bar (hundreds / tens). fills bottom cell first, then top */
void drawBar2Rows(uint8_t digit, uint8_t row0, const CRGB &col)
{
    uint8_t filled = (digit * 10 + 4) / 9;     // 0‑9 → 0‑10 pixels
    uint8_t placed = 0;

    for (uint8_t c = 0; c < 5; ++c)
    {
        if (placed < filled) { M5.dis.drawpix(xyIdx(c, row0 + 1), col); ++placed; }
        if (placed < filled) { M5.dis.drawpix(xyIdx(c, row0    ), col); ++placed; }
        if (placed >= filled)
        {
            /* clear remainder of this column */
            M5.dis.drawpix(xyIdx(c, row0    ), CRGB::Black);
            M5.dis.drawpix(xyIdx(c, row0 + 1), CRGB::Black);
        }
    }
}

/* bottom row bar for ones: filled green, tip white (odd) or green (even) */
void drawOnes(uint8_t digit)
{
    uint8_t filled = (digit * 5 + 4) / 9;      // 0‑9 → 0‑5 pixels
    for (uint8_t x = 0; x < 5; ++x)
    {
        if (x < filled)
        {
            bool tip   = (x == filled - 1);
            CRGB color = tip ? (digit & 1 ? CRGB::White : CRGB::Green)
                             : CRGB(0, 64, 0);           // dim green body
            M5.dis.drawpix(xyIdx(x, 4), color);
        }
        else
            M5.dis.drawpix(xyIdx(x, 4), CRGB::Black);
    }
}

void showValue(uint16_t v)
{
    M5.dis.clear();
    drawBar2Rows((v / 100) % 10, 0, CRGB::Red);      // rows 0‑1
    drawBar2Rows((v / 10)  % 10, 2, CRGB::Yellow);   // rows 2‑3
    drawOnes(v % 10);                                // row 4
}

/* ---------- simple auto‑range: gain + integration time ----------------- */
void setRange(uint8_t gIdx, uint8_t itIdx)
{
    switch (gIdx) {
      case 0: veml.setGain(VEML7700_GAIN_1_8); break;
      case 1: veml.setGain(VEML7700_GAIN_1_4); break;
      case 2: veml.setGain(VEML7700_GAIN_1  ); break;
      case 3: veml.setGain(VEML7700_GAIN_2  ); break;
    }
    switch (itIdx) {
      case 0: veml.setIntegrationTime(VEML7700_IT_25MS ); break;
      case 1: veml.setIntegrationTime(VEML7700_IT_50MS ); break;
      case 2: veml.setIntegrationTime(VEML7700_IT_100MS); break;
      case 3: veml.setIntegrationTime(VEML7700_IT_200MS); break;
    }
}

/* --------------------------- setup ------------------------------------ */
void setup()
{
    M5.begin(true, false, true);
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);

    if (!veml.begin()) {
        Serial.println("VEML7700 not found");
        while (true) delay(100);
    }

    setRange(0, 0);                         // start: gain 1/8, IT 25 ms
    M5.dis.clear();
}

/* ---------------------------- loop ------------------------------------ */
void loop()
{
    static uint8_t g = 0, it = 0;           // current gain / IT indices
    float lux = veml.readLux();

    /* auto‑range: keep lux in ~300‑25 000 */
    if (lux > 25000 && g > 0) { --g; setRange(g, it); return; }
    if (lux <   300 && g < 3) { ++g; setRange(g, it); return; }

    float wm2 = lux / 126.7f;               // daylight luminous efficacy
    uint16_t val = wm2 > 999 ? 999 : (uint16_t)round(wm2);

    Serial.printf("Lux:%7.0f  |  W/m²:%4u  |  gainIdx=%u\n", lux, val, g);
    showValue(val);
    delay(1000);
}
