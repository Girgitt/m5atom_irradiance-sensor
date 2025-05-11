#include <M5Atom.h>
#include <Wire.h>
#include <Adafruit_VEML7700.h>

#define I2C_SDA 25
#define I2C_SCL 21

Adafruit_VEML7700 veml;

#define ROLLING_WINDOW_SIZE 25  // Change this to desired N

uint16_t rollingBuffer[ROLLING_WINDOW_SIZE] = {0};
uint8_t rollingIndex = 0;
uint8_t rollingCount = 0;

uint16_t rollingBuffer_raw[ROLLING_WINDOW_SIZE] = {0};
uint8_t rollingIndex_raw = 0;
uint8_t rollingCount_raw = 0;

/* ---------- LED helpers ------------------------------------------------ */
static inline uint8_t xyIdx(uint8_t x, uint8_t y) { return y * 5 + x; }

/* two‑row bar (hundreds / tens). fills bottom cell first, then top */
void drawBar2RowsScaled(uint8_t value, uint8_t row0, const CRGB &col, uint16_t scale)
{
    // value: full value (e.g., val / 10 or val / 100)
    // scale: max input value for full bar (e.g., 10 for tens, 10 for hundreds)
    uint8_t filled = (value * 10 + scale / 2) / scale; // round to nearest of 10
    for (uint8_t p = 0; p < 10; ++p) {
        uint8_t x = p / 2;
        uint8_t y = row0 + ((p & 1) ? 0 : 1);  // even = bottom, odd = top
        M5.dis.drawpix(xyIdx(x, y), (p < filled) ? col : CRGB::Black);
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
    drawBar2RowsScaled(v / 100, 0, CRGB::Red, 10);           // hundreds, 0–999 → 0–10
    drawBar2RowsScaled((v / 10) % 10, 2, CRGB::Yellow, 10);  // tens, 0–99 → 0–10
    drawOnes(v % 10);                                        // row 4
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
    //static uint8_t g = 0, it = 0;           // current gain / IT indices
    //float lux = veml.readLux();
    float lux = veml.readLuxNormalized();  // correction for high-lux setRange(0, 0) which is gain 1/8, IT 25 ms
    

    // /* auto‑range: keep lux in ~300‑25 000 */
    // if (lux > 25000 && g > 0) { --g; setRange(g, it); return; }
    // if (lux <   300 && g < 3) { ++g; setRange(g, it); return; }

    //float wm2 = lux / 126.7f;               // daylight luminous efficacy - theoretical
    float wm2 = lux / 122;                    // based on https://www.extrica.com/article/21667
    
    

    // rolling average for wm2 for led display (1-999 range)
    rollingBuffer[rollingIndex] = wm2;
    rollingIndex = (rollingIndex + 1) % ROLLING_WINDOW_SIZE;
    if (rollingCount < ROLLING_WINDOW_SIZE) rollingCount++;

    uint32_t sum = 0;
    for (uint8_t i = 0; i < rollingCount; ++i) sum += rollingBuffer[i];
    uint16_t wm2_avgVal = (sum + rollingCount / 2) / rollingCount;

    // // rolling average for wm2 raw value
    // rollingBuffer_raw[rollingIndex_raw] = wm2;
    // rollingIndex_raw = (rollingIndex_raw + 1) % ROLLING_WINDOW_SIZE;
    // if (rollingCount_raw < ROLLING_WINDOW_SIZE) rollingCount_raw++;

    
    // uint32_t sum_raw = 0;
    // for (uint8_t i = 0; i < rollingCount_raw; ++i) sum_raw += rollingBuffer_raw[i];
    // float avgVal_raw = (sum_raw + rollingCount_raw / 2) / rollingCount_raw;
    uint16_t val_max_999 = wm2_avgVal > 999 ? 999 : (uint16_t)round(wm2_avgVal);
    uint16_t wm2_avg_val__min_1__max_999 = val_max_999 < 1 ? 1 : val_max_999;

    Serial.printf("Lux:%7.0f  |  W/m² (1-999):%4u  |  W/m² raw: %4u \n", lux, wm2_avg_val__min_1__max_999, wm2_avgVal);
    showValue(wm2_avg_val__min_1__max_999);
    delay(200);
}
