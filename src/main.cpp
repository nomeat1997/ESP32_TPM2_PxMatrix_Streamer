#include "PxMatrix.h"
#include "TPM2.h"
#include "FastLED.h"

#define P_LAT 22
#define P_A 19
#define P_B 23
#define P_C 18
#define P_D 5
#define P_E 15
#define P_OE 16

#define CONFIG_FREERTOS_HZ 50000 //Default 1000

#define PxMATRIX_COLOR_DEPTH 8
#define PxMATRIX_SPI_FREQUENCY 20000000

#define MATRIX_WIDTH 32
#define MATRIX_HEIGHT 16
#define DISPLAY_DRAW_TIME 30           //10-50 is usually fine
#define TIMER_PRESCALER 240            //NodeMCU ESP32S runs at 240MHz by default.
#define REFRESH_RATE_COUNTER_VALUE 666 //Microseconds since 240MHz/240 = 1MHz

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

PxMATRIX display(MATRIX_WIDTH, MATRIX_HEIGHT, P_LAT, P_OE, P_A, P_B, P_C, P_D, P_E);

uint8_t buffer[MATRIX_WIDTH * MATRIX_HEIGHT * 3];
TPM2 tpm2Driver(&Serial, buffer, sizeof(buffer));

const uint8_t DRAM_ATTR gamma8_red[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2,
    2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5,
    5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10,
    10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
    17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
    25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
    37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
    51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
    69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
    90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
    115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142,
    144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175,
    177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213,
    215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255};

const uint8_t DRAM_ATTR gamma8_green[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1,
    1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3,
    3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 7, 7, 7, 7,
    7, 7, 8, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12,
    13, 13, 14, 14, 15, 15, 15, 15, 16, 16, 17, 17, 18, 19,
    19, 19, 19, 20, 21, 21, 22, 23, 23, 23, 24, 25, 25, 26,
    27, 27, 27, 28, 29, 30, 31, 31, 31, 32, 33, 34, 35, 35,
    36, 37, 38, 39, 39, 39, 40, 41, 42, 43, 44, 45, 46, 46,
    47, 48, 49, 50, 50, 52, 53, 54, 54, 55, 57, 58, 58, 59,
    61, 62, 62, 64, 65, 66, 67, 68, 69, 70, 71, 73, 74, 75,
    76, 78, 78, 80, 81, 82, 83, 85, 86, 87, 89, 90, 91, 93,
    94, 95, 97, 98, 100, 101, 102, 104, 105, 107, 109, 109, 111, 113, 114, 116,
    117, 119, 121, 122, 124, 125, 127, 128, 130, 132, 134, 136, 137, 139, 140,
    143, 144, 146, 148, 150, 152, 153, 156, 157, 159, 161, 163, 165, 167, 169,
    171, 173, 175, 177, 179, 181, 183, 185, 187, 190, 191, 194, 196, 198, 200, 203};

const uint8_t DRAM_ATTR gamma8_blue[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3,
    3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 6,
    6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 8, 8, 9, 9, 9, 9,
    10, 10, 10, 10, 11, 11, 12, 12, 12, 12, 13, 13, 13, 14, 14, 15,
    15, 15, 16, 16, 17, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 21,
    22, 23, 23, 23, 24, 24, 25, 26, 26, 27, 27, 28, 29, 29, 30, 30,
    31, 31, 32, 33, 34, 34, 35, 35, 36, 37, 37, 38, 38, 40, 40, 41,
    41, 42, 43, 44, 44, 45, 46, 47, 48, 49, 49, 50, 51, 52, 52, 54,
    54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 63, 65, 66, 66, 68, 69,
    69, 71, 72, 72, 74, 75, 76, 77, 78, 79, 80, 82, 83, 83, 85, 86,
    87, 88, 89, 91, 92, 93, 94, 96, 97, 98, 99, 101, 102, 103, 105, 106,
    107, 109, 110, 111, 113, 114, 116, 117, 119, 120, 121, 123, 124, 126, 127, 129,
    130, 132, 133, 135, 136, 138, 140, 141, 143, 145, 146, 148, 150, 151, 153, 155};

void IRAM_ATTR display_updater()
{
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  display.display(DISPLAY_DRAW_TIME);
  portEXIT_CRITICAL_ISR(&timerMux);
}

void display_update_enable(bool is_enable)
{
  if (is_enable)
  {
    timer = timerBegin(0, TIMER_PRESCALER, true);
    timerAttachInterrupt(timer, &display_updater, true);
    timerAlarmWrite(timer, REFRESH_RATE_COUNTER_VALUE, true);
    timerAlarmEnable(timer);
  }
  else
  {
    timerDetachInterrupt(timer);
    timerAlarmDisable(timer);
  }
}

void IRAM_ATTR LEDUpdateTask(void *pvParameters)
{
  uint16_t i = 0;
  for (uint8_t y = 0; y < MATRIX_HEIGHT; y++)
  {
    for (uint8_t x = 0; x < MATRIX_WIDTH; x++)
    {
      display.drawPixelRGB888(x, y, (buffer[i]), (buffer[i + 1]), (buffer[i + 2]));
      i += 3;
    }
  }
  vTaskDelete(NULL);
}

void IRAM_ATTR LEDUpdateGammaCorrectedTask(void *pvParameters)
{
  uint16_t i = 0;
  for (uint8_t y = 0; y < MATRIX_HEIGHT; y++)
  {
    for (uint8_t x = 0; x < MATRIX_WIDTH; x++)
    {
      display.drawPixelRGB888(x, y, gamma8_red[buffer[i]], gamma8_green[buffer[i + 1]], gamma8_blue[buffer[i + 2]]);
      i += 3;
    }
  }
  vTaskDelete(NULL);
}

void IRAM_ATTR LEDUpdateCallbackGammaCorrected(uint8_t *data, uint16_t len)
{
  xTaskCreatePinnedToCore(LEDUpdateGammaCorrectedTask, "LEDUpdateGammaCorrectedTask", 1024, NULL, 6, NULL, 0);
}

void IRAM_ATTR LEDUpdateCallback(uint8_t *data, uint16_t len)
{
  xTaskCreatePinnedToCore(LEDUpdateTask, "LEDUpdateTask", 1024, NULL, 6, NULL, 0);
}

void setup()
{
  display.begin(8);
  display.clearDisplay();
  display.setBrightness(255);
  display.setFastUpdate(true);
  display.setDriverChip(SHIFT);
  display.setColorOrder(BBGGRR);
  display_update_enable(true);

  pinMode(4, INPUT_PULLUP);
  uint8_t gammaCorrectionDisabled = digitalRead(4);

  Serial.begin(1000000);
  Serial.setRxBufferSize(2048);

  if (gammaCorrectionDisabled)
    tpm2Driver.registerRxData(LEDUpdateCallback);
  else
    tpm2Driver.registerRxData(LEDUpdateCallbackGammaCorrected);
}

void loop()
{
  tpm2Driver.update();
}
