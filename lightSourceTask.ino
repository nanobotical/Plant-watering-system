void lightSourceTask(void* pvParameters) { // Core 1
  // LEDC: 13-bit resolution => duty 0..8191
  const int LEDC_CH   = 0;
  const int LEDC_FREQ = 5000;
  const int LEDC_RES  = 13;
  ledcSetup(LEDC_CH, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(LIGHT_SOURCE, LEDC_CH);

  int lightAdc = 0;

  for (;;) {
    // Wait for newest ambient reading (ADC 0..4095). Timeout keeps task alive.
    if (xQueueReceive(lightLevelQueue, &lightAdc, pdMS_TO_TICKS(250)) == pdTRUE) {
      // Invert mapping: darker room (small ADC) => brighter lamp (large duty)
      int duty = 8191 - (lightAdc * 8191) / 4095;
      if (duty < 0) duty = 0; if (duty > 8191) duty = 8191;

      // Optional: bias around a target level from settings.lightLevel
      // If you want to bias, uncomment:
      // int target = 0;
      // if (xSemaphoreTake(configSemaphore, pdMS_TO_TICKS(2)) == pdTRUE) {
      //   target = settings.lightLevel; // 0..4095 expected
      //   xSemaphoreGive(configSemaphore);
      // }
      // duty = 8191 - ( ((lightAdc - target) + 4095) * 8191 ) / 8191; // simple center bias

      ledcWrite(LEDC_CH, duty);
    }
    // else: keep last duty
  }
}