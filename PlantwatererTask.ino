// ============================ Plant Waterer: Servo Valve ============================
// Uses global `Servo actuator;` already attached to MOTOR in setup().
// Waters when humidity < settings.moisture AND tank level is OK (if sensor present).
// Blinks LED while watering. Includes watchdog + rest period.

void plantWatererTask(void* pvParameters) { // Core 1
  // ---- Tunables (adjust to your linkage) ----
  const int  VALVE_OPEN_ANGLE   = 90;     // servo angle that opens the valve
  const int  VALVE_CLOSE_ANGLE  = 0;      // servo angle that closes the valve
  const int  HUMIDITY_DEFAULT   = 60;     // %RH fallback if settings not readable
  const int  TANK_ADC_MIN       = 500;    // gate threshold for "tank OK" (tune)
  const int  WATER_MS_DEFAULT   = 1500;   // open-valve duration per cycle (ms)
  const int  MAX_WATER_MS       = 6000;   // hard cap watchdog (ms)
  const int  MIN_REST_MS        = 2000;   // minimum rest between cycles (ms)

  // If you don’t want tank gating, set to false.
  const bool USE_TANK_CHECK     = true;

  pinMode(LED_BLINKING, OUTPUT);
  digitalWrite(LED_BLINKING, LOW);
  actuator.write(VALVE_CLOSE_ANGLE); // ensure closed at start

  int rh = 0;              // humidity from humidityQueue (%RH)
  int tankAdc = 0;         // water level ADC from waterLevelQueue
  int moistThresh = HUMIDITY_DEFAULT;
  unsigned long lastWaterEndMs = 0;

  auto readMoistureThreshold = [&](){
    int thr = HUMIDITY_DEFAULT;
    if (xSemaphoreTake(configSemaphore, pdMS_TO_TICKS(5)) == pdTRUE) {
      thr = settings.moisture;
      xSemaphoreGive(configSemaphore);
    }
    return thr;
  };

  auto tankOkay = [&](int adc){
    if (!USE_TANK_CHECK) return true;
    return adc > TANK_ADC_MIN;
  };

  auto doseMs = [&](){
    // If you later add BLE command W=<ms>, return that here (guarded by configSemaphore).
    int ms = WATER_MS_DEFAULT;
    if (ms > MAX_WATER_MS) ms = MAX_WATER_MS;
    return ms;
  };

  auto valveOpen  = [&](){ actuator.write(VALVE_OPEN_ANGLE);  digitalWrite(LED_BLINKING, HIGH); };
  auto valveClose = [&](){ actuator.write(VALVE_CLOSE_ANGLE); digitalWrite(LED_BLINKING, LOW);  };

  for (;;) {
    // Block for latest humidity (~1 Hz producer)
    if (xQueueReceive(humidityQueue, &rh, pdMS_TO_TICKS(1200)) == pdTRUE) {
      // Drain water level queue non-blocking; keep newest
      int latest;
      while (xQueueReceive(waterLevelQueue, &latest, 0) == pdTRUE) tankAdc = latest;

      moistThresh = readMoistureThreshold();

      bool needWater = (rh < moistThresh);
      bool tankOK    = tankOkay(tankAdc);
      bool rested    = (millis() - lastWaterEndMs) >= (unsigned long)MIN_REST_MS;

      if (needWater && tankOK && rested) {
        int onMs = doseMs();

        valveOpen();
        // Watchdog-friendly wait (yield in small chunks)
        unsigned long start = millis();
        while ((millis() - start) < (unsigned long)onMs) {
          vTaskDelay(pdMS_TO_TICKS(50));
        }
        valveClose();

        lastWaterEndMs = millis();
      } else {
        // Ensure closed if not watering or conditions not met
        valveClose();
      }
    } else {
      // Stale humidity → fail-safe closed
      valveClose();
    }
  }
}