void LDRTask(void* pvParameters) { // Core 0
  // Default ~125 Hz unless overwritten by POT via frequencyQueue
  const int DEFAULT_MS = 8;                 // 8 ms ≈ 125 Hz
  const int MIN_MS     = 4;                 // safety floor
  const int MAX_MS     = 1000;              // safety ceiling

  int periodMs = DEFAULT_MS;
  int currLight = 0;

  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    // Drain frequencyQueue to keep only the newest period
    int newPeriod;
    while (xQueueReceive(frequencyQueue, &newPeriod, 0) == pdTRUE) {
      if (newPeriod < MIN_MS) newPeriod = MIN_MS;
      if (newPeriod > MAX_MS) newPeriod = MAX_MS;
      periodMs = newPeriod;
    }

    // Read ambient light (ADC 0..4095)
    currLight = analogRead(LDR);

    // Push to lightLevelQueue (latest wins): if full, drop oldest then push
    if (xQueueSend(lightLevelQueue, &currLight, 0) != pdTRUE) {
      int drop;
      xQueueReceive(lightLevelQueue, &drop, 0);
      xQueueSend(lightLevelQueue, &currLight, 0);
    }

    // Update moving average with a binary semaphore
    if (xSemaphoreTake(lightSemaphore, pdMS_TO_TICKS(2)) == pdTRUE) {
      addVal(currLight, (valueContainer*)&lightVals);
      xSemaphoreGive(lightSemaphore);
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(periodMs));
  }
}