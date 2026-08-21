/**
 * Read ADC values from ZONESTAR LCD buttons on MKS TinyBee
 * Connect LCD to EXP1, button ADC is on IO35 (GPIO35 / ADC1_CH7)
 * 
 * Open Serial Monitor at 115200 baud and press each button.
 * Note the values for: UP, DOWN, LEFT, RIGHT, ENTER (CENTER)
 * 
 * IMPORTANT: Marlin's ESP32 HAL scales the 12-bit ADC to 10-bit (0-1023).
 * The "10-bit scaled" column shows the value that Marlin actually uses
 * for button detection. Use these 10-bit values for configuration.
 */

#define ADC_PIN 35  // ADC_KEYPAD_PIN for MKS TinyBee

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ZONESTAR LCD Button ADC Reader ===");
  Serial.println("Press each button and note the values:");
  Serial.println("  UP, DOWN, LEFT, RIGHT, ENTER (CENTER)");
  Serial.println("Also note the value when NO button is pressed.");
  Serial.println("");
  Serial.println("  RAW (12-bit) | 10-bit scaled | Voltage");
  Serial.println("  (0-4095)     | (0-1023)      | (V)");
  Serial.println("==========================================\n");
}

void loop() {
  int rawValue = analogRead(ADC_PIN);
  
  // ESP32 ADC is 12-bit (0-4095)
  // Marlin scales to 10-bit: HAL_adc_result = mv * 1023.0 / 3300.0
  // Which is approximately: 10bit = raw * 1023 / 4095
  int scaled10bit = rawValue * 1023 / 4095;
  
  // Convert to voltage (3.3V reference)
  float voltage = (rawValue / 4095.0) * 3.3;
  
  static int lastRaw = -1;
  static unsigned long lastPrint = 0;
  
  // Print only when value changes or every 500ms
  if (abs(rawValue - lastRaw) > 2 || millis() - lastPrint > 500) {
    Serial.print("  ");
    Serial.print(rawValue);
    Serial.print("           ");
    Serial.print(scaled10bit);
    Serial.print("           ");
    Serial.print(voltage, 3);
    Serial.println(" V");
    
    lastRaw = rawValue;
    lastPrint = millis();
  }
  
  delay(50);
}