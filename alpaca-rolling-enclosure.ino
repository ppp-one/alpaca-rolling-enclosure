#include <ArduinoModbus.h>
#include <ArduinoRS485.h>

#define SLAVE_ID 1
#define FORWARD_COIL 0x0001
#define REVERSE_COIL 0x0002
#define FREQUENCY_REGISTER 0x1217

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Start Modbus RTU client
  if (!ModbusRTUClient.begin(9600)) {
    Serial.println("Failed to start Modbus RTU Client!");
    while (1);
  }
}

void loop() {
  // Move enclosure forwards
  moveEnclosure(true);
  delay(5000);  // Run for 5 seconds

  // Stop
  stopEnclosure();
  delay(2000);  // Pause for 2 seconds

  // Move enclosure backwards
  moveEnclosure(false);
  delay(5000);  // Run for 5 seconds

  // Stop
  stopEnclosure();
  delay(2000);  // Pause for 2 seconds
}

void moveEnclosure(bool forward) {
  uint16_t coil = forward ? FORWARD_COIL : REVERSE_COIL;
  
  // Set direction
  if (!ModbusRTUClient.coilWrite(SLAVE_ID, coil, true)) {
    Serial.print("Failed to set direction. Error: ");
    Serial.println(ModbusRTUClient.lastError());
  }

  // Set frequency to 50 Hz (5000 = 50.00 Hz)
  if (!ModbusRTUClient.holdingRegisterWrite(SLAVE_ID, FREQUENCY_REGISTER, 5000)) {
    Serial.print("Failed to set frequency. Error: ");
    Serial.println(ModbusRTUClient.lastError());
  }
}

void stopEnclosure() {
  // Turn off both forward and reverse coils
  ModbusRTUClient.coilWrite(SLAVE_ID, FORWARD_COIL, false);
  ModbusRTUClient.coilWrite(SLAVE_ID, REVERSE_COIL, false);
  
  // Set frequency to 0
  ModbusRTUClient.holdingRegisterWrite(SLAVE_ID, FREQUENCY_REGISTER, 0);
}
