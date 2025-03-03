#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

// Modbus communication parameters from the images
const int SLAVE_ID = 1;               // RS-485 Communication Station No.
const long BAUD_RATE = 9600;          // Baud Rate from image
const int RESPONSE_TIMEOUT = 2000;    // Error Detection Timer value (2.0s) in milliseconds
const int SERIAL_CONFIG = SERIAL_8N1; // None parity, 1 stop bit from image

// RS485
constexpr auto baudrate{9600};
// Calculate preDelay and postDelay in microseconds for stable RS-485 transmission
constexpr auto bitduration{1.f / baudrate};
constexpr auto wordlen{10.0f}; // OR 10.0f depending on the channel configuration
constexpr auto preDelayBR{bitduration * wordlen * 3.5f * 1e6};
constexpr auto postDelayBR{bitduration * wordlen * 3.5f * 1e6};

void setup()
{
  Serial.begin(9600); // Debug serial

  // Initialize Modbus RTU client
  RS485.setDelays(preDelayBR, postDelayBR);
  if (!ModbusRTUClient.begin(BAUD_RATE, SERIAL_CONFIG))
  {
    Serial.println("Failed to start Modbus RTU Client!");
    while (1)
      ;
  }

  // Set response timeout from image (2.0 seconds)
  ModbusRTUClient.setTimeout(RESPONSE_TIMEOUT);
}

void loop()
{
  // Set frequency to 25Hz (2500 = 25.00Hz)
  Serial.println("Setting frequency to 25Hz");
  // ModbusRTUClient.holdingRegisterWrite(SLAVE_ID, 0x1216, 2500);
  ModbusRTUClient.holdingRegisterWrite(SLAVE_ID, 0x5006, 2500);
  delay(1000);

  // Forward run
  Serial.println("Starting forward run");
  // ModbusRTUClient.coilWrite(SLAVE_ID, 0x0000, 0x0000);
  ModbusRTUClient.holdingRegisterWrite(SLAVE_ID, 0x5007, 0x0001); // Set bit 0 (FW)
  delay(5000);

  // Stop
  Serial.println("Stopping");
  // ModbusRTUClient.coilWrite(SLAVE_ID, 0x0000, 0x0000);
  ModbusRTUClient.holdingRegisterWrite(SLAVE_ID, 0x5007, 0x0000);
  delay(5000);

  // Set frequency to 5Hz (500 = 5.00Hz)
  Serial.println("Setting frequency to 5Hz");
  // ModbusRTUClient.holdingRegisterWrite(SLAVE_ID, 0x1216, 500);
  ModbusRTUClient.holdingRegisterWrite(SLAVE_ID, 0x5006, 500);
  delay(1000);

  // Reverse run
  Serial.println("Starting reverse run");
  // ModbusRTUClient.coilWrite(SLAVE_ID, 0x0001, 0xFF00);
  ModbusRTUClient.holdingRegisterWrite(SLAVE_ID, 0x5007, 0x0002); // Set bit 1 (RV)
  delay(5000);

  // Stop
  Serial.println("Stopping again");
  // ModbusRTUClient.coilWrite(SLAVE_ID, 0x0001, 0x0000);
  ModbusRTUClient.holdingRegisterWrite(SLAVE_ID, 0x5007, 0x0000);
  delay(5000);
}
