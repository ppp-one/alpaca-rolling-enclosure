# Alpaca Rolling Roof Enclosure

Custom Arduino Opta PLC controller for rolling roof observatory automation that monitors weather conditions and controls the roof shutter via a Variable Frequency Drive (VFD) motor controller.

<!-- gif -->
![Alpaca Rolling Roof Enclosure](./videos/inside-opening.gif)
![Alpaca Rolling Roof Enclosure](./videos/outside-closing-reverse.gif)


## Purpose

This PLC acts as an automated rolling roof controller that:
- Monitors a rain/weather safety sensor
- Tracks roof position via fully open/closed proximity sensors
- Controls roof movement through RS-485 Modbus communication with a VFD motor (OMRON 3G3M1-AB007, 3-PH 1HP 80-B4-B14-2 0.75kW, 1:100 gear box, MIG encoder 120-19-4-7)
- Monitors a safety relay for emergency stop conditions (OMRON G9SE-201)
- Exposes an ASCOM Alpaca API for both dome control and safety monitoring

## How It Works

The PLC continuously monitors sensor inputs and provides motor control through two communication channels:

### Sensor Inputs (via interrupt-driven digital pins)
1. **Rain/Weather Sensor** (A2): *Not currently connected* - pin reserved for future weather sensor integration
2. **Fully Closed Sensor** (A5): Detects when roof is completely closed (HIGH when closed)
3. **Fully Open Sensor** (A4): Detects when roof is completely open (HIGH when open)
4. **Safety Relay** (A6): Emergency stop relay (LOW = activated/error, HIGH = normal)

### Motor Control (via RS-485 Modbus RTU)
- Communicates with VFD (Variable Frequency Drive) motor controller
- Uses Modbus RTU protocol at 9600 baud, Station ID 1
- Sets motor frequency to 15 Hz (1500 = 15.00 Hz in VFD units)
- Controls direction through digital outputs D0 (close) and D2 (open)

### Shutter Logic

The system tracks five possible roof states:
- **shutterOpen** (0): Roof fully open, detected by fully open sensor
- **shutterClosed** (1): Roof fully closed, detected by fully closed sensor
- **shutterOpening** (2): Roof moving toward open position
- **shutterClosing** (3): Roof moving toward closed position
- **shutterError** (4): Error state triggered by safety relay activation

**Safety Behavior:**
- **External Control**: Opening/closing commands come from the master PLC or OCS, which monitors overall observatory safety
- Safety relay activation immediately stops all movement and sets error state
- Proximity sensors automatically stop movement when limits are reached
- Direction LEDs (D0, D2) indicate current movement direction

### Network Architecture

The PLC hosts an ASCOM Alpaca API at `http://192.168.1.14/` providing:
- **Safety Monitor** endpoints: `/api/v1/safetymonitor/0/` (reports local sensor status only)
- **Dome Control** endpoints: `/api/v1/dome/0/`

The controller exposes standard ASCOM Alpaca dome and safety monitor interfaces, allowing integration with observatory automation software. **Note**: Overall observatory safety decisions, if OCS fails, are made by the master PLC (192.168.1.10), which controls when this roof should open or close based on weather conditions and other factors.

## Hardware Setup

### Controller
- **Arduino Opta PLC** with Ethernet connection

### Sensor Connections (Analog Inputs)
- **A2**: Reserved for future weather sensor (not currently connected)
- **A4**: Fully open proximity sensor (HIGH when roof fully open)
- **A5**: Fully closed proximity sensor (HIGH when roof fully closed)
- **A6**: Safety relay input (LOW = emergency stop activated)

### Motor Control (Digital Outputs)
- **D0**: Close direction signal to VFD
- **D2**: Open direction signal to VFD
- **LED_D0**: Close direction indicator LED
- **LED_D2**: Open direction indicator LED

### RS-485 Modbus Connection
- Connect Opta's RS-485 terminals to VFD motor controller
- **Baud Rate**: 9600
- **Parity**: None
- **Stop Bits**: 1
- **VFD Station ID**: 1
- **Frequency**: Set to 15 Hz for roof operation

## Compilation & Upload

Requires [arduino-cli](https://arduino.github.io/arduino-cli/) installed with the Arduino Mbed Opta core (custom core: https://github.com/ppp-one/ArduinoCore-mbed, or use the default Arduino Mbed core, however HTTP queries will be slower with the default core).

```bash
# Compile (adjust FQBN to your installed core)
arduino-cli compile --fqbn arduino-git:mbed:opta ./

# Upload to Opta PLC (replace port as needed)
arduino-cli upload -p /dev/cu.usbmodem2101 --fqbn arduino-git:mbed:opta ./
```

## Configuration

### Network Settings
Edit the IP address in `alpaca-rolling-enclosure.ino`:
```cpp
byte ip[] = {192, 168, 1, 14}; // PLC's IP address
```

The MAC address is automatically read from the Opta board's built-in memory.

### Motor Settings
VFD frequency is configured in the `setup()` function:
```cpp
// Set frequency to 15 Hz (1500 = 15.00 Hz in VFD units)
ModbusRTUClient.holdingRegisterWrite(SLAVE_ID, FIRST_FREQUENCY_REGISTER, 1500);
ModbusRTUClient.holdingRegisterWrite(SLAVE_ID, FREQUENCY_REGISTER, 1500);
```

Adjust the frequency value (1500) if your roof requires different motor speeds. The value represents frequency × 100 (e.g., 2000 = 20.00 Hz).

### Modbus Parameters
If your VFD uses different settings, modify these constants:
```cpp
const int SLAVE_ID = 1;               // VFD Station ID
const long BAUD_RATE = 9600;          // Communication speed
const int RESPONSE_TIMEOUT = 2000;    // Timeout in milliseconds
```

## ASCOM Alpaca API

The controller implements standard ASCOM Alpaca interfaces accessible at `http://192.168.1.14/api/v1/`.

### Safety Monitor Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/safetymonitor/0/connected` | GET/PUT | Check/set connection status |
| `/safetymonitor/0/issafe` | GET | Returns local safety status (currently based on A2 pin, not connected) |
| `/safetymonitor/0/name` | GET | Device name |
| `/safetymonitor/0/driverversion` | GET | Driver version (0.0.1) |

**Note**: The `/issafe` endpoint is implemented but not actively used since no rain sensor is connected. Observatory safety decisions are handled by the master PLC (192.168.1.10). This controller focuses on roof movement and limit detection.

### Dome Control Endpoints

#### Status & Capabilities
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/dome/0/connected` | GET/PUT | Connection status |
| `/dome/0/name` | GET | Device name |
| `/dome/0/atpark` | GET | Returns true if roof is closed |
| `/dome/0/shutterstatus` | GET | Roof status (0-4: open, closed, opening, closing, error) |
| `/dome/0/slewing` | GET | Returns true if roof is moving |
| `/dome/0/canpark` | GET | Returns true (roof can park/close) |
| `/dome/0/cansetshutter` | GET | Returns true (roof can be controlled) |
| `/dome/0/canfindhome` | GET | Returns false (no home position) |
| `/dome/0/canslave` | GET | Returns false (doesn't slave to telescope) |

#### Control Commands
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/dome/0/openshutter` | PUT | Open the roof (starts opening if not already open) |
| `/dome/0/closeshutter` | PUT | Close the roof (starts closing if not already closed) |
| `/dome/0/park` | PUT | Park the roof (same as close) |
| `/dome/0/abortslew` | PUT | Stop all roof movement immediately |

### Query Parameters
All requests support standard ASCOM parameters:
- `ClientID`: Numeric client identifier (optional)
- `ClientTransactionID`: Client transaction counter (optional)

### Response Format
All responses follow ASCOM JSON format:
```json
{
  "ClientID": 0,
  "ClientTransactionID": 0,
  "ServerID": 0,
  "ErrorNumber": 0,
  "ErrorMessage": "",
  "Value": <result>
}
```

## Safety Features

### Watchdog Timer
- 20-second watchdog timer ensures the system resets if code hangs
- Timer is "kicked" (reset) in every loop iteration
- Provides automatic recovery from software faults

### Interrupt-Driven Sensors
All sensors use hardware interrupts (`CHANGE` mode) for immediate response:
- **Proximity sensors**: Stops motor immediately when limits are reached
- **Safety relay**: Emergency stop with instant motor halt
- **Weather sensor pin** (A2): Monitored but not connected; reserved for future use

### Hardware Safety Relay
The safety relay (A6) provides a fail-safe mechanism:
- When activated (LOW), immediately stops all movement
- Sets shutter status to error state
- Requires external reset to resume operation
- Can be connected to external e-stop buttons or safety systems

### Movement Protection
- Roof won't open if already fully open (checked via sensor)
- Roof won't close if already fully closed (checked via sensor)
- Proximity sensors automatically stop movement at limits
- Direction outputs are mutually exclusive (can't activate both simultaneously)

## Maintenance

### Regular Checks
1. **Sensor Alignment**: Verify proximity sensors reliably detect open/closed positions
2. **Master PLC Communication**: Ensure the master PLC (192.168.1.10) can successfully send open/close commands
3. **Modbus Communication**: Check serial output for VFD communication errors
4. **LED Indicators**: Observe D0/D2 LEDs to confirm proper direction signaling

### Troubleshooting

**Roof won't move:**
- Check safety relay status (A6 should be HIGH)
- Verify VFD is powered and responding to Modbus commands
- Check serial monitor for "Failed to start Modbus RTU Client!" error
- Confirm proximity sensors aren't both triggered simultaneously

**Roof won't stop at limits:**
- Check proximity sensor wiring and positioning
- Verify sensors are triggering interrupts (test with serial debug)
- Ensure sensors provide stable HIGH signal when activated

**"Ethernet failed" message:**
- Verify Ethernet cable connection
- Check MAC address is being read correctly from Opta memory
- Confirm IP address doesn't conflict with other network devices

**Modbus errors:**
- Check RS-485 wiring (A/B terminals correctly connected)
- Verify VFD station ID matches `SLAVE_ID` constant
- Confirm VFD baud rate is set to 9600
- Check for proper RS-485 termination if cable is long

**Watchdog resets:**
- If system resets frequently, the main loop may be blocking
- Check for network requests that take longer than 20 seconds
- Increase `WATCHDOG_TIMEOUT` if necessary (currently 20000 ms)

### Serial Monitor
Connect to the Opta via USB and open serial monitor at 9600 baud to see:
- IP address assignment confirmation
- Modbus initialization status
- Frequency setting confirmations
- Any Modbus communication errors
- Incoming HTTP request paths

## Integration Examples

### Python with alpyca
```python
from alpaca.dome import Dome

dome = Dome('192.168.1.14:80', 0)
dome.Connected = True

# Open the roof
dome.OpenShutter()

# Wait for completion
while dome.Slewing:
    time.sleep(1)

# Close the roof
dome.CloseShutter()
```
## Code Structure for Developers

### Main Components

**Global Variables:**
- `shutterStatus`: Current roof state (0-4)
- `rainSensorSafe`, `fullyOpenSensor`, `fullyClosedSensor`, `safetyRelayActivated`: Sensor states
- `isSafe`: Local safety status variable (not actively used since no weather sensor is connected)

**Key Functions:**
- `open_shutter()`: Activates open direction output (D2) if safe
- `close_shutter()`: Activates close direction output (D0) if safe
- `stop_shutter()`: Deactivates both direction outputs
- `endPoint()`: Handles all ASCOM Alpaca HTTP requests

**Interrupt Service Routines (ISRs):**
- `rainSensorSafeISR()`: Monitors A2 pin (not currently used - no sensor connected)
- `fullyOpenSensorISR()`: Stops motor when fully open
- `fullyClosedSensorISR()`: Stops motor when fully closed
- `safetyRelayISR()`: Emergency stop on relay activation

**Setup Process:**
1. Initialize serial communication (9600 baud)
2. Start watchdog timer (20s)
3. Configure Ethernet with MAC from board memory
4. Configure digital I/O pins
5. Initialize all sensor states by calling ISRs once
6. Attach hardware interrupts to sensor pins
7. Register ASCOM Alpaca endpoint routes
8. Start Ethernet server on port 80
9. Initialize Modbus RTU client
10. Set VFD frequency to 15 Hz

**Main Loop:**
1. Check for incoming Ethernet clients
2. Process HTTP requests through aWOT framework
3. Kick watchdog timer
4. Repeat

### Adding New Features

**To add a new sensor:**
1. Define pin constant (e.g., `constexpr int newSensorPin = A7;`)
2. Create global variable (e.g., `volatile bool newSensorState;`)
3. Write ISR function to read and process sensor
4. Configure pin in `setup()`: `pinMode(newSensorPin, INPUT);`
5. Call ISR once in `setup()` to initialize state
6. Attach interrupt: `attachInterrupt(digitalPinToInterrupt(newSensorPin), newSensorISR, CHANGE);`

**To modify VFD parameters:**
- Locate Modbus register addresses in your VFD manual
- Update `FIRST_FREQUENCY_REGISTER`, `FREQUENCY_REGISTER`, or add new register constants
- Use `ModbusRTUClient.holdingRegisterWrite()` or `.coilWrite()` in `setup()` or control functions

**To add custom endpoints:**
1. Add new route in `setup()`: `app.get("/custom/endpoint", &endPoint);`
2. Add handler logic in `endPoint()` function
3. Parse URL path with `split(req.path(), '/', index)`

## License

MIT License - see [LICENSE](LICENSE)
 
