#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <aWOT.h>
#include <strings.h> // for strncasecmp
#include "opta_info.h"
#include <drivers/Watchdog.h>

OptaBoardInfo *info;
OptaBoardInfo *boardInfo();

using namespace mbed;

// #define SLAVE_ID 1
#define FORWARD_COIL 0x0001
#define REVERSE_COIL 0x0002
#define FIRST_FREQUENCY_REGISTER 0x1216
#define FREQUENCY_REGISTER 0x5006

// Modbus communication parameters
const int SLAVE_ID = 1;               // RS-485 Communication Station No.
const long BAUD_RATE = 9600;          // Baud Rate from image
const int RESPONSE_TIMEOUT = 2000;    // Error Detection Timer value (2.0s) in milliseconds
const int SERIAL_CONFIG = SERIAL_8N1; // None parity, 1 stop bit from image
const unsigned long WATCHDOG_TIMEOUT = 20000; // 20 seconds watchdog timeout

// Ethernet.begin waits this long for a link, must stay well below WATCHDOG_TIMEOUT
const unsigned long ETHERNET_TIMEOUT = 10000;

// pause between stopping the motor and driving it the other way
const unsigned long REVERSE_DELAY = 500;

// stop if the roof has not left the limit it started from in this time
// TODO: measure the real time to clear the sensor and set this to about 3x that
const unsigned long NO_START_TIMEOUT = 20000;

// stop if a move takes longer than this, for a failed limit sensor or a jam
const unsigned long TRAVEL_TIMEOUT = 180000;

// Debug flag - set to true for Serial debugging. Serial writes block while a
// host has the USB port open but does not read it, so keep this off in use.
const bool DEBUG = false;

// Calculate preDelay and postDelay in microseconds for stable RS-485 transmission
constexpr auto bitduration{1.f / BAUD_RATE};
constexpr auto wordlen{10.0f}; // OR 10.0f depending on the channel configuration
constexpr auto preDelayBR{bitduration * wordlen * 3.5f * 1e6};
constexpr auto postDelayBR{bitduration * wordlen * 3.5f * 1e6};

// struct for possible shutter statuses
struct ShutterStatuses
{
  static const int shutterOpen = 0;
  static const int shutterClosed = 1;
  static const int shutterOpening = 2;
  static const int shutterClosing = 3;
  static const int shutterError = 4;
};

// motor direction, written by the shutter functions and the ISRs
enum MotorDirection
{
  MOTOR_STOPPED,
  MOTOR_OPENING,
  MOTOR_CLOSING
};
volatile MotorDirection motorDirection = MOTOR_STOPPED;

// motor start time and whether it started at the opposite limit, for the
// timeouts in check_limits. Only used by the loop thread.
unsigned long motorStartTime = 0;
bool motorStartedAtLimit = false;

// set when a timeout stops the motor, shutter status is then shutterError.
// It does not block commands, the next open or close clears it.
const char *motorFault = NULL;

// declared early for the prototypes the Arduino builder adds
struct AlpacaReply;

// ASCOM connected state. Commands are not refused while disconnected,
// because the master PLC sends closeshutter without connecting.
// It starts connected, because Astra connects only once at its own startup
// and must not see a disconnected device after a watchdog reset.
bool domeConnected = true;

// Define the pins for the sensors
// constexpr int doorSensorPin = A3;
constexpr int fullyOpenSensorPin = A4;
constexpr int fullyClosedSensorPin = A5;
constexpr int safetyRelayPin = A6;
// constexpr int vfdStatusPin = A7;

// Direction pins
constexpr int CLOSE_DIR = D0;
constexpr int CLOSE_DIR_LED = LED_D0;
constexpr int OPEN_DIR = D2;
constexpr int OPEN_DIR_LED = LED_D2;

// Alpaca error numbers
constexpr int32_t ERROR_NOT_IMPLEMENTED = 0x400;
constexpr int32_t ERROR_ACTION_NOT_IMPLEMENTED = 0x40C;
constexpr int32_t ERROR_DRIVER = 0x500;

const char *DRIVER_VERSION = "0.2.0";
const char *DEVICE_NAME = "Rolling roof by PPP";
const unsigned int ALPACA_PORT = 80;
const unsigned int DISCOVERY_PORT = 32227;

// incremented on every Alpaca response, only used by the loop thread
uint32_t serverTransactionID = 0;

// the media access control (ethernet hardware) address
byte mac[6];
// the IP address
byte ip[] = {192, 168, 1, 14}; // TODO: can this set more easily by the user - DHCP first then fixed?

// unique ID for the management API, filled from the MAC address in setup
char domeUniqueID[40];

// Create an Ethernet server
EthernetServer server(ALPACA_PORT);
EthernetUDP discoveryUdp;
Application app;

void debugPrint(const char *message)
{
  if (DEBUG)
  {
    Serial.println(message);
  }
}

// Sensors are read directly, so the state is never stale
bool fully_open()
{
  return digitalRead(fullyOpenSensorPin) == HIGH;
}

bool fully_closed()
{
  return digitalRead(fullyClosedSensorPin) == HIGH;
}

bool safety_relay_activated()
{
  return digitalRead(safetyRelayPin) == LOW;
}

void stop_shutter()
{
  digitalWrite(OPEN_DIR, LOW);
  digitalWrite(OPEN_DIR_LED, LOW);
  digitalWrite(CLOSE_DIR, LOW);
  digitalWrite(CLOSE_DIR_LED, LOW);
  motorDirection = MOTOR_STOPPED;
}

// Returns NULL on success, or an error message
const char *close_shutter()
{
  if (safety_relay_activated())
  {
    return "Safety relay is activated";
  }
  if (fully_open() && fully_closed())
  {
    return "Both limit sensors are active";
  }

  motorFault = NULL;

  if (motorDirection == MOTOR_OPENING)
  {
    stop_shutter();
    delay(REVERSE_DELAY);
  }

  // check and start without interrupts, so a limit ISR cannot run between
  // the sensor check and the output write
  noInterrupts();
  if (!fully_closed() && !safety_relay_activated() && motorDirection != MOTOR_CLOSING)
  {
    // release the other direction first, so both outputs are never on together
    digitalWrite(OPEN_DIR, LOW);
    digitalWrite(OPEN_DIR_LED, LOW);
    digitalWrite(CLOSE_DIR, HIGH);
    digitalWrite(CLOSE_DIR_LED, HIGH);
    motorDirection = MOTOR_CLOSING;
    motorStartTime = millis();
    motorStartedAtLimit = fully_open();
  }
  interrupts();

  return NULL;
}

// Returns NULL on success, or an error message
const char *open_shutter()
{
  if (safety_relay_activated())
  {
    return "Safety relay is activated";
  }
  if (fully_open() && fully_closed())
  {
    return "Both limit sensors are active";
  }

  motorFault = NULL;

  if (motorDirection == MOTOR_CLOSING)
  {
    stop_shutter();
    delay(REVERSE_DELAY);
  }

  noInterrupts();
  if (!fully_open() && !safety_relay_activated() && motorDirection != MOTOR_OPENING)
  {
    digitalWrite(CLOSE_DIR, LOW);
    digitalWrite(CLOSE_DIR_LED, LOW);
    digitalWrite(OPEN_DIR, HIGH);
    digitalWrite(OPEN_DIR_LED, HIGH);
    motorDirection = MOTOR_OPENING;
    motorStartTime = millis();
    motorStartedAtLimit = fully_closed();
  }
  interrupts();

  return NULL;
}

// Shutter status from the motor direction and the sensors.
// Stopped between the limits means the position is unknown, which is an error.
int shutter_status()
{
  bool open = fully_open();
  bool closed = fully_closed();
  MotorDirection direction = motorDirection;

  if (safety_relay_activated() || (open && closed) || motorFault != NULL)
  {
    return ShutterStatuses::shutterError;
  }
  if (direction == MOTOR_OPENING)
  {
    return ShutterStatuses::shutterOpening;
  }
  if (direction == MOTOR_CLOSING)
  {
    return ShutterStatuses::shutterClosing;
  }
  if (closed)
  {
    return ShutterStatuses::shutterClosed;
  }
  if (open)
  {
    return ShutterStatuses::shutterOpen;
  }
  return ShutterStatuses::shutterError;
}

// ISR for fully open sensor. Only stop when driving into this limit, so a
// bouncing sensor cannot stop a close that has just started.
void fullyOpenSensorISR()
{
  if (motorDirection == MOTOR_OPENING && fully_open())
  {
    stop_shutter();
  }
}

// ISR for fully closed sensor
void fullyClosedSensorISR()
{
  if (motorDirection == MOTOR_CLOSING && fully_closed())
  {
    stop_shutter();
  }
}

// ISR for safety relay
void safetyRelayISR()
{
  if (safety_relay_activated())
  {
    stop_shutter();
  }
}

// Polled backup for the ISRs, in case an edge is missed, and the motor timeouts
void check_limits()
{
  MotorDirection direction = motorDirection;
  if (direction == MOTOR_STOPPED)
  {
    return;
  }

  if (safety_relay_activated() ||
      (direction == MOTOR_OPENING && fully_open()) ||
      (direction == MOTOR_CLOSING && fully_closed()))
  {
    stop_shutter();
    return;
  }

  unsigned long running = millis() - motorStartTime;

  // the roof must leave the limit it started from, or the VFD did not start
  bool stillAtStart = direction == MOTOR_OPENING ? fully_closed() : fully_open();
  if (motorStartedAtLimit && stillAtStart && running > NO_START_TIMEOUT)
  {
    stop_shutter();
    motorFault = "Roof did not leave the limit sensor, motor stopped";
    debugPrint(motorFault);
    return;
  }

  if (running > TRAVEL_TIMEOUT)
  {
    stop_shutter();
    motorFault = "Roof did not reach the limit sensor in time, motor stopped";
    debugPrint(motorFault);
  }
}

// Alpaca parameter names are case insensitive. GET parameters are in the
// query string, PUT parameters are in the form encoded body.
void read_params(Request &req, char *params, int length)
{
  params[0] = '\0';

  if (req.method() == Request::GET)
  {
    strncpy(params, req.query(), length - 1);
    params[length - 1] = '\0';
  }
  else
  {
    int toRead = min(req.left(), length - 1);
    int count = req.readBytes(params, toRead);
    params[count] = '\0';
  }
}

bool find_param(const char *params, const char *name, char *value, int valueLength)
{
  size_t nameLength = strlen(name);
  const char *position = params;

  while (*position)
  {
    const char *end = strchr(position, '&');
    if (end == NULL)
    {
      end = position + strlen(position);
    }

    const char *equals = (const char *)memchr(position, '=', end - position);
    if (equals != NULL && (size_t)(equals - position) == nameLength && strncasecmp(position, name, nameLength) == 0)
    {
      int count = min((int)(end - equals - 1), valueLength - 1);
      memcpy(value, equals + 1, count);
      value[count] = '\0';
      return true;
    }

    position = *end ? end + 1 : end;
  }

  return false;
}

// Returns true if the parameter is "true" or "false", and sets result
bool find_bool_param(const char *params, const char *name, bool &result)
{
  char value[8];
  if (!find_param(params, name, value, sizeof(value)))
  {
    return false;
  }
  if (strcasecmp(value, "true") == 0)
  {
    result = true;
    return true;
  }
  if (strcasecmp(value, "false") == 0)
  {
    result = false;
    return true;
  }
  return false;
}

// Reply to one Alpaca request
struct AlpacaReply
{
  enum Type
  {
    NONE,  // method without a return value
    BOOL,
    INT,
    STRING,
    JSON   // value printed as is, for arrays and objects
  };

  Type type = NONE;
  bool boolValue = false;
  int32_t intValue = 0;
  const char *text = "";
  int32_t errorNumber = 0;
  const char *errorMessage = "";
  bool badRequest = false;

  void setBool(bool value)
  {
    type = BOOL;
    boolValue = value;
  }

  void setInt(int32_t value)
  {
    type = INT;
    intValue = value;
  }

  void setString(const char *value)
  {
    type = STRING;
    text = value;
  }

  void setJson(const char *value)
  {
    type = JSON;
    text = value;
  }

  void setError(int32_t number, const char *message)
  {
    errorNumber = number;
    errorMessage = message;
  }

  void setBadRequest(const char *message)
  {
    badRequest = true;
    errorMessage = message;
  }
};

void send_reply(Response &res, const char *params, AlpacaReply &reply)
{
  if (reply.badRequest)
  {
    res.status(400);
    res.set("Content-Type", "text/plain");
    res.print(reply.errorMessage);
    res.end();
    return;
  }

  char buffer[16];
  uint32_t clientTransactionID = 0;
  if (find_param(params, "ClientTransactionID", buffer, sizeof(buffer)))
  {
    clientTransactionID = strtoul(buffer, NULL, 10);
  }

  res.set("Content-Type", "application/json");
  res.print("{\"ClientTransactionID\":");
  res.print(clientTransactionID);
  res.print(",\"ServerTransactionID\":");
  res.print(++serverTransactionID);
  res.print(",\"ErrorNumber\":");
  res.print(reply.errorNumber);
  res.print(",\"ErrorMessage\":\"");
  res.print(reply.errorMessage);
  res.print("\"");

  // a value is only sent when there is no error
  if (reply.errorNumber == 0 && reply.type != AlpacaReply::NONE)
  {
    res.print(",\"Value\":");
    switch (reply.type)
    {
    case AlpacaReply::BOOL:
      res.print(reply.boolValue ? "true" : "false");
      break;
    case AlpacaReply::INT:
      res.print(reply.intValue);
      break;
    case AlpacaReply::STRING:
      res.print("\"");
      res.print(reply.text);
      res.print("\"");
      break;
    default:
      res.print(reply.text);
      break;
    }
  }

  res.print("}");
  res.end();
}

// Members common to all device types. Returns false if the member is unknown.
bool common_member(const char *member, bool isPut, const char *params, bool &connected,
                   const char *description, const char *deviceState, AlpacaReply &reply)
{
  if (isPut)
  {
    if (strcmp(member, "connected") == 0)
    {
      bool value;
      if (!find_bool_param(params, "Connected", value))
      {
        reply.setBadRequest("Connected parameter missing or invalid");
      }
      else
      {
        connected = value;
      }
    }
    else if (strcmp(member, "connect") == 0)
    {
      connected = true;
    }
    else if (strcmp(member, "disconnect") == 0)
    {
      connected = false;
    }
    else if (strcmp(member, "action") == 0)
    {
      reply.setError(ERROR_ACTION_NOT_IMPLEMENTED, "No actions are supported");
    }
    else if (strcmp(member, "commandblind") == 0 || strcmp(member, "commandbool") == 0 || strcmp(member, "commandstring") == 0)
    {
      reply.setError(ERROR_NOT_IMPLEMENTED, "Commands are not implemented");
    }
    else
    {
      return false;
    }
    return true;
  }

  if (strcmp(member, "connected") == 0)
  {
    reply.setBool(connected);
  }
  else if (strcmp(member, "connecting") == 0)
  {
    reply.setBool(false);
  }
  else if (strcmp(member, "name") == 0)
  {
    reply.setString(DEVICE_NAME);
  }
  else if (strcmp(member, "description") == 0)
  {
    reply.setString(description);
  }
  else if (strcmp(member, "driverinfo") == 0)
  {
    reply.setString("Arduino Opta rolling roof controller");
  }
  else if (strcmp(member, "driverversion") == 0)
  {
    reply.setString(DRIVER_VERSION);
  }
  else if (strcmp(member, "interfaceversion") == 0)
  {
    reply.setInt(3);
  }
  else if (strcmp(member, "supportedactions") == 0)
  {
    reply.setJson("[]");
  }
  else if (strcmp(member, "devicestate") == 0)
  {
    reply.setJson(deviceState);
  }
  else
  {
    return false;
  }
  return true;
}

// Handle the dome endpoint requests
void domeEndPoint(Request &req, Response &res)
{
  char member[32];
  char params[256];
  req.route("member", member, sizeof(member));
  read_params(req, params, sizeof(params));
  bool isPut = req.method() == Request::PUT;

  if (DEBUG)
  {
    Serial.print("Dome request: ");
    Serial.println(member);
  }

  int status = shutter_status();
  char deviceState[128];
  snprintf(deviceState, sizeof(deviceState),
           "[{\"Name\":\"AtPark\",\"Value\":%s},{\"Name\":\"ShutterStatus\",\"Value\":%d},{\"Name\":\"Slewing\",\"Value\":%s}]",
           status == ShutterStatuses::shutterClosed ? "true" : "false",
           status,
           motorDirection != MOTOR_STOPPED ? "true" : "false");

  AlpacaReply reply;

  if (common_member(member, isPut, params, domeConnected, "Rolling roof enclosure", deviceState, reply))
  {
    // handled
  }
  else if (isPut)
  {
    if (strcmp(member, "abortslew") == 0)
    {
      stop_shutter();
    }
    else if (strcmp(member, "closeshutter") == 0 || strcmp(member, "park") == 0)
    {
      const char *error = close_shutter();
      if (error)
      {
        reply.setError(ERROR_DRIVER, error);
      }
    }
    else if (strcmp(member, "openshutter") == 0)
    {
      const char *error = open_shutter();
      if (error)
      {
        reply.setError(ERROR_DRIVER, error);
      }
    }
    else if (strcmp(member, "findhome") == 0 || strcmp(member, "setpark") == 0 ||
             strcmp(member, "slewtoaltitude") == 0 || strcmp(member, "slewtoazimuth") == 0 ||
             strcmp(member, "synctoazimuth") == 0 || strcmp(member, "slaved") == 0)
    {
      reply.setError(ERROR_NOT_IMPLEMENTED, "Not implemented by a rolling roof");
    }
    else
    {
      reply.setBadRequest("Unknown dome member");
    }
  }
  else
  {
    if (strcmp(member, "atpark") == 0)
    {
      reply.setBool(status == ShutterStatuses::shutterClosed);
    }
    else if (strcmp(member, "shutterstatus") == 0)
    {
      reply.setInt(status);
    }
    else if (strcmp(member, "slewing") == 0)
    {
      reply.setBool(motorDirection != MOTOR_STOPPED);
    }
    else if (strcmp(member, "slaved") == 0)
    {
      reply.setBool(false);
    }
    else if (strcmp(member, "canpark") == 0 || strcmp(member, "cansetshutter") == 0)
    {
      reply.setBool(true);
    }
    else if (strcmp(member, "canfindhome") == 0 || strcmp(member, "cansetaltitude") == 0 ||
             strcmp(member, "cansetazimuth") == 0 || strcmp(member, "cansetpark") == 0 ||
             strcmp(member, "canslave") == 0 || strcmp(member, "cansyncazimuth") == 0)
    {
      reply.setBool(false);
    }
    else if (strcmp(member, "altitude") == 0 || strcmp(member, "azimuth") == 0 || strcmp(member, "athome") == 0)
    {
      reply.setError(ERROR_NOT_IMPLEMENTED, "Not implemented by a rolling roof");
    }
    else
    {
      reply.setBadRequest("Unknown dome member");
    }
  }

  send_reply(res, params, reply);
}

// Management API
void apiVersions(Request &req, Response &res)
{
  char params[256];
  read_params(req, params, sizeof(params));
  AlpacaReply reply;
  reply.setJson("[1]");
  send_reply(res, params, reply);
}

void managementDescription(Request &req, Response &res)
{
  char params[256];
  read_params(req, params, sizeof(params));
  char description[160];
  snprintf(description, sizeof(description),
           "{\"ServerName\":\"Alpaca rolling enclosure\",\"Manufacturer\":\"PPP\",\"ManufacturerVersion\":\"%s\",\"Location\":\"Observatory\"}",
           DRIVER_VERSION);
  AlpacaReply reply;
  reply.setJson(description);
  send_reply(res, params, reply);
}

void configuredDevices(Request &req, Response &res)
{
  char params[256];
  read_params(req, params, sizeof(params));
  char devices[160];
  snprintf(devices, sizeof(devices),
           "[{\"DeviceName\":\"%s\",\"DeviceType\":\"Dome\",\"DeviceNumber\":0,\"UniqueID\":\"%s\"}]",
           DEVICE_NAME, domeUniqueID);
  AlpacaReply reply;
  reply.setJson(devices);
  send_reply(res, params, reply);
}

// Runs last, for any request no route has answered
void badRequest(Request &req, Response &res)
{
  res.status(400);
  res.set("Content-Type", "text/plain");
  res.print("Unknown endpoint");
  res.end();
}

// Answer Alpaca discovery broadcasts with the HTTP port
void handle_discovery()
{
  int size = discoveryUdp.parsePacket();
  if (size <= 0)
  {
    return;
  }

  char packet[32] = {0};
  discoveryUdp.read(packet, min(size, (int)sizeof(packet) - 1));

  if (strncmp(packet, "alpacadiscovery", 15) == 0)
  {
    char reply[24];
    snprintf(reply, sizeof(reply), "{\"AlpacaPort\":%u}", ALPACA_PORT);
    discoveryUdp.beginPacket(discoveryUdp.remoteIP(), discoveryUdp.remotePort());
    discoveryUdp.write((const uint8_t *)reply, strlen(reply));
    discoveryUdp.endPacket();
  }
}

// setup
void setup()
{
  // set the direction outputs low before anything else
  pinMode(OPEN_DIR, OUTPUT);
  pinMode(OPEN_DIR_LED, OUTPUT);
  pinMode(CLOSE_DIR, OUTPUT);
  pinMode(CLOSE_DIR_LED, OUTPUT);
  stop_shutter();

  Serial.begin(9600);

  // wait for terminal to come up
  delay(1000);

  // start the watchdog
  mbed::Watchdog::get_instance().start(WATCHDOG_TIMEOUT);

  // set pins for sensors, before attachInterrupt so the ISRs never allocate
  // pinMode(doorSensorPin, INPUT);        // door sensor
  pinMode(fullyOpenSensorPin, INPUT);   // fully open sensor
  pinMode(fullyClosedSensorPin, INPUT); // fully closed sensor
  pinMode(safetyRelayPin, INPUT);       // safety relay
  // pinMode(vfdStatusPin, INPUT_PULLUP);  // VFD status

  // Attach interrupts to the sensors
  attachInterrupt(digitalPinToInterrupt(fullyOpenSensorPin), fullyOpenSensorISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(fullyClosedSensorPin), fullyClosedSensorISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(safetyRelayPin), safetyRelayISR, CHANGE);

  info = boardInfo();
  if (info->magic == 0xB5)
  {
    for (int i = 0; i < 6; i++)
    {
      mac[i] = info->mac_address[i];
    }
  }

  snprintf(domeUniqueID, sizeof(domeUniqueID), "%02x%02x%02x%02x-%02x%02x-4000-8000-000000000001",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  // the default Ethernet.begin waits 60 s for a link, longer than the watchdog
  IPAddress localIP(ip);
  IPAddress gateway(ip[0], ip[1], ip[2], 1);
  IPAddress subnet(255, 255, 255, 0);
  if (Ethernet.begin(mac, localIP, gateway, gateway, subnet, ETHERNET_TIMEOUT))
  {
    Serial.println(Ethernet.localIP());
  }
  else
  {
    // no link yet, restart and try again. The motor outputs are off.
    Serial.println("Ethernet failed, restarting");
    delay(100);
    NVIC_SystemReset();
  }

  // endpoints
  app.get("/api/v1/dome/0/:member", &domeEndPoint);
  app.put("/api/v1/dome/0/:member", &domeEndPoint);

  app.get("/management/apiversions", &apiVersions);
  app.get("/management/v1/description", &managementDescription);
  app.get("/management/v1/configureddevices", &configuredDevices);

  // must be registered last
  app.use(&badRequest);

  server.begin();
  discoveryUdp.begin(DISCOVERY_PORT);

  mbed::Watchdog::get_instance().kick();

  // Initialize Modbus RTU client
  RS485.setDelays(preDelayBR, postDelayBR);
  if (!ModbusRTUClient.begin(BAUD_RATE, SERIAL_CONFIG))
  {
    // continue without it, the VFD keeps its last frequency setting
    Serial.println("Failed to start Modbus RTU Client!");
    return;
  }

  // Set response timeout from image (2.0 seconds)
  ModbusRTUClient.setTimeout(RESPONSE_TIMEOUT);

  delay(1000); // Wait for the Modbus RTU client to initialize
  Serial.println("Modbus RTU Client initialized successfully!");

  // Set frequency to 15 Hz (1500 = 15.00 Hz)
  if (!ModbusRTUClient.holdingRegisterWrite(SLAVE_ID, FIRST_FREQUENCY_REGISTER, 1500))
  {
    Serial.print("Failed to set first frequency reference. Error: ");
    Serial.println(ModbusRTUClient.lastError());
  }
  if (!ModbusRTUClient.holdingRegisterWrite(SLAVE_ID, FREQUENCY_REGISTER, 1500))
  {
    Serial.print("Failed to set reference frequency. Error: ");
    Serial.println(ModbusRTUClient.lastError());
  }
}

// loop
void loop()
{
  check_limits();
  handle_discovery();

  EthernetClient client = server.accept();

  if (client.connected())
  {
    app.process(&client);
    client.stop();
  }
  mbed::Watchdog::get_instance().kick();
}

// opta manual
// https://docs.arduino.cc/tutorials/opta/user-manual/

// alpaca commands
// https://ascom-standards.org/api/
// https://ascom-standards.org/newdocs/dome.html
// https://ascom-standards.org/alpyca/alpaca.dome.html

// threads
// https://opta.findernet.com/en/tutorial/multithreading-opta-and-serie-7m

// context info
// https://github.com/lasselukkari/aWOT/blob/master/examples/RequestContext/RequestContext.ino

// other sensors to add
// https://lambermont.dyndns.org/astro/weatherstation/
// https://www.davisinstruments.com/collections/add-on-sensors/products/anemometer-for-vantage-pro2-vantage-pro

// Creating a Flash-Optimized Key-Value Store
// https://docs.arduino.cc/tutorials/portenta-h7/flash-optimized-key-value-store/

// arduino-cli compile --fqbn arduino:mbed_opta:opta ./
// arduino-cli upload -p /dev/cu.usbmodem1301 --fqbn arduino:mbed_opta:opta ./
