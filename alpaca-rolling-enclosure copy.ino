#include <ArduinoRS485.h>
#include <Ethernet.h>
#include <aWOT.h>
#include <cmath>
#include <unordered_map>
#include <string>
#include "opta_info.h"

OptaBoardInfo *info;
OptaBoardInfo *boardInfo();

using namespace mbed;

// struct for possible shutter statuses
struct ShutterStatuses
{
  static const int shutterOpen = 0;
  static const int shutterClosed = 1;
  static const int shutterOpening = 2;
  static const int shutterClosing = 3;
  static const int shutterError = 4;
};

int shutterStatus;

// proximity sensors
volatile bool doorSensor;
volatile bool fullyOpenSensor;
volatile bool fullyClosedSensor;

// safety relay
volatile bool safetyRelayAcivated;

// VFD status
volatile bool vfdStatus;

// rain sensor
volatile bool rainSensor;

// Define the pins for the sensors
constexpr int rainSensorPin = A2;
constexpr int doorSensorPin = A3;
constexpr int fullyOpenSensorPin = A4;
constexpr int fullyClosedSensorPin = A5;
constexpr int safetyRelayPin = A6;
constexpr int vfdStatusPin = A7;

// ISR for fully open sensor
void fullyOpenSensorISR()
{
  fullyOpenSensor = digitalRead(fullyOpenSensorPin);
}

// ISR for fully closed sensor
void fullyClosedSensorISR()
{
  fullyClosedSensor = digitalRead(fullyClosedSensorPin);
}

// RS485
constexpr auto baudrate{9600};
// Calculate preDelay and postDelay in microseconds for stable RS-485 transmission
constexpr auto bitduration{1.f / baudrate};
constexpr auto wordlen{10.0f}; // OR 10.0f depending on the channel configuration
constexpr auto preDelayBR{bitduration * wordlen * 3.5f * 1e6};
constexpr auto postDelayBR{bitduration * wordlen * 3.5f * 1e6};

// isSafe
bool isSafe = false;

// Alpaca error handling
int32_t ErrorNumber = 0;
String ErrorMessage = "\"\"";

// the media access control (ethernet hardware) address  TODO: can't this be set automatically?
byte mac[6];
// the IP address
byte ip[] = {192, 168, 1, 14}; // TODO: can this set more easily by the user - DHCP first then fixed?

// Create an Ethernet server
EthernetServer server(80);
Application app;

String split(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++)
  {
    if (data.charAt(i) == separator || i == maxIndex)
    {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

// Handle the endpoint requests
void endPoint(Request &req, Response &res)
{

  res.set("Content-Type", "application/json");
  auto url = split(req.path(), '/', 5);

  uint32_t ClientID = 0;
  uint32_t ClientTransactionID = 0;

  uint32_t ServerID = 0;            // TODO: what is this?
  uint32_t ServerTransactionID = 0; // TODO: what is this? increment?

  float Value;
  String ValueString;
  bool ValueBool;

  char buffer[16]; // Adjust the buffer size as needed

  if (req.query("ClientID", buffer, sizeof(buffer)))
  {
    ClientID = strtoul(buffer, NULL, 10);
  }

  if (req.query("ClientTransactionID", buffer, sizeof(buffer)))
  {
    ClientTransactionID = strtoul(buffer, NULL, 10);
  }

  if (url == "connected")
  {
    ValueBool = true;
  }
  else if (url == "name")
  {
    ValueString = "Dome and dome safety monitor by PPP"; // TODO: change this, add limits?
  }
  else if (url == "driverversion")
  {
    ValueString = "0.0.1";
  }
  else if (url == "issafe")
  {
    ValueBool = isSafe;
  }
  else if (url == "atpark")
  {
    if (shutterStatus == ShutterStatuses::shutterClosed)
    {
      ValueBool = true;
    }
    else
    {
      ValueBool = false;
    }
  }
  else if (url == "canfindhome")
  {
    ValueBool = false;
  }
  else if (url == "canpark")
  {
    ValueBool = true;
  }
  else if (url == "cansetaltitude")
  {
    ValueBool = false;
  }
  else if (url == "cansetazimuth")
  {
    ValueBool = false;
  }
  else if (url == "cansetpark")
  {
    ValueBool = false;
  }
  else if (url == "cansetshutter")
  {
    ValueBool = true;
  }
  else if (url == "canslave")
  {
    ValueBool = false;
  }
  else if (url == "cansyncazimuth")
  {
    ValueBool = false;
  }
  else if (url == "shutterstatus")
  {
    Value = shutterStatus;
  }
  else if (url == "slewing")
  {
    if (shutterStatus == ShutterStatuses::shutterOpening || shutterStatus == ShutterStatuses::shutterClosing)
    {
      ValueBool = true;
    }
    else
    {
      ValueBool = false;
    }
  }
  else if (url == "abortslew")
  {
    // call abort slew function
    ValueBool = true;
  }
  else if (url == "closeshutter")
  {
    // call close shutter function
    ValueBool = true;
  }
  else if (url == "openshutter")
  {
    // call open shutter function
    ValueBool = true;
  }
  else if (url == "park")
  {
    // call close shutter function
    ValueBool = true;
  }
  else
  {
    ErrorNumber = 1;
    ErrorMessage = "\"Invalid path\"";
  }

  res.print("{");
  res.print("\"ClientID\": ");
  res.print(ClientID);
  res.print(", ");
  res.print("\"ClientTransactionID\": ");
  res.print(ClientTransactionID);
  res.print(", ");
  res.print("\"ServerID\": ");
  res.print(ServerID);
  res.print(", ");
  res.print("\"ErrorNumber\": ");
  res.print(ErrorNumber);
  res.print(", ");
  res.print("\"ErrorMessage\": ");
  res.print(ErrorMessage);
  res.print(", ");
  res.print("\"Value\": ");
  if (url == "name" || url == "driverversion")
  {
    res.print("\"");
    res.print(ValueString);
    res.print("\"");
  }
  else if (url == "shutterstatus")
  {
    res.print(Value);
  }
  else
  {
    res.print(ValueBool ? "true" : "false");
  }
  res.print("}");

  // reset error
  ErrorNumber = 0;
  ErrorMessage = "\"\"";
}

// setup
void setup()
{

  Serial.begin(115200);

  //  Wait for terminal to come up
  delay(1000);

#if defined(GET_OPTA_OTP_BOARD_INFO)
  info = boardInfo();
  if (info->magic == 0xB5)
  {
    for (int i = 0; i < 6; i++)
    {
      mac[i] = info->mac_address[i];
    }
  }
#else
  Serial.println("ERROR: MAC address not successfully set");
  while (1)
    ;
#endif

  if (Ethernet.begin(mac, ip))
  {
    Serial.println(Ethernet.localIP());
  }
  else
  {
    Serial.println("Ethernet failed");
  }

  // set pins for LEDs
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_D0, OUTPUT);

  // set pins for sensors
  pinMode(rainSensorPin, INPUT);        // rain sensor
  pinMode(doorSensorPin, INPUT);        // door sensor
  pinMode(fullyOpenSensorPin, INPUT);   // fully open sensor
  pinMode(fullyClosedSensorPin, INPUT); // fully closed sensor
  pinMode(safetyRelayPin, INPUT);       // safety relay
  pinMode(vfdStatusPin, INPUT);         // VFD status

  // Attach interrupts to the sensors
  attachInterrupt(digitalPinToInterrupt(rainSensorPin), rainSensorISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(doorSensorPin), doorSensorISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(fullyOpenSensorPin), fullyOpenSensorISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(fullyClosedSensorPin), fullyClosedSensorISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(safetyRelayPin), safetyRelayISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(vfdStatusPin), vfdStatusISR, CHANGE);

  // endpoints
  app.get("/api/v1/safetymonitor/0/connected", &endPoint);
  app.put("/api/v1/safetymonitor/0/connected", &endPoint);
  app.get("/api/v1/safetymonitor/0/issafe", &endPoint);
  app.get("/api/v1/safetymonitor/0/name", &endPoint);
  app.get("/api/v1/safetymonitor/0/driverversion", &endPoint);

  app.get("/api/v1/dome/0/connected", &endPoint);
  app.put("/api/v1/dome/0/connected", &endPoint);
  app.get("/api/v1/dome/0/name", &endPoint);
  app.get("/api/v1/dome/0/driverversion", &endPoint);
  // app.get("/api/v1/dome/0/altitude", &endPoint);	//the dome altitude
  // app.get("/api/v1/dome/0/athome", &endPoint);	//indicates whether the dome is in the home position.
  app.get("/api/v1/dome/0/atpark", &endPoint); // indicates whether the dome is at the park position
  // app.get("/api/v1/dome/0/azimuth", &endPoint);	//the dome azimuth

  app.get("/api/v1/dome/0/canfindhome", &endPoint);    // indicates whether the dome can find the home position.
  app.get("/api/v1/dome/0/canpark", &endPoint);        // indicates whether the dome can be parked.
  app.get("/api/v1/dome/0/cansetaltitude", &endPoint); // indicates whether the dome altitude can be set
  app.get("/api/v1/dome/0/cansetazimuth", &endPoint);  // indicates whether the dome azimuth can be set
  app.get("/api/v1/dome/0/cansetpark", &endPoint);     // indicates whether the dome park position can be set
  app.get("/api/v1/dome/0/cansetshutter", &endPoint);  // indicates whether the dome shutter can be opened
  app.get("/api/v1/dome/0/canslave", &endPoint);       // indicates whether the dome supports slaving to a telescope
  app.get("/api/v1/dome/0/cansyncazimuth", &endPoint); // indicates whether the dome azimuth position can be synched

  app.get("/api/v1/dome/0/shutterstatus", &endPoint); // status of the dome shutter or roll-off roof
  // app.get("/api/v1/dome/0/slaved", &endPoint);	//indicates whether the dome is slaved to the telescope
  // app.put("/api/v1/dome/0/slaved", &endPoint);	//sets whether the dome is slaved to the telescope
  app.get("/api/v1/dome/0/slewing", &endPoint);      // indicates whether the any part of the dome is moving
  app.put("/api/v1/dome/0/abortslew", &endPoint);    // immediately cancel current dome operation.
  app.put("/api/v1/dome/0/closeshutter", &endPoint); // close the shutter or otherwise shield telescope from the sky.
  // app.put("/api/v1/dome/0/findhome", &endPoint);	//start operation to search for the dome home position.
  app.put("/api/v1/dome/0/openshutter", &endPoint); // open shutter or otherwise expose telescope to the sky.
  app.put("/api/v1/dome/0/park", &endPoint);        // rotate dome in azimuth to park position.
  // app.put("/api/v1/dome/0/setpark", &endPoint);	//set the current azimuth, altitude position of dome to be the park position
  // app.put("/api/v1/dome/0/slewtoaltitude", &endPoint);	//slew the dome to the given altitude position.
  // app.put("/api/v1/dome/0/slewtoazimuth", &endPoint);	//slew the dome to the given azimuth position.
  // app.put("/api/v1/dome/0/synctoazimuth", &endPoint);	//synchronize the current position of the dome to the given azimuth.

  server.begin();

  // RS485 init
  RS485.begin(baudrate);
  RS485.setDelays(preDelayBR, postDelayBR);
}

// loop
void loop()
{

  EthernetClient client = server.available();

  if (client.connected())
  {
    app.process(&client);
    client.stop();
  }
}

// opta manual
// https://docs.arduino.cc/tutorials/opta/user-manual/

// alpaca commands
// https://ascom-standards.org/api/
// https://ascom-standards.org/alpyca/alpaca.dome.html
// https://ascom-standards.org/alpyca/alpaca.safetymonitor.html

// threads
// https://opta.findernet.com/en/tutorial/multithreading-opta-and-serie-7m

// context info
// https://github.com/lasselukkari/aWOT/blob/master/examples/RequestContext/RequestContext.ino

// other sensors to add
// https://lambermont.dyndns.org/astro/weatherstation/
// https://www.davisinstruments.com/collections/add-on-sensors/products/anemometer-for-vantage-pro2-vantage-pro
// Kemo M152K rain sensor

// Creating a Flash-Optimized Key-Value Store
// https://docs.arduino.cc/tutorials/portenta-h7/flash-optimized-key-value-store/

// arduino-cli compile --fqbn arduino:mbed_opta:opta ./
// arduino-cli upload -p /dev/cu.usbmodem1301 --fqbn arduino:mbed_opta:opta ./