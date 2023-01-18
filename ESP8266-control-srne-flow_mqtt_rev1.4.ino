#include <ESP8266WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Npa";
const char* password = "12345678";
const char* mqtt_server = "lalidts.com";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
long lastMsg1 = 0;
char msg[200];
int value = 0;

/* Init to read value from srne Solar Charger */



/* Init flow */

byte sensorInterrupt = 0;
byte sensorPin = D5;
float calibrationFactor = 0.871618759;  // G1" = 0.24, G2" = 4.52
volatile byte pulseCount;
float flowRate;
unsigned long flowMilliLiters;
unsigned long totalMilliLiters;
int total1, total2;
unsigned long oldTime;

void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}

/* end of Init */

/* Init MQTT ! */

String pv_payload;
String bat_payload;
String load_payload;
String flow_payload;

/* end of Init */


int leds = D2;
const int led = D3;
String status;


void setup() {
  
  pinMode(leds, OUTPUT);  // Initialize the BUILTIN_LED pin as an output
  pinMode(led, OUTPUT);
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  delay(1000);
  // espSerial.begin(9600, SWSERIAL_8N1);
  // node.begin(1, espSerial);
  // delay(500);
  pinMode(sensorPin, INPUT);
  digitalWrite(sensorPin, HIGH);
}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.println((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(leds, LOW);  // Turn the LED on (Note that LOW is the voltage level
    Serial.println("\n\n\n\RECEIVE: 1\n\n\n\n");
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else if ((char)payload[0] == '0') {
    digitalWrite(leds, HIGH);  // Turn the LED off by making the voltage HIGH
    Serial.println("\n\n\n\nRECEIVE: 0\n\n\n\n");
  } else if ((char)payload[0] == '5') {
    digitalWrite(led, HIGH);  // Turn the LED off by making the voltage HIGH
    Serial.println("\n\n\n\nRECEIVE: 5\n\n\n\n");
  } else if ((char)payload[0] == '6') {
    digitalWrite(led, LOW);  // Turn the LED off by making the voltage HIGH
    Serial.println("\n\n\n\nRECEIVE: 6\n\n\n\n");
  } else if ((char)payload[0] == '3') {
    status = "on";
    Serial.println("\n\n\n\nRECEIVE: 6\n\n\n\n");
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("mobile/sub");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  readFlow();
  delay(1);


  long now1 = millis();
  if (now1 - lastMsg1 > 60000) {
    // readData();
    if (status >= "on" && flowRate < 1) {
      digitalWrite(leds, HIGH);
      client.publish("mobile/data/overload", "1");
    }
    lastMsg1 = now1;
  }

  long now = millis();
  if (now - lastMsg > 5000) {
    // readData();
    lastMsg = now;

    // pv_payload = "{\"PV-Volt\":" + String(pv_volt) + ",\"PV_Amp\":" + String(pv_amp) + ",\"PV_Power\":" + String(pv_power) + "}";  //, \"flow\":" + String(flowRate) + ",\"vol\":" + String(total1) + "
    // bat_payload = "{\"Bat_Volt\":" + String(bat_volt) + ",\"Bat_Amp\":" + String(bat_amp) + ",\"Bat_disE\":" + String(bat_disChargeEnergy) + ",\"Bat_chE\":" + String(bat_chargeEnergy) + "}";
    // load_payload = "{\"load_volt\":" + String(load_volt) + ",\"load_stat\":" + String(load_stat) + ",\"load_amp\":" + String(load_amp) + ",\"load_power\":" + String(load_power) + "}";
    flow_payload = "{\"flowrate\":" + String(flowRate) + ",\"volume\":" + String(total1) + "}";
    // client.publish("mobile/data/pv", msg);
    // delay(100);
    // bat_payload.toCharArray(msg, bat_payload.length() + 1);
    // client.publish("mobile/data/bat", msg);
    // delay(100);
    // load_payload.toCharArray(msg, load_payload.length() + 1);
    // client.publish("mobile/data/load", msg);
    // delay(100);
    flow_payload.toCharArray(msg, flow_payload.length() + 1);
    client.publish("mobile/data/flow", msg);
    // delay(100);
    Serial.println("Publish message: ");
  }
}
/*
void readData() {
  uint8_t j, result;
  pv_volt = readData(reg_pv_volt, 2);
  pv_amp = readData(reg_pv_amp, 3);
  pv_power = readData(reg_pv_power, 1);

  // BAT Status
  bat_volt = readData(reg_b_volt, 2);
  bat_amp = readData(reg_b_amp, 3);
  bat_disChargeEnergy = readData(reg_b_disChargeEnergy, 1);
  bat_chargeEnergy = readData(reg_b_chargeEnergy, 1);

  // LOAD Status
  load_volt = readData(reg_l_volt, 2);
  load_amp = readData(reg_l_amp, 3);
  load_power = readData(reg_l_power, 1);
  load_stat = readData(reg_b_loadstat, 1);

  
  Serial.print("PV_Volt: ");
  Serial.print(pv_volt);
  Serial.print("\t V_amp: ");
  Serial.print(pv_amp);
  Serial.print("\t PV_power: ");
  Serial.println(pv_power);
  Serial.println();

  Serial.print("Bat_Volt: ");
  Serial.print(bat_volt);
  Serial.print("\t bat_amp: ");
  Serial.print(bat_amp);
  Serial.print("\t bat_discharge: ");
  Serial.print(bat_disChargeEnergy);
  Serial.print("\t bat_Charge: ");
  Serial.println(bat_chargeEnergy);
  Serial.println();

  Serial.print("load_volt: ");
  Serial.print(load_volt);
  Serial.print("\t load_amp: ");
  Serial.print(load_amp);
  Serial.print("\t load_Power: ");
  Serial.print(load_power);
  Serial.print("\t load_stat: ");
  Serial.println(load_stat);
  Serial.println();
  
}
*/
/*
double readData(uint16_t m_startAddress, int version) {
  if (version == 1) {
    return readDataOneReg(m_startAddress);
  } else if (version == 2) {
    return readDataOneReg(m_startAddress) * 0.1d;
  } else if (version == 3) {
    return readDataOneReg(m_startAddress) * 0.01d;
  }
}
*/
/*
uint16_t readDataOneReg(uint16_t u16ReadAddress) {
  uint8_t result;
  uint16_t transferredData = 0;
  result = node.readHoldingRegisters(u16ReadAddress, 1);
  if (result == node.ku8MBSuccess) {
    transferredData = node.getResponseBuffer(0);
  }
  node.clearResponseBuffer();
  return transferredData;
}
*/
void readFlow() {
  if ((millis() - oldTime) > 1000)   // Only process counters once per second
  {
    detachInterrupt(sensorInterrupt);
    flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) * calibrationFactor;
    oldTime = millis();
    flowMilliLiters = (flowRate / 60) * 1000;
    totalMilliLiters += flowMilliLiters;
    total1 = totalMilliLiters / 1000;
    pulseCount = 0;
    attachInterrupt(sensorPin, pulseCounter, FALLING);

    //    Serial.print("Volume: "); Serial.print(total1); Serial.println(" L");
    //    Serial.print("flowRate: "); Serial.print(flowRate); Serial.println(" L/min\n");
  }
}
