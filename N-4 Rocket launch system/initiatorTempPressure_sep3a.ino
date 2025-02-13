/*
  NEPHAT GAKUYA GITHUA
  NAKUJA PROJECT
  MAIN INTIATOR
  - Transmits Load Cell (HX711) calibrated readings
  - Transmits Temp sensor(DS18B20) calibrated readings
  - Transmits Pressure sensor data
  - Ignition of Solid Rocket Motor
*/

// Include Libraries
#include <WiFi.h>
#include <esp_now.h>
#include <HX711_ADC.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//PRESSURE SENSOR
#define PRESSURE_PIN 13 // Analog pin connected to the pressure sensor

// Define Temp data wire connected to GPIO 35----------------------------------------------
#define ONE_WIRE_BUS 14
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Define Ignition pin----------------------------------------------
#define ign_pin 26
bool IGN_STATUS = false;
unsigned long t_ = 0;

// Define Load Cell pins----------------------------------------------
const int HX711_dout = 33; //mcu > HX711 dout pin, must be external interrupt capable!
const int HX711_sck = 32; //mcu > HX711 sck pin


//HX711 constructor:-------------------------------------------------
HX711_ADC LoadCell(HX711_dout, HX711_sck);
unsigned long t = 0;
volatile boolean newDataReady;


//interrupt routine:----------------------------------------------------------
void dataReadyISR() {
  if (LoadCell.update()) {
    newDataReady = 1;
  }
}


/**
 * formatMacAddress - formats the  macAddress.
 * @macAddr: is the macAddress that is copied onto the buffer.
 * @buffer: Is the pointer to location where macadress is copied.
 * @maxlength: is the maximum length that the sprintf function
 * should fill in the buffer.
 */
 
void formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength)
{
  snprintf(buffer, maxLength, "%02x:%02x:%02x:%02x:%02x:%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
}





/**
 * receiveCallback - function that is called whenever the ESP
 * manages to receive data packets i.e esp_now_register_recv_cb(receiveCallback);
 * @macAddr: macAddress needed inn the formatmacAddress function.
 const uint8_t *macAddr,
 */
void receiveCallback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int dataLen)
{
  // Only allow a maximum of 250 characters in the message + a null terminating byte
  char buffer[ESP_NOW_MAX_DATA_LEN + 1];
  int msgLen = min(ESP_NOW_MAX_DATA_LEN, dataLen);
  strncpy(buffer, (const char *)data, msgLen);
  buffer[msgLen] = '\0';

  // Format the MAC address
  char macStr[18];
  formatMacAddress(recv_info->src_addr, macStr, 18);

  // Check switch status
  if (strcmp("on", buffer) == 0)
  {
    //Start ignition-------------------------------------------------
    Serial.println("Starting engine: ");
    IGN_STATUS = true;
    t_ = millis();
  }
  
}



/**
 * sentCallback - function called when ESP sends data packets successfully.
 * @macaddrr: macaddress to be formatted by the formatmacAddress function.
 * @status: the status of ESP_NOW data communication protocol. this can be ESP_NOW_SEND_SUCCESS or 
 * ESP_NOW_SEND_FAIL
 */
void sentCallback(const uint8_t *macAddr, esp_now_send_status_t status)
{
  char macStr[18];
  formatMacAddress(macAddr, macStr, 18);
}



/**
 * broadcast - broadcasts message to devices within range
 * using ESP-NOW communication.
 * @message: pointer to variable that stores message to be sent.
 */
void broadcast(const String &message)
{
  // Broadcast a message to every device in range
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_peer_info_t peerInfo = {};
  memcpy(&peerInfo.peer_addr, broadcastAddress, 6);
  if (!esp_now_is_peer_exist(broadcastAddress))
  {
    esp_now_add_peer(&peerInfo);
  }
  // Send message
  esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *)message.c_str(), message.length());

  // Print results to serial monitor
  if (result == ESP_OK)
  {
    //Serial.println("Broadcast message success");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_INIT)
  {
    Serial.println("ESP-NOW not Init.");
  }
  else if (result == ESP_ERR_ESPNOW_ARG)
  {
    Serial.println("Invalid Argument");
  }
  else if (result == ESP_ERR_ESPNOW_INTERNAL)
  {
    Serial.println("Internal Error");
  }
  else if (result == ESP_ERR_ESPNOW_NO_MEM)
  {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_FOUND)
  {
    Serial.println("Peer not found.");
  }
  else
  {
    Serial.println("Unknown error");
  }
}
//PRESSURE SENSOR CONTINUATION
// Configuration constants
const int ADC_RESOLUTION = 4095;       // ESP32 ADC resolution (12-bit)
//const float SENSOR_VOLTAGE = 5.0; 
const float SENSOR_VOLTAGE = 3.3;  // Working voltage of the pressure sensor (5V)
const float SENSOR_MIN_VOLTAGE = 0.5; // Voltage at 0 MPa (adjust based on datasheet)
const float SENSOR_MAX_VOLTAGE = 4.5; // Voltage at maximum pressure
const float SENSOR_MAX_PRESSURE = 1.2; // Maximum pressure in MPa

// Number of readings to average
const int NUM_READINGS = 10;
// Variables to store pressure readings
float pressure_value = 0.0;

void setup()
{
  Serial.begin(115200);
  sensors.begin(); // Initialize temperature sensor

  // Configure the pressure pin as input
  pinMode(PRESSURE_PIN, INPUT);
    // Set ADC attenuation to allow readings up to 3.3V
  analogSetAttenuation(ADC_11db);
  
  //Set ign_pin to OUTPUT
  pinMode(ign_pin, OUTPUT);

  //LoadCell stuff -----------------------------------------------------------------
  Serial.println("Starting...");

  float calibrationValue; // calibration value
  calibrationValue = 5.702; // uncomment this if you want to set this value in the sketch

  LoadCell.begin();
  //LoadCell.setReverseOutput();
  unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue);
    LoadCell.tareNoDelay();// set calibration value (float)

    Serial.println("Startup is complete");
  }

  attachInterrupt(digitalPinToInterrupt(HX711_dout), dataReadyISR, FALLING);
  //----------------------------------------------------------

  // Set ESP32 in STA mode to begin with
  WiFi.mode(WIFI_STA);
  Serial.println("ESP-NOW Broadcast");

  // Print MAC address
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Disconnect from WiFi
  WiFi.disconnect();

  // Initialize ESP-NOW
  if (esp_now_init() == ESP_OK)
  {
    Serial.println("ESP-NOW Init Success");
    esp_now_register_recv_cb(receiveCallback);
    esp_now_register_send_cb(sentCallback);
  }
  else
  {
    Serial.println("ESP-NOW Init Failed");
    delay(3000);
    ESP.restart();
  }


}

void loop()
{
  //PRESSURE SENSOR
   // Read the analog signal from the pressure sensor (0-4095 range on ESP32)
  int sensor_value = analogRead(PRESSURE_PIN);
   // Convert the analog value to a voltage (assuming 3.3V reference)
  float voltage = sensor_value * (5 / 4095.0);
int totalAnalogValue = 0;
   // Collect multiple samples for averaging
   for (int i = 0; i < NUM_READINGS; i++) {
   totalAnalogValue += analogRead(PRESSURE_PIN);
   //delay(10);  // Short delay between readings
   }
  // Average the analog readings
  int analogValue = totalAnalogValue / NUM_READINGS;
  // Convert analog value to voltage
 // float voltage = (analogValue / float(ADC_RESOLUTION)) * SENSOR_VOLTAGE;

  // Map the voltage to the corresponding pressure (adjust based on the sensor's datasheet)
   pressure_value = mapPressure(voltage);

  // Display the pressure value on the serial monitor
   Serial.print("Pressure: ");
   Serial.print(pressure_value, 3);// Show pressure with 3 decimal places
   Serial.println(" MPa"); //SI unit of pressure sensor

   // Wait for a short interval before the next reading
  // delay(500);
// //TEST
//  int rawADC = analogRead(PRESSURE_PIN);
//   Serial.print("Raw ADC Value: ");
//   Serial.println(rawADC);

//   float voltage = (rawADC / float(ADC_RESOLUTION)) * SENSOR_VOLTAGE;
//   Serial.print("Voltage: ");
//   Serial.println(voltage, 3);

//   float pressure = mapPressure(voltage);
//   Serial.print("Pressure: ");
//   Serial.print(pressure, 3);
//   Serial.println(" MPa");

//   delay(500);
//*******************************************************************************************************
//**********************************************************************************************************
  //Temp Sensor
  sensors.requestTemperatures(); // Request temperature readings
  float temperature = sensors.getTempCByIndex(0); // Read temperature in Celsius

  if(temperature != DEVICE_DISCONNECTED_C) { // Check for valid temperature reading
   Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println("°C");
  }
  else{
    Serial.println("Error: Could not read temperature data");
  }
  
  const int serialPrintInterval = 0; //increase value to slow down serial print activity---------------------------------------

  // get smoothed value from the dataset:--------------------------------------------------------------
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();
      newDataReady = 0;
      
      Serial.println(i);
     
      //CHANGES TO BROADCASTING 
      //BROADCAST HX711 DATA ONLY 
      //broadcast(String(i));

      //BROADCAST HX711 DATA +TEMP SENSOR + PRESSURE SENSOR
      broadcast(String(i) + "," + String(temperature)+ "," + String(pressure_value) );  // Broadcast load cell data and temperature

      t = millis();
    }
  }

  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
  }

  //check if last tare operation is complete
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }
  //------------------------------------------------------------------
  
  if(IGN_STATUS && t_ + 15000 > millis()){
    digitalWrite(ign_pin, HIGH);
    
  }else{
    IGN_STATUS = false;
    digitalWrite(ign_pin, LOW);
  }

 
}
// Function to convert voltage to pressure
float mapPressure(float voltage) {
  // Ensure voltage is within the expected range
  if (voltage < SENSOR_MIN_VOLTAGE) voltage = SENSOR_MIN_VOLTAGE;
  if (voltage > SENSOR_MAX_VOLTAGE) voltage = SENSOR_MAX_VOLTAGE;

  // Calculate pressure based on sensor's voltage range
  //float pressure = (voltage - SENSOR_MIN_VOLTAGE) * SENSOR_MAX_PRESSURE / 
                   (SENSOR_MAX_VOLTAGE - SENSOR_MIN_VOLTAGE);
  // // Adjust the pressure calculation based on your sensor's characteristics
  // // Example: If 0.5V corresponds to 0 psi and 4.5V corresponds to 100 psi, use this formula
  // float pressure = (voltage - 0.5) * 100.0 / (4.0);
 // return pressure;
 return (voltage - SENSOR_MIN_VOLTAGE) * SENSOR_MAX_PRESSURE / (SENSOR_MAX_VOLTAGE - SENSOR_MIN_VOLTAGE);
}