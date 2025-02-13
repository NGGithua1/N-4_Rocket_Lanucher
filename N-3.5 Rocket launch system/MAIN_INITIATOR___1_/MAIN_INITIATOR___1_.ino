
/*
  NEPHAT GAKUYA GITHUA
  NAKUJA PROJECT
  MAIN INTIATOR
  - Transmits load cell calibrated readings
  - Ignition of motor
*/

// Include Libraries
#include <WiFi.h>
#include <esp_now.h>
#include <HX711_ADC.h> //------------------


// Define Ignition pin----------------------------------------------
#define ign_pin 33
bool IGN_STATUS = false;
unsigned long t_ = 0;

// Define Load Cell pins----------------------------------------------
const int HX711_dout = 13; //mcu > HX711 dout pin, must be external interrupt capable!
const int HX711_sck = 32; //mcu > HX711 sck pin


//HX711 constructor:-------------------------------------------------
HX711_ADC LoadCell(HX711_dout, HX711_sck);


unsigned long t = 0;
volatile boolean newDataReady;


//Ignition func-----------------------------------------------------------

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

void setup()
{
  
  // Set up Serial Monitor

  Serial.begin(115200);

  
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
\

}

void loop()
{
  const int serialPrintInterval = 0; //increase value to slow down serial print activity---------------------------------------

  // get smoothed value from the dataset:--------------------------------------------------------------
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();
      newDataReady = 0;
      //Serial.print("Load_cell output val: ");
      Serial.println(i);
      //n8Serial.println("g");
      broadcast(String(i));
      
      //Serial.print("  ");
      //Serial.println(millis() - t);
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
