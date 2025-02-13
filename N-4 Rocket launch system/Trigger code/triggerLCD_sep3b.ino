/*
  NEPHAT GAKUYA GITHUA
  NAKUJA PROGECT
  MAIN RESPONDENT
  - Receive & log Load Cell (HX711) data
  - Receive & log Temp sensor(DS18B20) data
  - Receive & log Pressure sensor data
  - Remote ignition
  - LCD display
  - Buzzer notification
*/

#include <WiFi.h>
#include <esp_now.h>
 //lcd with i2c
#include <LiquidCrystal_I2C.h>
int lcdColumns = 16;
int lcdRows = 2;
LiquidCrystal_I2C lcd(0x3F, lcdColumns, lcdRows);  

// buzzer setup
#include "pitches.h"
#define BUZZER_PIN  18 // ESP32 pin GPIO18 connected to piezo buzzer

// Define LED and pushbutton pins-------------------------------------------------
#define STATUS_BUTTON 32

int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

// Define LED and pushbutton state booleans-------------------------------------
bool buttonDown = true;

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
 */
 
void receiveCallback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int dataLen)
{
  // Only allow a maximum of 250 characters in the message + a null terminating byte
  char buffer[ESP_NOW_MAX_DATA_LEN + 1];
  int msgLen = min(ESP_NOW_MAX_DATA_LEN, dataLen);
  strncpy(buffer, (const char *)data, msgLen);

  // Make sure we are null terminated
  buffer[msgLen] = 0;

  // Format the MAC address
  char macStr[18];
  formatMacAddress(recv_info->src_addr, macStr, 18);

  // Send Debug log message to the serial port
  Serial.print(millis());Serial.print(", ");Serial.println(buffer);// sensor data stored in buffer
  // Display the data on the LCD
  //lcd.clear();
  //lcd.setCursor(0, 0);
  //lcd.print("  Data:  ");
  //lcd.scrollDisplayLeft();
  //lcd.setCursor(0, 1);
  //lcd.print( buffer);
  //lcd.scrollDisplayLeft();
 //delay(50);
  // Split the received data into its components (thrust, temperature, pressure)
  String dataString = String(buffer);
  int firstComma = dataString.indexOf(',');
  int secondComma = dataString.indexOf(',', firstComma + 1);

  if (firstComma != -1 && secondComma != -1) {
    String thrustStr = dataString.substring(0, firstComma);
    String temperatureStr = dataString.substring(firstComma + 1, secondComma);
    String pressureStr = dataString.substring(secondComma + 1);

    // Convert the values to float
    float thrust = thrustStr.toFloat();
    float temperature = temperatureStr.toFloat();
    float pressure = pressureStr.toFloat();

    // Display the data on the LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("F:" + String(thrust));
    lcd.setCursor(0, 1);
    lcd.print("T:" + String(temperature));
    lcd.setCursor(8, 1);
    lcd.print("P:" + String(pressure));
  } else {
    // In case of malformed data
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Invalid data");
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
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
    Serial.println("Broadcast message success");
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
   //buzzer cont
Serial.begin(115200); // Set up Serial Monitor
  pinMode (STATUS_BUTTON, INPUT_PULLUP); //SET ESP32 PIN TO INPUT PULL-UP MODE
  pinMode(BUZZER_PIN, OUTPUT);

//lcd with i2c continuation
lcd.init();
lcd.backlight();
lcd.setCursor(0, 0);
 lcd.print("NAKUJA PROJECT");
delay(3000);
lcd.clear();
lcd.setCursor(0,1);
  lcd.print("WELCOME NEPH");
  delay(3000);
  lcd.clear();
  
  // Set ESP32 in STA mode to begin with
  WiFi.mode(WIFI_STA);
  Serial.println("ESP-NOW Broadcast Demo");

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

  // Pushbutton uses built-in pullup resistor------------------------------------
  pinMode(STATUS_BUTTON, INPUT_PULLUP);

}

void loop()
{
  if (digitalRead(STATUS_BUTTON))
  {
    // Detect the transition from low to high
     
    if (!buttonDown){
      buttonDown = true;
      // Send a message
      broadcast("on"); 
       Serial.print("button is pressed");
     lcd.setCursor(0, 0);
 lcd.print("button pressed!");
delay(500);
lcd.clear();
lcd.setCursor(0,1);
  lcd.print("IGNITION!");
  //delay(15000);
  lcd.clear();
 

 // buzzer continuation
    for( int thisNote = 0; thisNote <8; thisNote++){
      int noteDuration = 1000 /noteDurations[thisNote];
      tone(BUZZER_PIN, melody[thisNote], noteDuration);

      int pauseBetweenNotes =noteDuration * 1.30;
      delay(pauseBetweenNotes);
      noTone(BUZZER_PIN);
    }
  
    }
    // Delay to avoid bouncing
    //delay(500);
  }
  else
  {
    // Reset the button state
    buttonDown = false;
  }

}
