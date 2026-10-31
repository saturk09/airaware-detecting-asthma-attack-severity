//NOTE: A lot of the current comments are ones found in sample code pieces, so they are reiterated throughout this code section.




#include <SPI.h>              // Wireless comms between sensor(s) and Arduino Nano IoT
#include <WiFiNINA.h>         // Used to connect Nano IoT to network
#include <ArduinoJson.h>      // Used for HTTP Request
#include "arduino_secrets.h"  // Used to store private network info


int spo2Offset = 35;        //added to adjust "sensor drift", or basically when the sensor is receiving too much noise and needs to balance off
int lastGoodSpO2 = 98;      //stores the most recent valid SPO2 reading that isn't under 0
float smoothedSpO2 = 97.0;  //uses "smoothing" to hold a reduced-noise version of SPO2 averaging
int lastGoodHR = 78;        //stores most recent valid heart rate, resting
float smoothedHR = 78.0;    //smoothed heart rate


// Define global variables and constants
const int trigPin = 2;  // D2 Arduino to pin Trig
const int echoPin = 3;  // D3 Arduino to pin Echo
long distance;          // long variable for distance
long duration;          // long variable for duration


//The following lines of code (19-24) were used from sample code and research to actively reduce noise in the red light and infrared light ratio when measured from the wrist
#define RATIO_BUFFER_SIZE 12                   //sets ratio buffer size to 12 for the red light/IR ratio of SPO2
float ratioBuffer[RATIO_BUFFER_SIZE] = { 0 };  //declares an array that holds red/IR ratio values
int ratioIndex = 0;                            //declares the index of the RATIO_BUFFER_SIZE array
float lastPerfusion = 0.0;                     //stores the "perfusion" or signal quality indicator index
const float MIN_PERFUSION = 0.45;              //stores a possible minimum acceptable perfusion


///////sensitive data entered in arduino_secrets.h for addresses/hotspots
char ssid[] = SECRET_SSID;  // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;           // your network key index number (needed only for WEP)
int status = WL_IDLE_STATUS;


// Initialize the Wifi client library
WiFiClient client;


// server address:
//char server[] = "jsonplaceholder.typicode.com"; // for public domain server
IPAddress server(172, 20, 10, 14);  // for localhost server (server IP address can be found with ipconfig or ifconfig)


unsigned long lastConnectionTime = 0;
const unsigned long postingInterval = 2500UL;  // delay between updates, in milliseconds (with our data processing, data will send at 7-8 second intervals)


//Libraries previously included through sample codes (and the BLE library for the XIAO)
#include <Wire.h>            //includes I2C communication library
#include "MAX30105.h"        //includes the SparkFun driver for the pulse ox
#include "spo2_algorithm.h"  //includes the SparkFun SPO2 calculation algorithm
#include <ArduinoBLE.h>      //includes BLE library for XIAO communication


MAX30105 particleSensor;  //defines the sensor


#define MAX_BRIGHTNESS 255  //sets the LED brightness of the red light, can vary depending on wrist vs. finger placement of pulse ox


//The lines 54-63 are all from sample code
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100];   //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100];   //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif


int32_t bufferLength;   //data length
int32_t spo2;           //SPO2 value
int8_t validSPO2;       //indicator to show if the SPO2 calculation is valid
int32_t heartRate;      //heart rate value
int8_t validHeartRate;  //indicator to show if the heart rate calculation is valid
byte pulseLED = 11;     //Must be on PWM pin
byte readLED = 13;      //Blinks with each data read


BLEDevice peripheral;       //sets a peripheral BLEdevice, names it emgChar to relate to the EMG
BLECharacteristic emgChar;  //declares BLE characteristic object to read EMG data from the XIAO
int emg = 0;                //sets a variable to put the EMG values in


//Setup code
void setup() {
  Serial.begin(115200);               //starts serial communication at 115200 baud
  Serial.println("Initializing...");  //prints "initializing" to show that the serial monitor began accordingly
  if (!BLE.begin()) {
    Serial.println("BLE failed");     //states that the BLE failed if it failed
    while (1)
      ;
  }                                                         //checks if BLE works/doesn't work (connected/doesn't connect)
  Serial.println("BLE ok");                                 //confirms that BLE worked
  BLE.scanForUuid("3cb18870-19c5-4292-b13d-442e2af8447a");  //scans for address scanned by device using a previous code file


  unsigned long scanStart = millis();     //(already found in sample code)
  while (millis() - scanStart < 10000) {  // 10 second timeout
    peripheral = BLE.available();         //checks BLE availability
    if (peripheral) {
      Serial.println("BLE device found!");  //says that the BLE device was found and the connection was successful
      break;                                //terminates loop execution without terminating code
    }
  }


  if (!peripheral) {
    Serial.println("BLE device not found, continuing anyway...");  //if peripheral device isn't found, continues to look for the device
  }
  if (WiFi.status() == WL_NO_MODULE) {                         //check wifi status (researched suggested code after failures)
    Serial.println("Communication with WiFi module failed!");  // don't continue
    while (true)
      ;
  }


  // check if firmware is outdated - not inherently necessary, though
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }


  // attempt to connect to Wifi network (part 2)
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);  // Connect to network
    status = WiFi.begin(ssid, pass);
    delay(1000);  // wait 1 second for connection
  }


  printWifiStatus();                                                            // print wifi status if functional
  Serial.println("XIAO found!");                                                //states that the XIAO is finally found
  BLE.stopScan();                                                               //stop scanning the BLE
  peripheral.connect();                                                         //connect to the peripheral device that was found
  peripheral.discoverAttributes();                                              //consider all of the attributes of the XIAO
  emgChar = peripheral.characteristic("e709a157-3ee8-46e7-8a91-bb0489c17ae8");  //define the EMG characteristic


  pinMode(pulseLED, OUTPUT);  //initializes LED pin on NANO
  pinMode(readLED, OUTPUT);   //initializes LED pin on NANO


  // Initialize sensor and halts if it isn't found, notifies the user (from sample code)
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("MAX30105 not found"));  //says that the sensor attached to the NANO wasn't found
    while (1)
      ;
  }


  Serial.println("Attach sensor to your wrist now! Press any key to start the sensor readings.");  //from the sample code; lets the sensor read begin when a key is pressed.
  while (Serial.available() == 0)
    ;
  Serial.read();






  //Setup to sense a nice looking saw tooth on the plotter
  //lines 147-152 are from sample code
  byte ledBrightness = 0x3F;  //Options: 0=Off to 255=50mA, we used 3F for inner wrist readings (medium-bright light)
  byte sampleAverage = 4;     //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;           //Options: 1 = Red onlhy, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 100;       //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;       //Options: 69, 118, 215, 411
  int adcRange = 4096;        //Options: 2048, 4096, 8192, 16384


  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);  //Configure sensor with these settings


  bufferLength = 50;  //buffer length of 50 stores 2 seconds of samples running at 25sps


  //Take an average of IR readings at power up
  Serial.println("Starting IR average...");
  const byte avgAmount = 64;  //takes 64 IR readings and averages them to get an IR baseline
  long baseValue = 0;
  for (byte x = 0; x < avgAmount; x++) {
    baseValue += particleSensor.getIR();  //Read the IR value
  }
  Serial.println("IR average done...");
  baseValue /= avgAmount;  //averages the IR baseline
}


float calculateSpO2(uint32_t* redBuf, uint32_t* irBuf, int32_t len) {
  double sumRed = 0, sumIR = 0;
  for (int i = 0; i < len; i++) {  //Sums all of the red and IR buffer values to get DC/steady state components
    sumRed += redBuf[i];
    sumIR += irBuf[i];
  }
  double dcRed = sumRed / len; //calculates mean of red signal
  double dcIR = sumIR / len;   //calculates mean of IR signal


  if (dcIR < 5000) return 0;   //checks if the IR signal is too weak, will reject the reading


  double varRed = 0, varIR = 0; //computes the variance of both signals, compiled from both sample code and research on IR and red light in SPO2 noise cancellation
  for (int i = 0; i < len; i++) {
    double dr = redBuf[i] - dcRed;
    double di = irBuf[i] - dcIR;
    varRed += dr * dr;
    varIR += di * di;
    //the top 4 lines sum squared deviations from the mean of the dc values
  }
  double acRed = sqrt(varRed / len);  //takes the root mean square to get the ac amplitude, again from sample code/research. Does this for red and IR light
  double acIR = sqrt(varIR / len);


  lastPerfusion = (acIR / dcIR) * 100.0;
  if (lastPerfusion < MIN_PERFUSION) return 0; //calculates the perfusion index, or the AC/DC ratio, and rejects it if the signal quality is too low.


  float ratio = (acRed / dcRed) / (acIR / dcIR);
  if (ratio < 0.3 || ratio > 2.5) return 0; //computes the ratio, using the Beer-Lambert SPO2 formula, and rejects "impossible" values for more accurate readings


  ratioBuffer[ratioIndex] = ratio;
  ratioIndex = (ratioIndex + 1) % RATIO_BUFFER_SIZE; //stores the new ratio in the buffer previously set to size 12


  float smoothed = 0;
  int count = 0;
  for (int i = 0; i < RATIO_BUFFER_SIZE; i++) { //averages all non-zero buffer entries to produce a ratio with the smoothed out global variables, too
    if (ratioBuffer[i] > 0.1) {
      smoothed += ratioBuffer[i];
      count++;
    }
  }
  smoothed = (count > 0) ? smoothed / count : ratio; //this line ignores uninitialized slots and uses all of the global variables to combine into a semi-final spo2 reading.


  float spo2Val = 172.0 - 52.0 * smoothed; //applies a linear formula (like beer-lambert's law) to convert the red light/IR ratio to an SPO2 percentage
  spo2Val = spo2Val + spo2Offset;          //adds calibration offset to correct the raw value
  return constrain(spo2Val, 75, 100);      //adds constraints to the result so that it has to fit a value fit for human survival
}


//loop for NANO
void loop() {
  StaticJsonDocument<200> doc;             //makes a 200 byte JSON document, used in sample code but not really necessary
  //(This section contains extra code from the sample code on the pulse oximeter)
  // if there's incoming data from the net connection, append each character to a variable
  String response = "";                    //reads HTTP response data from the server, as a string
  while (client.available()) {
    char c = client.read();
    response += (c);
  }


  // print out non-empty responses to serial monitor
  if (response != "") {
    Serial.println(response);
  }


  // repeat request after around 1 second
  static unsigned long lastPostTime = 0;


  if (millis() - lastPostTime > postingInterval) {  // Send every 2.5 seconds
    httpRequest();
    lastPostTime = millis();
  }


  //read the first 50 samples, and determine the signal range
  // First loop — replace the existing one
  for (byte i = 0; i < bufferLength; i++) {
    while (particleSensor.available() == false)
      particleSensor.check();


    // Read EMG on every sample, trying to take away duplication
      static int lastPrintedEMG = 0;
      if (peripheral.connected() && emgChar.canRead()) {
        emgChar.read();
        if (emgChar.valueLength() >= 2) {
          const uint8_t* val = emgChar.value();
          int newEMG = (int16_t)((val[1] << 8) | val[0]);
         
          //takes in only the new emg readings
          if (abs(newEMG - emg) > 0 || millis() - lastPrintedEMG > 800) { //says that if the new emg reading minus the existing one is greater than 0 (meaning the value changed), or if about 0.8 seconds have passed between the previous reading and this reading so the loop doesn't get stuck
            emg = newEMG;
            lastPrintedEMG = millis();
          }
        }
      }


    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();


    //printing all of the FINAL sensor values
    Serial.print(irBuffer[i]);
    Serial.print("\t");
    Serial.print(heartRate);
    Serial.print("\t");
    Serial.print(spo2);
    Serial.print("\t");
    Serial.println(emg);
  }
  //calculate heart rate and SpO2 after first 50 samples (first 2 seconds of samples)
  //this is done to "calibrate" the MAX30102 to adjust to the surroundings, but this begins to read the samples (the same code, copied)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);


  float betterSpO2 = calculateSpO2(redBuffer, irBuffer, bufferLength);


  if (betterSpO2 >= 65) { //if the new SpO2 reading is plausible (about 65% or higher) the stored value gets updated.
    smoothedSpO2 = (smoothedSpO2 * 0.68) + (betterSpO2 * 0.32);  // More movement
    spo2 = (int32_t)smoothedSpO2;
    lastGoodSpO2 = spo2;
    validSPO2 = 1;
  } else {
    spo2 = lastGoodSpO2; //if the reading is not plausible, the code falls back to the last good SPO2 reading
  }


  if (heartRate > 40 && heartRate < 180 && validHeartRate == 1) {
    smoothedHR = (smoothedHR * 0.85) + (heartRate * 0.15);   // if the new HR reading is plausible (about 75% or hgiher) the stored value gets updated.
    heartRate = (int32_t)smoothedHR;
    lastGoodHR = heartRate;
  } else {
    heartRate = lastGoodHR;  //if the reading is not plausible, the code falls back to the last good HR reading
  }


  // Debug output
  Serial.print("\tRaw:");
  Serial.print(betterSpO2, 1);
  Serial.print("\tSmoothed:");
  Serial.print(smoothedSpO2, 1);
  Serial.print("\tPerf:");
  Serial.println(lastPerfusion, 1);


  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 12; i < 50; i++) {
      redBuffer[i - 12] = redBuffer[i];
      irBuffer[i - 12] = irBuffer[i];
    }


    //take 25  sets of samples before calculating the heart rate.
    for (byte i = 38; i < 50; i++) {
      while (particleSensor.available() == false)
        particleSensor.check();


      // Read EMG on every sample, trying to take away duplication
      static int lastPrintedEMG = 0;
      if (peripheral.connected() && emgChar.canRead()) {
        emgChar.read();
        if (emgChar.valueLength() >= 2) {
          const uint8_t* val = emgChar.value();
          int newEMG = (int16_t)((val[1] << 8) | val[0]);
         
          //takes in only the new emg readings
          if (abs(newEMG - emg) > 0 || millis() - lastPrintedEMG > 800) { //says that if the new emg reading minus the existing one is greater than 0 (meaning the value changed), or if about 0.8 seconds have passed between the previous reading and this reading so the loop doesn't get stuck
            emg = newEMG;
            lastPrintedEMG = millis();
          }
        }
      }


      digitalWrite(readLED, !digitalRead(readLED));


      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample();


    //printing all of the FINAL sensor values
      Serial.print(irBuffer[i]);
      Serial.print("\t");
      Serial.print(heartRate);
      Serial.print("\t");
      Serial.print(spo2);
      Serial.print("\t");
      Serial.println(emg);
    }
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);


    float betterSpO2 = calculateSpO2(redBuffer, irBuffer, bufferLength);


    if (betterSpO2 >= 65) {
      smoothedSpO2 = (smoothedSpO2 * 0.68) + (betterSpO2 * 0.32);  // More movement
      spo2 = (int32_t)smoothedSpO2;
      lastGoodSpO2 = spo2;
      validSPO2 = 1;
    } else {
      spo2 = lastGoodSpO2;
    }


    if (heartRate > 40 && heartRate < 180 && validHeartRate == 1) {
    smoothedHR = (smoothedHR * 0.85) + (heartRate * 0.15);   // if the new HR reading is plausible (about 75% or hgiher) the stored value gets updated.
    heartRate = (int32_t)smoothedHR;
    lastGoodHR = heartRate;
  } else {
    heartRate = lastGoodHR;  //if the reading is not plausible, the code falls back to the last good HR reading
  }


    // Debug output
    Serial.print("\tRaw:");
    Serial.print(betterSpO2, 1);
    Serial.print("\tSmoothed:");
    Serial.print(smoothedSpO2, 1);
    Serial.print("\tPerf:");
    Serial.println(lastPerfusion, 1);
  }
}
// this method makes a HTTP connection to the server:
void httpRequest() {


  // close any connection before send a new request to free the socket
  client.stop();


  // call range() function to get distance
  range();


  // if there's a successful connection:
  if (client.connect(server, 5000)) {
    Serial.println("connecting...");


    // send the HTTP GET request with the distance as a parameter.
    // The Flask route to call should be inbetween the "/" and "?" (ex:  GET /test?...
    // where "test" is the Flask route that will GET the data, "distance" is the key
    // and the value is provided by:  String(distance))
    String request = "GET /data?emg=" + String(emg) + "&ox_saturation=" + String(spo2) + "&heartRate=" + String(heartRate) + " HTTP/1.1";
    client.println(request);


    // set the host as server IP address
    client.println("Host: 172.20.10.5");


    // other request properties
    client.println("User-Agent: ArduinoWiFi/1.1");
    client.println("Connection: close");
    client.println();


    // note the time that the connection was made:
    lastConnectionTime = millis();
  } else {
    Serial.println("connection failed");  // couldn't make a connection
  }
}


// connect to wifi network and display status (essential to detect network issues)
void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();  // board's IP on the network
  Serial.print("IP Address: ");
  Serial.println(ip);
  long rssi = WiFi.RSSI();  // received signal strength
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}


// collect distance values (Comments are from the sample code provided in the MAX30102 Library)
void range() {
  digitalWrite(trigPin, LOW);         // set trigPin LOW to clear it
  delayMicroseconds(2);               // 2 microsecond delay
  digitalWrite(trigPin, HIGH);        // set trigPin HIGH
  delayMicroseconds(10);              // 10 microsecond delay
  digitalWrite(trigPin, LOW);         // set trigPin LOW
  duration = pulseIn(echoPin, HIGH);  // set trigPin HIGH
  distance = duration * 0.034 / 2;    // distance (cm) calculation
}

