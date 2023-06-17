#include <SimpleDHT.h>
#include <Wire.h>
#include <RtcDS3231.h>
#include <SPI.h>
#include <Ethernet.h>



unsigned long Time_zero=millis();   // STARTS TO COUNT TIME AT POINT ZERO   WE DETERMINE 00:00:00
unsigned long Time_between_each_measure=60000; //  EVERY 60000 MILLI SECONDS IN MEASURES = 1 minuite

//DEFINITION IF PINS
#define voltage_sensor_pin A2
#define Current_sensor_pin A0
#define pinDHT22 4                // TEMPERATURE AND HUMIDITY SENSOR DIGITAL PIN No 4

//DEFINITION CONSTANTS AND MORE

#define countof(a) (sizeof(a) / sizeof(a[0]))

//DEFINITION OF VARIABLEE

// Variables for Measured Voltage and Calculated Current
const byte   significant_digits = 3;          // digits in printed measurements we want to be displayed

float Volt_current_value;   // the current measured by lts 25np current sensor displayed in volts
float lts25np_vout;         // Vout in volt of the lts 25np current sensor
float Lts_25_np_Current;  // the current measured  from lts 25np current sensor in Amps
float I2;                   // the secont value os the curent we want to keep ( expml I1=5 Amps I2=5.1 Amps )
const float  zero_current = 2.50244;        // 2.50025 volts is the zero current ( I = 0.000 Amps ) sensor point with corect flow in sensor terminals ( above 2.50025 volts it can measure 0-25 Amps ) 
const float  scale_factor = 0.02551111;         // scale factor for lts 25np  in V/Amp
                                          
 /* 
  * Constants for A/D converter resolutionArduino has 10-bit ADC,
  * so 1024 possible valuesReference voltage is 5V
  * if not using AREF external referenceZero point is half of Reference Voltage 
  */

float V_reference=4.096;                      // We use external voltage reference 4.096 Volts from LM4040
float total_ADC_resolution=1023.00;           // analog discrete values for Arduino MEGA/UNO 10-bit ADC                                                           
float Arduino_Voltage_sensor_pin_Value=0.00;  // the arduino pin that is conected to the voltage sensor( a value from 0-->1023) (ADC  STEP NUMBER )
float V_sensor_OUT=0.00;                      // actual voltage in volts from sensor to analog pin A2 (in this specific sensor x5 times smaller than the Volts we want to measure ( max Voltage we can put to sensor is 25 Volts))
float V_measured=0.00;                        // actual voltage that we want to measure example : the voltage of a 9 Volt batery
float R1=7500.00;                             // RESISTOR 1 OF VOLTAGE DIVIDER ON VOLTAGE SENSOR
float R2=30000.00;                            // RESISTOR 2 OF VOLTAGE DIVIDER ON VOLTAGE SENSOR
float Step_resolution = V_reference/total_ADC_resolution;      // How many volts does each step of ADC has ( 0.004003910068 Volts ) 
float temperature = 0;                        //Temperature of enviroment measured
float humidity = 0;                           //Humidity of enviroment measured


/* Constants for A/D converter resolution
   Arduino has 10-bit ADC, so 1024 possible values
   Reference voltage is 5V if not using AREF external reference
   Zero point is half of Reference Voltage 
*/

//OBJECT CREATION

SimpleDHT22 dht22(pinDHT22);   // CREATES THE OBJECT SimpleFHT11 FOR THE HUMIDITY AND TEMPERATURE SENSOR*/
RtcDS3231<TwoWire> Rtc(Wire);     // CREATES THE OBJECT FOR THE RTC

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xA8, 0x61, 0x0A, 0xAE, 0x89, 0xBB
};
IPAddress ip(192, 168, 1, 177); // ip adress of the web server for displaying  measured values

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(80);


//DECLARATION OF FUCTIONS

//MEASURES TEMPRATURE AND HUMIDITY DHT 22
void Temprature_and_Humidity(){
  /* start working...
  Serial.println("=================================");
  Serial.println("Sample DHT22...");
  @remark We use read2 to get a float data, such as 10.1*C
  if user doesn't care about the accurate data, use read to get a byte data, such as 10*C.
  */
  /*float temperature = 0;
  float humidity = 0; */
  int err = SimpleDHTErrSuccess;
  if ((err = dht22.read2(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("Read DHT22 failed, err="); Serial.print(SimpleDHTErrCode(err));
    Serial.print(","); Serial.println(SimpleDHTErrDuration(err)); delay(2000);
    return;
  }
  
  //Serial.print("Sample OK: ");
  Serial.print((float)temperature); //Serial.print(" *C, ");
  Serial.print(",");
  Serial.print((float)humidity); //Serial.println(" RH%");
  
  // DHT22 sampling rate is 0.5HZ.
}

//PRINTS DATE TIME YEAR OF RTC DS3231
void Print_date_time_year(){
    if (!Rtc.IsDateTimeValid()) 
    {
        if (Rtc.LastError() != 0)
        {
            // we have a communications error
            // see https://www.arduino.cc/en/Reference/WireEndTransmission for 
            // what the number means
            Serial.print("RTC communications error = ");
            Serial.println(Rtc.LastError());
        }
        else
        {
            // Common Causes:
            //    1) the battery on the device is low or even missing and the power line was disconnected
            Serial.println("RTC lost confidence in the DateTime!");
        }
    }

    RtcDateTime now = Rtc.GetDateTime();
    printDateTime(now);
    //Serial.println();

	/*RtcTemperature temp = Rtc.GetTemperature();
	temp.Print(Serial);
	// you may also get the temperature as a float and print it
  // Serial.print(temp.AsFloatDegC());
  Serial.println("C");
  *///
}

//AN INSIDE FUNCTION OF THE RTC DS3231
void printDateTime(const RtcDateTime& dt){
    char datestring[20];

    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u/%02u/%04u,%02u:%02u:%02u"),
            dt.Month(),
            dt.Day(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    Serial.print(datestring);
}

//fUNCTION THAT MEASURES THE VOLTAGE
void VOLTAGE_CALCULATION(){    
   Arduino_Voltage_sensor_pin_Value=analogRead(voltage_sensor_pin);
   V_sensor_OUT=(Arduino_Voltage_sensor_pin_Value*V_reference)/total_ADC_resolution; 
   V_measured=V_sensor_OUT/(R1/(R1+R2)); 
  Serial.print(V_measured,significant_digits);
}

//FUNCTION THAT CALCULATES THE CURRENT WITH LTS 25NP CURRENT SENSOR
void CURRENT_CALCULATION(){
    float I1=0.0; // current of lts 25 np
    lts25np_vout = Step_resolution*analogRead(Current_sensor_pin);  // calculates the output voltage of lts 25np
    
    //Serial.println(analogRead(Current_sensor_pin)); // prints the ADC value of analog input pin ( 0-1023 )
    //Serial.print("lts 25 np V out : ");
    //Serial.println(lts25np_vout,); // prints the output voltage of lts 25np
    //Serial.println("==============");
    Volt_current_value = lts25np_vout - zero_current; // calculates the match of Volts of lts 25np to Amps ( its gives the volts obove the zero current point )
    
    //Serial.print("Lts Vout in Volts : ");
    //Serial.println(Volt_current_value,7); // prints the diference in volta obove the zero current point 
    //Serial.println("==============");
    Lts_25_np_Current = Volt_current_value/scale_factor; // calculates the true current measured from lts 25np in Amps 
    if (Lts_25_np_Current <= 0.00)  // if the current is <=0 then it is measured as zero current
      Lts_25_np_Current=0.00;
    Serial.print(Lts_25_np_Current,significant_digits);   // prints the current measured of lts 25np in Amps
    //Serial.println("==============");
    //Serial.println("");
}



void setup(){
  
    Serial.begin(115200);
    analogReference(EXTERNAL);    // We use the external voltage of LM4040 4.096 Volts
    
    /*Use external voltage reference of lm4040  if we want to measure more precise but we have a deflection of -0.18 approximately volts , 
    without V_reference we have +/- 0.01 or -0.02 volt deflection (the deflection is based on unit 210e measurements)*/
    
    pinMode(pinDHT22,INPUT);
    pinMode(voltage_sensor_pin,INPUT); 
    pinMode(Current_sensor_pin,INPUT);

    Serial.print("compiled: ");
    Serial.print(__DATE__);
    Serial.println(__TIME__);

    //--------RTC SETUP ------------
    // if you are using ESP-01 then uncomment the line below to reset the pins to
    // the available pins for SDA, SCL
    // Wire.begin(0, 2); // due to limited pins, use pin 0 and 2 for SDA, SCL
    
    Rtc.Begin();
 
    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
    printDateTime(compiled);
    Serial.println();

    if (!Rtc.IsDateTimeValid()) 
    {
        if (Rtc.LastError() != 0)
        {
            // we have a communications error
            // see https://www.arduino.cc/en/Reference/WireEndTransmission for 
            // what the number means
            Serial.print("RTC communications error = ");
            Serial.println(Rtc.LastError());
        }
        else
        {
            // Common Causes:
            //    1) first time you ran and the device wasn't running yet
            //    2) the battery on the device is low or even missing

            Serial.println("RTC lost confidence in the DateTime!");

            // following line sets the RTC to the date & time this sketch was compiled
            // it will also reset the valid flag internally unless the Rtc device is
            // having an issue

            Rtc.SetDateTime(compiled);
        }
    }

    if (!Rtc.GetIsRunning())
    {
        Serial.println("RTC was not actively running, starting now");
        Rtc.SetIsRunning(true);
    }

    RtcDateTime now = Rtc.GetDateTime();
    if (now < compiled) 
    {
        Serial.println("RTC is older than compile time!  (Updating DateTime)");
        Rtc.SetDateTime(compiled);
    }
    else if (now > compiled) 
    {
        Serial.println("RTC is newer than compile time. (this is expected)");
    }
    else if (now == compiled) 
    {
        Serial.println("RTC is the same as compile time! (not expected but all is fine)");
    }

    // never assume the Rtc was last configured by you, so
    // just clear them to your needed state
    Rtc.Enable32kHzPin(false);
    Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone); 


    
  // You can use Ethernet.init(pin) to configure the CS pin
  //Ethernet.init(10);  // Most Arduino shields
  //Ethernet.init(5);   // MKR ETH Shield
  //Ethernet.init(0);   // Teensy 2.0
  //Ethernet.init(20);  // Teensy++ 2.0
  //Ethernet.init(15);  // ESP8266 with Adafruit FeatherWing Ethernet
  //Ethernet.init(33);  // ESP32 with Adafruit FeatherWing Ethernet

  // Open serial communications and wait for port to open:
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Ethernet WebServer Example");

  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  // start the server
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());

}


void loop() {
  unsigned long Time_now_for_current_and_voltage=millis();
  if(Time_now_for_current_and_voltage - Time_zero > Time_between_each_measure ){  //MEASURES THE CURRENT  EVERY SOME SECONDS
  Print_date_time_year();
  Serial.print(",");
  Temprature_and_Humidity();
  Serial.print(",");
  VOLTAGE_CALCULATION();
  Serial.print(",");
  CURRENT_CALCULATION();
  Serial.println("");
  Time_zero += Time_between_each_measure; //  EVERY SOME SECONDS
  }



// << FUNCTION FOR WEB SERVER FOR DISPALYING THE VALUES >>

  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    //Serial.println("new client");
    // an HTTP request ends with a blank line
    bool currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        //Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the HTTP request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard HTTP response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 5");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          // output the value of each analog input pin

         
          client.print("Temperature  :");
          client.println(temperature);
          client.println("<br />");  // the next value is displaied in a new line
          
          client.print("Humidity &nbsp;&nbsp;&nbsp;&nbsp;&nbsp:");
          client.println(humidity);
          client.println("<br />");  
          
          client.print("Voltage &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp:");
          client.println(V_measured);
          client.println("<br />");  
          
          client.print("Current &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp:");
          client.println(Lts_25_np_Current);
          client.println("<br />");  
          
          client.println("</html>");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    //Serial.println("client disconnected");
    //Serial.println("");
  }
}
