#include <RF24Network.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SPI.h>

#define ADC_VREF_mV    5000.0 // in millivolt
#define ADC_RESOLUTION 1024.0
#define tempsensor A0

// Motor B
 
int enB = 3;
int in3 = 5;
int in4 = 4;

//MPU6050
#include <Wire.h>
int ADXL345 = 0x53; // The ADXL345 sensor I2C address

float X_out, Y_out, Z_out;  // Outputs



RF24 radio(7,8);               // nRF24L01 (CE,CSN)
RF24Network network(radio); 

const uint16_t OBC = 015;
const uint16_t ADCS = 021;

char incomingData[32]= "C023";

void setup() {  
  SPI.begin();
  radio.begin();
  network.begin(90, ADCS);  //(channel, node address)
  radio.setDataRate(RF24_2MBPS); 
  Serial.begin(9600);
  pinMode(tempsensor, INPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  Wire.begin(); // Initiate the Wire library
  // Set ADXL345 in measuring mode
  Wire.beginTransmission(ADXL345); // Start communicating with the device 
  Wire.write(0x2D); // Access/ talk to POWER_CTL Register - 0x2D
  // Enable measurement
  Wire.write(8); // (8dec -> 0000 1000 binary) Bit D3 High for measuring enable 
  Wire.endTransmission();
  delay(10);

  //Off-set Calibration
  //X-axis
  Wire.beginTransmission(ADXL345);
  Wire.write(0x1E);
  Wire.write(1);
  Wire.endTransmission();
  delay(10);
  //Y-axis
  Wire.beginTransmission(ADXL345);
  Wire.write(0x1F);
  Wire.write(-2);
  Wire.endTransmission();
  delay(10);

  //Z-axis
  Wire.beginTransmission(ADXL345);
  Wire.write(0x20);
  Wire.write(-9);
  Wire.endTransmission();
  delay(10);
}

void loop() { 
  network.update();
  //===== Receiving =====//        
  while ( network.available() ) 
  {     // Is there any incoming data?   
    RF24NetworkHeader header;
    network.read(header, &incomingData, sizeof(incomingData)); // Read the incoming data
    Serial.println(incomingData);      

     
  }
    /*   //===== Sending =====// 
        char msg[] =  "C004";
        RF24NetworkHeader header4(OBC);
        network.write(header4,&msg, sizeof(msg)); // Send the data
        Serial.println(msg);*/
         if (strcmp(incomingData,"C002")==0)
    {
      //===== Sending =====//
        Serial.println("Command: Get Temprature Sensor Reading ");
        Serial.println(incomingData);
        // get the ADC value from the temperature sensor
        float adcVal = analogRead(tempsensor);
        // convert the ADC value to voltage in millivolt
        float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
        // convert the voltage to the temperature in Celsius
        float tempC = milliVolt / 10;

        String Temp_str = "T@ " + String(tempC, 4);
        char Temp_str_RX[32] ; 
        Temp_str.toCharArray (Temp_str_RX, Temp_str.length()+1);
        RF24NetworkHeader header1(OBC);
        network.write(header1, &Temp_str_RX, sizeof(Temp_str_RX)); // Send the data
        Serial.println(Temp_str);
        delay(5000);
      }
      
     

      else if (strcmp(incomingData,"C013")==0)
    {
       Serial.println("Command: Get accelerometer data ");
       Serial.println(incomingData);
      // === Read acceleromter data === //
        Wire.beginTransmission(ADXL345);
        Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
        X_out = ( Wire.read()| Wire.read() << 8); // X-axis value
        X_out = X_out/256; //For a range of +-2g, we need to divide the raw values by 256, according to the datasheet
        Y_out = ( Wire.read()| Wire.read() << 8); // Y-axis value
        Y_out = Y_out/256;
        Z_out = ( Wire.read()| Wire.read() << 8); // Z-axis value
        Z_out = Z_out/256;
      
      // Print the values on the serial monitor
        String Acc_str;
        char Acc_str_RX[64] ;

        Acc_str = "T@ x = " + String(X_out, 4) +" | y = " + String(Y_out, 4) +" | z = " + String(Z_out, 4) ;
        Acc_str.toCharArray (Acc_str_RX, Acc_str.length()+1);
        RF24NetworkHeader header1(OBC);
        network.write(header1, &Acc_str_RX, sizeof(Acc_str_RX)); // Send the data
        Serial.println(Z_out);
        Serial.println(Acc_str_RX);
        delay(5000);

      }
      
      
      else if (strcmp(incomingData,"C021")==0)
    {
      //===== Sending =====//
        Serial.println("Command: RW HIGH SPEED");
        Serial.println(incomingData);
        
        
        // Turn on motors         
        analogWrite(enB, 125);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW); 

        char speed_RX[32]= "T@" ; 
        RF24NetworkHeader header1(OBC);
        network.write(header1, &speed_RX, sizeof(speed_RX)); // Send the data
        Serial.println("MOTOR IS ON");
        Serial.println(speed_RX);
        delay(5000);
      }
      else if (strcmp(incomingData,"C022")==0)
      {
      //===== Sending =====//
        Serial.println("Command: RW LOW SPEED");
        Serial.println(incomingData);
        
        // Turn on motors         
        analogWrite(enB, 75);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW); 
      
        char speed_RX[32]= "T@" ; 
        RF24NetworkHeader header1(OBC);
        network.write(header1, &speed_RX, sizeof(speed_RX)); // Send the data

        Serial.println("MOTOR IS ON");
        Serial.println(speed_RX);
        delay(5000);
      }
        else if (strcmp(incomingData,"C023")==0)
      {
      //===== Sending =====//
        Serial.println("Command: RW STOP");
        Serial.println(incomingData);
        
        // Turn on motors  
        if (digitalRead(in3) == HIGH or digitalRead(in4) == HIGH){       
        
        // Now turn off motor
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);          
      
      }    
                   
        char speed_RX[32]= "T@" ; 
        RF24NetworkHeader header1(OBC);
        network.write(header1, &speed_RX, sizeof(speed_RX)); // Send the data
        Serial.println("MOTOR IS OFF");
        Serial.println(speed_RX);
        delay(5000);
      }
      
     else {
      //===== Sending =====// 
        char error_msg[] =  "T!";
        String msg = "Error in Received Command"; 
        RF24NetworkHeader header4(OBC);
        network.write(header4,error_msg, sizeof(error_msg)); // Send the data
        Serial.println(incomingData);
        Serial.println(msg);
        delay(5000);
      }


}
  
