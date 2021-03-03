#include <SPI.h>
#include <WiFi.h>
#include <Servo.h>
#include <Wire.h>
#include <LSM303.h>

#define MotorPWMPin   2
#define pPropeller   30
#define pRudder      32
#define pBowThrust   34
#define pRearThrust  36
#define neutralValue 1500

char ssid[] = "";
char pass[] = "";

IPAddress ip(192,168,1,50);           //static IP adress for WiFi shield
IPAddress server(192,168,1,66);       //**Enter IP address for the server**
int port = 2390;                      //communication port     
int status = WL_IDLE_STATUS;
WiFiClient client;

Servo sPropeller;
Servo sRudder;
Servo sBowThrust;
Servo sRearThrust;

String rPacket = "";

//Setting Desired RPM Here.
const int DesiredRPM=300;

// incoming serial byte
byte inByte = 0;         

//Data status explanation
//0:find header 0xFA
//1:find 2nd header 0xFA (Lidar is working)
//2:reads the data
unsigned char Data_status=0;

//index of byte in each packet(0-22)
unsigned char Data_loop_index=0;

//Holds the index of each packet(0-89)
unsigned char Data_4deg_index=0;

//Motor speed
unsigned char SpeedRPHhighbyte=0;  
unsigned char SpeedRPHLowbyte=0;
int SpeedRPH=0;

//Distance measurements
unsigned char distanceLowByte=0;
unsigned char distanceHighByte=0;

//Holds the measurements for a single scan
int distanceIndex=0;

const unsigned char PWM4dutyMax=255;
const unsigned char PWM4dutyMin=100;
//set a default value make motor start spinning
unsigned char PWM4duty=PWM4dutyMin;  

boolean newByte = false;

int bytesTransferred=0;
int safe_counter = 0;

bool start_serial = false;
bool sendLidar    = false;
bool startPacket  = false; 
bool IMU_ON       = true;


int  scansSent     = 0;
int previousRudder = 1500;
byte inByteSerial2 = 0;

LSM303 compass;
int imuHeading;
float timer;

void setup() {

    //Initialize Serial Ports
    Serial.begin(115200);   // USB serial
    Serial2.begin(115200);  // Bluetooth data
    Serial3.begin(115200);  // XV-11 LDS data 
    
    //Initialize Lidar pin and set zero speed
    pinMode(MotorPWMPin, OUTPUT); 
    analogWrite(MotorPWMPin, 0 );
    
    //Set static IP to WiFi shield  
    WiFi.config(ip);                   
    
    while (status != WL_CONNECTED) { 
        Serial.println(F("_Trying WiFi connection."));
        status = WiFi.begin(ssid, pass);
        delay(2000);
    }  
    Serial.println(F("_Connected to WiFi."));
    
    while ( !client.connected() ) {
        Serial.println(F("_Trying server connection."));
        client.flush();
        client.stop();
		//**IP Address for the server side**  		
        client.connect(server,port);    
    } 
    Serial.println(F("_Connected to server."));
    client.write('1');
    
    
    //Initialize motor pins
    sPropeller.attach(pPropeller);
    sRudder.attach(pRudder);
    sBowThrust.attach(pBowThrust);
    sRearThrust.attach(pRearThrust);
    
    //Set all motors to neutral
    sPropeller.writeMicroseconds(neutralValue);
    sRudder.writeMicroseconds(neutralValue);
    sBowThrust.writeMicroseconds(neutralValue);
    sRearThrust.writeMicroseconds(neutralValue);
    
    //Lidar starts spinning
    analogWrite(MotorPWMPin, 170 );
    
    //init_IMU();
    
    Serial.println(F("_Setup::Done")); 
    
    if (IMU_ON) {
        Wire.begin();
        compass.init();
        compass.enableDefault();
        compass.m_min = (LSM303::vector<int16_t>){-1515, -2097, -1166};
        compass.m_max = (LSM303::vector<int16_t>){+3037, +2615, +3070};
        timer=millis();
    }
}

void loop() {
        
   //unsigned long StartTime = micros();
   //run_IMU();  
   //unsigned long CurrentTime = micros();
   //unsigned long ElapsedTime = CurrentTime - StartTime;
   //Serial.print("Elapsed time=");
   //Serial.println(ElapsedTime);
 
   while (Serial3.available()) {
        inByte = Serial3.read();
        //Serial.write(inByte);              
        decodeData(inByte);    
   } 

   if ( !client.connected() ) {
        //***ATTENTION***
        Serial.println(F("_Lost Connection::Stopping all Motors"));
        sPropeller.writeMicroseconds(neutralValue);
        sRudder.writeMicroseconds(neutralValue);
        sBowThrust.writeMicroseconds(neutralValue);
        sRearThrust.writeMicroseconds(neutralValue);
      
      //Some code to try and connect again
      while (!client.connected()) {
        client.flush();   
        client.stop();    
        client.connect(server, port);
        Serial.println(F("_Trying server connection."));  
      }
      
      Serial.println(F("_Server::Connected again."));
      client.write('1');
      
   }
   
   
   //Packet Size: 6bytes
   //Packet Codes: 
   //  PR-> propeller
   //  RU-> rudder
   //  BT-> bow thruster
   //  RT-> rear thruster
   
   int clientOn = client.available() ;
   
   if ( clientOn && (clientOn%6==0) ) {
     
        //Serial.println(F("_Something is on the client."));

        for (int i=0 ; i < 6 ; i++ ) {
            rPacket += (char) client.read();
        }
    
        String packetCode =  rPacket.substring(0,2);
        int    pulseWidth =  rPacket.substring(2,7).toInt() ;
    
        //Check that given pulse width is within limits
        if ( (pulseWidth>1000) && (pulseWidth<2000) ) {
            if ( packetCode == "PR" ) {
                  sPropeller.writeMicroseconds(pulseWidth);
                  Serial.print("_propeller moves by ");
                  Serial.println(pulseWidth);
            }     
            if ( packetCode == "RU" ) {
              if ( (previousRudder > 1500) && (pulseWidth==1500) ) {
                    pulseWidth = 1450;
              }
                 sRudder.writeMicroseconds(pulseWidth);
                 Serial.print("_rudders moves by ");
                 Serial.println(pulseWidth);
                 
                 previousRudder = pulseWidth;
           }      
           if ( packetCode == "BT" ) {
                 sBowThrust.writeMicroseconds(pulseWidth);
                 Serial.print("_bow moves by ");
                 Serial.println(pulseWidth);
           }
           if ( packetCode == "RT" ) {
                 sRearThrust.writeMicroseconds(pulseWidth);
                 Serial.print("_rear moves by ");
                 Serial.println(pulseWidth);
           }
     }
     if ( packetCode == "LI" ) {
         sendLidar = true;
         analogWrite(MotorPWMPin, pulseWidth/10 );
         Serial.print("Lidar speed is now: ");
         Serial.println(pulseWidth/10);
      }  
    rPacket="";
  }
}


//decodeData()
void decodeData(unsigned char inByte)
{
  switch (Data_status){
  
      case 0: // no header
          if (inByte==0xFA) {
            Data_status=1;
            Data_loop_index=1;
          }
          break;
      
      case 1: // Find 2nd FA
          if (Data_loop_index==22){
            if (inByte==0xFA) {
              Data_status=2;
              Data_loop_index=1;
            } 
            else // if not FA search again
              Data_status=0;
          }
          else {
            Data_loop_index++;
          }
          break;
        
       case 2: // Read data out
           if (Data_loop_index==22){
                if (inByte==0xFA) {
                    Data_loop_index=1;
                } 
                else {// if not FA search again
                    Data_status=0;
                }
          }
          else {
                //Serial.println("Reading Data");
                readData(inByte);
                Data_loop_index++;
          }
          break;
   }
}

//readData()
void readData(unsigned char inByte){
  
  switch (Data_loop_index){
    
    case 1: // 4 degree index
    
        //Data_4deg_index: [0,89]
        Data_4deg_index = inByte-0xA0;
        
        if (Data_4deg_index==0) {           
            bytesTransferred=0;
            if (Serial2.available()) {
              
                 startPacket = true;
                 inByteSerial2 = Serial2.read();
                 if ( inByteSerial2 ) {
                     startPacket = false;
                     Serial.println(F("_Lidar::Bluetooth off"));
                 }
                 
            }
            //Serial2.write(0xFF);
            //Serial2.write(0xFF);
        }
        //check if index <0 or >89 
        break;
    
    case 2: // Speed in RPH low byte
        SpeedRPHLowbyte=inByte;
        break;
    
    case 3: // Speed in RPH high byte
        SpeedRPHhighbyte=inByte;
        SpeedRPH=(SpeedRPHhighbyte<<8)|SpeedRPHLowbyte;
      //  SpeedControl ( DesiredRPM ) ; 
        break;
        
    case 4:        
        distanceLowByte = inByte;
        if ( start_serial && startPacket ) {     
            Serial2.write(distanceLowByte);
        }
        bytesTransferred++;
        break;
    
    case 5:
        distanceHighByte = ( inByte & 0x3F );
        if ( start_serial && startPacket ) { 
            Serial2.write(distanceHighByte);
        }
        bytesTransferred++;
        //distance = (distanceHighByte<<8)|distanceLowByte;
        //distanceArray[4*Data_4deg_index + 0] = distance;
        break;
    
    case 8:
        distanceLowByte = inByte;
        if ( start_serial && startPacket ) {
            Serial2.write(distanceLowByte);
        }
        bytesTransferred++;
        break;
    
    case 9:
        distanceHighByte = ( inByte & 0x3F );
        if ( start_serial && startPacket ) {
            Serial2.write(distanceHighByte);
        }
        bytesTransferred++;
        //distance = (distanceHighByte<<8)|distanceLowByte;
        //distanceArray[4*Data_4deg_index + 1] = distance;
        break;
    
    case 12:
        distanceLowByte = inByte;
        if ( start_serial && startPacket ) {
            Serial2.write(distanceLowByte);
        }
        bytesTransferred++;
        break;
    
    case 13:
        distanceHighByte = ( inByte & 0x3F );
        if ( start_serial && startPacket ) {
            Serial2.write(distanceHighByte);
        }
        bytesTransferred++;
        //distance = (distanceHighByte<<8)|distanceLowByte;
        //distanceArray[4*Data_4deg_index + 2] = distance;
        break;
    
    case 16:
        distanceLowByte = inByte;
        if ( start_serial && startPacket ) {
            Serial2.write(distanceLowByte);
        } 
        bytesTransferred++;
        break;
    
    case 17:
        distanceHighByte = ( inByte & 0x3F );
        if ( start_serial && startPacket ) {
            Serial2.write(distanceHighByte);
        }
        bytesTransferred++;
        //distance = (distanceHighByte<<8)|distanceLowByte;
        //distanceArray[4*Data_4deg_index + 3] = distance;
        break;
        
    case 21:
         if (Data_4deg_index==89) {
            
             if ( start_serial && startPacket ) {
                if (IMU_ON) {
                  if ( (millis() - timer) >= 200 ) {
                    compass.read();
                    imuHeading = compass.heading();
                  }   
                      if ( imuHeading <= 9 ) {
                        Serial2.print(0);
                        Serial2.print(0);
                        Serial2.print(imuHeading);
                      }
                      else if ( imuHeading > 9 && imuHeading <= 99 ) {
                        Serial2.print(0);
                        Serial2.print(imuHeading);
                      }
                      else
                        Serial2.print(imuHeading);
      
                        timer = millis();
                }
             }             
            startPacket = false;            
            if ( (start_serial==false) && (bytesTransferred==720) ) {
              safe_counter++;
              //Serial.println(safe_counter);
              if ( safe_counter==20 )
                Serial.println(F("_Lidar::Ready."));
            }           
            if ( (safe_counter>20)  && (bytesTransferred==720) ) {  // && (sendLidar) )                          
                start_serial = true;
                //startPacket = true;                            
            }   
        }
        break;     
    default: // others do checksum
        break;
  }  
}

//SpeedControl()
void SpeedControl ( int RPMinput)
{
   // Updates the speed at packets 0,30,60 (3 times)
   if (Data_4deg_index%30==0) {  
   
    if (SpeedRPH < RPMinput*60)
	   // limit the max PWM make sure it don't overflow and make LDS stop working
       if (PWM4duty < PWM4dutyMax) PWM4duty++; 
       
    if (SpeedRPH > RPMinput*60)
		//Have to limit the lowest pwm keep motor running
       if(PWM4duty > PWM4dutyMin)  PWM4duty--;  
   } 
   
   analogWrite(MotorPWMPin, PWM4duty ); //update value
}

//freeRam()
//Returns the amount of the current free SRAM (int)
//Serial.println(freeRam());
int freeRam ()
{
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}


