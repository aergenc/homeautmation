
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <PubSubClient.h>
//#include <BME280.h>
//#include <BME280I2C.h>
//#include <EnvironmentCalculations.h>
#include <Adafruit_BMP280.h>
#define com D4
#define writePin D7
#define role D0
#define alev A0
#define SCL D1
#define SDA D2
//IPAddress ip(192, 168, 0, 109);IPAddress gateway(192, 168, 0, 200);IPAddress subnet(255, 255, 255, 0);
WiFiClient client;
PubSubClient mqttclient(client);
SoftwareSerial ModbusSer(D5, D6, false, 256);
Adafruit_BMP280 bmpgir;
Adafruit_BMP280 bmpcik;
//BME280I2C bmegir;
//BME280I2C bmecik;  
const char* mqttServer="192.168.0.211";
const int mqttPort = 1883;
const char* ipserver = "192.168.0.200";
const char* ssid = "aksicim";
const char* password = "0963258741";
int second=0,minute=0,hour=0,watchdogweb,i,j,trial;

unsigned long  millisecond=0, currentmillis=0, premillis=0,currentmicros=0,premicros=0,milli200ms,milli100ms,sayacdongu;
float tempgir(NAN), presgir(NAN),tempcik(NAN), prescik(NAN);;
int surcom[2],surref[2],surstatus[2],surfeed[2],surerror[2],modbusCom,modbusReg,modbusPar,modbusID[2],IDno,IDnoin,alevseviye;
const int logcomReg=8192,refReg=8193,logstaReg=8448,feedReg=8451,errReg=8449;
byte komut1=1,komut2=0,komut1e=1,komut2e=1,durum1=1,durum2=0,durum3=0,durum4=0,durum1e=1,durum2e=1,displaymode=1;
boolean tickms,ticks,tick200ms,tickm,tickh,sendserial,sendmqtt,cikisok,datareceived,measureok;  
boolean haberlesmehata,readstatus[2],readfeed[2],readerror[2],writecommand[2],writeref[2],readstatusok[2],readfeedok[2],readerrorok[2],writecommandok[2],writerefok[2],readok,writeok,newmodbus;

byte mqttsenddata[20];int mqttreceivedata[20];  String mqttreceivestr[20];

struct modbus{  byte modbusID,modbusbytecount, modbusIDin,modbusCom, modbusin[40], modbusout[40]; int modbusreg,modbusregnum,modbustimer,mk,mi;unsigned CRC,CRCgelen,CRCgiden; };
typedef struct modbus ModBusrtu; 

ModBusrtu ModbusRTU;

void setup() {pinMode(writePin,OUTPUT);pinMode(com,OUTPUT);pinMode(role,OUTPUT);
ModbusSer.begin(19200); Serial.begin(115200);Wire.begin();
for (ModbusRTU.mi=0;ModbusRTU.mi<20;ModbusRTU.mi++){ ModbusRTU.modbusin[ModbusRTU.mi]=ModbusRTU.mi;ModbusRTU.modbusout[ModbusRTU.mi]=ModbusRTU.mi;}
modbusID[0]=100;modbusID[1]=101;
if (!bmpgir.begin(0x76)) {Serial.println(F("Could not find a valid BMP280 input sensor, check wiring!"));}
if (!bmpcik.begin(0x77)) {Serial.println(F("Could not find a valid BMP280 output sensor, check wiring!"));}
// while(!bmegir.begin()) { Serial.println("Could not find BME280 sensor!");  }
//  switch(bmegir.chipModel())
//  {  case BME280::ChipModel_BME280:
//       Serial.println("Found BME280 sensor! Success.");       break;
//     case BME280::ChipModel_BMP280:
//       Serial.println("Found BMP280 sensor! No Humidity available.");       break;
//     default:
//       Serial.println("Found UNKNOWN sensor! Error!");}
WIFI_Connect();  MQTTreconnect(); }
//attachInterrupt(digitalPinToInterrupt(encoderin),encoderread,RISING); digitalWrite(writePin,LOW);}


void mqttsend(){ mqttsenddata[0]=2; mqttsenddata[1]=durum1;mqttsenddata[2]=durum2;
                 mqttsenddata[3]=lowByte(surstatus[0]);mqttsenddata[4]=highByte(surstatus[0]); mqttsenddata[5]=lowByte(surstatus[1]);mqttsenddata[6]=highByte(surstatus[1]);
                 mqttsenddata[7]=lowByte(surfeed[0]);mqttsenddata[8]=highByte(surfeed[0]); mqttsenddata[9]=lowByte(surfeed[1]);mqttsenddata[10]=highByte(surfeed[1]);
                 mqttsenddata[11]=byte(surerror[0]);  mqttsenddata[12]=byte(surerror[1]); 
                 mqttsenddata[13]=lowByte(int(tempgir*10)); mqttsenddata[14]=highByte(int(tempgir*10)); 
                 mqttsenddata[15]=lowByte(int((presgir-101000)*10));mqttsenddata[16]=highByte(int((presgir-101000)*10)); 
                 mqttsenddata[17]=lowByte(int(tempcik*10)); mqttsenddata[18]=highByte(int(tempcik*10)); 
                 mqttsenddata[19]=lowByte(int((prescik-101000)*10));mqttsenddata[20]=highByte(int((prescik-101000)*10)); 
    mqttclient.publish("SomineDurum", mqttsenddata,21);
    Serial.print("Sending Message:");  for (int i = 0; i < 21; i++) {    Serial.print(mqttsenddata[i]);Serial.print(", ");}
    Serial.println();  Serial.println("-----------------------");digitalWrite(com,0); 
    }
    
void mqttreceive(char* topic, byte* payload, unsigned int length) {int j=-1;  for (int i=0;i<19; i++) {mqttreceivestr[i]=' ';} 
   Serial.print("Receiving Message:");  for (int i = 0; i < length; i++) {    Serial.print(char(payload[i]));
  if ((char(payload[i])!='[') and (char(payload[i])!=',') and (char(payload[i])!=']')) {mqttreceivestr[j]=mqttreceivestr[j]+String(char(payload[i]));} else j=j+1;}  Serial.println();  Serial.println("-----------------------");
  for (int i=0;i<19; i++) {mqttreceivedata[i]=mqttreceivestr[i].toInt();}
  
  komut1=(komut1 xor lowByte(mqttreceivedata[0]));  komut2=(komut2 xor highByte(mqttreceivedata[0]));surcom[0]=( mqttreceivedata[1]);surcom[1]=( mqttreceivedata[2]);
  surref[0]=mqttreceivedata[3];  surref[1]=mqttreceivedata[4];   
  watchdogweb=0; sendmqtt=false;  writecommand[0]=false; digitalWrite(com,1); }
  
void MQTTreconnect() {  // Loop until we're reconnected
  mqttclient.setServer(mqttServer, mqttPort);
  mqttclient.setCallback(mqttreceive); trial=0; 
  while (!mqttclient.connected() && trial<5) {    Serial.print("Attempting MQTT connection...");
     if (mqttclient.connect("SomineESP","aergenc","mart8377")) {  trial=0;    Serial.println("connected");
       mqttclient.publish("SomineDurum", "Hello from Somine");mqttclient.subscribe("SomineKomut"); }    else {      Serial.print("failed, rc=");      Serial.print(mqttclient.state());      Serial.println(" try again in 5 seconds");
      delay(2000);  trial++;  }  }}  
 

void WIFI_Connect(){  WiFi.disconnect();  Serial.println("Booting Sketch...");  WiFi.mode(WIFI_STA);  WiFi.begin(ssid, password);    // Wait for connection
  if (WiFi.status() != WL_CONNECTED) {delay(500);Serial.print(".");} else {Serial.println(""); Serial.print("Connected to: "); Serial.println(ssid); Serial.print("IP address: "); 
Serial.println(WiFi.localIP()); }}


void ModbusReceive(){newmodbus=true;for (ModbusRTU.mi=0;ModbusRTU.mi<40;ModbusRTU.mi++){ ModbusRTU.modbusin[ModbusRTU.mi]=ModbusSer.read();delay(1);}
                     ModbusRTU.modbusIDin=ModbusRTU.modbusin[0];ModbusRTU.modbusCom=ModbusRTU.modbusin[1]; 
                     if  (ModbusRTU.modbusIDin==modbusID[0]) IDnoin=0; if  (ModbusRTU.modbusIDin==modbusID[1]) IDnoin=1;
                     if (ModbusRTU.modbusCom==3){
                     if (readstatus[IDnoin] && !readfeed[IDnoin]){surstatus[IDnoin]=(ModbusRTU.modbusin[3]<<8)+ModbusRTU.modbusin[4];surerror[IDnoin]=(ModbusRTU.modbusin[5]<<8)+ModbusRTU.modbusin[6];
                                                                  readstatusok[IDnoin]=true ;readerrorok[IDnoin]=true;}//Serial.print("status:");Serial.print(IDnoin);Serial.print(":");Serial.println(surstatus[IDnoin]); } 
                     if (readfeed[IDnoin]){surfeed[IDnoin]=(ModbusRTU.modbusin[3]<<8)+ModbusRTU.modbusin[4];readfeedok[IDnoin]=true;}//Serial.print("feed:");Serial.print(IDnoin);Serial.print(":");Serial.println(surfeed[IDnoin]);}
                     }
                     if (ModbusRTU.modbusCom==6){
                     if (writecommand[IDnoin]){writecommandok[IDnoin]=true;}//Serial.print("command:");Serial.println(IDnoin); } 
                     if (writeref[IDnoin]){writerefok[IDnoin]=true;}}//Serial.print("ref:");Serial.println(IDnoin);} }
                     //Serial.print("okumafeed: "); for(j=0;j<10;j++) {Serial.print(ModbusRTU.modbusin[j]);Serial.print(":");} Serial.println("serison5");
                     }

void ModbusCommandSend(){digitalWrite(writePin,HIGH);
  //MODBUS Yazma
  ModbusRTU.modbusout[0]=modbusID[IDno]; ModbusRTU.modbusout[1]=modbusCom;ModbusRTU.modbusout[2]=highByte(modbusReg);  ModbusRTU.modbusout[3]=lowByte(modbusReg); ModbusRTU.modbusout[4]=highByte(modbusPar);  ModbusRTU.modbusout[5]=lowByte(modbusPar); 
  ModbusRTU.CRCgiden=ModRTU_CRC(ModbusRTU.modbusout,6);ModbusRTU.modbusout[6]=lowByte(ModbusRTU.CRCgiden); ModbusRTU.modbusout[7]=highByte(ModbusRTU.CRCgiden);
  digitalWrite(writePin,HIGH);delay(1);
                                  for (ModbusRTU.mk=0;ModbusRTU.mk<8;ModbusRTU.mk++) { ModbusSer.write(ModbusRTU.modbusout[ModbusRTU.mk]); } delay(0); digitalWrite(writePin,LOW);
   //Serial.print("beginmodbusout ");for (ModbusRTU.mk=0;ModbusRTU.mk<8;ModbusRTU.mk++) {Serial.print(ModbusRTU.modbusout[ModbusRTU.mk],HEX);Serial.print(":");} Serial.println("end "); 
   }
void ModbusRead(){digitalWrite(writePin,HIGH);
  //MODBUS Yazma
  ModbusRTU.modbusout[0]=modbusID[IDno]; ModbusRTU.modbusout[1]=modbusCom;ModbusRTU.modbusout[2]=highByte(modbusReg);  ModbusRTU.modbusout[3]=lowByte(modbusReg); ModbusRTU.modbusout[4]=highByte(modbusPar);  ModbusRTU.modbusout[5]=lowByte(modbusPar); 
  ModbusRTU.CRCgiden=ModRTU_CRC(ModbusRTU.modbusout,6);ModbusRTU.modbusout[6]=lowByte(ModbusRTU.CRCgiden); ModbusRTU.modbusout[7]=highByte(ModbusRTU.CRCgiden);
  digitalWrite(writePin,HIGH);delay(1);
                                  for (ModbusRTU.mk=0;ModbusRTU.mk<8;ModbusRTU.mk++) { ModbusSer.write(ModbusRTU.modbusout[ModbusRTU.mk]); } delay(0); digitalWrite(writePin,LOW);
   //Serial.print("beginmodbusout ");for (ModbusRTU.mk=0;ModbusRTU.mk<8;ModbusRTU.mk++) {Serial.print(ModbusRTU.modbusout[ModbusRTU.mk],HEX);Serial.print(":");} Serial.println("end "); 
   }
unsigned  ModRTU_CRC(byte buf[20], int len){  unsigned int crc = 0xFFFF;
      for (int pos = 0; pos < len; pos++) {   crc ^= (unsigned int)buf[pos];   // XOR byte into least sig. byte of crc
      for (int i = 8; i != 0; i--) {   if ((crc & 0x0001) != 0) {  crc >>= 1; crc ^= 0xA001;}   else    crc >>= 1;   } }  return crc;  }
//void ReadTempgir
//(   Stream* client)
//{  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
//   BME280::PresUnit presUnit(BME280::PresUnit_Pa);
//
//   bmegir.read(presgir, tempgir, humgir, tempUnit, presUnit);
//
//   client->print("Temp: ");   client->print(tempgir);   client->print("°"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
//   client->print("\t\tHumidity: ");   client->print(humgir);   client->print("% RH");   client->print("\t\tPressure: ");
//   client->print(presgir);   client->println(" Pa");
//}     
void ReadTempgir(){Serial.print(F("TemperatureGir = "));  tempgir=bmpgir.readTemperature();  Serial.print(tempgir);    Serial.println(" *C");
                   Serial.print(F("Pressure = "));    presgir=bmpgir.readPressure(); Serial.print(presgir);    Serial.println(" Pa");}     

void ReadTempcik(){ Serial.print(F("TemperatureCik = "));  tempcik=bmpcik.readTemperature();Serial.print(tempcik);   Serial.println(" *C");
                    Serial.print(F("Pressure = "));    prescik=bmpcik.readPressure(); Serial.print(prescik);    Serial.println(" Pa");} 
void loop() { currentmillis=millis();
 if ((currentmillis-premillis)>=10) {sayacdongu=currentmillis-premillis; millisecond+=10;milli200ms+=10;milli100ms+=10; tickms=!tickms; premillis=currentmillis;watchdogweb+=10; } 
 if (milli100ms>=100){ milli100ms=0; }
 if (milli200ms>=200){tick200ms=true;milli200ms=0;}
 if (millisecond>=1000){ millisecond=0; second+=1; ticks=!ticks;sendserial=false; cikisok=false;measureok=false; writeok=false;  writecommand[0]=false; readstatus[0]=false;readstatus[1]=false;}
  if (second>=60) {second=0; tickm=true;    }
// digitalWrite(encind,digitalRead(encoderin));
 if (!mqttclient.connected()) {    MQTTreconnect();  } mqttclient.loop();
 if (ModbusSer.available()>0 ) { ModbusReceive();   }

 if (!writecommand[0])   {writecommand[0]=true; writecommand[1]=false;  writeref[0]=false;  writecommandok[0]=false;IDno=0; modbusCom=6; modbusReg=logcomReg; modbusPar=surcom[0]; ModbusCommandSend(); } 
 if (millisecond==100 && !writeref[0])       {writeref[0]=true;     writerefok[0]=false;    IDno=0; modbusCom=6; modbusReg=refReg;    modbusPar=surref[0]; ModbusCommandSend(); } 
  
 if (millisecond==150 && !readstatus[0])  {readstatus[0]=true; readfeed[0]=false; readstatusok[0]=false; IDno=0; modbusCom=3; modbusReg=logstaReg; modbusPar=2; ModbusRead(); } 
 if (millisecond==200 && !readfeed[0])    {readfeed[0]=true;   readfeedok[0]=false;   IDno=0; modbusCom=3; modbusReg=feedReg;   modbusPar=1; ModbusRead(); } 
 

 if (millisecond==300 && !writecommand[1])   {writecommand[1]=true; writeref[1]=false; writecommandok[1]=false; IDno=1; modbusCom=6; modbusReg=logcomReg; modbusPar=surcom[1]; ModbusCommandSend(); } 
 if (millisecond==350 && !writeref[1])       {writeref[1]=true;  writerefok[1]=false;      IDno=1; modbusCom=6; modbusReg=refReg;    modbusPar=surref[1]; ModbusCommandSend(); } 
  
 if (millisecond==400 && !readstatus[1])  {readstatus[1]=true; readfeed[1]=false; readstatusok[1]=false; IDno=1; modbusCom=3; modbusReg=logstaReg; modbusPar=2;  ModbusRead(); } 
 if (millisecond==450 && !readfeed[1])    {readfeed[1]=true;  readfeedok[1]=false;  IDno=1; modbusCom=3; modbusReg=feedReg;   modbusPar=1; ModbusRead(); } 



 
 if(millisecond==500 && !measureok) { measureok=true;  ReadTempgir();ReadTempcik(); //ReadTempgir(&Serial); 
                  alevseviye=analogRead(A0); Serial.print("alevseviye:");Serial.println(alevseviye);
                  if (alevseviye<500) {bitWrite(durum1,2,1);} else {bitWrite(durum1,2,0);}   }
 //
// if (!writeok){ writeok=true; Serial.print("sur0stat "); Serial.println(surstat[0]);
//                              Serial.print("sur0feed "); Serial.println(surfeed[0]);
//                              Serial.print("sur0error "); Serial.println(surerror[0]);
// }

 
 
 if(millisecond==600 &&!cikisok){cikisok=true; digitalWrite(role,bitRead(komut1,0));
                                               bitWrite(durum1,0,bitRead(komut1,0));
                                               bitWrite(durum2,0,writecommandok[0]);bitWrite(durum2,1,writecommandok[1]);
 }
 if(millisecond==700 &&!sendmqtt){ sendmqtt=true; mqttsend();}                                 

 //yield();
 if (tickm){tickm=false; if (WiFi.status() != WL_CONNECTED) WIFI_Connect(); }


}
