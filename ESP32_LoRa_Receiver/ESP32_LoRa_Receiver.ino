#include "RtcDS3231.h"
#include "Wire.h"
#include <time.h> // Estas bibliotecas são exclusivas do sistema. Estão presentes nos packets hardware ESP32.
#include <sys/time.h> // Utilizam o contador interno do EPS32.
#include "WiFi.h"
#include "TinyGPS.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "LoRa.h"
//ESP.restart();
//////////////////////////////// Wifi configuration ////////////////////////////////
const char* ssid     = "Your_SSID"; // Insira aqui os dados de sua rede Wi-Fi
const char* password = "Your_PASSWORD";
///////////////////////////////// Time configuration ///////////////////////////////
long timezone = -4;
struct tm timeinfo;
unsigned long dly = 0 ;
////////////////////////////// SD Card configuration ///////////////////////////////
#define SD_CS 27 //default 15 
#define SD_SCK 14 // default 14
#define SD_MOSI 12 // default 13
#define SD_MISO 13 // 12
SPIClass sd_spi(HSPI);
unsigned long recivedata = 0 ;
String DataMessage = "\0";
String MsgPacket = "\0";
String TimeLocal = "\0";
String MsgErro = "\0";
String Erro = "\0";
String Sinal = "\0";
/////////////////////////////////// Functions //////////////////////////////////////
void Task(void *Parametro);
void appendFile(fs::FS &fs, const char * path, const char * message);
void GetTimeLocal();
void ErrorMsg();
void cbk(int packetSize);
void GPSdata();
double calcDist(float ilat, float ilog, float flat, float flon);
double regress(double x);
/////////////////////////////// Lora configuration /////////////////////////////////
#define BAND    433E6
#define PABOOST true
#define LoRa_SS 5
#define LoRa_MOSI 18
#define LoRa_MISO 19
#define LoRa_SCK 21
#define LoRa_RST 26
#define Lora_DIO0 15
String packSize;
/////////////////////////////// RTC configuration /////////////////////////////////
RtcDS3231<TwoWire> Rtc(Wire);
/////////////////////////////// GPS configuration /////////////////////////////////
TinyGPS gps;
String SinalGPS = "\0";
float flat, flon, alt, ilat, ilog;
double dist;

////////////////////////////////////SETUP INIT////////////////////////////////////////
void setup() {
  ///////////////////////////////////// UART Init ////////////////////////////////////
  Serial.begin(115200);
  Serial2.begin (9600, SERIAL_8N1, 4, 15); //rx , tx - GPS
  Serial.flush();
  delay(50);
  String DateCompile = ("Compiled in: " + String(__DATE__) + " " + String(__TIME__) + "\n");
  Serial.println(DateCompile);
  pinMode(2, OUTPUT);
  ////////////////////////////////// SD Card Init ////////////////////////////////////
  sd_spi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, sd_spi)) {
    Serial.println("Card Mount Failed");
    digitalWrite(2, HIGH);
    return;
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    digitalWrite(2, HIGH);
    return;
  }
  ///////////////////////////////////// Wifi Init ////////////////////////////////////
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  delay(1000);
  for (int i = 0; i < 3; i++) {
    if (WiFi.status() == WL_CONNECTED) {
      digitalWrite(2, HIGH);
      delay(1000);
      digitalWrite(2, LOW);
      Serial.println("WiFi connected");
      MsgErro = "WiFi connected. ";
      appendFile(SD, "/logRX.txt", MsgErro.c_str());
      MsgErro = "\0";
      MsgErro = "IP address: ";
      MsgErro += WiFi.localIP().toString().c_str();
      MsgErro += "\n";
      Serial.print(MsgErro);
      appendFile(SD, "/logRX.txt", MsgErro.c_str());
      MsgErro = "\0";
      break;
    }
    Serial.print("Reconnecting to ");
    Serial.print(ssid);
    long inicio = (micros() + 10000000);
    while (inicio > micros()) {
      //if (wifi != "CONNECTED") {
      if (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      } else break;
    }
    Serial.println("\nConnection fail");
    MsgErro = "Wi-Fi Connection fail\n";
    appendFile(SD, "/logRX.txt", MsgErro.c_str());
    MsgErro = "\0";
  }
  ///////////////////////////////////// RTC Init ////////////////////////////////////
  Wire.begin(16, 17); //SDA, SCL
  Rtc.Begin();
  if (WiFi.status() != WL_CONNECTED) {
jump:
    MsgErro = "RTC used time\n";
    appendFile(SD, "/logRX.txt", MsgErro.c_str());
    MsgErro = "\0";
    timeval tv;//Cria a estrutura temporaria para funcao abaixo.
    tv.tv_sec = (Rtc.GetDateTime() + 946684800);//Atribui a data atual.
    settimeofday(&tv, NULL);//Configura o RTC para manter a data atribuida
    if ((RtcDateTime(__DATE__, __TIME__) + 120) > Rtc.GetDateTime()) {
      appendFile(SD, "/logRX.txt", DateCompile.c_str());
      appendFile(SD, "/gpsRX.txt", "LAT,LOG,SAT,Precisão,Altura,Date GPS,Time GPS\n");
      appendFile(SD, "/dataRX.txt", "LocalDate,LocalTime,LocalLAT,LocalLOG,Distance,RSSI,SNR,PackSize,TxDate,TxTime,DHT01Temp,DHT01Hum,DHT02Temp,DHT02Hum,DHT03Temp,DHT03Hum,PT100,LAT,LOG\n");
    }
  }
  else {
    Serial.println("Contacting Time Server");
    int cont = 0;
repeat:
    configTime(3600 * timezone, 3600, "pool.ntp.org");
    getLocalTime(&timeinfo, 5000);
    Serial.println(&timeinfo, "Now is: %A, %B %d %Y %H:%M:%S");
    int ano = timeinfo.tm_year + 1900;
    int mes = timeinfo.tm_mon + 1;
    int dia = timeinfo.tm_mday;
    int hora = timeinfo.tm_hour;
    int minuto = timeinfo.tm_min;
    int segundo = timeinfo.tm_sec;
    cont++;
    if (cont > 10)goto jump;
    if (ano < 2020) goto repeat;
    RtcDateTime HorarioServer = RtcDateTime(ano, mes, dia, hora, minuto, segundo);
    if (!Rtc.IsDateTimeValid()) {
      if (Rtc.LastError() != 0) {
        MsgErro = "RTC communications error = " + Rtc.LastError();
        appendFile(SD, "/logRX.txt", MsgErro.c_str());
        MsgErro = "\0";
        Serial.print("RTC communications error = ");
        Serial.println(Rtc.LastError());
      }
      else {
        MsgErro = "RTC lost confidence in the DateTime!";
        appendFile(SD, "/logRX.txt", MsgErro.c_str());
        MsgErro = "\0";
        Serial.println("RTC lost confidence in the DateTime!");
        Rtc.SetDateTime(HorarioServer);
      }
    }
    if (!Rtc.GetIsRunning()) {
      MsgErro = "RTC was not actively running, starting now";
      appendFile(SD, "/logRX.txt", MsgErro.c_str());
      MsgErro = "\0";
      Serial.println("RTC was not actively running, starting now");
      Rtc.SetIsRunning(true);
    }
    if ( ((Rtc.GetDateTime() + 60) < HorarioServer) && (ano >= 2020))  {
      MsgErro = "RTC is older than server time!  (Updating DateTime)";
      appendFile(SD, "/logRX.txt", MsgErro.c_str());
      MsgErro = "\0";
      Serial.println("RTC is older than server time!  (Updating DateTime)");
      Rtc.SetDateTime(HorarioServer);
    }
    if ( (Rtc.GetDateTime()  > (HorarioServer + 60)) && (ano >= 2020))  {
      MsgErro = "RTC is advanced than server time!  (Updating DateTime)";
      appendFile(SD, "/logRX.txt", MsgErro.c_str());
      MsgErro = "\0";
      Serial.println("RTC is advanced than server time!  (Updating DateTime)");
      Rtc.SetDateTime(HorarioServer);
    }
    if ((RtcDateTime(__DATE__, __TIME__) + 120) > HorarioServer) {
      appendFile(SD, "/logRX.txt", DateCompile.c_str());
      appendFile(SD, "/gpsRX.txt", "LAT,LOG,SAT,Precisão,Altura,Date GPS,Time GPS\n");
      appendFile(SD, "/dataRX.txt", "LocalDate,LocalTime,LocalLAT,LocalLOG,Distance,RSSI,SNR,PackSize,TxDate,TxTime,DHT01Temp,DHT01Hum,DHT02Temp,DHT02Hum,DHT03Temp,DHT03Hum,PT100,LAT,LOG\n");
    }
  }
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  //////////////////////////////////// LoRa Init /////////////////////////////////////
  SPI.begin(LoRa_SCK, LoRa_MISO, LoRa_MOSI, LoRa_SS);
  LoRa.setPins(LoRa_SS, LoRa_RST, Lora_DIO0); // default (5,14,26)
  if (!LoRa.begin(BAND, PABOOST)) {
    Serial.print("Starting LoRa failed!\r\n");
    MsgErro = "Starting LoRa failed!";
    ErrorMsg();
    digitalWrite(2, HIGH);
    delay(10000);
    digitalWrite(2, LOW);
  }
  else {
    //LoRa.setSpreadingFactor(11); //default SF11 {6-12}
    Serial.print("LoRa Initial success!\r\n");
    MsgErro = "LoRa Initial success!";
    ErrorMsg();
  }
  ///////////////////////////////// Create Secondary Task /////////////////////////////////
  xTaskCreatePinnedToCore (Task, "Task", 8192, NULL, 0, NULL, 0);
  disableCore0WDT();
  //////////////////////////////////// END SETUP /////////////////////////////////////
  GetTimeLocal();
  MsgErro = ("Start system in: " + TimeLocal + "\n");
  appendFile(SD, "/logRX.txt", MsgErro.c_str());
  MsgErro = "\0";
}
void loop()
{
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    cbk(packetSize);
    recivedata = micros ();
  }
  if ((recivedata + 90000000) <= micros()) { // this delay must be synchronized with the transmission interval
    recivedata = micros ();
    SPI.begin(LoRa_SCK, LoRa_MISO, LoRa_MOSI, LoRa_SS);
    LoRa.setPins(LoRa_SS, LoRa_RST, Lora_DIO0); // default (5,14,26)
    if (!LoRa.begin(BAND, PABOOST)) {
      Serial.print("Starting LoRa failed!\r\n");
      MsgErro = "Starting LoRa failed!";
      ErrorMsg();
      pinMode(2, OUTPUT);
      digitalWrite(2, HIGH);
      delay(10000);
      digitalWrite(2, LOW);
    }
    else {
      Serial.print("LoRa Initial success!\r\n");
      MsgErro = "LoRa Initial success!";
      ErrorMsg();
    }
  }
  if (DataMessage != MsgPacket) {
    //appendFile(SD, "/dataRX.txt", DataMessage.c_str());
    MsgPacket = DataMessage;
    if (flat != 0 ) {
      String latStr, logStr;
      latStr = DataMessage.substring(60, 70);
      logStr = DataMessage.substring(71, 81);
      ilat = latStr.toFloat();
      ilog = logStr.toFloat();
      dist = calcDist (ilat, ilog, flat, flon);
    }
    GetTimeLocal();
    Sinal = LoRa.packetRssi();
    Sinal = "RSSI: " + Sinal + " Packet Size: " + packSize + " Distance: " + String(dist);
    Serial.println(Sinal);
    Sinal = TimeLocal + "," + SinalGPS + "," + String(dist) + "," + LoRa.packetRssi() + "," + LoRa.packetSnr() + "," + packSize + "," + DataMessage;
    appendFile(SD, "/DataRX.txt", Sinal.c_str());
    Serial.print(Sinal);//
  }
}

void cbk(int packetSize) {
  DataMessage = "\0";
  packSize = String(packetSize, DEC);
  //Serial.println("Received " + packSize + " bytes");
  for (int i = 0; i < packetSize; i++) {
    DataMessage += (char) LoRa.read();
  }
  //Serial.println (DataMessage);
}

void Task(void *Parametro) {
  delay(100);
  (void) Parametro;
  for (;;) { // A Task shall never return or exit.
    GPSdata();
  }
}

void ErrorMsg() {
  if (MsgErro != Erro) {
    Erro = MsgErro;
    GetTimeLocal();
    TimeLocal += " => ";
    TimeLocal += MsgErro + "\n";
    appendFile(SD, "/logRX.txt", TimeLocal.c_str());
    TimeLocal = "\0";
  }
}

void appendFile(fs::FS &fs, const char * path, const char * message) {
  sd_spi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, sd_spi)) {
    Serial.println("Card Mount Failed");
    digitalWrite(2, HIGH);
    while (true);
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    digitalWrite(2, HIGH);
    while (true);
  }
  //Serial.printf("Appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    digitalWrite(2, HIGH);
    while (true);
  }
  if (file.print(message)) {
    //Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
    digitalWrite(2, HIGH);
    while (true);
  }
  file.close();
}

void GetTimeLocal()
{
  if (!getLocalTime(&timeinfo)) {
    MsgErro = "Failed to obtain time";
    ErrorMsg();
    return;
  }
  String dia = String (timeinfo.tm_mday);
  if ((timeinfo.tm_mday) < 10) dia = + "0" + dia;
  String mes = String (timeinfo.tm_mon + 1);
  if ((timeinfo.tm_mon + 1) < 10) mes = "0" + mes;
  String ano = String (timeinfo.tm_year + 1900); //1900
  ano = ano.substring(2, 4);
  String hora = String (timeinfo.tm_hour);
  if ((timeinfo.tm_hour) < 10) hora = "0" + hora;
  String minuto = String (timeinfo.tm_min);
  if ((timeinfo.tm_min) < 10) minuto = "0" + minuto;
  String segundo = String (timeinfo.tm_sec);
  if ((timeinfo.tm_sec) < 10) segundo = "0" + segundo;
  TimeLocal = dia + "/" + mes + "/" + ano + "," + hora + ":" + minuto + ":" + segundo;
  //Serial.println(TimeLocal);
}

void GPSdata()
{
  bool newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial2.available())
    {
      char c = Serial2.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }
  if (newData)
  {
    //float flat, flon, alt;
    int year;
    byte month, day, hour, minute, second, hundredths, sat, prec;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
    sat = gps.satellites();
    prec = gps.hdop();
    alt = gps.f_altitude();
    if (hour > 3)hour = (hour - 4);
    else {
      hour = (hour + 20);
      day = (day - 1);
    }
    char sz[60];
    sprintf(sz, "%f,%f,%d,%d,%0.2f,%02d/%02d/%02d,%02d:%02d:%02d",
            flat, flon, sat, prec, alt, day, month, year, hour, minute, second);
    SinalGPS = sz;
    SinalGPS += "\n";
    appendFile(SD, "/gpsRX.txt", SinalGPS.c_str());
    char sz1[21];
    sprintf(sz1, "%f,%f,%0.2f",
            flat, flon, alt);
    SinalGPS = sz1;
  }
}

double calcDist(float ilat, float ilog, float flat, float flon) {
  double dlong = (flon - ilog) ;
  //Serial.println(dlong, 6);
  double dlat = (flat - ilat) ;
  //Serial.println(dlat, 6);
  double c = sqrt(dlong * dlong  + dlat * dlat );
  //Serial.println (c, 7);
  return regress (c);
}

double regress(double x) {
  double terms[] = {
    -3.1689916842841064e+000,
    1.0901575726347862e+005,
    -4.6643053664161905e+005,
    5.7156360892116681e+007,
    -3.9168386961030402e+009,
    1.5607395647971548e+011,
    -3.4824757033661943e+012,
    3.2706372378635406e+013,
    2.9106720619946300e+014,
    -1.1856451505307860e+016,
    1.4006946266044288e+017,
    -7.8102632313790259e+017,
    1.7393110833417582e+018
  };

  double t = 1;
  double r = 0;
  for (double c : terms) {
    r += c * t;
    t *= x;
  }
  return r;
}
