#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <SensirionI2CSht4x.h>
#include <HardwareSerial.h>
#include <Wire.h>


SensirionI2CSht4x sht4x;

static char recv_buf[512];
static bool is_exist = false;

HardwareSerial MySerial0(0);


#include <Arduino.h>
#include <EEPROM.h>

#include "sensirion_common.h"
#include "sgp30.h"

#define LOOP_TIME_INTERVAL_MS  1000
#define BASELINE_IS_STORED_FLAG  (0X55)
//#define ARRAY_TO_U32(a)  (a[0]<<24|a[1]<<16|a[2]<<8|a[3])    //MSB first  //Not suitable for 8-bit platform

void array_to_u32(u32* value, u8* array) {
    (*value) = (*value) | (u32)array[0] << 24;
    (*value) = (*value) | (u32)array[1] << 16;
    (*value) = (*value) | (u32)array[2] << 8;
    (*value) = (*value) | (u32)array[3];
}
void u32_to_array(u32 value, u8* array) {
    if (!array) {
        return;
    }
    array[0] = value >> 24;
    array[1] = value >> 16;
    array[2] = value >> 8;
    array[3] = value;
}

/*
    Reset baseline per hour,store it in EEPROM;
*/
void  store_baseline(void) {
    static u32 i = 0;
    u32 j = 0;
    u32 iaq_baseline = 0;
    u8 value_array[4] = {0};
    i++;
    Serial.println(i);
    if (i == 3600) {
        i = 0;
        if (sgp_get_iaq_baseline(&iaq_baseline) != STATUS_OK) {
            Serial.println("get baseline failed!");
        } else {
            Serial.println(iaq_baseline, HEX);
            Serial.println("get baseline");
            u32_to_array(iaq_baseline, value_array);
            for (j = 0; j < 4; j++) {
                EEPROM.write(j, value_array[j]);
                Serial.print(value_array[j]);
                Serial.println("...");
            }
            EEPROM.write(j, BASELINE_IS_STORED_FLAG);
        }
    }
    delay(LOOP_TIME_INTERVAL_MS);
}

/*  Read baseline from EEPROM and set it.If there is no value in EEPROM,retrun .
    Another situation: When the baseline record in EEPROM is older than seven days,Discard it and return!!

*/
void set_baseline(void) {
    u32 i = 0;
    u8 baseline[5] = {0};
    u32 baseline_value = 0;
    for (i = 0; i < 5; i++) {
        baseline[i] = EEPROM.read(i);
        Serial.print(baseline[i], HEX);
        Serial.print("..");
    }
    Serial.println("!!!");
    if (baseline[4] != BASELINE_IS_STORED_FLAG) {
        Serial.println("There is no baseline value in EEPROM");
        return;
    }
    /*
        if(baseline record in EEPROM is older than seven days)
        {
        return;
        }
    */
    array_to_u32(&baseline_value, baseline);
    sgp_set_iaq_baseline(baseline_value);
    Serial.println(baseline_value, HEX);
}




static int at_send_check_response(char *p_ack, int timeout_ms, char*p_cmd, ...)
{
  int ch = 0;
  int index = 0;
  int startMillis = 0;
  va_list args;
  memset(recv_buf, 0, sizeof(recv_buf));
  va_start(args, p_cmd);
  MySerial0.printf(p_cmd, args);
  Serial.printf(p_cmd, args);
  va_end(args);
  delay(200);
  startMillis = millis();

  if (p_ack == NULL)
  {
    return 0;
  }

  do
  {
    while (MySerial0.available() > 0)
    {
      ch = MySerial0.read();
      recv_buf[index++] = ch;
      Serial.print((char)ch);
      delay(2);
    }

    if (strstr(recv_buf, p_ack) != NULL)
    {
      return 1;
    }

  } while (millis() - startMillis < timeout_ms);
  return 0;
}

static int recv_prase(void)
{
  char ch;
  int index = 0;
  memset(recv_buf, 0, sizeof(recv_buf));
  while (MySerial0.available() > 0)
  {
    ch = MySerial0.read();
    recv_buf[index++] = ch;
    Serial.print((char)ch);
    delay(2);
  }

  if (index)
  {
    char *p_start = NULL;
    char data[32] = {
      0,
    };
    int rssi = 0;
    int snr = 0;

    p_start = strstr(recv_buf, "+TEST: RX \"05345454544");
    if (p_start)
    {
      p_start = strstr(recv_buf, "05345454544");
      if (p_start && (1 == sscanf(p_start, "05345454544%s", data)))
      {
        data[1] = 0;
        Serial.print("Motor State:");
        Serial.print(data);
      }

      p_start = strstr(recv_buf, "RSSI:");
      if (p_start && (1 == sscanf(p_start, "RSSI:%d,", &rssi)))
      {
        Serial.print("rssi:");
        Serial.print(rssi);
      }
      p_start = strstr(recv_buf, "SNR:");
      if (p_start && (1 == sscanf(p_start, "SNR:%d", &snr)))
      {

        Serial.print("snr :");
        Serial.print(snr);
      }
      return 1;
    }
  }
  return 0;
}

void findAndReplace(char *str, char findChar, char replaceChar) {
    for (int i = 0; str[i] != '\0'; i++) {
        if (str[i] == findChar) {
            str[i] = replaceChar;
        }
    }
}

static int node_recv(uint32_t timeout_ms)
{
  at_send_check_response("+TEST: RXLRPKT", 1000, "AT+TEST=RXLRPKT\r\n");
  int startMillis = millis();
  do
  {
    if (recv_prase())
    {
      return 1;
    }
  } while (millis() - startMillis < timeout_ms);
  return 0;
}

static int node_send(void)
{
   //Lora
    static uint16_t count = 0;
    int ret = 0;
    char data[64];
    char cmd[256];
   
    // SHT-40
    uint16_t error;
    char errorMessage[256];
    delay(10);

    float temperature;
    float humidity;
    
    error = sht4x.measureHighPrecision(temperature, humidity);
    if (error) 
    {
        Serial.print("Error trying to execute measureHighPrecision(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else 
    {
        Serial.print("Temperature:");
        Serial.print(temperature);
        Serial.print("\t");
        Serial.print("Humidity:");
        Serial.println(humidity);
    }

    //SGP30
    s16 err = 0;
    u16 tvoc_ppb, co2_eq_ppm;
    float CO2;
    err = sgp_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm);
    if (err == STATUS_OK) 
    {
        Serial.print("tVOC  Concentration:");
        Serial.print(tvoc_ppb);
        Serial.println("ppb");

        Serial.print("CO2eq Concentration:");
        CO2 = co2_eq_ppm;
        Serial.print(CO2);
        Serial.println("ppm");
    } else 
    {
        Serial.println("error reading IAQ values\n");
    }
    store_baseline();
    
    
    //Moisture Sensor
    float moisture;
    int sensorPin = A0;
    int analogValue = analogRead(sensorPin);
    moisture = map(analogValue,0,600,0.0,100.0);
    Serial.print("Moisture = " );
    Serial.println(moisture);

    
    char humStr[10];
    char temStr[10];
    char moiStr[10];
    char CO2Str[10];
    
    snprintf(humStr, sizeof(humStr), "%05.2f", humidity);
    snprintf(temStr, sizeof(temStr), "%05.2f", temperature);
    snprintf(moiStr, sizeof(moiStr), "%05.2f", moisture);
    snprintf(CO2Str, sizeof(CO2Str), "%05.2f", CO2);
    

    findAndReplace(humStr, '.', 'A');
    findAndReplace(temStr, '.', 'A');
    findAndReplace(moiStr, '.', 'A');
    findAndReplace(CO2Str, '.', 'A');
    
    //Sending data to the Gateway
    memset(data, 0, sizeof(data));
    sprintf(data, "%s%s%s%s", temStr, humStr, moiStr,CO2Str);
    sprintf(cmd, "AT+TEST=TXLRPKT,\"5345454544%s\"\r\n", data);
    Serial.println(data);
    Serial.println(cmd);
    

    ret = at_send_check_response("TX DONE", 2000, cmd);
    if (ret == 1)
    {
  
      Serial.print("Sent successfully!\r\n");
    }
    else
    {
      Serial.print("Send failed!\r\n");
    }
    return ret;



    
}



void setup()
{   
 
    Serial.begin(115200);
    //Configure MySerial0 on pins TX=6 and RX=7 (-1, -1 means use the default)
    Serial.println("Starting Node");
    
    MySerial0.begin(9600, SERIAL_8N1, -1, -1);
    
    while (!Serial) 
    {
       delay(100);
    }

    Wire.begin();

    uint16_t error;
    char errorMessage[256];

    sht4x.begin(Wire);
    

    uint32_t serialNumber;
    error = sht4x.serialNumber(serialNumber);
    if (error) 
    {
        Serial.print("Error trying to execute serialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } 
    else 
    {
        Serial.print("Serial Number: ");
        Serial.println(serialNumber);
    }


   // Checking Lora Module
   Serial.print("LoRa P2P Communication!\r\n");

  if (at_send_check_response("+AT: OK", 100, "AT\r\n"))
  {
    is_exist = true;
    at_send_check_response("+MODE: TEST", 1000, "AT+MODE=TEST\r\n");
    at_send_check_response("+TEST: RFCFG F:866000000, SF12, BW125K, TXPR:12, RXPR:15, POW:14dBm, CRC:ON, IQ:OFF, NET:OFF", 1000, "AT+TEST=RFCFG,866,SF12,125,12,15,14,ON,OFF,OFF\r\n");
    delay(200);
  }
  else
  {
    is_exist = false;
    Serial.print("No E5 module found.\r\n");
  }

   // SGP30
    s16 err;
    u16 scaled_ethanol_signal, scaled_h2_signal;

    /*  Init module,Reset all baseline,The initialization takes up to around 15 seconds, during which
        all APIs measuring IAQ(Indoor air quality ) output will not change.Default value is 400(ppm) for co2,0(ppb) for tvoc*/
    while (sgp_probe() != STATUS_OK) {
        Serial.println("SGP failed");
        while (1);
    }
    /*Read H2 and Ethanol signal in the way of blocking*/
    err = sgp_measure_signals_blocking_read(&scaled_ethanol_signal,
                                            &scaled_h2_signal);
    if (err == STATUS_OK) {
        Serial.println("get ram signal!");
    } else {
        Serial.println("error reading signals");
    }
    // err = sgp_iaq_init();
    set_baseline();
    
}

static void node_send_then_recv(uint32_t timeout)
{   
    int ret = 0;
    ret = node_send();
    if (!ret)
    {
        Serial.print("\r\n");
        return;
    }
    
    int ret2 = 0;
    ret2 = node_recv(timeout);
    delay(10);
    if (!ret2)
    {
        Serial.print("\r\n");
        return;
    }
    
}

void loop()
{
  node_send();
  delay(20000);
}
