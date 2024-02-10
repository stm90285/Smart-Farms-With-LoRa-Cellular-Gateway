
#include <SPI.h>
#include <Notecard.h>

#include <HardwareSerial.h>
HardwareSerial MySerial0(0);


#define productUID "com.gmail.stm90285:lora_farming"
Notecard notecard;

static char recv_buf[512];
static bool is_exist = false;

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
        char tem[10],hum[10],moi[10],CO2[10];
        float moisture;
        
        p_start = strstr(recv_buf, "+TEST: RX \"05345454544");
        if (p_start)
        {
            p_start = strstr(recv_buf, "05345454544");
            if (p_start && (1 == sscanf(p_start, "05345454544%s", data)))
            {   
                Serial.println(p_start);
                sscanf(p_start,"05345454544%5s%5s%5s%5s\"",&tem,&hum,&moi,&CO2);
                findAndReplace(tem, 'A', '.');
                findAndReplace(hum, 'A', '.');
                findAndReplace(moi, 'A', '.');
                findAndReplace(CO2, 'A', '.');
                moisture = atof(moi);
                Serial.print("Temp: ");
                Serial.print(tem);
                Serial.print("\r\n");
                Serial.print("Humidity: ");
                Serial.print(hum);
                Serial.print("\r\n");
                Serial.print("Moisture: ");
                Serial.print(moi);
                Serial.print("\r\n");
                Serial.print("CO2: ");
                Serial.print(CO2);
                PostHttpRequest(tem, hum, String(moisture),CO2);
            }

            p_start = strstr(recv_buf, "RSSI:");
            if (p_start && (1 == sscanf(p_start, "RSSI:%d,", &rssi)))
            {
                Serial.print("rssi:");
                Serial.print(rssi);
                Serial.print("\r\n");
            }
            p_start = strstr(recv_buf, "SNR:");
            if (p_start && (1 == sscanf(p_start, "SNR:%d", &snr)))
            {
                Serial.print("snr :");
                Serial.print(snr);
                Serial.print("\r\n");
            }
            return 1;
        }
    }
    return 0;
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

void findAndReplace(char *str, char findChar, char replaceChar) {
    for (int i = 0; str[i] != '\0'; i++) {
        if (str[i] == findChar) {
            str[i] = replaceChar;
        }
    }
}

void PostHttpRequest(String temperature, String humidity, String moisture,String Carbondioxide)
{
  Serial.println("Start Sending Data");
  J *req = notecard.newRequest("note.add");
  if (req != NULL)
    {
        JAddBoolToObject(req, "sync", true);
        J *body = JAddObjectToObject(req, "body");
        if (body != NULL)
        {
            JAddNumberToObject(body, "Temperature", temperature.toInt());
            JAddNumberToObject(body, "Humidity", humidity.toInt());
            JAddNumberToObject(body, "Moisture", moisture.toInt());
            JAddNumberToObject(body, "CO2", Carbondioxide.toInt());
        }
        notecard.sendRequest(req);
    }
}

void setup(void)
{

    Serial.begin(115200);
    while (!Serial);
    Serial.println("Starting Node");
    MySerial0.begin(9600, SERIAL_8N1, -1, -1);
    
    Serial.print("Smart Farm With LoRa Cellular Gateway\r\n");
    
    notecard.setDebugOutputStream(Serial);
    notecard.begin();

    
    J *restore = notecard.newRequest("card.restore");
    JAddBoolToObject(restore, "delete", true);
    notecard.sendRequest(restore);
    delay(10000);
    
    J *req = notecard.newRequest("hub.set");
    JAddStringToObject(req, "product", productUID);
    JAddStringToObject(req, "mode", "continuous");
    JAddBoolToObject(req, "sync", true);
    JAddNumberToObject(req, "inbound", 1);
    JAddNumberToObject(req, "outbound", 1);
    notecard.sendRequestWithRetry(req, 5);
    delay(20000);
   

    while(!is_exist){
      if (at_send_check_response("+AT: OK", 100, "AT\r\n"))
      {
          is_exist = true;
          at_send_check_response("+MODE: TEST", 1000, "AT+MODE=TEST\r\n");
          at_send_check_response("+TEST: RFCFG", 1000, "AT+TEST=RFCFG,866,SF12,125,12,15,14,ON,OFF,OFF\r\n");
          delay(200);
      }
      else
      {
          is_exist = false;
          Serial.print("No E5 module found.\r\n");
      }
    }
}

void loop(void)
{
    node_recv(2000);
}
