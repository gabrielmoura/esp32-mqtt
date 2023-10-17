#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebSocketsServer.h>
//#include <FS.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include <WiFi.h>
//#include <WiFiClient.h>
#include <FFat.h>
#include <DHT.h>
#include <PubSubClient.h>
// Responsavel pelo paralelismo
//#include <esp_ipc.h>
#include <freertos/task.h>

/* Relay da Lampada */
#define RL_LAMP 22
bool RL_LAMPS = false;

/* Digital Sensor de Ruido */
#define DS_PALMA 35

/* Definicoes do sensor DHT22 */
#define DHTPIN 26 //GPIO que está ligado o pino de dados do sensor

#define DHTTYPE DHT11


/* Defines do MQTT */
#define TOPICO_SUBSCRIBE_LED "/device/t000111/led"
#define TOPICO_PUBLISH_TEMPERATURA "/device/t000111/temp"
#define TOPICO_PUBLISH_DISTANCIA "/device/t000111/dist"
#define TOPICO_PUBLISH_UMIDADE "/device/t000111/hum"
#define TOPICO_PUBLISH_PALMA "/device/t000111/palma"

#define ID_MQTT "esp32_mqttgggg"

/* Variaveis, constantes e objetos globais */
DHT dht(DHTPIN, DHTTYPE);
/**
 * Definição de Broker
 * broker.emqx.io
 * broker.hivemq.com
 */
const char *BROKER_MQTT = "jackal.rmq.cloudamqp.com"; //URL do broker MQTT que se deseja utilizar
int BROKER_PORT = 1883;                               // Porta do Broker MQTT
const char *mqttUser = "change_me";
const char *mqttPassword = "token";

#define USE_SERIAL Serial

/**
 * Prototypes
 */
float faz_leitura_temperatura(void);
float faz_leitura_umidade(void);
//void initWiFi(void);
void initMQTT(void);
void mqtt_callback(char *topic, byte *payload, unsigned int length);
void reconnectMQTT(void);
//void reconnectWiFi(void);
void VerificaConexoesWiFIEMQTT(void);
void MqttLoop(void);
void setupPin(void);
void externLoop(void *arg); //

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
void handleNotFound();
void handleRoot();
void configureDevice();
void startConfigWebpage();

const char *wifiSsid = "Moura";
const char *wifiPassword = "Strong@Password";
const char *apSsid = "ESP32 IoT";
const char *apPassword = "";

uint8_t configButton = 23;
uint8_t builtinLed = 5;
uint8_t led_conf = LED_BUILTIN;

String ssid;
String password;
bool isStaticIp;
String staticIp;
String netmask;
String gateway;
bool blankDevice;
bool keepConfigWegpage;
//uint8_t restartConfigWebpage = 0;
uint8_t wifiStatus;

unsigned long connectTimeout;

WiFiClient client;
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(9090);
PubSubClient MQTT(client);

void setupPin(void)
{
    pinMode(configButton, INPUT); // Jumper para segurar configurações
    pinMode(builtinLed, OUTPUT);
    pinMode(led_conf, OUTPUT); // Led que indicará se o Painel de configuração esta acessivel
    pinMode(DS_PALMA, INPUT);  //Sensor de RUIDO(PALMAS)
    pinMode(RL_LAMP,OUTPUT);
}

void setup()
{
    Serial.begin(115200);
    setupPin();
    dht.begin();

    if (!FFat.begin(true))
    {
        Serial.println("Couldn't mount the filesystem.");
    }

    //FFat.remove("/deviceConfig.txt");

    configureDevice();

    // ssid = "";
    // password = "";
    // isStaticIp = false;
    // staticIp = "";
    // netmask = "";
    // gateway = "";
    //blankDevice = true;

    if (digitalRead(configButton) == LOW || blankDevice == true)
    {

        WiFi.disconnect(true);
        WiFi.mode(WIFI_AP_STA);

        webSocket.begin();
        webSocket.onEvent(webSocketEvent);

        server.on("/", handleRoot);
        server.onNotFound(handleNotFound);
        server.begin();
        startConfigWebpage();
    }

    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    Serial.print("Connecting to ");
    Serial.println(ssid);

    if (isStaticIp == true)
    {
        IPAddress _ip;
        IPAddress _gateway;
        IPAddress _netmask;
        _ip.fromString(staticIp);
        _gateway.fromString(gateway);
        _netmask.fromString(netmask);
        WiFi.config(_ip, _gateway, _netmask);
    }

    //if(password == "") {
    //  WiFi.begin(ssid.c_str());
    //} else {
    WiFi.begin(ssid.c_str(), password.c_str());
    //}

    connectTimeout = millis() + (15 * 1000);
    while (true)
    {
        wifiStatus = WiFi.status();

        if ((wifiStatus == WL_CONNECTED) || (wifiStatus == WL_NO_SSID_AVAIL) ||
            (wifiStatus == WL_CONNECT_FAILED) || (millis() >= connectTimeout))
            break;

        delay(100);
    }
    //if(wifiStatus != WL_CONNECTED)
    // {
    //  startConfigWebpage();
    //}

    // while (WiFi.status() != WL_CONNECTED)
    // {
    //   restartConfigWebpage++;
    //   delay(1000);
    //   if(restartConfigWebpage >= 10){
    //     startConfigWebpage();
    //   }
    // }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    delay(100);

    // server.on("/", handleRoot);
    // server.onNotFound(handleNotFound);
    // server.begin();
    initMQTT();

    //esp_ipc_call(PRO_CPU_NUM, externLoop, NULL);
//  xTaskCreatePinnedToCore(externLoop,
//                          "Loop para recepçao externa",
//                          2048,
//                          NULL,
//                          2,
//                          NULL,
//                          PRO_CPU_NUM);
}

void loop()
{
    MqttLoop();
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
        case WStype_DISCONNECTED:
            USE_SERIAL.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED:
        {
            IPAddress ip = webSocket.remoteIP(num);
            USE_SERIAL.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
            size_t len;

            DynamicJsonDocument deviceInfo(JSON_ARRAY_SIZE(500));
            deviceInfo["deviceInfo"]["macAddress"] = WiFi.macAddress();
            deviceInfo["deviceInfo"]["ssid"] = ssid;
            deviceInfo["deviceInfo"]["password"] = password;
            deviceInfo["deviceInfo"]["isStaticIp"] = isStaticIp;
            deviceInfo["deviceInfo"]["staticIp"] = staticIp;
            deviceInfo["deviceInfo"]["netmask"] = netmask;
            deviceInfo["deviceInfo"]["gateway"] = gateway;

            len = measureJson(deviceInfo);
            char jsonToSend[len];
            serializeJson(deviceInfo, Serial);
            serializeJson(deviceInfo, jsonToSend, len + 1);
            webSocket.sendTXT(num, jsonToSend, strlen(jsonToSend));
            //webSocket.sendTXT(num, "{Connected}");
        }
            break;
        case WStype_TEXT:
        {
            USE_SERIAL.printf("[%u] get Text: %s\n", num, payload);

            StaticJsonDocument<512> jsonBuffer;
            deserializeJson(jsonBuffer, payload);
            JsonObject jsonObject = jsonBuffer.as<JsonObject>();

            if (strcmp((const char *)payload, "scanNetworks") == 0)
            {
                uint8_t n;
                size_t len;
                DynamicJsonDocument networksList(JSON_ARRAY_SIZE(50));
                Serial.println("scan start");
                n = WiFi.scanNetworks();
                if (n == 0)
                {
                    Serial.println("no networks found");
                }
                else
                {
                    Serial.print(n);
                    Serial.println(" networks found");
                    for (uint8_t i = 0; i < n; ++i)
                    {
                        // Pruint8_t SSID and RSSI for each network found
                        Serial.print(i + 1);
                        Serial.print(": ");
                        Serial.print(WiFi.SSID(i));
                        Serial.print(" (");
                        Serial.print(WiFi.RSSI(i));
                        Serial.println(")");
                        //networks[i] = WiFi.SSID(i);
                        networksList["ssidArray"].add(WiFi.SSID(i));
                        delay(10);
                    }
                }
                len = measureJson(networksList);
                char networksArray[len];
                serializeJson(networksList, Serial);
                //Serial.println("");
                //Serial.println(len);
                serializeJson(networksList, networksArray, len + 1);
                webSocket.sendTXT(num, networksArray, strlen(networksArray));
            }

            if (strcmp((const char *)payload, "ping") == 0)
            {
                webSocket.sendTXT(num, "{\"pong\":true}");
            }

            if (strcmp((const char *)payload, "deviceConfiguration") == 0)
            {
                //webSocket.sendTXT(num, "{\"pong\":true}");
                JsonVariant _ssid = jsonObject["deviceConfiguration"]["ssid"];
                JsonVariant _password = jsonObject["deviceConfiguration"]["password"];
                JsonVariant _isStaticIp = jsonObject["deviceConfiguration"]["isStaticIp"];
                JsonVariant _staticIp = jsonObject["deviceConfiguration"]["staticIp"];
                JsonVariant _netmask = jsonObject["deviceConfiguration"]["netmask"];
                JsonVariant _gateway = jsonObject["deviceConfiguration"]["gateway"];
                ssid = _ssid.as<String>();
                password = _password.as<String>();
                isStaticIp = _isStaticIp.as<bool>();
                staticIp = _staticIp.as<String>();
                netmask = _netmask.as<String>();
                gateway = _gateway.as<String>();

                int len = measureJson(jsonObject);
                char buff[len];
                serializeJson(jsonObject, buff, len + 1);
                File file = FFat.open("/deviceConfig.txt", "w");
                if (!file)
                {
                    Serial.println("Wifi file write error");
                    return;
                }
                if (file.print(buff))
                {
                    Serial.println("Wifi file was written");
                    Serial.println(buff);
                }
                else
                {
                    Serial.println("File write failed");
                }
                file.close();
                blankDevice = false;
                webSocket.sendTXT(num, "{\"saveOk\":true}");
            }

            if (strcmp((const char *)payload, "eraseConfig") == 0)
            {
                ssid = "";
                password = "";
                isStaticIp = false;
                staticIp = "";
                netmask = "";
                gateway = "";
                FFat.remove("/deviceConfig.txt");
                blankDevice = true;
                webSocket.sendTXT(num, "{\"eraseOk\":true}");
            }

            if (strcmp((const char *)payload, "startDevice") == 0)
            {
                //configureDevice();
                webSocket.sendTXT(num, "{\"startDeviceOk\":true}");
                //blankDevice = false;
                keepConfigWegpage = false;
                ESP.restart(); //Reiniciar ESP ao mandar Iniciar
            }

            if (strcmp((const char *)payload, "testWifi") == 0)
            {
                if (blankDevice == true)
                {
                    Serial.print("Cant test blank device\n");
                    webSocket.sendTXT(num, "{\"testWifiStatus\":\"blankdevice\"}");
                    return;
                }
                Serial.print("Testing connection to: ");
                //WiFi.mode(WIFI_AP_STA);
                Serial.println(ssid);
                Serial.print("Password ");
                Serial.println(password);
                WiFi.disconnect(true);
                WiFi.begin(ssid.c_str(), password.c_str());
                connectTimeout = millis() + (20 * 1000);
                while (true)
                {
                    wifiStatus = WiFi.status();
                    Serial.printf("(%d).", wifiStatus);
                    if ((wifiStatus == WL_CONNECTED) || (wifiStatus == WL_NO_SSID_AVAIL) ||
                        (wifiStatus == WL_CONNECT_FAILED) || (millis() >= connectTimeout))
                        break;
                    delay(100);
                }
                switch (wifiStatus)
                {
                    case WL_NO_SSID_AVAIL:
                        Serial.print("WL_NO_SSID_AVAIL\n");
                        webSocket.sendTXT(num, "{\"testWifiStatus\":\"WL_NO_SSID_AVAIL\"}");
                        break;
                    case WL_CONNECT_FAILED:
                        Serial.print("WL_CONNECT_FAILED\n");
                        webSocket.sendTXT(num, "{\"testWifiStatus\":\"WL_CONNECT_FAILED\"}");
                        break;
                    case WL_CONNECTED:
                        Serial.print("WL_CONNECTED\n");
                        webSocket.sendTXT(num, "{\"testWifiStatus\":\"WL_CONNECTED\"}");
                        break;
                    default:
                        Serial.print("default\n");
                        webSocket.sendTXT(num, "{\"testWifiStatus\":\"error\"}");
                        break;
                }
                WiFi.disconnect();
            }
        }
            break;
        case WStype_BIN:
        case WStype_PING:
        case WStype_PONG:
        case WStype_ERROR:
        case WStype_FRAGMENT_TEXT_START:
        case WStype_FRAGMENT_BIN_START:
        case WStype_FRAGMENT:
        case WStype_FRAGMENT_FIN:
            break;
    }
}

void handleRoot()
{

    Serial.println("HandleRoot");
    SPIFFS.begin();
    File file = SPIFFS.open("/index.html.gz", FILE_READ);

    server.streamFile(file, "text/html");

    //server.send(200,"text/html",file);
    file.close();
    SPIFFS.end();
}

void handleNotFound()
{
    server.send(404, "text/plain", "404: Not found");
}

void configureDevice()
{
    Serial.println("Configuring device...");
    File file = FFat.open("/deviceConfig.txt", "r");
    int len = file.size();
    char buff[len];
    if (!file)
    {
        Serial.println("Failed to open config file, blankDevice true");
        ssid = "";
        password = "";
        isStaticIp = false;
        staticIp = "";
        netmask = "";
        gateway = "";
        blankDevice = true;
        return;
    }
    while (file.available())
    {
        file.readBytes(buff, len);
    }
    file.close();
    StaticJsonDocument<512> jsonBuffer;
    deserializeJson(jsonBuffer, buff);
    JsonObject jsonObject = jsonBuffer.as<JsonObject>();
    JsonVariant _ssid = jsonObject["deviceConfiguration"]["ssid"];
    JsonVariant _password = jsonObject["deviceConfiguration"]["password"];
    JsonVariant _isStaticIp = jsonObject["deviceConfiguration"]["isStaticIp"];
    JsonVariant _staticIp = jsonObject["deviceConfiguration"]["staticIp"];
    JsonVariant _netmask = jsonObject["deviceConfiguration"]["netmask"];
    JsonVariant _gateway = jsonObject["deviceConfiguration"]["gateway"];
    ssid = _ssid.as<String>();
    password = _password.as<String>();
    isStaticIp = _isStaticIp.as<bool>();
    staticIp = _staticIp.as<String>();
    netmask = _netmask.as<String>();
    gateway = _gateway.as<String>();

    //if(ssid == "" || (isStaticIp == true && (staticIp == "" || netmask == "" || gateway == "")))

    blankDevice = false;

    Serial.println("------ WIFI ------");
    Serial.println(ssid);
    Serial.println(password);
    Serial.println(isStaticIp);
    Serial.println(staticIp);
    Serial.println(netmask);
    Serial.println(gateway);
    Serial.println("------------------");

    Serial.println("Configure OK");
}

void startConfigWebpage()
{
    digitalWrite(led_conf, HIGH); // Ligando LED para sinalização de configuração
    Serial.println("Starting AP for configuration");
    keepConfigWegpage = true;
    WiFi.mode(WIFI_AP_STA);
    //WiFi.disconnect();
    WiFi.softAP(apSsid, apPassword);

    Serial.print("IP: ");
    Serial.println(WiFi.softAPIP());

    Serial.println("Starting Websocket loop");
    while (keepConfigWegpage) //mantem o loop da pagina de configuração até que um ssid e uma senha senha definida
    {
        webSocket.loop();
        server.handleClient();
    }
}

/*
 * Implementações
 */

/* Função: faz a leitura de temperatura (sensor DHT22)
 * Parametros: nenhum
 * Retorno: temperatura (graus Celsius)
 */
float faz_leitura_temperatura(void)
{
    float t = dht.readTemperature();
    float result;

    if (!(isnan(t)))
        result = t;
    else
        result = -99.99;

    return result;
}

/* Função: faz a leitura de umidade relativa do ar (sensor DHT22)
 * Parametros: nenhum
 * Retorno: umidade (0 - 100%)
 */
float faz_leitura_umidade(void)
{
    float h = dht.readHumidity();
    float result;

    if (!(isnan(h)))
        result = h;
    else
        result = -99.99;

    return result;
}
/* Função: inicializa parâmetros de conexão MQTT(endereço do
 *         broker, porta e seta função de callback)
 * Parâmetros: nenhum
 * Retorno: nenhum
 */
void initMQTT(void)
{
    MQTT.setServer(BROKER_MQTT, BROKER_PORT); //informa qual broker e porta deve ser conectado
    MQTT.setCallback(mqtt_callback);          //atribui função de callback (função chamada quando qualquer informação de um dos tópicos subescritos chega)
}
/* Função: função de callback
 *         esta função é chamada toda vez que uma informação de
 *         um dos tópicos subescritos chega)
 * Parâmetros: nenhum
 * Retorno: nenhum
 */
void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
    String msg;

    /* obtem a string do payload recebido */
    for (int i = 0; i < length; i++)
    {
        char c = (char)payload[i];
        msg += c;
    }

    Serial.println("Chegou a seguinte string via MQTT: ");
    Serial.println(msg);

    /* toma ação dependendo da string recebida */
    if (msg.equals("on"))
    {
        digitalWrite(RL_LAMP, HIGH);
        Serial.println("Lampada aceso mediante comando MQTT");
        RL_LAMPS = true;
    }

    if (msg.equals("off"))
    {
        digitalWrite(RL_LAMP, LOW);
        Serial.println("Lampada apagado mediante comando MQTT");
        RL_LAMPS = false;
    }
}

/* Função: reconecta-se ao broker MQTT (caso ainda não esteja conectado ou em caso de a conexão cair)
 *         em caso de sucesso na conexão ou reconexão, o subscribe dos tópicos é refeito.
 * Parâmetros: nenhum
 * Retorno: nenhum
 */
void reconnectMQTT(void)
{
    while (!MQTT.connected())
    {
        Serial.print("* Tentando se conectar ao Broker MQTT: ");
        Serial.println(BROKER_MQTT);
        if (MQTT.connect(ID_MQTT, mqttUser, mqttPassword))
        {
            Serial.println("Conectado com sucesso ao broker MQTT!");
            MQTT.subscribe(TOPICO_SUBSCRIBE_LED);
        }
        else
        {
            Serial.println("Falha ao reconectar no broker.");
            Serial.println("Havera nova tentatica de conexao em 2s");
            delay(2000);
            //VerificaConexoesWiFIEMQTT();
        }
    }
}

/* Função: verifica o estado das conexões WiFI e ao broker MQTT.
 *         Em caso de desconexão (qualquer uma das duas), a conexão
 *         é refeita.
 * Parâmetros: nenhum
 * Retorno: nenhum
 */
void VerificaConexoesWiFIEMQTT(void)
{
    if (!MQTT.connected())
    {
        reconnectMQTT(); //se não há conexão com o Broker, a conexão é refeita
    }
    if (!WiFi.isConnected())
    {
        delay(2000);
        WiFi.disconnect(true);
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid.c_str(), password.c_str());
        //se não há conexão com o WiFI, a conexão é refeita}
    }
}

void MqttLoop(void)
{
    // Serial.print(xTaskGetTickCount()); // Imprime os Tickets
    // Serial.println(xPortGetCoreID());  // Imprime Core usado pelo loop

    //Serial.println(xPortGetCoreID()); //Imprime Core usado pelo loop
    char temperatura_str[10] = {0};
    char umidade_str[10] = {0};

    /* garante funcionamento das conexões WiFi e ao broker MQTT */
    VerificaConexoesWiFIEMQTT();

    /* Compoe as strings a serem enviadas pro dashboard (campos texto) */
    sprintf(temperatura_str, "%.2fC", faz_leitura_temperatura());
    sprintf(umidade_str, "%.2f", faz_leitura_umidade());

    /*  Envia as strings ao dashboard MQTT */
    MQTT.publish(TOPICO_PUBLISH_TEMPERATURA, temperatura_str);
    MQTT.publish(TOPICO_PUBLISH_UMIDADE, umidade_str);

    /* keep-alive da comunicação com broker MQTT */
    MQTT.loop();

    /* Refaz o ciclo após 2 segundos */
    delay(2000);
}

void externLoop(void *arg)
{
    while (1)
    {
        if (!!digitalRead(DS_PALMA)) //Detectando Palma/Ruindo
        {

            // Serial.print(xTaskGetTickCount()); // Imprime os Tickets
            // Serial.println(xPortGetCoreID());  // Imprime Core usado pelo loop
            MQTT.publish(TOPICO_PUBLISH_PALMA, "true");
            if (!RL_LAMPS) // Se desligado, ligar
            {
                digitalWrite(RL_LAMP, HIGH);
                RL_LAMPS = true;
                MQTT.publish(TOPICO_SUBSCRIBE_LED, "on");
                Serial.println("Lampada acessa mediante palma");
            }
            else
            {
                digitalWrite(RL_LAMP, LOW);
                RL_LAMPS = false;
                MQTT.publish(TOPICO_SUBSCRIBE_LED, "off");
                Serial.println("Lampada apagada mediante palma");
            }
        }
        vTaskDelay(100); //Conta 100 mili segundo
    }
}