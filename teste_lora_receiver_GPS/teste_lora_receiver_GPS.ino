#include "LoRaWan_APP.h"  // Inclui a biblioteca LoRaWAN necessária para comunicação LoRa.
#include "Arduino.h"      // Inclui a biblioteca padrão do Arduino.
#include "string.h"       // Inclui a biblioteca para manipulação de strings.
#include "HT_SSD1306Wire.h" // Inclui a biblioteca para controlar o display OLED.
#include "images.h"       // Inclui uma biblioteca para gerenciar imagens no display.
#include "HT_TinyGPS++.h"
#include "WiFi.h"

double destLat = 0.0;
double destLon = 0.0;
double destCont = 0.0;
double distance = 0.0;
String local = "";
extern SSD1306Wire display; // Declara uma instância externa do display OLED.

#define GPS_BAUDRATE 9600
#define EARTH_RADIUS 6371000.0  // Raio médio da Terra em metros
#define RF_FREQUENCY                                915000000 // Hz     Define a frequência de operação do LoRa em Hz.
#define LORA_BANDWIDTH                              0         // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]    Define a largura de banda do LoRa: 0 = 125 kHz.
#define LORA_SPREADING_FACTOR                       9         // 7[SF7..SF12]    Define o fator de espalhamento do LoRa
#define LORA_CODINGRATE                             1         // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]     Define a taxa de codificação do LoRa
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx     Define o comprimento do preâmbulo do LoRa.
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols     Define o tempo limite do símbolo do LoRa.
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false     // Define se o comprimento do payload é fixo ou não.
#define LORA_IQ_INVERSION_ON                        false     // Define se a inversão do IQ (quadratura inversa) está ativada.

#define RX_TIMEOUT_VALUE                            10000     // Define o valor do tempo limite de recebimento em milissegundos.
#define BUFFER_SIZE                                 50 // Define o tamanho do payload aqui.

char rxpacket[BUFFER_SIZE];  // Declara um array para armazenar o pacote recebido.

static RadioEvents_t RadioEvents;  // Declara uma estrutura de eventos de rádio.

int16_t rssi, rxSize;  // Declara variáveis para armazenar o valor do RSSI e o tamanho do pacote recebido.

bool lora_idle = true;  // Declara uma flag para indicar se o LoRa está inativo.

const char *ssid = "seu_ssid_aqui";      // Substitua pelo seu SSID
const char *password = "sua_senha_aqui";  // Substitua pela sua senha

TinyGPSPlus gps;

WiFiServer server(80);

void connectToWiFi();
void updateGPSData();
String generateHTML(const String &message, int16_t rssi);
void sendHTML(WiFiClient client);
String formatDateTime(TinyGPSDate &date, TinyGPSTime &time);
double toRadians(double degree);
double haversineDistance(double lat1, double lon1, double lat2, double lon2);



void setup() {
    Serial.begin(115200);  // Inicializa a comunicação serial na taxa de 115200 bps.
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);  // Inicializa a MCU (Unidade de Microcontrole).

    display.init();  // Inicializa o display OLED.
    display.flipScreenVertically();  // Inverte a orientação do display.
    display.setFont(ArialMT_Plain_10);  // Define a fonte do texto no display.
    
   
    display.clear();  // Limpa o display OLED.
    display.setTextAlignment(TEXT_ALIGN_LEFT);  // Define o alinhamento do texto à esquerda.
    display.drawString(0, 0, "Aguardando Dados");
    display.display();

    RadioEvents.RxDone = OnRxDone;  // Define o evento que será chamado quando a recepção for concluída.
    Radio.Init(&RadioEvents);  // Inicializa o rádio com os eventos definidos.
    Radio.SetChannel(RF_FREQUENCY);  // Define o canal de operação do rádio.

    // Configura o rádio para modo de recepção com os parâmetros definidos anteriormente.
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
 
    Serial2.begin(GPS_BAUDRATE, SERIAL_8N1, 46, 45);

    // Conectar-se à rede WiFi
    connectToWiFi();

    server.begin();

    Serial.println("ESP32 - GPS Module com Web Server");                    
}

void loop() {
    if (lora_idle) {  // Verifica se o LoRa está inativo.
        lora_idle = false;  // Define a flag como false para indicar que o LoRa está ativo.
        Serial.println("into RX mode");  // Envia uma mensagem pela serial indicando que entrou no modo de recepção.
        Radio.Rx(0);  // Coloca o rádio em modo de recepção.
    }
    Radio.IrqProcess();  // Processa as interrupções do rádio.
   
    if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Conexão WiFi perdida. Tentando reconectar...");
    connectToWiFi();
  }

  updateGPSData();
  distance = haversineDistance(gps.location.lat(), gps.location.lng(), destLat, destLon);

  WiFiClient client = server.available();
  if (client) {
    Serial.println("Novo cliente conectado");
    
    // Enviar a página HTML para o cliente
    sendHTML(client);

    delay(100);
    client.stop();
    Serial.println("Cliente desconectado");
  }
 
}



void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssiValue, int8_t snr) {
    rssi = rssiValue;  // Armazena o valor do RSSI globalmente.
    rxSize = size;  // Armazena o tamanho do pacote recebido.
    memcpy(rxpacket, payload, size);  // Copia o conteúdo do payload para o array rxpacket.
    rxpacket[size] = '\0';  // Adiciona o caractere nulo ao final do array para formar uma string.
    Radio.Sleep();  // Coloca o rádio em modo de suspensão.
    Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n", rxpacket, rssi, rxSize);  // Exibe na serial o pacote recebido, o valor do RSSI e o tamanho do pacote.
    lora_idle = true;  // Define a flag como true para indicar que o LoRa está inativo novamente.

    // Lida com os dados recebidos aqui.
    LoRaData();  // Chama a função para processar os dados recebidos.

    // Atualiza o display OLED com a mensagem recebida.
    

    String rxString = String(rxpacket);

       // Extrai a latitude
    int latEndIndex = rxString.indexOf(",");  // Encontra a posição da vírgula
    String latString = rxString.substring(0, latEndIndex); // A latitude está antes da vírgula
    destLat = latString.toDouble(); // Converte para double

    // Extrai a longitude
    int lonStartIndex = latEndIndex + 2;  // Pula a vírgula e o espaço
    int lonEndIndex = rxString.indexOf(",", lonStartIndex);  // Encontra a posição da segunda vírgula
    String lonString = rxString.substring(lonStartIndex, lonEndIndex); // A longitude está entre a primeira e segunda vírgula
    destLon = lonString.toDouble(); // Converte para double

    // Extrai o contador
    int contStartIndex = lonEndIndex + 2;  // Pula a segunda vírgula e o espaço
    String contString = rxString.substring(contStartIndex);
    
    destCont = contString.toDouble(); // Converte para double
    local = "https://www.google.com/maps/place/" + String(destLat, 8) + "," + String(destLon, 8);

    updateDisplay();
}



void LoRaData() {
    display.clear();  // Limpa o display OLED.
    display.setTextAlignment(TEXT_ALIGN_LEFT);  // Define o alinhamento do texto à esquerda.
    display.setFont(ArialMT_Plain_10);  // Define a fonte do texto.
    display.drawString(0, 15, "Received " + String(rxSize) + " bytes");  // Desenha uma string no display indicando o número de bytes recebidos.
    display.drawStringMaxWidth(0, 26, 128, rxpacket);  // Desenha a string recebida no display.
    display.display();  // Atualiza o display para mostrar as mudanças.
}

void updateDisplay() {

    display.clear();  // Limpa o display OLED.
    display.setTextAlignment(TEXT_ALIGN_LEFT);  // Define o alinhamento do texto à esquerda.
    display.setFont(ArialMT_Plain_10);  // Define a fonte do texto.
    display.drawString(0, 0, "RSSI: " + String(rssi) + " dBm");  // Desenha o valor do RSSI no display.
    display.drawStringMaxWidth(0, 15, 128, "Distância: "+ String(distance) + " m");
    display.drawStringMaxWidth(0, 30, 128, "Contador: "+ String(destCont));
 String ip = String(WiFi.localIP()[0]) + "." + 
                String(WiFi.localIP()[1]) + "." + 
                String(WiFi.localIP()[2]) + "." + 
                String(WiFi.localIP()[3]);
                
    display.drawString(0, 45, "IP: " + ip);
    display.display();  // Atualiza o display para mostrar as mudanças.
}


void connectToWiFi() {
  Serial.println("Conectando ao WiFi...");
  WiFi.begin(ssid, password);
  int attempts = 0;

  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConectado ao WiFi!");
    Serial.print("Endereço IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFalha na conexão ao WiFi. Verifique as credenciais.");
  }
}

void updateGPSData() {
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      // Os dados do GPS são atualizados
    }
  }
}

String generateHTML() { 
  String html = "<html><head><meta charset='UTF-8'>";
  
  // Estilos CSS
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; background-color: #f0f0f0; margin: 0; padding: 20px; }";
  html += "h1 { color: #333; }";
  html += "p { color: #555; font-size: 18px; }";
  html += "a { color: #007BFF; text-decoration: none; }";
  html += "a:hover { text-decoration: underline; }";
  html += "div.container { background-color: #fff; min-height: 300px; height: auto; padding: 20px; border-radius: 10px; box-shadow: 0 0 10px rgba(0,0,0,0.1); }";
  html += "</style>";
  
  html += "</head><body>";
  html += "<div class='container'>";
  html += "<h1>ESP32 - Dados do GPS</h1>";

  html += "<p><strong>RSSI:</strong> " + String(rssi) + " dBm</p>";
  html += "<p><strong>Contador:</strong> " + String(destCont) + "</p>";
  html += "<p><strong>Latitude emissor (Paciente):</strong> " + String(destLat, 8) + "</p>";
  html += "<p><strong>Longitude emissor (Paciente):</strong> " + String(destLon, 8) + "</p>";
  
  html += "<p><strong>Latitude receptor (Cuidador):</strong> " + String(gps.location.lat(), 8) + "</p>";
  html += "<p><strong>Longitude receptor (Cuidador):</strong> " + String(gps.location.lng(), 8) + "</p>";
  html += "<p><strong>GPS Data & Hora:</strong> " + formatDateTime(gps.date, gps.time) + "</p>";

  // Calcular a distância
  distance = haversineDistance(gps.location.lat(), gps.location.lng(), destLat, destLon);
  html += "<p><strong>Distância até o emissor:</strong> " + String(distance, 2) + " metros</p>";
  html += "<p><strong>Satélites Visíveis:</strong> " + String(gps.satellites.value()) + "</p>";

  // Links para o Google Maps
  html += "<p><a href='https://www.google.com/maps/dir/?api=1&origin=" 
          + String(gps.location.lat(), 8) + "," + String(gps.location.lng(), 8) + "&destination=" 
          + String(destLat, 8) + "," + String(destLon, 8) + "' target='_blank'>Abrir no Google Maps a Rota</a></p>";
  html += "<p><a href='https://www.google.com/maps/place/" 
          + String(gps.location.lat(), 8) + "," + String(gps.location.lng(), 8) 
          + "' target='_blank'>Abrir no Google Maps a Localização do receptor (Cuidador)</a></p>";
  html += "<p><a href='https://www.google.com/maps/place/" + String(destLat, 8) + "," 
          + String(destLon, 8) + "' target='_blank'>Abrir no Google Maps a Localização do emissor (Paciente)</a></p>"; 
  updateDisplay();
  html += "</div></body></html>";
  return html;
}






void sendHTML(WiFiClient client) {
  String html = generateHTML();

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();
  client.println(html);
}

String formatDateTime(TinyGPSDate &date, TinyGPSTime &time) {
  int hour = time.hour() - 3; //GMT horário de Brasilia
  int minute = time.minute();
  int second = time.second();

  if (hour < 0) {
    hour += 24;        //tratar problema GMT com horário negativo(de meia noite a 3 da manhã)
  }

  return String(date.year()) + "-"
       + String(date.month()) + "-"
       + String(date.day()) + " "
       + String(hour) + ":"
       + String(minute) + ":"
       + String(second);
}

double toRadians(double degree) {
  return degree * M_PI / 180.0;
}

double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
  // Convertendo coordenadas de graus para radianos
  lat1 = toRadians(lat1);
  lon1 = toRadians(lon1);
  lat2 = toRadians(lat2);
  lon2 = toRadians(lon2);

  // Diferença nas latitudes e longitudes
  double dLat = lat2 - lat1;
  double dLon = lon2 - lon1;

  // Fórmula de Haversine
  double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
             cos(lat1) * cos(lat2) *
             sin(dLon / 2.0) * sin(dLon / 2.0);

  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

  // Distância em metros
  double distance1 = EARTH_RADIUS * c;

  return distance1;
}
