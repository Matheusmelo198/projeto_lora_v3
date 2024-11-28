#include <TinyGPS++.h>
#include <WiFi.h>

#define GPS_BAUDRATE 9600
#define EARTH_RADIUS 6371000.0  // Raio médio da Terra em metros

const char *ssid = "seu_ssid_aqui";      // Substitua pelo seu SSID
const char *password = "sua_senha_aqui";  // Substitua pela sua senha

TinyGPSPlus gps;

WiFiServer server(80);

void connectToWiFi();
void updateGPSData();
String generateHTML();
void sendHTML(WiFiClient client);
String formatDateTime(TinyGPSDate &date, TinyGPSTime &time);
double toRadians(double degree);
double haversineDistance(double lat1, double lon1, double lat2, double lon2);

void setup() {
  Serial.begin(115200);
  Serial2.begin(GPS_BAUDRATE);

  // Conectar-se à rede WiFi
  connectToWiFi();

  server.begin();

  Serial.println("ESP32 - GPS Module com Web Server");
}

void loop() {
  // Verificar se a conexão WiFi está ativa
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Conexão WiFi perdida. Tentando reconectar...");
    connectToWiFi();
  }

  updateGPSData();

  WiFiClient client = server.available();
  if (client) {
    Serial.println("Novo cliente conectado");
    
    // Enviar a página HTML para o cliente
    sendHTML(client);

    delay(100);
    client.stop();
    Serial.println("Cliente desconectado");
  }

  // Outras verificações e lógica do loop
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
  String html = "<html><head><meta charset='UTF-8'></head>";
  html += "<body>";
  html += "<h1>ESP32 - Dados do   GPS</h1>";

  if (gps.location.isValid()) {
    html += "<p>Latitude: " + String(gps.location.lat(), 8) + "</p>";
    html += "<p>Longitude: " + String(gps.location.lng(), 8) + "</p>";
    html += "<p>Altitude: " + String(gps.altitude.isValid() ? String(gps.altitude.meters()) : "INVALID") + " meters</p>";
    html += "<p>Speed: " + String(gps.speed.isValid() ? String(gps.speed.kmph()) : "INVALID") + " km/h</p>";
    html += "<p>GPS Date & Time: " + formatDateTime(gps.date, gps.time) + "</p>";

    // Coordenadas do ponto de destino
    double destLat = -20.511661;
    double destLon = -43.843188;

    // Calcular a distância
    double distance = haversineDistance(
      gps.location.lat(), gps.location.lng(),
      destLat, destLon
    );

    html += "<p>Distância até o destino: " + String(distance, 2) + " metros</p>";
    html+="<p>Satélites Visíveis: "+ String(gps.satellites.value()) + " </p>";
    html += "<p><a  href='https://www.google.com/maps/dir/?api=1&origin=" + String(gps.location.lat(), 8) + ","
            + String(gps.location.lng(), 8) + "&destination=" + String(destLat, 8) + "," + String(destLon, 8) + "' target='_blank'>Abrir no Google Maps a Rota</a></p>";
    html += "<p><a href='https://www.google.com/maps/place/" + String(gps.location.lat(), 8) + ","
            + String(gps.location.lng(), 8) + "' target='_blank'>Abrir no Google Maps a Localização do GPS</a></p>";      
  } else {
    html += "<p>Location: INVALID</p>";
  }

  html += "</body></html>";
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
  double distance = EARTH_RADIUS * c;

  return distance;
}
