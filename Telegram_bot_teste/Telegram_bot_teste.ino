
#include <UniversalTelegramBot.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#define BOT_TOKEN "seu_token_aqui"
#define SSID "seu_ssid_aqui"
#define PASSWORD "sua_senha_aqui"



WiFiClientSecure client;
UniversalTelegramBot bot(BOT_TOKEN, client);
uint32_t lastCheckTime = 0;

// Substitua "SEU_CHAT_ID_AQUI" pelo Chat ID obtido do BotFather
String chatId = "SEU_CHAT_ID_AQUI";

void setup() {
  Serial.begin(115200);
  setupWiFi();
}

void setupWiFi() {
  Serial.print("Connecting to SSID: ");
  Serial.println(SSID);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println("Connected");
}

void loop() {
  uint32_t now = millis();
  if (now - lastCheckTime > 1000) {
    lastCheckTime = now;
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    handleNewMessages(numNewMessages);
  }
}

void handleNewMessages(int numNewMessages) {
  for (int i = 0; i < numNewMessages; i++) {
    String chatId = String(bot.messages[i].chat_id);
    String senderId = String(bot.messages[i].from_id);

    boolean validSender = validateSender(senderId);

    if (!validSender) {
      bot.sendMessage(chatId, "Desculpe, você não tem permissão");
      continue;
    }

    String text = bot.messages[i].text;

    if (text == "/start") {
      handleStart(chatId);
    } else if (text == "/hello") {
      handleHello(chatId);
    } else if (text == "/test") {
      handleTest(chatId);
    } else if (text == "/help") {
      handleHelp(chatId);
    } else if (text == "/menu") {
      handleMenu(chatId);
    } else {
      handleNotFound(chatId);
    }
  }
}

boolean validateSender(String senderId) {
  // Pode adicionar IDs de remetentes válidos conforme necessário
  return true;
}

void handleStart(String chatId) {
  String message = "Bem-vindo! Este é um bot de teste.\n";
  message += "Use /hello, /test, ou /help para interagir.";
  bot.sendMessage(chatId, message);
}

void handleHello(String chatId) {
  bot.sendMessage(chatId, "Olá! A comunicação está ok.");
}

void handleTest(String chatId) {
  bot.sendMessage(chatId, "Teste bem-sucedido! A comunicação está ok.");
}

void handleHelp(String chatId) {
  String message = "Comandos disponíveis:\n";
  message += "/hello - Saudação amigável\n";
  message += "/test - Teste de comunicação\n";
  message += "/help - Mostra esta mensagem de ajuda\n";
  message += "/menu - Mostra o menu de comandos";
  bot.sendMessage(chatId, message);
}

void handleMenu(String chatId) {
  String message = "Menu de Comandos:\n";
  message += "/hello - Saudação amigável\n";
  message += "/test - Teste de comunicação\n";
  message += "/help - Mostra esta mensagem de ajuda\n";
  message += "/menu - Mostra o menu de comandos";
  bot.sendMessage(chatId, message);
}

void handleNotFound(String chatId) {
  bot.sendMessage(chatId, "Comando não encontrado");
}
