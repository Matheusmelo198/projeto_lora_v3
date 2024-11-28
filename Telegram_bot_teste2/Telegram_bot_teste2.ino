
#include <UniversalTelegramBot.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#define BOT_TOKEN "seu_token_aqui"
#define SSID "seu_ssid_aqui"
#define PASSWORD "sua_senha_aqui"


const unsigned long BOT_MTBS = 1000; // mean time between scan messages

unsigned long bot_lasttime = millis(); // last time messages' scan has been done
WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

void handleNewMessages(int numNewMessages);

void handleMenu(const String& chatId)
{
  String message = "Command Menu:\n";
  message += "/help - Get bot usage help\n";
  message += "/start - Message sent when you open a chat with a bot\n";
  message += "/status - Answer device current status\n";
  message += "/test1 - Test 1: Communication check\n";
  message += "/test2 - Test 2: Communication check\n";
  message += "/menu - Show the command menu";
  bot.sendMessage(chatId, message, "Markdown");
}

void handleNewMessages(int numNewMessages)
{
  Serial.print("handleNewMessages ");
  Serial.println(numNewMessages);

  String answer;

  for (int i = 0; i < numNewMessages; i++)
  {

    telegramMessage &msg = bot.messages[i];
    Serial.println("Received " + msg.text);
    int atIndex = msg.text.indexOf("@");
    String comando = msg.text.substring(0, atIndex);
    Serial.println("ola" + comando);

    if (comando == "/help")
      answer = "So you need _help_, uh? me too! use /start or /status";
    else if (comando == "/start")
      answer = "Olá *" + msg.from_name + "*";
    else if (comando == "/status")
      answer = "All is good here, thanks for asking!";
    else if (comando == "/test1")
      answer = "Test 1 successful! Communication is ok.";
    else if (comando == "/test2")
      answer = "Test 2 successful! Communication is ok.";
    else if (comando == "/menu")
      handleMenu(msg.chat_id);
    else
      answer = "Say what?";

    bot.sendMessage(String(msg.chat_id), answer, "Markdown");
  }
}

void bot_setup()
{
  const String commands = F("["
                            "{\"command\":\"help\",  \"description\":\"Get bot usage help\"},"
                            "{\"command\":\"start\", \"description\":\"Message sent when you open a chat with a bot\"},"
                            "{\"command\":\"status\",\"description\":\"Answer device current status\"},"
                            "{\"command\":\"test1\", \"description\":\"Test 1: Communication check\"},"
                            "{\"command\":\"test2\", \"description\":\"Test 2: Communication check\"},"
                            "{\"command\":\"menu\",  \"description\":\"Show the command menu\"}" // no comma on the last command
                            "]");
  bot.setMyCommands(commands);
}

void setup()
{
  Serial.begin(115200);
  Serial.println();

  // attempt to connect to Wifi network:
  Serial.print("Connecting to Wifi SSID ");
  Serial.print(SSID);
  WiFi.begin(SSID, PASSWORD);
  secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.print("\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());

  Serial.print("Retrieving time: ");
  configTime(0, 0, "pool.ntp.org"); // get UTC time via NTP
  time_t now = time(nullptr);
  while (now < 24 * 3600)
  {
    Serial.print(".");
    delay(100);
    now = time(nullptr);
  }
  Serial.println(now);

  bot_setup();
}

void loop()
{
  if (millis() - bot_lasttime > BOT_MTBS)
  {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while (numNewMessages)
    {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }

    bot_lasttime = millis();
  }
}