#include "LoRaWan_APP.h"  // Inclui a biblioteca LoRaWAN necessária para comunicação LoRa.
#include "Arduino.h"      // Inclui a biblioteca padrão do Arduino.
#include "HT_SSD1306Wire.h" // Inclui a biblioteca para controlar o display OLED.
#include "images.h"       // Inclui uma biblioteca para gerenciar imagens no display.
#include "HT_TinyGPS++.h"

#define GPS_BAUDRATE 9600
#define EARTH_RADIUS 6371000.0  // Raio médio da Terra em metros
#define RF_FREQUENCY                                915000000 // Hz  Define a frequência de operação do LoRa em Hz.
#define TX_OUTPUT_POWER                             20        // dBm    Define a potência de saída do LoRa em dBm.
#define LORA_BANDWIDTH                              0         // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved] Define a largura de banda do LoRa: 0 = 125 kHz.  
#define LORA_SPREADING_FACTOR                       9         // 7[SF7..SF12]   Define o fator de espalhamento do LoRa  
#define LORA_CODINGRATE                             1         // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]    Define a taxa de codificação do LoRa
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx   Define o comprimento do preâmbulo do LoRa
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols     Define o tempo limite do símbolo do LoRa.
#define LORA_FIX_LENGTH_PAYLOAD_ON false // Define se o comprimento do payload é fixo ou não.
#define LORA_IQ_INVERSION_ON false // Define se a inversão de IQ está ativada.

#define RX_TIMEOUT_VALUE 10000    // Define o valor de tempo limite de recepção.
#define BUFFER_SIZE 50            // Define o tamanho do buffer para o payload.

char txpacket[BUFFER_SIZE]; // Declara um buffer para armazenar o pacote de transmissão.

static RadioEvents_t RadioEvents; // Estrutura para eventos de rádio, como TX e RX.

double txNumber; // Declara uma variável para armazenar o número do pacote transmitido.

bool lora_idle = true; // Declara uma variável para indicar se o LoRa está ocioso.

extern SSD1306Wire display; // Declara uma instância externa do display OLED.

TinyGPSPlus gps;



String formatDateTime(TinyGPSDate &date, TinyGPSTime &time);
double toRadians(double degree);

void setup() {
    Serial.begin(115200); // Inicia a comunicação serial a 115200 bps.
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE); // Inicializa o microcontrolador. (3,0)

    display.init(); // Inicializa o display OLED.
    display.flipScreenVertically(); // Inverte a tela do display.
    display.setFont(ArialMT_Plain_10); // Define a fonte para o display.

    txNumber = 0; // Inicializa a variável txNumber com 0.

    // Associa as funções de callback para eventos de TX e timeout.
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;

    Radio.Init(&RadioEvents); // Inicializa o rádio com os eventos definidos.
    Radio.SetChannel(RF_FREQUENCY); // Configura a frequência do rádio.

    // Configura os parâmetros de transmissão do LoRa.
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

    Serial2.begin(GPS_BAUDRATE, SERIAL_8N1, 46, 45);

 updateDisplay(String("Latitude: "+  String(0.0)+", Longitude: "+ String(0.0))+ ", Cont: "+ String(0.0));
                
}

void loop() {
    updateGPSData();
    if (lora_idle == true) {
       // delay(1000); 
      txNumber += 0.01; // Incrementa txNumber em 0.01.
        // Formata a string do pacote a ser enviado.
       // sprintf(txpacket, "Hello world number %0.2f", txNumber);
      if (gps.location.isValid()) {
            sprintf(txpacket, "%0.8f, %0.8f,  %0.2f", gps.location.lat(), gps.location.lng(), txNumber);
        } else {
          //  sprintf(txpacket, "%0.8f, %0.8f,  %0.2f", gps.location.lat(), gps.location.lng(), txNumber);
            sprintf(txpacket, "GPS: dados inválidos, sem coordenadas, %0.2f", txNumber);
      }

        // Exibe no console o pacote que será enviado.
      Serial.printf("\r\nsending packet \"%s\" , length %d\r\n", txpacket, strlen(txpacket));

      Radio.Send((uint8_t *)txpacket, strlen(txpacket)); // Envia o pacote LoRa.
      lora_idle = false; // Define o LoRa como ocupado.
      delay(60000); // Aguarda 6 segundos. mudar para 6000
    }
    Radio.IrqProcess(); // Processa interrupções de rádio.
    //delay(300000); // Aguarda 6 segundos. mudar para 6000 
}

void OnTxDone(void) {
    updateDisplay(String("Latitude: "+  String(gps.location.lat(), 8)+", Longitude: "+ String(gps.location.lng(), 8))+ ", Cont: "+ String(txNumber));
    Serial.println("TX done......"); // Exibe no console que a transmissão foi concluída.
    lora_idle = true; // Define o LoRa como ocioso.
    // Atualiza o display OLED com a mensagem enviada.
    //updateDisplay(String("Latitude: "+  String(gps.location.lat(), 8)+", Longitude: "+ String(gps.location.lng(), 8))+ ", Cont: "+ String(txNumber));
}

void OnTxTimeout(void) {
    Radio.Sleep(); // Coloca o rádio em modo de suspensão.
    Serial.println("TX Timeout......"); // Exibe no console que ocorreu um timeout.
    lora_idle = true; // Define o LoRa como ocioso.
}

void updateDisplay(const String &message) {
    display.clear(); // Limpa o display.
    display.setTextAlignment(TEXT_ALIGN_LEFT); // Alinha o texto à esquerda.
    display.setFont(ArialMT_Plain_10); // Define a fonte do texto.
    display.drawString(0, 0, "Sent:"); // Desenha "Sent:" na posição (0, 0).
    display.drawStringMaxWidth(0, 15, 128, message); // Desenha a mensagem enviada.
    display.display(); // Atualiza o display.
}

void updateGPSData() {
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      // Os dados do GPS são atualizados
    }
  }
}
