#include "LoRaWan_APP.h"  // Inclui a biblioteca LoRaWAN necessária para comunicação LoRa.
#include "Arduino.h"      // Inclui a biblioteca padrão do Arduino.
#include "HT_SSD1306Wire.h" // Inclui a biblioteca para controlar o display OLED.
#include "images.h"       // Inclui uma biblioteca para gerenciar imagens no display.


#define RF_FREQUENCY                                915000000 // Hz  Define a frequência de operação do LoRa em Hz.
#define TX_OUTPUT_POWER                             21        // dBm    Define a potência de saída do LoRa em dBm.
#define LORA_BANDWIDTH                              0         // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved] Define a largura de banda do LoRa: 0 = 125 kHz.  
#define LORA_SPREADING_FACTOR                       10         // 7[SF7..SF12]   Define o fator de espalhamento do LoRa  
#define LORA_CODINGRATE                             1         // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]    Define a taxa de codificação do LoRa
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx   Define o comprimento do preâmbulo do LoRa
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols     Define o tempo limite do símbolo do LoRa.
#define LORA_FIX_LENGTH_PAYLOAD_ON false // Define se o comprimento do payload é fixo ou não.
#define LORA_IQ_INVERSION_ON false // Define se a inversão de IQ está ativada.

#define RX_TIMEOUT_VALUE 10000    // Define o valor de tempo limite de recepção.
#define BUFFER_SIZE 30            // Define o tamanho do buffer para o payload.

char txpacket[BUFFER_SIZE]; // Declara um buffer para armazenar o pacote de transmissão.

static RadioEvents_t RadioEvents; // Estrutura para eventos de rádio, como TX e RX.

double txNumber; // Declara uma variável para armazenar o número do pacote transmitido.

bool lora_idle = true; // Declara uma variável para indicar se o LoRa está ocioso.



void setup() {
    Serial.begin(115200); // Inicia a comunicação serial a 115200 bps.
    Mcu.begin(3, 0); // Inicializa o microcontrolador.


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
}

void loop() {
    if (lora_idle == true) {
        delay(1000); // Aguarda 1 segundo.
        txNumber += 0.01; // Incrementa txNumber em 0.01.
        // Formata a string do pacote a ser enviado.
        sprintf(txpacket, "Hello world number %0.2f", txNumber);

        // Exibe no console o pacote que será enviado.
        Serial.printf("\r\nsending packet \"%s\" , length %d\r\n", txpacket, strlen(txpacket));

        Radio.Send((uint8_t *)txpacket, strlen(txpacket)); // Envia o pacote LoRa.
        lora_idle = false; // Define o LoRa como ocupado.
    }
    Radio.IrqProcess(); // Processa interrupções de rádio.
}

void OnTxDone(void) {
    Serial.println("TX done......"); // Exibe no console que a transmissão foi concluída.
    lora_idle = true; // Define o LoRa como ocioso.

}

void OnTxTimeout(void) {
    Radio.Sleep(); // Coloca o rádio em modo de suspensão.
    Serial.println("TX Timeout......"); // Exibe no console que ocorreu um timeout.
    lora_idle = true; // Define o LoRa como ocioso.
}


