#include "LoRaWan_APP.h"  // Inclui a biblioteca LoRaWAN necessária para comunicação LoRa.
#include "Arduino.h"      // Inclui a biblioteca padrão do Arduino.
#include "string.h"       // Inclui a biblioteca para manipulação de strings.
#include "HT_SSD1306Wire.h" // Inclui a biblioteca para controlar o display OLED.
#include "images.h"       // Inclui uma biblioteca para gerenciar imagens no display.

extern SSD1306Wire display; // Declara uma instância externa do display OLED.

#define RF_FREQUENCY                                915000000 // Hz     Define a frequência de operação do LoRa em Hz.
#define LORA_BANDWIDTH                              0         // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]    Define a largura de banda do LoRa: 0 = 125 kHz.
#define LORA_SPREADING_FACTOR                       9         // 7[SF7..SF12]    Define o fator de espalhamento do LoRa
#define LORA_CODINGRATE                             1         // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]     Define a taxa de codificação do LoRa
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx     Define o comprimento do preâmbulo do LoRa.
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols     Define o tempo limite do símbolo do LoRa.
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false     // Define se o comprimento do payload é fixo ou não.
#define LORA_IQ_INVERSION_ON                        false     // Define se a inversão do IQ (quadratura inversa) está ativada.

#define RX_TIMEOUT_VALUE                            10000     // Define o valor do tempo limite de recebimento em milissegundos.
#define BUFFER_SIZE                                 30 // Define o tamanho do payload aqui.

char rxpacket[BUFFER_SIZE];  // Declara um array para armazenar o pacote recebido.

static RadioEvents_t RadioEvents;  // Declara uma estrutura de eventos de rádio.

int16_t rssi, rxSize;  // Declara variáveis para armazenar o valor do RSSI e o tamanho do pacote recebido.

bool lora_idle = true;  // Declara uma flag para indicar se o LoRa está inativo.

void setup() {
    Serial.begin(115200);  // Inicializa a comunicação serial na taxa de 115200 bps.
    Mcu.begin();  // Inicializa a MCU (Unidade de Microcontrole).

    display.init();  // Inicializa o display OLED.
    display.flipScreenVertically();  // Inverte a orientação do display.
    display.setFont(ArialMT_Plain_10);  // Define a fonte do texto no display.
    
    updateDisplay("Waiting for data...", 0); // Mensagem inicial do display

    RadioEvents.RxDone = OnRxDone;  // Define o evento que será chamado quando a recepção for concluída.
    Radio.Init(&RadioEvents);  // Inicializa o rádio com os eventos definidos.
    Radio.SetChannel(RF_FREQUENCY);  // Define o canal de operação do rádio.

    // Configura o rádio para modo de recepção com os parâmetros definidos anteriormente.
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
}

void loop() {
    if (lora_idle) {  // Verifica se o LoRa está inativo.
        lora_idle = false;  // Define a flag como false para indicar que o LoRa está ativo.
        Serial.println("into RX mode");  // Envia uma mensagem pela serial indicando que entrou no modo de recepção.
        Radio.Rx(0);  // Coloca o rádio em modo de recepção.
    }
    Radio.IrqProcess();  // Processa as interrupções do rádio.
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    rssi = rssi;  // Armazena o valor do RSSI.
    rxSize = size;  // Armazena o tamanho do pacote recebido.
    memcpy(rxpacket, payload, size);  // Copia o conteúdo do payload para o array rxpacket.
    rxpacket[size] = '\0';  // Adiciona o caractere nulo ao final do array para formar uma string.
    Radio.Sleep();  // Coloca o rádio em modo de suspensão.
    Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n", rxpacket, rssi, rxSize);  // Exibe na serial o pacote recebido, o valor do RSSI e o tamanho do pacote.
    lora_idle = true;  // Define a flag como true para indicar que o LoRa está inativo novamente.

    // Lida com os dados recebidos aqui.
    LoRaData();  // Chama a função para processar os dados recebidos.

    // Atualiza o display OLED com a mensagem recebida.
    updateDisplay(String(rxpacket), rssi);  // Chama a função para atualizar o display com a mensagem e o valor do RSSI.
}

void LoRaData() {
    display.clear();  // Limpa o display OLED.
    display.setTextAlignment(TEXT_ALIGN_LEFT);  // Define o alinhamento do texto à esquerda.
    display.setFont(ArialMT_Plain_10);  // Define a fonte do texto.
    display.drawString(0, 15, "Received " + String(rxSize) + " bytes");  // Desenha uma string no display indicando o número de bytes recebidos.
    display.drawStringMaxWidth(0, 26, 128, rxpacket);  // Desenha a string recebida no display.
    display.display();  // Atualiza o display para mostrar as mudanças.
}

void updateDisplay(const String &message,  int16_t rssi) {
    display.clear();  // Limpa o display OLED.
    display.setTextAlignment(TEXT_ALIGN_LEFT);  // Define o alinhamento do texto à esquerda.
    display.setFont(ArialMT_Plain_10);  // Define a fonte do texto.
    display.drawString(0, 0, "Received:");  // Desenha uma string no display.
    display.drawStringMaxWidth(0, 15, 128, message);  // Desenha a mensagem recebida no display.
    display.drawString(0, 30, "RSSI: " + String(rssi) + " dBm");  // Desenha o valor do RSSI no display.
    display.display();  // Atualiza o display para mostrar as mudanças.
}
