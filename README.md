# Sensor de nível de caixa d'água

Firmware para NodeMCU ESP8266 com sensor ultrassônico HC-SR04. O dispositivo mede o nível e o volume de água, mantém um histórico de 24 horas em memória RTC e disponibiliza os dados por interface web e API REST local.

<img width="1272" height="910" alt="image" src="https://github.com/user-attachments/assets/978de21a-61f5-4688-8067-c731457b4161" />

Mantenedor: [mantenedor](https://github.com/mantenedor)

## Recursos

- configuração inicial por ponto de acesso aberto `quantagua`;
- armazenamento de SSID, senha, intervalo de coleta, fuso e geometria na EEPROM;
- reservatórios cilíndricos verticais ou retangulares;
- cálculo de nível, volume, margem física e transbordamento;
- intervalos de coleta de 5, 10, 15, 20 ou 30 minutos;
- sincronização NTP e timestamps UTC;
- histórico de 24 horas com gráfico, zoom e navegação temporal;
- modo contínuo ou economia de energia com sono profundo;
- aferição manual do sensor com o reservatório vazio;
- endpoint REST `GET /data`, compatível com coleta pelo Home Assistant.

## Hardware e pinagem

| NodeMCU ESP8266 | Ligação |
|---|---|
| `VIN / 5V` | alimentação regulada de 5 V e VCC do HC-SR04 |
| `GND` | GND comum da alimentação e do HC-SR04 |
| `D5 / GPIO14` | TRIG do HC-SR04 |
| `D6 / GPIO12` | ECHO do HC-SR04 por divisor resistivo para 3,3 V |
| `D0 / GPIO16` | jumper físico até `RST` para despertar do sono profundo |
| `FLASH / GPIO0` | manter pressionado por 10 segundos para apagar a configuração |

O sinal ECHO do HC-SR04 opera em 5 V e não deve ser ligado diretamente ao ESP8266. Use divisor resistivo ou conversor de nível lógico.

## Primeira configuração

1. Energize o dispositivo sem credenciais válidas armazenadas.
2. Conecte-se ao SSID aberto `quantagua`.
3. Acesse `http://192.168.4.1`.
4. Informe a rede Wi-Fi, o intervalo, o fuso, o formato e as medidas do reservatório.
5. Clique em **Aplicar**.
6. Após a conexão, a página exibirá a rede e o IP obtido.
7. Clique em **Finalizar configuração** e conecte-se à rede configurada.

O ponto de acesso `quantagua` é desligado ao final do processo. Para restaurar toda a configuração, mantenha o botão `FLASH` pressionado por 10 segundos enquanto o firmware estiver ativo.

## Geometria do reservatório

Para cilindro vertical, a largura é tratada como diâmetro:

```text
área da base = π × (largura / 2)²
```

Para reservatório retangular:

```text
área da base = largura × profundidade
```

Para ambos:

```text
altura útil nominal = (capacidade em litros / 1000) / área da base
volume real = área da base × altura da água × 1000
transbordamento = máximo(0, volume real - capacidade nominal)
```

A configuração é rejeitada quando a altura total não comporta a capacidade informada ou quando a posição do sensor excede o espaço livre acima do nível nominal de 100%.

## Compilação

Configuração validada:

- Arduino IDE com core ESP8266 3.1.2;
- placa `NodeMCU 1.0 (ESP-12E Module)`;
- FQBN `esp8266:esp8266:nodemcuv2`;
- CPU em 80 MHz;
- upload inicialmente em 115200 baud.

Abra [sensor_nivel_agua.ino](sensor_nivel_agua.ino), selecione a placa e a porta correspondentes e compile ou carregue o firmware.

## API REST

```http
GET /data
```

A resposta inclui a leitura atual, timestamp, RSSI, parâmetros do reservatório, intervalo, fuso, estado do NTP, modo de energia e histórico das últimas 24 horas.

## Economia de energia

No modo econômico, o ESP8266 acorda, realiza a coleta, mantém HTTP/REST disponível por 60 segundos e entra em sono profundo até o próximo intervalo. O despertar automático exige o jumper físico `D0 / GPIO16 → RST`.

## Segurança

Não existem credenciais Wi-Fi fixas no código. SSID e senha informados durante a configuração são armazenados na EEPROM do dispositivo sem criptografia. Arquivos locais de segredos e artefatos de compilação estão excluídos pelo `.gitignore`.

## Licença

Distribuído sob a [PolyForm Noncommercial License 1.0.0](LICENSE). O uso comercial é proibido pelos termos dessa licença. A PolyForm Noncommercial não é uma licença open source aprovada pela OSI, pois restringe o campo de uso.
