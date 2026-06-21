# Sensor de Nivel da Caixa d'Agua

## Objetivo

Medir o nivel de uma caixa d'agua de 5.000 litros usando:

- NodeMCU DOITING com modulo ESP-12F / ESP8266;
- sensor ultrassonico HC-SR04;
- alimentacao por powerbank;
- interface web local.

O sketch atual esta em `sensor_nivel_agua.ino`.

## Hardware e IDE

- Placa na Arduino IDE: `NodeMCU 1.0 (ESP-12E Module)`.
- Modulo identificado no projeto: ESP-12F / ESP8266.
- Conversor USB detectado pelo Windows: Silicon Labs CP2102,
  `VID_10C4` / `PID_EA60`.
- O driver aplicavel e o Silicon Labs CP210x VCP; o driver CH340 nao atende
  esta placa especifica.
- Frequencia da CPU: 80 MHz.
- Flash: 4 MB.
- Velocidade inicial de upload: 115200.
- `TRIG`: D5 / GPIO14.
- `ECHO`: D6 / GPIO12.
- Para despertar automaticamente do sono profundo, instalar um fio entre
  `D0 / GPIO16` e `RST`.
- O botao `FLASH / GPIO0`, mantido pressionado por 10 segundos enquanto o
  firmware esta executando, apaga as configuracoes persistentes e reinicia
  o ponto de acesso `quantagua`.
- O `ECHO` do HC-SR04 e 5 V. A entrada do ESP8266 e 3,3 V, portanto deve ser usado divisor resistivo ou conversor de nivel.

## Parametros iniciais

- Formato: cilindrico vertical.
- Altura total: 1,51 m.
- Largura/diametro: 2,25 m.
- Capacidade nominal: 5.000 L.
- Sensor: 10 cm acima do nivel nominal de 100%.
- Para formato retangular, tambem deve ser informada a profundidade.

### Definicoes

- **Altura total:** altura fisica interna do reservatorio.
- **Altura util nominal:** coluna de agua necessaria para atingir a
  capacidade nominal com a area de base informada.
- **Posicao do sensor:** distancia entre a face do sensor e o nivel nominal
  de 100%. Valor inicial: 10 cm.
- **Distancia do sensor:** distancia media aferida entre o sensor e a superficie refletora.
- **Altura estimada da agua:** coluna de agua calculada dentro da caixa.
- **Espaco livre:** altura total menos altura util nominal.

## Formula do nivel

```text
altura estimada da agua =
  altura util nominal -
  (distancia do sensor - posicao do sensor)
```

Forma equivalente:

```text
altura estimada da agua =
  altura util nominal +
  posicao do sensor -
  distancia do sensor
```

O percentual e calculado por:

```text
nivel (%) =
  altura estimada da agua / altura util nominal * 100
```

O nivel exibido e armazenado fica limitado entre 0% e 100%. Quando a altura
da agua ultrapassa a altura util nominal, o volume real pode superar a
capacidade e o excedente e informado como transbordamento.

## Estimativa de volume

Reservatorio cilindrico vertical, usando a largura como diametro:

```text
area da base = pi * (largura / 2)^2
```

Reservatorio retangular:

```text
area da base = largura * profundidade
```

Para ambos:

```text
altura util nominal = (capacidade em litros / 1000) / area da base
espaco livre = altura total - altura util nominal
volume real = area da base * altura da agua * 1000
transbordamento = max(0, volume real - capacidade nominal)
```

A configuracao e rejeitada quando a altura util excede a altura total ou
quando a posicao do sensor excede o espaco livre. O HTML mostra a altura
util calculada e sugere a maior posicao viavel para o sensor.

Para os valores iniciais cilindricos:

```text
area da base = 3,9761 m2
altura util nominal = 1,2575 m
espaco livre = 0,2525 m = 25,25 cm
sensor de 10 cm = valido
```

## Afericao do HC-SR04

- Uma afericao e a media de 5 pulsos validos.
- Sao exigidos pelo menos 3 pulsos validos.
- Ha uma pausa de 60 ms entre pulsos.
- Fora da calibracao, o intervalo e configuravel em 5, 10, 15, 20 ou
  30 minutos. Durante a calibracao, as afericoes continuam a cada 3 segundos.
- O valor mostrado como `Distancia do sensor` deve ser essa media direta, sem conversao.

## Afericao da posicao do sensor

- A energizacao nao inicia mais calibracao automatica.
- Ao pressionar `Aferir sensor`, abre-se uma janela de 30 segundos.
- O reservatorio deve estar vazio, pois a maior distancia observada deve
  representar o fundo.
- Durante esse periodo, nivel e litros ficam indisponiveis.
- O sketch registra a maior media observada como distancia ate o fundo.
- Ao final:

```text
posicao aferida do sensor =
  distancia observada ate o fundo - altura util nominal
```

- O resultado so e aceito entre zero e o espaco livre calculado.
- Quando aceito, atualiza a configuracao EEPROM e apaga o historico anterior.
- Despertares do sono profundo nao repetem a afericao.

## Ajuste da posicao do sensor

A posicao do sensor nao e mais reduzida automaticamente por leituras menores.
Ela muda apenas pela pagina de configuracao ou pela afericao manual com o
reservatorio vazio.

## Interface web

A interface mostra:

- volume estimado;
- nivel percentual;
- altura estimada da agua;
- distancia do sensor;
- profundidade da caixa;
- posicao do sensor;
- transbordamento e litros acima da capacidade nominal;
- margem restante ate o topo fisico;
- RSSI do Wi-Fi;
- timestamp e horario da ultima coleta;
- historico de 24 horas;
- botao para restaurar a calibracao;
- botao de economia de energia;
- botao de configuracao.

Na configuracao do reservatorio:

- formatos disponiveis: cilindrico vertical e retangular;
- cilindrico solicita altura total, largura/diametro e capacidade;
- retangular tambem solicita profundidade;
- ambos solicitam a posicao do sensor acima do nivel nominal de 100%;
- o navegador calcula altura util e espaco livre antes de aceitar o formulario;
- o firmware repete a mesma validacao no servidor;
- alteracoes geometricas apagam o historico RTC anterior;
- alteracoes apenas de rede, intervalo ou fuso preservam o historico.

O grafico:

- inicia mostrando 24 horas completas;
- ajusta a largura ao tamanho da tela;
- ajusta automaticamente as marcacoes do eixo temporal;
- permite zoom pela roda do mouse;
- permite zoom por pinca em dispositivos moveis;
- permite navegar no tempo por arraste;
- limita o zoom entre 30 minutos e 24 horas;
- mostra tooltip com tempo, nivel e volume estimado;
- mostra um marcador no ponto selecionado;
- permite fixar o tooltip com toque curto no mobile;
- liga continuamente todos os pontos coletados em ordem cronologica;
- posiciona os pontos proporcionalmente pelos timestamps;
- usa 288 slots de 5 minutos na memoria RTC para cobrir 24 horas;
- compacta cada nivel em um byte, com resolucao aproximada de 0,4%;
- respeita o intervalo de coleta configurado, deixando slots sem coleta vazios.

O frontend consulta os dados a cada 30 segundos.

O historico continua armazenando apenas o nivel percentual. Altura e litros
exibidos no tooltip sao recalculados no navegador para preservar a RAM da
ESP8266.

## API HTTP

O endpoint atual e:

```text
GET /data
```

Ele retorna:

- RSSI;
- distancia, altura, nivel, litros, transbordamento, margem ate o topo,
  pulsos validos e timestamp atuais;
- geometria calculada, parametros e estado da afericao;
- intervalo de coleta, modo de energia, fuso horario e estado do NTP;
- historico completo reconstruido a partir da memoria RTC.

O endpoint funciona com o Home Assistant, mas transfere o historico completo
a cada consulta. Existe uma proposta ainda nao implementada para criar um
endpoint leve, como `/api/current`, contendo somente os valores atuais.

## Configuracao de rede, intervalo e fuso

- As credenciais Wi-Fi nao ficam mais fixas no codigo-fonte.
- Quando nao ha credenciais validas na EEPROM, o ESP8266 cria o ponto de
  acesso aberto `quantagua`, com pagina em `http://192.168.4.1`.
- A pagina permite configurar SSID, senha, intervalo de coleta, fuso,
  formato, dimensoes, capacidade e posicao do sensor.
- Intervalos permitidos: 5, 10, 15, 20 e 30 minutos.
- Fusos permitidos: `America/Sao_Paulo`, `America/Noronha`,
  `America/Manaus`, `America/Rio_Branco` e `UTC`.
- A senha nunca e devolvida ao navegador. Campo de senha vazio preserva a
  senha atual quando o SSID nao e alterado.
- Ao aplicar, o ESP opera temporariamente em `AP+STA`: mantem `quantagua`,
  conecta a rede informada e solicita IP por DHCP sem reiniciar.
- A pagina informa a rede conectada e o IP obtido. Em caso de falha por
  30 segundos, o AP permanece ativo para nova tentativa.
- Ao clicar em `Finalizar configuracao`, o AP e desligado. O navegador
  aguarda 10 segundos para a reconexao automatica do dispositivo do usuario
  e redireciona para o IP obtido.
- Depois de configurado e finalizado, o SSID `quantagua` desaparece.
- O botao `Configuracao` abre a pagina pela rede atual; ele nao ativa o AP
  imediatamente.
- O formato EEPROM foi versionado novamente para incluir a geometria. Apos
  carregar esta versao do firmware, configuracoes da versao anterior sao
  consideradas invalidas e o AP `quantagua` e aberto para nova configuracao.

## NTP, coleta e economia de energia

- O firmware consulta NTP e usa timestamps Unix/ISO 8601 nas coletas.
- O grafico converte o timestamp para o fuso configurado.
- No modo continuo, as coletas sao alinhadas aos intervalos configurados.
- No modo de economia, o ESP acorda, coleta, disponibiliza HTTP/REST por
  60 segundos e entra em sono profundo ate o proximo intervalo.
- O despertar automatico depende obrigatoriamente do jumper fisico
  `D0 / GPIO16 -> RST`.
- O modo escolhido e a calibracao sao preservados na RTC durante o sono.
- O frontend continua consultando `/data` a cada 30 segundos enquanto o
  dispositivo estiver ativo. Isso nao cria novas coletas: o grafico muda
  somente quando existe um novo ponto no intervalo configurado.

## Estimativa de energia adotada

Valores conceituais usados nas discussoes de alimentacao:

- consumo continuo estimado do NodeMCU + HC-SR04: 0,475 W;
- energia em 24 horas no modo continuo: 11,4 Wh;
- uma celula 18650 de 3.000 mAh: aproximadamente 9,4 Wh uteis, assumindo
  conversao com 85% de eficiencia;
- autonomia continua estimada com essa celula: aproximadamente 20 horas;
- painel considerado: 5 V / 500 mA, potencia maxima de 2,5 W;
- com 5 horas equivalentes de sol pleno e 75% de eficiencia global:
  aproximadamente 9,4 Wh uteis gerados por dia;
- o sistema solar precisa de carregador com `power path`/`load sharing`;
  um BMS comum protege a celula, mas nao gerencia sozinho a prioridade entre
  painel, carga e bateria.

Esses valores sao estimativas. O consumo real deve ser medido no hardware.

## Home Assistant

O Home Assistant roda em Docker e consulta os sensores com a integracao REST.
A frequencia definida e de 30 segundos.

Estrutura de referencia:

```yaml
rest:
  - resource: "http://IP_DO_SENSOR/data"
    scan_interval: 30
    timeout: 10
    sensor:
      - name: "Caixa d'agua 5k"
        unique_id: caixa_agua_5k_nivel
        value_template: "{{ value_json.current.levelPercent }}"
        unit_of_measurement: "%"
        state_class: measurement
```

Cada caixa deve usar nomes e `unique_id` exclusivos. Entidades conhecidas:

```text
sensor.caixa_d_agua_nivel
sensor.caixa_d_agua_volume
sensor.caixa_d_agua_5k
sensor.caixa_d_agua_5k_volume
sensor.caixa_d_agua_5k_altura
sensor.caixa_d_agua_5k_distancia
sensor.sensor_da_caixa_5k_rssi
```

O historico e mantido pelo `recorder` do Home Assistant. A API de historico
foi validada. Os timestamps retornados pela API ficam em UTC e devem ser
convertidos pelo consumidor para `America/Sao_Paulo`.

## MQTT e notificacoes

Arquitetura definida:

```text
ESP -> REST -> Home Assistant -> MQTT -> bot shell -> Telegram
```

O broker MQTT ja esta integrado ao Home Assistant. MQTT sera usado para
eventos; o historico permanecera no Home Assistant.

Topico sugerido:

```text
casa/caixa_agua/eventos
```

Eventos pretendidos, ainda nao implementados:

- nivel baixo;
- enchimento iniciado;
- enchimento parado;
- payload com evento, nivel, litros e horario.

## Ruido e filtragem

O sensor em producao apresentou ruido previsivel e predominantemente para
baixo. Em uma amostra extensa, o nivel oscilou aproximadamente entre 46% e
55% com a agua parada.

Decisao inicial para o Home Assistant:

- preservar o sensor original para diagnostico;
- criar um ajudante estatistico;
- usar percentil 90;
- usar janela de 10 minutos;
- usar tamanho de amostra 30;
- manter a ultima amostra.

Regras preliminares, ainda nao implementadas:

```text
nivel baixo: patamar <= 25% por 5 minutos
rearme: patamar >= 30%
enchimento: patamar sobe >= 2% em 10 minutos
parada: variacao < 0,5% por 10 minutos
```

Esses limites devem ser confirmados depois de observar o sensor filtrado.

## Restauracao

O botao `Aferir sensor` inicia por 30 segundos a medicao manual do fundo.
Ele nao restaura dimensoes padrao. O reservatorio deve estar vazio.

Para restaurar rede, intervalo, fuso e geometria:

```text
manter FLASH / GPIO0 pressionado por 10 segundos com o firmware ativo
```

O pino `RST` nao pode medir uma pressao longa porque mantem o ESP8266 em
reset enquanto estiver pressionado. No modo economico, deve-se acordar o
dispositivo, soltar `RST` e entao manter `FLASH` pressionado.

## Estado da versao inicial

Esta versao foi considerada suficiente como versao inicial para continuar a evolucao.

O sketch foi compilado com sucesso pela Arduino IDE usando a placa
`NodeMCU 1.0 (ESP-12E Module)`. Resultado observado:

```text
RAM global/estatica: 41.052 / 80.192 bytes (51%)
IRAM:                60.471 / 65.536 bytes (92%)
Flash:              281.924 / 1.048.576 bytes (26%)
```

O uso de IRAM esta alto, mas a compilacao foi concluida. Nao otimizar ou
alterar o firmware apenas por esse numero sem uma decisao explicita.

O upload falhou inicialmente porque `COM3` era uma porta Bluetooth. O
Windows detectou o CP2102 com erro 28 (driver ausente). O estado final do
upload apos a instalacao do driver CP210x ainda nao esta documentado.

## Estado atual da compilacao e diagnostico da IDE

O firmware atual foi compilado com sucesso pela `arduino-cli` fornecida pela
Arduino IDE, usando `esp8266:esp8266:nodemcuv2` e core ESP8266 3.1.2:

```text
RAM global/estatica: 32.224 / 80.192 bytes (40%)
IRAM:                60.807 / 65.536 bytes (92%)
Flash:              332.052 / 1.048.576 bytes (31%)
```

Foi observado na Arduino IDE o erro:

```text
invalid character '\x00' looking for beginning of value
```

O `sensor_nivel_agua.ino` foi verificado e nao contem bytes nulos. A mesma
versao compilou normalmente pela CLI. A causa localizada foi um arquivo de
cache da Arduino IDE totalmente preenchido com `0x00`:

```text
C:\Users\wagtono\AppData\Local\arduino\sketches\
A2F7B032DC2F6B1F74FBF5332409E2B7\compile_commands.json
```

O arquivo tinha 53.016 bytes, todos nulos. A correcao indicada e fechar a
Arduino IDE e remover essa pasta de cache, que sera recriada. A remocao ainda
nao foi autorizada nem executada.

## Estado do repositorio em 2026-06-18

- Branch atual: `master`.
- O repositorio ainda nao possui commits.
- Arquivos locais principais: `AGENTS.md`, `CONTEXTO_PROJETO.md` e
  `sensor_nivel_agua.ino`.
- Os tres arquivos estao nao rastreados pelo Git.
- Nenhum commit, push ou pull request foi solicitado ou realizado.

## Licenca e publicacao

- Licenca definida: PolyForm Noncommercial License 1.0.0.
- O uso comercial do firmware e proibido pelos termos da licenca.
- A PolyForm Noncommercial nao e uma licenca open source aprovada pela OSI,
  pois restringe o campo de uso.
- Conta de publicacao e manutencao informada: `mantenedor`.
- Repositorio publico pretendido: `sensor-nivel-caixa-agua`.
- Foram criados `README.md`, `LICENSE` e `.gitignore`.
- A conta `mantenedor` foi autenticada no GitHub CLI para a publicacao.

## Pontos para revisao futura

O HC-SR04 apresenta pequenas oscilacoes mesmo diante de uma superficie parada. Depois de calibrar o fundo, uma leitura ligeiramente menor pode produzir uma pequena altura de agua e alguns litros. Nenhuma zona morta ou tolerancia adicional deve ser aplicada sem decisao explicita.

- Compilar e validar o sketch na Arduino IDE.
- Observar o ajudante estatistico do Home Assistant.
- Definir e implementar as automacoes MQTT.
- Avaliar a criacao do endpoint leve `/api/current`.

## Seguranca

Chaves privadas, tokens e senhas nunca devem ser registrados neste
repositorio. Credenciais expostas durante testes devem ser revogadas e
substituidas.

As credenciais fixas foram removidas do sketch. SSID e senha configurados em
execucao sao armazenados na EEPROM em texto nao criptografado e nao devem ser
registrados no repositorio ou em logs. O ponto de acesso `quantagua` e aberto;
durante a configuracao, as credenciais trafegam sem criptografia Wi-Fi.

## Orientacao para o proximo chat

Antes de alterar o projeto:

1. Ler este documento.
2. Ler integralmente `sensor_nivel_agua.ino`.
3. Nao reintroduzir credenciais fixas no codigo. Preservar o fluxo de
   configuracao EEPROM/AP+STA e o grafico, salvo instrucao expressa.
4. Entregar sempre o sketch completo quando solicitado.
5. Questionar conflitos de definicao antes de modificar formulas.
6. Nao alterar filtros, limites de alerta ou arquitetura sem decisao expressa.
7. Nao registrar segredos, tokens ou chaves no repositorio.
8. Conferir `git status` porque, na ultima consolidacao, o repositorio ainda
   nao possuia commits e todos os arquivos estavam nao rastreados.
9. Para sono profundo, considerar obrigatorio o jumper `D0/GPIO16 -> RST`.
10. Nao confundir auto-refresh HTTP de 30 segundos com intervalo de coleta.
11. O cache corrompido da Arduino IDE foi diagnosticado, mas ainda nao foi
    removido.
