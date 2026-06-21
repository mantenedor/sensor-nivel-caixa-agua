/*
  Pinagem do conjunto

  NodeMCU DOITING / ESP-12F (ESP8266)     HC-SR04 / alimentacao
  ------------------------------------------------------------------
  VIN / 5V                               Alimentacao regulada de 5 V
  GND                                    GND comum da alimentacao e HC-SR04
  VIN / 5V                               VCC do HC-SR04
  D5 / GPIO14                            TRIG do HC-SR04
  D6 / GPIO12                            ECHO do HC-SR04 via divisor resistivo
  D0 / GPIO16                            RST do NodeMCU (jumper fisico)
  FLASH / GPIO0                          Botao de restauracao das configuracoes

  Observacoes:
  - O ECHO do HC-SR04 opera em 5 V e nao deve ser conectado diretamente
    ao ESP8266. O divisor resistivo deve reduzir o sinal para 3,3 V.
  - Instalar um fio ligando diretamente o pino D0 ao pino RST do NodeMCU.
    Esse jumper D0/GPIO16 -> RST permite que o temporizador interno acorde
    o ESP8266 automaticamente depois do sono profundo.
  - Todos os modulos devem compartilhar o mesmo GND.
  - Nao aplicar 5 V diretamente nos pinos GPIO ou no pino 3V3.
  - Com o firmware ativo, manter FLASH pressionado por 10 segundos apaga
    rede, intervalo e fuso e reinicia o ponto de acesso quantagua.

  Calculos geometricos:

  Reservatorio cilindrico vertical:
    area da base (m2) = PI * (largura / 2)^2

  Reservatorio retangular:
    area da base (m2) = largura * profundidade

  Para ambos os formatos:
    capacidade (m3) = capacidade em litros / 1000
    altura util nominal (m) = capacidade (m3) / area da base
    espaco livre (m) = altura total - altura util nominal

  A configuracao somente e valida quando:
    altura util nominal <= altura total
    distancia do sensor ao nivel nominal <= espaco livre

  A distancia configurada do sensor e medida entre a face do HC-SR04 e o
  nivel nominal de 100%. Portanto:
    altura da agua = altura util nominal + distancia do sensor
                      - distancia medida
    volume real (L) = area da base * altura da agua * 1000

  Quando o volume real ultrapassa a capacidade informada, o nivel exibido
  permanece limitado a 100% e o excedente e informado como transbordamento.
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <time.h>

const char* CONFIG_AP_SSID = "quantagua";
const uint32_t WIFI_CONFIG_MAGIC = 0x57494633;
const size_t EEPROM_SIZE = 512;
const char* DEFAULT_TIME_ZONE = "America/Sao_Paulo";

#define TRIG_PIN D5
#define ECHO_PIN D6
#define CONFIG_RESET_PIN 0

enum TankShape : uint8_t {
  TANK_CYLINDRICAL = 0,
  TANK_RECTANGULAR = 1
};

const float DEFAULT_TANK_HEIGHT_M = 1.51;
const float DEFAULT_TANK_WIDTH_M = 2.25;
const float DEFAULT_TANK_DEPTH_M = 2.25;
const float DEFAULT_TANK_CAPACITY_L = 5000.0;
const float DEFAULT_SENSOR_POSITION_CM = 10.0;

const unsigned long HISTORY_SLOT_SECONDS = 5UL * 60UL;
const unsigned long CALIBRATION_READ_INTERVAL_MS = 3000;
const unsigned long DEPTH_CALIBRATION_MS = 30000;
const unsigned long ACTIVE_WINDOW_MS = 60000;
const unsigned long NTP_TIMEOUT_MS = 15000;

// 288 intervalos de cinco minutos representam exatamente 24 horas.
const int HISTORY_SIZE = 288;
const uint8_t INVALID_HISTORY_LEVEL = 255;
const uint32_t RTC_MAGIC = 0x43415842;

const unsigned long ECHO_TIMEOUT_US = 30000;

// Cada afericao e a media de cinco pulsos em sequencia curta.
const int SAMPLE_COUNT = 5;
const int MIN_VALID_SAMPLES = 3;
const unsigned long SAMPLE_PAUSE_MS = 60;

ESP8266WebServer server(80);

struct WifiConfig {
  uint32_t magic;
  uint32_t checksum;
  char ssid[33];
  char password[65];
  uint16_t collectionIntervalMinutes;
  char timeZone[32];
  uint8_t tankShape;
  float tankHeightM;
  float tankWidthM;
  float tankDepthM;
  float tankCapacityL;
  float sensorPositionCm;
};

static_assert(
  sizeof(WifiConfig) <= EEPROM_SIZE,
  "A configuracao excede o espaco reservado na EEPROM"
);

WifiConfig wifiConfig;
bool configurationMode = false;
bool configurationConnectionRequested = false;
bool configurationConnectionActive = false;
bool finishConfigurationRequested = false;
unsigned long configurationRequestMs = 0;
unsigned long configurationConnectionStartMs = 0;
unsigned long flashButtonPressedMs = 0;
unsigned long collectionIntervalSeconds = HISTORY_SLOT_SECONDS;

struct RtcState {
  uint32_t magic;
  uint32_t checksum;
  uint32_t lastHistorySlot;
  float tankDepthCm;
  float sensorPositionCm;
  float observedBottomDistanceCm;
  uint8_t energySavingMode;
  uint8_t calibrationComplete;
  uint8_t hasObservedBottom;
  uint8_t reserved;
  uint8_t historyLevels[HISTORY_SIZE];
};

static_assert(
  sizeof(RtcState) <= 512,
  "O estado excede os 512 bytes da memoria RTC do ESP8266"
);

RtcState rtcState;

unsigned long lastReadMs = 0;
unsigned long activeWindowStartMs = 0;
uint32_t lastReadSlot = 0;
bool ntpSynchronized = false;
bool energySavingMode = false;

// Parametros geometricos derivados da configuracao persistente.
float tankTotalHeightCm = DEFAULT_TANK_HEIGHT_M * 100.0;
float tankDepthCm = 0;
float sensorPositionCm = DEFAULT_SENSOR_POSITION_CM;
float tankBaseAreaM2 = 0;
float tankCapacityLiters = DEFAULT_TANK_CAPACITY_L;
float tankFreeboardCm = 0;

// Estado da calibracao inicial da profundidade.
bool depthCalibrationActive = false;
unsigned long depthCalibrationStartMs = 0;

// Maior media aferida durante os 30 segundos de calibracao.
bool hasObservedBottom = false;
float observedBottomDistanceCm = 0;

float currentDistanceCm = -1;
float currentWaterHeightCm = -1;
float currentLevelPercent = -1;
float currentLiters = -1;
float currentOverflowLiters = 0;
float currentMarginToTopCm = -1;
bool currentOverflow = false;
int currentValidSamples = 0;
time_t currentCollectionTimestamp = 0;

void monitorConfigurationResetButton();

uint32_t calculateWifiChecksum(const WifiConfig& config) {
  const uint8_t* bytes =
    reinterpret_cast<const uint8_t*>(&config);
  uint32_t checksum = 2166136261UL;

  for (size_t i = 8; i < sizeof(WifiConfig); i++) {
    checksum ^= bytes[i];
    checksum *= 16777619UL;
  }

  return checksum;
}

bool isAllowedCollectionInterval(uint16_t minutes) {
  return
    minutes == 5 ||
    minutes == 10 ||
    minutes == 15 ||
    minutes == 20 ||
    minutes == 30;
}

bool isAllowedTimeZone(const String& timeZone) {
  return
    timeZone == "America/Sao_Paulo" ||
    timeZone == "America/Noronha" ||
    timeZone == "America/Manaus" ||
    timeZone == "America/Rio_Branco" ||
    timeZone == "UTC";
}

float calculateTankBaseAreaM2(
  uint8_t shape,
  float widthM,
  float depthM
) {
  if (shape == TANK_CYLINDRICAL) {
    float radiusM = widthM / 2.0;
    return PI * radiusM * radiusM;
  }

  if (shape == TANK_RECTANGULAR) {
    return widthM * depthM;
  }

  return 0;
}

float calculateRequiredWaterHeightM(
  uint8_t shape,
  float widthM,
  float depthM,
  float capacityL
) {
  float baseAreaM2 =
    calculateTankBaseAreaM2(shape, widthM, depthM);

  if (baseAreaM2 <= 0) {
    return -1;
  }

  return (capacityL / 1000.0) / baseAreaM2;
}

bool isTankConfigurationValid(
  uint8_t shape,
  float heightM,
  float widthM,
  float depthM,
  float capacityL,
  float sensorPositionCm
) {
  if (
    (shape != TANK_CYLINDRICAL &&
     shape != TANK_RECTANGULAR) ||
    !isfinite(heightM) ||
    !isfinite(widthM) ||
    !isfinite(depthM) ||
    !isfinite(capacityL) ||
    !isfinite(sensorPositionCm) ||
    heightM <= 0 ||
    widthM <= 0 ||
    capacityL <= 0 ||
    sensorPositionCm < 0 ||
    (shape == TANK_RECTANGULAR && depthM <= 0)
  ) {
    return false;
  }

  float requiredHeightM = calculateRequiredWaterHeightM(
    shape,
    widthM,
    depthM,
    capacityL
  );
  float availableSensorSpaceM = heightM - requiredHeightM;

  return
    requiredHeightM > 0 &&
    requiredHeightM <= heightM &&
    sensorPositionCm / 100.0 <= availableSensorSpaceM;
}

bool tankGeometryChanged(
  uint8_t shape,
  float heightM,
  float widthM,
  float depthM,
  float capacityL,
  float sensorPositionCm
) {
  const float epsilon = 0.0001;

  return
    wifiConfig.tankShape != shape ||
    fabs(wifiConfig.tankHeightM - heightM) > epsilon ||
    fabs(wifiConfig.tankWidthM - widthM) > epsilon ||
    fabs(wifiConfig.tankDepthM - depthM) > epsilon ||
    fabs(wifiConfig.tankCapacityL - capacityL) > 0.1 ||
    fabs(wifiConfig.sensorPositionCm - sensorPositionCm) > 0.01;
}

void applyTankConfigurationToRuntime() {
  tankBaseAreaM2 = calculateTankBaseAreaM2(
    wifiConfig.tankShape,
    wifiConfig.tankWidthM,
    wifiConfig.tankDepthM
  );
  tankDepthCm = calculateRequiredWaterHeightM(
    wifiConfig.tankShape,
    wifiConfig.tankWidthM,
    wifiConfig.tankDepthM,
    wifiConfig.tankCapacityL
  ) * 100.0;
  tankTotalHeightCm = wifiConfig.tankHeightM * 100.0;
  sensorPositionCm = wifiConfig.sensorPositionCm;
  tankCapacityLiters = wifiConfig.tankCapacityL;
  tankFreeboardCm = tankTotalHeightCm - tankDepthCm;
}

bool loadWifiConfig() {
  EEPROM.get(0, wifiConfig);

  if (
    wifiConfig.magic != WIFI_CONFIG_MAGIC ||
    wifiConfig.checksum != calculateWifiChecksum(wifiConfig)
  ) {
    return false;
  }

  wifiConfig.ssid[sizeof(wifiConfig.ssid) - 1] = '\0';
  wifiConfig.password[sizeof(wifiConfig.password) - 1] = '\0';
  wifiConfig.timeZone[sizeof(wifiConfig.timeZone) - 1] = '\0';

  if (
    strlen(wifiConfig.ssid) == 0 ||
    !isAllowedCollectionInterval(
      wifiConfig.collectionIntervalMinutes
    ) ||
    !isAllowedTimeZone(String(wifiConfig.timeZone)) ||
    !isTankConfigurationValid(
      wifiConfig.tankShape,
      wifiConfig.tankHeightM,
      wifiConfig.tankWidthM,
      wifiConfig.tankDepthM,
      wifiConfig.tankCapacityL,
      wifiConfig.sensorPositionCm
    )
  ) {
    return false;
  }

  collectionIntervalSeconds =
    wifiConfig.collectionIntervalMinutes * 60UL;
  applyTankConfigurationToRuntime();

  return true;
}

bool saveWifiConfig(
  const String& ssid,
  const String& password,
  uint16_t collectionIntervalMinutes,
  const String& timeZone,
  uint8_t tankShape,
  float tankHeightM,
  float tankWidthM,
  float tankDepthM,
  float tankCapacityL,
  float sensorPositionCm
) {
  if (
    ssid.length() == 0 ||
    ssid.length() > 32 ||
    password.length() > 64 ||
    !isAllowedCollectionInterval(collectionIntervalMinutes) ||
    !isAllowedTimeZone(timeZone) ||
    !isTankConfigurationValid(
      tankShape,
      tankHeightM,
      tankWidthM,
      tankDepthM,
      tankCapacityL,
      sensorPositionCm
    )
  ) {
    return false;
  }

  memset(&wifiConfig, 0, sizeof(wifiConfig));
  wifiConfig.magic = WIFI_CONFIG_MAGIC;
  ssid.toCharArray(wifiConfig.ssid, sizeof(wifiConfig.ssid));
  password.toCharArray(
    wifiConfig.password,
    sizeof(wifiConfig.password)
  );
  wifiConfig.collectionIntervalMinutes =
    collectionIntervalMinutes;
  timeZone.toCharArray(
    wifiConfig.timeZone,
    sizeof(wifiConfig.timeZone)
  );
  wifiConfig.tankShape = tankShape;
  wifiConfig.tankHeightM = tankHeightM;
  wifiConfig.tankWidthM = tankWidthM;
  wifiConfig.tankDepthM = tankDepthM;
  wifiConfig.tankCapacityL = tankCapacityL;
  wifiConfig.sensorPositionCm = sensorPositionCm;
  wifiConfig.checksum = calculateWifiChecksum(wifiConfig);

  collectionIntervalSeconds = collectionIntervalMinutes * 60UL;
  applyTankConfigurationToRuntime();

  EEPROM.put(0, wifiConfig);
  return EEPROM.commit();
}

void clearStoredConfiguration() {
  WifiConfig emptyConfig;
  memset(&emptyConfig, 0, sizeof(emptyConfig));
  EEPROM.put(0, emptyConfig);
  EEPROM.commit();
}

void initializeDefaultWifiConfig() {
  memset(&wifiConfig, 0, sizeof(wifiConfig));
  wifiConfig.collectionIntervalMinutes = 5;
  strncpy(
    wifiConfig.timeZone,
    DEFAULT_TIME_ZONE,
    sizeof(wifiConfig.timeZone) - 1
  );
  wifiConfig.tankShape = TANK_CYLINDRICAL;
  wifiConfig.tankHeightM = DEFAULT_TANK_HEIGHT_M;
  wifiConfig.tankWidthM = DEFAULT_TANK_WIDTH_M;
  wifiConfig.tankDepthM = DEFAULT_TANK_DEPTH_M;
  wifiConfig.tankCapacityL = DEFAULT_TANK_CAPACITY_L;
  wifiConfig.sensorPositionCm = DEFAULT_SENSOR_POSITION_CM;
  collectionIntervalSeconds = HISTORY_SLOT_SECONDS;
  applyTankConfigurationToRuntime();
}

bool persistWifiConfig() {
  wifiConfig.checksum = calculateWifiChecksum(wifiConfig);
  EEPROM.put(0, wifiConfig);
  return EEPROM.commit();
}

uint32_t calculateRtcChecksum(const RtcState& state) {
  const uint8_t* bytes =
    reinterpret_cast<const uint8_t*>(&state);
  uint32_t checksum = 2166136261UL;

  for (size_t i = 8; i < sizeof(RtcState); i++) {
    checksum ^= bytes[i];
    checksum *= 16777619UL;
  }

  return checksum;
}

void saveRtcState() {
  rtcState.magic = RTC_MAGIC;
  rtcState.tankDepthCm = tankDepthCm;
  rtcState.sensorPositionCm = sensorPositionCm;
  rtcState.observedBottomDistanceCm = observedBottomDistanceCm;
  rtcState.energySavingMode = energySavingMode ? 1 : 0;
  rtcState.calibrationComplete = depthCalibrationActive ? 0 : 1;
  rtcState.hasObservedBottom = hasObservedBottom ? 1 : 0;
  rtcState.checksum = calculateRtcChecksum(rtcState);

  ESP.rtcUserMemoryWrite(
    0,
    reinterpret_cast<uint32_t*>(&rtcState),
    sizeof(RtcState)
  );
}

bool loadRtcState() {
  if (!ESP.rtcUserMemoryRead(
    0,
    reinterpret_cast<uint32_t*>(&rtcState),
    sizeof(RtcState)
  )) {
    return false;
  }

  if (
    rtcState.magic != RTC_MAGIC ||
    rtcState.checksum != calculateRtcChecksum(rtcState)
  ) {
    return false;
  }

  tankDepthCm = rtcState.tankDepthCm;
  sensorPositionCm = rtcState.sensorPositionCm;
  observedBottomDistanceCm = rtcState.observedBottomDistanceCm;
  energySavingMode = rtcState.energySavingMode == 1;
  hasObservedBottom = rtcState.hasObservedBottom == 1;
  depthCalibrationActive = rtcState.calibrationComplete == 0;

  return true;
}

void initializeRtcState() {
  memset(&rtcState, 0, sizeof(rtcState));
  memset(
    rtcState.historyLevels,
    INVALID_HISTORY_LEVEL,
    sizeof(rtcState.historyLevels)
  );
  rtcState.lastHistorySlot = 0;
  energySavingMode = false;
}

void clearRtcHistory() {
  memset(
    rtcState.historyLevels,
    INVALID_HISTORY_LEVEL,
    sizeof(rtcState.historyLevels)
  );
  rtcState.lastHistorySlot = 0;
  saveRtcState();
}

String formatIsoTimestamp(time_t timestamp) {
  if (timestamp <= 0) {
    return "";
  }

  struct tm timeInfo;
  gmtime_r(&timestamp, &timeInfo);

  char buffer[25];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &timeInfo);
  return String(buffer);
}

bool synchronizeNtp() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  unsigned long startMs = millis();
  while (
    time(nullptr) < 1700000000 &&
    millis() - startMs < NTP_TIMEOUT_MS
  ) {
    delay(200);
    yield();
  }

  ntpSynchronized = time(nullptr) >= 1700000000;
  return ntpSynchronized;
}

float readStableDistanceCm() {
  float sum = 0;
  int validCount = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long duration =
      pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);

    if (duration > 0) {
      float distanceCm = duration * 0.0343 / 2.0;

      if (distanceCm >= 2.0 && distanceCm <= 400.0) {
        sum += distanceCm;
        validCount++;
      }
    }

    delay(SAMPLE_PAUSE_MS);
    yield();
  }

  currentValidSamples = validCount;

  if (validCount < MIN_VALID_SAMPLES) {
    return -1;
  }

  return sum / validCount;
}

void startDepthCalibration() {
  hasObservedBottom = false;
  observedBottomDistanceCm = 0;

  depthCalibrationActive = true;
  depthCalibrationStartMs = millis();
}

void collectDepthCalibration(float stableDistanceCm) {
  if (
    !hasObservedBottom ||
    stableDistanceCm > observedBottomDistanceCm
  ) {
    observedBottomDistanceCm = stableDistanceCm;
    hasObservedBottom = true;
  }
}

void finishDepthCalibration() {
  if (hasObservedBottom) {
    float measuredSensorPositionCm =
      observedBottomDistanceCm - tankDepthCm;

    if (
      measuredSensorPositionCm >= 0 &&
      measuredSensorPositionCm <= tankFreeboardCm
    ) {
      sensorPositionCm = measuredSensorPositionCm;
      wifiConfig.sensorPositionCm = measuredSensorPositionCm;
      persistWifiConfig();
      clearRtcHistory();
    } else {
      Serial.println(
        "Afericao rejeitada: altura do sensor fora do espaco livre"
      );
    }
  }

  depthCalibrationActive = false;
  saveRtcState();

  Serial.print("Calibracao concluida | Fundo aferido: ");
  Serial.print(observedBottomDistanceCm, 2);
  Serial.print(" cm | Distancia sensor/nivel nominal: ");
  Serial.print(sensorPositionCm, 2);
  Serial.println(" cm");
}

void storeReading(float levelPercent, time_t timestamp) {
  if (timestamp <= 0) {
    return;
  }

  uint32_t slot =
    static_cast<uint32_t>(timestamp) / HISTORY_SLOT_SECONDS;

  if (rtcState.lastHistorySlot > 0) {
    uint32_t elapsedSlots = slot - rtcState.lastHistorySlot;

    if (elapsedSlots >= HISTORY_SIZE) {
      memset(
        rtcState.historyLevels,
        INVALID_HISTORY_LEVEL,
        sizeof(rtcState.historyLevels)
      );
    } else {
      for (
        uint32_t missingSlot = rtcState.lastHistorySlot + 1;
        missingSlot < slot;
        missingSlot++
      ) {
        rtcState.historyLevels[missingSlot % HISTORY_SIZE] =
          INVALID_HISTORY_LEVEL;
      }
    }
  }

  uint8_t compactLevel = static_cast<uint8_t>(round(
    constrain(levelPercent, 0.0, 100.0) * 254.0 / 100.0
  ));

  rtcState.historyLevels[slot % HISTORY_SIZE] = compactLevel;
  rtcState.lastHistorySlot = slot;
  saveRtcState();
}

void performReading() {
  float stableDistanceCm = readStableDistanceCm();

  if (stableDistanceCm < 0) {
    Serial.print("Afericao invalida: ");
    Serial.print(currentValidSamples);
    Serial.print("/");
    Serial.print(SAMPLE_COUNT);
    Serial.println(" pulsos validos");
    return;
  }

  // Este e o valor diretamente aferido: media dos pulsos validos.
  currentDistanceCm = stableDistanceCm;

  if (depthCalibrationActive) {
    collectDepthCalibration(stableDistanceCm);

    if (
      millis() - depthCalibrationStartMs <
      DEPTH_CALIBRATION_MS
    ) {
      currentWaterHeightCm = -1;
      currentLevelPercent = -1;
      currentLiters = -1;
      currentOverflowLiters = 0;
      currentMarginToTopCm = -1;
      currentOverflow = false;

      Serial.print("Aferindo altura do sensor | Distancia aferida: ");
      Serial.print(currentDistanceCm, 2);
      Serial.print(" cm | Maior media: ");
      Serial.print(observedBottomDistanceCm, 2);
      Serial.println(" cm");
      return;
    }

    finishDepthCalibration();
  }

  // Altura da agua = altura util nominal + distancia configurada entre
  // sensor e nivel de 100% - distancia medida pelo HC-SR04.
  float waterHeightCm =
    tankDepthCm -
    (currentDistanceCm - sensorPositionCm);

  waterHeightCm =
    constrain(waterHeightCm, 0.0, tankTotalHeightCm);

  float rawLevelPercent =
    waterHeightCm / tankDepthCm * 100.0;

  currentWaterHeightCm = waterHeightCm;
  currentLevelPercent =
    constrain(rawLevelPercent, 0.0, 100.0);
  currentLiters = tankBaseAreaM2 *
    (currentWaterHeightCm / 100.0) * 1000.0;
  currentOverflowLiters = max(
    0.0f,
    currentLiters - tankCapacityLiters
  );
  currentOverflow = rawLevelPercent > 100.0;
  currentMarginToTopCm = max(
    0.0f,
    tankTotalHeightCm - currentWaterHeightCm
  );

  currentCollectionTimestamp = time(nullptr);

  if (ntpSynchronized && currentCollectionTimestamp > 0) {
    uint32_t collectionSlot =
      static_cast<uint32_t>(currentCollectionTimestamp) /
      HISTORY_SLOT_SECONDS;

    if (collectionSlot != rtcState.lastHistorySlot) {
      storeReading(
        currentLevelPercent,
        currentCollectionTimestamp
      );
    }
  }

  Serial.print("Distancia aferida (media de ");
  Serial.print(currentValidSamples);
  Serial.print("): ");
  Serial.print(currentDistanceCm, 2);
  Serial.print(" cm | Altura util nominal: ");
  Serial.print(tankDepthCm, 2);
  Serial.print(" cm | Posicao do sensor: ");
  Serial.print(sensorPositionCm, 2);
  Serial.print(" cm | Altura da agua: ");
  Serial.print(currentWaterHeightCm, 2);
  Serial.print(" cm | Nivel: ");
  Serial.print(currentLevelPercent, 1);
  Serial.print("% | Volume: ");
  Serial.print(currentLiters, 0);
  Serial.print(" L | Transbordamento: ");
  Serial.print(currentOverflowLiters, 0);
  Serial.println(" L");
}

void resetCalibration() {
  startDepthCalibration();

  currentWaterHeightCm = -1;
  currentLevelPercent = -1;
  currentLiters = -1;
  currentOverflowLiters = 0;
  currentMarginToTopCm = -1;
  currentOverflow = false;

  Serial.println("Afericao do sensor iniciada por 30 segundos");
  Serial.println("O reservatorio deve estar vazio para aferir o fundo");
}

const char CONFIG_HTML_PAGE[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="pt-BR">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Configuracao de Wi-Fi</title>
  <style>
    * { box-sizing: border-box; }
    body {
      margin: 0;
      padding: 20px;
      font-family: Arial, sans-serif;
      background: #f4f6f8;
      color: #1f2933;
    }
    .panel {
      max-width: 460px;
      margin: 40px auto;
      padding: 24px;
      border: 1px solid #d8dee4;
      border-radius: 8px;
      background: #fff;
    }
    h1 { margin-top: 0; font-size: 24px; }
    p { color: #637083; line-height: 1.5; }
    label { display: block; margin-top: 16px; font-weight: bold; }
    input, select {
      width: 100%;
      margin-top: 6px;
      padding: 10px;
      border: 1px solid #b8c2cc;
      border-radius: 5px;
      font-size: 16px;
    }
    button {
      width: 100%;
      margin-top: 20px;
      padding: 11px;
      border: 1px solid #087f5b;
      border-radius: 5px;
      background: #087f5b;
      color: #fff;
      font-size: 16px;
      cursor: pointer;
    }
    .field-row {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
    }
    .validation {
      margin-top: 16px;
      padding: 12px;
      border-radius: 5px;
      background: #f4f6f8;
      line-height: 1.5;
    }
    .validation.error {
      background: #fef2f2;
      color: #b42318;
    }
    [hidden] { display: none !important; }
  </style>
</head>
<body>
  <main class="panel">
    <h1>Configurar rede Wi-Fi</h1>
    <p>
      Informe a rede que sera usada pelo sensor. A configuracao sera
      armazenada na memoria. O ponto de acesso permanecera ativo ate a
      confirmacao do endereco IP obtido.
    </p>
    <form method="post" action="/apply-configuration">
      <label for="ssid">SSID</label>
      <input id="ssid" name="ssid" maxlength="32" required>

      <label for="password">Senha</label>
      <input
        id="password"
        name="password"
        type="password"
        maxlength="64"
        autocomplete="new-password"
        placeholder="Deixe em branco para manter a senha atual"
      >

      <label for="collectionInterval">Intervalo de coleta</label>
      <select id="collectionInterval" name="collectionInterval">
        <option value="5">5 minutos</option>
        <option value="10">10 minutos</option>
        <option value="15">15 minutos</option>
        <option value="20">20 minutos</option>
        <option value="30">30 minutos</option>
      </select>

      <label for="timeZone">Fuso horario</label>
      <select id="timeZone" name="timeZone">
        <option value="America/Sao_Paulo">Brasilia</option>
        <option value="America/Noronha">Fernando de Noronha</option>
        <option value="America/Manaus">Manaus</option>
        <option value="America/Rio_Branco">Rio Branco</option>
        <option value="UTC">UTC</option>
      </select>

      <h2>Reservatorio e sensor</h2>

      <label for="tankShape">Formato</label>
      <select id="tankShape" name="tankShape">
        <option value="cylindrical">Cilindrico</option>
        <option value="rectangular">Retangular</option>
      </select>

      <div class="field-row">
        <div>
          <label for="tankHeight">Altura total (m)</label>
          <input
            id="tankHeight"
            name="tankHeight"
            type="number"
            min="0.001"
            step="0.001"
            required
          >
        </div>
        <div>
          <label for="tankWidth">Largura / diametro (m)</label>
          <input
            id="tankWidth"
            name="tankWidth"
            type="number"
            min="0.001"
            step="0.001"
            required
          >
        </div>
      </div>

      <div id="tankDepthGroup" hidden>
        <label for="tankDepth">Profundidade (m)</label>
        <input
          id="tankDepth"
          name="tankDepth"
          type="number"
          min="0.001"
          step="0.001"
          required
        >
      </div>

      <div class="field-row">
        <div>
          <label for="tankCapacity">Capacidade nominal (L)</label>
          <input
            id="tankCapacity"
            name="tankCapacity"
            type="number"
            min="1"
            step="1"
            required
          >
        </div>
        <div>
          <label for="sensorPosition">Sensor acima de 100% (cm)</label>
          <input
            id="sensorPosition"
            name="sensorPosition"
            type="number"
            min="0"
            step="0.1"
            required
          >
        </div>
      </div>

      <div id="tankValidation" class="validation"></div>

      <button type="submit">Aplicar</button>
    </form>
  </main>
  <script>
    const form = document.querySelector("form");
    const tankShape = document.getElementById("tankShape");
    const tankHeight = document.getElementById("tankHeight");
    const tankWidth = document.getElementById("tankWidth");
    const tankDepth = document.getElementById("tankDepth");
    const tankDepthGroup = document.getElementById("tankDepthGroup");
    const tankCapacity = document.getElementById("tankCapacity");
    const sensorPosition = document.getElementById("sensorPosition");
    const tankValidation = document.getElementById("tankValidation");

    function validateTankConfiguration() {
      const shape = tankShape.value;
      const heightM = Number(tankHeight.value);
      const widthM = Number(tankWidth.value);
      const depthM = Number(tankDepth.value);
      const capacityL = Number(tankCapacity.value);
      const sensorCm = Number(sensorPosition.value);
      const isRectangular = shape === "rectangular";

      tankDepthGroup.hidden = !isRectangular;
      tankDepth.required = isRectangular;
      sensorPosition.setCustomValidity("");
      tankHeight.setCustomValidity("");

      const baseAreaM2 = shape === "cylindrical"
        ? Math.PI * Math.pow(widthM / 2, 2)
        : widthM * depthM;
      const requiredHeightM = (capacityL / 1000) / baseAreaM2;
      const freeboardM = heightM - requiredHeightM;

      if (
        !Number.isFinite(baseAreaM2) ||
        !Number.isFinite(requiredHeightM) ||
        baseAreaM2 <= 0 ||
        requiredHeightM <= 0
      ) {
        tankValidation.className = "validation error";
        tankValidation.textContent =
          "Informe medidas e capacidade validas.";
        return false;
      }

      if (requiredHeightM > heightM) {
        tankHeight.setCustomValidity("Altura total insuficiente");
        tankValidation.className = "validation error";
        tankValidation.textContent =
          "Dados inconsistentes: a capacidade exige altura minima de " +
          requiredHeightM.toFixed(3) + " m.";
        return false;
      }

      const maximumSensorCm = Math.max(0, freeboardM * 100);

      if (sensorCm > maximumSensorCm) {
        sensorPosition.setCustomValidity("Altura do sensor inviavel");
        tankValidation.className = "validation error";
        tankValidation.textContent =
          "Altura do sensor inviavel. Reduza para no maximo " +
          maximumSensorCm.toFixed(1) + " cm.";
        return false;
      }

      tankValidation.className = "validation";
      tankValidation.textContent =
        "Altura util para a capacidade: " +
        requiredHeightM.toFixed(3) + " m. Espaco livre: " +
        maximumSensorCm.toFixed(1) + " cm.";
      return true;
    }

    fetch("/configuration-data", { cache: "no-store" })
      .then(response => response.json())
      .then(data => {
        document.getElementById("ssid").value = data.ssid || "";
        document.getElementById("collectionInterval").value =
          String(data.collectionIntervalMinutes);
        document.getElementById("timeZone").value = data.timeZone;
        tankShape.value = data.tankShape;
        tankHeight.value = data.tankHeightM;
        tankWidth.value = data.tankWidthM;
        tankDepth.value = data.tankDepthM;
        tankCapacity.value = data.tankCapacityL;
        sensorPosition.value = data.sensorPositionCm;
        validateTankConfiguration();
      })
      .catch(error => console.error(
        "Erro ao carregar configuracao:",
        error
      ));

    [
      tankShape,
      tankHeight,
      tankWidth,
      tankDepth,
      tankCapacity,
      sensorPosition
    ].forEach(element => {
      element.addEventListener("input", validateTankConfiguration);
      element.addEventListener("change", validateTankConfiguration);
    });

    form.addEventListener("submit", event => {
      if (!validateTankConfiguration()) {
        event.preventDefault();
        form.reportValidity();
      }
    });
  </script>
</body>
</html>
)rawliteral";

const char CONFIG_CONNECTION_PAGE[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="pt-BR">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Conectando a rede</title>
  <style>
    * { box-sizing: border-box; }
    body {
      margin: 0;
      padding: 20px;
      font-family: Arial, sans-serif;
      background: #f4f6f8;
      color: #1f2933;
    }
    .panel {
      max-width: 520px;
      margin: 40px auto;
      padding: 24px;
      border: 1px solid #d8dee4;
      border-radius: 8px;
      background: #fff;
    }
    h1 { margin-top: 0; font-size: 24px; }
    p { line-height: 1.6; }
    .hint { color: #637083; }
    button {
      width: 100%;
      margin-top: 16px;
      padding: 11px;
      border: 1px solid #087f5b;
      border-radius: 5px;
      background: #087f5b;
      color: #fff;
      font-size: 16px;
      cursor: pointer;
    }
    button:disabled { cursor: wait; opacity: 0.6; }
  </style>
</head>
<body>
  <main class="panel">
    <h1 id="title">Conectando a rede...</h1>
    <div id="status">
      <p>Aguarde enquanto o dispositivo solicita um endereco IP.</p>
      <p class="hint">
        Se esta pagina parar de atualizar, conecte-se a rede aberta
        <strong>quantagua</strong> e acesse
        <strong>http://192.168.4.1</strong>.
      </p>
    </div>
    <button id="finishButton" type="button" hidden>
      Finalizar configuracao
    </button>
  </main>
  <script>
    const title = document.getElementById("title");
    const status = document.getElementById("status");
    const finishButton = document.getElementById("finishButton");

    function escapeHtml(value) {
      const element = document.createElement("div");
      element.textContent = value;
      return element.innerHTML;
    }

    async function loadConnectionStatus() {
      try {
        const response = await fetch("/connection-status", {
          cache: "no-store"
        });
        const data = await response.json();

        if (data.status === "connected") {
          const network = escapeHtml(data.ssid);
          const ip = escapeHtml(data.ip);
          title.textContent = "Rede conectada";
          status.innerHTML =
            "<p><strong>Rede conectada:</strong> " + network + "</p>" +
            "<p><strong>IP obtido:</strong> " + ip + "</p>" +
            "<p>Conecte-se a rede <strong>" + network +
            "</strong> e acesse <strong>http://" + ip + "</strong>.</p>";
          finishButton.hidden = false;
          return;
        }

        if (data.status === "failed") {
          title.textContent = "Nao foi possivel conectar";
          status.innerHTML =
            "<p>Verifique o SSID e a senha informados.</p>" +
            "<p><a href=\"/configuration-page\">" +
            "Voltar para configuracao</a></p>";
        }
      } catch (error) {
        console.error("Erro ao consultar conexao:", error);
      }
    }

    finishButton.addEventListener("click", async () => {
      finishButton.disabled = true;
      finishButton.textContent = "Finalizando...";

      try {
        const response = await fetch("/finish-configuration", {
          method: "POST",
          cache: "no-store"
        });

        if (!response.ok) {
          throw new Error("Falha ao finalizar");
        }

        const data = await response.json();
        let remainingSeconds = 10;

        status.innerHTML =
          "<p>O ponto de acesso <strong>quantagua</strong> sera " +
          "desligado.</p><p id=\"redirectStatus\"></p>";

        const redirectStatus =
          document.getElementById("redirectStatus");

        function updateRedirectStatus() {
          redirectStatus.textContent =
            "Aguardando a reconexao a rede configurada. " +
            "Acesso ao IP " + data.ip + " em " +
            remainingSeconds + " segundos.";
        }

        updateRedirectStatus();

        const countdown = setInterval(() => {
          remainingSeconds--;
          updateRedirectStatus();

          if (remainingSeconds <= 0) {
            clearInterval(countdown);
          }
        }, 1000);

        setTimeout(() => {
          window.location.replace("http://" + data.ip);
        }, 10000);
      } catch (error) {
        finishButton.disabled = false;
        finishButton.textContent = "Finalizar configuracao";
      }
    });

    loadConnectionStatus();
    setInterval(loadConnectionStatus, 1000);
  </script>
</body>
</html>
)rawliteral";

const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="pt-BR">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Sensor de Nivel</title>

  <style>
    * { box-sizing: border-box; }

    body {
      margin: 0;
      padding: 20px;
      font-family: Arial, sans-serif;
      background: #f4f6f8;
      color: #1f2933;
    }

    .header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      margin-bottom: 18px;
    }

    h1 {
      margin: 0;
      font-size: 24px;
    }

    button {
      padding: 9px 14px;
      border: 1px solid #b42318;
      border-radius: 6px;
      background: #fff;
      color: #b42318;
      font-size: 14px;
      cursor: pointer;
    }

    button:hover { background: #fef3f2; }
    button:disabled { cursor: wait; opacity: 0.6; }

    .header-actions {
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
    }

    #powerModeButton {
      border-color: #087f5b;
      color: #087f5b;
    }

    #powerModeButton:hover { background: #ecfdf5; }

    #configurationButton {
      border-color: #2563eb;
      color: #2563eb;
    }

    #configurationButton:hover { background: #eff6ff; }

    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
      gap: 12px;
      margin-bottom: 20px;
    }

    .card {
      padding: 14px;
      background: #fff;
      border: 1px solid #d8dee4;
      border-radius: 6px;
    }

    .label {
      color: #637083;
      font-size: 13px;
    }

    .value {
      margin-top: 6px;
      font-size: 26px;
      font-weight: bold;
    }

    .status {
      min-height: 16px;
      margin-top: 5px;
      color: #637083;
      font-size: 12px;
    }

    .chart-box {
      padding: 14px;
      overflow-x: auto;
      background: #fff;
      border: 1px solid #d8dee4;
      border-radius: 6px;
    }

    .chart-content {
      position: relative;
      width: 100%;
      min-width: 0;
    }

    .chart-title {
      margin: 0 0 10px;
      font-size: 16px;
    }

    canvas {
      display: block;
      width: 100%;
      height: 380px;
      cursor: crosshair;
      touch-action: none;
      user-select: none;
    }

    .chart-tooltip {
      position: absolute;
      display: none;
      min-width: 120px;
      padding: 8px 10px;
      border: 1px solid #c8d0d8;
      border-radius: 5px;
      background: rgba(255, 255, 255, 0.96);
      box-shadow: 0 2px 8px rgba(31, 41, 51, 0.16);
      color: #1f2933;
      font-size: 12px;
      line-height: 1.5;
      pointer-events: none;
      white-space: nowrap;
    }

    @media (max-width: 640px) {
      body { padding: 12px; }
      .header { align-items: flex-start; flex-direction: column; }
    }
  </style>
</head>

<body>
  <div class="header">
    <h1>Sensor de Nivel de Agua</h1>
    <div class="header-actions">
      <button id="configurationButton" type="button">
        Configuracao
      </button>
      <button id="powerModeButton" type="button">
        Ativar economia de energia
      </button>
      <button id="resetButton" type="button">
        Aferir sensor
      </button>
    </div>
  </div>

  <div class="grid">
    <div class="card">
      <div class="label">Volume estimado</div>
      <div class="value" id="volume">-- L</div>
    </div>

    <div class="card">
      <div class="label">Nivel</div>
      <div class="value" id="level">--%</div>
    </div>

    <div class="card">
      <div class="label">Altura estimada da agua</div>
      <div class="value" id="waterHeight">-- cm</div>
    </div>

    <div class="card">
      <div class="label">Distancia do sensor</div>
      <div class="value" id="distance">-- cm</div>
      <div class="status" id="sampleStatus"></div>
    </div>

    <div class="card">
      <div class="label">Altura util nominal</div>
      <div class="value" id="tankDepth">-- cm</div>
      <div class="status" id="depthStatus"></div>
    </div>

    <div class="card">
      <div class="label">Sensor acima de 100%</div>
      <div class="value" id="sensorPosition">-- cm</div>
    </div>

    <div class="card">
      <div class="label">Transbordamento</div>
      <div class="value" id="overflow">Nao</div>
      <div class="status" id="overflowStatus"></div>
    </div>

    <div class="card">
      <div class="label">Margem ate o topo fisico</div>
      <div class="value" id="marginToTop">-- cm</div>
    </div>

    <div class="card">
      <div class="label">RSSI Wi-Fi</div>
      <div class="value" id="rssi">-- dBm</div>
    </div>

    <div class="card">
      <div class="label">Ultima coleta</div>
      <div class="value" id="collectionTime">--</div>
      <div class="status" id="powerModeStatus"></div>
    </div>
  </div>

  <div class="chart-box">
    <h2 class="chart-title">Historico das ultimas 24 horas</h2>
    <div class="chart-content">
      <canvas id="chart" height="380"></canvas>
      <div class="chart-tooltip" id="chartTooltip"></div>
    </div>
  </div>

  <script>
    const WINDOW_SECONDS = 24 * 60 * 60;
    const CHART_LEFT = 55;
    const CHART_RIGHT = 25;
    const CHART_TOP = 25;
    const CHART_BOTTOM = 65;
    const MIN_VIEW_SECONDS = 30 * 60;
    const MAX_VIEW_SECONDS = WINDOW_SECONDS;
    const ZOOM_FACTOR = 1.2;
    const TAP_MAX_DISTANCE_PX = 10;
    const TAP_MAX_DURATION_MS = 350;
    const TIME_INTERVALS_SECONDS = [
      5 * 60,
      10 * 60,
      15 * 60,
      30 * 60,
      60 * 60,
      2 * 60 * 60,
      3 * 60 * 60,
      6 * 60 * 60
    ];
    let chartPoints = [];
    let windowStartEpoch = 0;
    let configuredTimeZone = "America/Sao_Paulo";
    let configuredTankCapacityLiters = 5000;
    let selectedChartPoint = null;
    let viewStartSeconds = 0;
    let viewDurationSeconds = WINDOW_SECONDS;
    const activePointers = new Map();
    let pointerGesture = null;

    function formatTime(seconds) {
      if (!windowStartEpoch) {
        return "--:--";
      }

      return new Date(
        (windowStartEpoch + seconds) * 1000
      ).toLocaleTimeString("pt-BR", {
        hour: "2-digit",
        minute: "2-digit",
        timeZone: configuredTimeZone
      });
    }

    function formatElapsedTime(seconds) {
      const hours = Math.floor(seconds / 3600);
      const minutes = Math.floor((seconds % 3600) / 60);
      const remainingSeconds = seconds % 60;

      return String(hours).padStart(2, "0") + ":" +
        String(minutes).padStart(2, "0") + ":" +
        String(remainingSeconds).padStart(2, "0");
    }

    function estimateLitersFromLevel(levelPercent) {
      const ratio = Math.max(0, Math.min(1, levelPercent / 100));
      return configuredTankCapacityLiters * ratio;
    }

    function setValue(id, value, suffix, decimals) {
      document.getElementById(id).textContent =
        value >= 0
          ? value.toFixed(decimals) + suffix
          : "--" + suffix;
    }

    function updateSummary(data) {
      const current = data.current;
      const parameters = data.parameters;
      configuredTimeZone =
        parameters.timeZone || "America/Sao_Paulo";
      configuredTankCapacityLiters = parameters.tankCapacityL;

      document.getElementById("volume").textContent =
        current.liters >= 0
          ? Math.round(current.liters).toLocaleString("pt-BR") + " L"
          : "-- L";

      setValue("level", current.levelPercent, "%", 1);
      setValue("waterHeight", current.waterHeightCm, " cm", 1);
      setValue("distance", current.distanceCm, " cm", 2);
      setValue("tankDepth", parameters.tankDepthCm, " cm", 2);
      setValue(
        "sensorPosition",
        parameters.sensorPositionCm,
        " cm",
        2
      );
      setValue(
        "marginToTop",
        current.marginToTopCm,
        " cm",
        1
      );

      document.getElementById("overflow").textContent =
        current.overflow ? "SIM" : "Nao";
      document.getElementById("overflowStatus").textContent =
        current.overflow
          ? Math.round(current.overflowLiters).toLocaleString("pt-BR") +
            " L acima da capacidade nominal"
          : "Dentro da capacidade nominal";

      document.getElementById("rssi").textContent =
        data.rssi + " dBm";

      document.getElementById("collectionTime").textContent =
        current.timestampIso
          ? new Date(current.timestampIso).toLocaleString("pt-BR", {
              timeZone: configuredTimeZone
            })
          : "--";

      const energySaving = parameters.energySavingMode;
      document.getElementById("powerModeButton").textContent =
        energySaving
          ? "Usar modo continuo"
          : "Ativar economia de energia";
      document.getElementById("powerModeStatus").textContent =
        energySaving
          ? "Sono profundo entre coletas de " +
            parameters.collectionIntervalSeconds / 60 + " minutos"
          : "Funcionamento continuo";

      document.getElementById("sampleStatus").textContent =
        "Media de " + current.validSamples + "/5 pulsos";

      document.getElementById("depthStatus").textContent =
        parameters.calibrationActive
          ? "Aferindo sensor por 30 segundos"
          : parameters.hasObservedBottom
          ? "Fundo aferido em " +
            parameters.observedBottomDistanceCm.toFixed(2) + " cm"
          : "Calculada pelas medidas configuradas";
    }

    function clamp(value, minimum, maximum) {
      return Math.max(minimum, Math.min(maximum, value));
    }

    function clampViewStart(start, duration) {
      return clamp(start, 0, WINDOW_SECONDS - duration);
    }

    function chartXToElapsed(x, graphWidth) {
      const ratio = clamp(
        (x - CHART_LEFT) / graphWidth,
        0,
        1
      );

      return viewStartSeconds + ratio * viewDurationSeconds;
    }

    function elapsedToChartX(elapsed, graphWidth) {
      return CHART_LEFT +
        graphWidth *
        (elapsed - viewStartSeconds) /
        viewDurationSeconds;
    }

    function chooseTimeInterval(graphWidth) {
      const targetInterval =
        viewDurationSeconds / Math.max(1, graphWidth / 90);

      return TIME_INTERVALS_SECONDS.find(
        interval => interval >= targetInterval
      ) || TIME_INTERVALS_SECONDS[TIME_INTERVALS_SECONDS.length - 1];
    }

    function drawChart(points) {
      const canvas = document.getElementById("chart");
      const ctx = canvas.getContext("2d");

      if (points) {
        windowStartEpoch = points.length > 0
          ? points[0].timestampEpoch
          : Math.floor(Date.now() / 1000);
        chartPoints = points.map(point => ({
          timestampEpoch: point.timestampEpoch,
          elapsed: point.timestampEpoch - windowStartEpoch,
          level: point.level
        }));
      }

      const left = CHART_LEFT;
      const right = CHART_RIGHT;
      const top = CHART_TOP;
      const bottom = CHART_BOTTOM;
      const graphWidth = canvas.width - left - right;
      const graphHeight = canvas.height - top - bottom;

      if (graphWidth <= 0 || graphHeight <= 0) {
        return;
      }

      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.font = "12px Arial";

      for (let percent = 0; percent <= 100; percent += 25) {
        const y = top + graphHeight * (1 - percent / 100);

        ctx.strokeStyle = "#d8dee4";
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(left, y);
        ctx.lineTo(canvas.width - right, y);
        ctx.stroke();

        ctx.fillStyle = "#637083";
        ctx.textAlign = "right";
        ctx.fillText(percent + "%", left - 8, y + 4);
      }

      const timeInterval = chooseTimeInterval(graphWidth);
      const firstTimeMark =
        Math.ceil(viewStartSeconds / timeInterval) * timeInterval;
      const viewEndSeconds = viewStartSeconds + viewDurationSeconds;

      for (
        let elapsed = firstTimeMark;
        elapsed <= viewEndSeconds;
        elapsed += timeInterval
      ) {
        const x = elapsedToChartX(elapsed, graphWidth);

        ctx.strokeStyle = "#edf0f2";
        ctx.beginPath();
        ctx.moveTo(x, top);
        ctx.lineTo(x, top + graphHeight);
        ctx.stroke();

        ctx.fillStyle = "#637083";
        ctx.textAlign = "center";
        ctx.fillText(
          formatTime(elapsed),
          x,
          top + graphHeight + 25
        );
      }

      if (chartPoints.length === 0) {
        return;
      }

      ctx.strokeStyle = "#087f5b";
      ctx.lineWidth = 2;
      ctx.beginPath();

      let lineStarted = false;

      chartPoints.forEach(point => {
        if (
          point.elapsed < viewStartSeconds ||
          point.elapsed > viewEndSeconds
        ) {
          return;
        }

        const x = elapsedToChartX(point.elapsed, graphWidth);

        const y =
          top + graphHeight * (1 - point.level / 100);

        if (!lineStarted) {
          ctx.moveTo(x, y);
          lineStarted = true;
        } else {
          ctx.lineTo(x, y);
        }
      });

      if (lineStarted) {
        ctx.stroke();
      }

      if (
        selectedChartPoint &&
        selectedChartPoint.elapsed >= viewStartSeconds &&
        selectedChartPoint.elapsed <= viewEndSeconds
      ) {
        const x = elapsedToChartX(
          selectedChartPoint.elapsed,
          graphWidth
        );
        const y =
          CHART_TOP +
          graphHeight * (1 - selectedChartPoint.level / 100);

        ctx.fillStyle = "#ffffff";
        ctx.strokeStyle = "#087f5b";
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.arc(x, y, 6, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
      }
    }

    function resizeChartCanvas() {
      const canvas = document.getElementById("chart");
      const width = Math.max(
        280,
        Math.floor(canvas.parentElement.clientWidth)
      );

      if (canvas.width !== width) {
        canvas.width = width;
      }

      drawChart();
    }

    function hideChartTooltip() {
      if (selectedChartPoint) {
        selectedChartPoint = null;
        drawChart();
      }

      document.getElementById("chartTooltip").style.display = "none";
    }

    function findNearestChartPoint(elapsed) {
      if (chartPoints.length === 0) {
        return null;
      }

      let nearest = chartPoints[0];
      let nearestDistance = Math.abs(nearest.elapsed - elapsed);

      for (let i = 1; i < chartPoints.length; i++) {
        const distance = Math.abs(chartPoints[i].elapsed - elapsed);

        if (distance >= nearestDistance) {
          break;
        }

        nearest = chartPoints[i];
        nearestDistance = distance;
      }

      return nearest;
    }

    function getCanvasX(event) {
      const canvas = document.getElementById("chart");
      const rect = canvas.getBoundingClientRect();

      return (event.clientX - rect.left) *
        canvas.width / rect.width;
    }

    function getCanvasY(event) {
      const canvas = document.getElementById("chart");
      const rect = canvas.getBoundingClientRect();

      return (event.clientY - rect.top) *
        canvas.height / rect.height;
    }

    function showChartTooltipAt(event, requireLineProximity) {
      if (pointerGesture) {
        hideChartTooltip();
        return;
      }

      const canvas = document.getElementById("chart");
      const tooltip = document.getElementById("chartTooltip");
      const graphWidth = canvas.width - CHART_LEFT - CHART_RIGHT;
      const graphHeight = canvas.height - CHART_TOP - CHART_BOTTOM;
      const x = getCanvasX(event);
      const y = getCanvasY(event);

      if (
        x < CHART_LEFT ||
        x > canvas.width - CHART_RIGHT ||
        y < CHART_TOP ||
        y > CHART_TOP + graphHeight
      ) {
        hideChartTooltip();
        return;
      }

      const elapsed = chartXToElapsed(x, graphWidth);
      const point = findNearestChartPoint(elapsed);

      if (
        !point ||
        point.elapsed < viewStartSeconds ||
        point.elapsed > viewStartSeconds + viewDurationSeconds
      ) {
        hideChartTooltip();
        return;
      }

      const pointX = elapsedToChartX(point.elapsed, graphWidth);
      const pointY =
        CHART_TOP + graphHeight * (1 - point.level / 100);

      if (
        requireLineProximity &&
        (
          Math.abs(x - pointX) > 14 ||
          Math.abs(y - pointY) > 12
        )
      ) {
        hideChartTooltip();
        return;
      }

      selectedChartPoint = point;
      drawChart();

      const estimatedLiters = estimateLitersFromLevel(point.level);

      tooltip.innerHTML =
        "<strong>Coleta:</strong> " +
        new Date(point.timestampEpoch * 1000).toLocaleString("pt-BR", {
          timeZone: configuredTimeZone
        }) +
        "<br><strong>Nivel:</strong> " +
        point.level.toFixed(2) + "%" +
        "<br><strong>Volume estimado:</strong> " +
        Math.round(estimatedLiters).toLocaleString("pt-BR") + " L";

      tooltip.style.display = "block";

      const tooltipWidth = tooltip.offsetWidth;
      const tooltipHeight = tooltip.offsetHeight;
      const left =
        pointX + tooltipWidth + 18 < canvas.width
          ? pointX + 12
          : pointX - tooltipWidth - 12;
      const top = Math.max(
        0,
        Math.min(
          canvas.height - tooltipHeight,
          pointY - tooltipHeight - 12
        )
      );

      tooltip.style.left = left + "px";
      tooltip.style.top = top + "px";
    }

    function showChartTooltip(event) {
      showChartTooltipAt(event, true);
    }

    function setChartView(duration, anchorElapsed, anchorRatio) {
      const newDuration = clamp(
        duration,
        MIN_VIEW_SECONDS,
        MAX_VIEW_SECONDS
      );

      viewDurationSeconds = newDuration;
      viewStartSeconds = clampViewStart(
        anchorElapsed - newDuration * anchorRatio,
        newDuration
      );

      hideChartTooltip();
      drawChart();
    }

    function zoomChart(event) {
      event.preventDefault();

      const canvas = document.getElementById("chart");
      const graphWidth = canvas.width - CHART_LEFT - CHART_RIGHT;
      const x = getCanvasX(event);
      const anchorRatio = clamp(
        (x - CHART_LEFT) / graphWidth,
        0,
        1
      );
      const anchorElapsed = chartXToElapsed(x, graphWidth);
      const duration =
        event.deltaY < 0
          ? viewDurationSeconds / ZOOM_FACTOR
          : viewDurationSeconds * ZOOM_FACTOR;

      setChartView(duration, anchorElapsed, anchorRatio);
    }

    function pointerCenterX(first, second) {
      const canvas = document.getElementById("chart");
      const rect = canvas.getBoundingClientRect();
      const clientX = (first.clientX + second.clientX) / 2;

      return (clientX - rect.left) * canvas.width / rect.width;
    }

    function pointerDistance(first, second) {
      return Math.hypot(
        second.clientX - first.clientX,
        second.clientY - first.clientY
      );
    }

    function startPointerGesture() {
      const pointers = Array.from(activePointers.values());

      hideChartTooltip();

      if (pointers.length === 1) {
        pointerGesture = {
          type: "pan",
          pointerId: pointers[0].pointerId,
          startX: getCanvasX(pointers[0]),
          startClientX: pointers[0].clientX,
          startClientY: pointers[0].clientY,
          startTime: Date.now(),
          moved: false,
          viewStart: viewStartSeconds
        };
        return;
      }

      if (pointers.length >= 2) {
        const canvas = document.getElementById("chart");
        const graphWidth = canvas.width - CHART_LEFT - CHART_RIGHT;
        const centerX = pointerCenterX(pointers[0], pointers[1]);
        const centerRatio = clamp(
          (centerX - CHART_LEFT) / graphWidth,
          0,
          1
        );

        pointerGesture = {
          type: "pinch",
          initialDistance: pointerDistance(pointers[0], pointers[1]),
          initialDuration: viewDurationSeconds,
          anchorElapsed:
            viewStartSeconds + centerRatio * viewDurationSeconds
        };
      }
    }

    function handlePointerDown(event) {
      const canvas = document.getElementById("chart");

      canvas.setPointerCapture(event.pointerId);
      activePointers.set(event.pointerId, event);
      startPointerGesture();
    }

    function handlePointerMove(event) {
      if (!activePointers.has(event.pointerId)) {
        if (event.pointerType === "mouse") {
          showChartTooltip(event);
        }
        return;
      }

      activePointers.set(event.pointerId, event);

      if (!pointerGesture) {
        return;
      }

      const canvas = document.getElementById("chart");
      const graphWidth = canvas.width - CHART_LEFT - CHART_RIGHT;
      const pointers = Array.from(activePointers.values());

      if (pointerGesture.type === "pan" && pointers.length === 1) {
        const movement = Math.hypot(
          event.clientX - pointerGesture.startClientX,
          event.clientY - pointerGesture.startClientY
        );

        if (movement > TAP_MAX_DISTANCE_PX) {
          pointerGesture.moved = true;
        }

        const deltaX =
          getCanvasX(event) - pointerGesture.startX;
        const elapsedDelta =
          deltaX * viewDurationSeconds / graphWidth;

        viewStartSeconds = clampViewStart(
          pointerGesture.viewStart - elapsedDelta,
          viewDurationSeconds
        );
        drawChart();
        return;
      }

      if (pointerGesture.type === "pinch" && pointers.length >= 2) {
        const distance = Math.max(
          1,
          pointerDistance(pointers[0], pointers[1])
        );
        const centerX = pointerCenterX(pointers[0], pointers[1]);
        const centerRatio = clamp(
          (centerX - CHART_LEFT) / graphWidth,
          0,
          1
        );
        const duration = clamp(
          pointerGesture.initialDuration *
            pointerGesture.initialDistance / distance,
          MIN_VIEW_SECONDS,
          MAX_VIEW_SECONDS
        );

        viewDurationSeconds = duration;
        viewStartSeconds = clampViewStart(
          pointerGesture.anchorElapsed - duration * centerRatio,
          duration
        );
        drawChart();
      }
    }

    function handlePointerEnd(event) {
      const completedGesture = pointerGesture;
      const isTouchTap =
        completedGesture &&
        completedGesture.type === "pan" &&
        completedGesture.pointerId === event.pointerId &&
        event.pointerType !== "mouse" &&
        !completedGesture.moved &&
        Date.now() - completedGesture.startTime <=
          TAP_MAX_DURATION_MS;

      activePointers.delete(event.pointerId);

      if (activePointers.size > 0) {
        startPointerGesture();
      } else {
        pointerGesture = null;
      }

      if (isTouchTap) {
        showChartTooltipAt(event, false);
      }
    }

    async function loadData() {
      try {
        const response = await fetch("/data", {
          cache: "no-store"
        });

        if (!response.ok) {
          throw new Error("Resposta HTTP " + response.status);
        }

        const data = await response.json();
        updateSummary(data);
        drawChart(data.history);
      } catch (error) {
        console.error("Erro ao carregar dados:", error);
      }
    }

    async function resetCalibration() {
      const confirmed = window.confirm(
        "O reservatorio esta vazio? A afericao medira o fundo " +
        "por 30 segundos e atualizara a altura do sensor."
      );

      if (!confirmed) {
        return;
      }

      const button = document.getElementById("resetButton");
      button.disabled = true;
      button.textContent = "Aferindo...";

      try {
        const response = await fetch("/reset", {
          method: "POST",
          cache: "no-store"
        });

        if (!response.ok) {
          throw new Error("Falha ao aferir");
        }

        await loadData();
      } catch (error) {
        window.alert("Nao foi possivel iniciar a afericao.");
      } finally {
        button.disabled = false;
        button.textContent = "Aferir sensor";
      }
    }

    async function togglePowerMode() {
      const button = document.getElementById("powerModeButton");
      button.disabled = true;

      try {
        const response = await fetch("/power-mode", {
          method: "POST",
          cache: "no-store"
        });

        if (!response.ok) {
          throw new Error("Falha ao alternar modo de energia");
        }

        await loadData();
      } catch (error) {
        window.alert("Nao foi possivel alternar o modo de energia.");
      } finally {
        button.disabled = false;
      }
    }

    function enterConfigurationMode() {
      window.location.href = "/configuration-page";
    }

    document
      .getElementById("resetButton")
      .addEventListener("click", resetCalibration);

    document
      .getElementById("powerModeButton")
      .addEventListener("click", togglePowerMode);

    document
      .getElementById("configurationButton")
      .addEventListener("click", enterConfigurationMode);

    document
      .getElementById("chart")
      .addEventListener("wheel", zoomChart, { passive: false });

    document
      .getElementById("chart")
      .addEventListener("pointerdown", handlePointerDown);

    document
      .getElementById("chart")
      .addEventListener("pointermove", handlePointerMove);

    document
      .getElementById("chart")
      .addEventListener("pointerup", handlePointerEnd);

    document
      .getElementById("chart")
      .addEventListener("pointercancel", handlePointerEnd);

    document
      .getElementById("chart")
      .addEventListener("pointerleave", event => {
        if (!activePointers.has(event.pointerId)) {
          hideChartTooltip();
        }
      });

    if (window.ResizeObserver) {
      new ResizeObserver(resizeChartCanvas).observe(
        document.querySelector(".chart-content")
      );
    } else {
      window.addEventListener("resize", resizeChartCanvas);
    }

    resizeChartCanvas();
    loadData();
    setInterval(loadData, 30000);
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  if (
    configurationConnectionRequested ||
    configurationConnectionActive
  ) {
    server.send_P(
      200,
      "text/html; charset=utf-8",
      CONFIG_CONNECTION_PAGE
    );
  } else if (configurationMode) {
    server.send_P(
      200,
      "text/html; charset=utf-8",
      CONFIG_HTML_PAGE
    );
  } else {
    server.send_P(
      200,
      "text/html; charset=utf-8",
      HTML_PAGE
    );
  }
}

void handleConfigurationPage() {
  activeWindowStartMs = millis();
  server.send_P(
    200,
    "text/html; charset=utf-8",
    CONFIG_HTML_PAGE
  );
}

String escapeJson(const char* value) {
  String escaped;

  for (size_t i = 0; value[i] != '\0'; i++) {
    if (value[i] == '\\' || value[i] == '"') {
      escaped += '\\';
    }
    escaped += value[i];
  }

  return escaped;
}

void handleConfigurationData() {
  activeWindowStartMs = millis();
  String response = "{\"ssid\":\"";
  response += escapeJson(wifiConfig.ssid);
  response += "\",\"collectionIntervalMinutes\":";
  response += String(wifiConfig.collectionIntervalMinutes);
  response += ",\"timeZone\":\"";
  response += escapeJson(wifiConfig.timeZone);
  response += "\",\"tankShape\":\"";
  response += (
    wifiConfig.tankShape == TANK_RECTANGULAR
      ? "rectangular"
      : "cylindrical"
  );
  response += "\",\"tankHeightM\":";
  response += String(wifiConfig.tankHeightM, 3);
  response += ",\"tankWidthM\":";
  response += String(wifiConfig.tankWidthM, 3);
  response += ",\"tankDepthM\":";
  response += String(wifiConfig.tankDepthM, 3);
  response += ",\"tankCapacityL\":";
  response += String(wifiConfig.tankCapacityL, 0);
  response += ",\"sensorPositionCm\":";
  response += String(wifiConfig.sensorPositionCm, 1);
  response += "}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", response);
}

void handleApplyConfiguration() {
  String ssid = server.arg("ssid");
  String password = server.arg("password");
  String timeZone = server.arg("timeZone");
  uint16_t collectionIntervalMinutes =
    static_cast<uint16_t>(
      server.arg("collectionInterval").toInt()
    );
  String tankShapeValue = server.arg("tankShape");
  uint8_t tankShape = tankShapeValue == "rectangular"
    ? TANK_RECTANGULAR
    : TANK_CYLINDRICAL;
  float tankHeightM = server.arg("tankHeight").toFloat();
  float tankWidthM = server.arg("tankWidth").toFloat();
  float tankDepthM = server.arg("tankDepth").toFloat();
  float tankCapacityL = server.arg("tankCapacity").toFloat();
  float configuredSensorPositionCm =
    server.arg("sensorPosition").toFloat();
  ssid.trim();

  bool geometryChanged = tankGeometryChanged(
    tankShape,
    tankHeightM,
    tankWidthM,
    tankDepthM,
    tankCapacityL,
    configuredSensorPositionCm
  );

  if (
    password.length() == 0 &&
    ssid == String(wifiConfig.ssid) &&
    wifiConfig.magic == WIFI_CONFIG_MAGIC
  ) {
    password = String(wifiConfig.password);
  }

  if (!saveWifiConfig(
    ssid,
    password,
    collectionIntervalMinutes,
    timeZone,
    tankShape,
    tankHeightM,
    tankWidthM,
    tankDepthM,
    tankCapacityL,
    configuredSensorPositionCm
  )) {
    server.send(
      400,
      "text/plain; charset=utf-8",
      "Configuracao invalida."
    );
    return;
  }

  if (geometryChanged) {
    depthCalibrationActive = false;
    hasObservedBottom = false;
    observedBottomDistanceCm = 0;
    clearRtcHistory();
    currentWaterHeightCm = -1;
    currentLevelPercent = -1;
    currentLiters = -1;
    currentOverflowLiters = 0;
    currentMarginToTopCm = -1;
    currentOverflow = false;
  }

  server.send_P(
    200,
    "text/html; charset=utf-8",
    CONFIG_CONNECTION_PAGE
  );

  configurationMode = true;
  configurationConnectionRequested = true;
  configurationConnectionActive = false;
  configurationRequestMs = millis();
}

void handleConnectionStatus() {
  String status = "connecting";

  if (WiFi.status() == WL_CONNECTED) {
    status = "connected";
  } else if (
    WiFi.status() == WL_CONNECT_FAILED ||
    WiFi.status() == WL_NO_SSID_AVAIL ||
    (
      configurationConnectionActive &&
      millis() - configurationConnectionStartMs >= 30000
    )
  ) {
    status = "failed";
  }

  String response = "{\"status\":\"";
  response += status;
  response += "\",\"ssid\":\"";
  response += escapeJson(wifiConfig.ssid);
  response += "\",\"ip\":\"";
  response += (
    WiFi.status() == WL_CONNECTED
      ? WiFi.localIP().toString()
      : ""
  );
  response += "\"}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", response);
}

void handleFinishConfiguration() {
  if (WiFi.status() != WL_CONNECTED) {
    server.send(
      409,
      "application/json",
      "{\"ok\":false,\"error\":\"not_connected\"}"
    );
    return;
  }

  String response = "{\"ok\":true,\"ip\":\"";
  response += WiFi.localIP().toString();
  response += "\"}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", response);
  finishConfigurationRequested = true;
  configurationRequestMs = millis();
}

void handleReset() {
  resetCalibration();
  activeWindowStartMs = millis();

  server.sendHeader("Cache-Control", "no-store");
  server.send(
    200,
    "application/json",
    "{\"ok\":true}"
  );
}

void handlePowerMode() {
  energySavingMode = !energySavingMode;
  activeWindowStartMs = millis();
  saveRtcState();

  server.sendHeader("Cache-Control", "no-store");
  server.send(
    200,
    "application/json",
    energySavingMode
      ? "{\"energySavingMode\":true}"
      : "{\"energySavingMode\":false}"
  );
}

void handleData() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", "");

  String chunk;
  chunk.reserve(512);

  chunk = "{";

  chunk += "\"rssi\":";
  chunk += String(WiFi.RSSI());

  chunk += ",\"current\":{";
  chunk += "\"distanceCm\":";
  chunk += String(currentDistanceCm, 2);
  chunk += ",\"waterHeightCm\":";
  chunk += String(currentWaterHeightCm, 2);
  chunk += ",\"levelPercent\":";
  chunk += String(currentLevelPercent, 2);
  chunk += ",\"liters\":";
  chunk += String(currentLiters, 0);
  chunk += ",\"overflow\":";
  chunk += currentOverflow ? "true" : "false";
  chunk += ",\"overflowLiters\":";
  chunk += String(currentOverflowLiters, 0);
  chunk += ",\"marginToTopCm\":";
  chunk += String(currentMarginToTopCm, 2);
  chunk += ",\"validSamples\":";
  chunk += String(currentValidSamples);
  chunk += ",\"timestampEpoch\":";
  chunk += String(static_cast<uint32_t>(currentCollectionTimestamp));
  chunk += ",\"timestampIso\":\"";
  chunk += formatIsoTimestamp(currentCollectionTimestamp);
  chunk += "\"";
  chunk += "}";

  chunk += ",\"parameters\":{";
  chunk += "\"tankDepthCm\":";
  chunk += String(tankDepthCm, 2);
  chunk += ",\"tankTotalHeightCm\":";
  chunk += String(tankTotalHeightCm, 2);
  chunk += ",\"tankFreeboardCm\":";
  chunk += String(tankFreeboardCm, 2);
  chunk += ",\"tankCapacityL\":";
  chunk += String(tankCapacityLiters, 0);
  chunk += ",\"tankShape\":\"";
  chunk += (
    wifiConfig.tankShape == TANK_RECTANGULAR
      ? "rectangular"
      : "cylindrical"
  );
  chunk += "\"";
  chunk += ",\"sensorPositionCm\":";
  chunk += String(sensorPositionCm, 2);
  chunk += ",\"hasObservedBottom\":";
  chunk += hasObservedBottom ? "true" : "false";
  chunk += ",\"calibrationActive\":";
  chunk += depthCalibrationActive ? "true" : "false";
  chunk += ",\"observedBottomDistanceCm\":";
  chunk += String(observedBottomDistanceCm, 2);
  chunk += ",\"energySavingMode\":";
  chunk += energySavingMode ? "true" : "false";
  chunk += ",\"ntpSynchronized\":";
  chunk += ntpSynchronized ? "true" : "false";
  chunk += ",\"collectionIntervalSeconds\":";
  chunk += String(collectionIntervalSeconds);
  chunk += ",\"timeZone\":\"";
  chunk += escapeJson(wifiConfig.timeZone);
  chunk += "\"";
  chunk += "}";

  chunk += ",\"history\":[";

  server.sendContent(chunk);
  chunk = "";

  uint32_t latestSlot = rtcState.lastHistorySlot;
  uint32_t firstSlot =
    latestSlot >= HISTORY_SIZE - 1
      ? latestSlot - (HISTORY_SIZE - 1)
      : 0;
  bool firstHistoryItem = true;

  for (uint32_t slot = firstSlot; slot <= latestSlot; slot++) {
    uint8_t compactLevel =
      rtcState.historyLevels[slot % HISTORY_SIZE];

    if (compactLevel == INVALID_HISTORY_LEVEL) {
      continue;
    }

    if (!firstHistoryItem) {
      chunk += ",";
    }

    firstHistoryItem = false;
    time_t timestamp =
      static_cast<time_t>(slot * HISTORY_SLOT_SECONDS);
    float level = compactLevel * 100.0 / 254.0;

    chunk += "{\"timestampEpoch\":";
    chunk += String(static_cast<uint32_t>(timestamp));
    chunk += ",\"timestampIso\":\"";
    chunk += formatIsoTimestamp(timestamp);
    chunk += "\"";
    chunk += ",\"level\":";
    chunk += String(level, 2);
    chunk += "}";

    if (chunk.length() >= 480) {
      server.sendContent(chunk);
      chunk = "";
      yield();
    }
  }

  chunk += "]}";
  server.sendContent(chunk);
  server.sendContent("");
}

void startConfigurationAccessPoint() {
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(CONFIG_AP_SSID);

  configurationMode = true;
  ntpSynchronized = false;

  Serial.println();
  Serial.print("Modo de configuracao ativo | SSID aberto: ");
  Serial.println(CONFIG_AP_SSID);
  Serial.print("Endereco de configuracao: http://");
  Serial.println(WiFi.softAPIP());
}

void startConfiguredNetworkConnection() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(CONFIG_AP_SSID);
  WiFi.disconnect(false);
  delay(100);
  WiFi.begin(wifiConfig.ssid, wifiConfig.password);

  configurationConnectionRequested = false;
  configurationConnectionActive = true;
  configurationConnectionStartMs = millis();
  configurationMode = true;

  Serial.print("Conectando a rede configurada: ");
  Serial.println(wifiConfig.ssid);
}

void finishConfiguration() {
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);

  configurationMode = false;
  configurationConnectionRequested = false;
  configurationConnectionActive = false;
  finishConfigurationRequested = false;

  synchronizeNtp();
  activeWindowStartMs = millis();
  performReading();
  lastReadMs = millis();

  if (ntpSynchronized) {
    lastReadSlot =
      static_cast<uint32_t>(time(nullptr)) /
      collectionIntervalSeconds;
  }

  Serial.println("Configuracao finalizada; ponto de acesso desligado");
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiConfig.ssid, wifiConfig.password);

  Serial.println();
  Serial.print("Conectando ao Wi-Fi: ");
  Serial.println(wifiConfig.ssid);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    monitorConfigurationResetButton();
  }

  Serial.println();
  Serial.println("Wi-Fi conectado");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

bool isDeepSleepWake() {
  return ESP.getResetReason() == "Deep-Sleep Wake";
}

void monitorConfigurationResetButton() {
  if (digitalRead(CONFIG_RESET_PIN) == LOW) {
    if (flashButtonPressedMs == 0) {
      flashButtonPressedMs = millis();
    } else if (millis() - flashButtonPressedMs >= 10000) {
      Serial.println("Configuracoes apagadas pelo botao FLASH");
      clearStoredConfiguration();
      delay(100);
      ESP.restart();
    }
  } else {
    flashButtonPressedMs = 0;
  }
}

void enterDeepSleep() {
  saveRtcState();

  time_t now = time(nullptr);
  uint32_t activeSeconds = ACTIVE_WINDOW_MS / 1000UL;
  uint32_t sleepSeconds =
    collectionIntervalSeconds > activeSeconds
      ? collectionIntervalSeconds - activeSeconds
      : collectionIntervalSeconds;

  if (ntpSynchronized && now > 0) {
    uint32_t remainder =
      static_cast<uint32_t>(now) % collectionIntervalSeconds;
    sleepSeconds = collectionIntervalSeconds - remainder;

    if (sleepSeconds < 10) {
      sleepSeconds += collectionIntervalSeconds;
    }
  }

  Serial.print("Sono profundo por ");
  Serial.print(sleepSeconds);
  Serial.println(" segundos");
  Serial.flush();

  ESP.deepSleep(
    static_cast<uint64_t>(sleepSeconds) * 1000000ULL,
    WAKE_RF_DEFAULT
  );
  delay(100);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  EEPROM.begin(EEPROM_SIZE);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(CONFIG_RESET_PIN, INPUT_PULLUP);
  digitalWrite(TRIG_PIN, LOW);

  bool restoredAfterSleep = isDeepSleepWake() && loadRtcState();

  if (!restoredAfterSleep) {
    initializeRtcState();
  } else {
    depthCalibrationActive = false;
    Serial.println("Estado restaurado da memoria RTC");
  }

  bool hasWifiCredentials = loadWifiConfig();

  if (hasWifiCredentials) {
    connectWiFi();
    synchronizeNtp();
  } else {
    initializeDefaultWifiConfig();
    startConfigurationAccessPoint();
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/reset", HTTP_POST, handleReset);
  server.on("/power-mode", HTTP_POST, handlePowerMode);
  server.on(
    "/configuration-page",
    HTTP_GET,
    handleConfigurationPage
  );
  server.on(
    "/configuration-data",
    HTTP_GET,
    handleConfigurationData
  );
  server.on(
    "/apply-configuration",
    HTTP_POST,
    handleApplyConfiguration
  );
  server.on(
    "/connection-status",
    HTTP_GET,
    handleConnectionStatus
  );
  server.on(
    "/finish-configuration",
    HTTP_POST,
    handleFinishConfiguration
  );
  server.begin();

  Serial.println("Servidor Web iniciado");

  if (configurationMode) {
    return;
  }

  performReading();
  lastReadMs = millis();
  activeWindowStartMs = millis();

  if (ntpSynchronized) {
    lastReadSlot =
      static_cast<uint32_t>(time(nullptr)) /
      collectionIntervalSeconds;
  }
}

void loop() {
  server.handleClient();
  monitorConfigurationResetButton();

  unsigned long nowMs = millis();

  if (
    configurationConnectionRequested &&
    nowMs - configurationRequestMs >= 500
  ) {
    startConfiguredNetworkConnection();
  }

  if (
    finishConfigurationRequested &&
    nowMs - configurationRequestMs >= 500
  ) {
    finishConfiguration();
  }

  if (configurationMode) {
    yield();
    return;
  }

  if (depthCalibrationActive) {
    if (
      nowMs - lastReadMs >= CALIBRATION_READ_INTERVAL_MS
    ) {
      lastReadMs = nowMs;
      performReading();
    }
  } else if (ntpSynchronized) {
    uint32_t currentSlot =
      static_cast<uint32_t>(time(nullptr)) /
      collectionIntervalSeconds;

    if (currentSlot != lastReadSlot) {
      lastReadSlot = currentSlot;
      lastReadMs = nowMs;
      performReading();
    }
  } else if (
    nowMs - lastReadMs >= collectionIntervalSeconds * 1000UL
  ) {
    lastReadMs = nowMs;
    performReading();
  }

  if (
    energySavingMode &&
    !depthCalibrationActive &&
    nowMs - activeWindowStartMs >= ACTIVE_WINDOW_MS
  ) {
    enterDeepSleep();
  }

  yield();
}
