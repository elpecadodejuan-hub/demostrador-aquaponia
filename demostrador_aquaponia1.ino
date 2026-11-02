/*
  Demostrador de aquaponía v2.2 - Arduino Mega 2560
  Lectura secuencial, ciclo de ozono automático con electroválvulas,
  permisivos de seguridad, calibración en EEPROM.

  Librerías (Gestor de librerías):
    - Adafruit ADS1X15, OneWire, DallasTemperature, LiquidCrystal I2C (Frank de Brabander)

  CICLO DE OZONO (comando o / pulsador):
    NORMAL -> LLENADO (EV1 hasta flotador alto) -> OZONO (15 min) -> REPOSO (aire, desgasifica)
    -> TRASPASO (EV2 hacia el carbón y el lazo, solo con permisivos) -> NORMAL
  El lazo principal (bomba y UV) sigue funcionando en todos los estados.

  PERMISIVOS (EV2 solo abre si se cumplen TODOS):
    sin alarma de ozono en aire, ozonizador apagado, hay agua en el recipiente de ozono,
    la pecera no está llena, y (si HAY_ORP_OZONO) el ORP del recipiente bajó a
    ORP inicial + ORP_DELTA_TRASPASO_MV. EV1 y EV2 nunca abren a la vez.

  COMANDOS (monitor serie 115200, "Nueva línea", terminar con Enter):
    o  iniciar ciclo        x  abortar          t  traspaso manual
    r  resetear alarmas     s  estado           cal  ver calibración
    p7 / p4          calibrar pH con buffer 7,00 / 4,01
    orp <mV>         calibrar ORP del lazo con solución de referencia
    orpo <mV>        calibrar ORP del recipiente de ozono
    seco / mojado    calibrar humedad de suelo
    borrar           valores de fábrica

  ---------------- MAPA DE PINES (Mega) ----------------
  I2C (20 SDA, 21 SCL): LCD 0x27, ADS1115 #1 0x48 (pH, ORP, TDS, turbidez),
                        ADS1115 #2 0x49 (ORP recipiente de ozono; ADDR a VDD)
  A0 suelo | A1 MQ-131 | 2 caudal (interrupción) | 4 DS18B20 | 22 pulsador
  23-27 flotadores | 30-37 módulo de 8 relés | 40-43 alimentación conmutada de sensores
  FUTURO (libres): A2-A15, 28-29, 38-39, 44-49 (segundo módulo de relés),
                   50-53 (SPI: tarjeta SD), Serial1 pines 18/19 (ESP32 / WiFi), Serial2 y Serial3.
*/

#include <stddef.h>      // offsetof (checksum de la EEPROM)
#include <Wire.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include <Adafruit_ADS1X15.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>

// ---------------- OPCIONES (activar o desactivar funciones) ----------------
// OJO con el watchdog: el bootloader original del Mega 2560 NO lo soporta y puede
// quedar en bucle de reinicios hasta cortar la alimentación. Activar (1) solo después
// de instalar un bootloader compatible (por ejemplo MegaCore con Optiboot).
#define USAR_WATCHDOG 0

const bool HAY_ORP_OZONO       = true;   // sonda ORP + ADS1115 #2 en el recipiente de ozono
const bool HAY_FLOTADOR_CAPILAR = true;  // quinto flotador (alto) en el depósito de mediterráneas
const bool LOG_CSV             = false;  // una línea CSV por ciclo (para registrar en PC)
const bool RELE_ACTIVO_BAJO    = true;   // la mayoría de los módulos activan con LOW
const uint8_t FLOTADOR_AGUA_LEVEL = LOW; // contacto a GND = hay agua a esa altura

// ---------------- PINES ----------------
const uint8_t PIN_SUELO    = A0;
const uint8_t PIN_MQ131    = A1;
const uint8_t PIN_CAUDAL   = 2;    // YF-S201
const uint8_t PIN_DS18B20  = 4;    // con 4,7 kΩ a 5 V
const uint8_t PIN_PULSADOR = 22;   // a GND, con pull-up interno

const uint8_t F_TANQUE_MIN  = 23;  // hay agua en la pecera
const uint8_t F_TANQUE_MAX  = 24;  // pecera llena
const uint8_t F_OZONO_MIN   = 25;  // hay agua en el recipiente de ozono
const uint8_t F_OZONO_MAX   = 26;  // recipiente de ozono lleno
const uint8_t F_CAPILAR_MAX = 27;  // depósito de mediterráneas lleno (opcional)

const uint8_t R_BOMBA = 30, R_UV = 31, R_OZONO = 32, R_AIRE = 33;
const uint8_t R_EV1 = 34;          // red -> recipiente de ozono (llenado)
const uint8_t R_EV2 = 35;          // salida del recipiente de ozono -> carbón -> lazo
const uint8_t R_EV3 = 36;          // entrada al cultivo
const uint8_t R_EV4 = 37;          // capilaridad (mediterráneas)

// Alimentación conmutada de sondas. 43 (turbidez) va a la compuerta de un MOSFET IRLZ44N
// que alimenta el sensor, porque este consume más de lo que un pin entrega.
const uint8_t PIN_PWR_PH = 40, PIN_PWR_ORP = 41, PIN_PWR_EC = 42, PIN_PWR_TURB = 43;

// Canales ADS1115
const uint8_t CH_PH = 0, CH_ORP = 1, CH_TDS = 2, CH_TURB = 3;  // ADS #1
const uint8_t CH_ORP_OZONO = 0;                                // ADS #2

// ---------------- TIEMPOS Y UMBRALES ----------------
const unsigned long PASO_MS         = 30000UL;           // 30 s entre sensores
const uint8_t       N_PASOS         = 10;                // ciclo de 5 min
const unsigned long LLENADO_MAX_MS  = 3UL * 60 * 1000;   // tope del llenado
const unsigned long OZONO_MS        = 15UL * 60 * 1000;
const unsigned long REPOSO_MS       = 20UL * 60 * 1000;  // mínimo antes de poder traspasar
const unsigned long REPOSO_MAX_MS   = 60UL * 60 * 1000;  // si no se cumplen permisivos, se cancela
const unsigned long TRASPASO_MAX_MS = 6UL * 60 * 1000;   // tope del traspaso
const unsigned long GRACIA_UV_MS    = 10000UL;
const unsigned long MQ_CALENT_MS    = 180000UL;
const unsigned long EV4_PERIODO_MS  = 3600000UL;         // capilaridad: cada 1 h...
const unsigned long EV4_ABIERTA_MS  = 300000UL;          // ...durante 5 min

const int   O3_DELTA          = 80;    // alarma: lectura > base + delta (unidades ADC)
const uint8_t O3_LECTURAS     = 3;     // segundos seguidos sobre el umbral
const float ORP_DELTA_TRASPASO_MV = 80.0;
const float ORP_A             = -1.0;  // ORP(mV) = ORP_A * mV_sensor + offset calibrado
const float FACTOR_CAUDAL     = 7.5;   // Hz por L/min del YF-S201
const float CAUDAL_MIN_LPM    = 0.5;

// ---------------- CALIBRACIÓN EN EEPROM ----------------
struct Calib {
  uint16_t magic;
  float phV7, phV4;
  float orpB, orpOzonoB;
  float sueloSeco, sueloMojado;
  uint8_t chk;
};
const uint16_t CAL_MAGIC = 0xA9C2;
const int CAL_DIR = 0;
Calib cal;

uint8_t checksumCal(const Calib &c) {
  const uint8_t *p = (const uint8_t *)&c;
  uint8_t s = 0x5A;
  for (size_t i = 0; i < offsetof(Calib, chk); i++) {
    s = (uint8_t)((s << 1) | (s >> 7)) ^ p[i];
  }
  return s;
}

void valoresPorDefecto() {            // valores de ejemplo: hay que calibrar
  cal.phV7 = 1.50;
  cal.phV4 = 1.95;
  cal.orpB = 2000.0;
  cal.orpOzonoB = 2000.0;
  cal.sueloSeco = 520.0;
  cal.sueloMojado = 260.0;
}

void guardarCalibracion() {
  cal.magic = CAL_MAGIC;
  cal.chk = checksumCal(cal);
  EEPROM.put(CAL_DIR, cal);           // put solo escribe los bytes que cambian
}

void cargarCalibracion() {
  Calib tmp;
  EEPROM.get(CAL_DIR, tmp);
  if (tmp.magic == CAL_MAGIC && tmp.chk == checksumCal(tmp)) {
    cal = tmp;
    Serial.println(F("[CAL] calibración cargada de EEPROM"));
  } else {
    valoresPorDefecto();
    Serial.println(F("[CAL] sin calibración válida, valores de fábrica (calibrar)"));
  }
}

void imprimirCal() {
  Serial.print(F("[CAL] pH7 V=")); Serial.print(cal.phV7, 3);
  Serial.print(F(" pH4 V=")); Serial.print(cal.phV4, 3);
  Serial.print(F(" ORP b=")); Serial.print(cal.orpB, 1);
  Serial.print(F(" ORPoz b=")); Serial.print(cal.orpOzonoB, 1);
  Serial.print(F(" suelo seco=")); Serial.print(cal.sueloSeco, 0);
  Serial.print(F(" mojado=")); Serial.println(cal.sueloMojado, 0);
}

// ---------------- TIPOS ----------------
enum Estado : uint8_t { NORMAL, LLENADO, OZONO, REPOSO, TRASPASO };
enum Sensor : uint8_t { S_TEMP, S_PH, S_ORP, S_TURB, S_EC, S_SUELO, S_NINGUNO };
enum CalAccion : uint8_t { C_NINGUNA, C_PH7, C_PH4, C_ORP, C_ORPO, C_SECO, C_MOJADO };

struct Flotador {
  uint8_t pin;
  bool valor;    // true = hay agua a esa altura (estable)
  bool ultimo;   // lectura anterior, para antirrebote
};

// Orden: temperatura primero (compensa pH y EC) y EC al final (inyecta corriente al agua)
const Sensor PROGRAMA[N_PASOS] = {
  S_TEMP, S_PH, S_ORP, S_TURB, S_EC, S_SUELO,
  S_NINGUNO, S_NINGUNO, S_NINGUNO, S_NINGUNO
};

// ---------------- OBJETOS Y VARIABLES ----------------
Adafruit_ADS1115 ads;     // 0x48
Adafruit_ADS1115 ads2;    // 0x49
bool ads2Ok = false;
OneWire oneWire(PIN_DS18B20);
DallasTemperature ds(&oneWire);
LiquidCrystal_I2C lcd(0x27, 16, 2);

Estado estado = NORMAL;
Sensor sensorActivo = S_NINGUNO;
CalAccion calAccion = C_NINGUNA;
float calOrpRef = 0;
uint8_t paso = 0;
unsigned long tPaso = 0, tEncendido = 0, tEstado = 0, tCal = 0;
unsigned long tRapido = 0, tLcd = 0, tPrint = 0, tPulsador = 0, tBombaOn = 0, tEv4 = 0;
bool bombaPrev = false, ev4Activa = false;

Flotador flTanqueMin  = { F_TANQUE_MIN,  false, false };
Flotador flTanqueMax  = { F_TANQUE_MAX,  false, false };
Flotador flOzonoMin   = { F_OZONO_MIN,   false, false };
Flotador flOzonoMax   = { F_OZONO_MAX,   false, false };
Flotador flCapilarMax = { F_CAPILAR_MAX, false, false };

float tempC = 25.0, ph = NAN, orp = NAN, tds = NAN, ntu = NAN, sueloPct = NAN;
float orpOzono = NAN, orpBase = NAN;
float caudalLpm = 0.0;
int o3Raw = 0, o3Base = 0;
uint8_t o3Cuenta = 0;
unsigned int ciclosOzono = 0;   // ciclos de ozono completados desde el encendido
bool calentado = false, alarmaO3 = false, bloqueoCiclo = false;
bool salidas[4] = { false, false, false, false };   // estado de EV1..EV4
volatile unsigned int pulsos = 0;

char linea[20];
uint8_t nLinea = 0;

void isrCaudal() { pulsos++; }

// ---------------- UTILIDADES ----------------
void evento(const __FlashStringHelper *msg) {
  Serial.print(F("[t="));
  Serial.print(millis() / 1000UL);
  Serial.print(F("s] "));
  Serial.println(msg);
}

void rele(uint8_t pin, bool on) {
  bool nivel = RELE_ACTIVO_BAJO ? !on : on;
  digitalWrite(pin, nivel ? HIGH : LOW);
}

void iniciarRele(uint8_t pin) {
  digitalWrite(pin, RELE_ACTIVO_BAJO ? HIGH : LOW);  // apagado antes de ser salida
  pinMode(pin, OUTPUT);
}

void iniciarFlotador(Flotador &f) {
  pinMode(f.pin, INPUT_PULLUP);
  f.valor = (digitalRead(f.pin) == FLOTADOR_AGUA_LEVEL);
  f.ultimo = f.valor;
}

// Antirrebote: el valor cambia solo si dos lecturas seguidas (1 s) coinciden
void actualizarFlotador(Flotador &f) {
  bool raw = (digitalRead(f.pin) == FLOTADOR_AGUA_LEVEL);
  if (raw == f.ultimo) f.valor = raw;
  f.ultimo = raw;
}

void ordenar(int16_t *v, uint8_t n) {
  for (uint8_t i = 1; i < n; i++) {
    int16_t x = v[i];
    int8_t j = i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
    v[j + 1] = x;
  }
}

// La mediana descarta valores atípicos mejor que el promedio
int analogMediana(uint8_t pin, uint8_t n) {
  int16_t m[15];
  if (n > 15) n = 15;
  for (uint8_t i = 0; i < n; i++) { m[i] = analogRead(pin); delayMicroseconds(300); }
  ordenar(m, n);
  return m[n / 2];
}

float leerVoltaje(Adafruit_ADS1115 &adc, uint8_t canal) {
  const uint8_t n = 11;
  int16_t m[n];
  for (uint8_t i = 0; i < n; i++) m[i] = adc.readADC_SingleEnded(canal);
  ordenar(m, n);
  return adc.computeVolts(m[n / 2]);
}

char letraEstado() {
  switch (estado) {
    case LLENADO:  return 'L';
    case OZONO:    return 'O';
    case REPOSO:   return 'R';
    case TRASPASO: return 'T';
    default:       return 'N';
  }
}

const __FlashStringHelper *nombreEstado() {
  switch (estado) {
    case LLENADO:  return F("LLENADO");
    case OZONO:    return F("OZONO");
    case REPOSO:   return F("REPOSO");
    case TRASPASO: return F("TRASPASO");
    default:       return F("NORMAL");
  }
}

uint8_t pinPotencia(Sensor s) {
  switch (s) {
    case S_PH:   return PIN_PWR_PH;
    case S_ORP:  return PIN_PWR_ORP;
    case S_EC:   return PIN_PWR_EC;
    case S_TURB: return PIN_PWR_TURB;
    default:     return 255;
  }
}

unsigned long tiempoEstabilizacion(Sensor s) {
  switch (s) {
    case S_PH:
    case S_ORP:  return 20000UL;
    case S_EC:   return 5000UL;
    case S_TURB: return 3000UL;
    default:     return 0;
  }
}

// ---------------- LECTURAS ----------------
void leerTemperatura() {
  ds.requestTemperatures();
  float t = ds.getTempCByIndex(0);
  if (t != DEVICE_DISCONNECTED_C) tempC = t;
  Serial.print(F("[LECT] temp C = ")); Serial.println(tempC, 1);
}

void leerSuelo() {
  float raw = analogMediana(PIN_SUELO, 9);
  sueloPct = constrain((cal.sueloSeco - raw) * 100.0 / (cal.sueloSeco - cal.sueloMojado), 0.0, 100.0);
  Serial.print(F("[LECT] suelo % = ")); Serial.println(sueloPct, 0);
}

void leerOrpOzono() {
  if (!HAY_ORP_OZONO || !ads2Ok) return;
  float mv = leerVoltaje(ads2, CH_ORP_OZONO) * 1000.0;
  orpOzono = ORP_A * mv + cal.orpOzonoB;
}

void leerSensorActivo() {
  switch (sensorActivo) {
    case S_PH: {
      float v = leerVoltaje(ads, CH_PH);
      float pendiente = (7.00 - 4.01) / (cal.phV7 - cal.phV4);
      ph = 7.00 + (v - cal.phV7) * pendiente;
      Serial.print(F("[LECT] pH = ")); Serial.println(ph, 2);
      break;
    }
    case S_ORP: {
      float mv = leerVoltaje(ads, CH_ORP) * 1000.0;
      orp = ORP_A * mv + cal.orpB;
      Serial.print(F("[LECT] ORP mV = ")); Serial.println(orp, 0);
      break;
    }
    case S_TURB: {
      float v = leerVoltaje(ads, CH_TURB);
      if (v < 2.5) ntu = 3000.0;
      else ntu = max(0.0, -1120.4 * v * v + 5742.3 * v - 4352.9);
      Serial.print(F("[LECT] turbidez NTU (aprox) = ")); Serial.println(ntu, 0);
      break;
    }
    case S_EC: {
      float v = leerVoltaje(ads, CH_TDS);
      float coef = 1.0 + 0.02 * (tempC - 25.0);   // compensación por temperatura
      float vc = v / coef;
      tds = (133.42 * vc * vc * vc - 255.86 * vc * vc + 857.39 * vc) * 0.5;
      Serial.print(F("[LECT] TDS ppm = ")); Serial.println(tds, 0);
      break;
    }
    default: break;
  }
}

void apagarSensor() {
  if (sensorActivo == S_NINGUNO) return;
  digitalWrite(pinPotencia(sensorActivo), LOW);
  sensorActivo = S_NINGUNO;
}

void iniciarPaso(Sensor s) {
  apagarSensor();
  switch (s) {
    case S_TEMP:  leerTemperatura(); break;
    case S_SUELO: leerSuelo(); break;
    case S_PH:
    case S_ORP:
    case S_EC:
    case S_TURB:
      sensorActivo = s;
      tEncendido = millis();
      digitalWrite(pinPotencia(s), HIGH);
      break;
    default: break;
  }
}

void reiniciarSecuencia() {
  paso = 0;
  tPaso = millis();
  iniciarPaso(PROGRAMA[paso]);
}

void registroCsv() {
  Serial.print(F("CSV,")); Serial.print(millis() / 1000UL);
  Serial.print(','); Serial.print(tempC, 1);
  Serial.print(','); Serial.print(ph, 2);
  Serial.print(','); Serial.print(orp, 0);
  Serial.print(','); Serial.print(tds, 0);
  Serial.print(','); Serial.print(ntu, 0);
  Serial.print(','); Serial.println(sueloPct, 0);
}

// Secuencia: un sensor cada 30 s, con alimentación conmutada y espera de estabilización
void secuencia(unsigned long ahora) {
  if (estado != NORMAL || calAccion != C_NINGUNA) return;
  if (sensorActivo != S_NINGUNO && ahora - tEncendido >= tiempoEstabilizacion(sensorActivo)) {
    leerSensorActivo();
    apagarSensor();
  }
  if (ahora - tPaso >= PASO_MS) {
    tPaso = ahora;
    paso = (paso + 1) % N_PASOS;
    iniciarPaso(PROGRAMA[paso]);
    if (LOG_CSV && paso == 7) registroCsv();
  }
}

// ---------------- CALIBRACIÓN (sin bloquear el bucle) ----------------
void iniciarCal(CalAccion a, float ref) {
  if (estado != NORMAL)       { Serial.println(F("[CAL] solo en estado normal")); return; }
  if (calAccion != C_NINGUNA) { Serial.println(F("[CAL] ya hay una calibración en curso")); return; }
  if ((a == C_ORP || a == C_ORPO) && (ref < -1000 || ref > 1500)) {
    Serial.println(F("[CAL] valor de referencia fuera de rango"));
    return;
  }
  if (a == C_ORPO && !(HAY_ORP_OZONO && ads2Ok)) {
    Serial.println(F("[CAL] no hay ORP en el recipiente de ozono"));
    return;
  }
  apagarSensor();                      // la secuencia queda en pausa
  calAccion = a;
  calOrpRef = ref;
  tCal = millis();
  switch (a) {
    case C_PH7:
    case C_PH4:
      digitalWrite(PIN_PWR_PH, HIGH);
      Serial.println(F("[CAL] sonda de pH en el buffer, midiendo en 30 s"));
      break;
    case C_ORP:
      digitalWrite(PIN_PWR_ORP, HIGH);
      Serial.println(F("[CAL] sonda de ORP en la solución, midiendo en 30 s"));
      break;
    case C_ORPO:
      Serial.println(F("[CAL] ORP del recipiente en la solución, midiendo en 30 s"));
      break;
    default:
      Serial.println(F("[CAL] midiendo suelo en 5 s"));
      break;
  }
}

void gestionCal(unsigned long ahora) {
  if (calAccion == C_NINGUNA) return;
  bool esSuelo = (calAccion == C_SECO || calAccion == C_MOJADO);
  if (ahora - tCal < (esSuelo ? 5000UL : 30000UL)) return;

  bool ok = false;
  switch (calAccion) {
    case C_PH7: {
      float v = leerVoltaje(ads, CH_PH);
      if (fabs(v - cal.phV4) < 0.05) Serial.println(F("[CAL] error: pH 7 y pH 4 dan el mismo voltaje"));
      else { cal.phV7 = v; ok = true; }
      digitalWrite(PIN_PWR_PH, LOW);
      break;
    }
    case C_PH4: {
      float v = leerVoltaje(ads, CH_PH);
      if (fabs(v - cal.phV7) < 0.05) Serial.println(F("[CAL] error: pH 7 y pH 4 dan el mismo voltaje"));
      else { cal.phV4 = v; ok = true; }
      digitalWrite(PIN_PWR_PH, LOW);
      break;
    }
    case C_ORP: {
      float mv = leerVoltaje(ads, CH_ORP) * 1000.0;
      cal.orpB = calOrpRef - ORP_A * mv;
      ok = true;
      digitalWrite(PIN_PWR_ORP, LOW);
      break;
    }
    case C_ORPO: {
      float mv = leerVoltaje(ads2, CH_ORP_OZONO) * 1000.0;
      cal.orpOzonoB = calOrpRef - ORP_A * mv;
      ok = true;
      break;
    }
    case C_SECO: {
      float raw = analogMediana(PIN_SUELO, 9);
      if (fabs(raw - cal.sueloMojado) < 20) Serial.println(F("[CAL] error: seco y mojado muy parecidos"));
      else { cal.sueloSeco = raw; ok = true; }
      break;
    }
    case C_MOJADO: {
      float raw = analogMediana(PIN_SUELO, 9);
      if (fabs(raw - cal.sueloSeco) < 20) Serial.println(F("[CAL] error: seco y mojado muy parecidos"));
      else { cal.sueloMojado = raw; ok = true; }
      break;
    }
    default: break;
  }
  if (ok) {
    guardarCalibracion();
    Serial.println(F("[CAL] guardada en EEPROM"));
    imprimirCal();
  }
  calAccion = C_NINGUNA;
  reiniciarSecuencia();
}

// ---------------- PERMISIVOS Y CICLO DE OZONO ----------------
bool permisivoTraspaso() {
  if (alarmaO3) return false;                         // ozono en el aire
  if (estado == OZONO) return false;                  // ozonizador activo
  if (!calentado) return false;                       // MQ-131 sin referencia
  if (!flOzonoMin.valor) return false;                // nada que traspasar
  if (flTanqueMax.valor) return false;                // pecera llena
  if (salidas[0]) return false;                       // EV1 abierta: nunca juntas
  if (HAY_ORP_OZONO) {
    if (!ads2Ok || isnan(orpOzono) || isnan(orpBase)) return false;
    if (orpOzono > orpBase + ORP_DELTA_TRASPASO_MV) return false;   // aún hay ozono disuelto
  }
  return true;
}

void abortarCiclo(const __FlashStringHelper *causa) {
  estado = NORMAL;
  bloqueoCiclo = true;                                // nuevo ciclo solo tras revisar y resetear (r)
  evento(causa);
  reiniciarSecuencia();
}

void iniciarOzono() {
  if (calAccion != C_NINGUNA) { Serial.println(F("[OZONO] calibrando, esperar")); return; }
  if (estado != NORMAL)       { Serial.println(F("[OZONO] ya hay un ciclo en curso")); return; }
  if (!calentado)             { Serial.println(F("[OZONO] MQ-131 calentando, esperar")); return; }
  if (alarmaO3 || bloqueoCiclo) { Serial.println(F("[OZONO] bloqueado por alarma o falla, revisar y resetear con r")); return; }
  if (flOzonoMin.valor)       { Serial.println(F("[OZONO] el recipiente tiene agua: traspasar (t) o vaciar")); return; }
  if (flTanqueMax.valor)      { Serial.println(F("[OZONO] pecera llena, no hay donde traspasar")); return; }
  if (HAY_ORP_OZONO && !ads2Ok) { Serial.println(F("[OZONO] falta el ADS1115 #2 (ORP del recipiente)")); return; }
  apagarSensor();             // la secuencia queda en pausa
  estado = LLENADO;
  tEstado = millis();
  evento(F("LLENADO iniciado, secuencia en pausa"));
}

void iniciarTraspasoManual() {
  if (estado != NORMAL || calAccion != C_NINGUNA) { Serial.println(F("[TRASPASO] solo desde estado normal")); return; }
  if (!permisivoTraspaso()) { Serial.println(F("[TRASPASO] permisivos no cumplidos")); return; }
  apagarSensor();
  estado = TRASPASO;
  tEstado = millis();
  evento(F("TRASPASO manual iniciado"));
}

void abortarManual() {
  switch (estado) {
    case LLENADO:
    case TRASPASO:
      estado = NORMAL;
      reiniciarSecuencia();
      evento(F("ciclo abortado, vuelve a normal"));
      break;
    case OZONO:
      estado = REPOSO;
      tEstado = millis();
      evento(F("ozono abortado, pasa a reposo"));
      break;
    default:
      Serial.println(F("[X] nada para abortar"));
      break;
  }
}

void gestionEstado(unsigned long ahora) {
  switch (estado) {
    case LLENADO:
      if (flOzonoMax.valor) {
        leerOrpOzono();
        orpBase = orpOzono;                           // referencia antes de ozonizar
        estado = OZONO;
        tEstado = ahora;
        evento(F("recipiente lleno, OZONO iniciado"));
      } else if (ahora - tEstado >= LLENADO_MAX_MS) {
        abortarCiclo(F("FALLA: el llenado no llegó al flotador alto, EV1 cerrada"));
      }
      break;
    case OZONO:
      if (ahora - tEstado >= OZONO_MS) {
        ciclosOzono++;
        estado = REPOSO;
        tEstado = ahora;
        evento(F("fin del ozono, REPOSO y desgasificación"));
      } else if (!flOzonoMin.valor) {
        estado = REPOSO;
        tEstado = ahora;
        evento(F("sin agua en el recipiente, ozono cortado, REPOSO"));
      }
      break;
    case REPOSO: {
      unsigned long t = ahora - tEstado;
      if (t >= REPOSO_MS && permisivoTraspaso()) {
        estado = TRASPASO;
        tEstado = ahora;
        evento(F("permisivos cumplidos, TRASPASO iniciado"));
      } else if (t >= REPOSO_MAX_MS) {
        abortarCiclo(F("FALLA: permisivos sin cumplir, traspaso cancelado (EV2 cerrada)"));
      }
      break;
    }
    case TRASPASO:
      if (!flOzonoMin.valor || flTanqueMax.valor || ahora - tEstado >= TRASPASO_MAX_MS) {
        estado = NORMAL;
        evento(F("fin del traspaso, vuelve a normal"));
        reiniciarSecuencia();
      }
      break;
    default: break;
  }
}

// ---------------- SALIDAS Y ENCLAVAMIENTOS ----------------
bool ev4Deseada(unsigned long ahora) {
  if (!ev4Activa && ahora - tEv4 >= EV4_PERIODO_MS) { ev4Activa = true; tEv4 = ahora; }
  if (ev4Activa && (ahora - tEv4 >= EV4_ABIERTA_MS || (HAY_FLOTADOR_CAPILAR && flCapilarMax.valor))) {
    ev4Activa = false;
    tEv4 = ahora;
  }
  return ev4Activa && estado == NORMAL && flTanqueMin.valor;
}

void aplicarSalidas(unsigned long ahora) {
  // El lazo principal sigue en todos los estados: solo depende del nivel de la pecera
  bool bomba = flTanqueMin.valor;
  if (bomba && !bombaPrev) tBombaOn = ahora;
  bombaPrev = bomba;
  bool uv = bomba && (ahora - tBombaOn >= GRACIA_UV_MS) && (caudalLpm >= CAUDAL_MIN_LPM);

  bool aire  = (estado == OZONO || estado == REPOSO);
  bool ozono = (estado == OZONO) && !alarmaO3 && calentado && flOzonoMin.valor && aire;
  bool ev1   = (estado == LLENADO) && !flOzonoMax.valor && !bloqueoCiclo;
  bool ev2   = (estado == TRASPASO) && permisivoTraspaso();
  bool ev3   = bomba;                      // ajustar si EV3 tiene un ciclo propio
  bool ev4   = ev4Deseada(ahora);

  // Enclavamientos duros, por encima de cualquier estado
  if (ozono) ev2 = false;                  // con ozono encendido, EV2 nunca abre
  if (ev1)   ev2 = false;                  // EV1 y EV2 nunca juntas
  if (ev2)   ev1 = false;

  rele(R_BOMBA, bomba);
  rele(R_UV, uv);
  rele(R_OZONO, ozono);
  rele(R_AIRE, aire);
  rele(R_EV1, ev1);
  rele(R_EV2, ev2);
  rele(R_EV3, ev3);
  rele(R_EV4, ev4);

  salidas[0] = ev1; salidas[1] = ev2; salidas[2] = ev3; salidas[3] = ev4;
}

void imprimirEstado() {
  Serial.print(F("[ESTADO] ")); Serial.print(nombreEstado());
  Serial.print(F(" niveles(tMin,tMax,oMin,oMax)="));
  Serial.print(flTanqueMin.valor); Serial.print(flTanqueMax.valor);
  Serial.print(flOzonoMin.valor); Serial.print(flOzonoMax.valor);
  Serial.print(F(" EV="));
  for (uint8_t i = 0; i < 4; i++) Serial.print(salidas[i]);
  Serial.print(F(" caudal=")); Serial.print(caudalLpm, 1);
  Serial.print(F(" o3raw=")); Serial.print(o3Raw);
  Serial.print(F(" orpOz=")); Serial.print(orpOzono, 0);
  Serial.print(F(" alarma=")); Serial.print(alarmaO3);
  Serial.print(F(" bloqueo=")); Serial.print(bloqueoCiclo);
  Serial.print(F(" ciclos=")); Serial.println(ciclosOzono);
}

// Tarea rápida (1 s): seguridad siempre activa, incluso con la secuencia en pausa
void tareaRapida(unsigned long ahora, unsigned long dt) {
  actualizarFlotador(flTanqueMin);
  actualizarFlotador(flTanqueMax);
  actualizarFlotador(flOzonoMin);
  actualizarFlotador(flOzonoMax);
  if (HAY_FLOTADOR_CAPILAR) actualizarFlotador(flCapilarMax);

  noInterrupts();
  unsigned int p = pulsos;
  pulsos = 0;
  interrupts();
  caudalLpm = (p / (dt / 1000.0)) / FACTOR_CAUDAL;

  o3Raw = analogMediana(PIN_MQ131, 9);
  if (!calentado && ahora >= MQ_CALENT_MS) {      // referencia con el ozono apagado
    o3Base = o3Raw;
    calentado = true;
    Serial.print(F("[MQ131] listo, base = ")); Serial.println(o3Base);
  }
  if (calentado && !alarmaO3) {
    // Varias lecturas seguidas sobre el umbral: evita falsas alarmas sin retrasar la real
    if (o3Raw > o3Base + O3_DELTA) o3Cuenta++; else o3Cuenta = 0;
    if (o3Cuenta >= O3_LECTURAS) {
      alarmaO3 = true;
      evento(F("ALARMA: ozono en aire alto, ozonizador cortado, EV2 bloqueada"));
      if (estado == OZONO || estado == TRASPASO) { estado = REPOSO; tEstado = ahora; }
    }
  }

  if (estado != NORMAL) leerOrpOzono();           // sigue el ORP del recipiente durante el ciclo

  aplicarSalidas(ahora);

  if (ahora - tPrint >= 10000UL) {
    tPrint = ahora;
    imprimirEstado();
  }
}

// ---------------- INTERFAZ ----------------
void fnum(float v, uint8_t dec, char *out) {
  if (isnan(v)) strcpy(out, "--");
  else dtostrf(v, 1, dec, out);
}

void escribirLinea(uint8_t fila, const char *txt) {
  lcd.setCursor(0, fila);
  uint8_t n = 0;
  while (txt[n] && n < 16) { lcd.write(txt[n]); n++; }
  while (n < 16) { lcd.write(' '); n++; }
}

void mostrarLcd(unsigned long ahora) {
  char a[10], b[10], c[10], l[24];
  uint8_t pagina = (ahora / 3000UL) % 3;
  if (pagina == 0) {
    fnum(ph, 1, a); fnum(tempC, 1, b);
    snprintf(l, sizeof l, "pH%s T%s %c", a, b, letraEstado());
    escribirLinea(0, l);
    fnum(orp, 0, a); fnum(tds, 0, b);
    snprintf(l, sizeof l, "ORP%s TDS%s%s", a, b, alarmaO3 ? " !O3" : "");
    escribirLinea(1, l);
  } else if (pagina == 1) {
    fnum(caudalLpm, 1, a); fnum(ntu, 0, b);
    snprintf(l, sizeof l, "Q%s NTU%s", a, b);
    escribirLinea(0, l);
    snprintf(l, sizeof l, "Niv%d%d%d%d EV%d%d%d%d",
             flTanqueMin.valor, flTanqueMax.valor, flOzonoMin.valor, flOzonoMax.valor,
             salidas[0], salidas[1], salidas[2], salidas[3]);
    escribirLinea(1, l);
  } else {
    fnum(orpOzono, 0, a);
    snprintf(l, sizeof l, "ORPoz %s %c", a, letraEstado());
    escribirLinea(0, l);
    if (isnan(orpBase)) strcpy(c, "--"); else fnum(orpBase + ORP_DELTA_TRASPASO_MV, 0, c);
    snprintf(l, sizeof l, "Limite %s C%u", c, ciclosOzono);
    escribirLinea(1, l);
  }
}

void procesarComando(const char *s) {
  if (!strcmp(s, "o")) iniciarOzono();
  else if (!strcmp(s, "x")) abortarManual();
  else if (!strcmp(s, "t")) iniciarTraspasoManual();
  else if (!strcmp(s, "r")) {
    alarmaO3 = false; bloqueoCiclo = false; o3Cuenta = 0;
    Serial.println(F("[ALARMAS] reseteadas"));
  }
  else if (!strcmp(s, "s")) imprimirEstado();
  else if (!strcmp(s, "p7")) iniciarCal(C_PH7, 0);
  else if (!strcmp(s, "p4")) iniciarCal(C_PH4, 0);
  else if (!strncmp(s, "orpo ", 5)) iniciarCal(C_ORPO, atof(s + 5));
  else if (!strncmp(s, "orp ", 4)) iniciarCal(C_ORP, atof(s + 4));
  else if (!strcmp(s, "seco")) iniciarCal(C_SECO, 0);
  else if (!strcmp(s, "mojado")) iniciarCal(C_MOJADO, 0);
  else if (!strcmp(s, "cal")) imprimirCal();
  else if (!strcmp(s, "borrar")) {
    valoresPorDefecto();
    guardarCalibracion();
    Serial.println(F("[CAL] valores de fábrica restaurados"));
  }
  else Serial.println(F("[?] comando desconocido"));
}

void leerComandos() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (nLinea > 0) {
        linea[nLinea] = 0;
        procesarComando(linea);
        nLinea = 0;
      }
    } else if (nLinea < sizeof(linea) - 1) {
      linea[nLinea++] = c;
    }
  }
}

void leerPulsador(unsigned long ahora) {
  if (digitalRead(PIN_PULSADOR) == LOW && ahora - tPulsador > 500) {
    tPulsador = ahora;
    iniciarOzono();
  }
}

// ---------------- SETUP Y LOOP ----------------
void setup() {
#if USAR_WATCHDOG
  wdt_disable();
#endif
  Serial.begin(115200);
  Serial.print(F("[INICIO] demostrador v")); Serial.println(F("2.2 Mega"));

  const uint8_t reles[] = { R_BOMBA, R_UV, R_OZONO, R_AIRE, R_EV1, R_EV2, R_EV3, R_EV4 };
  for (uint8_t i = 0; i < 8; i++) iniciarRele(reles[i]);   // todo apagado primero

  const uint8_t pwr[] = { PIN_PWR_PH, PIN_PWR_ORP, PIN_PWR_EC, PIN_PWR_TURB };
  for (uint8_t i = 0; i < 4; i++) { digitalWrite(pwr[i], LOW); pinMode(pwr[i], OUTPUT); }

  iniciarFlotador(flTanqueMin);
  iniciarFlotador(flTanqueMax);
  iniciarFlotador(flOzonoMin);
  iniciarFlotador(flOzonoMax);
  if (HAY_FLOTADOR_CAPILAR) iniciarFlotador(flCapilarMax);

  pinMode(PIN_PULSADOR, INPUT_PULLUP);
  pinMode(PIN_CAUDAL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_CAUDAL), isrCaudal, RISING);

  Wire.begin();
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(25000, true);    // 25 ms: si el bus se cuelga, se recupera solo
#endif

  lcd.init();
  lcd.backlight();
  lcd.print(F("Demostrador v2.2"));

  if (!ads.begin(0x48)) Serial.println(F("[ERROR] no se encuentra el ADS1115 #1 (0x48)"));
  ads.setGain(GAIN_TWOTHIRDS);         // +/-6,144 V: admite salidas de hasta 5 V
  if (HAY_ORP_OZONO) {
    ads2Ok = ads2.begin(0x49);
    if (ads2Ok) ads2.setGain(GAIN_TWOTHIRDS);
    else Serial.println(F("[ERROR] no se encuentra el ADS1115 #2 (0x49): el traspaso queda bloqueado"));
  }

  ds.begin();
  ds.setResolution(10);

  cargarCalibracion();
  reiniciarSecuencia();
  tRapido = millis();
  tEv4 = millis();
  Serial.println(F("[INICIO] secuencia cada 5 min, 30 s entre sensores"));

#if USAR_WATCHDOG
  wdt_enable(WDTO_4S);                 // si el bucle se traba 4 s, reinicia en estado seguro
#endif
}

void loop() {
#if USAR_WATCHDOG
  wdt_reset();
#endif
  unsigned long ahora = millis();

  leerComandos();
  leerPulsador(ahora);

  unsigned long dt = ahora - tRapido;
  if (dt >= 1000UL) {
    tRapido = ahora;
    tareaRapida(ahora, dt);
  }

  gestionEstado(ahora);
  gestionCal(ahora);
  secuencia(ahora);

  if (ahora - tLcd >= 1000UL) {
    tLcd = ahora;
    mostrarLcd(ahora);
  }
}
