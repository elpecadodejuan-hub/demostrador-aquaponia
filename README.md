# demostrador-aquaponia
Demostrador de aquaponía v2.2 para Arduino Mega 2560
# Demostrador de Aquaponía v2.2

Sistema didáctico de aquaponía controlado por **Arduino Mega 2560**.  
Incluye lectura secuencial de sensores, ciclo automático de ozono con electroválvulas, permisivos de seguridad y calibración persistente en EEPROM.

## Características principales

- Lectura secuencial de sensores cada 30 s (ciclo completo \~5 min)
- Alimentación conmutada de sondas (pH, ORP, EC, turbidez)
- Ciclo automático de ozono con estados:  
  `NORMAL → LLENADO → OZONO → REPOSO → TRASPASO → NORMAL`
- Enclavamiento duro: EV1 y EV2 nunca abren al mismo tiempo
- Permisivos estrictos para el traspaso (ORP, niveles, alarma de ozono en aire)
- Quinto flotador para el depósito de mediterráneas (EV4)
- Calibración de pH, ORP (lazo y recipiente) y humedad de suelo en EEPROM
- LCD 16×2 con 3 páginas de información
- Comandos por monitor serie + pulsador físico

## Hardware

| Componente              | Función                                      | Pines / Dirección      |
|-------------------------|----------------------------------------------|------------------------|
| Arduino Mega 2560       | Cerebro                                      | -                      |
| ADS1115 #1              | pH, ORP lazo, TDS, turbidez                  | 0x48                   |
| ADS1115 #2              | ORP recipiente de ozono                      | 0x49                   |
| DS18B20                 | Temperatura del agua                         | Pin 4                  |
| YF-S201                 | Caudalímetro                                 | Pin 2 (interrupción)   |
| MQ-131                  | Alarma de ozono en aire                      | A1                     |
| Sensor humedad capacitivo | Humedad de suelo (mediterráneas)           | A0                     |
| Flotadores ×5           | Niveles de pecera, recipiente ozono y depósito | 23-27                |
| Módulo 8 relés          | Bomba, UV, ozono, aire, EV1-EV4              | 30-37                  |
| Alimentación sensores   | Conmutación de sondas (MOSFET en turbidez)   | 40-43                  |
| LCD 16×2 I2C            | Visualización                                | 0x27                   |
| Pulsador                | Inicio de ciclo de ozono                     | 22                     |

## Librerías necesarias

Instalar desde el Gestor de librerías del Arduino IDE:

- Adafruit ADS1X15
- OneWire
- DallasTemperature
- LiquidCrystal I2C (Frank de Brabander)

## Comandos (monitor serie 115200)

| Comando     | Acción                                      |
|-------------|---------------------------------------------|
| `o`         | Iniciar ciclo de ozono                      |
| `x`         | Abortar ciclo actual                        |
| `t`         | Traspaso manual (solo si se cumplen permisivos) |
| `r`         | Resetear alarmas y bloqueos                 |
| `s`         | Mostrar estado completo                     |
| `p7` / `p4` | Calibrar pH con buffer 7,00 / 4,01          |
| `orp <mV>`  | Calibrar ORP del lazo                       |
| `orpo <mV>` | Calibrar ORP del recipiente de ozono        |
| `seco` / `mojado` | Calibrar humedad de suelo              |
| `cal`       | Ver valores de calibración actuales         |
| `borrar`    | Restaurar valores de fábrica                |

## Ciclo de ozono y seguridad

El lazo principal (bomba + UV) sigue funcionando en todos los estados.

**Permisivos para abrir EV2 (traspaso):**
- Sin alarma de ozono en aire (MQ-131)
- Ozonizador apagado
- Hay agua en el recipiente de ozono
- La pecera no está llena
- ORP del recipiente bajó lo suficiente respecto al valor inicial

EV1 y EV2 tienen enclavamiento hardware/software: nunca se abren juntas.

## Licencia

Proyecto didáctico. Usalo, modificalo y compartilo libremente.
