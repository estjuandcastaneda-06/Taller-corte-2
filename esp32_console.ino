/*
  esp32_console.ino
  ------------------
  Firmware de la consola de mandos física para el taller "Real-to-Sim".
  La MISMA consola se usa para las 3 partes (drones, Baxter, Atlas):
  el PC decide qué hacer con los datos según el modo que el propio ESP32
  reporta (interruptor MODE_PIN).

  HARDWARE (ajustar pines según tu montaje):
    - 2 joysticks analógicos (o 3 potenciómetros) -> ejes X, Y, Z
        EJE_X -> GPIO34 (ADC1_CH6)
        EJE_Y -> GPIO35 (ADC1_CH7)
        EJE_Z -> GPIO32 (ADC1_CH4)   (p.ej. potenciómetro deslizante para altura/Z)
    - Botón 1 (BTN1): siguiente punto A/B/C (drones) / agarrar-soltar (Baxter) /
                      siguiente postura (Atlas)
        -> GPIO25, a GND, con INPUT_PULLUP
    - Botón 2 (BTN2): modo automático on/off (drones) / cambiar brazo (Baxter) /
                      cambiar parte del cuerpo (Atlas)
        -> GPIO26, a GND, con INPUT_PULLUP
    - Joystick: VCC -> 3V3 (¡NO a 5V, el ADC del ESP32 es de 3.3 V!), GND -> GND.
      Al encender, NO toques el joystick durante ~1 s: se calibra el centro.
    - Interruptor de modo (MODE_PIN): selecciona a qué script va dirigido
        -> GPIO27, a GND, con INPUT_PULLUP (abierto = "D" drones, cerrado = "B" Baxter/Atlas)

  PROTOCOLO (ver common/serial_bridge.py en el lado de Python):
    Se envía una línea de texto por Serial a 115200 baudios, cada ~20 ms:

        MODE,X,Y,Z,BTN1,BTN2\n

    X, Y, Z están normalizados en [-1.00, 1.00] con 2 decimales
    (0.00 = centro / reposo del joystick).
*/

const int PIN_X    = 34;
const int PIN_Y    = 35;
const int PIN_Z    = 32;
const int PIN_BTN1 = 25;
const int PIN_BTN2 = 26;
const int PIN_MODE = 27;

const int ADC_MAX = 4095;      // resolución del ADC del ESP32 (12 bits)
int centerX = ADC_MAX / 2;     // se recalibran en setup()
int centerY = ADC_MAX / 2;
int centerZ = ADC_MAX / 2;
const int DEADZONE = 80;       // zona muerta alrededor del centro, evita ruido/deriva

unsigned long lastSend = 0;
const unsigned long SEND_PERIOD_MS = 20; // 50 Hz

int readAveraged(int pin) {
  long sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(pin);
  }
  return sum / 8;
}

float readAxisNormalized(int pin, int center) {
  int centered = readAveraged(pin) - center;
  if (abs(centered) < DEADZONE) {
    return 0.0f;
  }
  // se normaliza por separado cada lado, porque el centro real no es exactamente 2047
  float span = (centered > 0) ? (float)(ADC_MAX - center) : (float)center;
  float norm = (float)centered / span;
  if (norm > 1.0f) norm = 1.0f;
  if (norm < -1.0f) norm = -1.0f;
  return norm;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);
  pinMode(PIN_MODE, INPUT_PULLUP);
  analogReadResolution(12);

  // Calibración del centro del joystick (no tocarlo al encender)
  long sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < 50; i++) {
    sx += analogRead(PIN_X);
    sy += analogRead(PIN_Y);
    sz += analogRead(PIN_Z);
    delay(10);
  }
  centerX = sx / 50;
  centerY = sy / 50;
  centerZ = sz / 50;
}

void loop() {
  unsigned long now = millis();
  if (now - lastSend < SEND_PERIOD_MS) {
    return;
  }
  lastSend = now;

  float x = readAxisNormalized(PIN_X, centerX);
  float y = readAxisNormalized(PIN_Y, centerY);
  float z = readAxisNormalized(PIN_Z, centerZ);

  // INPUT_PULLUP: presionado = LOW -> lo invertimos para que 1 = presionado
  int btn1 = (digitalRead(PIN_BTN1) == LOW) ? 1 : 0;
  int btn2 = (digitalRead(PIN_BTN2) == LOW) ? 1 : 0;

  // MODE_PIN abierto (HIGH, pull-up) -> "D" (drones)
  // MODE_PIN a GND (LOW)             -> "B" (Baxter / Atlas)
  char mode = (digitalRead(PIN_MODE) == LOW) ? 'B' : 'D';

  Serial.print(mode);
  Serial.print(',');
  Serial.print(x, 2);
  Serial.print(',');
  Serial.print(y, 2);
  Serial.print(',');
  Serial.print(z, 2);
  Serial.print(',');
  Serial.print(btn1);
  Serial.print(',');
  Serial.println(btn2);
}
