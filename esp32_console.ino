
const int PIN_X    = 34;
const int PIN_Y    = 35;
const int PIN_Z    = 32;
const int PIN_BTN1 = 25;
const int PIN_BTN2 = 26;
const int PIN_MODE = 27;

const int ADC_MAX = 4095;      
int centerX = ADC_MAX / 2;     
int centerY = ADC_MAX / 2;
int centerZ = ADC_MAX / 2;
const int DEADZONE = 80;       

unsigned long lastSend = 0;
const unsigned long SEND_PERIOD_MS = 20; 

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

 
  int btn1 = (digitalRead(PIN_BTN1) == LOW) ? 1 : 0;
  int btn2 = (digitalRead(PIN_BTN2) == LOW) ? 1 : 0;


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
