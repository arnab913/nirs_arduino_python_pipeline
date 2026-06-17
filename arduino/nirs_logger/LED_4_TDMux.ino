const int detectorPin = A0;

// Use safe pins (avoid D0/D1)
const int LED_PINS[] = {2, 3, 4, 5};
const int NLEDS = sizeof(LED_PINS) / sizeof(LED_PINS[0]);

const unsigned long Ton_ms  = 50;
const unsigned long Toff_ms = 50;
const unsigned long Ts_ms   = 2;

const int discard = 3;

// latest true per LED
float lastTrue[4] = {0,0,0,0};

// capture control
bool captureRequested = false;
int captureCount = 0;
float capSum[4] = {0,0,0,0};

float adcToVolt(int adc) {
  return (adc * 5.0) / 1023.0;
}

void allLedsOff() {
  for (int i = 0; i < NLEDS; i++) digitalWrite(LED_PINS[i], LOW);
}

float avgWindow(bool ledOn, int ledIndex) {
  // ledOn=false -> all off
  // ledOn=true  -> turn on the selected LED only
  if (!ledOn) {
    allLedsOff();
  } else {
    allLedsOff();
    digitalWrite(LED_PINS[ledIndex], HIGH);
  }

  unsigned long t0 = millis();
  float sum = 0.0;
  int n = 0;
  int k = 0;

  unsigned long Tw = ledOn ? Ton_ms : Toff_ms;

  while (millis() - t0 < Tw) {
    float raw = adcToVolt(analogRead(detectorPin));
    if (k >= discard) { sum += raw; n++; }
    k++;
    delay(Ts_ms);
  }

  return (n > 0) ? (sum / n) : 0.0;
}

void setup() {
  for (int i = 0; i < NLEDS; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }
  Serial.begin(115200);
  Serial.println("LED,idx,Voff,Von,true");  // header
}

void loop() {
  // Read commands
  while (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'd' || cmd == 'D') {
      captureRequested = true;
      captureCount = 0;
      for (int i = 0; i < NLEDS; i++) capSum[i] = 0.0;
      Serial.println("# D received, capturing 3 cycles...");
    }
  }

  // One full cycle over all LEDs
  for (int i = 0; i < NLEDS; i++) {
    float Voff = avgWindow(false, i);
    float Von  = avgWindow(true,  i);
    float tru  = Von - Voff;
    lastTrue[i] = tru;

    // Stream one summary line per LED
    Serial.print("LED,");
    Serial.print(i+1);
    Serial.print(",");
    Serial.print(Voff, 6);
    Serial.print(",");
    Serial.print(Von,  6);
    Serial.print(",");
    Serial.println(tru, 6);
  }

  allLedsOff();

  // Capture averaging over 3 full cycles
  if (captureRequested) {
    for (int i = 0; i < NLEDS; i++) capSum[i] += lastTrue[i];
    captureCount++;

    if (captureCount >= 3) {
      Serial.print("CAP,");
      for (int i = 0; i < NLEDS; i++) {
        float avg = capSum[i] / 3.0;
        Serial.print(avg, 6);
        if (i < NLEDS - 1) Serial.print(",");
      }
      Serial.println();
      Serial.println("# Capture done.");
      captureRequested = false;
    }
  }
}