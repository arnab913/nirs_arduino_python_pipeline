const int LED = 3;
const int detectorPin = A0;

const unsigned long Ton_ms  = 500;
const unsigned long Toff_ms = 500;
const unsigned long Ts_ms   = 10;

const int discard = 3;

float lastTrue = 0.0;

// capture control
bool captureRequested = false;
int captureCount = 0;
double captureSum = 0.0;

float adcToVolt(int adc) {
  return (adc * 5.0) / 1023.0;
}

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  Serial.begin(115200);
  Serial.println("raw,true");
}

void loop() {

  // ---- robust serial read: consume all chars ----
  while (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'd' || cmd == 'D') {
      captureRequested = true;
      captureCount = 0;
      captureSum = 0.0;
      Serial.println("# D received, capturing 3 true values...");
    }
  }

  // ---------------- OFF window ----------------
  digitalWrite(LED, LOW);
  unsigned long t0 = millis();

  float sumOff = 0.0;
  int nOff = 0;
  int k = 0;

  while (millis() - t0 < Toff_ms) {
    float raw = adcToVolt(analogRead(detectorPin));

    Serial.print(raw, 4);
    Serial.print(",");
    Serial.println(lastTrue, 4);

    if (k >= discard) { sumOff += raw; nOff++; }
    k++;

    delay(Ts_ms);
  }

  float Voff_avg = (nOff > 0) ? (sumOff / nOff) : 0.0;

  // ---------------- ON window ----------------
  digitalWrite(LED, HIGH);
  t0 = millis();

  float sumOn = 0.0;
  int nOn = 0;
  k = 0;

  while (millis() - t0 < Ton_ms) {
    float raw = adcToVolt(analogRead(detectorPin));

    Serial.print(raw, 4);
    Serial.print(",");
    Serial.println(lastTrue, 4);

    if (k >= discard) { sumOn += raw; nOn++; }
    k++;

    delay(Ts_ms);
  }

  float Von_avg = (nOn > 0) ? (sumOn / nOn) : 0.0;

  // Update true once per full cycle
  lastTrue = Von_avg - Voff_avg;

  // ---- capture next 3 lastTrue updates ----
  if (captureRequested) {
    captureSum += lastTrue;
    captureCount++;

    if (captureCount >= 3) {
      float trueAvg = (float)(captureSum / 3.0);
      Serial.print("CAP,");
      Serial.println(trueAvg, 6);
      Serial.println("# Capture done.");
      captureRequested = false;
    }
  }

  digitalWrite(LED, LOW);
}
