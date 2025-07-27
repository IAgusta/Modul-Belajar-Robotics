// BFD-1000 Sensor Pin Mapping
const int irLeft2  = 18;
const int irLeft1  = 5;
const int irCenter = 17;
const int irRight1 = 16;
const int irRight2 = 4;

void setup() {
  Serial.begin(115200);

  // Initialize all sensor pins as input
  pinMode(irLeft2, INPUT);
  pinMode(irLeft1, INPUT);
  pinMode(irCenter, INPUT);
  pinMode(irRight1, INPUT);
  pinMode(irRight2, INPUT);

  Serial.println("BFD-1000 Sensor Test Start");
}

void loop() {
  int sLeft2  = digitalRead(irLeft2);
  int sLeft1  = digitalRead(irLeft1);
  int sCenter = digitalRead(irCenter);
  int sRight1 = digitalRead(irRight1);
  int sRight2 = digitalRead(irRight2);

  // Print values in order: L2 L1 C R1 R2
  Serial.print("Sensors: ");
  Serial.print(sLeft2); Serial.print(" ");
  Serial.print(sLeft1); Serial.print(" ");
  Serial.print(sCenter); Serial.print(" ");
  Serial.print(sRight1); Serial.print(" ");
  Serial.println(sRight2);

  delay(200); // Short delay for readability
}
