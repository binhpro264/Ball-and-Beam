#include <Servo.h>

// ===== PIN =====
#define TRIG_IN 6
#define ECHO_IN 5

#define TRIG_SP 8                 
#define ECHO_SP 7

#define SERVO_PIN 9

Servo servo;

// ===== SYSTEM =====
int center = 90;
int maxAngle = 30;

float tolerance = 0.8;

// ===== PID =====
float Kp = 4;
float Ki = 0.3;
float Kd = 2.5;

float prevError = 0;
float integral = 0;

// ===== FILTER =====
float inputFilt = 0;
float spFilt = 0;
float alpha = 0.5;

// ===== READ SENSOR =====
float readUS(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long t = pulseIn(echo, HIGH, 3000);
  if (t == 0) return -1;

  return t * 0.034 / 2.0;
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_IN, OUTPUT);
  pinMode(ECHO_IN, INPUT);

  pinMode(TRIG_SP, OUTPUT);
  pinMode(ECHO_SP, INPUT);

  servo.attach(SERVO_PIN);
  servo.write(center);
}

void loop() {
  float rawIn = readUS(TRIG_IN, ECHO_IN);
  float rawSP = readUS(TRIG_SP, ECHO_SP);

  if (rawIn < 0 || rawSP < 0) return;

  float input = rawIn;
  float setpoint = rawSP;

  float error = setpoint - input;

  // ===== DEADZONE (ĐÃ SỬA: Không dùng return để tránh ngắt Serial) =====
  if (abs(error) < tolerance) {
    error = 0; 
    integral = 0; // Xóa tích phân để tránh tích lũy sai số khi đã đứng yên
  }

  // ===== PID =====
  integral += error;

  float derivative = error - prevError;

  float control = Kp * error + Ki * integral + Kd * derivative;

  prevError = error;

  // ===== LIMIT =====
  if (control > maxAngle) control = maxAngle;
  if (control < -maxAngle) control = -maxAngle;

  int angle = center - (int)control;

  servo.write(angle);

  // ===== IN ĐỒ THỊ (ĐÃ SỬA: Gom đồ thị thu nhỏ lại 10 lần và reset bộ đếm) =====
  static int plot_counter = 0;
  plot_counter++;

  if (plot_counter >= 20) { // Cứ 10 chu kỳ mới gửi lên máy tính 1 lần
    Serial.print("Setpoint:");
    Serial.print(setpoint);
    Serial.print("\t"); 

    Serial.print("Input:");
    Serial.print(input);
    Serial.print("\t");

    Serial.print("Error:");
    Serial.print(error);
    Serial.print("\t");

    Serial.print("Control:");
    Serial.println(control); 

    plot_counter = 0; // Reset bộ đếm về 0 để tiếp tục chu kỳ mới
  }

  delay(5);
}