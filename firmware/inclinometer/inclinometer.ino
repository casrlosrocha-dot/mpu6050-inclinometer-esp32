#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// ----- Pines I2C ESP32 -----
const int PIN_SDA = 21;
const int PIN_SCL = 22;

// Direcciones I2C
const uint8_t LCD_ADDR = 0x27;
const uint8_t MPU_ADDR = 0x68;

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// ==============================================================
// 1. CALIBRACIÓN Y DESFASE DE ÁNGULO
// ==============================================================
// Ajusta este valor si al poner el sensor en 0° marca unos grados de desviación.
const float OFFSET_ANGULO = 0.0; 

// Puntos de control ideales en grados (Valores estándar)
const float CAL_PUNTO_0   = 0.0;
const float CAL_PUNTO_90  = 90.0;
const float CAL_PUNTO_180 = 180.0;
const float CAL_PUNTO_270 = 270.0;
const float CAL_PUNTO_360 = 360.0;

const float TOLERANCIA_PUNTO = 3.0; // Margen de tolerancia (+/- grados)

// Variables de tiempo
unsigned long previo_ms = 0;
const unsigned long INTERVALO_MS = 250;

void iniciarMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Registro PWR_MGMT_1
  Wire.write(0x00); // Despertar sensor
  Wire.endTransmission(true);
}

void leerAceleracion(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)6, true);

  if (Wire.available() >= 6) {
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
  }
}

void setup() {
  Serial.begin(115200);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(50000);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Inclinometro G  ");
  lcd.setCursor(0, 1);
  lcd.print("Iniciando MPU...");

  iniciarMPU();
  delay(1000);
  lcd.clear();
}

void loop() {
  unsigned long actual_ms = millis();

  if (actual_ms - previo_ms >= INTERVALO_MS) {
    previo_ms = actual_ms;

    int16_t raw_ax = 0, raw_ay = 0, raw_az = 0;
    leerAceleracion(raw_ax, raw_ay, raw_az);

    float ax = (float)raw_ax / 16384.0;
    float ay = (float)raw_ay / 16384.0;
    float az = (float)raw_az / 16384.0;

    // 1. Ángulo crudo/bruto según la gravedad
    float radianes = atan2(ay, ax);
    float angulo_bruto = radianes * (180.0 / M_PI);

    if (angulo_bruto < 0) {
      angulo_bruto += 360.0;
    }

    // --- CORRECCIÓN DE SENTIDO DE GIRO ---
    // Invierte el sentido de rotación para que 90° y 270° queden correctos
    angulo_bruto = 360.0 - angulo_bruto;
    if (angulo_bruto >= 360.0) {
      angulo_bruto -= 360.0;
    }

    // 2. Aplicar offset de calibración y normalizar a rango 0 - 360°
    float angulo = angulo_bruto - OFFSET_ANGULO;
    angulo = fmodf(angulo, 360.0);
    if (angulo < 0) {
      angulo += 360.0;
    }

    // 3. Identificación del punto de control más cercano
    String estado_control = "Intermedio";
    if (fabs(angulo - CAL_PUNTO_0) <= TOLERANCIA_PUNTO || fabs(angulo - CAL_PUNTO_360) <= TOLERANCIA_PUNTO) {
      estado_control = "P0 / 360";
    } else if (fabs(angulo - CAL_PUNTO_90) <= TOLERANCIA_PUNTO) {
      estado_control = "Punto 90";
    } else if (fabs(angulo - CAL_PUNTO_180) <= TOLERANCIA_PUNTO) {
      estado_control = "Punto 180";
    } else if (fabs(angulo - CAL_PUNTO_270) <= TOLERANCIA_PUNTO) {
      estado_control = "Punto 270";
    }

    // Formateo para pantalla LCD
    char buffer_l1[17];
    char buffer_l2[17];

    snprintf(buffer_l1, sizeof(buffer_l1), "Ang: %5.1f deg  ", angulo);
    snprintf(buffer_l2, sizeof(buffer_l2), "Zona: %-10s", estado_control.c_str());

    lcd.setCursor(0, 0);
    lcd.print(buffer_l1);
    lcd.setCursor(0, 1);
    lcd.print(buffer_l2);

    // Salida Serial para verificación
    Serial.print("Ang_Bruto: "); Serial.print(angulo_bruto, 1);
    Serial.print(" | Ang_Calibrado: "); Serial.print(angulo, 1);
    Serial.print(" deg | Estado: "); Serial.println(estado_control);
  }
}
