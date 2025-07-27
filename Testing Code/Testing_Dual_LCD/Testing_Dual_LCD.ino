#include <Wire.h>
#include <U8g2lib.h>
#include <LiquidCrystal_I2C.h>
// OLED with U8g2 (SSD1306 128x64 I2C)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0);

// 16x2 I²C LCD (change address if needed)
LiquidCrystal_I2C lcd(0x27, 16, 2);
void setup() {
  Serial.begin(115200);
  Wire.begin();  // Shared I²C bus
  
  oled.begin();  // OLED init
  lcd.begin();   // LCD init
  lcd.backlight();  // Enable LCD backlight

  // Test output
  lcd.setCursor(0, 0);
  lcd.print("Hello LCD");

  oled.clearBuffer();
  oled.setFont(u8g2_font_ncenB08_tr);
  oled.drawStr(0, 24, "Hello OLED");
  oled.sendBuffer();
}
void loop() {
  lcd.setCursor(0, 1);
  lcd.print("Time: ");
  lcd.print(millis() / 1000);

  oled.clearBuffer();
  oled.setCursor(0, 50);
  oled.print("Millis:");
  oled.print(millis() / 1000);
  oled.sendBuffer();

  delay(500);
}
