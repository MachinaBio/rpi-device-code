///////////////////////////////////////////////////
// Utilty methods for Kernel Controller 
// 
// July 9, 2012
//
///////////////////////////////////////////////////

void splashScreen() {
  lcd.clear();
  lcd.print("-------------Aquinas");
  lcd.setCursor(1,0);
  lcd.print("Kernel Controller v01");
  lcd.setCursor(2,5);
  lcd.print("DOI 1000.32/1000");
  lcd.setCursor(3,6);
  lcd.print("by CJQ at UCSD");
  delay(2000);
}

void drawScreen1() {
  lcd.clear();
  lcd.print("Air");
  lcd.setCursor(0,13);
  lcd.print("Set");
  lcd.setCursor(1,0);
  lcd.print("Tub");
  lcd.setCursor(1,13);
  lcd.print("Gas");
  lcd.setCursor(2,0);
  lcd.print("Hum");
  lcd.setCursor(3,0);
  lcd.print("Obj");
}

void drawScreen2() {
  lcd.clear();
  lcd.print("Air");
  lcd.setCursor(0,13);
  lcd.print("Set");
  lcd.setCursor(1,0);
  lcd.print("Sub");
  lcd.setCursor(1,13);
  lcd.print("Ports");
  lcd.setCursor(2,0);
  lcd.print("Brd");
  lcd.setCursor(2,9);
  lcd.print("A=");
  lcd.setCursor(3,0);
  lcd.print("Dew");
  lcd.setCursor(3,9);
  lcd.print("B=");
}
