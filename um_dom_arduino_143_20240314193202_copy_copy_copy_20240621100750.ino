#include <Wire.h>
#include <Adafruit_HTU21DF.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>


//#include <EEPROM.h>
//#define EEPROM_ADDR 0
/////#include "GyverButton.h"
//GButton myButt(12, LOW_PULL, NORM_OPEN);

#define servo_invert 1       // если серва крутится не в ту сторону, изменить значение (1 на 0, 0 на 1)
#define battery_min 3000     // минимальный уровень заряда батареи для отображения
#define battery_max 4200     // максимальный уровень заряда батареи для отображения
// диапазон для 3 пальчиковых/мизинчиковых батареек: 3000 - 4700
// диапазон для одной банки литиевого аккумулятора: 3000 - 4200
//-----------------------НАСТРОЙКИ---------------------

#define servo_Vcc 12           // пин питания, куда подключен мосфет

//------ИНВЕРСИЯ------
#if servo_invert == 1
#define servo_180 0
#define servo_0 180
#else
#define servo_0 0
#define servo_180 180
#endif
//------ИНВЕРСИЯ------

//------БИБЛИОТЕКИ------
#include <Servo.h>             // библиотека серво
// вспомогательная библиотека датчика
#include <Adafruit_BMP085.h>   // библиотека датчика
       // библиотека сна
//------ИНВЕРСИЯ------

#include "GyverNTC.h"
GyverNTC ntc (A1, 9970, 3950, 25, 10000); //изменить второй пункт под мой резистр
unsigned long tmrq = millis();
boolean tem = false;

boolean wake_flag, move_arrow;
int sleep_count, angle, delta, last_angle = 90;
float k = 0.8;
float my_vcc_const = 1.080;    // константа вольтметра
unsigned long pressure, aver_pressure, pressure_array[6], time_array[6];
unsigned long sumX, sumY, sumX2, sumXY;
unsigned long obnoq;
unsigned long maxq;
unsigned long cc;
float a, b;
boolean butt_flagq = 0;
boolean buttq;
boolean led_flagq = 0;
//boolean myBu = 0;
unsigned long last_pressq;
float temp_in = ntc.getTempAverage();
float temp_in_MAX = 0;
unsigned long wake;
int Pres_predict = 0;
byte w = 0;


Servo servo;
Adafruit_BMP085 bmp; //объявить датчик с именем bmp



boolean f_flag = false;
boolean butt_flag = false;
boolean butt;
  unsigned long last_press;
  unsigned long tmr;
  unsigned long obno;
  unsigned long max;
  unsigned long epr;
  float humidity_MAX = 0;
  float temp_MAX = 0;
  float temp, humidity;
  int Lpg_MAX = 0;
  int Co_MAX = 0;
  int Smoke_MAX = 0;
  int Co2_MAX = 0;
  int Lpg, Co, Smoke, Co2;
  float temp_out = bmp.readTemperature();
  float temp_out_MAX;

  boolean LED_PIN_RK = false;
  boolean LED_PIN_GK = false;
  boolean LED_PIN_BK = false;


#define TFT_CS 9
#define TFT_RST 7
#define TFT_DC 8
#define MQ2_PIN A0
#define BUZZER_PIN 3
#define LED_PIN_R 2 //красный светодиод
#define LED_PIN_B 4 //синий светодиод
#define LED_PIN_G 5 //зелёный светодиод
#include <MQ2.h>
#include <GyverPower.h>



Adafruit_HTU21DF htu = Adafruit_HTU21DF();
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
#define PIN_MQ2  A0
MQ2 mq2(PIN_MQ2);


// среднее арифметичсекое от давления
long aver_sens() {
  pressure = 0;
  for (byte i = 0; i < 10; i++) {
    pressure += bmp.readPressure();
  }
  aver_pressure = pressure / 10;
  return aver_pressure;
}



void setup() {

//  if (EEPROM.read(2) !='w') { //если ключ не совпал
///EEPROM.put(EEPROM_ADDR, Pres_predict);//пишем предыдущие настройки
//EEPROM.write(2,'w');//пишем правельный ключ
//  }
//  else {
//    EEPROM.get(EEPROM_ADDR, Pres_predict);//иначе читаем настройки
//  }

    pinMode(12, INPUT_PULLUP);// кнопка от свет. диода давления
  pinMode(10, OUTPUT);// светодиод
  pinMode(A7, OUTPUT);//моторчик
  pinMode(A6, INPUT);
  Serial.begin(9600);
  tft.setTextSize(1);
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST7735_BLACK);
  tft.setTextColor(ST7735_WHITE);
  pinMode(servo_Vcc, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A2, OUTPUT);
  digitalWrite(A3, 1);             // подать питание на датчик
  digitalWrite(A2, 0);
  bmp.begin(BMP085_ULTRAHIGHRES);  // включить датчик

  pressure = aver_sens();          // найти текущее давление по среднему арифметическому
  for (byte i = 0; i < 6; i++) {   // счётчик от 0 до 5
    pressure_array[i] = pressure;  // забить весь массив текущим давлением
    time_array[i] = i;             // забить массив времени числами 0 - 5
  }


 
  tft.setTextSize(1);
  Serial.begin(9600);
  htu.begin();
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST7735_BLACK);
  tft.setTextColor(ST7735_WHITE);
  pinMode(MQ2_PIN, INPUT);
  pinMode(6, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN_R, OUTPUT); //красный светодиод
  pinMode(LED_PIN_B, OUTPUT); //синий светодиод
  pinMode(LED_PIN_G, OUTPUT); //зелёный светодиод
  mq2.begin();
}

void loop() {
if (wake_flag) {
    pressure = aver_sens();                          // найти текущее давление по среднему арифметическому
    for (byte i = 0; i < 5; i++) {                   // счётчик от 0 до 5 (да, до 5. Так как 4 меньше 5)
      pressure_array[i] = pressure_array[i + 1];     // сдвинуть массив давлений КРОМЕ ПОСЛЕДНЕЙ ЯЧЕЙКИ на шаг назад
    }
    pressure_array[5] = pressure;                    // последний элемент массива теперь - новое давление

    sumX = 0;
    sumY = 0;
    sumX2 = 0;
    sumXY = 0;
    for (int i = 0; i < 6; i++) {                    // для всех элементов массива
      sumX += time_array[i];
      sumY += (long)pressure_array[i];
      sumX2 += time_array[i] * time_array[i];
      sumXY += (long)time_array[i] * pressure_array[i];
    }
    a = 0;
    a = (long)6 * sumXY;             // расчёт коэффициента наклона приямой
    a = a - (long)sumX * sumY;
    a = (float)a / (6 * sumX2 - sumX * sumX);
    // Вопрос: зачем столько раз пересчитывать всё отдельными формулами? Почему нельзя считать одной большой?
    // Ответ: а затем, что ардуинка не хочет считать такие большие числа сразу, и обязательно где-то наё*бывается,
    // выдавая огромное число, от которого всё идёт по пи*зде. Почему с матами? потому что устал отлаживать >:O
    delta = a * 6;                   // расчёт изменения давления

    angle = map(delta, -250, 250, servo_0, servo_180);  // пересчитать в угол поворота сервы
    angle = constrain(angle, 0, 180);                   // ограничить диапазон

    Pres_predict = map(delta, -250, 250, -100, 100);
    Pres_predict = constrain(Pres_predict, -100, 100);       

    // дальше такая фишка: если угол несильно изменился с прошлого раза, то нет смысла лишний раз включать серву
    // и тратить энергию/жужжать. Так что находим разницу, и если изменение существенное - то поворачиваем стрелку    
    if (abs(angle - last_angle) > 7) move_arrow = 1;

    if (move_arrow) {
      last_angle = angle;
      move_arrow = 0;
    }


    wake_flag = 0;
                    // задержка для стабильности
  }
  if (millis() - cc >= 8000) {

  sleep_count++;            // +1 к счетчику просыпаний
    cc = millis();
  }
  if (sleep_count >= 74) {  // если время сна превысило 10 минут (75 раз по 8 секунд - подгон = 70)
    wake_flag = 1;          // рарешить выполнение расчета
    sleep_count = 0;        // обнулить счетчик
                // задержка для стабильности
  }















     // myButt.tick();

//if (millis() - epr >= 7200000) {
//  EEPROM.put(EEPROM_ADDR, angle);
//}

//EEPROM.get(EEPROM_ADDR, Pres_predict);

if (millis() - tmrq >= 400) {
NTC();
tmrq = millis();
}




  temp_in = ntc.getTempAverage();

  butt = !digitalRead(6);
  LED_PIN_GK = digitalRead (LED_PIN_G);
  LED_PIN_RK = digitalRead (LED_PIN_R);

  power.hardwareEnable(PWR_ADC);

       float* values = mq2.read(true);

 humidity = htu.readHumidity();
 temp = htu.readTemperature();
 Lpg = mq2.readLPG();
 Co = mq2.readCO();
 Smoke = mq2.readSmoke();
 Co2 = analogRead(PIN_MQ2);


   if (temp > temp_MAX) {
  temp_MAX = temp;
  }

  if (humidity > humidity_MAX) {
  humidity_MAX = humidity;
  }

  if (Lpg > Lpg_MAX and Lpg - Lpg_MAX >= 2) {
  Lpg_MAX = Lpg;
  }

  if (Co > Co_MAX and Co - Co_MAX >= 2) {
  Co_MAX = Co;
  }

  if (Smoke > Smoke_MAX and Smoke - Smoke_MAX >= 2) {
  Smoke_MAX = Smoke;
  }

  if (Co2 > Co2_MAX and Co2 - Co2_MAX >= 2) {
  Co2_MAX = Co2;
  }

     if (ntc.getTempAverage() > temp_in_MAX) {
  temp_in_MAX = ntc.getTempAverage();
  }

         if (bmp.readTemperature() > temp_out_MAX) {
  temp_out_MAX = bmp.readTemperature();
  }



if (micros() - obno >= 4) {
  tft.setCursor(0, 0);
  tft.print("Humidity: ");
    tft.fillRect(55, 0, 73, 8, ST7735_BLACK);
  tft.print(humidity);
  tft.print(" %\n");
  tft.print("Temp: ");
    tft.fillRect(30, 8, 98, 8, ST7735_BLACK);
  tft.print(temp);
  tft.print(" C\n");
  tft.print("Lpg: ");
    tft.fillRect(25, 16, 103, 8, ST7735_BLACK);
  tft.print(Lpg);
  tft.print(" ppm\n");
  tft.print("Co: ");
      tft.fillRect(20, 24, 108, 8, ST7735_BLACK);
  tft.print(Co);
  tft.print(" ppm\n");
  tft.print("Smoke: ");
      tft.fillRect(35, 32, 93, 8, ST7735_BLACK);
  tft.print(Smoke);
  tft.print(" ppm\n");
  tft.print("Co2: ");
      tft.fillRect(25, 40, 103, 8, ST7735_BLACK);
  tft.print(Co2);
  tft.print(" ppm\n");
  tft.print("\n");

  tft.print("Humidity_MAX: ");
      tft.fillRect(75, 56, 53, 8, ST7735_BLACK);
  tft.print(humidity_MAX);
  tft.print(" %\n");
  tft.print("Temp_MAX: ");
      tft.fillRect(55, 64, 73, 8, ST7735_BLACK);
  tft.print(temp_MAX);
  tft.print(" C\n");
  tft.print("Lpg_MAX: ");
      tft.fillRect(45, 72, 83, 8, ST7735_BLACK);
  tft.print(Lpg_MAX);
  tft.print(" ppm\n");
  tft.print("Co_MAX: ");
      tft.fillRect(40, 80, 88, 8, ST7735_BLACK);
  tft.print(Co_MAX);
  tft.print(" ppm\n");
  tft.print("Smoke_MAX: ");
      tft.fillRect(60, 88, 63, 8, ST7735_BLACK);
  tft.print(Smoke_MAX);
  tft.print(" ppm\n");
  tft.print("Co2_MAX: ");
      tft.fillRect(45, 96, 83, 8, ST7735_BLACK);
  tft.print(Co2_MAX);
  tft.print(" ppm\n");
  //tft.print("\n");
  //tft.print("Pressure: ");
  //tft.print(BMP);
  //tft.print("\n");
  //tft.print("Weather: ");
  //tft.print(wet); // процент возможности дождя
  //tft.print(" %\n");

tft.setCursor(0, 112); // МОжЕТ ДОбАВЛю ПРОПУСК ОДНОЙ СТРОКИ
  tft.print("Pressure: ");
      tft.fillRect(55, 112, 73, 8, ST7735_BLACK);
  tft.print(pressure);
  tft.print(" hPa\n");
  tft.print("Pres predict: ");
        tft.fillRect(75, 120, 53, 8, ST7735_BLACK);
  tft.print(Pres_predict);
tft.print(" %\n");

tft.print("temp out: ");
      tft.fillRect(55, 128, 73, 8, ST7735_BLACK);
tft.print(bmp.readTemperature());
  tft.print(" C\n");
  tft.print("temp out_MAX: ");
        tft.fillRect(75, 136, 53, 8, ST7735_BLACK);
  tft.print(temp_out_MAX);
  tft.print(" C\n");

  tft.print("temp in: ");
      tft.fillRect(45, 144, 83, 8, ST7735_BLACK);
tft.print(ntc.getTempAverage());
  tft.print(" C\n");
  tft.print("temp in_MAX: ");
        tft.fillRect(70, 152, 58, 8, ST7735_BLACK);
  tft.print(temp_in_MAX);
  tft.print(" C\n");
//  tft.print(led_flagq);

  obno = micros();
  }
  
  if (millis() - max >= 86400000) {
  humidity_MAX = 0;
  temp_MAX = 0;
  Lpg_MAX = 0;
  Co_MAX = 0;
  Smoke_MAX = 0;
  Co2_MAX = 0;
  temp_in_MAX = 0;
  max = millis();
  }

          buttq = !digitalRead(12);
  if (buttq == true && butt_flagq == false && millis() - last_pressq > 65) {
    butt_flagq = true;
    digitalWrite(10, false);
    last_pressq = millis();
  }
  if (buttq == false && butt_flagq == true) {
    butt_flagq = false;
  }

      if (Pres_predict <= 10 and led_flagq == true) {
digitalWrite(10, false);
led_flagq = false;
      }
      else if (Pres_predict >= 20 and led_flagq == false) {
digitalWrite(10, true);
led_flagq = true;
      }

    if (butt == true && butt_flag == false && millis() - last_press > 65) {
    butt_flag = true;
    Serial.println("Button pressed");
    f_flag = !f_flag;
    digitalWrite (LED_PIN_G, LOW);
    digitalWrite (LED_PIN_R, LOW);
    digitalWrite (LED_PIN_B, f_flag); // 1 нажатие = 1. 2 нажатия = 0
    last_press = millis();
  }
  if (butt == false && butt_flag == true) {
    butt_flag = false;
    Serial.println("Button released");
  }
  LED_PIN_BK = digitalRead (LED_PIN_B);

  if (Co2 >= 1000 and LED_PIN_BK == LOW or Co >= 4 and LED_PIN_BK == LOW or Lpg >= 4 and LED_PIN_BK == LOW or Smoke >= 4  and LED_PIN_BK == LOW) {
  pinMode(BUZZER_PIN, OUTPUT);
  tone (BUZZER_PIN, 500);
  digitalWrite (LED_PIN_R, HIGH);
  digitalWrite (LED_PIN_G, LOW);
  }
  else if (LED_PIN_BK == LOW) {
  noTone(BUZZER_PIN);
  digitalWrite (BUZZER_PIN, LOW);
  pinMode(BUZZER_PIN, INPUT);
  digitalWrite (LED_PIN_R, LOW);
  digitalWrite (LED_PIN_G, HIGH);
  }
  if (LED_PIN_BK == HIGH) {
  noTone(BUZZER_PIN);
  digitalWrite (BUZZER_PIN, LOW);
  pinMode(BUZZER_PIN, INPUT);
    digitalWrite (LED_PIN_G, LOW);
    digitalWrite (LED_PIN_R, LOW);
  }
}




void NTC() {



 if (temp_in >= 40 and LED_PIN_GK == HIGH) {
analogWrite(7, 1000); // добавить плавный старт, если не будет работать
digitalWrite(LED_PIN_R, true);
pinMode(BUZZER_PIN, OUTPUT);
tone (BUZZER_PIN, 800);
tem = true;

 }

        else if (temp_in <= 35 and tem == true) {
analogWrite(7, 0);
tem = false;
//myBu = false;
digitalWrite(LED_PIN_R, false);
noTone(BUZZER_PIN);
  digitalWrite (BUZZER_PIN, LOW);
  pinMode(BUZZER_PIN, INPUT);
  }

//  if (myButt.isDouble() and tem == true) {
//analogWrite(7, 0);
//tem = false;
//myBu = true;
//digitalWrite(LED_PIN_R, false);
//noTone(BUZZER_PIN);
//  digitalWrite (BUZZER_PIN, LOW);
 // pinMode(BUZZER_PIN, INPUT);
 // }
}