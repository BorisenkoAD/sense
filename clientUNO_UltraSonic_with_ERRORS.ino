#include <Ethernet.h>

#define KASSA 21        // номер кассы
#define TIMEOUT 200     // задержка в секундах между срабатываниями УЗД
                        // задержка была 500 мс, но из-за ф-ции сброса
                        // таймера - в нем 300 мс - заменил на 200 мс
#define PORT 2200       // порт для подключения к удаленному компу
#define DIST_MAX_1 10   //срабатывание c 10 см
#define SERIAL_SPEED 9600 // скорость работы Serial

#define ERROR_NOERROR 0
#define ERROR_1 1
#define ERROR_2 2

#define TRIG_1 2
#define ECHO_1 3

#define LED_PIN_ACTIVE      18       // А5 индикация нормальной работы устройства (моргает значит не зависло)
#define LED_PIN_ERROR_SENSE 16       // А2 индикация ошибки - нет датчика(ошибка датчика)
#define LED_PIN_ERROR_LAN   15       // А0 индикация ошибки - нет сети

#define BLINK_INTERVAL  1000UL      // интервал между включение/выключением светодиода (1 секунда)

int US_status_1 = LOW;              // изначально флаг срабатывания УЗ неактивен
int pulsePin = 9;                   // дудим в 9ю ногу для сброса счетчика в WDT
byte ErrorState = ERROR_NOERROR;    // изначально ошибок нет.

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

IPAddress subnet(255, 255, 252, 0);
IPAddress ip(192, 168, 4, KASSA);      // ip устройства для 55 кассы (100я касса для теста)
IPAddress server(192, 168, 5, 85);
IPAddress Dns(192, 168, 5, 139);
IPAddress gateway(192, 168, 5, 139);

EthernetClient client;

void setup() {
  pinMode(TRIG_1, OUTPUT);              //инициируем как выход
  pinMode(ECHO_1, INPUT);               //инициируем как вход

  DDRC |= (1 << 0);                       // LED_PIN_ACTIVE как выход
  DDRC |= (1 << 2);                       // LED_PIN_ERROR_SENSE как выход
  DDRC |= (1 << 5);                       // LED_PIN_ERROR_LAN как выход
  //  pinMode(LED_PIN_ACTIVE, OUTPUT);      //индикация ошибок выключена т.е. негорит
  //  pinMode(LED_PIN_ERROR_SENSE, OUTPUT); //индикация ошибок выключена т.е. негорит
  //  pinMode(LED_PIN_ERROR_LAN, OUTPUT);   //индикация ошибок выключена т.е. негорит
  //  blinkLedTest (LED_PIN_ACTIVE, LED_PIN_ERROR_SENSE, LED_PIN_ERROR_LAN);
  blinkLedTest ();
  Serial.begin(SERIAL_SPEED);
  Ethernet.begin(mac, ip, Dns, gateway, subnet);
  delay(1000);

  Serial.println("Connecting...");
  while (client) {
    ; // ожидаем подключения клиента
  }
  Serial.println("Arduino reset");
}

void loop() {
  // heartbeat(); перенес в фрагмент отправка кода на сервер
  blinkLed(BLINK_INTERVAL);                       // мигаем зеленым если все в порядке
  static unsigned long previousMillis = 0;        // храним время последнего переключения светодиода
  static int distance_sm_1;
  distance_sm_1 = usRead(TRIG_1, ECHO_1) ;        // возвращаем дистанцию
  //  distance_sm_1 = ultrasonic.Ranging(CM);        // возвращаем дистанцию
  //  Serial.println(distance_sm_1);
  if (distance_sm_1 < DIST_MAX_1) {
    if (US_status_1 == LOW) {
      Serial.println("US#1 Motion detected!");
      ////////////////данные датчика на сервер НАЧАЛО///////////////////////
      if (client.connect(server, PORT)) {
        Serial.println("US#1 connected");
        ErrorState = ERROR_NOERROR;
        //        client.print(KASSA);
        client.println("21a1");
        client.stop();
        heartbeat();
      } else {
        Serial.println("US#1 connection failed");

        if (ErrorState == ERROR_NOERROR)
        {
          ErrorState = ERROR_1;
        }
        else if (ErrorState == ERROR_1)
        {
          ErrorState = ERROR_1;
        }
      }
      ////////////////данные датчика на сервер КОНЕЦ ///////////////////////
      US_status_1 = HIGH;
    }
  } else {
    if (US_status_1 == HIGH) {
      Serial.println("US#1 Motion ended!");
      US_status_1 = LOW;
    }
  }
  delay(TIMEOUT);
  ////////////////////// Конец обработки первого датчика  ///////////////
  switch (ErrorState) {
    case ERROR_1:
      // подаем напругу на LED_PIN_ERROR_LAN (горит красный)
      digitalWrite(LED_PIN_ERROR_LAN, HIGH);
      break;

    case ERROR_2:
      // подаем напругу на LED_PIN_ERROR_SENCE (горит желтый)
      digitalWrite(LED_PIN_ERROR_SENSE, HIGH);
      break;

    case ERROR_NOERROR:
      // отключаем напругу с LED_PIN_ERROR (погас красный)
      digitalWrite(LED_PIN_ERROR_LAN, LOW);
      break;

    default:
      // очевидно что-то понапутано с алгоритмом
      // и ErrorState приняла непредвиденное значение.
      // Дадим об этом знать, например поморгаем красным,
      // а заодно все повесем на 1,5 секунды ибо нехуй.

      digitalWrite(LED_PIN_ERROR_LAN, HIGH);
      delay(1000);
      digitalWrite(LED_PIN_ERROR_LAN, LOW);
      delay(500);
  }
}

int usRead(byte Trig, byte Echo)        // принимаю  - byte
{
  static unsigned int impulseTime = 0;   // добавил static
  static unsigned int distance_sm = 0;   // добавил static
  digitalWrite(Trig, HIGH);           // Подаем импульс на вход trig дальномера
  delayMicroseconds(9);              // равный 10 микросекундам
  digitalWrite(Trig, LOW);            // Отключаем
  impulseTime = pulseIn(Echo, HIGH);  // Замеряем длину импульса
  distance_sm = impulseTime / 58;     // Пересчитываем в сантиметры
  return distance_sm;
}
// работа индикатора АКТИВНОСТЬ
void blinkLed(unsigned long interval ) {
  static unsigned long prevTime = 0; // время когда последний раз переключали диод
  if (millis() - prevTime > interval) {
    prevTime = millis();  //
    digitalWrite(LED_PIN_ACTIVE, !digitalRead(LED_PIN_ACTIVE));
  }
}
// функция первичного теста светодиодов
//void blinkLedTest(byte led1, byte led2, byte led3){
void blinkLedTest() {
  for (int i = 0; i < 3; i++) {
    PORTC &= ~(1 << 0);
    PORTC &= ~(1 << 2);
    PORTC &= ~(1 << 5);
    _delay_ms(500);      // ждем 500 миллисекунд
    PORTC |= (1 << 0);  // устанавливаем высокий уровень на выводе PB5
    PORTC |= (1 << 2);
    PORTC |= (1 << 5);
    _delay_ms(500);      // ждем 500 миллисекунд
    PORTC &= ~(1 << 0);
    PORTC &= ~(1 << 2);
    PORTC &= ~(1 << 5);
    _delay_ms(500);      // ждем 500 миллисекунд
  }
}
// функция сброса таймера для WDT
void heartbeat() {
  pinMode(pulsePin, OUTPUT);
  digitalWrite(pulsePin, LOW);
  delay(300);
  // Return to high-Z
  pinMode(pulsePin, INPUT);
  Serial.println("Heartbeat sent");
}
