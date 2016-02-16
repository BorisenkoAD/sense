#include <Ethernet.h>

#define KASSA 49       // номер кассы
#define TIMEOUT 150     // задержка в секундах между срабатываниями УЗД
#define PORT 2200       // порт для подключения к удаленному компу
#define TRIG_1 2
#define ECHO_1 3
#define DIST_MAX_1 10     //срабатывание c 10 см
#define SERIAL_SPEED 9600 // скорость работы Serial

#define ERROR_NOERROR 0
#define ERROR_1 1
#define ERROR_2 2

#define LED_PIN_ACTIVE      19       // А5 индикация нормальной работы устройства (моргает значит не зависло)
#define LED_PIN_ERROR_SENSE 16       // А2 индикация ошибки - нет датчика(ошибка датчика)
#define LED_PIN_ERROR_LAN   14       // А0 индикация ошибки - нет сети

#define BLINK_INTERVAL  1000UL      // интервал между включение/выключением светодиода (1 секунда)

int US_status_1 = LOW;              // изначально флаг срабатывания УЗ неактивен
byte ErrorState = ERROR_NOERROR;    //изначально ошибок нет.

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

IPAddress subnet(255, 255, 252, 0);
IPAddress ip(192, 168, 4, KASSA);      // ip устройства для 55 кассы (100я касса для теста)
IPAddress server(192, 168, 5, 85); 
IPAddress Dns(192, 168, 5, 139);
IPAddress gateway(192, 168, 5, 139);

EthernetClient client;

void setup() {
//  wdt_disable();                        //отключаем таймер
  pinMode(TRIG_1, OUTPUT);              //инициируем как выход
  pinMode(ECHO_1, INPUT);               //инициируем как вход
  pinMode(LED_PIN_ACTIVE, OUTPUT);      //индикация ошибок выключена т.е. негорит
  pinMode(LED_PIN_ERROR_SENSE, OUTPUT); //индикация ошибок выключена т.е. негорит
  pinMode(LED_PIN_ERROR_LAN, OUTPUT);   //индикация ошибок выключена т.е. негорит
  Serial.begin(SERIAL_SPEED);
  Ethernet.begin(mac, ip, Dns, gateway, subnet);
  delay(1000);
//  wdt_enable (WDTO_8S);                 //установили таймер на 8 сек.
  Serial.println("Connecting...");      
  while (client) {
    ; // ожидаем подключения клиента
  }
}

void loop() {
//  wdt_reset();                                    //обнулили таймер иначе будет резет
  blinkLed(BLINK_INTERVAL);                       // мигаем зеленым если все в порядке
  static unsigned long previousMillis = 0;        // храним время последнего переключения светодиода
  static int distance_sm_1;
  distance_sm_1 = usRead(TRIG_1, ECHO_1);         // возвращаем дистанцию
//  Serial.println(distance_sm_1);
  if (distance_sm_1 < DIST_MAX_1) {
    if (US_status_1 == LOW) {
      Serial.println("US#1 Motion detected!");
////////////////данные датчика на сервер НАЧАЛО///////////////////////
      if (client.connect(server, PORT)) {
        Serial.println("US#1 connected");
        ErrorState = ERROR_NOERROR;
        client.print(KASSA);        
        client.println("a1");
        client.stop();
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
  delayMicroseconds(10);              // равный 10 микросекундам
  digitalWrite(Trig, LOW);            // Отключаем
  impulseTime = pulseIn(Echo, HIGH);  // Замеряем длину импульса
  distance_sm = impulseTime / 58;     // Пересчитываем в сантиметры
  return distance_sm;
}
//работа индикатора АКТИВНОСТЬ на 
void blinkLed(unsigned long interval ){
  static unsigned long prevTime = 0; // время когда последний раз переключали диод
  if(millis() - prevTime > interval) {
    prevTime = millis();  // 
    digitalWrite(LED_PIN_ACTIVE,!digitalRead(LED_PIN_ACTIVE)); 
  }
}
