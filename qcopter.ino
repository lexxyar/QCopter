//#include <SD.h>

#include <avr/interrupt.h>
#include <USBAPI.h>
#include "MPU6050.h"            // Подключаем библиотеку гироскопа
#include <Wire.h>
#include <SFE_BMP180.h>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
#define LED_PIN 13              // Номер пина для светодиода
#define ULTRASONIC_TRIG_PIN 3   // Номер пина ультразвука (триггер)
#define ULTRASONIC_ECHO_PIN 4   // Номер пина ультразвука (эхо)
#define GYRO_CALIBRATION_COUNT 200   // Количество итераций калибровки
#define METEO_CALIBRATION_COUNT 20   // Количество итераций калибровки
#define RAD 57.14286            // Значение для перевода в градусы
//#define DEBUG true              // Режим отладки
#define LED_ON false    // Использование светодиода
#define METEO_DATA_MEASURE_WAIT 1 // задержка измерения барометра from 0 to 3 (highest res, longest wait).

bool blinkState = false;        // Состояние моргания светодиода

/**
 * Структура данных радиоприемника
 */
struct RadioChannel {
    byte lastCahnnel;       // Последний канал
    unsigned long timer;    // Таймер
    int input;              // Значение
} radio[5];                 // Массив из 5 каналов (0 не используется)

/**
 * Структура данных гироскопа
 */
struct GyroData {
    MPU6050 device;              // Переменная гироскопа
    int roll = 0;   // X                // Значение по X
    int pitch = 0;  // Y                // Значение по Y
    int yaw = 0;    // Z                // Значение по Z
    int baseRoll = 0;   // X                // Базовое значение по X
    int basePitch = 0;  // Y                // Базовое значение по Y
    int baseYaw = 0;    // Z                // Базовое значение по Z
    bool isConnected = false;   // Состояние подключения датчика
} accelgyro;                    // Переменная гироскопа

/**
 * Структура данных барометра
 */
struct MeteoData {
    SFE_BMP180 device;          // Переменная барометра
    double temperature = 0;     // Показатель температуры
    double pressure = 0;        // Показатель давления
    double altitude = 0;        // Показатель высоты
    bool isConnected = false;   // Состояние подключения датчика
    double baseline = 0;        // Базовый показатель давления
    bool isError = false;       // Наличие ошибки
    String errorMessage = "";   // Текст ошибки
} meteo;                        // Переменная барометра

/**
 * Структура данных ультразвука
 */
struct UltrasonicData {
    long duration;      // Длительность
    long distanceCm;    // Дистанция в см
} ultrasonic;           // Переменная ультразвука

/**
 * Проверяет, все ли устройства подключены
 *
 * @return true|false
 */
bool IsDeviceOk();

/**
 * Возвра
 * @return
 */
//double getPressure();

//void PrintAccelGyro();

/**
 * Считывает данные барометра
 */
void GetMeteoData();

/**
 * Моргает светодиодом n раз
 * @param times Количество вспышек
 */
void BlinkTimes(int times);

/**
 * Переключает состояние светодиода
 * @param state true|false
 */
void SwitchLed(bool state);

/**
 * Считывает значение расстояния с ультразвука
 */
void GetDistance();

//void PrintSignals();

unsigned long current_time;     // Переменная хранения времени

/**
 * Функция установки
 */
void setup() {
    pinMode(LED_PIN, OUTPUT);       // Устанавливаем пин светодиода на вывод
    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);   // Устанавливаем пин триггера ультразвука на вывод

    BlinkTimes(3);      // Моргнем 3 раза

//    if (DEBUG) {
//        Serial.begin(57600);
//
//        // initialize device
//        Serial.println("Initializing I2C devices...");
//
//        // verify connection
//        Serial.println("Testing device connections...");
//    }

    accelgyro.device.initialize();      // Инициализируем гироскоп
    accelgyro.isConnected = accelgyro.device.testConnection();  // Проверяем соединение

    if (accelgyro.isConnected) {    // Если соединение установлено
//        if (DEBUG) {
//            Serial.println("MPU6050 connection successful");
//            Serial.print("MPU6050 calibration...");
//        }
        int counter = 0;
        double rollCall;                // Значение калибровки X
        double pitchCall;               // Значение калибровки Y
        double yawCall;                 // Значение калибровки Z
        for (counter = 0; counter < GYRO_CALIBRATION_COUNT; counter++) {
            accelgyro.device.getRotation(&accelgyro.roll, &accelgyro.pitch, &accelgyro.yaw);
            rollCall += accelgyro.roll;
            pitchCall += accelgyro.pitch;
            yawCall += accelgyro.yaw;
            if (counter % 100 == 0) {

//                if (DEBUG) {
//                    Serial.print(".");
//                }
            }
            delay(4);
        }

        accelgyro.baseRoll = (int) (rollCall / GYRO_CALIBRATION_COUNT);
        accelgyro.basePitch = (int) (pitchCall / GYRO_CALIBRATION_COUNT);
        accelgyro.baseYaw = (int) (yawCall / GYRO_CALIBRATION_COUNT);

//        if (DEBUG) {
//            Serial.println("");
//            Serial.print("Base Roll: ");
//            Serial.print(accelgyro.baseRoll);
//            Serial.print("\tBase Pitch: ");
//            Serial.print(accelgyro.basePitch);
//            Serial.print("\tBase Yaw: ");
//            Serial.println(accelgyro.baseYaw);
//            Serial.println("");
//        }
    }
//    else {
//        if (DEBUG) {
//            Serial.println("MPU6050 connection failed");
//        }
//    }

    BlinkTimes(3);


    meteo.isConnected = meteo.device.begin();
    if (meteo.isConnected) {
//        if (DEBUG) {
//            Serial.println("BMP180 connection successful");
//            Serial.print("BMP180 calibration...");
//        }
        int counter = 0;
        double baselineCounter = 0;
        for (counter = 0; counter < METEO_CALIBRATION_COUNT; counter++) {
            GetMeteoData();
            baselineCounter += meteo.pressure;
//            if (DEBUG) {
//                if (counter % 10 == 0) {
//                    Serial.print(".");
//                }
//            }
            delay(4);
        }
        meteo.baseline = baselineCounter / METEO_CALIBRATION_COUNT;

//        if (DEBUG) {
//            Serial.println("");
//            Serial.print("Baseline pressure: ");
//            Serial.print(meteo.baseline);
//            Serial.println(" mb");
//        }
    }
//    else {
//        if (DEBUG) {
//            Serial.println("BMP180 init fail (disconnected?)\n\n");
//        }
//    }
    BlinkTimes(3);

    // https://www.youtube.com/watch?v=bENjl1KQbvo&list=PL0K4VDicBzsibZqfa42DVxC8CGCMB7G2G&index=3
    // Установим возможность прерывания на пины 8 - 11
    PCICR |= (1 << PCIE0);  //Set PCIE0 to enable PCMSK0 scan.
    PCMSK0 |= (1 << PCINT0); //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT1); //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT2); //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT3); //Set PCINT3 (digital input 11)to trigger an interrupt on state change.



    blinkState = false;
    SwitchLed(blinkState);

    if (!IsDeviceOk()) {
//        if (DEBUG) {
//            Serial.println("Device connection error. Run TERMINATED!");
//        }
        while (1) {
            blinkState = !blinkState;
            SwitchLed(blinkState);
            delay(600);
        }
    }

}

void loop() {
    if (meteo.isConnected) {
        GetMeteoData();
    }

    GetDistance();

    if (accelgyro.isConnected) {
        accelgyro.device.getRotation(&accelgyro.roll, &accelgyro.pitch, &accelgyro.yaw);

        accelgyro.roll -= accelgyro.baseRoll;
        accelgyro.pitch -= accelgyro.basePitch;
        accelgyro.yaw -= accelgyro.baseYaw;

        accelgyro.roll /= RAD;
        accelgyro.pitch /= RAD;
        accelgyro.yaw /= RAD;
    }

    // blink LED to indicate activity
    blinkState = !blinkState;
    SwitchLed(blinkState);

//    if (DEBUG) {
//        PrintSignals();
//    }


    delay(2500);
} // loop

/**
 * Обработка прерываний
 */
ISR(PCINT0_vect) {
    current_time = micros();
    //Channel 1=========================================
    if (PINB & B00000001) {                                        //Is input 8 high?
        if (radio[1].lastCahnnel == 0) {                                   //Input 8 changed from 0 to 1
            radio[1].lastCahnnel = 1;                                      //Remember current input state
            radio[1].timer = current_time;                                  //Set timer_1 to current_time
        }
//            Serial.println("1");
    } else if (radio[1].lastCahnnel ==
               1) {                                //Input 8 is not high and changed from 1 to 0
        radio[1].lastCahnnel = 0;                                        //Remember current input state
        radio[1].input = current_time - radio[1].timer;                //Channel 1 is current_time - timer_1
    }
    //Channel 2=========================================
    if (PINB & B00000010) {                                       //Is input 9 high?
        if (radio[2].lastCahnnel == 0) {                                   //Input 9 changed from 0 to 1
            radio[2].lastCahnnel = 1;                                      //Remember current input state
            radio[2].timer = current_time;                                  //Set timer_2 to current_time
        }
//            Serial.println("2");
    } else if (radio[2].lastCahnnel ==
               1) {                                //Input 9 is not high and changed from 1 to 0
        radio[2].lastCahnnel = 0;                                        //Remember current input state
        radio[2].input = current_time - radio[2].timer;                //Channel 2 is current_time - timer_2
    }
    //Channel 3=========================================
    if (PINB & B00000100) {                                       //Is input 10 high?
        if (radio[3].lastCahnnel == 0) {                                   //Input 10 changed from 0 to 1
            radio[3].lastCahnnel = 1;                                      //Remember current input state
            radio[3].timer = current_time;                                  //Set timer_3 to current_time
        }
//            Serial.println("3");
    } else if (radio[3].lastCahnnel ==
               1) {                                //Input 10 is not high and changed from 1 to 0
        radio[3].lastCahnnel = 0;                                        //Remember current input state
        radio[3].input = current_time - radio[3].timer;                //Channel 3 is current_time - timer_3

    }
    //Channel 4=========================================
    if (PINB & B00001000) {                                       //Is input 11 high?
        if (radio[4].lastCahnnel == 0) {                                   //Input 11 changed from 0 to 1
            radio[4].lastCahnnel = 1;                                      //Remember current input state
            radio[4].timer = current_time;                                  //Set timer_4 to current_time
        }
//            Serial.println("4");
    } else if (radio[4].lastCahnnel ==
               1) {                                //Input 11 is not high and changed from 1 to 0
        radio[4].lastCahnnel = 0;                                        //Remember current input state
        radio[4].input = current_time - radio[4].timer;                //Channel 4 is current_time - timer_4
    }
}

//void PrintSignals() {
//
//    Serial.print("Roll:");
//    if (radio[1].input - 1480 < 0) {
//        Serial.print("<<<");
//    } else if (radio[1].input - 1520 > 0) {
//        Serial.print(">>>");
//    } else {
//        Serial.print("-+-");
//    }
//    Serial.print(radio[1].input);
//
//    Serial.print("\tNick:");
//    if (radio[2].input - 1480 < 0) {
//        Serial.print("^^^");
//    } else if (radio[2].input - 1520 > 0) {
//        Serial.print("vvv");
//    } else {
//        Serial.print("-+-");
//    }
//    Serial.print(radio[2].input);
//
//    Serial.print("\tGas:");
//    if (radio[3].input - 1480 < 0) {
//        Serial.print("vvv");
//    } else if (radio[3].input - 1520 > 0) {
//        Serial.print("^^^");
//    } else {
//        Serial.print("-+-");
//    }
//    Serial.print(radio[3].input);
//
//    Serial.print("\tYaw:");
//    if (radio[4].input - 1480 < 0) {
//        Serial.print("<<<");
//    } else if (radio[4].input - 1520 > 0) {
//        Serial.print(">>>");
//    } else {
//        Serial.print("-+-");
//    }
//    Serial.println(radio[4].input);
//
//
//    if (accelgyro.isConnected) {
//        Serial.print("Roll:");
//        if (accelgyro.roll - 1480 < 0) {
//            Serial.print("<<<");
//        } else if (accelgyro.roll - 1520 > 0) {
//            Serial.print(">>>");
//        } else {
//            Serial.print("-+-");
//        }
//        Serial.print(accelgyro.roll);
//
//        Serial.print("\tPitch:");
//        if (accelgyro.pitch - 1480 < 0) {
//            Serial.print("vvv");
//        } else if (accelgyro.pitch - 1520 > 0) {
//            Serial.print("^^^");
//        } else {
//            Serial.print("-+-");
//        }
//        Serial.print(accelgyro.pitch);
//
//        Serial.print("\tYaw:");
//        if (accelgyro.yaw - 1480 < 0) {
//            Serial.print("<<<");
//        } else if (accelgyro.yaw - 1520 > 0) {
//            Serial.print(">>>");
//        } else {
//            Serial.print("-+-");
//        }
//        Serial.println(accelgyro.yaw);
//    }
//
//    if (meteo.isConnected) {
//        Serial.print("Base line: ");
//        Serial.print(meteo.baseline);
//        Serial.print("\tAltitude: ");
//        Serial.print(meteo.altitude);
//        Serial.print("\tPressure:");
//        Serial.print(meteo.pressure);
//        Serial.print("\tTemperature:");
//        Serial.println(meteo.temperature);
//    }
//
//
//    Serial.print("Ultrasonic distance: ");
//    if (ultrasonic.distanceCm <= 0) {
//        Serial.print("Out of range\t");
//        Serial.print(ultrasonic.distanceCm);
//        Serial.println("cm");
//    } else {
//        Serial.print(ultrasonic.distanceCm);
//        Serial.println("cm");
//    }
//
//}

//void PrintAccelGyro() {
//    Serial.print("Roll:");
//    if (accelgyro.roll > 0) {
//        Serial.print("-__");
//    } else if (accelgyro.roll < 0) {
//        Serial.print("__-");
//    } else if (accelgyro.roll == 0) {
//        Serial.print("-+-");
//    }
//    Serial.print(accelgyro.roll);
//    Serial.print("\t");
//
//    Serial.print("Pitch:");
//    if (accelgyro.pitch > 0) {
//        Serial.print("vvv");
//    } else if (accelgyro.pitch < 0) {
//        Serial.print("^^^");
//    } else if (accelgyro.pitch == 0) {
//        Serial.print("---");
//    }
//    Serial.print(accelgyro.pitch);
//    Serial.print("\t");
//    Serial.print("Yaw:");
//    if (accelgyro.yaw > 0) {
//        Serial.print("<<<");
//    } else if (accelgyro.yaw < 0) {
//        Serial.print(">>>");
//    } else if (accelgyro.yaw == 0) {
//        Serial.print("-+-");
//    }
//    Serial.println(accelgyro.yaw);
//}

void GetMeteoData() {
    char status;
    double T, P;
    meteo.isError = false;

    // You must first get a temperature measurement to perform a pressure reading.

    // Start a temperature measurement:
    // If request is successful, the number of ms to wait is returned.
    // If request is unsuccessful, 0 is returned.

    status = meteo.device.startTemperature();
    if (status != 0) {
        // Wait for the measurement to complete:

        delay(status);

        // Retrieve the completed temperature measurement:
        // Note that the measurement is stored in the variable T.
        // Use '&T' to provide the address of T to the function.
        // Function returns 1 if successful, 0 if failure.

        status = meteo.device.getTemperature(T);
        if (status != 0) {
            meteo.temperature = T;
            // Start a pressure measurement:
            // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
            // If request is successful, the number of ms to wait is returned.
            // If request is unsuccessful, 0 is returned.

            status = meteo.device.startPressure(METEO_DATA_MEASURE_WAIT);
            if (status != 0) {
                // Wait for the measurement to complete:
                delay(status);

                // Retrieve the completed pressure measurement:
                // Note that the measurement is stored in the variable P.
                // Use '&P' to provide the address of P.
                // Note also that the function requires the previous temperature measurement (T).
                // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
                // Function returns 1 if successful, 0 if failure.

                status = meteo.device.getPressure(P, T);
                if (status != 0) {
                    meteo.pressure = P;
                    meteo.altitude = meteo.device.altitude(P, meteo.baseline);
                } else {
                    meteo.isError = true;
                    meteo.errorMessage = "error starting pressure measurement\n";
                }
            } else {
                meteo.isError = true;
                meteo.errorMessage = "error starting pressure measurement\n";
            }
        } else {
            meteo.isError = true;
            meteo.errorMessage = "error retrieving temperature measurement\n";
        }
    } else {
        meteo.isError = true;
        meteo.errorMessage = "error retrieving temperature measurement\n";
    }
}

/*
double getPressure() {
    char status;
    double T, P;
    meteo.isError = false;

    // You must first get a temperature measurement to perform a pressure reading.

    // Start a temperature measurement:
    // If request is successful, the number of ms to wait is returned.
    // If request is unsuccessful, 0 is returned.

    status = meteo.device.startTemperature();
    if (status != 0) {
        // Wait for the measurement to complete:

        delay(status);

        // Retrieve the completed temperature measurement:
        // Note that the measurement is stored in the variable T.
        // Use '&T' to provide the address of T to the function.
        // Function returns 1 if successful, 0 if failure.

        status = meteo.device.getTemperature(T);
        if (status != 0) {
            // Start a pressure measurement:
            // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
            // If request is successful, the number of ms to wait is returned.
            // If request is unsuccessful, 0 is returned.

            status = meteo.device.startPressure(METEO_DATA_MEASURE_WAIT);
            if (status != 0) {
                // Wait for the measurement to complete:
                delay(status);

                // Retrieve the completed pressure measurement:
                // Note that the measurement is stored in the variable P.
                // Use '&P' to provide the address of P.
                // Note also that the function requires the previous temperature measurement (T).
                // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
                // Function returns 1 if successful, 0 if failure.

                status = meteo.device.getPressure(P, T);
                if (status != 0) {
                    return (P);
                } else {
                    meteo.isError = true;
                    meteo.errorMessage = "error retrieving pressure measurement\n";
                }
            } else {
                meteo.isError = true;
                meteo.errorMessage = "error starting pressure measurement\n";
            }
        } else {
            meteo.isError = true;
            meteo.errorMessage = "error retrieving temperature measurement\n";
        }
    } else {
        meteo.isError = true;
        meteo.errorMessage = "error retrieving temperature measurement\n";
    }
    return 0;
}
*/
/*
double getTemperature() {
    char status;
    double t;
    meteo.isError = false;

    // If you want to measure altitude, and not pressure, you will instead need
    // to provide a known baseline pressure. This is shown at the end of the sketch.

    // You must first get a temperature measurement to perform a pressure reading.

    // Start a temperature measurement:
    // If request is successful, the number of ms to wait is returned.
    // If request is unsuccessful, 0 is returned.

    status = meteo.device.startTemperature();
    if (status != 0) {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed temperature measurement:
        // Note that the measurement is stored in the variable T.
        // Function returns 1 if successful, 0 if failure.

        status = meteo.device.getTemperature(t);
        if (status != 0) {
            return t;
        } else {
            meteo.isError = true;
            meteo.errorMessage = "error retrieving temperature measurement\n";
        }
    } else {
        meteo.isError = true;
        meteo.errorMessage = "error retrieving temperature measurement\n";
    }
    return 0;
}
 */

bool IsDeviceOk() {
//    if (DEBUG) {
//        Serial.print("Gyro connection:  ");
//        Serial.println(accelgyro.isConnected ? "true" : "false");
//        Serial.print("Meteo connection: ");
//        Serial.println(accelgyro.isConnected ? "true" : "false");
//    }
    return accelgyro.isConnected && meteo.isConnected;
}

void BlinkTimes(int times) {
    int blinkDelay = 100;
    blinkState = false;
    SwitchLed(blinkState);
    delay(blinkDelay);

    for (int i = 0; i < times * 2; i++) {
        blinkState = !blinkState;
        SwitchLed(blinkState);
        delay(blinkDelay);
    }

    blinkState = false;
    SwitchLed(blinkState);
    delay(blinkDelay);
}

void SwitchLed(bool state) {
    if (LED_ON) {
        digitalWrite(LED_PIN, blinkState);
    }
}

void GetDistance() {

    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    ultrasonic.duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
    //Serial.println(duration);

    // convert the time into a distance
    ultrasonic.distanceCm = ultrasonic.duration / 29.1 / 2;
//    distanceIn = duration / 74 / 2;
}

#pragma clang diagnostic pop