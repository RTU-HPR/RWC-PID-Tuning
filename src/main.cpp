#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <EEPROM.h>
#include <stdlib.h>

#include "webpage.h"
#include "motor.h"
#include "pid.h"
#include "filter.h"


#define BNO_TICK_FREQ 500
#define BNO_TICK_PERIOD 1000000 / BNO_TICK_FREQ - 1
#define MOTOR_TICK_FREQ 200
#define MOTOR_TICK_PERIOD 1000000 / MOTOR_TICK_FREQ - 1
#define TELEM_FREQ 10
#define TELEM_TICK_PERIOD 1000 / TELEM_FREQ - 1
#define PID_SWITCH_TRESHOLD 12.0f

#define BNO_CALIBRATION_CHCK_FREQ 10
#define BNO_CALIBRATION_CHCK_PERIOD 1000 / BNO_CALIBRATION_CHCK_FREQ - 1

#define WHITE_LED 5
#define GREEN_LED 6
#define RED_LED 7

uint64_t calibrationTick;

struct PIDconf;
#define PID_SETTINGS_EEPROM_SIZE 2 * sizeof(PIDconf)

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29, &Wire);

PID ppid(0.08, 0.02, 0.0002, -250.0, 250.0);
PID spid(15, 1, 0.1, -250.0, 250.0);

LowPassFIR filter(23);

IPAddress IP(10, 0, 0, 1);
IPAddress GW(255, 255, 255, 0);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const char *ssid = "platform";
const char *password = "123456789";

float setpoint = 0;
unsigned long motorTick = 0;
unsigned long platformTick = 0;
unsigned long telemTick = 0;
unsigned long prevTick = 0;

float prevMeasurment = 0.0f;
bool afterZero = false;
float target = 0.0f;
bool pointingPIDApplied = true, speedPIDApplied = true;

unsigned char wsClients = 0;

const PROGMEM float filterk[] = {
    0.009082479966205859,
    0.015952216480505963,
    0.023234853965074423,
    0.030695728078538806,
    0.038084022811660879,
    0.045143296703246204,
    0.051622456922709739,
    0.057286661223692671,
    0.061927626244948367,
    0.065372845521188064,
    0.067493270732698937,
    0.068209082699060161,
    0.067493270732698937,
    0.065372845521188064,
    0.061927626244948367,
    0.057286661223692671,
    0.051622456922709739,
    0.045143296703246204,
    0.038084022811660879,
    0.030695728078538806,
    0.023234853965074423,
    0.015952216480505963,
    0.009082479966205859,
};

bool mode = true;
bool on = true;

struct PIDconf
{
  float p;
  float i;
  float d;
};

PIDconf pointing = {.p = 0.08, .i = 0.02, .d = 0.0002}, speed = {.p = 15, .i = 1, .d = 0.1};

char buffer[512];

void initWebserver()
{
  server.on(
      "/",
      HTTP_GET,
      [](AsyncWebServerRequest *request)
      {
        request->send_P(200, "text/html", indexPage);
      });

  server.on(
      "/toggle-mode",
      HTTP_POST,
      [](AsyncWebServerRequest *request) {},
      NULL,
      [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
      {
        if (data[0] == '1')
        {
          mode = false;
        }
        else
        {
          mode = true;
        }
        request->send(200);
      });

  server.on(
      "/point-target",
      HTTP_POST,
      [](AsyncWebServerRequest *request) {},
      NULL,
      [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
      {
        target = (atoff((const char *)data) <= 359.0f ? atoff((const char *)data) : target);
        request->send(200);
      });
  server.on(
      "/on",
      HTTP_POST,
      [](AsyncWebServerRequest *request) {},
      NULL,
      [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
      {
        if (data[0] == '1')
        {
          on = true;
        }
        else
        {
          on = false;
        }
        request->send(200);
      });

  server.on(
      "/settings/pointing/p",
      HTTP_GET,
      [](AsyncWebServerRequest *request)
      {
        char settingsBuffer[16];
        sprintf(settingsBuffer, "%.5f", pointing.p);
        request->send_P(200, "text/html", settingsBuffer);
      });

  server.on(
      "/settings/pointing/i",
      HTTP_GET,
      [](AsyncWebServerRequest *request)
      {
        char settingsBuffer[16];
        sprintf(settingsBuffer, "%.5f", pointing.i);
        request->send_P(200, "text/html", settingsBuffer);
      });

  server.on(
      "/settings/pointing/d",
      HTTP_GET,
      [](AsyncWebServerRequest *request)
      {
        char settingsBuffer[16];
        sprintf(settingsBuffer, "%.5f", pointing.d);
        request->send_P(200, "text/html", settingsBuffer);
      });

  server.on(
      "/settings/speed/p",
      HTTP_GET,
      [](AsyncWebServerRequest *request)
      {
        char settingsBuffer[16];
        sprintf(settingsBuffer, "%.5f", speed.p);
        request->send_P(200, "text/html", settingsBuffer);
      });

  server.on(
      "/settings/speed/i",
      HTTP_GET,
      [](AsyncWebServerRequest *request)
      {
        char settingsBuffer[16];
        sprintf(settingsBuffer, "%.5f", speed.i);
        request->send_P(200, "text/html", settingsBuffer);
      });

  server.on(
      "/settings/speed/d",
      HTTP_GET,
      [](AsyncWebServerRequest *request)
      {
        char settingsBuffer[16];
        sprintf(settingsBuffer, "%.5f", speed.d);
        request->send_P(200, "text/html", settingsBuffer);
      });

  server.on(
      "/settings/pointing/p",
      HTTP_POST,
      [](AsyncWebServerRequest *request) {},
      NULL,
      [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
      {
        float value = atoff((const char *)data);
        pointing.p = value;
        pointingPIDApplied = false;
        request->send(200);
      });

  server.on(
      "/settings/pointing/i",
      HTTP_POST,
      [](AsyncWebServerRequest *request) {},
      NULL,
      [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
      {
        float value = atoff((const char *)data);
        pointing.i = value;
        pointingPIDApplied = false;
        request->send(200);
      });

  server.on(
      "/settings/pointing/d",
      HTTP_POST,
      [](AsyncWebServerRequest *request) {},
      NULL,
      [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
      {
        float value = atoff((const char *)data);
        pointing.d = value;
        pointingPIDApplied = false;
        request->send(200);
      });

  server.on(
      "/settings/speed/p",
      HTTP_POST,
      [](AsyncWebServerRequest *request) {},
      NULL,
      [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
      {
        float value = atoff((const char *)data);
        speed.p = value;
        speedPIDApplied = false;
        request->send(200);
      });

  server.on(
      "/settings/speed/i",
      HTTP_POST,
      [](AsyncWebServerRequest *request) {},
      NULL,
      [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
      {
        float value = atoff((const char *)data);
        speed.i = value;
        speedPIDApplied = false;
        request->send(200);
      });

  server.on(
      "/settings/speed/d",
      HTTP_POST,
      [](AsyncWebServerRequest *request) {},
      NULL,
      [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
      {
        float value = atoff((const char *)data);
        speed.d = value;
        speedPIDApplied = false;
        request->send(200);
      });

  server.on(
      "/save-settings",
      HTTP_GET,
      [](AsyncWebServerRequest *request)
      {
        EEPROM.put(0, pointing);
        EEPROM.put(sizeof(PIDconf), speed);
        EEPROM.commit();
        request->send(200);
      });
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    wsClients += 1;
    break;
  case WS_EVT_DISCONNECT:
    wsClients -= 1;
    break;
  case WS_EVT_DATA:
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}

void initWebSocket()
{
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void convertData(char *output, float pSetpoint, float pValue, float sSetpoint, float sValue)
{
  sprintf(output, "{"
                  "\"speed\":{"
                  "\"setpoint\": %.2f,"
                  "\"value\": %.2f"
                  "},"
                  "\"pointing\":{"
                  "\"setpoint\": %.2f,"
                  "\"value\": %.2f"
                  "}"
                  "}",
          sSetpoint, sValue, pSetpoint, pValue);
}

void setup()
{

  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  Wire.setPins(48, 47);

  EEPROM.begin(PID_SETTINGS_EEPROM_SIZE);
  pointing = EEPROM.get(0, pointing);
  speed = EEPROM.get(sizeof(PIDconf), speed);

  if (!bno.begin())
  {
    while (1)
      ;
  }

  motor0.setInterruptHandler(motor0InterruptHandler);
  motor0.init();
  motor0.enable();

  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(IP, IP, GW);
  IPAddress IP = WiFi.softAPIP();

  initWebserver();
  initWebSocket();
  server.begin();

  filter.setCoefficients((float *)filterk);
}

void loop()
{
  float currentPos, curSpeed;
  if (on)
  {
    if (micros() - motorTick > MOTOR_TICK_PERIOD)
    {
      motorTick = micros();
      motor0.tick(micros());
    }

    if (micros() - platformTick > BNO_TICK_PERIOD)
    {
      platformTick = micros();
      sensors_event_t rps;
      bno.getEvent(&rps, Adafruit_BNO055::VECTOR_GYROSCOPE);

      curSpeed = filter.performFiltering(rps.gyro.z);

      if (rps.gyro.z > PID_SWITCH_TRESHOLD || rps.gyro.z < -PID_SWITCH_TRESHOLD || !mode)
      {
        spid.setpoint = 0.0f;
        ppid.reset();
      }
      else if (mode)
      {
        imu::Quaternion quat = bno.getQuat();
        imu::Vector<3> euler = quat.toEuler();
        currentPos = euler[0] * RAD_TO_DEG;
        currentPos = 360.0f - ((currentPos < 0.0f) ? currentPos + 360.0f : currentPos);
        float error = currentPos - target;
        error = !(error > 180 || error < -180) ? error : (error > 180 ? error - 360 : error + 360); // refactor this
        spid.setpoint = ppid.tick(-error, micros());
      }
      motor0.pid.setpoint = spid.tick(curSpeed, micros());
    }
  }
  else
  {
    motor0.brake();
    ppid.reset();
    spid.reset();
  }
  if (!speedPIDApplied || !pointingPIDApplied)
  {
    ppid.reset(pointing.p, pointing.i, pointing.d);
    spid.reset(speed.p, speed.i, speed.d);
    speedPIDApplied = true;
    pointingPIDApplied = true;
  }

  if (millis() - telemTick > TELEM_TICK_PERIOD)
  {
    telemTick = millis();
    convertData(buffer, target, currentPos, spid.setpoint, curSpeed);
    ws.textAll(buffer);
  }

      if (millis() - calibrationTick > BNO_CALIBRATION_CHCK_PERIOD)
    {
        calibrationTick = millis();
        uint8_t calibration[4];
        bno.getCalibration(&calibration[0], &calibration[1], &calibration[2], &calibration[3]);

        if (calibration[1] == 3)
        {
            digitalWrite(GREEN_LED, 1);
        }
        else
        {
            digitalWrite(GREEN_LED, 0);
        }

        if (calibration[3] == 3)
        {
            digitalWrite(RED_LED, 1);
        }
        else
        {
            digitalWrite(RED_LED, 0);
        }
    }
}