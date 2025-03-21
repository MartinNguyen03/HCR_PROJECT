//Cameron Wade Medicine Dispenser setup
//20/03/25
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>

// WiFi credentials
const char* ssid = "CameronArduino";
const char* password = "CCWCM34PD";

// Servo and LED Pins
#define SERVO_UPPER_PIN 9
#define SERVO_LOWER_PIN 17
#define LED_RED 27 // 34
#define LED_GREEN 14 // 35

// Servo objects
Servo servoUpper;
Servo servoLower;

// Define open and closed positions
const int SERVO_UPPER_OPEN = 0;
const int SERVO_UPPER_CLOSED = 45;
const int SERVO_LOWER_OPEN = 15;
const int SERVO_LOWER_CLOSED = 60;

// Web server on port 80 (IP 192.168.137.240)
WebServer server(80);

void setup() {
    Serial.begin(115200);
    
    // Attach servos
    servoUpper.attach(SERVO_UPPER_PIN);
    servoLower.attach(SERVO_LOWER_PIN);
    
    // Set LED pins as output
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);

    //
    setState("OPEN", SERVO_UPPER_OPEN, SERVO_LOWER_OPEN, LOW, HIGH); // Default state OPEN
    
    // Connect to WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected!");
    Serial.println(WiFi.localIP());
    
    // Define web routes
    server.on("/", HTTP_GET, handleRoot);
    server.on("/set", HTTP_GET, handleSet);
    server.on("/state", HTTP_GET, handleState);
    
    server.begin();
    Serial.println("Begin Web server");
}

void loop() {
    server.handleClient();
}

void handleRoot() {
    server.send(200, "text/html", R"rawliteral(
        <html>
        <head>
            <title>Servo Control</title>
        </head>
        <body>
            <h1>ESP32 Servo Control</h1>
            <p>Enter angle values:</p>
            <form action="/set">
                Upper Servo: <input type="number" name="upper" min="0" max="180"> <br>
                Lower Servo: <input type="number" name="lower" min="0" max="180"> <br>
                <input type="submit" value="Set Angles">
            </form>
            <h2>Preset States</h2>
            <a href="/state?s=0"><button>OPEN</button></a>
            <a href="/state?s=1"><button>ReadyLoad</button></a>
            <a href="/state?s=2"><button>CLOSED</button></a>
            <a href="/state?s=3"><button>ReadyUnload</button></a>
        </body>
        </html>
    )rawliteral");
}

void handleSet() {
    if (server.hasArg("upper")) {
        int upperAngle = server.arg("upper").toInt();
        upperAngle = constrain(upperAngle, 0, 180);
        servoUpper.write(upperAngle);
        Serial.print("Upper Servo Angle: "); Serial.println(upperAngle);
    }
    
    if (server.hasArg("lower")) {
        int lowerAngle = server.arg("lower").toInt();
        lowerAngle = constrain(lowerAngle, 0, 180);
        servoLower.write(lowerAngle);
        Serial.print("Lower Servo Angle: "); Serial.println(lowerAngle);
    }
    
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Redirecting...");
}

void handleState() {
    if (server.hasArg("s")) {
        int state = server.arg("s").toInt();
        switch (state) {
            case 0: setState("OPEN", SERVO_UPPER_OPEN, SERVO_LOWER_OPEN, LOW, HIGH); break;
            case 1: setState("ReadyLoad", SERVO_UPPER_OPEN, SERVO_LOWER_CLOSED, HIGH, LOW); break;
            case 2: setState("CLOSED", SERVO_UPPER_CLOSED, SERVO_LOWER_CLOSED, LOW, HIGH); break;
            case 3: setState("ReadyUnload", SERVO_UPPER_CLOSED, SERVO_LOWER_CLOSED, HIGH, LOW); break;
        }
    }
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Redirecting...");
}

void setState(const char* state, int posUpper, int posLower, int ledRed, int ledGreen) {
    Serial.print("State: "); Serial.println(state);
    servoUpper.write(posUpper);
    servoLower.write(posLower);
    digitalWrite(LED_RED, ledRed);
    digitalWrite(LED_GREEN, ledGreen);
}
