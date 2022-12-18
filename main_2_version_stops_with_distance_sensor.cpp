#include <Arduino.h>
#include <TimerOne.h>
#include <VL53L0X.h>

// Nano pins definition
#define MOT1_A 2
#define MOT1_B 3
#define MOT2_A 11
#define MOT2_B 12
#define PWM1 9   
#define DIR1 4
#define PWM2 10                        
#define DIR2 7
#define LED 13
#define FUNC_BOT 8
#define IR1 A6   
#define IR2 A3
#define IR3 A2                         
#define IR4 A1
#define IR5 A0
#define VOLT A7

//PID time period calculation
#define PERIOD 3
//Distance time period calculation
#define MEASURE 100

//Declare Finite state machines
typedef struct
{
    uint8_t state, new_state;
    unsigned long tes, tis;
} fsm_t;

//Variables definition
fsm_t fsm_FUNC, fsm_BOT;
VL53L0X object;
uint8_t Bot, prev_Bot, start=0, calib=0, motDir1=1, motDir2=1;
uint16_t valVolt, velo, sensor[5], refTracker[10]={}, motorSpeed1, motorSpeed2, speed=350, curve_spd = 150, FlipFlop=500;
uint32_t normSensor[5];
float dist, prev_dist, volt, motorSpeed, position, P, I, D, lastP, kP=0.3, kI=0.1, kD=0.05;
unsigned long mils=0, curmils=0, distmils=0;

//Update state machines state
void update_state(fsm_t &fsm)
{
    if (fsm.state != fsm.new_state)
    {
        fsm.state = fsm.new_state;
        fsm.tes = millis();
        fsm.tis = 0;
    }
}

//Initial setup 
void setup() 
{
    Serial.begin(115200);
    Wire.begin();
    Timer1.initialize(1500);
    fsm_FUNC.state = 0;
    fsm_BOT.state = 0;
    update_state(fsm_BOT);
    update_state(fsm_FUNC);
    pinMode(DIR1, OUTPUT);
    pinMode(DIR2, OUTPUT);
    pinMode(PWM1, OUTPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(LED, OUTPUT);
    pinMode(IR1, INPUT);
    pinMode(IR2, INPUT);
    pinMode(IR3, INPUT);
    pinMode(IR4, INPUT);
    pinMode(IR5, INPUT);
    pinMode(VOLT, INPUT);
    pinMode(FUNC_BOT, INPUT);
    digitalWrite(LED, LOW);
    if (!object.init())
    {
        Serial.println("Failed to detect and initialize sensor!");
        while (1)
        {
        }
    }
    object.startContinuous(100);
} 

//Send speed and direction to motors 
void motor()
{
    digitalWrite(DIR1, motDir1);
    digitalWrite(DIR2, motDir2);
    Timer1.pwm(PWM1, motorSpeed1);
    Timer1.pwm(PWM2, motorSpeed2);
}

//Read sensors values
void tracker()
{
    sensor[0] = analogRead(IR1);
    sensor[1] = analogRead(IR2);
    sensor[2] = analogRead(IR3);
    sensor[3] = analogRead(IR4);
    sensor[4] = analogRead(IR5);
}

//Stop robot
void stop()
{
    fsm_BOT.new_state = 0;
    fsm_FUNC.new_state = 2;
    start = 0;
    calib = 0;
    motorSpeed1 = 0;
    motorSpeed2 = 0;
}

//Sensing obstacle distance
void obstacle()
{
    dist = object.readRangeContinuousMillimeters() - 60;
    if(dist < prev_dist && dist < 200)
    {
        FlipFlop = 3000;
        stop();
    }
}

//Sensing battery voltage
void voltage()
{
    valVolt = analogRead(VOLT);
    volt = valVolt * 0.0048876 * 2;
    if(volt < 6.4)
    {
        FlipFlop = 2000;
        stop();
    }
}

//Prints to aid configuration
void print(int i)
{
    /* Serial.print("Sensor[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.print(sensor[i]);
    Serial.print("   Normalized Sensor[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.print(normSensor[i]);
    Serial.print("   Reference Sensor min[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.print(refTracker[i]);
    Serial.print("  Max: ");
    Serial.println(refTracker[i + 5]); */
    Serial.print("Distance to obstacle: ");
    Serial.println(dist);
    /* Serial.print("Motor Speed: ");
    Serial.print(motorSpeed);
    Serial.print("  Motor Speed 1: ");
    Serial.print(motorSpeed1);
    Serial.print("  Motor Speed 2: ");
    Serial.println(motorSpeed2); */
}

//Calibrate sensors finding its minimun and Maximun values
void calibrate()
{
    digitalWrite(LED, HIGH);
    mils = millis();
    curmils = millis();
    while (millis() < mils + 5000)
    {
        for(int i = 0; i < 5; i++)
        {
            tracker();
            if (sensor[i] < refTracker[i] || refTracker[i] == 0)
                refTracker[i] = sensor[i];
            if (sensor[i] > refTracker[i+5] || refTracker[i+5] == 0)
                refTracker[i+5] = sensor[i];
        }
        if(millis() < curmils + 1000)
            digitalWrite(LED, LOW);
        else if(millis() > curmils + 1000)
            digitalWrite(LED, HIGH);

        if(millis() > curmils + 2000)
            curmils = millis();
    }
    if(refTracker[0] == 0 || refTracker[1] == 0 || refTracker[2] == 0  || refTracker[3] == 0 || refTracker[4] == 0 ||
    refTracker[5] == 0 || refTracker[6] == 0 || refTracker[7] == 0 || refTracker[8] == 0 || refTracker[9] == 0)
    {
        calib = 0;
        digitalWrite(LED, LOW);
    }
    else
    {
        calib = 1;
        digitalWrite(LED, HIGH); 
    }
}

//Finite state machine for botton
void bot_machine()
{
    if (fsm_BOT.state == 0 && (!Bot && prev_Bot))
        fsm_BOT.new_state = 1;
    else if (fsm_BOT.state == 1 && (!Bot && prev_Bot))
        fsm_BOT.new_state = 0;
    else if (fsm_BOT.state == 1 && fsm_BOT.tis > 2000)
        fsm_BOT.new_state = 2;
}

//Finite sate machine for stopped, calibrate or start phases
void func_machine()
{
    if (fsm_FUNC.state == 0 && !calib && fsm_BOT.state == 2)
    {
        fsm_FUNC.new_state = 1; //Calibrate State
        fsm_BOT.new_state = 0;
    }
    else if (fsm_FUNC.state == 0 && calib && fsm_BOT.state == 2)
    {
        fsm_FUNC.new_state = 2; //ON State
        start = 1;
        fsm_BOT.new_state = 0;
    }
    else if (fsm_FUNC.state == 1 && fsm_FUNC.tis > 500) 
    {
        fsm_FUNC.new_state = 0;
        calibrate();
    }
    else if (fsm_FUNC.state == 2 && fsm_FUNC.tis < FlipFlop)
        digitalWrite(LED, LOW);
    else if (fsm_FUNC.state == 2 && fsm_FUNC.tis > FlipFlop)
    {
        digitalWrite(LED, HIGH);
        if (fsm_FUNC.tis > 2*FlipFlop)
        {
            fsm_FUNC.tes = millis();
            fsm_FUNC.tis = 0;
        }
    }
}

//Deterimine rotation direction of motor after PID calculation
void direct()
{
    if ((motorSpeed * (-1)) > speed)
        motDir1 = 0;
    else if ((motorSpeed * (-1)) < speed)
        motDir1 = 1;
    if (motorSpeed > speed)
        motDir2 = 0;
    else if (motorSpeed < speed)
        motDir2 = 1;
}

//Runining normal with PID control for straight line or large curves
void normal_run()
{
    position = (0 * normSensor[0] + 1000L * normSensor[1] + 2000L * normSensor[2] + 3000L * normSensor[3] + 4000L * normSensor[4])
    / (normSensor[0] + normSensor[1] + normSensor[2] + normSensor[3] + normSensor[4]);

    // PID Implementation
    velo = speed;
    P = position - 2000;
    I = (lastP * PERIOD) + (((P - lastP) * PERIOD) / 2);
    D = (P - lastP) / PERIOD;
    lastP = P;

    motorSpeed = (kP * P) + (kI * I) + (kD * D);

    motorSpeed1 = velo - motorSpeed;
    motorSpeed2 = velo + motorSpeed;

    direct();
}

//Running hard left curve (90º+) procedure
void left_run()
{
    motorSpeed = 100;

    motDir1 = 0;
    motorSpeed1 = velo;
    motorSpeed2 = velo + motorSpeed;

}

//Running hard right curve (90º+) procedure
void right_run()
{
    motorSpeed = 100;

    motorSpeed1 = velo + motorSpeed;
    motDir2 = 0;
    motorSpeed2 = velo;

}

//Deterimne which moving algorithm to use
void move()
{
    if (sensor[2] < (refTracker[2] + 150) && sensor[0] > (refTracker[5] - 150) && sensor[4] > (refTracker[9] - 150))
        normal_run();
    else if (sensor[2] < (refTracker[2] + 150) && (sensor[0] < (refTracker[0] + 150) && sensor[1] < (refTracker[1] + 150)))
        left_run();
    else if (sensor[2] < (refTracker[2] + 150) && (sensor[3] < (refTracker[3] + 150) && sensor[4] < (refTracker[4] + 150)))
        right_run();
}

void loop() 
{
    voltage();
    prev_dist = dist;
    prev_Bot = Bot;
    Bot = digitalRead(FUNC_BOT);
    fsm_BOT.tis = millis() - fsm_BOT.tes;
    fsm_FUNC.tis = millis() - fsm_FUNC.tes;
    
    bot_machine();

    func_machine();

    //If in runing mode and on track with no obstacle and good voltage
    if (start == 1)
    {
        if ((millis() - mils) >= PERIOD)
        {
            mils = millis();
            tracker();
            for (int i = 0; i < 5; i++)
            {
                normSensor[i] = (((sensor[i] - refTracker[i]) * 1000L) / (refTracker[i + 5] - refTracker[i]));
            }
            move();
        }
        if ((millis() - distmils) >= MEASURE)
        {
            distmils = millis();
            obstacle();
        }              
    }

    motor();
    update_state(fsm_BOT);
    update_state(fsm_FUNC);
    /* for (int i = 0; i <5; i++)
        print(i); */
}
