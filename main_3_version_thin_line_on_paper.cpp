#include <Arduino.h>
#include <TimerOne.h>

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

//Declare Finite state machines
typedef struct
{
    uint8_t state, new_state;
    unsigned long tes, tis;
} fsm_t;

//Variables definition
fsm_t fsm_dir;
uint16_t sensor[5];

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
    Timer1.initialize(1500);
    fsm_dir.state = 0;
    update_state(fsm_dir);
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
} 

//Program loop
void loop()
{
    sensor[0] = analogRead(IR1);
    sensor[1] = analogRead(IR2);
    sensor[2] = analogRead(IR3);
    sensor[3] = analogRead(IR4);
    sensor[4] = analogRead(IR5);

    //Determine Direction machine state
    if (sensor[2] > 250)
    {
        if (fsm_dir.state == 0 && (sensor[2] > sensor[0] || sensor[2] > sensor[1]))
            fsm_dir.new_state = 1;
        if (fsm_dir.state == 0 && (sensor[2] > sensor[3] || sensor[2] > sensor[4]))
            fsm_dir.new_state = 2;
        if ((fsm_dir.state == 1 || fsm_dir.state == 2) && (sensor[2] < sensor[0] && sensor[2] < sensor[1] && sensor[2] < sensor[3] && sensor[2] < sensor[4]))
            fsm_dir.new_state = 0;
    }

    //Forward or turn
    if (fsm_dir.state == 0)
    {
        digitalWrite(DIR1, 1);
        digitalWrite(DIR2, 1);
        Timer1.pwm(PWM1, 250);
        Timer1.pwm(PWM2, 250);
    }
    else if (fsm_dir.state == 1)
    {
        digitalWrite(DIR1, 0);
        digitalWrite(DIR2, 1);
        Timer1.pwm(PWM1, 150);
        Timer1.pwm(PWM2, 150);
    }
    else if (fsm_dir.state == 2)
    {
        digitalWrite(DIR1, 1);
        digitalWrite(DIR2, 0);
        Timer1.pwm(PWM1, 150);
        Timer1.pwm(PWM2, 150);
    }

    update_state(fsm_dir);
}