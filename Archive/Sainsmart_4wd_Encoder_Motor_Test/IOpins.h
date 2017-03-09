
//Hardware set the pins
// L298D Motor Controller
#define ENA 3  //enable A on pin 5 (must be a pwm pin)   Speed Control
#define ENB 2  //enable B on pin 3 (must be a pwm pin)   Speed Control

#define IN1 5  //IN1 on pin controls Motor A  Motor A is the left side Motor
#define IN2 4  //IN2 on pin controls Motor A
#define IN3 6 //IN3 on pin conrtols Motor B  Motor B is the Right side Motor
#define IN4 7  //IN4 on pin controls Motor B 

const int rightmotorpin1 = IN1;  //signal output 1 of Dc motor 
const int rightmotorpin2 = IN2;  //signal output 2 of Dc motor 
const int leftmotorpin1  = IN3;  //signal output 3 of Dc motor 
const int leftmotorpin2  = IN4;  //signal output 4 of Dc motor 

//set interrupts and encoder pins
//const int l_int = 5;
//const int r_int = 4;
const int l_encoder = 24;
const int r_encoder = 25;



// pins for the HC-SR04 Ultrasonic Distance Sensor
//const int EchoPin = 48; // HC-SR04 Ultrasonic signal input
//const int TrigPin = 49; // HC-SR04 Ultrasonic signal output
//const int HeadServopin = 8; // pin for signal input of headservo

