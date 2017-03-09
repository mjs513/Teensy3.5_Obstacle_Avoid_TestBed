/*
The posible baudrates are:
1-------1200
2-------2400
3------4800
4------9600
5------19200
6-------38400
7------57600
8-----115200
9------230400
A-----460800
B-----921600
C-----1382400


*** Important. If you re-set the BT baud rate, you must change the myserial baud to match what was just set. Steps:

Assuming the BT module baud default is 9600 Baud set mySerial to 9600 and you want to change it to 57600.
1) leave the line:  //mySerial.begin(9600); as you have to been at the current baud setting to be able to send the command to the BT module.
2) uncomment the line:  //mySerial.print("AT+BAUD7"); // Set baudrate to 57600
3) run the sketch.  This will set the baud to 57600.  However, you may not see the response OK. 
4) uncomment the line //mySerial.begin(57600); and download the sketch again - this sets myserial to the new setting.
5) now that myserial is set to 57600 open the serial monitor - this will restart the sketch.  You should see a secomd line after "Goodnight Moon"
that will show the BT module responding to the change.
6) be sure to re-comment the line //mySerial.print("AT+BAUD7"); when done

*/
#define mySerial Serial3

void setup()
{
//Serial.begin(9600);
//Serial.begin(57600);
Serial.begin(57600);
Serial.println("Goodnight moon!");
mySerial.begin(57600);
//mySerial.begin(57600);
//mySerial.begin(115200);    //if you change the baud and want to re-run this sketch, make sure this baud rate matches the new rate.


mySerial.print("AT");

mySerial.print("AT+VERSION");

//mySerial.print("AT+PIN1234"); // Set pin to 1234  was 1342
//delay(1000);
//mySerial.print("AT+NAMEJY-MCU-HC06"); // Set the name to JY-MCU-HC06
//delay(1000);
//mySerial.print("AT+BAUD4"); // Set baudrate to 9600
//mySerial.print("AT+BAUD7"); // Set baudrate to 57600
//mySerial.print("AT+BAUD7"); // Set baudrate to 115200

}

void loop() // run over and over
{
if (mySerial.available())
Serial.write(mySerial.read());
if (Serial.available())
mySerial.write(Serial.read());
}

