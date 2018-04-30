import processing.serial.*;
import net.java.games.input.*;
import org.gamecontrolplus.*;
import org.gamecontrolplus.gui.*;


ControlIO control;
int x,y,z,a,b,c,d,X,Y,NX,NY,But,LSTICK;
ControlDevice device;


import processing.serial.*;

Serial myPort;  // Create object from Serial class
public static final char HEADER    = 'H';
public static final char A_TAG = 'M';
public static final char B_TAG = 'X';

void setup()
{
  size(512, 512);
  ControlButton button;
  ControlHat hat;
  ControlSlider slider;
  control = ControlIO.getInstance(this);
  device = control.getMatchedDevice("ArdCont");

  String portName = Serial.list()[0];
  myPort = new Serial(this, portName, 9600);
}

void draw(){
y=-1*int(map(device.getSlider("LSTICKY").getValue(), 0,1,0,255));
x=int(map(device.getSlider("LSTICKX").getValue(),0,1,0,255));
z=int(map(device.getSlider("TRIGGERS").getValue(),0,1,0,255));
a=int(device.getButton("A").getValue());
b=int(device.getButton("B").getValue());
c=int(device.getButton("X").getValue());
d=int(device.getButton("Y").getValue());
LSTICK=int(device.getButton("LSTICK").getValue());
if(y>0){
    Y=y;
    NY=0;}
if(y<0){
    NY=y;
    Y=0;}
if(x>0){
    X=x;
    NX=0;}
if(x<0){
    NX=x;
    X=0;}
if(LSTICK==8){
  Y=0;
  X=0;
  NY=0;
  NX=0;
}


if(a==8){But=2;}
if(b==8){But=3;}
if(c==8){But=1;}
if(d==8){But=0;}

 if (z<0) 
 {
  
  print(But);
  print(",");
  print(X);
  print(",");
  print(NX);
  print(",");
  print(Y);
  print(",");
  println(NY);
  delay(25);
  sendMessage(A_TAG, But,X,NX,Y,NY);
  }
}

void serialEvent(Serial p) {
  // handle incoming serial data
  String inString = myPort.readStringUntil('\n');
  if(inString != null) {     
    print( inString );   // echo text string from Arduino
  }
}

//void mousePressed() {
//  sendMessage(A_TAG, But,b,c,d);
//}

void sendMessage(char tag, int a, int b, int c, int d, int e){
  // send the given index and value to the serial port
  myPort.write(HEADER);
  myPort.write(tag);
  myPort.write((char)(a / 256)); // msb
  myPort.write(a & 0xff);  //lsb
  myPort.write((char)(b / 256)); // msb
  myPort.write(b & 0xff);  //lsb
  myPort.write((char)(c / 256)); // msb
  myPort.write(c & 0xff);  //lsb
  myPort.write((char)(d / 256)); // msb
  myPort.write(d & 0xff);  //lsb
  myPort.write((char)(e / 256)); // msb
  myPort.write(e & 0xff);  //lsb
}
