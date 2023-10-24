import controlP5.*;
import processing.serial.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

//CHANGE THIS! Find the name of your serial port in the Arduino IDE
String serialPortName = "/dev/cu.usbserial-A7006QAG";

ControlP5 cp5;

IntList positions;
IntList targets;
int nPositions = 180;
int curPosition = 0;
int curTarget = 0;
float curP = 12.0;

// variables for I and D
float curI = 5.0;
float curD = 0.0;

int looper = 0;
String inputString;



//Number boxes
Numberbox nb[];
int P = 1;
int I = 2;
int D = 3;
int TARGET = 0;
int NUMBER_LEFT = 60;
int NUMBER_WIDTH = 100;
int NUMBER_HEIGHT = 20;
int NUMBER_TOP = 30;

int LABEL_LEFT = 10;
int ARDUINO_VALUE_LEFT = NUMBER_LEFT + NUMBER_WIDTH + 10;
int SEND_BUTTON_ID = 1;
Button btnSend;

//oscilloscope
int OSC_LEFT = 25;
int OSC_BOTTOM = 250;
int OSC_WIDTH = 500;
int OSC_HEIGHT = 100;
int MAX_VALUE = 2000;
int MIN_VALUE = 0;


int SERIAL_WRITE_LENGTH = 32;

Serial myPort;


int currentTimerVal = 0; // in ms
int timeInterval = 10; // in ms
float currTime = 0.0f;


void setup()
{
  size(600, 400);
  frameRate(30);
  cp5 = new ControlP5(this);
  myPort = new Serial(this, serialPortName, 115200);
  positions = new IntList();
  targets = new IntList();
  
  nb =  new Numberbox[4];
  nb[TARGET] = cp5.addNumberbox("Target")
      .setPosition(NUMBER_LEFT, NUMBER_TOP * (0.5))
      .setSize(NUMBER_WIDTH, NUMBER_HEIGHT)
      .setMultiplier(0.1)
      .setValue((int)0);
  
  nb[P] = cp5.addNumberbox("Position");
  nb[P].setPosition(NUMBER_LEFT, NUMBER_TOP * (1.5))
      .setSize(NUMBER_WIDTH, NUMBER_HEIGHT)
      .setMultiplier(0.1)
      .setValue(curP);

  nb[I] = cp5.addNumberbox("Integral");
  nb[I].setPosition(NUMBER_LEFT, NUMBER_TOP * (2.5))
      .setSize(NUMBER_WIDTH, NUMBER_HEIGHT)
      .setMultiplier(0.01)
      .setValue(curI);

  nb[D] = cp5.addNumberbox("Derivative");
  nb[D].setPosition(NUMBER_LEFT, NUMBER_TOP * (3.5))
      .setSize(NUMBER_WIDTH, NUMBER_HEIGHT)
      .setMultiplier(0.01)
      .setValue(curD);
      
  btnSend = cp5.addButton("UpdateArduino")
    .setPosition(NUMBER_LEFT + NUMBER_WIDTH*1.75, NUMBER_HEIGHT)
    .setId(SEND_BUTTON_ID);


    // send the initial values to the arduino
    UpdateArduino();
}

public void controlEvent(ControlEvent theEvent) {
  if (SEND_BUTTON_ID == theEvent.getController().getId())
  {
    UpdateArduino();
  }
}

void draw() {
  background(255);
  fill(0, 0, 0);
  stroke(0,0,255);
  
  DrawLabels();
  UpdatePositions();
  DrawOscilloscope();

  // uncomment the following for constant updating
  // if(millis() - currentTimerVal > timeInterval)
  // {
  //   currentTimerVal = millis();
  //   currTime += timeInterval/1000.0;
  //   UpdateArduino();
  // }
}

/*
  Detailed functions
*/


void DrawLabels()
{
  fill(0);
  text("Target:", LABEL_LEFT, NUMBER_TOP * 1);
  text("P:", LABEL_LEFT, NUMBER_TOP * 2);

  // add the labels for I and D
  text("I:", LABEL_LEFT, NUMBER_TOP * 3);
  text("D:", LABEL_LEFT, NUMBER_TOP * 4);
  
  text("Arduino values", ARDUINO_VALUE_LEFT, 10);
  text(""+curTarget, ARDUINO_VALUE_LEFT, NUMBER_TOP*1);
  text(""+curP, ARDUINO_VALUE_LEFT, NUMBER_TOP*2);
}

void WriteFloat(float f)
{
  String s = str(f);
  while(s.length() < SERIAL_WRITE_LENGTH)
  {
    s = s + "\0";
  }

   myPort.write(s);
}

void WriteInt(int i)
{
  String s = str(i);
  while(s.length() < SERIAL_WRITE_LENGTH)
  {
    s = s + "\0";
  }
   myPort.write(s);
}

public void UpdateArduino()
{
   myPort.write('p');
   curP = nb[P].getValue();
   WriteFloat(curP);

   myPort.write('i');
   curI = nb[I].getValue();
   WriteFloat(curI);

   myPort.write('d');
   curD = nb[D].getValue();
   WriteFloat(curD);

   myPort.write('t');
   curTarget = (int)nb[TARGET].getValue();
   WriteInt(curTarget);
}

void UpdatePositions()
{
  boolean added = false;
  inputString = "";
  
  byte input[] = new byte[256];
  while(myPort.available() > 0)
  {
    input = myPort.readBytes();
   }
   if (input != null)
   {
     inputString = new String(input);
     String[] inputStrings = inputString.split("\r\n");
     if (inputStrings.length > 2)
     {
       curPosition = Integer.parseInt(inputStrings[inputStrings.length-2]);
     }
   }     
  
  positions.append(curPosition);
  targets.append(curTarget);
  
  while (positions.size() > nPositions)
  {
    positions.remove(0);
    targets.remove(0);
  }
}


void DrawOscilloscope()
{
  noFill();
  stroke(0,0,0);
  line(OSC_LEFT, OSC_BOTTOM, OSC_LEFT+OSC_WIDTH, OSC_BOTTOM);
  line(OSC_LEFT, OSC_BOTTOM+OSC_HEIGHT, OSC_LEFT, OSC_BOTTOM-OSC_HEIGHT);
    
  //draw TARGET
  noFill();
  stroke(255,0,0);
  beginShape();
  for (int i = 0; i < targets.size(); i++)
  {
    int offset_x = (int)(i * OSC_WIDTH/(float)nPositions);
    int offset_y = (int)(targets.get(i) / (float)MAX_VALUE * OSC_HEIGHT);
    vertex(OSC_LEFT+offset_x, OSC_BOTTOM - offset_y); 
  }
  endShape();
  fill(255,0,0);
  text(curTarget, (int)(OSC_LEFT+positions.size()/ (float)nPositions*OSC_WIDTH+50), (int)(OSC_BOTTOM - curTarget/ (float)MAX_VALUE*OSC_HEIGHT));
  
  //DRAW POSITION
  noFill();
  stroke(0,0,255);
  beginShape();
  for (int i = 0; i < positions.size(); i++)
  {
    int offset_x = (int)(i * OSC_WIDTH/(float)nPositions);
    int offset_y = (int)(positions.get(i) / (float)MAX_VALUE * OSC_HEIGHT);
    vertex(OSC_LEFT+offset_x, OSC_BOTTOM - offset_y); 
  }
  int curValue = positions.get(positions.size()-1);
  endShape();
  fill(0,0,255);
  text(curValue, (int)(OSC_LEFT+positions.size()/ (float)nPositions*OSC_WIDTH+20), (int)(OSC_BOTTOM - curValue/ (float)MAX_VALUE*OSC_HEIGHT));
}
