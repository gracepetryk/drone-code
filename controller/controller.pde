import processing.serial.*;

import net.java.games.input.*;
import org.gamecontrolplus.*;
import org.gamecontrolplus.gui.*;
import net.java.games.input.*;



ControlIO control;
ControlDevice stick;

Serial arduino;
String formatControls()
{
    String pitch = Float.toString(map(stick.getSlider("pitch").getValue(), -1, 1, 0.8, 1.2));
    String yaw = Float.toString(map(stick.getSlider("yaw").getValue(), -1,1,0.8,1.2));
    String roll = Float.toString(map(stick.getSlider("roll").getValue(), -1,1,0.8,1.2));
    String throttle = Float.toString(map(stick.getSlider("throttle").getValue(), -1,1,0,1));
    return "<p"+pitch+"y"+yaw+"r"+roll+"t"+throttle+"bL>";
}

void setup()
{ 
    size(640,480);
    control = ControlIO.getInstance(this);
    stick = control.getMatchedDevice("dronecontrol");
    textSize(14);
    arduino = new Serial(this, Serial.list()[1], 57600);
    
}

void draw()
{
    clear();
    text("pitch: " + map(stick.getSlider("pitch").getValue(), -1,1,0.8,1.2), 10, 10);
    text("yaw: " + map(stick.getSlider("yaw").getValue(), -1,1,0.8,1.2), 10, 25);
    text("roll: " + map(stick.getSlider("roll").getValue(), -1,1,0.8,1.2), 10, 40);
    text("throttle: " + map(stick.getSlider("throttle").getValue(), -1,1,0,1), 10, 55);
    text(formatControls(), 10, 70);
    arduino.write(formatControls());
    delay(10);
    while(arduino.available() > 1)
    {
        String inBuffer = arduino.readString();   
        if (inBuffer != null) {
          println(inBuffer);
        }
    }
}