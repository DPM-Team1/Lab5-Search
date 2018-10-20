package ca.mcgill.ecse211.Lab5;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class ColorSensor {

	private static final Port colorPort = LocalEV3.get().getPort("S4");	
	

	public static void main() {
		
	    EV3ColorSensor sensor = new EV3ColorSensor(SensorPort.S4);
	    sensor.setFloodlight(false);
	    //LCD.drawString("Init", 2, 2);
	    LCD.setAutoRefresh(false);
	    SensorMode brightnessSensorMode = sensor.getRGBMode();
	    float[] sample = new float[brightnessSensorMode.sampleSize()];
	    
	    float RP, GP, BP;
	    
		LCD.clear();

	    while(true) {
	        brightnessSensorMode.fetchSample(sample, 0);
	        try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
			}
	        //LCD.drawString("entered ", 0, 3);
	      /*  LCD.refresh();
	        LCD.clear();
	       // System.out.println("R: " + sample[0] + " G: " + sample[1] + " B: " + sample[2]);
			LCD.drawString("R: ", 0, 0);
			LCD.drawString("G: ", 0, 1);
			LCD.drawString("B: ", 0, 2);
	        LCD.drawInt((int)(sample[0] *1000), 3, 0);
	        LCD.drawInt((int)(sample[1] *1000), 3, 1);
	        LCD.drawInt((int)(sample[2] *1000), 3, 2);*/
	        
	        RP = sample[0] / (sample[0] + sample[1] + sample[2]);
	        GP = sample[1] / (sample[0] + sample[1] + sample[2]);
	        BP = sample[2] / (sample[0] + sample[1] + sample[2]);
	        
	        if (GP > RP && GP > BP && RP > BP) {
	        	LCD.drawString("Green", 0, 3);
	        	Sound.beep();
	        }
	        else if (GP > RP && GP > BP && BP > RP) {
	        	LCD.drawString("Blue", 0, 3);
	        	Sound.beep();
	        }
	        else if (RP > GP && RP > BP && RP > 0.7) {
	        	LCD.drawString("Orange", 0, 3);
	        	Sound.beep();
	        }
	        else if (RP > GP && RP > BP && RP < 0.7) {
	        	LCD.drawString("Yellow", 0, 3);
	        	Sound.beep();
	        }
	        
	       
	       /* if (Button.waitForAnyPress() == Button.ID_ESCAPE) {
	        	System.exit(0);
	        }*/
	    }
	    
		
		
	}
	
	

}

