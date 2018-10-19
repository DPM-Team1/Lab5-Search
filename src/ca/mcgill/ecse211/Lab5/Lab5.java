package ca.mcgill.ecse211.Lab5;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab5 {

	
	private static final EV3LargeRegulatedMotor leftMotor =
		      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor =
		      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	
	private static final Port colorPort = LocalEV3.get().getPort("S4");	
	
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	//public static final double WHEEL_RAD = 2.2;
	//public static final double TRACK = 9.7;

	public static void main(String[] args) {

	    }
	    
	
}

