package ca.mcgill.ecse211.Lab5;

import ca.mcgill.ecse211.Lab5.Navigation;
import ca.mcgill.ecse211.Lab5.Odometer;
import ca.mcgill.ecse211.Lab5.UltrasonicLocalization;
import ca.mcgill.ecse211.Lab5.LightLocalization;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab5 {

	
	private static final EV3LargeRegulatedMotor leftMotor =
		      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor rightMotor =
		      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	
	private static final Port usPort = LocalEV3.get().getPort("S3");	
	private static final Port colorPort = LocalEV3.get().getPort("S4");	
	private static final Port leftPort = LocalEV3.get().getPort("S2");	
	private static final Port rightPort = LocalEV3.get().getPort("S1");	
	
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 9.9;

	public static void main(String[] args) {
		

		// Setting up the Ultrasonic Sensor
		@SuppressWarnings("resource")							    // Because we don't bother to close this resource
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);		// usSensor is the instance
		SampleProvider usValue = usSensor.getMode("Distance");		// usValue provides samples from this instance
		float[] usData = new float[usValue.sampleSize()];			// usData is the buffer in which data are returned

		//Setting up the Color Sensor
		SensorModes colorSensor = new EV3ColorSensor(colorPort);		// colorSensor is the instance
		SampleProvider colorValue = colorSensor.getMode("Red");			// colorValue provides samples from this instance
		float[] colorData = new float[colorValue.sampleSize()];			// colorData is the buffer in which data are returned
		
		//Setting up the Color Sensor
		SensorModes leftSensor = new EV3ColorSensor(leftPort);		// colorSensor is the instance
		SampleProvider leftColorValue = leftSensor.getMode("Red");			// colorValue provides samples from this instance
		float[] leftColorData = new float[leftColorValue.sampleSize()];			// colorData is the buffer in which data are returned
		
		//Setting up the Color Sensor
		SensorModes rightSensor = new EV3ColorSensor(rightPort);		// colorSensor is the instance
		SampleProvider rightColorValue = rightSensor.getMode("Red");			// colorValue provides samples from this instance
		float[] rightColorData = new float[rightColorValue.sampleSize()];			// colorData is the buffer in which data are returned
		
		//Setting up display + instantiation
		final TextLCD t = LocalEV3.get().getTextLCD();
		int buttonChoice;
		Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);

		//Display choice of user 
		do {
			t.clear();

			t.drawString("< Left  | Right >", 0, 0);
			t.drawString("        |        ", 0, 1);
			t.drawString(" do     | do    ", 0, 2);
			t.drawString(" Falling| Rising ", 0, 3);
			t.drawString(" Edge   | Edge ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} 
		//Start the program that the user selected
		while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		//Falling edge
		if (buttonChoice == Button.ID_LEFT){
			UltrasonicLocalization usl = new UltrasonicLocalization(odo, usValue, usData, UltrasonicLocalization.LocalizationMethod.FALLING_EDGE);
			usl.doLocalization();
		} 
		//Rising edge
		else {
			UltrasonicLocalization usl = new UltrasonicLocalization(odo, usValue, usData, UltrasonicLocalization.LocalizationMethod.RISING_EDGE);
			usl.doLocalization();
		}

		Navigation navi = new Navigation(odo);
		
		//Wait for button to be pressed - TA measure time
		buttonChoice = Button.waitForAnyPress();
		while (buttonChoice != Button.ID_ENTER && buttonChoice != Button.ID_ESCAPE);

		//Light Localization
		if (buttonChoice == Button.ID_ENTER) {
			LightLocalization lsl = new LightLocalization(odo, leftColorValue, leftColorData, rightColorValue, rightColorData, navi);
			lsl.doLocalization();	
		}
		//Terminate
		else
			System.exit(0);
		
		//Terminate
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);	

	 }
	    
	
}

