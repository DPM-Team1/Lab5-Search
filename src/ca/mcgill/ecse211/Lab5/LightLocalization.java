package ca.mcgill.ecse211.Lab5;

//imports
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

import java.util.Arrays;

import ca.mcgill.ecse211.Lab5.Navigation;

/**
 * This class navigates the robot to the (0,0,0) position using the light sensor
 * @author Garine Imamedjian
 * @author Rebecca Weill
 */
public class LightLocalization {
	
	//constants
	public static int MOTOR_SPEED = 70;				//Motor speed when advancing and turning
	public static int DISTANCE_FROM_EDGE = 10;		//Distance needed to back up to read y values
	public static int ACCELERATION = 600;			//Low acceleration rate to reduce drifting
	private static double lightSensorDistanceX = 4.5;		//Distance from center of robot to the light sensor horizontally
	private static double lightSensorDistanceY = 4;		//Distance from center of robot to the light sensor vertically
	private static double firstIntensityL;
	private static double firstIntensityR;
	private static double thresholdPercentage = 0.25;
	private static double lineValue = 0.27;
	
	//class variables
	private Odometer odo;
	private SampleProvider leftColorSensor;
	private float[] leftColorData;
	private SampleProvider rightColorSensor;
	private float[] rightColorData;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	Navigation navi;

	/**
	 * This is the class constructor
	 * 
	 * @param odo
	 * @param colorSensor
	 * @param colorData
	 * @param navi
	 */
	public LightLocalization(Odometer odo, SampleProvider leftColorSensor, float[] leftColorData, SampleProvider rightColorSensor, float[] rightColorData, Navigation navi) {
		//get incoming values for variables
		this.odo = odo;
		this.leftColorSensor = leftColorSensor;
		this.leftColorData = leftColorData;
		this.rightColorSensor = rightColorSensor;
		this.rightColorData = rightColorData;
		this.navi = navi;
		//set up motors
		EV3LargeRegulatedMotor[] motors = odo.getMotors();
		this.leftMotor = motors[0];		
		this.rightMotor = motors[1];
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
		
	}
	
	



	/**
	 * This method is the main method of the class that runs 
	 * the localization of the robot with the light sensor
	 * We assume that theta is 0 to start and correct the X and y positions
	 */
	public void doLocalization() {
		
		//variable to be used when correcting odometer
		double position[] = new double [3];
		//double leftColor = getLColorData()[0];
		//double rightColor = getRColorData()[0];
		double leftColor;
		double rightColor;

		
		firstIntensityL = getMedianIntensity(leftColorSensor);
		firstIntensityR = getMedianIntensity(rightColorSensor);

		//set the speeds of the motors, go forward
		leftMotor.setSpeed(MOTOR_SPEED);
		rightMotor.setSpeed(MOTOR_SPEED);
		leftMotor.forward();
		rightMotor.forward();
		
		leftColor= getMedianIntensity(leftColorSensor);
		rightColor = getMedianIntensity(rightColorSensor);

		//While not on crossing line, sleep to give time to other threads
		while (((firstIntensityL - leftColor)/firstIntensityL < thresholdPercentage)
			&& ((firstIntensityR - rightColor)/firstIntensityR < thresholdPercentage)) {
		//while (( leftColor > lineValue) && ( rightColor > lineValue)) {
			//leftColor = getLColorData()[0];
			//rightColor = getRColorData()[0];
			leftColor= getMedianIntensity(leftColorSensor);
			rightColor = getMedianIntensity(rightColorSensor);
			try {
				Thread.sleep(25);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		Sound.beep();
		//leftColor = getLColorData()[0];
		//rightColor = getRColorData()[0];
		leftColor= getMedianIntensity(leftColorSensor);
		rightColor = getMedianIntensity(rightColorSensor);
		
		
		//if (leftColor <= lineValue && rightColor <= lineValue) {
		if (((firstIntensityL - leftColor)/firstIntensityL >= thresholdPercentage)
				&& ((firstIntensityR - rightColor)/firstIntensityR >= thresholdPercentage)) {
			leftMotor.stop(true);
			rightMotor.stop(false);
			Sound.beep();
		}
		
		//if ((firstIntensityL - getMedianIntensity(leftColorSensor))/firstIntensityL >= thresholdPercentage) {
		//else if (leftColor <= lineValue && rightColor > lineValue) {
		else if (((firstIntensityL - leftColor)/firstIntensityL >= thresholdPercentage)
				&& ((firstIntensityR - rightColor)/firstIntensityR < thresholdPercentage)) {

			rightMotor.forward();
			//rightColor = getRColorData()[0];
			rightColor = getMedianIntensity(rightColorSensor);
			
			while ((firstIntensityR - rightColor)/firstIntensityR < thresholdPercentage) {
			//while (rightColor > lineValue) {
				//rightColor = getRColorData()[0];
				rightColor = getMedianIntensity(rightColorSensor);
				try {
					Thread.sleep(25);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			
			rightMotor.stop(false);
			Sound.beep();
		}
		
		//else if ((firstIntensityR - getMedianIntensity(rightColorSensor))/firstIntensityR >= thresholdPercentage) {
		//else if (rightColor <= lineValue && leftColor > lineValue) {
		else if (((firstIntensityL - leftColor)/firstIntensityL < thresholdPercentage)
				&& ((firstIntensityR - rightColor)/firstIntensityR >= thresholdPercentage)) {

			leftMotor.forward();
			//leftColor = getLColorData()[0];
			leftColor = getMedianIntensity(leftColorSensor);
			
			
			while ((firstIntensityL - leftColor)/firstIntensityL < thresholdPercentage) {
			//while (leftColor > lineValue) {
				//leftColor = getLColorData()[0];
				leftColor = getMedianIntensity(leftColorSensor);
				try {
					Thread.sleep(25);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			
			leftMotor.stop(false);
			Sound.beep();
		}
		
		// Crossed black line, stop and reset odometer
		//Sound.beep();
		//leftMotor.stop(true);
		//rightMotor.stop(true);
		position[0] = lightSensorDistanceX;
		position[1] = 0;
		position[2] = 0;
		boolean posBool[] = new boolean [3];
		posBool[0] = true;
		posBool[1] = false;
		posBool[2] = false;
		odo.setPosition(position, posBool);

		//now go backwards to reset and start with y
		leftMotor.rotate(-convertDistance(Lab5.WHEEL_RAD, DISTANCE_FROM_EDGE), true);
		rightMotor.rotate(-convertDistance(Lab5.WHEEL_RAD, DISTANCE_FROM_EDGE), false);

		//Rotate to go towards y axis
		leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90.0), true);
		rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90.0), false);

		firstIntensityL = getMedianIntensity(leftColorSensor);
		firstIntensityR = getMedianIntensity(rightColorSensor);

		//set the speeds of the motors, go forward
		leftMotor.setSpeed(MOTOR_SPEED);
		rightMotor.setSpeed(MOTOR_SPEED);
		leftMotor.forward();
		rightMotor.forward();
		
		leftColor= getMedianIntensity(leftColorSensor);
		rightColor = getMedianIntensity(rightColorSensor);

		//While not on crossing line, sleep to give time to other threads
		while (((firstIntensityL - leftColor)/firstIntensityL < thresholdPercentage)
			&& ((firstIntensityR - rightColor)/firstIntensityR < thresholdPercentage)) {
		//while (( leftColor > lineValue) && ( rightColor > lineValue)) {
			//leftColor = getLColorData()[0];
			//rightColor = getRColorData()[0];
			leftColor= getMedianIntensity(leftColorSensor);
			rightColor = getMedianIntensity(rightColorSensor);
			try {
				Thread.sleep(25);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		Sound.beep();
		//leftColor = getLColorData()[0];
		//rightColor = getRColorData()[0];
		leftColor= getMedianIntensity(leftColorSensor);
		rightColor = getMedianIntensity(rightColorSensor);
		
		
		//if (leftColor <= lineValue && rightColor <= lineValue) {
		if (((firstIntensityL - leftColor)/firstIntensityL >= thresholdPercentage)
				&& ((firstIntensityR - rightColor)/firstIntensityR >= thresholdPercentage)) {
			leftMotor.stop(true);
			rightMotor.stop(false);
			Sound.beep();
		}
		
		//if ((firstIntensityL - getMedianIntensity(leftColorSensor))/firstIntensityL >= thresholdPercentage) {
		//else if (leftColor <= lineValue && rightColor > lineValue) {
		else if (((firstIntensityL - leftColor)/firstIntensityL >= thresholdPercentage)
				&& ((firstIntensityR - rightColor)/firstIntensityR < thresholdPercentage)) {

			rightMotor.forward();
			//rightColor = getRColorData()[0];
			rightColor = getMedianIntensity(rightColorSensor);
			
			while ((firstIntensityR - rightColor)/firstIntensityR < thresholdPercentage) {
			//while (rightColor > lineValue) {
				//rightColor = getRColorData()[0];
				rightColor = getMedianIntensity(rightColorSensor);
				try {
					Thread.sleep(25);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			
			rightMotor.stop(false);
			Sound.beep();
		}
		
		//else if ((firstIntensityR - getMedianIntensity(rightColorSensor))/firstIntensityR >= thresholdPercentage) {
		//else if (rightColor <= lineValue && leftColor > lineValue) {
		else if (((firstIntensityL - leftColor)/firstIntensityL < thresholdPercentage)
				&& ((firstIntensityR - rightColor)/firstIntensityR >= thresholdPercentage)) {

			leftMotor.forward();
			//leftColor = getLColorData()[0];
			leftColor = getMedianIntensity(leftColorSensor);
			
			
			while ((firstIntensityL - leftColor)/firstIntensityL < thresholdPercentage) {
			//while (leftColor > lineValue) {
				//leftColor = getLColorData()[0];
				leftColor = getMedianIntensity(leftColorSensor);
				try {
					Thread.sleep(25);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			
			leftMotor.stop(false);
			Sound.beep();
		}
		
		// Crossed black line, stop and reset odometer
		//Sound.beep();
		//leftMotor.stop(true);
		//rightMotor.stop(true);
		position[0] = 0;
		position[1] = lightSensorDistanceY;
		position[2] = 0;
		posBool[0] = false;
		posBool[1] = true;
		posBool[2] = false;
		odo.setPosition(position, posBool);

		//advance to compensate for the offset of the light sensor
		leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, lightSensorDistanceY), true);
		rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, lightSensorDistanceY), false);

		//Rotate 90 degrees to face theta 0
		leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90.0), true);
		rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90.0), false);

		//Advance to set x to 0 and copensate for the offset of the light sensor
		leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, DISTANCE_FROM_EDGE + lightSensorDistanceX), true);
		rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, DISTANCE_FROM_EDGE + lightSensorDistanceX), false);

	}
	
	
	  /**
	   * This method is used to convert the angle needed to turn to the number of wheel rotations necessary to do so
	   * @param radius
	   * @param width
	   * @param angle
	   * @return rotations	
	   */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	  /**
	   * This method is used to convert the distance needed to travel to the number of wheel rotations necessary to do so
	   * @param radius
	   * @param distance
	   * @return rotations	
	   */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	
	private float getMedianIntensity(SampleProvider colorSensor) {
		
		int N= 5;
		
		float[] samples = new float[N];
		
		for (int i = 0; i < N ; i++) {
			
			colorSensor.fetchSample(samples, i);
		}
		
		Arrays.sort(samples);
		
		return (samples[N/2] + samples[(N/2) +1]) / 2.0f;
	}
	
	private float[] getLColorData() {
		leftColorSensor.fetchSample(leftColorData, 0);
		return leftColorData;
	}

	private float[] getRColorData() {
		rightColorSensor.fetchSample(rightColorData, 0);
		return rightColorData;
	}

}