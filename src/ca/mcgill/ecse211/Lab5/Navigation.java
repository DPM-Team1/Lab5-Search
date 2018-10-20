package ca.mcgill.ecse211.Lab5;

//imports
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class takes care of the speeds set and then turning function needed when localizing
 * @author Garine Imamedjian
 * @author Rebecca Weill
 */
public class Navigation {
	
	//Initialize class variables
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	
	//Constants
	final static int MOTOR_SPEED = 90;			//Motor speed when turning
	final static int ACCELERATION = 2000;		//Acceleration rate to reduce drifting 
	final static double DEGREE_ERROR = 3.0;		//Acceptable error of degree
	private static final int FORWARD_SPEED = 150;		//Speed of the motors when advancing to the next destination
	private static final int MOTOR_DIFF = 1;			//Constant subtracted to compensate for the inequality of the motors
	private static final int ROTATE_SPEED = 100;		//Speed of the motors when the robot is turning towards its next destination

	/**
	 * This is the class constructor
	 * 
	 * @param odo
	 */
	public Navigation(Odometer odo) {
		this.odometer = odo;

		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];

		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
	}

	/**
	 * This method sets the motor speeds for both wheels at the same time
	 * @param lSpd
	 * @param rSpd
	 */
	public void setSpeeds(float lSpd, float rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}


	/**
	 * This method turns the robot to the angle passed
	 * it then stops the motors or not, depending on the stop boolean
	 * @param angle
	 * @param stop
	 */
	public void turnToUS(double angle, boolean stop) {

		//Calculate difference in angle
		double error = angle - this.odometer.getAng();

		//Do not turn and/or stop turning if angle to turn is too small
		while (Math.abs(error) > DEGREE_ERROR) {

			//Compute difference in angle
			error = angle - this.odometer.getAng();

			//Turn the right way
			if (error < -180.0) {
				this.setSpeeds(-MOTOR_SPEED, MOTOR_SPEED);
			} else if (error < 0.0) {
				this.setSpeeds(MOTOR_SPEED, -MOTOR_SPEED);
			} else if (error > 180.0) {
				this.setSpeeds(MOTOR_SPEED, -MOTOR_SPEED);
			} else {
				this.setSpeeds(-MOTOR_SPEED, MOTOR_SPEED);
			}
		}

		//boolean passed = true, stop motors
		if (stop) {
			this.setSpeeds(0, 0);
		}
	}
	
	
	
	
	

}
