package ca.mcgill.ecse211.Lab5;

//imports
import lejos.utility.Timer;
import lejos.utility.TimerListener;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import ca.mcgill.ecse211.Lab5.*;

/**
 * This class calculates the position of the robot (x, y, and theta) using its internal tachometer count.
 * @author Garine Imamedjian
 * @author Rebecca Weill
 */
public class Odometer implements TimerListener {

	//Initialize class variables
	private Timer timer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	
	//Constants and variables
	private final int SLEEP_INT = 20;		//Sleep interval for the odometer to give time to other threads
	private double WHEEL_RAD;				//Wheel radius
	private double TRACK;					//distance between wheels
	private double x;
	private double y;
	private double theta;
	private double[] oldDH;
	private double[] dDH;

	/**
	 * This is the class constructor
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @param INTERVAL
	 * @param autostart
	 */
	public Odometer (EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, int INTERVAL, boolean autostart) {

		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		// radius and track
		this.WHEEL_RAD = Lab5.WHEEL_RAD;
		this.TRACK = Lab5.TRACK;

		this.x = 0.0;
		this.y = 0.0;
		this.theta = 90.0;
		this.oldDH = new double[2];
		this.dDH = new double[2];

		if (autostart) {
			// if the timeout interval is given as <= 0, default to 20ms timeout 
			this.timer = new Timer((INTERVAL <= 0) ? INTERVAL : SLEEP_INT, this);
			this.timer.start();
		} else
			this.timer = null;
	}

	// functions to start/stop the timerlistener
	public void stop() {
		if (this.timer != null)
			this.timer.stop();
	}
	public void start() {
		if (this.timer != null)
			this.timer.start();
	}

	/**
	 * This method calculates the displacement and heading of the robot
	 * @param data
	 */
	private void getDisplacementAndHeading(double[] data) {
		int leftTacho, rightTacho;
		leftTacho = leftMotor.getTachoCount();
		rightTacho = rightMotor.getTachoCount();

		data[0] = (leftTacho * WHEEL_RAD + rightTacho * WHEEL_RAD) * Math.PI / 360.0;
		data[1] = (rightTacho * WHEEL_RAD - leftTacho * WHEEL_RAD) / TRACK;
	}

	/**
	 * This method recomputes the odometer values using the displacement and heading changes
	 */
	public void timedOut() {
		this.getDisplacementAndHeading(dDH);
		dDH[0] -= oldDH[0];
		dDH[1] -= oldDH[1];

		// update the position in a critical region
		synchronized (this) {
			theta += dDH[1];
			theta = fixDegAngle(theta);

			x += dDH[0] * Math.cos(Math.toRadians(theta));
			y += dDH[0] * Math.sin(Math.toRadians(theta));
		}

		oldDH[0] += dDH[0];
		oldDH[1] += dDH[1];
	}

	/**
	 * This is the accessor method to get the x position
	 * @return x
	 */
	public double getX() {
		synchronized (this) {
			return x;
		}
	}

	/**
	 * This is the accessor method to get the y position
	 * @return y
	 */
	public double getY() {
		synchronized (this) {
			return y;
		}
	}

	/**
	 * This is the accessor method to get the theta position
	 * @return theta
	 */
	public double getAng() {
		synchronized (this) {
			return theta;
		}
	}

	/**
	 * This is the mutator method to set the full position of the robot (x, y and theta)
	 * @param position
	 * @param update
	 */
	public void setPosition(double[] position, boolean[] update) {
		synchronized (this) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	/**
	 * This is the accessor method to get the full position of the robot (x, y and theta)
	 * @param position
	 */
	public void getPosition(double[] position) {
		synchronized (this) {
			position[0] = x;
			position[1] = y;
			position[2] = theta;
		}
	}

	/**
	 * This is the mutator method to set the x position
	 * @param x
	 */
	public void setX(double xPos) {
		x = xPos;
	}

	/**
	 * This is the mutator method to set the y position
	 * @param y
	 */
	public void setY(double yPos) {
		y = yPos;
	}

	/**
	 * This is the mutator method to set the theta position
	 * @param t
	 */
	public void setT(double t) {
		theta = t;
	}


	/**
	 * This is the accessor method to get the left and right motors
	 * @return leftMotor
	 * @return rightMotor
	 */
	public EV3LargeRegulatedMotor [] getMotors() {
		return new EV3LargeRegulatedMotor[] {this.leftMotor, this.rightMotor};
	}

	/**
	 * This is the accessor method to get the left motor
	 * @return leftMotor
	 */
	public EV3LargeRegulatedMotor getLeftMotor() {
		return this.leftMotor;
	}

	/**
	 * This is the accessor method to get the right motor
	 * @return rightMotor
	 */
	public EV3LargeRegulatedMotor getRightMotor() {
		return this.rightMotor;
	}

	/**
	 * This method corrects the angle if it is negative or over 360 degrees
	 * @param angle
	 * @return angle %360.0
	 */
	public static double fixDegAngle(double angle) {
		if (angle < 0.0)
			angle = 360.0 + (angle % 360.0);

		return angle % 360.0;
	}

	/**
	 * This method computes the angle between 2 angles
	 * @param a
	 * @param b
	 * @return d-360.0
	 */
	public static double minimumAngleFromTo(double a, double b) {
		double d = fixDegAngle(b - a);

		if (d < 180.0)
			return d;
		else
			return d - 360.0;
	}
}
