package ca.mcgill.ecse211.Lab5;

//imports
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
* This class navigates the robot through a series of waypoints and gets it to the final destination
* @author Garine Imamedjian
* @author Rebecca Weill
*/
public class TravelNav extends Thread {
	
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	int distance;
	int ringDistance = 15;
	private SampleProvider us;
	private float[] usData;
	
	/**
	 * This is the class constructor
	 * @param odometer
	 * @param leftMotor
	 * @param rightMotor
	 */
	public TravelNav(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, 
			SampleProvider us, float[] usData){
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.us = us;
		this.usData = usData;
		
	}
	
	//constants
	private static final int FORWARD_SPEED = 225;		//Speed of the motors when advancing to the next destination
	private static final int MOTOR_DIFF = 10;			//Constant subtracted to compensate for the inequality of the motors
	private static final int ROTATE_SPEED = 150;		//Speed of the motors when the robot is turning towards its next destination
	private static final double WHEEL_RAD = Lab5.WHEEL_RAD;
	private static final double TRACK = Lab5.TRACK;
	private static final double PI = Math.PI;
	private static final double TILE_SIZE = 30.48;
	
	
	private static boolean navigating = false;			//Boolean used to represent when the robot is navigating

	/**
	 * This is the run method required for the thread
	 * These are the different waypoints the robot needs to travel to (measured in tiles)
	 */
	@Override
	public void run() {

	}
	
	/**
	 * This method implements orientation change of the robot and the navigation needed to get to the next waypoint
	 * @param x (measured in tiles)
	 * @param y (measured in tiles)
	 */
	public void travelTo(double x, double y) {
		
		//reset motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			motor.stop();
			motor.setAcceleration(100);
		}
		
		navigating = true;
		
		//get x and y in centimeters
		x = x * TILE_SIZE;
		y = y * TILE_SIZE;
		
		//calculate trajectory path and angle
		double dX = x - odometer.getX();
		double dY = y - odometer.getY();
		
		//Find smallest necessary angle to turn to
		double dTh;
	    if(dX >= 0 & dY >= 0){
	      dTh = (180/Math.PI) * Math.atan(dX/dY);
	    }else if(dX >= 0 & dY < 0){
	      dTh = 90 + (180/Math.PI) * -Math.atan(dY/dX);
	    }else if(dX < 0 & dY < 0){
	      dTh = 180 + (180/Math.PI) * Math.atan(dX/dY);
	    }else{
	      dTh = 270 + (180/Math.PI) * -Math.atan(dY/dX);
	    }
		
		//rotate to correct angle
		Sound.beepSequenceUp();
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		turnTo(dTh);
		
		//Find distance that needs to be traveled
		double distance = Math.hypot(dX, dY);
		
		//move forward correct distance
		Sound.beepSequence();
		leftMotor.setSpeed(FORWARD_SPEED - MOTOR_DIFF);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(distance),true);
		rightMotor.rotate(convertDistance(distance),false);
	}
	
	/**
	 * This method implements orientation change of the robot and the navigation needed to get to the next waypoint
	 * It also detects whenever there is an object in it's way and goes into avoiding mode
	 * @param tileX (measured in tiles)
	 * @param tileY (measured in tiles)
	 */
	public void travelToRing(double tileX, double tileY) {
		
		//reset motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			motor.stop();
			motor.setAcceleration(500);
		}
		
		navigating = true;
		
		//get x and y in centimeters
		double x = tileX * TILE_SIZE;
		double y = tileY * TILE_SIZE;
		
		//calculate trajectory path and angle
		double dX = x - odometer.getX();
		double dY = y - odometer.getY();
		
		//Find smallest necessary angle to turn to;
		double dTh;
	    if(dX >= 0 & dY >= 0){
	      dTh = (180/Math.PI) * Math.atan(dX/dY);
	    }else if(dX >= 0 & dY < 0){
	      dTh = 90 + (180/Math.PI) * -Math.atan(dY/dX);
	    }else if(dX < 0 & dY < 0){
	      dTh = 180 + (180/Math.PI) * Math.atan(dX/dY);
	    }else{
	      dTh = 270 + (180/Math.PI) * -Math.atan(dY/dX);
	    }
		
		//rotate to correct angle
		Sound.beepSequenceUp();
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		turnTo(dTh);
		
		//Find distance that needs to be traveled
		double trajDistance = Math.hypot(dX, dY);
		
		//move forward correct distance
		Sound.beepSequence();
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(trajDistance),true);
		rightMotor.rotate(convertDistance(trajDistance),true);
		
		
		
		while (leftMotor.isMoving() || rightMotor.isMoving()) { // Scan the surrounding when the robot is moving

			us.fetchSample(usData,0);							// acquire data
			this.distance=(int)(usData[0]*100.0);				// extract from buffer, cast to int
			filter(this.distance);								//Pass through filter to eliminate false values
			
			//If object detected
			if(this.distance <= ringDistance){
				Sound.beep();
				leftMotor.stop(true); 			// Stop the robot and quit navigation mode
				rightMotor.stop(false);
				navigating = false;
			}
			
			try { Thread.sleep(50); } catch(Exception e){}		// Sleep
		}
		
		if (!this.isNavigating()){
			ringDetection(); 					// Implements bangbang controller to avoid the obstacle
			navigating = true; 					// re-enable navigation mode
			travelTo(tileX,tileY); 				// continue traveling to destination
		}
		
	}
	

	private void ringDetection() {
		// TODO Auto-generated method stub
		
	}

	/**
	 * This method changes the angle of the robot from it current theta to
	 * the theta needed to travel to the next waypoint
	 * @param th
	 */
	public void turnTo(double th) {
		
		//get current angle
		double currentAngle = odometer.getAng();
		double dT = 0;
		
		//Get smallest angle
		dT = th - currentAngle;
		if (dT > 180)
			dT = dT -360;
		else if (dT < -180)
			dT = dT + 360;
	
		//rotate robot to correct theta
		leftMotor.rotate(convertAngle(dT), true);
		rightMotor.rotate(-convertAngle(dT), false);
		
	}
	
	
	  /**
	   * This method is used to convert the distance needed to travel to the number of wheel rotations necessary to do so
	   * @param distance (in cm)
	   * @return rotations	
	   */
	  private static int convertDistance(double distance) {
	    int rotations = (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
	    return rotations;
	  }
	  
	  
	  /**
	   * This method is used to convert the angle needed to turn to the number of wheel rotations necessary to do so
	   * @param angle (in degrees)
	   * @return rotations	
	   */
	  private static int convertAngle(double angle) {
	    int rotations = convertDistance(Math.PI * TRACK * angle / 360.0);
	    return rotations;
	  }
	 

	  /**
	   * This method is used to know if the robot is currently navigating
	   * @return navigating	
	   */
	public boolean isNavigating() {
		return navigating;
	}
	
	  /**
	   * This method is used to filter out any false values given by the ultrasonic sensor
	   * @param distance	
	   */
	public void filter(int distance){
		
		int FILTER_OUT = 25;
		int filterControl = 0;
		
			if (distance >= 255 && filterControl < FILTER_OUT) {
				// bad value, do not set the distance var, however do increment the
				// filter value
				filterControl++;
			} else if (distance >= 255) {
				// We have repeated large values, so there must actually be nothing
				// there: leave the distance alone
				this.distance = distance;
			} else {
				// distance went below 255: reset filter and leave
				// distance alone.
				filterControl = 0;
				this.distance = distance;
			}
	}
	
}
