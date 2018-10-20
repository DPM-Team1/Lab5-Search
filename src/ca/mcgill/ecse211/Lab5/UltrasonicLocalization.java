package ca.mcgill.ecse211.Lab5;

//import
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * This class orients the robot to the 0 angle using the Ultrasonic sensor
 * @author Garine Imamedjian
 * @author Rebecca Weill
 */
public class UltrasonicLocalization {
	public enum LocalizationMethod { FALLING_EDGE, RISING_EDGE };		//Localization method enumeration 

	//Initialize class variables
	private Odometer odo;
	private SampleProvider usSensor;
	private float[] usData;
	private LocalizationMethod locMet;
	private Navigation nav;

	//Constants
	private static final float MAX_DISTANCE_F = 50;		//Max distance for falling edge
	private static final float WALL_DISTANCE_R = 18;	//Wall distance for rising edge
	private static final float EDGE_DISTANCE_F = 16;	//Distance used to detect falling edge, 13 if middle
	private static final float EDGE_DISTANCE_R = 33;	//Distance used to detect rising edge
	private static final float MOTOR_SPEED = 70;		//Motor speed, slow to avoid huge drifting problem

	/**
	 * This is the class constructor
	 * 
	 * @param odo
	 * @param usSensor
	 * @param usData
	 * @param locType
	 */
	public UltrasonicLocalization(Odometer odo,  SampleProvider usSensor, float[] usData, LocalizationMethod locMet) {
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.locMet = locMet;
		this.nav = new Navigation(odo);
	}

	/**
	 * This method is the main method of the class that runs the localization method chosen
	 */
	public void doLocalization() {

		//Initialize the 2 angles that will be measured
		double A = 0;
		double B = 0;

		//Falling edge
		if (locMet == LocalizationMethod.FALLING_EDGE) {

			//Set speed to turn counter-clockwise
			nav.setSpeeds(MOTOR_SPEED,-MOTOR_SPEED);

			//Turn until it sees no wall
			while (true) {
				if (getFilteredData()>=MAX_DISTANCE_F) {
					break;
				}
			}
			
			// Turn until it sees wall, save angle A
			while (true) {
				if (getFilteredData()<EDGE_DISTANCE_F) {
					A = odo.getAng();
					Sound.beep();
					break;
				}
			}

			//Set speed to turn clockwise
			nav.setSpeeds(-MOTOR_SPEED,MOTOR_SPEED);
			
			//Turn until it sees no wall
			while (true) {
				if (getFilteredData()>=MAX_DISTANCE_F) {
					break;
				}
			}

			// Turn until it sees wall, save angle B
			while (true) {
				if (getFilteredData()<EDGE_DISTANCE_F) {
					B = odo.getAng();
					Sound.beep();
					break;
				}
			}

			//Calculate angle that the robot needs to turn
			double endAngle = getEndAngle(A,B);
			
			//Adjust negative and over 360 angles and turn
			if (endAngle<0) {
				nav.turnToUS(endAngle+360,true);
			} else if (endAngle>360){
				nav.turnToUS(endAngle-360,true);
			} else {
				nav.turnToUS(endAngle, true);
			}

			//Update the odometer position
			odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true});
		} 

		
		//Rising edge
		else {
			
			//Set speed to turn counter-clockwise
			nav.setSpeeds(MOTOR_SPEED,-MOTOR_SPEED);

			//Turn until it sees a wall
			while (true) {
				if (getFilteredData()<=WALL_DISTANCE_R) {
					break;
				}
			}
			
			//Turn until it sees no wall, save angle B
			while (true) {
				if (getFilteredData()>EDGE_DISTANCE_R) {
					B = odo.getAng();
					Sound.beep();
					break;
				} 
			}

			//Set speed to turn clockwise
			nav.setSpeeds(-MOTOR_SPEED,MOTOR_SPEED);

			//Turn until it sees a wall
			while (true) {
				if (getFilteredData()<=WALL_DISTANCE_R) {
					break;
				}
			}
			
			//Turn until it sees no wall, save angle A
			while (true) {
				if (getFilteredData()>EDGE_DISTANCE_R) {
					A = odo.getAng();
					Sound.beep();
					break;
				} 
			}
			
			
			//Calculate angle that the robot needs to turn
			double endAngle = getEndAngle(A,B);
			
			//Adjust negative and over 360 angles and turn
			if (endAngle<0) {
				nav.turnToUS(endAngle+360,true);
			} else if (endAngle>360){
				nav.turnToUS(endAngle-360,true);
			} else {
				nav.turnToUS(endAngle, true);
			}

			//Update the odometer position
			odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true});
		}
	}

	
	/**
	 * This method is a filter that sets the maximum distance
	 * detected by the Ultrasonic Sensor to MAX_DISTANCE_F
	 * @return distance
	 */
	public float getFilteredData() {
		usSensor.fetchSample(usData, 0);
		float distance = usData[0]*100;

		if (distance > MAX_DISTANCE_F) distance = MAX_DISTANCE_F;

		return distance;
	}

	
	/**
	 * This method calculates the angle needed to turn by the robot
	 * depending on the angles it detected
	 * @param a
	 * @param b
	 * @return endAngle
	 */
	private double getEndAngle(double a, double b) {
		if (a > b) {
			return ((a+b)/2 - 228);
		}
		return ((a+b)/2 - 45);
	}



}
