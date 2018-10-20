package ca.mcgill.ecse211.Lab5;

//imports
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

/**
 * This class is used to display the content of 
 * the odometer variables (x, y, Theta)
 * and the Ultrasonic sensor distance
 * @author Garine Imamedjian
 * @author Rebecca Weill
 */
public class Display implements TimerListener{
	public static final int LCD_REFRESH = 100;		//interval at which the display is refreshed to give other threads time
	
	//Initialize class variables
	private Odometer odo;
	private UltrasonicLocalization usl;
	private LightLocalization ll;
	private Timer lcdTimer;
	private TextLCD LCD = LocalEV3.get().getTextLCD();;

	// arrays for displaying data
	private double [] pos;

	/**
	 * This is the class constructor
	 * @param odo
	 * @param usl
	 */
	public Display(Odometer odo, UltrasonicLocalization usl) {
		this.odo = odo;
		this.lcdTimer = new Timer(LCD_REFRESH, this);
		this.usl = usl;

		// initialise the arrays for displaying data
		pos = new double [3];

		// start the timer
		lcdTimer.start();
	}

	/**
	 * This is the method that is run with the timer 
	 * to display the different desired values:
	 * from Odometer : x, y, t
	 * and the distance from the ultrasonic sensor
	 */
	public void timedOut() { 
		odo.getPosition(pos);
		double distance = usl.getFilteredData();
		LCD.clear();
		LCD.drawString("X: ", 0, 0);
		LCD.drawString("Y: ", 0, 1);
		LCD.drawString("T: ", 0, 2);
		LCD.drawString("D: ", 0, 3);
		LCD.drawInt((int)(pos[0]), 3, 0);
		LCD.drawInt((int)(pos[1]), 3, 1);
		LCD.drawInt((int)pos[2], 3, 2);
		LCD.drawString(String.valueOf(distance), 3, 3);
	}
}