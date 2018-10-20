package ca.mcgill.ecse211.Lab5;

import ca.mcgill.ecse211.Lab5.UltrasonicLocalization.LocalizationMethod;
import lejos.robotics.SampleProvider;

//beware of atan() / NaN error
public class Driver {
	
	//Initialize class variables
	private Odometer odo;
	private SampleProvider usSensor;
	private float[] usData;
	private SampleProvider leftSensor;
	private float[] leftData;
	private SampleProvider rightSensor;
	private float[] rightData;
	private TravelNav navi;
	
	public Driver(Odometer odo,  SampleProvider usSensor, float[] usData, 
			SampleProvider leftSensor, float[] leftData, SampleProvider rightSensor, float[] rightData, TravelNav travNav) {
		this.odo = odo;
		
		this.usSensor = usSensor;
		this.usData = usData;
		this.leftSensor = leftSensor;
		this.leftData = leftData;
		this.rightSensor = rightSensor;
		this.rightData = rightData;
		this.navi = travNav;
	}
	
	public void doSpiral(int N, int M, int startX, int startY, int endX, int endY) {
		
		navi.travelTo(startX,startY);
		
		odo.setX(0);
		odo.setY(0);
		
		navi.travelToRing(0, 3);
		navi.travelToRing(1, 3);
		navi.travelToRing(1, 0);
		navi.travelToRing(2, 0);
		navi.travelToRing(2, 3);
		navi.travelToRing(3, 3);
		navi.travelToRing(3, 0);
		
		/*int x, y;
		
		if (N < M) {

			y = 0;
			
			//while (!found) {
			while (true) {
				navi.travelToRing(M, y);
				break;
				
				
			}
		}
		
		else {
			navi.travelToRing(M, 0);
			
			//while (!found) {
			while (true) {

				break;
				
			}
		}
		
		
		
		*/
		
		navi.travelTo(M, N);
		
		
		
		
		
	}
	
	

}
