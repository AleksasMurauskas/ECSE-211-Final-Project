package finalProject;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import finalProject.Odometer;
import finalProject.Final;

public class USLocalizer {

	/**
	 * constants for our car and for the noise margin
	 */
	public static int ROTATION_SPEED = 220;
	private static final double NOISE_MARGIN_D = 27.00;
	private static final double NOISE_MARGIN_K = 18;
	private int SC;
	
	/**
	 * will be used to correct the angle
	 */
	private double deltaTheta;
	private Odometer odometer;
	
	/**
	 * list of data will be used to determine the falling&rising edge
	 */
	private float[] usData;
	private SampleProvider usDistance;
	
	/**
	 * motor
	 */
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	



	/**
	 * Constructor to initialize variables
	 * @param SC starting corner
	 * @param odo Odometer
	 * @param leftMotor EV3LargeRegulatedMotor for left motor
	 * @param rightMotor EV3LargeRegulatedMotor for right motor
	 * @param usDistance SampleProvider for detection of distance
	 */
	public USLocalizer(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,SampleProvider usDistance,int SC) {
		this.odometer = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.usDistance = usDistance;
		this.usData = new float[usDistance.sampleSize()];
		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);
		this.SC = SC;
	}

	
	/**
	 * A method to determine which localization method to write
	 * 
	 */
	public void localize() {
			Sound.beep();
			localizeFallingEdge();
	}

	
	
	/**
	 * A method to localize position using the falling edge
	 * 
	 */
	public void localizeFallingEdge() {

		double angleAlpha, angleBeta, turningAngle;

		// rotate counter-clock-wisely
		while (getWallDistace() < NOISE_MARGIN_D + NOISE_MARGIN_K) {
			leftMotor.backward();
			rightMotor.forward();
		}
		
		// rotate to the first falling angle
		while (getWallDistace() > NOISE_MARGIN_D) {
			leftMotor.backward();
			rightMotor.forward();
		}
		carStop();
		Sound.buzz();
		
		// record the first falling angle	
		angleAlpha = odometer.getXYT()[2];
		System.out.println(angleAlpha);
		// rotate clock-wisely
		while (getWallDistace() < NOISE_MARGIN_D + NOISE_MARGIN_K) {
			leftMotor.forward();
			rightMotor.backward();
		}

		// rotate to the second falling angle
		while (getWallDistace() > NOISE_MARGIN_D) {
			leftMotor.forward();
			rightMotor.backward();
		}
		carStop();
		Sound.buzz();
		
		//record the second falling angle
		angleBeta = odometer.getXYT()[2];
		System.out.println(angleAlpha);

		// calculate angle of rotation
		// this calculation is from the tutorial
		if (angleAlpha < angleBeta) {
			deltaTheta = 45 - (angleAlpha + angleBeta) / 2;
		} else if (angleAlpha > angleBeta) {
			deltaTheta = 225 - (angleAlpha + angleBeta) / 2;
		}
		turningAngle = deltaTheta + odometer.getXYT()[2];
		System.out.println(turningAngle);

		// rotate robot to the theta = 0.0 and we account for small error
		leftMotor.rotate(-convertAngle(Final.WHEEL_RAD,Final.TRACK, turningAngle), true);
		rightMotor.rotate(convertAngle(Final.WHEEL_RAD, Final.TRACK, turningAngle), false);

		// reset odometer to theta = 0
		 odometer.setTheta((0 + 360 -this.SC*90)%360);

	}

	/**
	 * A method to get the distance from our sensor
	 * 
	 * @return
	 */
	private int getWallDistace() {
		usDistance.fetchSample(usData, 0);
		return (int) (usData[0] * 100);
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of a angle to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	/**
	  * make the car stop
	  */
	  private void carStop() {
		    leftMotor.stop(true);
		    rightMotor.stop();
		  }

}
