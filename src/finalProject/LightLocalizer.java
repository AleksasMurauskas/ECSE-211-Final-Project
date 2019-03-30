package finalProject;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;


public class LightLocalizer {

	/**
	 * constants for our car
	 */
	private static final int ROTATION_SPEED = 220;
	private static double distanceLS = 11.4; //distance between the light sensor and the center of rotation: change in function of robot
	public static final double tileSize = 30.48;
	/**
	 * constants will be used in calculation
	 */
	private static final double ROUND_ANGLE = 360;
	private static final double RIGHT_ANGLE = 90;
	
	/**
	 * threshold for light sensor detection
	 */
	private static final double COLOR_THRESHOLD_A = 0.20;
	private static final double COLOR_THRESHOLD_B = 0.38;
	
	/**
	 * odometer for data correction
	 */
	public static Odometer odometer;
	
	/**
	 * car motors for navigate
	 */
	public static EV3LargeRegulatedMotor leftMotor;
	public static EV3LargeRegulatedMotor rightMotor;
	

	/**
	 * Instantiate the EV3 Color Sensor
	 */
	private static final EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	private float sample;
	private SensorMode idColour;
	double[] thetaOnLine;

	/**
	 * Constructor for our light localizer
	 * @param SC starting corner
	 * @param odometer 
	 * @param leftMotor
	 * @param rightMotor
	 */
	public LightLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {

		LightLocalizer.odometer = odometer;
		LightLocalizer.leftMotor = leftMotor;
		LightLocalizer.rightMotor = rightMotor;

		idColour = lightSensor.getRedMode(); // set the sensor light to red
		thetaOnLine = new double[4];
	}

	/**
	 * This method localizes the robot using the light sensor to precisely move to
	 * the right location
	 */
	public void localize() {
		double x1 = 0;
		double x2 = 0;
		double y1 = 0;
		double y2 = 0;
		double angle = 0;
		//initialize the index of the number of lines being detected
		int index = 0;
		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);
		
		
		// ensure that we are in the right postion before rotating
		findPostion();

		
		// Scan all four lines and record our angle
		while (index < 4) {	
			
			//rotating clock-wisely
			leftMotor.setSpeed(ROTATION_SPEED);
			rightMotor.setSpeed(ROTATION_SPEED);
			leftMotor.forward();
			rightMotor.backward();
			
			//fetch sample continuously
			sample = fetchSample();
			if (sample < COLOR_THRESHOLD_A) {
				angle = odometer.getXYT()[2];
				//get the theta for positive x axis, negative y axis, negative x axis, and positive y axis
				 if((angle - 0 < 45) || (360 - angle <45)) {
			          x1 = angle;
			        }
			        else if(Math.abs(angle - 90) < 45) {
			          y1 = angle;
			        }
			        else if (Math.abs(angle-180) < 45) {
			          x2 = angle;
			        }
			        else if (Math.abs(angle-270) < 45) {
			          y2 = angle;
			        }
				Sound.beep();
				index++;
			}
		}
		
		//stop when we detect the four line which is the positive y axis
		leftMotor.stop(true);
		rightMotor.stop();
		//now calculate the current position by trigonometry
		double thetaX, thetaY, deltaX, deltaY, deltaTheta;
		thetaX= Math.toRadians((x2-x1)/2);
		thetaY= Math.toRadians((y2-y1)/2);
		deltaX = Math.cos(thetaX)*distanceLS;
		System.out.println(deltaX);
		deltaY = Math.cos(thetaY)*distanceLS;
		System.out.println(deltaY);
		deltaTheta=odometer.getXYT()[2];
	    odometer.setX(tileSize-deltaX);
	    odometer.setY(tileSize-deltaY);
	}
	/**
	 * the overload method of localize() which will let the car do the localization near the target point
	 * @param x x coordinate
	 * @param y y coordinate
	 */
	public void localize(int x,int y,int corner, boolean farAway) {
		double x1 = 0;
		double x2 = 0;
		double y1 = 0;
		double y2 = 0;
		double angle = 0;
		//initialize the index of the number of lines being detected
		int index = 0;
		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);
		
		
		// ensure that we are in the right postion before rotating
		if(!farAway) {
			findPostion();
		}else {
			closeToOrigin(corner);
		}
		
		// Scan all four lines and record our angle
		while (index < 4) {	
			
			//rotating clock-wisely
			leftMotor.setSpeed(ROTATION_SPEED);
			rightMotor.setSpeed(ROTATION_SPEED);
			leftMotor.forward();
			rightMotor.backward();
			
			//fetch sample continuously
			sample = fetchSample();
			if (sample < COLOR_THRESHOLD_A) {
				angle = odometer.getXYT()[2];
				//get the theta for positive x axis, negative y axis, negative x axis, and positive y axis
				 if((angle - 0 < 50) || (360 - angle <50)) {
			          x1 = angle;
			        }
			        else if(Math.abs(angle - 90) < 50) {
			          y1 = angle;
			        }
			        else if (Math.abs(angle-180) < 50) {
			          x2 = angle;
			        }
			        else if (Math.abs(angle-270) < 50) {
			          y2 = angle;
			        }
				Sound.beep();
				index++;
			}
		}
		
		//stop when we detect the four line which is the positive y axis
		leftMotor.stop(true);
		rightMotor.stop();

		//now calculate the current position by trigonometry
		double thetaX, thetaY, deltaX, deltaY, deltaTheta;
		thetaX= Math.toRadians((x2-x1)/2);
		thetaY=Math.toRadians((y2-y1)/2);;
		deltaX = Math.cos(thetaX)*distanceLS;
		deltaY = Math.cos(thetaY)*distanceLS;
		deltaTheta=odometer.getXYT()[2];
		if (corner == 0) {	  
		    odometer.setX(x*tileSize-deltaX);
		    odometer.setY(y*tileSize-deltaX);
		}else if (corner == 1) {
			odometer.setX(x*tileSize + deltaX);
			odometer.setY(y*tileSize-deltaY);
		}else if (corner == 2) {
			odometer.setX(x*tileSize + deltaX);
			odometer.setY(y*tileSize + deltaY);
		}else {
			odometer.setX(x*tileSize - deltaX );
			odometer.setY(y*tileSize + deltaY);
		}
		
		
		// set the calculated position
		//odometer.setXYT(deltaX,deltaY,deltaTheta);

	}

	
	/**
	 * This method moves the robot's sensor a bit in front of the origin
	 * so that the car can start rotating
	 */
	public void findPostion() {

		//turn to the angle which face the origin
		LightLocalizer.leftMotor.setSpeed(ROTATION_SPEED);
		LightLocalizer.rightMotor.setSpeed(ROTATION_SPEED);
		LightLocalizer.leftMotor.rotate(convertAngle(Final.WHEEL_RAD, Final.TRACK,RIGHT_ANGLE/2), true);
		LightLocalizer.rightMotor.rotate(-convertAngle(Final.WHEEL_RAD, Final.TRACK, RIGHT_ANGLE/2), false);


		// get sample
		sample = fetchSample();

		// move forward past the origin until light sensor sees the line
		while (sample > COLOR_THRESHOLD_B) {
			sample = fetchSample();
			leftMotor.forward();
			rightMotor.forward();
		}
		
		//stop the motor
		leftMotor.stop(true);
		rightMotor.stop();

		// Move forwards so our origin is close to origin
		LightLocalizer.leftMotor.setSpeed(ROTATION_SPEED);
		LightLocalizer.rightMotor.setSpeed(ROTATION_SPEED);
		leftMotor.rotate(-convertDistance(Final.WHEEL_RAD, distanceLS + 4), true);
		rightMotor.rotate(-convertDistance(Final.WHEEL_RAD, distanceLS + 4), false);


	}
	private void closeToOrigin(int corner) {
	    carSetSpeed(ROTATION_SPEED);
			  
	    //drive to location where light sensor can get 4 lines
	    sample = fetchSample();	  
	    while (sample > COLOR_THRESHOLD_B) {
	    	sample = fetchSample();
	    	carMoveForward();
	    }
	    carStop();
	    //sees line, back up by 15 cms
	    leftMotor.rotate(convertDistance(Final.WHEEL_RAD, distanceLS + 2), true);
	    rightMotor.rotate(convertDistance(Final.WHEEL_RAD, distanceLS + 2), false);
			  
	    carSetSpeed(ROTATION_SPEED);
	    //turns 90 degrees to the right
	    
	    carTurnRight(RIGHT_ANGLE);
	   
	    carSetSpeed(ROTATION_SPEED);
	    //drives forward until line
	    sample = fetchSample();
	    while (sample > COLOR_THRESHOLD_B) {
	    	sample = fetchSample();
	    	carMoveForward();
	    }
	    Sound.beep();
	    carStop();
	    carSetSpeed(ROTATION_SPEED);
	    //sees line, back up
	    carMove(-(distanceLS + 2));
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
	 * This method allows us to convert distance we want to travel to the number of rotation the wheels
	 * @param radius of the wheel
	 * @param distance between wheels
	 * @param angle of the turn
	 * @return number of rotations
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	
	
	
	/**
	 * This method gets the color value of the light sensor
	 * @return a float represents the color value
	 */
	public float fetchSample() {
		float[] colorValue = new float[idColour.sampleSize()];
		idColour.fetchSample(colorValue, 0);
		return colorValue[0];
	}
	private void carMoveForward() {
	    leftMotor.forward();
	    rightMotor.forward();
	  }
	  private void carMove(double distance) { 
	    leftMotor.rotate(convertDistance(Final.WHEEL_RAD, distance), true);
	    rightMotor.rotate(convertDistance(Final.WHEEL_RAD, distance), false);
	  }
	  private void carTurnRight(double angles) {
	    leftMotor.rotate(convertAngle(Final.WHEEL_RAD, Final.TRACK,angles), true);
	    rightMotor.rotate(-convertAngle(Final.WHEEL_RAD, Final.TRACK, angles), false);
	  }
	  private void carTurnLeft(double angles) {
	    leftMotor.rotate(-convertAngle(Final.WHEEL_RAD, Final.TRACK,angles), true);
	    rightMotor.rotate(convertAngle(Final.WHEEL_RAD, Final.TRACK, angles), false);
	  }
		
	  private void carStop() {
	    leftMotor.stop(true);
	    rightMotor.stop();
	  }
	  private void carRotate() {
	    leftMotor.forward();
	    rightMotor.backward();
	  }
	  private void carSetSpeed( int speed) {
	    leftMotor.setSpeed(speed);
	    rightMotor.setSpeed(speed);
	  }

}
