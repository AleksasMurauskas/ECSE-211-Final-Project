package weightMeasure;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Grabbing  {

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	public static final double WHEEL_RAD = 2.10;
	public static final double TRACK = 13.42 ;
	/**
	 * get motor instance
	 */
	private static final EV3LargeRegulatedMotor GRAB_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	
	/**
	 * set the speed of the grabber
	 */
	private static final int GRABING_SPEED = 70;
	
	/**
	 * set the angle the car will make before grabbing
	 */
	private static final int PRE_GRAB_TURNING_ANGLE = 40;
	
	/**
	 * set the turning angle of the grabber
	 */
	private static final int GRAB_ANGLE = 60;
	
	/**
	 * set the distance we want to travel forward before we grab
	 */
	private static final int PRE_GRAB_DISTANCE = 4;
	
	/**
	 * set the distance we want to travel backward after we grab
	 */
	private static final int POST_GRAB_DISTANCE = 15;
	
	/**
	 * constructor
	 */
	public Grabbing(EV3LargeRegulatedMotor leftMotor,EV3LargeRegulatedMotor rightMotor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}


	/**
	 * This method allow us to grab
	 */
	public void grab() {
		GRAB_MOTOR.rotate(-GRAB_ANGLE);
		int pre = GRAB_MOTOR.getTachoCount();
		carTurnRight(PRE_GRAB_TURNING_ANGLE);
		carMove(PRE_GRAB_DISTANCE);
		GRAB_MOTOR.setSpeed(GRABING_SPEED);
		GRAB_MOTOR.rotate(2*GRAB_ANGLE);
		System.out.println(GRAB_MOTOR.getTachoCount() - pre);
	}
	
	
	/**
	 * This method allow us to leave the can
	 */	
	public void leave() {
	
		GRAB_MOTOR.setSpeed(GRABING_SPEED);
		GRAB_MOTOR.rotate(-GRAB_ANGLE);
		carMoveBackward(POST_GRAB_DISTANCE);
		
	}
	
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	private void carTurnRight(float angles) {
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK,angles), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD,TRACK, angles), false);
	}
	private void carTurnLeft(float angles) {
		leftMotor.rotate(-convertAngle(WHEEL_RAD,TRACK,angles), true);
		rightMotor.rotate(convertAngle(WHEEL_RAD,TRACK, angles), false);
	}
	private static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	private void carMove(double distance) {
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		leftMotor.rotate(convertDistance(WHEEL_RAD, distance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, distance), false);
	}
	public void carMoveBackward(double distance) {
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		leftMotor.rotate(-convertDistance(WHEEL_RAD, distance), true);
		rightMotor.rotate(-convertDistance(WHEEL_RAD, distance), false);
	}
	public void setFloat() {
		//GRAB_MOTOR.forward();
		GRAB_MOTOR.flt();
	}
	public int getToco() {
		return GRAB_MOTOR.getTachoCount();
	}
	public void setfixed() {
		//GRAB_MOTOR.
	}
	
	
}
