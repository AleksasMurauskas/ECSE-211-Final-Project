package ca.mcgill.ecse211.lab3;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Grabbing  {

	
	/**
	 * get motor instance
	 */
	private static final EV3LargeRegulatedMotor GRAB_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	
	/**
	 * set the speed of the grabber
	 */
	private static final int GRABING_SPEED = 50;
	
	/**
	 * set the angle the car will make before grabbing
	 */
	private static final int PRE_GRAB_TURNING_ANGLE = 50;
	
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
	public Grabbing() {
		
	}


	/**
	 * This method allow us to grab
	 */
	public static void grab() {	
		Navigation.turnRight(PRE_GRAB_TURNING_ANGLE);
		Navigation.travelForward(PRE_GRAB_DISTANCE);
		GRAB_MOTOR.setSpeed(GRABING_SPEED);
		GRAB_MOTOR.rotate(GRAB_ANGLE);				
	}
	
	
	/**
	 * This method allow us to leave the can
	 */	
	public static void leave() {
	
		GRAB_MOTOR.setSpeed(GRABING_SPEED);
		GRAB_MOTOR.rotate(-GRAB_ANGLE);
		Navigation.travelBackward(POST_GRAB_DISTANCE);
		
	}
	
	
	
	
	
	
	
	
}
