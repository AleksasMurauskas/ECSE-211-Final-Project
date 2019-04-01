package weightMeasure;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class WeightMain {
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		Button.waitForAnyPress();
		Grabbing grab = new Grabbing(leftMotor,rightMotor);
		grab.grab();
		//grab.setFloat();
		//int pre = grab.getToco();
		//grab.carMoveBackward(10);
		//int cur = grab.getToco();
		//System.out.println(cur - pre);
	}

}
