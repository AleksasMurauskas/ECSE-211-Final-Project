package finalProject;
import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import finalProject.Odometer;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
public class Final {
	 //colorDetector sensor related objects
	 static EV3ColorSensor colorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
	   
	  // Ultrasonic sensor related objects
	 private static SampleProvider sampleProviderUS;
	 static EV3UltrasonicSensor usSensor;
	 static float[] usValues;
	 private static final String SERVER_IP = "192.168.2.26";
	 private static final int TEAM_NUMBER = 12;
	 public static Odometer odometer;
	 private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	 private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	 private static final Port usPort = LocalEV3.get().getPort("S1"); //port for the Ultrasonic Sensor
	 private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	 public static final double WHEEL_RAD = 2.10;
	 public static final double TRACK = 13.42 ;
	 public static final double tileSize = 30.48;
	 public static final String[] colors = {"blue","green","yellow","red"};
	  // Enable/disable printing of debug info from the WiFi class
	 private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

	  @SuppressWarnings("rawtypes")
	 public static void main(String[] args) throws OdometerExceptions {
		  
		  LightLocalizer ll;
		  USLocalizer usLocal;
		  Navigation navi;
		  odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
	      System.out.println("Running..");

	     // Initialize WifiConnection class
	     WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);
	     try {
	         /*
	          * getData() will connect to the server and wait until the user/TA presses the "Start" button
	          * in the GUI on their laptop with the data filled in. Once it's waiting, you can kill it by
	          * pressing the upper left hand corner button (back/escape) on the EV3. getData() will throw
	          * exceptions if it can't connect to the server (e.g. wrong IP address, server not running on
	          * laptop, not connected to WiFi router, etc.). It will also throw an exception if it connects
	          * but receives corrupted data or a message from the server saying something went wrong. For
	          * example, if TEAM_NUMBER is set to 1 above but the server expects teams 17 and 5, this robot
	          * will receive a message saying an invalid team number was specified and getData() will throw
	          * an exception letting you know.
	          */
	         Map data = conn.getData();
	         System.out.println("Map:\n" + data);

	         int redTeam = ((Long) data.get("RedTeam")).intValue();
	         int redCorner = ((Long) data.get("RedCorner")).intValue();
	         usSensor = new EV3UltrasonicSensor(usPort);
	         sampleProviderUS = usSensor.getMode("Distance");
	         usValues = new float[sampleProviderUS.sampleSize()];
	         usLocal = new USLocalizer(odometer,leftMotor,rightMotor,sampleProviderUS,redCorner);
	         int red_ll_x = ((Long)data.get("Red_LL_x")).intValue();
	         int red_ll_y = ((Long)data.get("Red_LL_y")).intValue();
	         int red_ur_x = ((Long)data.get("Red_UR_x")).intValue();
	         int red_ur_y = ((Long)data.get("Red_UR_y")).intValue();
	         ll = new LightLocalizer(odometer,leftMotor,rightMotor,redCorner);
	         int tnr_ll_x =  ((Long) data.get("TNR_LL_x")).intValue();
	         int tnr_ll_y = ((Long)data.get("TNR_LL_y")).intValue();
	         int tnr_ur_x = ((Long)data.get("TNR_UR_x")).intValue(); 
	         int tnr_ur_y = ((Long)data.get("TNR_UR_y")).intValue();
	         int szr_ll_x = ((Long)data.get("SZR_LL_x")).intValue();
	         int szr_ll_y = ((Long)data.get("SZR_LL_y")).intValue();
	         int szr_ur_x = ((Long)data.get("SZR_UR_x")).intValue();
	         int szr_ur_y = ((Long)data.get("SZR_UR_y")).intValue();
	         int island_ll_x = ((Long)data.get("Island_LL_x")).intValue();
	         int island_ll_y = ((Long)data.get("Island_LL_y")).intValue();
	         int island_ur_x = ((Long)data.get("Island_UR_x")).intValue();
	         int island_ur_y = ((Long)data.get("Island_UR_y")).intValue();
	         navi = new Navigation(leftMotor,rightMotor,1,szr_ur_x,szr_ll_x,szr_ur_y,szr_ll_y,tnr_ur_x,tnr_ll_x,tnr_ur_y,tnr_ll_y,red_ur_x,red_ll_x,red_ur_y,red_ll_y,island_ur_x,island_ll_x,island_ur_y,island_ll_y,sampleProviderUS, usValues,ll);
	         usLocal.localize();
	         ll.localize();
	         navi.work();
	       } catch (Exception e) {
	         System.err.println("Error: " + e.getMessage());
	       }

	 }

}
