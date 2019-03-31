package finalProject;

import finalProject.ColorClassification;
import finalProject.LightLocalizer;
import finalProject.Odometer;
import finalProject.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;

public class Navigation {
	  private static final int FORWARD_SPEED = 200;
	  public static final String[] colors = {"blue","green","yellow","red"};
	  private int targetColour;
	  private int SC;
	  private int szrLLx;
	  private int tunLLx;
	  private int islLLx;
	  private int redLLx;
	  private int szrURx;
	  private int tunURx;
	  private int islURx;
	  private int redURx;
	  private int szrLLy;
	  private int tunLLy;
	  private int islLLy;
	  private int redLLy;
	  private int szrURy;
	  private int tunURy;
	  private int islURy;
	  private int redURy;
	  private boolean upsearch;
	  private boolean downsearch;
	  private boolean leftsearch;
	  private boolean rightsearch;
	  
	  public boolean found = false;
	  public boolean obstacle;
	  public static final int size = 12;
	  public static final double tileSize = 30.48;
	  private static final int ROTATE_SPEED = 100;
	  public static Odometer odometer = Final.odometer;
	  private static EV3LargeRegulatedMotor leftMotor ;
	  private static EV3LargeRegulatedMotor rightMotor;
	  public static EV3ColorSensor colorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
	  private static ColorClassification colorDetector = new ColorClassification(colorSensor);
	  static final double leftRadius = 2.10;
	  static final double rightRadius = 2.10;
	  static final double track = 13.6;
	  private static final int safeDistance= 8;
	  public static EV3GyroSensor gySensor = new EV3GyroSensor(SensorPort.S2);
	  private static SampleProvider gyAngles = gySensor.getAngleMode();
	  private SampleProvider us;
	  private float[] usValues;
	  private static float[] angles = new float[gyAngles.sampleSize()]; 
	  private boolean travelling;
	  private LightLocalizer ll;
	  static double jobx;
	  static double joby;
	  public boolean search;
	  private float[] searchHelper = new float[size];
	  private int helperCounter = 0;
	  private int[][] searchMap ; 
	  private static final double COLOR_THRESHOLD_B = 0.38;
	public Navigation(EV3LargeRegulatedMotor leftmotor,EV3LargeRegulatedMotor rightmotor,SampleProvider us, float[] usValues,LightLocalizer ll) throws OdometerExceptions {
		this.SC = 0;
		this.us = us;
		this.usValues = usValues;
		this.obstacle = false;
		this.found = false;
		this.search = false;
		this.travelling = false;
		leftMotor = leftmotor;
	    rightMotor = rightmotor;
	    jobx = 0;
	    joby = 0;
	    this.ll = ll;
	  }
	public void setData (int targetColour,int szrURx,int szrLLx,int szrURy,int szrLLy,int tunURx,int tunLLx,int tunURy,int tunLLy,int redURx,int redLLx,int redURy,int redLLy,int islURx,int islLLx,int islURy,int islLLy,int sc) {
		this.SC = sc;
		this.targetColour = targetColour;
		this.szrLLx = szrLLx;
		this.szrLLy = szrLLy;
		this.szrURx = szrURx;
		this.szrURy = szrURy;
		this.redLLx = redLLx;
		this.redLLy = redLLy;
		this.redURx = redURx;
		this.redURy = redURy;
		this.tunLLx = tunLLx;
		this.tunLLy = tunLLy;
		this.tunURx = tunURx;
		this.tunURy = tunURy;
		this.islLLx = islLLx;
		this.islLLy = islLLy;
		this.islURx = islURx;
		this.islURy = islURy;
		this.upsearch = (islURy - szrURy > 0);
		this.downsearch = (szrLLy > islLLy);
		this.leftsearch = (szrLLx > islLLx);
		this.rightsearch = (szrURx < islURx);
		searchMap = new int[2][((szrURx-szrLLx) / 2 + ((szrURx - szrLLx) % 2)) * ((szrURy-szrLLy) / 2 + ((szrURy - szrLLy) % 2))];
	}
	private void positionCorrection() {
		if (this.SC == 0) {
			odometer.setXYT(tileSize, tileSize, 0);
		}else if (this.SC == 1) {
			odometer.setXYT(14*tileSize, tileSize, 270);
		}else if (this.SC == 2) {
			odometer.setXYT(14*tileSize, 8*tileSize, 180);
		}else {
			odometer.setXYT(tileSize, 8*tileSize, 90);
		}
	}
	/**
	 * the car will navigate to the point (x*tileSize,y*tileSize)
	 * @param x x coordinate 
	 * @param y y coordinate
	 * @throws OdometerExceptions
	 */
	  public void travelTo(double x,double y ) throws OdometerExceptions{
		System.out.println("aim at "+ x + " " + y);
		int dx1 = (int) (odometer.getXYT()[0]-x);
		int dy1 = (int) (odometer.getXYT()[1]-y);
		double distance1 = Math.hypot(dx1, dy1);
		if(distance1 < 0.5) {
		    System.out.println(" here");
		    return;
		} 
		jobx = x;
		joby = y;
		double theta, dX, dY, distance;
	    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
	      motor.stop();
	      motor.setAcceleration(3000);
	    }

	    // Sleep for 2 seconds
	    try {
	    	
	      Thread.sleep(2000);
	    } catch (InterruptedException e) {
	      // There is nothing to be done here
	    }
	    System.out.println("aim at "+ x + " " + y);
	  //  Sound.beepSequenceUp();
	    //TO DO SEE POSITION TO NEXT POSITION 
	   // angle = odometer.getXYT()[2];
	    
	    dX = (int)(x - odometer.getXYT()[0]);
	    dY = (int)(y - odometer.getXYT()[1]);
	    double oldx = odometer.getXYT()[0];
	    double oldy = odometer.getXYT()[1];
	    distance = Math.sqrt(dX*dX + dY*dY);
	    System.out.println(distance);
	    theta = Math.toDegrees(Math.atan(dX/dY));
	    
	    if(dX < 0 && dY < 0) {
	    	theta += 180;
	    }
	    if(dX > 0 && dY < 0) {
	    	theta += 180;
	    }
	    if(dX < 0 && dY > 0) {
	    	theta += 360;
	    }
	    turnTo(theta);
	    double correctAngle = gyroFetch();
	    double speed = 0;
	    this.travelling = true;
	   if(search) {
		   leftMotor.setSpeed((int)(FORWARD_SPEED*0.8));
		   rightMotor.setSpeed((int)(FORWARD_SPEED*0.8));
	    speed = FORWARD_SPEED*0.8;
	   }else {
		   leftMotor.setSpeed(FORWARD_SPEED);
		   rightMotor.setSpeed(FORWARD_SPEED);
		   speed = FORWARD_SPEED;
	   
	   }
		   if(distance < 14) {
		   leftMotor.setSpeed((int) (FORWARD_SPEED * 0.7));
		   rightMotor.setSpeed((int) (FORWARD_SPEED * 0.7)); 
		   leftMotor.rotate(convertDistance(leftRadius, distance), true);
		   rightMotor.rotate(convertDistance(rightRadius, distance), false);
		   return;
		   
	   }
	   leftMotor.forward();
	   rightMotor.forward();
	   // leftMotor.rotate(convertDistance(leftRadius, distance), true);
	   // rightMotor.rotate(convertDistance(rightRadius, distance), true);
	   
	    while(true) {
	    	//if(obstacle) {
	    	directionCorrection(correctAngle,(int)speed);
	    	int dx = (int)(odometer.getXYT()[0]-jobx);
	    	int dy = (int)(odometer.getXYT()[1]-joby);
	    	distance1 = Math.hypot(dx, dy);
	    	System.out.println(distance1);
	    	if(distance1 < 10) {
	    	   carStop();
	    	   travelTo(x,y);
	    	   return;
	    	}
	    	else if(distance1 < 1) {
	    		break;
	    	}
	    	if (losePath(oldx,oldy)) {
	    		travelTo(jobx,joby);
	    	}
	    	if(usFetch() < 13 && (distance1 > 20 && !search)) {
	    		avoid(usFetch());
	    		travelTo(x,y);
	    	}
	    }
	    carStop();
	    travelling = false;
	   
	  }
	  /**
	   * this method is used to call the color classification and detect the color of the can and avoid the can during the 
	   * travelling state 
	   * @param distance the detected distance by us sensor
	   * @throws OdometerExceptions
	   */
	  public  void avoid(double distance) throws OdometerExceptions{
	    leftMotor.stop(true);
		rightMotor.stop();
		
		travelling = false;
		  if (predictPath() == 1){
		    RightAvoid();
		  }
		  else if (predictPath() == 0){
		    leftAvoid();
		  }
		  if (found) {
				return;
			}
		
		this.obstacle = false;
		   
	  }
	  
	  

	  /**
	   * turning to a certain degree,call by travelTo()
	   * @param theta target degree
	   */
	  public void turnTo (double theta) {
		  double angle,smallestAngle;
		 // angle = odometer.getXYT()[2];
		  angle = gyroFetch();
		    if((theta - angle) > 180) {
		    	smallestAngle = theta - angle - 360;
		    	//turnLeft(360-(theta-angle));
		    }
		    else if((theta - angle) < -180) {
		    	smallestAngle = theta - angle + 360;
		    	//turnRight(360 -(angle-theta));
		    }
		    else {
		    	smallestAngle = theta - angle;
		    	//turnRight(smallestAngle);
		    }
		        // turn 90 degrees clockwise
		        leftMotor.setSpeed(ROTATE_SPEED);
		        rightMotor.setSpeed(ROTATE_SPEED);

		        leftMotor.rotate(convertAngle(leftRadius, track, smallestAngle), true);
		        rightMotor.rotate(-convertAngle(rightRadius, track, smallestAngle), false);
		        angle = gyroFetch();
		        double error = 0;
		        if((theta - angle) > 180) {
			    	error = theta - angle - 360;
			    	//turnLeft(360-(theta-angle));
			    }
			    else if((theta - angle) < -180) {
			    	error = theta - angle + 360;
			    	//turnRight(360 -(angle-theta));
			    }
			    else {
			    	error = theta - angle;
			    	//turnRight(smallestAngle);
			    }
		        while (Math.abs(error) > 1.2) {
		        	leftMotor.setSpeed(ROTATE_SPEED);
			        rightMotor.setSpeed(ROTATE_SPEED);

			        leftMotor.rotate(convertAngle(leftRadius, track, error), true);
			        rightMotor.rotate(-convertAngle(rightRadius, track, error), false);
			        carStop();
			        angle = gyroFetch();
			        if((theta - angle) > 180) {
				    	error = theta - angle - 360;
				    	//turnLeft(360-(theta-angle));
				    }
				    else if((theta - angle) < -180) {
				    	error = theta - angle + 360;
				    	//turnRight(360 -(angle-theta));
				    }
				    else {
				    	error = theta - angle;
				    	//turnRight(smallestAngle);
				    }
		        }
	  }
	   boolean  isNavigating(){
		  return this.travelling;
	  }
	   /**
	    * this method is used to determine the turning direction depends on the position of the car,call by avoid function
	    * @return the turning direction 1 means right ,0 means left
	    */

	  public int predictPath() {

			double currx = odometer.getXYT()[0];
			double curry = odometer.getXYT()[1];
			double currTheta = odometer.getXYT()[2];
			
			if (currTheta > 340 || currTheta <= 20) {//going up
				if (currx < (szrLLx+0.5)*tileSize) {
					return 1;
					//wallFollowRight();            // 1 represents right dodge and 0 represents left dodge
				} 
				else if (currx > (szrURx-0.5)*tileSize){
					return 0;
					//wallFollowLeft();
				}
			} 
			else if(currTheta >= 70 && currTheta < 110){//going right
				if (curry < (szrLLy+0.5)*tileSize) {
					return 0;
					//wallFollowLeft();
				} 
				else if (curry > (szrURy-0.5)*tileSize) {
					return 1;
					//wallFollowRight();
				}
			}
			else if(currTheta > 160 && currTheta < 200){//going down
				if (currx < (szrLLx+0.5)*tileSize) {
					return 0;
					//wallFollowLeft();
				} 
				else if (currx > (szrURx-0.5)*tileSize) {
					return 1;
					//wallFollowRight();
				}
			}
			else if(currTheta > 250 && currTheta < 290){//going left
				if (curry <= (szrLLy+0.5)*tileSize ) {
					return 1;
					//wallFollowRight();
				} 
				else if (curry > (szrURy-0.5)*tileSize) {
					System.out.println("8");
					return 0;
					//wallFollowLeft();
				}
			}
				//wallFollowRight();
				//return 1;
				return 0;
	  
		
		}
	  /**
	   * the method help the car turn right,and determine whether need to test the color again,called by avoid function
	   *
	   */
	  public void RightAvoid() throws OdometerExceptions {
		  carStop();
		  turnRight(45);
		  carStop();
		  goStraightLine(25,FORWARD_SPEED/2);
		  carStop();
	      turnLeft(90);
	      carStop();
	      goStraightLine(25,FORWARD_SPEED/2);
	      carStop();
	      
		  
		}
	  /**
	   * the method help the car turn right,and determine whether need to test the color again,called by avoid function
	   * 
	   */
		
		public void leftAvoid () throws OdometerExceptions {
			carStop();
			turnLeft(45);
		    carStop();
		    gyroFetch();
			goStraightLine(25,FORWARD_SPEED/2);
		      turnRight(90);
		      carStop();
		      gyroFetch();
		      goStraightLine(25,FORWARD_SPEED/2);
		      carStop();
		}
	/**
	 * calculate the angle that wheel need to rotate to make car move forward a certain distance
	 * @param radius
	 * @param distance
	 * @return the angle that wheel need to rotate
	 */

	  private static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	  }
	/**
	 * calculate the angle that wheel need to rotate to make car turning a certain angle
	 * @param radius
	 * @param width
	 * @param angle
	 * @return
	 */
	 
	  private static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	  }
	 
	  /**
	   * get the distance value from the ultrasonic sensor
	   * @return the distance value read by ultrasonic sensor
	   */
	  private float usFetch() {
		  us.fetchSample(usValues, 0);
		  if ((usValues[0]*100)>200) {
			  return 200;
		  }else {
			  return usValues[0]*100;
		  }
	  }
	  /**
	   * get the angular value from the  gyrosensor
	   * @return the angular value measure by gyrosensor 
	   */
	  private double gyroFetch() {
		  angleCorrection();
		  return odometer.getXYT()[2];
	  }
	  /**
	   * use the gyro sensor to correct the angle value in the odometer
	   */
	  private void angleCorrection() {
		  gyAngles.fetchSample(angles, 0);
		  double angle = (0 + 360 -this.SC*90)%360 + angles[0];
		  if (angle >= 0) {
			  odometer.setXYT(odometer.getXYT()[0], odometer.getXYT()[1],angle);
		  }else {
			  odometer.setXYT(odometer.getXYT()[0], odometer.getXYT()[1], 360+angle);
		  }
	  }
	  /**
	   * use the sensor the check whether the car go a straight line,and modified the direction if there's a deviation
	   * @param degree
	   * @param speed
	   */
	  private void directionCorrection(double degree,int speed) {
		    double degree1 = gyroFetch();
			if(degree1 - degree  >= 2) {
				leftMotor.setSpeed(speed);
		        rightMotor.setSpeed((speed+(int)(degree1 - degree)*6));
		        leftMotor.forward();
	  		rightMotor.forward();
		this.gyAngles.fetchSample(angles, 0);
	  }else if(degree1 - degree <=-2) {
	  		leftMotor.setSpeed((float)(speed+(int)(degree - degree1)*6));
		        rightMotor.setSpeed(speed);
		        leftMotor.forward();
	  		rightMotor.forward();
		this.gyAngles.fetchSample(angles, 0);
	  	}
	  }
	  /**
	   * let car move forward a certain street and use the sensor to do the direction correction
	   * @param distance
	   * @param speed
	   */
	  private void goStraightLine(double distance,int speed) {
		  carStop();
		  for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		      motor.stop();
		      motor.setAcceleration(3000);
		    }

		    // Sleep for 2 seconds
		    try {
		    	
		      Thread.sleep(2000);
		    } catch (InterruptedException e) {
		      // There is nothing to be done here
		    }
		  double angle = gyroFetch();
		  double p1 = (leftMotor.getTachoCount()+rightMotor.getTachoCount())*leftRadius*Math.PI/360;
		  double p2 = (leftMotor.getTachoCount()+rightMotor.getTachoCount())*leftRadius*Math.PI/360;
		 double distance1 = p2 - p1;
		int speed1;
		while(distance1 < distance) {
			speed1 =(int) (speed - 25/((distance - distance1)+1));
			leftMotor.setSpeed(speed);
			rightMotor.setSpeed(speed);
			leftMotor.forward();
			rightMotor.forward();
			for(int i = 0;i<3;i++);
			directionCorrection(angle,speed1);
			p2 = (leftMotor.getTachoCount()+rightMotor.getTachoCount())*leftRadius*Math.PI/360;
			distance1 = p2 - p1;
		}
		  carStop();
		  try {
		    	
		      Thread.sleep(500);
		    } catch (InterruptedException e) {
		      // There is nothing to be done here
		  }
	  }
	/**
	 * make the car turning right
	 * @param degree
	 */
	  private void turnRight(double degree) {
		  if(degree <= 1) {
			  return;
		  }
		  double minAngle = 0;
		    int speed;
			 double angle = gyroFetch();
			 double angle1 = gyroFetch();
			 while((Math.abs(angle1 - angle - degree)>=1) && (Math.abs((angle - angle1) - (360-degree))>=1)){
				 minAngle = Math.min((Math.abs(angle1 - angle - degree)), Math.abs((angle - angle1) - (360-degree)));
				 speed = (int)(80 - 25/(minAngle+1));
				 leftMotor.setSpeed(speed);
			     rightMotor.setSpeed(speed);
			     leftMotor.forward();
			     rightMotor.backward();
			     angle1 = gyroFetch();
			 }
				leftMotor.stop(true);
				rightMotor.stop();
			}
	  /**
	   * make the car turning left
	   * @param degree
	   */
			private void turnLeft(double degree) {
				if (degree <= 1) {
					return;
				}
				int speed;
				double minAngle = 0;
				 double angle = gyroFetch();
				 double angle1 = gyroFetch();
				 while((Math.abs(angle - angle1 - degree)>=1) && (Math.abs((angle1 - angle) - (360-degree))>=1)){
					 minAngle = Math.min((Math.abs(angle - angle1 - degree)), Math.abs((angle1 - angle) - (360-degree)));
					 speed = (int)(80 - 25/(minAngle+1));
					 leftMotor.setSpeed(speed);
				     rightMotor.setSpeed(speed);
				     leftMotor.backward();
				     rightMotor.forward();
				     angle1 = gyroFetch();
				 }
					leftMotor.stop(true);
					rightMotor.stop();
				}
			/**
			 * check if the car is in the last point of the search process
			 * @return
			 */
	 
	 /**
	  * check if car is far from the destination after travelTo function if the answer is true ,the travelTo function will be call again
	  * @param x
	  * @param y
	  * @return
	  */
	 private boolean losePath(double x,double y) {
		 double curx = odometer.getXYT()[0];
		 double cury = odometer.getXYT()[1];
		// System.out.println(curx+" "+cury);
		 boolean x1=false,y1=false;
		 if (jobx < x && curx < x) {
			 x1 = (jobx - curx >= 15);
		 }else if (jobx > x && curx > x) {
			 x1 = ((curx - jobx) >= 15);
		 }else if ((jobx > x && curx < x)||(jobx < x && curx > x)) {
			 x1 = (Math.abs(curx - x) >= 15);
		 }
		 if (joby < y && cury < y) {
			 y1 = (joby - cury >= 15);
		 }else if (joby > y && cury > y) {
			 y1 = ((cury - joby) >= 15);
		 }else if ((joby > y && cury < y)||(joby < y && cury > y)) {
			 y1 = Math.abs(cury - y) >= 10;
		 }
		 if (x1||y1) {
			 System.out.println(x1);
			 System.out.println(y1);
			 System.out.println("lose");
		 }
		 return x1||y1;
	 }
	 /**
	  * make the car stop
	  */
	private void carStop() {
		    leftMotor.stop(true);
		    rightMotor.stop();
	}
	private boolean farAway(int x,int y) {
		int curx = (int)odometer.getXYT()[0];
		int cury = (int)odometer.getXYT()[2];
		return Math.abs(x*30 -curx) + Math.abs(y*30 - cury) > 300;
	}
	private int getCorner() {
		if (this.SC == 0) {
			if (tunLLx == 0) {
				return 1;
			}else if(tunLLy == 0) {
				return 3;
			}else {
				return 0;
			}
		}else if (this.SC == 1) {
			if(tunURx == 15) {
				return 0;
			}else if(tunLLy == 0) {
				return 2;
			}else {
				return 1;
			}
		}else if (this.SC == 2) {
			if (tunURx == 15) {
				return 3;
			}else if(tunURy == 9){
				return 1;
			}else {
				return 2;
			}
		}else {
			if (tunURy == 9) {
				return 0;
			}else if (tunLLx == 0) {
				return 2;
			}else {
				return 3;
			}
		}
		
	}
	 /**
	  * the car will travel to near the tunnel ,do the localization and go through the tunnel
	  * @throws OdometerExceptions
	  */
	public void travelTunnel() throws OdometerExceptions {
		int corner = getCorner();
		double angle = (0 + 360 -corner*90)%360;
		if ((tunLLx + 1 <= redURx)||(tunURy-tunLLy == 2)) {
			if(this.SC == 0) {
				if(this.SC == corner) {
					boolean far = farAway(tunLLx,tunLLy);
					travelTo((tunLLx - 0.3)*tileSize,(tunLLy-1.3)*tileSize);
					turnTo(angle);
					ll.localize(tunLLx, tunLLy - 1, corner, far);
					travelTo((tunLLx + 0.5)*tileSize,(tunLLy - 1)*tileSize);
					turnTo(0);
				}else {
					boolean far = farAway(tunURx,tunLLy);
					travelTo((tunURx + 0.3)*tileSize,(tunLLy-1.3)*tileSize);
					turnTo(angle);
					ll.localize(tunURx, tunLLy - 1, corner, far);
					travelTo((tunLLx + 0.5)*tileSize,(tunLLy - 1)*tileSize);
					turnTo(0);
				}
			}else if (this.SC == 1) {
				if(this.SC == corner) {
					boolean far = farAway(tunURx,tunLLy);
					travelTo((tunURx + 0.3)*tileSize,(tunLLy-1.3)*tileSize);
					turnTo(angle);
					ll.localize(tunURx, tunLLy - 1, corner, far);
					travelTo((tunLLx + 0.5)*tileSize,(tunLLy - 1)*tileSize);
					turnTo(0);
				}else {
					boolean far = farAway(tunLLx,tunLLy);
					travelTo((tunLLx - 0.3)*tileSize,(tunLLy-1.3)*tileSize);
					turnTo(angle);
					ll.localize(tunLLx, tunLLy - 1, corner, far);
					travelTo((tunLLx + 0.5)*tileSize,(tunLLy - 1)*tileSize);
					turnTo(0);
				}
			}else if (this.SC == 2) {
				if(this.SC == corner) {
					boolean far = farAway(tunURx,tunURy);
					travelTo((tunURx + 0.3)*tileSize,(tunURy+1.3)*tileSize);
					turnTo(angle);
					ll.localize(tunURx, tunURy + 1, corner, far);
					travelTo((tunURx - 0.5)*tileSize,(tunURy + 1)*tileSize);
					turnTo(180);
				}else {
					boolean far = farAway(tunLLx,tunURy);
					travelTo((tunLLx - 0.3)*tileSize,(tunURy+1.3)*tileSize);
					turnTo(angle);
					ll.localize(tunLLx, tunURy + 1, corner, far);
					travelTo((tunURx - 0.5)*tileSize,(tunURy + 1)*tileSize);
					turnTo(180);
				}
			}else {
				if(this.SC == corner) {
					boolean far = farAway(tunLLx,tunURy);
					travelTo((tunLLx - 0.3)*tileSize,(tunURy+1.3)*tileSize);
					turnTo(angle);
					ll.localize(tunLLx, tunURy + 1, corner, far);
					travelTo((tunURx - 0.5)*tileSize,(tunURy + 1)*tileSize);
					turnTo(180);
				}else {
					boolean far = farAway(tunURx,tunURy);
					travelTo((tunURx + 0.3)*tileSize,(tunURy+1.3)*tileSize);
					turnTo(angle);
					ll.localize(tunURx, tunURy + 1, corner, far);
					travelTo((tunURx - 0.5)*tileSize,(tunURy + 1)*tileSize);
					turnTo(180);
				}
			}
		}else {
			if(this.SC == 0) {
				if(this.SC == corner) {
					boolean far = farAway(tunLLx,tunLLy);
					travelTo((tunLLx - 1.3)*tileSize,(tunLLy-0.3)*tileSize);
					turnTo(angle);
					ll.localize(tunLLx-1, tunLLy, corner, far);
					travelTo((tunLLx - 1)*tileSize,(tunLLy + 0.5)*tileSize);
					turnTo(90);
				}else {
					boolean far = farAway(tunLLx,tunURy);
					travelTo((tunLLx - 1.3)*tileSize,(tunURy+0.3)*tileSize);
					turnTo(angle);
					ll.localize(tunLLx-1, tunURy, corner, far);
					travelTo((tunLLx - 1)*tileSize,(tunLLy + 0.5)*tileSize);
					turnTo(90);
				}
			}else if (this.SC == 1) {
				if(this.SC == corner) {
					boolean far = farAway(tunURx,tunLLy);
					travelTo((tunURx + 1.3)*tileSize,(tunLLy-0.3)*tileSize);
					turnTo(angle);
					ll.localize(tunURx+1, tunLLy, corner, far);
					travelTo((tunURx + 1)*tileSize,(tunLLy + 0.5)*tileSize);
					turnTo(270);
				}else {
					boolean far = farAway(tunURx,tunURy);
					travelTo((tunURx + 1.3)*tileSize,(tunURy+0.3)*tileSize);
					turnTo(angle);
					ll.localize(tunURx+1, tunURy, corner, far);
					travelTo((tunURx + 1)*tileSize,(tunLLy + 0.5)*tileSize);
					turnTo(270);
				}
			}else if (this.SC == 2) {
				if(this.SC == corner) {
					boolean far = farAway(tunURx,tunURy);
					travelTo((tunURx + 1.3)*tileSize,(tunURy+0.3)*tileSize);
					turnTo(angle);
					ll.localize(tunURx+1, tunURy, corner, far);
					travelTo((tunURx + 1)*tileSize,(tunLLy + 0.5)*tileSize);
					turnTo(270);
				}else {
					boolean far = farAway(tunURx,tunLLy);
					travelTo((tunURx + 1.3)*tileSize,(tunLLy-0.3)*tileSize);
					turnTo(angle);
					ll.localize(tunURx+1, tunLLy, corner, far);
					travelTo((tunURx + 1)*tileSize,(tunLLy + 0.5)*tileSize);
					turnTo(270);
				}
			}else {
				if(this.SC == corner) {
					boolean far = farAway(tunLLx,tunURy);
					travelTo((tunLLx - 1.3)*tileSize,(tunURy+0.3)*tileSize);
					turnTo(angle);
					ll.localize(tunLLx-1, tunURy, corner, far);
					travelTo((tunLLx - 1)*tileSize,(tunLLy + 0.5)*tileSize);
					turnTo(90);
				}else {
					boolean far = farAway(tunLLx,tunLLy);
					travelTo((tunLLx - 1.3)*tileSize,(tunLLy-0.3)*tileSize);
					turnTo(angle);
					ll.localize(tunLLx-1, tunLLy, corner, far);
					travelTo((tunLLx - 1)*tileSize,(tunLLy + 0.5)*tileSize);
					turnTo(90);
				}
			}
		}
		goStraightLine(4*tileSize,200);
	 }
	 /**
	  * construct an array which contain a list of points which car will visit and do the research
	  */
	 public void constructMap() {
		 int i = 0;
		 for (int j = 1;j<= (szrURx - szrLLx) - 1;j += 2) {
			 for(int k = 1;k<=(szrURy - szrLLy) - 1;k += 2) {
				 searchMap[0][i]= szrLLx + j;
				 searchMap[1][i]= szrLLy + k;
				 System.out.println(searchMap[0][i]+" "+searchMap[1][i]);
				 i = i + 1;
			 }
			 if ((szrURy - szrLLy) % 2 == 1) {
				 searchMap[0][i] = szrLLx + j;
				 searchMap[1][i] = szrURy - 1;
				 System.out.println(searchMap[0][i]+" "+searchMap[1][i]);
				 i = i + 1;
			 }
			 
		 }
		 if ((szrURx - szrLLx) % 2 == 1) {
			 for(int k = 1;k<=(szrURy - szrLLy) - 1;k += 2) {
				 searchMap[0][i]= szrURx - 1;
				 searchMap[1][i]= szrLLy + k;
				 System.out.println(searchMap[0][i]+" "+searchMap[1][i]);
				 i = i + 1;
			 }
			 if ((szrURy - szrLLy) % 2 == 1) {
				 searchMap[0][i] = szrURx - 1;
				 searchMap[1][i] = szrURy - 1;
				 System.out.println(searchMap[0][i]+" "+searchMap[1][i]);
				 i = i + 1;
			 }
		 }
	 }
	 /**
	  * when found a can when rotation, the car will stop and move to in front of the can and do the research
	  * @param distance the detected distance by us sensor, the distance of can
	  * @throws OdometerExceptions
	  */
	 public void travelSearch(double distance) throws OdometerExceptions {
		 carStop();
		 //leftMotor.rotate(convertDistance(leftRadius, distance-3), true);
		 //rightMotor.rotate(convertDistance(rightRadius, distance-3), false);
		 if (distance > 20) {
		 goStraightLine(distance - 5,70);
		 }else {
			 normalGoForward(distance - 5);	 
		 }
		 carStop();
		 int color = colorDetector.findColor();
		 if (color == this.targetColour) {
			 makeNoise(10);
			 travelTo(szrURx * tileSize,szrURy * tileSize);
			 makeNoise(5);
			 grab();
			 boolean heavy = weightMeasure();
			 if (heavy) {
				 found = true;
				 backToOwnArea();
			 }else {
				 travelTo(jobx,joby);
				 return;
			 }
		 }else {
			 normalGoBackward(distance - 5);
			 carStop();
			 searchHelper = new float[size];
			 helperCounter = 0;
		 }
		 
	 }
	 /**
	  * after the car reach the visit point in the search map,
	  * car will rotate about 360 to see if there is any can near the car
	  * @throws OdometerExceptions
	  */
	 public void turnSearch() throws OdometerExceptions {
		 this.search = true;
		 if (usFetch() < 5) {
			 normalGoForward(8);
			 carStop();
			 normalGoBackward(8);
			 carStop();
		 }
		 double preTarget = -1;
		 double angle = gyroFetch();
		 double curAngle = gyroFetch();
		 boolean start = false;
		 double difAngle = 0;
		 while(!start || difAngle < 340) {
			 leftMotor.setSpeed(80);
			  rightMotor.setSpeed(80);
			  leftMotor.forward();
			  rightMotor.backward();
			  curAngle = gyroFetch();
			  helper();
			  if (detected()) {				  
				  carStop();
				  if (preTarget == -1 || !compareAngle(preTarget,curAngle)){
					 leftMotor.rotate(-convertAngle(leftRadius, track, 25), true);
				     rightMotor.rotate(convertAngle(rightRadius, track, 25), false);
				     if (usFetch()<45){
				     travelSearch(usFetch());
				     }
				     leftMotor.rotate(convertAngle(leftRadius, track, 25), true);
				     rightMotor.rotate(-convertAngle(rightRadius, track, 25 ), false);
				     preTarget = curAngle;
				  }
			  }
			  if (angle > 180 && curAngle < angle) {
				  difAngle = 360 - (angle - curAngle);
			  }else if (angle <= 180 && curAngle >= angle) {
				  difAngle = curAngle -angle;
			  }else if (angle <= 180 && curAngle < angle) {
				  difAngle = 360 - (angle - curAngle);
			  }
			  else {
				  difAngle = curAngle - angle;
			  }
			  System.out.println(difAngle+" degree");
			  if (!start && difAngle > 20) {
				  start = true;
			  }
		 }
	 }
	 private boolean compareAngle(double angle,double curAngle) {
		 double difAngle = -1;
		 if (angle > 180 && curAngle < angle) {
			  difAngle = 360 - (angle - curAngle);
		  }else if (angle <= 180 && curAngle >= angle) {
			  difAngle = curAngle -angle;
		  }else if (angle <= 180 && curAngle < angle) {
			  difAngle = 360 - (angle - curAngle);
		  }else {
			  difAngle = curAngle - angle;
		  }
		 System.out.println(difAngle <= 40);
		 return difAngle <= 40;
	 }
	 public void makeNoise(int num) {
		 for(int i = 0;i< num;i++) {
			Sound.beep(); 
		 }
	 }
	 /**
	  * the program will call the constructMap() method to generate the searchMap,
	  * the car will visit the point on the map one by one 
	  * @throws OdometerExceptions
	  */
	 public void search() throws OdometerExceptions {
		 constructMap();
		 int dX =  ((szrURx-szrLLx) / 2) + ((szrURx-szrLLx) % 2);
		 int dY =  ((szrURy-szrLLy) / 2) + ((szrURy-szrLLy) % 2);
		 for (int i = 1;i<=dX;i++) {
			 if (i % 2 == 1) {
				 for (int j = (i-1)*dY + 1;j<=i*dY;j++) {
					 //System.out.println(searchMap[0][j-1] + " " + searchMap[1][j-1]);
					 travelTo(searchMap[0][j-1]*tileSize,searchMap[1][j-1]*tileSize);
					 turnSearch();
				 }
			 }else {
				 for (int j = i*dY;j>=(i-1)*dY+1;j--) {
					// System.out.println(searchMap[0][j-1] + " " + searchMap[1][j-1]);
					 travelTo(searchMap[0][j-1]*tileSize,searchMap[1][j-1]*tileSize);
					 turnSearch();
				 }
			 }
		 }
	 }
	 /**
	  * travel to the left-lower point of search area
	  * @throws OdometerExceptions
	  */
	 public void travelToSearchArea() throws OdometerExceptions {
		 boolean far = farAway(szrLLx,szrLLy);
		 if(this.leftsearch && this.downsearch) {
			 travelTo((szrLLx-0.5)*tileSize,(szrLLy-0.5)*tileSize);
			 turnTo(0);
			 ll.localize(szrLLx, szrLLy,0,far);
		 }else if (this.leftsearch && !this.downsearch) {
			 travelTo((szrLLx-0.5)*tileSize,(szrLLy+0.5)*tileSize);
			 turnTo(90);
			 ll.localize(szrLLx, szrLLy,3,far);
		 }else if (!this.leftsearch && this.downsearch) {
			 travelTo((szrLLx+0.5)*tileSize,(szrLLy-0.5)*tileSize);
			 turnTo(270);
			 ll.localize(szrLLx, szrLLy,3,far);
		 }else {
			 travelTo((szrLLx+0.5)*tileSize,(szrLLy+0.5)*tileSize);
			 turnTo(180);
			 ll.localize(szrLLx, szrLLy,3,far);
		 }
		 travelTo(szrLLx*tileSize,szrLLy*tileSize);
		 makeNoise(5);
	 }
	 private void helper() {
		 float value = usFetch();
		 if (helperCounter > size - 1) {
			for (int i = 0;i <= size - 2;i++) {
				searchHelper[i]=searchHelper[i+1];
			}
			searchHelper[size - 1] = value;
		 }else {
			 searchHelper[helperCounter] = value;
		 }
		 helperCounter++;
	 }
	 private boolean detected() {
		 return searchHelper[size/2]< searchHelper[size - 1] && searchHelper[size / 2]< searchHelper[0] && searchHelper[size/2] < 45;
	 }
	 /**
	  * this function is the primary function in the navigation class,it calls other function to reach the destination
	  * @throws OdometerExceptions
	  */
	 public void work() throws OdometerExceptions {
		 gySensor.reset();
		 try {
		    	
		      Thread.sleep(1000);
		    } catch (InterruptedException e) {
		      // There is nothing to be done here
		    }
	     positionCorrection();
	     System.out.println("here");
		 travelTunnel();
		 travelToSearchArea();
		 search();
		 makeNoise(5);
	 }
	 private void normalGoForward(double distance) {
		 for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		      motor.stop();
		      motor.setAcceleration(3000);
		    }

		    // Sleep for 2 seconds
		    try {
		    	
		      Thread.sleep(2000);
		    } catch (InterruptedException e) {
		      // There is nothing to be done here
		    }
		    leftMotor.rotate(convertDistance(leftRadius, distance), true);
			rightMotor.rotate(convertDistance(rightRadius, distance), false);
			  try {
			    	
			      Thread.sleep(500);
			    } catch (InterruptedException e) {
			      // There is nothing to be done here
			    }
	 }
	 private void normalGoBackward(double distance) {
		 for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		      motor.stop();
		      motor.setAcceleration(3000);
		    }

		    // Sleep for 2 seconds
		    try {
		    	
		      Thread.sleep(2000);
		    } catch (InterruptedException e) {
		      // There is nothing to be done here
		    }
		    leftMotor.rotate(-convertDistance(leftRadius, distance), true);
			rightMotor.rotate(-convertDistance(rightRadius, distance), false);
			  try {
			    	
			      Thread.sleep(500);
			    } catch (InterruptedException e) {
			      // There is nothing to be done here
			    }
	 }
	 private boolean weightMeasure() throws OdometerExceptions {
		 boolean up = false,down = false,left = false,right = false;
		 if (szrLLx-islLLx > 0) {
			 left = true;
		 }
		 if (szrLLy - islLLy > 0) {
			 down = true;
		 }
		 if (islURx - szrURx > 0) {
			 right = true;
		 }
		 if (islURy - szrURy > 0) {
			 down = true;
		 }
		 double curx = odometer.getXYT()[0];
		 double cury = odometer.getXYT()[1];
		 double llx = szrLLx*tileSize;
		 double lly = szrLLy * tileSize;
		 double urx = szrURx*tileSize;
		 double ury = szrURy*tileSize;
		 if (left) {
			if ((cury - lly) <=(ury-cury)) {
				travelTo((szrLLx-0.5)*tileSize,szrLLy*tileSize);
				turnTo(0);
			}else {
				travelTo((szrLLx-0.5)*tileSize,szrURy*tileSize);
				turnTo(180);
			}
		 }else if (down) {
			if ((curx - llx)<=(urx - curx)) {
				travelTo(szrLLx*tileSize,(szrLLy-0.5)*tileSize); 
				turnTo(90);
			}else {
				travelTo(szrURx*tileSize,(szrLLy-0.5)*tileSize);
				turnTo(270);
			}
		 }else if (up) {
			if ((curx - llx)<=(urx - curx)) {
				travelTo(szrLLx*tileSize,(szrURy+0.5)*tileSize); 
				turnTo(90);
			}else {
				travelTo(szrURx*tileSize,(szrURy+0.5)*tileSize);
				turnTo(270);
			} 
		 }else if (right) {
			if ((cury - lly) <=(ury-cury)) {
				travelTo((szrURx+0.5)*tileSize,szrLLy*tileSize);
				turnTo(0);
			}else {
				travelTo((szrURx+0.5)*tileSize,szrURy*tileSize);
				turnTo(180);
			}
		 }
		 return timeMeasure();
	 }
	 private boolean timeMeasure() {
		 while(ll.fetchSample() > COLOR_THRESHOLD_B) {
			 leftMotor.forward();
			 rightMotor.forward();
		 }
		 carStop();
		 long startTime = System.nanoTime();
		 boolean filter = false;
		 while(ll.fetchSample() > COLOR_THRESHOLD_B || !filter) {
			 leftMotor.forward();
			 rightMotor.forward();
			 if (!filter && ll.fetchSample() > COLOR_THRESHOLD_B) {
				 filter = true;
			 }
		 }
		 carStop();
		 long endTime = System.nanoTime();
		 long duration = endTime - startTime;
		 return duration > 000; // need to find the 
	 }
	 private void grab() {
		 
	 }
	 private void backToOwnArea() { 
		 
	 }
}
