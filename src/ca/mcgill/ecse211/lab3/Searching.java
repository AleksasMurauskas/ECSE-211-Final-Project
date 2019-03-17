package ca.mcgill.ecse211.lab3;

public class Searching {
	
	/**
	 * the degree of a full angle is 360 degree
	 * will be used in the search method
	 */
	private static final int FULL_ANGLE = 360;
	
	/**
	 * this is the distance we want the car to have between the can 
	 * when we start our color classification
	 */
	private static final double DETECT_DISTANT = 8;
	
	/**
	 * the distance between the car and a can, -1 means not detected
	 */
	public static double targetDist=-1;
	
	
	
	
	/**
	 * default constructor
	 */
	public Searching() {
		
	}
	
	/**
	 * This method allow us to search for cans in an elegant routine
	 * @param LL: lower left of the map
	 * @param UR: upper right of the map
	 * @param target: the place where we leave our fund can 
	 */
	public static void search(int[] LL, int[] UR, int[]target) {	
		
		
		double[][] map=getMap(LL,UR); 	//get the map we will use 
		int currentDestination = 0; 		//set a counter to mark which destinations we have already looped through

		
		while( currentDestination < map.length ) {
			
			//see if there are cans detected on the way to the current destination
			targetDist = NavigationObstacle.travelSearch(map[currentDestination][0],map[currentDestination][1]);
			//if detected, process the can
			if (targetDist!=-1)	processTarget(targetDist, target, map[currentDestination]);		
			
			//see if there are cans detected around the current destination
			targetDist = NavigationObstacle.turnSearch(FULL_ANGLE);
			//if detected, process the can
			if (targetDist!=-1)	processTarget(targetDist, target,map[currentDestination]);
			
			currentDestination++;	
			
		}
	
	}	
	
	/**
	 * This method allow us to process the can once we find one
	 * @param distance: distance between car and the can
	 * @param target: the place where we leave our fund can 
	 * @param currentDestination: mark the current destination we are at
	 */
	public static void processTarget(double distance, int[] target,double[] currentDestination) {
		
		//travel to a appropriate position near the can
		Navigation.travelForward(distance-DETECT_DISTANT);
		
		//TODO: perform color classification
		
		//grab the can
		Grabbing.grab();
		
		//travel to the target can
		Navigation.travelTo(target[0], target[1]);
		
		//leave the can
		Grabbing.leave();
		
		//reset the target distance to not fund 
		targetDist = -1;
		
		//back to its initial position
		Navigation.travelTo(currentDestination[0], currentDestination[1]);
		
	}

	
	
	
	
	
	
	
	
	
	/**
	 * This method allow us to find an appropriate routine given the upper right & lower left
	 * @param LL
	 * @param UR
	 * @return a list of destinations
	 */
	public static double[][] getMap(int[] LL, int[] UR) {
		
		//TODO: need to implement this method
		double map[][]=null;

		
		return map;	
		
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
}
