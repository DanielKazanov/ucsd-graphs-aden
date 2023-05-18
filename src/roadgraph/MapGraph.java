/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	private int numVertices;
	private int numEdges;
	private HashMap<GeographicPoint,MapNode> vertices;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor
		numVertices = 0;
		numEdges = 0;
		vertices = new HashMap<GeographicPoint, MapNode>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method
		return vertices.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method.
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (ONLY if the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method
		if (vertices.containsKey(location) || location == null) {
			return false;
		}
		
		vertices.put(location, new MapNode());
		numVertices++;
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2. This will perform the required 
	 * error checking, get the MapNode associated with the "from" point and call the
	 * addEdge() method of the MapNode instance.
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method
	    if (vertices.get(from) == null || vertices.get(to) == null) {
	    	throw new IllegalArgumentException("Point(s) have not already been added as nodes to the graph.");
	    }
		
		if (from == null || to == null || roadName == null || roadType == null || length < 0) {
			 throw new IllegalArgumentException("Argument(s) are/is null.");
		}
		
	    vertices.get(from).addEdge(to, roadName, roadType, length);
	    numEdges++;
	}
	
	/** Find the path from start to goal using breadth first search - without MapApp.
	 *  Calls the MapApp version to actually execute the search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search - Called by MapApp
	 *  Use of helper methods is encouraged
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method and any helper methods.
		
		// Hook for visualization.  See writeup. Note that this may actually need to be
		// located in a helper method.
		//nodeSearched.accept(next.getLocation());

		Map<GeographicPoint, GeographicPoint> parentHashMap = new HashMap<>();
	    Set<GeographicPoint> visited = new HashSet<>();
	    visited.add(start);
		Queue<GeographicPoint> myQ = new LinkedList<GeographicPoint>();
	    myQ.add(start);
	    
	    while (!myQ.isEmpty()) {
	    	GeographicPoint currentNode = myQ.poll();
	    	nodeSearched.accept(currentNode);
	    	
	    	if (currentNode.equals(goal)) {
	    		return path(start, goal, parentHashMap);
	    	}
	    	
	        for (GeographicPoint neighborPoint : vertices.get(currentNode).getNeighborPoints()) {
	            if (!visited.contains(neighborPoint)) {
	                parentHashMap.put(neighborPoint, currentNode);
	                visited.add(neighborPoint);
	            	myQ.add(neighborPoint);
	            }
	        }
	    }
		
		return null;
	}
	
	/**
	 * Creates a path from start to goal.
	 * 
	 * @param start the start
	 * @param goal the goal
	 * @param parentHashMap the parent hash map
	 * @return the path
	 */
	public List<GeographicPoint> path(GeographicPoint start, GeographicPoint goal, Map<GeographicPoint, GeographicPoint> parentHashMap) {
	    List<GeographicPoint> path = new ArrayList<GeographicPoint>();
	    GeographicPoint currentNode = goal;
	    
	    while (currentNode != null) {
	        path.add(0, currentNode);
	        
	        if (currentNode.equals(start)) {
	            break;
	        }
	        
	        currentNode = parentHashMap.get(currentNode);
	    }
	    
	    return path;
	}
	
	// DO NOT CODE ANYTHING BELOW THIS LINE UNTIL PART2

	/** Find the path from start to goal using Dijkstra's algorithm - without MapApp.
	 *  Calls the MapApp version to actually execute the search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm - Called by MapApp
	 *  Use of helper methods is encouraged
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method with Part 2

		// Hook for visualization.  See writeup. Note that this may actually need to be
		// located in a helper method.
		//nodeSearched.accept(next.getLocation());
		
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
	    Set<GeographicPoint> visited = new HashSet<>();
	    HashMap<GeographicPoint, Double> distToNode = new HashMap<>();
	    Queue<QueueElement> queue = new PriorityQueue<>();
	    
	    for (GeographicPoint n : vertices.keySet()) {
	    	distToNode.put(n, Double.POSITIVE_INFINITY);
	    }
	    
	    distToNode.put(start, 0.0);
	    queue.add(new QueueElement(start, 0.0));
	    
	    while (!queue.isEmpty()) {
	    	QueueElement element = queue.poll();
	    	GeographicPoint curr = element.getPoint();
	    	
	    	if (!visited.contains(curr)) {
	    		visited.add(curr);
	    		nodeSearched.accept(curr);
	    		
    		    if (curr.equals(goal)) {
    		    	return path(start, goal, parentMap);
                }
    		    
    		    enqueue(curr, parentMap, visited, distToNode, queue);
	    	}
	    }
		
		return null;
	}
	
	/**
	 * Completes the enqueue for Dijkstra algorithm.
	 * 
	 * @param curr the current node
	 * @param parentMap the parent map
	 * @param visited the visited nodes
	 * @param distToNode the distance to node
	 * @param queue the queue
	 */
	public void enqueue(GeographicPoint curr, HashMap<GeographicPoint, GeographicPoint> parentMap, Set<GeographicPoint> visited, HashMap<GeographicPoint, Double> distToNode, Queue<QueueElement> queue) {
		if (vertices.get(curr).getNeighborPoints().size() == 0) {
			return;
		}
		
		for (GeographicPoint n : vertices.get(curr).getNeighborPoints()) {
             if (!visited.contains(n)) {
                 double distToNext = distToNode.get(curr) + vertices.get(curr).getEdge(n).getRoadLength();
                 
                 if (distToNext < distToNode.get(n)) {
                     parentMap.put(n, curr);
                     queue.add(new QueueElement(n, distToNext));
                     distToNode.put(n, distToNext);
                 }
             }
         }
	}

	/** Find the path from start to goal using A-Star search - without MapApp.
	 *  Calls the MapApp version to actually execute the search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search - Called by MapApp
	 *  Use of helper methods is encouraged
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method with Part 2
		
		// Hook for visualization.  See writeup. Note that this may actually need to be
		// located in a helper method.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should visit 13 nodes and AStar should visit 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should visit 37 nodes and AStar should visit 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
	}
	
}
