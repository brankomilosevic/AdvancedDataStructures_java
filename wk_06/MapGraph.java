/**
 * @author UCSD MOOC development team and Branko Milosevic
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between streets
 *
 */
package roadgraph;

import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;

import geography.GeographicPoint;
import util.GraphLoader;

public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	
	private HashMap<GeographicPoint, MapNode> mPointNodeMap;
	private HashSet<MapEdge> mEdges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		mPointNodeMap = new HashMap<GeographicPoint, MapNode>();
		mEdges = new HashSet<MapEdge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return mPointNodeMap.values().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		return mPointNodeMap.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return mEdges.size();
	}

	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		
		if (location == null) return false;
		
		MapNode n = mPointNodeMap.get(location);
		if (n != null) return false;
		
		n = new MapNode(location);
		mPointNodeMap.put(location, n);
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
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

		//TODO: Implement this method in WEEK 3
		
		// check validity of arguments
		if (from == null || to == null) throw new IllegalArgumentException("Geographic point not valid!!!");
		if (length < 0) throw new IllegalArgumentException("Street length have to be positive number!!!");
		if (roadName == null  || roadType == null) throw new IllegalArgumentException("... where the streets have no name ...");
		
		MapNode n1 = mPointNodeMap.get(from);
		MapNode n2 = mPointNodeMap.get(to);

		// check if nodes are valid
		if (n1 == null) throw new IllegalArgumentException("addEdge: from " + from + " is not in graph");
		if (n2 == null)	throw new IllegalArgumentException("addEdge: to " + to + " is not in graph");
		
		// actually adding the edge to the node start
		MapEdge edge = new MapEdge(roadName, roadType, n1, n2, length);
		mEdges.add(edge);
		n1.addEdge(edge);
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
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
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		// initial conditions check
		if (!checkNodes(start, goal)) return null;
		
		MapNode startNode = mPointNodeMap.get(start);
		MapNode endNode = mPointNodeMap.get(goal);

		// setup to begin BFS
		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		Queue<MapNode> toExplore = new LinkedList<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		toExplore.add(startNode);
		MapNode next = null;

		// traverse the graph
		while (!toExplore.isEmpty()) 
		{
			next = toExplore.remove();

			// hook for visualization
			nodeSearched.accept(next.getLocation());

			// we found it !!!
			if (next.equals(endNode)) break; 

			// moving through the graph and building parent map
			for (MapNode neighbor : getNeighbors(next)) 
			{
				if (!visited.contains(neighbor)) 
				{
					visited.add(neighbor);
					parentMap.put(neighbor, next);
					toExplore.add(neighbor);
				}
			}
		}

		// Reconstruct the parent path
		return reconstructPath(parentMap, startNode, endNode, next.equals(endNode));

	}
	
	private Set<MapNode> getNeighbors(MapNode node) {
		// TODO Auto-generated method stub
		return node.getNeighbors();
	}

	private boolean checkNodes(GeographicPoint start, GeographicPoint goal) {
		if (start == null || goal == null) {
			throw new NullPointerException("GeoPoints not valid!!!");
		}
		if (mPointNodeMap.get(start) == null) {
			System.err.println("Start node " + start + " does not exist");
			return false;
		}
		if (mPointNodeMap.get(goal) == null) {
			System.err.println("End node " + goal + " does not exist");
			return false;
		}
		return true;
	}
	
	
	private List<GeographicPoint> reconstructPath (HashMap<MapNode, 
			MapNode> parentMap, MapNode start, MapNode goal, boolean pathFound) 
	{
		if (!pathFound) 
		{
			System.out.println("No path found from " + start + " to " + goal);
			return null;
		}
		
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode current = goal;

		while (!current.equals(start)) 
		{
			path.addFirst(current.getLocation());
			current = parentMap.get(current);
		}

		// add start
		path.addFirst(start.getLocation());
		return path;
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
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
	
	/** Find the path from start to goal using Dijkstra's algorithm
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
		// TODO: Implement this method in WEEK 4
		
		if (!checkNodes(start, goal)) return null;
		
		// set it up start, goal, parent map, queue to explore and visited set of vertices
		MapNode startNode = mPointNodeMap.get(start);
		MapNode endNode = mPointNodeMap.get(goal);

		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();

		// initialize distances
		for (MapNode m : mPointNodeMap.values()) {
			m.setCalculatedDistance(Double.MAX_VALUE);
			m.setPredictedDistance(Double.MAX_VALUE);
		}
		
		startNode.setCalculatedDistance(0.0);
		startNode.setPredictedDistance(0.0);

		toExplore.add(startNode);
		MapNode next = null;

		// let us loop..
		while (!toExplore.isEmpty()) {
			next = toExplore.poll();

			if (!visited.contains(next)) {
				visited.add(next);

				// hook for visualization
				nodeSearched.accept(next.getLocation());

				if (next.equals(endNode)) break;

				// distances to the neighbors
				HashMap<MapNode, Double> distances = new HashMap<MapNode, Double>();
				for (MapEdge e : next.getEdges()) distances.put(e.getEndNode(), e.getLength());

				for (MapNode neighbor : getNeighbors(next)) 
				{
					if (!visited.contains(neighbor)) 
					{
						double distanceOfNode = next.getCalculatedDistance() + distances.get(neighbor);
						
						if (distanceOfNode < neighbor.getCalculatedDistance())
						{
							neighbor.setCalculatedDistance(distanceOfNode);
							// distanceOfNode += 0.0;
							neighbor.setPredictedDistance(distanceOfNode);
							parentMap.put(neighbor, next);
							toExplore.offer(neighbor);
						}
					}
				}
			}
		}
		// end of loop
			
		return reconstructPath(parentMap, startNode, endNode, endNode.equals(next));
	}


	
	

	/** Find the path from start to goal using A-Star search
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
	
	/** Find the path from start to goal using A-Star search
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
		// TODO: Implement this method in WEEK 4
		
		if (!checkNodes(start, goal)) return null;
		
		// set it up start, goal, parent map, queue to explore and visited set of vertices
		MapNode startNode = mPointNodeMap.get(start);
		MapNode endNode = mPointNodeMap.get(goal);

		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();

		// initialize distances
		for (MapNode m : mPointNodeMap.values()) {
			m.setCalculatedDistance(Double.MAX_VALUE);
			m.setPredictedDistance(Double.MAX_VALUE);
		}
		
		startNode.setCalculatedDistance(0.0);
		startNode.setPredictedDistance(0.0);

		toExplore.add(startNode);
		MapNode next = null;

		// let us loop..
		while (!toExplore.isEmpty()) {
			next = toExplore.poll();

			if (!visited.contains(next)) {
				visited.add(next);

				// hook for visualization
				nodeSearched.accept(next.getLocation());

				if (next.equals(endNode)) break;

				// distances to the neighbors
				HashMap<MapNode, Double> distances = new HashMap<MapNode, Double>();
				for (MapEdge e : next.getEdges()) distances.put(e.getEndNode(), e.getLength());

				for (MapNode neighbor : getNeighbors(next)) 
				{
					if (!visited.contains(neighbor)) 
					{
						double distanceOfNode = next.getCalculatedDistance() + distances.get(neighbor);
						
						if (distanceOfNode < neighbor.getCalculatedDistance())
						{
							neighbor.setCalculatedDistance(distanceOfNode);
							
							//this is the difference between A-star and Dijkstra
							distanceOfNode += neighbor.getLocation().distance(endNode.getLocation());
							//--------------------------------------------------
							
							neighbor.setPredictedDistance(distanceOfNode);
							parentMap.put(neighbor, next);
							toExplore.offer(neighbor);
						}
					}
				}
			}
		}
		// end of loop
			
		return reconstructPath(parentMap, startNode, endNode, endNode.equals(next));
		
		
	}

	
	
	
	
	
	
	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
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
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		*/
		
		/* Use this code in Week 3 End of Week Quiz */
		
		/*
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		*/


	}
	
}
