/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.List;
import java.util.ListIterator;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Set;
import java.util.HashSet;
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
	//TODO: Add your member variables here in WEEK 3
	private Map<GeographicPoint, MapNode> vertices;
	private int numEdges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		vertices = new HashMap<GeographicPoint, MapNode>();
		numEdges = 0;
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return vertices.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		return vertices.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return numEdges;
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
		MapNode vertex = new MapNode(location);
		if (vertices.keySet().contains(vertex) || location == null) {
			return false;
		}
		vertices.put(location, vertex);
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
		if (!vertices.keySet().contains(from) || !vertices.keySet().contains(to)) {
			throw new IllegalArgumentException("Points haven't added to graph.");
		}
		if (from == null || to == null || roadName == null || roadType == null || length < 0) {
			throw new IllegalArgumentException("Arguments is null");
		}
		MapNode start = vertices.get(from);
		MapNode end = vertices.get(to);
		MapEdge edge = new MapEdge(start, end, roadName, roadType, length);
		edge.setStreetName(roadName);
		edge.setStreetType(roadType);
		edge.setDistance(length);
		start.addEdge(edge);
		start.addNeighbor(end);
		numEdges++;
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighed)
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
	 * @return The list of intersections that form the shortest (unweighed)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		//initialize
		MapNode startNode = vertices.get(start);
		MapNode endNode = vertices.get(goal);
		
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		// Do the search - put the code details for the search to bfsSearch function
		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		boolean found = bfsSearch(startNode, endNode, parentMap, nodeSearched);

		if (!found) {
			System.out.println("No path exists");
			return new LinkedList<GeographicPoint>();
		}

		// reconstruct the path - detailed codes are in constructPath function
		return constructPath(startNode, endNode, parentMap);
	}
	
	
	private boolean bfsSearch(MapNode start, MapNode goal, HashMap<MapNode, MapNode> parentMap, Consumer<GeographicPoint> nodeSearched) {
		Queue<MapNode> toExplore = new LinkedList<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		toExplore.add(start);
		nodeSearched.accept(start.getLoc());
		boolean found = false;

		while (!toExplore.isEmpty()) {
			MapNode curr = toExplore.remove();
			if (curr == goal) {
				found = true;
				nodeSearched.accept(curr.getLoc());
				break;
			}
			List<MapNode> neighbors = curr.getNeighbors();
			ListIterator<MapNode> it = neighbors.listIterator(neighbors.size());
			while (it.hasPrevious()) {
				MapNode next = it.previous();
				if (!visited.contains(next)) {
					visited.add(next);
					nodeSearched.accept(next.getLoc());
					parentMap.put(next, curr);
					toExplore.add(next);
				}
			}
		}
		return found;
	}

	private List<GeographicPoint> constructPath(MapNode start, MapNode goal, HashMap<MapNode, MapNode> parentMap) {
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode curr = goal;
		while (curr != start) {
			path.addFirst(curr.getLoc());
			curr = parentMap.get(curr);
		}
		path.addFirst(start.getLoc());
		return path;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	/** Dijkstra(S,G):
     * Initialize: Priority queue (PQ), visited HashSet, parent HashMap, and distances to infinity.
     * Enqueue {S,0} onto the PQ
     * while PQ is not empty:
     *    dequeue node curr from front of queue
     *    if (curr is not visited)
     *        add curr to visited set
     *        if (curr == G) return parent map
     *        for each of curr's neighbors, n, not in visited set:
     *            if path through curr to n is shorter
     *                update curr as n's parent in parent map
     *                enqueue {n,distance} into the PQ
     *if we get here then there's no path from S to G
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
		//initialize
		MapNode startNode = vertices.get(start);
		MapNode endNode = vertices.get(goal);
		
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		// Do the search - put the code details for the search to bfsSearch function
		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		boolean found = dijkstraSearch(startNode, endNode, parentMap, nodeSearched);

		if (!found) {
			System.out.println("No path exists");
			return new LinkedList<GeographicPoint>();
		}

		// reconstruct the path - detailed codes are in constructPath function
		return constructPath(startNode, endNode, parentMap);
	}
	
	public boolean dijkstraSearch(MapNode start, MapNode goal, HashMap<MapNode, MapNode> parentMap, Consumer<GeographicPoint> nodeSearched) {
		PriorityQueue<Pair<MapNode, Double>> toExplore = new PriorityQueue<Pair<MapNode, Double>>(numEdges+1, Comparator.comparing(Pair::getValue));
		HashSet<MapNode> visited = new HashSet<MapNode>();
		for (GeographicPoint g: vertices.keySet()) {
			vertices.get(g).setToDistance(Double.POSITIVE_INFINITY);
		}
		start.setToDistance(0.00);
		toExplore.add(new Pair<MapNode, Double>(start, start.getToDistance()));
		nodeSearched.accept(start.getLoc());
		boolean found = false;

		while (!toExplore.isEmpty()) {
			Pair<MapNode, Double> curr = toExplore.remove();
			if (!visited.contains(curr.getKey())) {
				visited.add(curr.getKey());
				nodeSearched.accept(curr.getKey().getLoc());
				if (curr.getKey() == goal) {
					found = true;
					break;
				}
			}
			List<MapNode> neighbors = curr.getKey().getNeighbors();
			ListIterator<MapNode> it = neighbors.listIterator(neighbors.size());
			while (it.hasPrevious()) {
				MapNode next = it.previous();
				double nextToCurr = curr.getKey().neighborDistance(next);
				if (!visited.contains(next)) {
					double newDistance = curr.getValue() + nextToCurr;
					if (newDistance < next.getToDistance()) {
						next.setToDistance(newDistance);
						parentMap.put(next, curr.getKey());
						toExplore.add(new Pair<MapNode, Double>(next, newDistance));
					}
				}
			}
		}
		System.out.println(visited.size());
		return found;
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
		//initialize
		MapNode startNode = vertices.get(start);
		MapNode endNode = vertices.get(goal);
		
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		// Do the search - put the code details for the search to bfsSearch function
		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		boolean found = aStarSearchSearch(startNode, endNode, parentMap, nodeSearched);

		if (!found) {
			System.out.println("No path exists");
			return new LinkedList<GeographicPoint>();
		}

		// reconstruct the path - detailed codes are in constructPath function
		return constructPath(startNode, endNode, parentMap);
	}

	public boolean aStarSearchSearch(MapNode start, MapNode goal, HashMap<MapNode,MapNode> parentMap, Consumer<GeographicPoint> nodeSearched) {
		PriorityQueue<Pair<MapNode, Double>> toExplore = new PriorityQueue<Pair<MapNode, Double>>(numEdges+1, Comparator.comparing(Pair::getValue));
		HashSet<MapNode> visited = new HashSet<MapNode>();
		start.setToDistance(0.00);
		for (GeographicPoint g: vertices.keySet()) {
			vertices.get(g).setToDistance(Double.POSITIVE_INFINITY);
		}
		start.setToDistance(0.00);
		toExplore.add(new Pair<MapNode, Double>(start, start.getToDistance()));
		nodeSearched.accept(start.getLoc());
		boolean found = false;

		while (!toExplore.isEmpty()) {
			Pair<MapNode, Double> curr = toExplore.remove();
			if (!visited.contains(curr.getKey())) {
				visited.add(curr.getKey());
				nodeSearched.accept(curr.getKey().getLoc());
				if (curr.getKey() == goal) {
					found = true;
					break;
				}
			}
			List<MapNode> neighbors = curr.getKey().getNeighbors();
			ListIterator<MapNode> it = neighbors.listIterator(neighbors.size());
			while (it.hasPrevious()) {
				MapNode next = it.previous();
				double nextToCurr = curr.getKey().neighborDistance(next);
				double estNextToGoal = next.getLoc().distance(goal.getLoc());
				double estCurrToGoal = curr.getKey().getLoc().distance(goal.getLoc());
				if (!visited.contains(next)) {
					double newDistance = curr.getValue() - estCurrToGoal + nextToCurr + estNextToGoal;
					if (newDistance < next.getToDistance()) {
						next.setToDistance(newDistance);
						parentMap.put(next, curr.getKey());
						toExplore.add(new Pair<MapNode, Double>(next, newDistance));
					}
				}
			}
		}
		System.out.println(visited.size());
		return found;
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
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		List<GeographicPoint> testroutebfs = simpleTestMap.bfs(testStart,testEnd);
		System.out.println(testroutebfs);
		
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

		
		
		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
			
	}
	
}

