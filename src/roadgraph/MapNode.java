package roadgraph;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import geography.GeographicPoint;


public class MapNode {
    
	private List<MapNode> neighbors;
	private GeographicPoint loc;
	private double toDistance;
	private List<MapEdge> edges;

	public MapNode(GeographicPoint loc)
	{
		this.loc = loc;
		this.toDistance = Double.POSITIVE_INFINITY;
		neighbors = new LinkedList<MapNode>();
		edges = new ArrayList<MapEdge>();
	}

	public void addNeighbor(MapNode neighbor) 
	{
		neighbors.add(neighbor);
	}
	
	/**
	 * @return the neighbors
	 */
	public List<MapNode> getNeighbors() {
		return neighbors;
	}
	
	
	public void addEdge(MapEdge edge) {
		edges.add(edge);
	}
	
	/**
	 * 
	 * @return the edge list
	 */
	public List<MapEdge> getEdges() {
		return edges;
	}

	/**
	 * @return the location
	 */
	public GeographicPoint getLoc() {
		return loc;
	}
	
	/**
	 * 
	 * @return distance from start node to this node
	 */
	public double getToDistance() {
		return toDistance;
	}
	
	
	public void setToDistance(Double toDistance) {
		this.toDistance = toDistance;
	}
	
	/**
	 * 
	 * @return distance to the specific neighbor node
	 */
	public double neighborDistance(MapNode neighbor) {
		double length = 0.00;
		for (MapEdge e: edges) {
			if (e.getEnd() == neighbor) {
				length = e.getDistance();
			}
		}
		return length;
	}
}
