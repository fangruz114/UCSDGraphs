package roadgraph;

public class Pair<T1, T2> {
	
	private MapNode node;
	private double toDistance;

	public Pair(MapNode node, double toDistance) {
		this.node = node;
		this.toDistance = toDistance;
	}
	
	/*
	 * 
	 * @return the MapNode in the pair
	 */
	public MapNode getKey() {
		return node;
	}
	
	/*
	 * 
	 * @return the distance value in the pair
	 */
	public double getValue() {
		return toDistance;
	}
}
