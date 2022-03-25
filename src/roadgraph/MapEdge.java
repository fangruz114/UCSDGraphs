package roadgraph;

public class MapEdge {
    
	private MapNode start;
	private MapNode end;
	private String streetName;
	private String streetType;
	private double distance;
	
	public MapEdge(MapNode start, MapNode end) {
		this.start = start;
		this.end = end;
		this.streetName = null;
		this.streetType = null;
		this.distance = 0.0;
	}
	
	public MapEdge(MapNode start, MapNode end, String streetName, String streetType, double distance) {
		this.start = start;
		this.end = end;
		this.streetName = streetName;
		this.streetType = streetType;
		this.distance = distance;
	}
	
	public String getStreetName() {
		return streetName;
	}
	
	public void setStreetName(String streetName) {
		this.streetName = streetName;
	}
	
	public String getStreetType() {
		return streetType;
	}
	
	public void setStreetType(String streetType) {
		this.streetType = streetType;
	}
	
	public double getDistance() {
		return distance;
	}
	
	public void setDistance(double distance) {
		this.distance = distance;
	}
	
	public MapNode getStart() {
		return start;
	}
	
	public MapNode getEnd() {
		return end;
	}
}
