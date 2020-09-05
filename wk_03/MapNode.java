/**
 * A class to represent a node in the map
 * @author UCSD MOOC development team and Branko Milosevic
 * 
 * Class representing a vertex (or node) in our MapGraph
 *
 */
package roadgraph;

import java.util.HashSet;
import java.util.Set;

import geography.GeographicPoint;

class MapNode
{
	/** The list of edges out of this node */
	private HashSet<MapEdge> mEdges;
		
	/** The latitude and longitude of this node */
	private GeographicPoint mLocation;
	
	MapNode(GeographicPoint loc)
	{
		mLocation = loc;
		mEdges = new HashSet<MapEdge>();
	}
		
	void addEdge(MapEdge edge)
	{
		mEdges.add(edge);
	}
	
	/** Return the neighbors of this MapNode */
	Set<MapNode> getNeighbors()
	{
		Set<MapNode> neighbors = new HashSet<MapNode>();
		for (MapEdge edge : mEdges) {
			neighbors.add(edge.getOtherNode(this));
		}
		return neighbors;
	}
	
	/** get the location of a node */
	GeographicPoint getLocation()
	{
		return mLocation;
	}
	
	/** return the edges out of this node */
	Set<MapEdge> getEdges()
	{
		return mEdges;
	}
	
	/** Returns whether two nodes are equal === locations are the same */
	public boolean equals(Object o)
	{
		if (!(o instanceof MapNode) || (o == null)) {
			return false;
		}
		MapNode node = (MapNode)o;
		return node.mLocation.equals(this.mLocation);
	}
	
	/** Might be useful...
	 * @return The HashCode for this node, which is the HashCode for the 
	 * underlying point
	 */
	public int HashCode()
	{
		return mLocation.hashCode();
	}
	
	/** ToString to print out a MapNode method
	 *  @return the string representation of a MapNode
	 */
	public String toString()
	{
		String toReturn = "[NODE at location (" + mLocation + ")";
		toReturn += " intersects streets: ";
		for (MapEdge e: mEdges) {
			toReturn += e.getRoadName() + ", ";
		}
		toReturn += "]";
		return toReturn;
	}

}