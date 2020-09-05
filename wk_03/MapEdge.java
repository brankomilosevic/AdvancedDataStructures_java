/**
 * @author UCSD Intermediate Programming MOOC team and Branko Milosevic
 *
 * A directed edge in a map graph 
 * 	from Node start 
 * 	to Node end 
 * 	with length, road name and road type
 */

package roadgraph;

import geography.GeographicPoint;

class MapEdge 
{
	/** The name of the road */
	private String mRoadName;
	
	/** The type of the road */
	private String mRoadType;
	
	/** The two end-points of the edge */
	private MapNode mStart;
	private MapNode mEnd;
	
	/** The length of the road segment, in km */
	private double mLength;
	
	/** for initialization */
	static final double DEFAULT_LENGTH = 0.01;
	
	/** Create a new MapEdge object
	 * 
	 * @param roadName
	 * @param roadType
	 * @param n1  The point at one end of the segment
	 * @param n2  The point at the other end of the segment
	 * @param length
	 */

	MapEdge(String roadName, String roadType, MapNode n1, MapNode n2, double length) 
	{
		mRoadName = roadName;
		mStart = n1;
		mEnd = n2;
		mRoadType = roadType;
		mLength = length;
	}
	
	// return the MapNode for the end point
	MapNode getEndNode() {
	   return mEnd;
	}
	
	// return the location of the start point
	GeographicPoint getStartPoint()
	{
		return mStart.getLocation();
	}
	
	// return the location of the end point
	GeographicPoint getEndPoint()
	{
		return mEnd.getLocation();
	}
	
	// return the length
	double getLength()
	{
		return mLength;
	}
	
	// return road name
	public String getRoadName()
	{
		return mRoadName;
	}
	
	// given one node in an edge, return the other node
	MapNode getOtherNode(MapNode node)
	{
		if (node.equals(mStart)) 
			return mEnd;
		else if (node.equals(mEnd))
			return mStart;
		
		throw new IllegalArgumentException("Point that is not on the edge!!!");
	}
	
	// return String containing details about the edge
	public String toString()
	{
		String toReturn = "[EDGE between ";
		toReturn += "\n\t" + mStart.getLocation();
		toReturn += "\n\t" + mEnd.getLocation();
		toReturn += "\nRoad name: " + mRoadName + " Road type: " + mRoadType +
				" Segment length: " + String.format("%.3g", mLength) + "km";
		
		return toReturn;
	}

}