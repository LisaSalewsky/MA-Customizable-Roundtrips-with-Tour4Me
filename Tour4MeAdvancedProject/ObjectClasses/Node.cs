using System;
using System.Collections.Generic;


namespace Tour4MeAdvancedProject.ObjectClasses
{

    public class Node : ICloneable
    {
        public double Lat { get; set; }
        public double Lon { get; set; }
        public double Elevation { get; set; }
        public Guid NodeId { get; set; }
        public int GraphNodeId { get; set; }
        public List<Edge> Incident { get; set; }
        public HashSet<string> Suroundings { get; set; } = new HashSet<string>();
        public double ShortestDistance { get; set; }

        public Node ( int graphId, Guid id, double nodeLat, double nodeLon )
        {
            Lat = nodeLat;
            Lon = nodeLon;
            GraphNodeId = graphId;
            NodeId = id;
            Incident = new List<Edge>();
        }

        public Node ( int graphId, Guid id, double lat, double lon, double elevation ) : this( graphId, id, lat, lon )
        {
            Elevation = elevation;
        }

        public Node ()
        {
            // Default constructor
            Incident = new List<Edge>();
        }

        public double Distance ( double sourceLat, double sourceLon )
        {
            double lonA = sourceLon - Lon;
            double latA = sourceLat - Lat;

            return Math.Sqrt( ( latA * latA ) + ( lonA * lonA ) );
        }


        public double VecAngle ( Node n1, Node n2 )
        {
            if (Compare( n1 ) || Compare( n2 ) || n1.Compare( n2 )) { return 0; }

            double lonA = n1.Lon - Lon;
            double latA = n1.Lat - Lat;

            double lonB = n2.Lon - Lon;
            double latB = n2.Lat - Lat;

            return Math.Acos( ( ( lonA * lonB ) + ( latA * latB ) ) /
                             ( Math.Sqrt( ( lonA * lonA ) + ( latA * latA ) ) * Math.Sqrt( ( lonB * lonB ) + ( latB * latB ) ) ) ) / Math.PI;
        }

        public bool Compare ( Node otherNode )
        {
            return GraphNodeId == otherNode.GraphNodeId;
        }

        public bool IsRight ( Node l1, Node l2 )
        {
            return ( ( ( l2.Lon - l1.Lon ) * ( Lat - l1.Lat ) ) - ( ( l2.Lat - l1.Lat ) * ( Lon - l1.Lon ) ) ) <= 0;
        }


        public object Clone ()
        {
            // Deep copy the Incident list
            List<Edge> clonedIncident = new List<Edge>( Incident.Count );
            foreach (Edge edge in Incident)
            {
                clonedIncident.Add( (Edge)edge.Clone() );
            }

            // Deep copy the Suroundings set
            HashSet<string> clonedSuroundings = new HashSet<string>( Suroundings );

            return new Node
            {
                Lat = Lat,
                Lon = Lon,
                Elevation = Elevation,
                NodeId = NodeId,
                GraphNodeId = GraphNodeId,
                Incident = clonedIncident,
                Suroundings = clonedSuroundings,
                ShortestDistance = ShortestDistance
            };
        }
    }

}