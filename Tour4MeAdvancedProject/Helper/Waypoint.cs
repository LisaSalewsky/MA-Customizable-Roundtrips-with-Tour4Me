using System;
using System.Collections.Generic;
using Tour4MeAdvancedProject.ObjectClasses;

namespace Tour4MeAdvancedProject.Helper
{
    public class Waypoint : ICloneable
    {
        public int NodeID { get; set; }
        public List<int> Path { get; set; }
        public List<Edge> Edges { get; set; }

        public Waypoint ( int nodeId, List<int> path, List<Edge> edges )
        {
            NodeID = nodeId;
            Path = path;
            Edges = edges;
        }

        public Waypoint ( Waypoint wp )
        {
            NodeID = wp.NodeID;
            Path = wp.Path;
            Edges = wp.Edges;
        }


        public object Clone ()
        {
            return new Waypoint( NodeID, new List<int>( Path ), Edges.ConvertAll( edge => (Edge)edge.Clone() ) );
        }

    }
}