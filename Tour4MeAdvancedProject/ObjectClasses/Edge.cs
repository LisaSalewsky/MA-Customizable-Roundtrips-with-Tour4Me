using System;
using System.Collections.Generic;

namespace Tour4MeAdvancedProject.ObjectClasses
{

    public class Edge
    {
        //private double preference;

        public Guid Id { get; set; }
        public int GraphId { get; set; }
        public double ShoelaceForward { get; set; }
        public double ShoelaceBackward { get; set; }
        public bool Reversed { get; set; }
        public List<string> Tags { get; set; }
        public List<Tuple<double, double>> GeoLocations { get; set; }

        public Node SourceNode { get; set; }
        public Node TargetNode { get; set; }
        public double Pheromone { get; set; } = 1;
        public double TrailIntensity { get; set; } = 1;
        public double Cost { get; set; }
        public double Profit { get; set; }

        public Edge ()
        {
            Tags = new List<string>();
            GeoLocations = new List<Tuple<double, double>>();
        }

        public Edge ( int edgeGraphId, Node source, Node target, double nodeCost )
        {
            byte[] sourceBytes = BitConverter.GetBytes( source.GraphNodeId );
            byte[] targetBytes = BitConverter.GetBytes( target.GraphNodeId );

            if (BitConverter.IsLittleEndian)
            {
                Array.Reverse( sourceBytes );
                Array.Reverse( targetBytes );
            }
            byte[] combinedBytes = new byte[ 16 ];
            Buffer.BlockCopy( sourceBytes, 0, combinedBytes, 0, 4 );
            Buffer.BlockCopy( targetBytes, 0, combinedBytes, 4, 4 );
            for (int i = 8; i < 16; i++)
            {
                combinedBytes[ i ] = 0;
            }
            // Create a GUID from the concatenated byte array
            Id = new Guid( combinedBytes );
            GraphId = edgeGraphId;

            if (source.GraphNodeId < target.GraphNodeId)
            {
                SourceNode = source;
                TargetNode = target;
            }
            else
            {
                SourceNode = target;
                TargetNode = source;
            }
            Cost = nodeCost;

            Tags = new List<string>();
            GeoLocations = new List<Tuple<double, double>>();
        }
        public Edge ( Guid id, int graphId, Node source, Node target, double nodeCost ) : this( graphId, source, target, nodeCost )
        {
            Id = id;
        }

        public Edge ( Edge e, bool reversed )
        {
            Id = e.Id;
            GraphId = e.GraphId;
            SourceNode = e.SourceNode;
            TargetNode = e.TargetNode;

            Reversed = reversed;

            Tags = e.Tags;
            GeoLocations = e.GeoLocations;

            Pheromone = e.Pheromone;
            Profit = e.Profit;
            Cost = e.Cost;
        }

        public bool LessThan ( Edge otherEdge )
        {
            return SourceNode.GraphNodeId < otherEdge.SourceNode.GraphNodeId || ( SourceNode == otherEdge.SourceNode && TargetNode.GraphNodeId < otherEdge.TargetNode.GraphNodeId );
        }
    }

}