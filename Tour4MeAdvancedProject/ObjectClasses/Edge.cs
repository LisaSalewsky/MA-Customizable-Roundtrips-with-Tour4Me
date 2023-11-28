using System;
using System.Collections.Generic;

namespace Tour4MeAdvancedProject.ObjectClasses
{

    public struct DirEdge
    {
        public Edge Edge { get; set; }
        public bool Reversed { get; set; }
        public DirEdge(Edge edge, bool reversed)
        {
            Edge = edge;
            Reversed = reversed;
        }
    }


    public class Path
    {
        public List<DirEdge> Edges { get; } 
        public List<int> Visited { get; set; }
        public double Quality { get; set; }
        public double Length { get; set; }

        public Path()
        {
            Edges = new List<DirEdge>();
            Visited = new List<int>();
        }

    }


    public class Edge
    {
        //private double preference;

        public int Id { get; set; }
        public double ShoelaceForward { get; set; }
        public double ShoelaceBackward { get; set; }
        public List<string> Tags { get; set; }
        public List<Tuple<double, double>> GeoLocations { get; set; }

        public Edge()
        {
            Tags = new List<string>();
            GeoLocations = new List<Tuple<double, double>>();
        }

        public Edge(int nodeId, int source, int target, double nodeCost)
        {
            Id = nodeId;
            if (source < target)
            {
                S = source;
                T = target;
            }
            else
            {
                S = target;
                T = source;
            }
            Cost = nodeCost;

            Tags = new List<string>();
            GeoLocations = new List<Tuple<double, double>>();
        }

        public int S { get; set; }
        public int T { get; set; }
        public double Pheromone { get; set; } = 1;
        public double Cost { get; set; }
        public double Profit { get; set; }

        public bool LessThan(Edge otherEdge)
        {
            return S < otherEdge.S || (S == otherEdge.S && T < otherEdge.T);
        }
    }

}