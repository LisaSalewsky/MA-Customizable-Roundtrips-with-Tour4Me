using System;
using System.Collections.Generic;

namespace Tour4MeAdvancedProject.ObjectClasses
{

    public class Edge
    {
        //private double preference;

        public int Id { get; set; }
        public double ShoelaceForward { get; set; }
        public double ShoelaceBackward { get; set; }
        public bool Reversed { get; set; }
        public List<string> Tags { get; set; }
        public List<Tuple<double, double>> GeoLocations { get; set; }

        public Node SourceNode { get; set; }
        public Node TargetNode { get; set; }
        public double Pheromone { get; set; } = 1;
        public double TrailIntensity { get; set; }
        public double Cost { get; set; }
        public double Profit { get; set; }

        public Edge()
        {
            Tags = new List<string>();
            GeoLocations = new List<Tuple<double, double>>();
        }

        public Edge(Node source, Node target, double nodeCost)
        {
            int.TryParse(source.Id.ToString() + target.Id.ToString(), out int id);
            Id = id;
            if (source.Id < target.Id)
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

        public Edge (Edge e, bool reversed)
        {
            Id = e.Id;
            SourceNode = e.SourceNode;
            TargetNode = e.TargetNode;

            Reversed = reversed;

            Tags = e.Tags;
            GeoLocations = e.GeoLocations;

            Pheromone = e.Pheromone;
            Profit = e.Profit;
            Cost = e.Cost;
        }

        public bool LessThan(Edge otherEdge)
        {
            return SourceNode.Id < otherEdge.SourceNode.Id || (SourceNode == otherEdge.SourceNode && TargetNode.Id < otherEdge.TargetNode.Id);
        }
    }

}