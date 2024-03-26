﻿using System.Collections.Generic;
using System.Linq;

namespace Tour4MeAdvancedProject.ObjectClasses
{
    public class Path
    {
        public List<Edge> Edges { get; }
        public List<int> Visited { get; set; }
        public double Quality { get; set; }
        public double Length { get; set; }
        public double Elevation { get; set; } = 100;
        public double Steepness { get; set; } = 100;
        public double CoveredArea { get; set; }
        public string SurroundingTags { get; set; }
        public string PathTypes { get; set; }
        public string Surfaces { get; set; }

        public Path ()
        {
            Edges = new List<Edge>();
            Visited = new List<int>();
        }

        public Path ( Path p )
        {
            Edges = p.Edges;
            Visited = p.Visited;
            Length = p.Length;
            Elevation = p.Elevation;
            Steepness = p.Steepness;
            CoveredArea = p.CoveredArea;
            SurroundingTags = p.SurroundingTags;
            PathTypes = p.PathTypes;
            Surfaces = p.Surfaces;

        }

        public Path ( List<Edge> edges, List<int> visited, double quality, Path p ) : this( p )
        {
            Edges = edges;
            Visited = visited;
            Quality = quality;
            Length = edges.Sum( x => x.Cost );
        }

        public Path ( List<Edge> edges, List<int> visited, double quality, double length )
        {
            Edges = edges;
            Visited = visited;
            Quality = quality;
            Length = length;
        }

        public void Add ( Edge addEdge, int neighbor, double newProfit )
        {
            if (addEdge != null)
            {
                Edges.Add( addEdge );
                Quality += newProfit;
                Length += addEdge.Cost;
            }
            Visited.Add( neighbor );
        }
    }
}