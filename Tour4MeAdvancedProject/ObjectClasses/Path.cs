using System;
using System.Collections.Generic;
using System.Linq;

namespace Tour4MeAdvancedProject.ObjectClasses
{
    public class Path
    {
        public List<Edge> Edges { get; set; }
        public List<int> Visited { get; set; }
        public double Quality { get; set; }
        public double Length { get; set; }
        public double Elevation { get; set; } = 100;
        public double Steepness { get; set; } = 100;
        public double CoveredArea { get; set; }
        public string SurroundingTags { get; set; }
        public string PathTypes { get; set; }
        public string Surfaces { get; set; }
        // 4 bounding coordinates
        // left, bottom, top, right
        public Tuple<double, double>[] BoundingCoordinates { get; set; } = new Tuple<double, double>[] {
            Tuple.Create( 0.0, 0.0 ), Tuple.Create( 0.0, 0.0 ), Tuple.Create( 0.0, 0.0 ), Tuple.Create( 0.0, 0.0 ) };

        public Path ( Tuple<double, double> start )
        {
            Edges = new List<Edge>();
            Visited = new List<int>();
            BoundingCoordinates = new Tuple<double, double>[] {
            start, start, start, start };
        }

        public Path ( Path p )
        {
            Edges = new List<Edge>( p.Edges );
            Visited = new List<int>( p.Visited );
            Length = p.Length;
            Elevation = p.Elevation;
            Steepness = p.Steepness;
            CoveredArea = p.CoveredArea;
            SurroundingTags = p.SurroundingTags;
            PathTypes = p.PathTypes;
            Surfaces = p.Surfaces;
            BoundingCoordinates = p.BoundingCoordinates;

        }

        public Path ( List<Edge> edges, List<int> visited, double quality, Path p ) : this( p )
        {
            Edges = edges;
            Visited = visited;
            Quality = quality;
            Length = edges.Sum( x => x.Cost );
        }

        public Path ( List<Edge> edges, List<int> visited, double quality, double length, Tuple<double, double> start )
        {
            Edges = edges;
            Visited = visited;
            Quality = quality;
            Length = length;
            BoundingCoordinates = new Tuple<double, double>[] {
            start, start, start, start };
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
        public void Insert ( Edge addEdge, int idx, int neighbor, double newProfit )
        {
            if (addEdge != null)
            {
                Edges.Insert( idx, addEdge );
                Quality += newProfit;
                Length += addEdge.Cost;
            }
            Visited.Add( neighbor );
        }


        public void UpdateBoundingCoordinates ( ref Tuple<double, double>[] boundingCoordinates, Node neighbor )
        {

            // insert the 4 boudning coordinates
            // left
            if (boundingCoordinates[ 0 ].Item1 > neighbor.Lat)
            {
                boundingCoordinates[ 0 ] = Tuple.Create( neighbor.Lat, neighbor.Lon );
            }
            // right
            if (boundingCoordinates[ 3 ].Item1 < neighbor.Lat)
            {
                boundingCoordinates[ 3 ] = Tuple.Create( neighbor.Lat, neighbor.Lon );
            }
            // top
            if (boundingCoordinates[ 2 ].Item2 > neighbor.Lon)
            {
                boundingCoordinates[ 2 ] = Tuple.Create( neighbor.Lat, neighbor.Lon );
            }
            //bottom
            if (boundingCoordinates[ 1 ].Item2 < neighbor.Lon)
            {
                boundingCoordinates[ 1 ] = Tuple.Create( neighbor.Lat, neighbor.Lon );
            }
        }
    }
}