using System.Collections.Generic;
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
        public string SurroundingTags { get; set; } = "surroundigns test";
        public string PathTypes { get; set; }
        public string Surfaces { get; set; }

        public Path ()
        {
            Edges = new List<Edge>();
            Visited = new List<int>();
        }

        public Path ( List<Edge> edges, List<int> visited, double quality )
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