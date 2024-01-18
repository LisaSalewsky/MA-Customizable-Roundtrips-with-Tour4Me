using System.Collections.Generic;

namespace Tour4MeAdvancedProject.ObjectClasses
{
    public class Path
    {
        private readonly int Id;
        public List<Edge> Edges { get; }
        public List<int> Visited { get; set; }
        public double Quality { get; set; }
        public double Length { get; set; }

        public Path ()
        {
            Edges = new List<Edge>();
            Visited = new List<int>();
        }

        public Path ( List<Edge> edges, List<int> visited, double quality, double length )
        {
            Edges = edges;
            Visited = visited;
            Quality = quality;
            Length = visited.Count;
        }

        public void Add ( Edge addEdge, int neighbor, double newProfit, double newLength )
        {
            if (addEdge != null)
            {
                Edges.Add( addEdge );
            }
            Visited.Add( neighbor );
            Quality += newProfit;
            Length += newLength;
        }
    }
}