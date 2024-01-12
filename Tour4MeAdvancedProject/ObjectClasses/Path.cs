using System;
using System.Collections.Generic;
using System.Linq;
using System.Web;

namespace Tour4MeAdvancedProject.ObjectClasses
{
    public class Path
    {
        private int Id;
        public List<Edge> Edges { get; }
        public List<int> Visited { get; set; }
        public double Quality { get; set; }
        public double Length { get; set; }

        public Path()
        {
            Edges = new List<Edge>();
            Visited = new List<int>();
        }

    }
}