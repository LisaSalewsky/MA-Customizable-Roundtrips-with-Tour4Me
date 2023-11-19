using System;
using System.Collections.Generic;
using System.Linq;
using System.Web;

namespace Tour4MeAdvancedProject.Helper
{
    public class EnumHelper
    {
       public enum HighwayTag
        {
            Footway,
            Cycleway,
            Unclassified,
            Residential,
            Path,
            Track,
            Secondary
        }

        public enum SurfaceTag
        {
            Paved,
            Cobblestone,
            Gravel,
            Unpaved,
            Compacted,
            FineGravel,
            Rock,
            Pebblestone
        }

        public enum Algo
        {
            Greedy,
            minCost,
            ILS
        }
    }
}