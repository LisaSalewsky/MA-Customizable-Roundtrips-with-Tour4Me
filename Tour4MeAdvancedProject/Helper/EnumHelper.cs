using System.Collections.Generic;

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
            ILS,
            AntColony,
            AntMinCost,
            AntGreedy,
            SimulatedAnnealingGreedy,
            SimulatedAnnealingMinCost,
            SimulatedAnnealingAnt,
            SimulatedAnnealingEmpty,
            SimulatedAnnealingFullyRandom
        }

        public enum SolveStatus
        {
            Unsolved,
            Optimal,
            Feasible,
            Timeout
        }

        public enum TourShape
        {
            Round,
            UTurn,
            Complex,
            Custom
        }

    }

    public static class Surroundings
    {
        public enum SurroundingType
        {
            // from tag natural
            Forest,
            CoastalArea,
            Mountains,
            Grassland,
            // values from tag Landuse (features residential and recreational/ open space)
            City,
            // from tag natural
            WaterFeatures,
            Others
        }
        public static Dictionary<SurroundingType, string[]> Values = new Dictionary<SurroundingType, string[]>
        {
            { SurroundingType.Forest, new[] { "Coppice", "Forest", "Grove", "Heath", "Orchard", "Scrub", "Tree", "Tree Group", "Tree row", "Wood" } },
            { SurroundingType.CoastalArea, new[] { "Bay", "Beach", "Cape", "Cliff", "Coastline", "Dune", "Reef", "Rock", "Sand", "Shoal", "Spit" } },
            { SurroundingType.Mountains, new[] { "Cliff", "Fell", "Glacier", "Hill", "Moraine", "Peak", "Ridge", "Scree", "Saddle", "Tundra", "Valley" } },
            { SurroundingType.Grassland, new[] { "Grass", "Grassland", "Meadow", "Pasture" } },
            { SurroundingType.City, new[] { "Apartments", "Greenfield", "Houses", "Park", "Playground", "Recreation ground", "Residential", "Residential area" } },
            { SurroundingType.WaterFeatures, new[] { "Geyser", "Hot spring", "Spring", "Water" } },
            { SurroundingType.Others, new[] { "Bare rock", "Sink"  } }
        };
    }
}