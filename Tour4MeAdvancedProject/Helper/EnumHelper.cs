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
            SimulatedAnnealing,
            Genetic
        }

        public enum SolveStatus
        {
            Unsolved,
            Optimal,
            Feasible,
            Timeout
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
            { SurroundingType.Forest, new[] { "wood", "forest", "scrub", "heath", "tree_row", "tree", "coppice", "tree_group", "grove", "orchard" } },
            { SurroundingType.CoastalArea, new[] { "coastline", "beach", "sand", "dune", "shoal", "reef", "bay", "cape", "spit", "cliff", "rock" } },
            { SurroundingType.Mountains, new[] { "peak", "ridge", "hill", "fell ", "tundra ", "saddle", "valley", "cliff", "scree", "glacier", "moraine" } },
            { SurroundingType.Grassland, new[] { "grass", "grassland", "meadow", "pasture" } },
            { SurroundingType.City, new[] { "residential", "apartments", "houses", "residential_area", "park", "playground", "recreation_ground", "greenfield" } },
            { SurroundingType.WaterFeatures, new[] { "spring", "hot_spring", "geyser", "water"} },
            { SurroundingType.Others, new[] { "bare_rock", "sink" } }
        };
    }
}