using System;
using System.Collections.Generic;
using System.Linq;
using Tour4MeAdvancedProject.ObjectClasses;
using static Tour4MeAdvancedProject.Helper.EnumHelper;

namespace Tour4MeAdvancedProject.Helper
{
    public static class Utils
    {

        public static void AddTags ( ref HashSet<SurfaceTag> addedSurfaceTags, ref HashSet<HighwayTag> addedPathTypes, ref HashSet<string> addedSurroundings, string currentTag )
        {
            if (Enum.TryParse<SurfaceTag>( currentTag, true, out SurfaceTag surfaceTag ))
            {
                _ = addedSurfaceTags.Add( surfaceTag );
            }
            if (Enum.TryParse<HighwayTag>( currentTag, true, out HighwayTag pathType ))
            {
                _ = addedPathTypes.Add( pathType );
            }
            bool tagFound = false;
            foreach (string[] tagsArray in Surroundings.Values.Values)
            {
                if (tagsArray.Contains( currentTag ))
                {
                    tagFound = true;
                    // Tag found in one of the arrays
                    break;
                }
            }

            if (tagFound)
            {
                _ = addedSurroundings.Add( currentTag );
            }
        }

        public static void CalculateElevationDiffAndSteepness ( Edge edge, ref double maxSteepness, ref double currentElevationDiff )
        {
            double elevationDiff = Math.Abs( edge.SourceNode.Elevation - edge.TargetNode.Elevation );

            if (( elevationDiff / edge.Cost * 100 ) > maxSteepness && elevationDiff != 0)
            {
                maxSteepness = elevationDiff / edge.Cost * 100;
            }

            currentElevationDiff += elevationDiff;
        }
        public static void CalculateElevationDiff ( Edge edge, ref double currentElevationDiff )
        {
            double elevationDiff = Math.Abs( edge.SourceNode.Elevation - edge.TargetNode.Elevation );
            currentElevationDiff += elevationDiff;
        }

        internal static void CaculateQualityValues ( Problem problem, Edge v, double currentElevation, ref double currentEdgeProfits, ref double currentArea, ref double currentQuality )
        {
            currentEdgeProfits += v.Cost * v.Profit;
            currentArea += !v.Reversed ? v.ShoelaceForward : v.ShoelaceBackward;
            currentQuality = problem.GetQuality( currentEdgeProfits, currentArea, currentElevation, problem.Path.Length );
        }


        public static void UpdateCurrentProblemPathMetadata ( ref Problem CurrentProblem, HashSet<SurfaceTag> addedSurfaceTags, HashSet<HighwayTag> addedPathTypes, HashSet<string> addedSurroundings, double currentEdgeProfits, double currentArea, double currentQuality, double currentPathsMaxSteepness, double currentElevationDiff, Tuple<double, double>[] boudingCoordinates )
        {
            CurrentProblem.Path.Length = CurrentProblem.Path.Edges.Sum( x => x.Cost );
            CurrentProblem.Path.Steepness = currentPathsMaxSteepness;
            CurrentProblem.Path.Elevation = currentElevationDiff / 2;
            CurrentProblem.Path.BoundingCoordinates = boudingCoordinates;
            CurrentProblem.Path.PathTypes = string.Join( ", ", addedPathTypes );
            CurrentProblem.Path.Surfaces = string.Join( ", ", addedSurfaceTags );
            CurrentProblem.Path.SurroundingTags = string.Join( ", ", addedSurroundings );
            //CurrentProblem.Path.Quality = CurrentProblem.GetQuality( CurrentProblem.GetProfit( CurrentProblem.Path.Visited ), CurrentProblem.GetArea( CurrentProblem.Path.Visited ), CurrentProblem.Path.Elevation );
            CurrentProblem.Path.TotalEdgeProfits = currentEdgeProfits;
            CurrentProblem.Path.Quality = currentQuality;
            CurrentProblem.Path.CoveredArea = currentArea;
        }


    }
}