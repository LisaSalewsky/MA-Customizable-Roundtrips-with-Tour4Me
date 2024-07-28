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

        internal static void CaculateQualityValues ( Problem problem, Edge v, int startNodeId, double currentDistance, double currentElevation, ref double currentSteepness, ref double currentEdgeProfits, ref double currentArea, ref double currentQuality )
        {
            currentEdgeProfits += !v.Visited ? v.Cost * v.Profit : 0;
            v.Visited = true;
            currentArea += v.SourceNode.GraphNodeId == startNodeId ? v.ShoelaceForward : v.ShoelaceBackward;
            double edgeSteepness = Math.Abs( v.SourceNode.Elevation - v.TargetNode.Elevation );
            currentSteepness = edgeSteepness > currentSteepness ? edgeSteepness : currentSteepness;

            currentQuality = problem.GetQuality( currentEdgeProfits, currentArea, currentElevation / 2, currentSteepness, currentDistance );
            if (currentQuality != currentQuality)
            {
                Console.WriteLine( "ahhhhh" );
            }
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
            if (currentQuality != currentQuality)
            {
                Console.WriteLine( "ahhhhh" );
            }
        }

        public static void UpdateMetadata ( Path visitedPath, Problem p )
        {

            HashSet<SurfaceTag> addedSurfaceTags = new HashSet<SurfaceTag>();
            HashSet<HighwayTag> addedPathTypes = new HashSet<HighwayTag>();
            HashSet<string> addedSurroundings = new HashSet<string>();
            double elevationDiff = 0;
            double maxSteepness = 0;
            List<int> visited = visitedPath.Visited;

            List<Edge> pathEdges = visitedPath.Edges;
            p.Path.Length = pathEdges.Sum( x => x.Cost );

            double totalEdgeProfits = 0;
            int i = 0;
            foreach (Edge edge in pathEdges)
            {
                int startNodeId = visited[ i ];
                i++;
                foreach (string tag in edge.Tags)
                {
                    Utils.AddTags( ref addedSurfaceTags, ref addedPathTypes, ref addedSurroundings, tag );
                }
                Utils.CalculateElevationDiffAndSteepness( edge, ref maxSteepness, ref elevationDiff );
                //Utils.CaculateQualityValues( p, edge, startNodeId, elevationDiff, ref currentEdgeProfits, ref currentArea, ref currentQuality );
                totalEdgeProfits += !edge.Visited ? edge.Profit * edge.Cost : 0;
                edge.Visited = true;
            }
            pathEdges.ForEach( x => x.Visited = false );

            p.Path.CoveredArea = p.GetArea( visited );

            p.Path.TotalEdgeProfits = totalEdgeProfits;
            //p.Path.TotalEdgeProfits = pathEdges.Sum( x => x.Profit * x.Cost );
            p.Path.Quality = p.GetQuality( p.Path.TotalEdgeProfits, p.Path.CoveredArea, elevationDiff / 2, maxSteepness, p.Path.Length );
            if (p.Path.Quality != p.Path.Quality)
            {
                Console.WriteLine( "ahhhhh" );
            }

            // update all current path values
            p.Path.Visited = visited;
            p.Path.Edges = pathEdges;
            p.Path.Elevation = elevationDiff / 2;
            p.Path.Steepness = maxSteepness;
            //p.Path.CoveredArea = currentArea;
            //p.Path.TotalEdgeProfits = currentEdgeProfits;
            //p.Path.Quality = currentQuality;
            p.Path.PathTypes = string.Join( ", ", addedPathTypes );
            p.Path.Surfaces = string.Join( ", ", addedSurfaceTags );
            p.Path.SurroundingTags = string.Join( ", ", addedSurroundings );
        }

        public static void UpdatePathMetadata ( Path visitedPath, Problem p )
        {

            HashSet<SurfaceTag> addedSurfaceTags = new HashSet<SurfaceTag>();
            HashSet<HighwayTag> addedPathTypes = new HashSet<HighwayTag>();
            HashSet<string> addedSurroundings = new HashSet<string>();
            double elevationDiff = 0;
            double maxSteepness = 0;
            List<int> visited = visitedPath.Visited;

            List<Edge> pathEdges = visitedPath.Edges;
            visitedPath.Length = pathEdges.Sum( x => x.Cost );

            double totalEdgeProfits = 0;
            int i = 0;
            foreach (Edge edge in pathEdges)
            {
                int startNodeId = visited[ i ];
                i++;
                foreach (string tag in edge.Tags)
                {
                    Utils.AddTags( ref addedSurfaceTags, ref addedPathTypes, ref addedSurroundings, tag );
                }
                Utils.CalculateElevationDiffAndSteepness( edge, ref maxSteepness, ref elevationDiff );
                //Utils.CaculateQualityValues( p, edge, startNodeId, elevationDiff, ref currentEdgeProfits, ref currentArea, ref currentQuality );
                totalEdgeProfits += !edge.Visited ? edge.Profit * edge.Cost : 0;
                edge.Visited = true;
            }
            pathEdges.ForEach( x => x.Visited = false );

            visitedPath.CoveredArea = p.GetArea( visited );

            visitedPath.TotalEdgeProfits = totalEdgeProfits;
            //p.Path.TotalEdgeProfits = pathEdges.Sum( x => x.Profit * x.Cost );
            visitedPath.Quality = p.GetQuality( visitedPath.TotalEdgeProfits, visitedPath.CoveredArea, elevationDiff / 2, maxSteepness, visitedPath.Length );
            if (visitedPath.Quality != visitedPath.Quality)
            {
                Console.WriteLine( "ahhhhh" );
            }

            // update all current path values
            visitedPath.Visited = visited;
            visitedPath.Edges = pathEdges;
            visitedPath.Elevation = elevationDiff;
            visitedPath.Steepness = maxSteepness;
            //p.Path.CoveredArea = currentArea;
            //p.Path.TotalEdgeProfits = currentEdgeProfits;
            //p.Path.Quality = currentQuality;
            visitedPath.PathTypes = string.Join( ", ", addedPathTypes );
            visitedPath.Surfaces = string.Join( ", ", addedSurfaceTags );
            visitedPath.SurroundingTags = string.Join( ", ", addedSurroundings );
        }



        public static void UpdateMetadata ( Problem p, List<Waypoint> waypointList, List<int> nodeIdsL, List<int> nodeIdsR, ref Path returnPath )
        {

            HashSet<SurfaceTag> addedSurfaceTags = new HashSet<SurfaceTag>();
            HashSet<HighwayTag> addedPathTypes = new HashSet<HighwayTag>();
            HashSet<string> addedSurroundings = new HashSet<string>();
            double elevationDiff = 0;
            double maxSteepness = 0;
            double currentEdgeProfits = 0;
            double currentArea = 0;
            double currentQuality = 0;
            List<int> visited = new List<int>();
            List<Edge> pathEdges = new List<Edge>();

            Tuple<double, double>[] boundingCoordinates = returnPath.BoundingCoordinates;
            foreach (int nodeId in nodeIdsL)
            {
                returnPath.UpdateBoundingCoordinates( ref boundingCoordinates, p.Graph.VNodes[ nodeId ] );
            }
            if (nodeIdsR != null)
            {
                foreach (int nodeId in nodeIdsR)
                {
                    returnPath.UpdateBoundingCoordinates( ref boundingCoordinates, p.Graph.VNodes[ nodeId ] );
                }
            }


            foreach (Waypoint wp in waypointList)
            {
                visited.Add( wp.NodeID );
                visited = visited.Concat( wp.Path ).ToList();
                pathEdges = pathEdges.Concat( wp.Edges ).ToList();
            }

            int i = 0;
            foreach (Edge edge in pathEdges)
            {
                int startNodeId = visited[ i ];
                i++;
                foreach (string tag in edge.Tags)
                {
                    Utils.AddTags( ref addedSurfaceTags, ref addedPathTypes, ref addedSurroundings, tag );
                }
                Utils.CalculateElevationDiffAndSteepness( edge, ref maxSteepness, ref elevationDiff );
                Utils.CaculateQualityValues( p, edge, startNodeId, pathEdges.Sum( x => x.Cost ), elevationDiff, ref currentEdgeProfits, ref maxSteepness, ref currentArea, ref currentQuality );
                pathEdges.ForEach( x => x.Visited = false );
            }


            // update all current path values
            returnPath.Visited = visited;
            returnPath.Edges = pathEdges;
            returnPath.Elevation = elevationDiff / 2;
            returnPath.Steepness = maxSteepness;
            returnPath.CoveredArea = currentArea;
            returnPath.TotalEdgeProfits = currentEdgeProfits;
            returnPath.Quality = currentQuality;
            returnPath.BoundingCoordinates = boundingCoordinates;
            returnPath.Length = pathEdges.Sum( x => x.Cost );
            returnPath.PathTypes = string.Join( ", ", addedPathTypes );
            returnPath.Surfaces = string.Join( ", ", addedSurfaceTags );
            returnPath.SurroundingTags = string.Join( ", ", addedSurroundings );
            if (currentQuality != currentQuality)
            {
                Console.WriteLine( "ahhhhh" );
            }
        }

    }
}