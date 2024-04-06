using System;
using System.Collections.Generic;
using Tour4MeAdvancedProject.Helper;
using Tour4MeAdvancedProject.ObjectClasses;
using static Tour4MeAdvancedProject.Helper.EnumHelper;


namespace Tour4MeAdvancedProject.Solver
{

    public class JoggerSolver : Selection
    {
        public JoggerSolver () { }

        public override SolveStatus Solve ( ref Problem CurrentProblem )
        {
            double initRingSize = 5;
            double tempRingSize = 1000;

            List<Tuple<int, Path>> sRing = CurrentProblem.Graph.CalculateRing( CurrentProblem.Start, ( CurrentProblem.TargetDistance / 3 ) - initRingSize, ( CurrentProblem.TargetDistance / 3 ) + initRingSize, 100, null );

            HashSet<int> contained = new HashSet<int>();
            foreach (Tuple<int, Path> pair in sRing)
            {
                _ = contained.Add( pair.Item1 );
            }

            double bestQuality = -1;

            int[] visited = new int[ CurrentProblem.Graph.VEdges.Count ];
            int visitedIndex = 0;

            int index = 0;

            HashSet<SurfaceTag> addedSurfaceTags = new HashSet<SurfaceTag>();
            HashSet<HighwayTag> addedPathTypes = new HashSet<HighwayTag>();
            HashSet<string> addedSurroundings = new HashSet<string>();
            double currentPathsMaxSteepness = 0;
            double currentElevationDiff = 0;
            double length = 0;

            foreach (Tuple<int, Path> pair in sRing)
            {
                index++;
                int pointA = pair.Item1;
                Path pathSA = pair.Item2;

                // builds two rings (a smaller one with radius targetDist/3 - tempringSize and
                // a bigger one with radius targetDist/3 + tempringSize)
                // capped to 10 nodes
                // only returns those nodes that are not already inside sRing
                List<Tuple<int, Path>> tempRing = CurrentProblem.Graph.CalculateRing( pointA, ( CurrentProblem.TargetDistance / 3 ) - tempRingSize, ( CurrentProblem.TargetDistance / 3 ) + tempRingSize, 10, contained );

                foreach (Tuple<int, Path> tPair in tempRing)
                {
                    int pointB = tPair.Item1;

                    bool isContained = false;
                    Path pathSB = new Path( Tuple.Create( CurrentProblem.Graph.CenterLat, CurrentProblem.Graph.CenterLon ) );

                    foreach (Tuple<int, Path> sPair in sRing)
                    {
                        if (sPair.Item1 == pointB)
                        {
                            isContained = true;
                            pathSB = sPair.Item2;
                            break;
                        }
                    }

                    if (isContained)
                    {
                        visitedIndex++;
                        double profit = 0;
                        double area = 0;
                        List<int> finalPath = new List<int>();
                        List<Edge> finalEdges = new List<Edge>();

                        foreach (Edge v in pathSA.Edges)
                        {
                            if (visited[ v.GraphId ] != visitedIndex)
                            {
                                profit += v.Cost * v.Profit;
                                visited[ v.GraphId ] = visitedIndex;
                            }
                            area += !v.Reversed ? v.ShoelaceForward : v.ShoelaceBackward;

                            finalPath.Add( v.Reversed ? v.TargetNode.GraphNodeId : v.SourceNode.GraphNodeId );
                            finalEdges.Add( v );
                        }

                        foreach (Edge v in tPair.Item2.Edges)
                        {
                            if (visited[ v.GraphId ] != visitedIndex)
                            {
                                profit += v.Cost * v.Profit;
                                visited[ v.GraphId ] = visitedIndex;
                            }
                            area += !v.Reversed ? v.ShoelaceForward : v.ShoelaceBackward;
                            //Debug.Assert( finalPath[ finalPath.Count - 1 ] == ( !v.Reversed ? v.TargetNode.GraphNodeId : v.SourceNode.GraphNodeId ) );
                            finalPath.Add( v.Reversed ? v.TargetNode.GraphNodeId : v.SourceNode.GraphNodeId );
                            finalEdges.Add( v );
                        }

                        for (int i = pathSB.Edges.Count - 1; i >= 0; i--)
                        {
                            Edge v = pathSB.Edges[ i ];

                            if (visited[ v.GraphId ] != visitedIndex)
                            {
                                profit += v.Cost * v.Profit;
                                visited[ v.GraphId ] = visitedIndex;
                            }
                            area += !v.Reversed ? v.ShoelaceForward : v.ShoelaceBackward;
                            //Debug.Assert( finalPath[ finalPath.Count - 1 ] == ( v.Reversed ? v.TargetNode.GraphNodeId : v.SourceNode.GraphNodeId ) );
                            finalPath.Add( v.Reversed ? v.TargetNode.GraphNodeId : v.SourceNode.GraphNodeId );
                            finalEdges.Add( v );
                        }

                        //Debug.Assert( CurrentProblem.Graph.GetEdge( finalPath[ finalPath.Count - 1 ], finalPath[ 0 ] ) != null );

                        finalPath.Add( finalPath[ 0 ] );

                        double quality = CurrentProblem.GetQuality( profit, area );

                        if (quality > bestQuality)
                        {
                            bestQuality = quality;
                            CurrentProblem.Path = new Path( finalEdges, finalPath, quality, length, Tuple.Create( CurrentProblem.Graph.CenterLat, CurrentProblem.Graph.CenterLon ) );
                        }
                        CurrentProblem.Path.CoveredArea = area;
                    }
                }
            }

            //_ = Parallel.ForEach( CurrentProblem.Path.Edges, e =>
            foreach (Edge e in CurrentProblem.Path.Edges)
            {

                for (int j = 0; j < e.Tags.Count; j++)
                {
                    string currentTag = e.Tags[ j ];
                    Utils.AddTags( ref addedSurfaceTags, ref addedPathTypes, ref addedSurroundings, currentTag );
                }
                Utils.CalculateElevationDiffAndSteepness( e, ref currentPathsMaxSteepness, ref currentElevationDiff );
                length += e.Cost;
            }
            //);

            CurrentProblem.Path.Steepness = currentPathsMaxSteepness / 2;
            CurrentProblem.Path.Elevation = currentElevationDiff;

            CurrentProblem.Path.PathTypes = string.Join( ", ", addedPathTypes );
            CurrentProblem.Path.Surfaces = string.Join( ", ", addedSurfaceTags );
            CurrentProblem.Path.SurroundingTags = string.Join( ", ", addedSurroundings );

            return bestQuality == -1 ? SolveStatus.Unsolved : SolveStatus.Feasible;
        }

    }

}