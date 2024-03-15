using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Diagnostics.Eventing.Reader;
using System.Linq;
using Tour4MeAdvancedProject.ObjectClasses;
using static Tour4MeAdvancedProject.Helper.EnumHelper;


namespace Tour4MeAdvancedProject.Solver
{

    public class JoggerSolver : Selection
    {
        public JoggerSolver () { }

        public override SolveStatus Solve ( Problem CurrentProblem )
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
            HashSet<PathType> addedPathTypes = new HashSet<PathType>();

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
                    Path pathSB = new Path();

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
                        double length = 0;

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
                            for (int i = 0; i < v.Tags.Count; i++)
                            {
                                string currentTag = v.Tags[ i ];
                                if (Enum.TryParse<SurfaceTag>( currentTag, true, out SurfaceTag surfaceTag ))
                                {
                                    _ = addedSurfaceTags.Add( surfaceTag );
                                }
                                if (Enum.TryParse<PathType>( currentTag, true, out PathType pathType ))
                                {
                                    _ = addedPathTypes.Add( pathType );
                                }

                            }
                            length += v.Cost;
                        }

                        foreach (Edge v in tPair.Item2.Edges)
                        {
                            if (visited[ v.GraphId ] != visitedIndex)
                            {
                                profit += v.Cost * v.Profit;
                                visited[ v.GraphId ] = visitedIndex;
                            }
                            area += !v.Reversed ? v.ShoelaceForward : v.ShoelaceBackward;
                            Debug.Assert( finalPath[ finalPath.Count - 1 ] == ( !v.Reversed ? v.TargetNode.GraphNodeId : v.SourceNode.GraphNodeId ) );
                            finalPath.Add( v.Reversed ? v.TargetNode.GraphNodeId : v.SourceNode.GraphNodeId );
                            finalEdges.Add( v );
                            for (int i = 0; i < v.Tags.Count; i++)
                            {
                                string currentTag = v.Tags[ i ];
                                if (Enum.TryParse<SurfaceTag>( currentTag, true, out SurfaceTag surfaceTag ))
                                {
                                    _ = addedSurfaceTags.Add( surfaceTag );
                                }
                                if (Enum.TryParse<PathType>( currentTag, true, out PathType pathType ))
                                {
                                    _ = addedPathTypes.Add( pathType );
                                }

                            }
                            length += v.Cost;
                        }

                        for (int i = pathSB.Edges.Count - 1; i >= 0; i--)
                        {
                            Edge v = pathSB.Edges[ i ];

                            if (visited[ v.GraphId ] != visitedIndex)
                            {
                                profit += v.Cost * v.Profit;
                                visited[ v.GraphId ] = visitedIndex;
                            }
                            area += v.Reversed ? v.ShoelaceForward : v.ShoelaceBackward;
                            Debug.Assert( finalPath[ finalPath.Count - 1 ] == ( v.Reversed ? v.TargetNode.GraphNodeId : v.SourceNode.GraphNodeId ) );
                            finalPath.Add( !v.Reversed ? v.TargetNode.GraphNodeId : v.SourceNode.GraphNodeId );
                            finalEdges.Add( v );
                            for (int j = 0; j < v.Tags.Count; j++)
                            {
                                string currentTag = v.Tags[ j ];
                                if (Enum.TryParse<SurfaceTag>( currentTag, true, out SurfaceTag surfaceTag ))
                                {
                                    _ = addedSurfaceTags.Add( surfaceTag );
                                }
                                if (Enum.TryParse<PathType>( currentTag, true, out PathType pathType ))
                                {
                                    _ = addedPathTypes.Add( pathType );
                                }

                            }
                            length += v.Cost;
                        }

                        Debug.Assert( CurrentProblem.Graph.GetEdge( finalPath[ finalPath.Count - 1 ], finalPath[ 0 ] ) != null );

                        finalPath.Add( finalPath[ 0 ] );

                        double quality = CurrentProblem.GetQuality( profit, area );

                        if (quality > bestQuality)
                        {
                            bestQuality = quality;
                            CurrentProblem.Path = new Path( finalEdges, finalPath.ToHashSet(), quality, length );
                        }
                    }
                }
            }

            CurrentProblem.Path.PathTypes = string.Join( ", ", addedPathTypes );
            CurrentProblem.Path.Surfaces = string.Join( ", ", addedSurfaceTags );

            return bestQuality == -1 ? SolveStatus.Unsolved : SolveStatus.Feasible;
        }
    }

}