using System;
using System.Collections.Generic;
using Tour4MeAdvancedProject.Helper;
using Tour4MeAdvancedProject.ObjectClasses;
using static Tour4MeAdvancedProject.Helper.EnumHelper;


namespace Tour4MeAdvancedProject.Solver
{

    public class SelectionSolver : Selection
    {
        public SelectionSolver () { }

        public override SolveStatus Solve ( ref Problem P )
        {
            List<bool> visited = new List<bool>();
            visited.AddRange( new bool[ P.Graph.VEdges.Count ] );

            int current = P.Start;
            P.Path.Add( null, current, 0 );

            HashSet<SurfaceTag> addedSurfaceTags = new HashSet<SurfaceTag>();
            HashSet<HighwayTag> addedPathTypes = new HashSet<HighwayTag>();
            HashSet<string> addedSurroundings = new HashSet<string>();
            double currentPathsMaxSteepness = 0;
            double currentElevationDiff = 0;


            //Path endPath = new Path();
            double length = 0;
            bool validCandidates = true;

            while (validCandidates)
            {
                validCandidates = false;
                Edge bestEdge = null;
                int bestNeigh = 0;
                double bestProfit = -9999999999;

                foreach (Edge e in P.Graph.VNodes[ current ].Incident)
                {
                    int neigh = e.SourceNode.GraphNodeId == current ? e.TargetNode.GraphNodeId : e.SourceNode.GraphNodeId;
                    double dis = P.Graph.ShortestPath( P.Start, neigh );

                    if (dis > P.TargetDistance - length - e.Cost)
                    {
                        continue;
                    }

                    double prof = !visited[ e.GraphId ] ? e.Profit * e.Cost : -dis;

                    if (prof > bestProfit)
                    {
                        bestProfit = prof;
                        bestNeigh = neigh;
                        bestEdge = e;
                        validCandidates = true;
                    }
                }
                bestProfit = bestEdge != null ? bestEdge.Profit * bestEdge.Cost : bestProfit;

                if (validCandidates)
                {
                    P.Path.Add( bestEdge, bestNeigh, bestProfit );
                    foreach (string currentTag in bestEdge.Tags)
                    {
                        Utils.AddTags( ref addedSurfaceTags, ref addedPathTypes, ref addedSurroundings, currentTag );
                    }
                    Utils.CalculateElevationDiffAndSteepness( bestEdge, ref currentPathsMaxSteepness, ref currentElevationDiff );

                    visited[ bestEdge.GraphId ] = true;
                    length += bestEdge.Cost;
                    current = bestNeigh;
                }
            }

            Console.WriteLine( length );
            P.Path.Steepness = currentPathsMaxSteepness / 2;
            P.Path.Elevation = currentElevationDiff;

            P.Path.PathTypes = string.Join( ", ", addedPathTypes );
            P.Path.Surfaces = string.Join( ", ", addedSurfaceTags );
            P.Path.SurroundingTags = string.Join( ", ", addedSurroundings );

            return SolveStatus.Feasible;
        }
    }
}