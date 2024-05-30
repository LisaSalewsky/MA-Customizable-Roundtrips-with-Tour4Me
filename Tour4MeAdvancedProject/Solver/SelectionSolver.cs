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
            visited.AddRange( new bool[ P.Graph.VEdges.Length ] );

            int current = P.Start;
            P.Path.Add( null, current, 0 );

            HashSet<SurfaceTag> addedSurfaceTags = new HashSet<SurfaceTag>();
            HashSet<HighwayTag> addedPathTypes = new HashSet<HighwayTag>();
            HashSet<string> addedSurroundings = new HashSet<string>();
            double currentPathsMaxSteepness = 0;
            double currentElevationDiff = 0;
            double currentEdgeProfits = 0;
            double currentArea = 0;
            double currentQuality = 0;

            Tuple<double, double> startCoordinates = Tuple.Create( P.Graph.VNodes[ P.Start ].Lat, P.Graph.VNodes[ P.Start ].Lon );
            Tuple<double, double>[] boudingCoordinates = new Tuple<double, double>[] {
            startCoordinates , startCoordinates , startCoordinates , startCoordinates  };


            //Path endPath = new Path();
            double length = 0;
            bool validCandidates = true;

            while (validCandidates)
            {
                validCandidates = false;
                Edge bestEdge = null;
                int bestNeigh = -1;
                int startNodeId = -1;
                double bestProfit = -9999999999;

                foreach (Edge e in P.Graph.VNodes[ current ].Incident)
                {
                    int neigh = e.SourceNode.GraphNodeId == current ? e.TargetNode.GraphNodeId : e.SourceNode.GraphNodeId;
                    startNodeId = e.SourceNode.GraphNodeId == current ? e.SourceNode.GraphNodeId : e.TargetNode.GraphNodeId;
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
                    P.Path.UpdateBoundingCoordinates( ref boudingCoordinates, P.Graph.VNodes[ bestNeigh ] );
                    P.Path.BoundingCoordinates = boudingCoordinates;
                    visited[ bestEdge.GraphId ] = true;
                    length += bestEdge.Cost;
                    current = bestNeigh;
                    foreach (string currentTag in bestEdge.Tags)
                    {
                        Utils.AddTags( ref addedSurfaceTags, ref addedPathTypes, ref addedSurroundings, currentTag );
                    }
                    Utils.CalculateElevationDiffAndSteepness( bestEdge, ref currentPathsMaxSteepness, ref currentElevationDiff );
                    Utils.CaculateQualityValues( P, bestEdge, startNodeId, length, currentElevationDiff, ref currentEdgeProfits, ref currentArea, ref currentQuality );

                }
            }

            P.Path.Edges.ForEach( e => e.Visited = false );
            //Utils.UpdateCurrentProblemPathMetadata( ref P, addedSurfaceTags, addedPathTypes, addedSurroundings, currentEdgeProfits, currentArea, currentQuality, currentPathsMaxSteepness, currentElevationDiff, boudingCoordinates );
            Utils.UpdateMetadata( P.Path, P );

            return SolveStatus.Feasible;
        }
    }
}