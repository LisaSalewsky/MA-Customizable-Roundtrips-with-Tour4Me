using System;
using System.Collections.Generic;
using Tour4MeAdvancedProject.ObjectClasses;
using static Tour4MeAdvancedProject.Helper.EnumHelper;


namespace Tour4MeAdvancedProject.Solver
{

    public class SelectionSolver : Selection
    {
        public SelectionSolver () { }

        public override SolveStatus Solve ( Problem P )
        {
            List<bool> visited = new List<bool>();
            visited.AddRange( new bool[ P.Graph.VEdges.Count ] );

            int current = P.Start;
            P.Path.Add( null, current, 0, 0 );

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

                    double prof = !visited[ e.Id ] ? e.Profit * e.Cost : -dis;

                    if (prof > bestProfit)
                    {
                        bestProfit = e.Profit;
                        bestNeigh = neigh;
                        bestEdge = e;
                        validCandidates = true;
                        length += e.Cost;
                    }
                }

                if (validCandidates)
                {
                    P.Path.Add( bestEdge, bestNeigh, bestProfit, length );
                    visited[ bestEdge.Id ] = true;
                    length += bestEdge.Cost;
                    current = bestNeigh;
                }
            }

            Console.WriteLine( length );
            return SolveStatus.Feasible;
        }
    }
}