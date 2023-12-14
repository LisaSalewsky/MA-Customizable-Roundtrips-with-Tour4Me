using System;
using System.Collections.Generic;
using System.Diagnostics;
using Tour4MeAdvancedProject.ObjectClasses;
using static Tour4MeAdvancedProject.Helper.EnumHelper;


namespace Tour4MeAdvancedProject.Solver
{

    public class SelectionSolver : Selection
    {
        public SelectionSolver() { }

        public override SolveStatus Solve(Problem P)
        {
            List<bool> visited = new List<bool>();
            visited.AddRange(new bool[P.Graph.VEdges.Count]);

            int current = P.Start;
            P.Path.Add(current);

            //Path endPath = new Path();
            double length = 0;
            bool validCandidates = true;

            while (validCandidates)
            {
                validCandidates = false;
                Edge bestEdge = null;
                int bestNeigh = 0;
                double bestProfit = -9999999999;

                foreach (Edge e in P.Graph.VNodes[current].Incident)
                {
                    int neigh = e.SourceNode == current ? e.TargetNode : e.SourceNode;
                    double dis = P.Graph.ShortestPath(P.Start, neigh);

                    if (dis > P.TargetDistance - length - e.Cost)
                    {
                        continue;
                    }

                    double prof = !visited[e.Id] ? e.Profit * e.Cost : -dis;

                    if (prof > bestProfit)
                    {
                        bestProfit = e.Profit;
                        bestNeigh = neigh;
                        bestEdge = e;
                        validCandidates = true;
                    }
                }

                if (validCandidates)
                {
                    P.Path.Add(bestNeigh);
                    visited[bestEdge.Id] = true;
                    length += bestEdge.Cost;
                    current = bestNeigh;
                }
            }

            Console.WriteLine(length);
            return SolveStatus.Feasible;
        }
    }
}