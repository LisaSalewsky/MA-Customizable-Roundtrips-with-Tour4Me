using System;
using System.Collections.Generic;
using System.Diagnostics;
using Tour4MeAdvancedProject.ObjectClasses;
using static Tour4MeAdvancedProject.Helper.EnumHelper;


namespace Tour4MeAdvancedProject.Solver
{

    public class JoggerSolver : Selection
    {
        public JoggerSolver() { }

        public override SolveStatus Solve(Problem P)
        {
            double initRingSize = 5;
            double tempRingSize = 1000;

            List<Tuple<int, Path>> sRing = P.Graph.Ring(P.Start, P.TargetDistance / 3 - initRingSize, P.TargetDistance / 3 + initRingSize, 100, null);

            HashSet<int> contained = new HashSet<int>();
            foreach (var pair in sRing)
            {
                contained.Add(pair.Item1);
            }

            double bestQuality = -1;

            int[] visited = new int[P.Graph.VEdges.Count];
            int visitedIndex = 0;

            int index = 0;

            foreach (var pair in sRing)
            {
                index++;
                int pointA = pair.Item1;
                Path pathSA = pair.Item2;

                List<Tuple<int, Path>> tempRing = P.Graph.Ring(pointA, P.TargetDistance / 3 - tempRingSize, P.TargetDistance / 3 + tempRingSize, 10, contained);

                foreach (var tPair in tempRing)
                {
                    int pointB = tPair.Item1;

                    bool isContained = false;
                    Path pathSB = new Path();

                    foreach (var sPair in sRing)
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

                        foreach (DirEdge v in pathSA.Edges)
                        {
                            if (visited[v.Edge.Id] != visitedIndex)
                            {
                                profit += v.Edge.Cost * v.Edge.Profit;
                                visited[v.Edge.Id] = visitedIndex;
                            }
                            area += !v.Reversed ? v.Edge.ShoelaceForward : v.Edge.ShoelaceBackward;

                            finalPath.Add(v.Reversed ? v.Edge.T : v.Edge.S);
                        }

                        foreach (DirEdge v in tPair.Item2.Edges)
                        {
                            if (visited[v.Edge.Id] != visitedIndex)
                            {
                                profit += v.Edge.Cost * v.Edge.Profit;
                                visited[v.Edge.Id] = visitedIndex;
                            }
                            area += !v.Reversed ? v.Edge.ShoelaceForward : v.Edge.ShoelaceBackward;
                            Debug.Assert(finalPath[finalPath.Count - 1] == (!v.Reversed ? v.Edge.T : v.Edge.S));
                            finalPath.Add(v.Reversed ? v.Edge.T : v.Edge.S);
                        }

                        for (int i = pathSB.Edges.Count - 1; i >= 0; i--)
                        {
                            DirEdge v = pathSB.Edges[i];

                            if (visited[v.Edge.Id] != visitedIndex)
                            {
                                profit += v.Edge.Cost * v.Edge.Profit;
                                visited[v.Edge.Id] = visitedIndex;
                            }
                            area += v.Reversed ? v.Edge.ShoelaceForward : v.Edge.ShoelaceBackward;
                            Debug.Assert(finalPath[finalPath.Count - 1] == (v.Reversed ? v.Edge.T : v.Edge.S));
                            finalPath.Add(!v.Reversed ? v.Edge.T : v.Edge.S);
                        }

                        Debug.Assert(P.Graph.GetEdge(finalPath[finalPath.Count - 1], finalPath[0]) != null);

                        finalPath.Add(finalPath[0]);

                        double quality = P.GetQuality(profit, area);

                        if (quality > bestQuality)
                        {
                            bestQuality = quality;
                            P.Path = finalPath;
                        }
                    }
                }
            }

            if (bestQuality == -1)
            {
                return SolveStatus.Unsolved;
            }

            return SolveStatus.Feasible;
        }
    }

}