﻿using System;
using System.Collections.Generic;
using System.Diagnostics;
using Tour4MeAdvancedProject.ObjectClasses;
using static Tour4MeAdvancedProject.Helper.EnumHelper;


namespace Tour4MeAdvancedProject.Solver
{

    public class JoggerSolver : Selection
    {
        public JoggerSolver() { }

        public override SolveStatus Solve(Problem CurrentProblem)
        {
            double initRingSize = 5;
            double tempRingSize = 1000;

            List<Tuple<int, Path>> sRing = CurrentProblem.Graph.CalculateRing(CurrentProblem.Start, CurrentProblem.TargetDistance / 3 - initRingSize, CurrentProblem.TargetDistance / 3 + initRingSize, 100, null);

            HashSet<int> contained = new HashSet<int>();
            foreach (var pair in sRing)
            {
                contained.Add(pair.Item1);
            }

            double bestQuality = -1;

            int[] visited = new int[CurrentProblem.Graph.VEdges.Count];
            int visitedIndex = 0;

            int index = 0;

            foreach (var pair in sRing)
            {
                index++;
                int pointA = pair.Item1;
                Path pathSA = pair.Item2;

                // builds two rings (a smaller one with radius targetDist/3 - tempringSize and
                // a bigger one with radius targetDist/3 + tempringSize)
                // capped to 10 nodes
                // only returns those nodes that are not already inside sRing
                List<Tuple<int, Path>> tempRing = CurrentProblem.Graph.CalculateRing(pointA, CurrentProblem.TargetDistance / 3 - tempRingSize, CurrentProblem.TargetDistance / 3 + tempRingSize, 10, contained);

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

                            finalPath.Add(v.Reversed ? v.Edge.TargetNode : v.Edge.SourceNode);
                        }

                        foreach (DirEdge v in tPair.Item2.Edges)
                        {
                            if (visited[v.Edge.Id] != visitedIndex)
                            {
                                profit += v.Edge.Cost * v.Edge.Profit;
                                visited[v.Edge.Id] = visitedIndex;
                            }
                            area += !v.Reversed ? v.Edge.ShoelaceForward : v.Edge.ShoelaceBackward;
                            Debug.Assert(finalPath[finalPath.Count - 1] == (!v.Reversed ? v.Edge.TargetNode : v.Edge.SourceNode));
                            finalPath.Add(v.Reversed ? v.Edge.TargetNode : v.Edge.SourceNode);
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
                            Debug.Assert(finalPath[finalPath.Count - 1] == (v.Reversed ? v.Edge.TargetNode : v.Edge.SourceNode));
                            finalPath.Add(!v.Reversed ? v.Edge.TargetNode : v.Edge.SourceNode);
                        }

                        Debug.Assert(CurrentProblem.Graph.GetEdge(finalPath[finalPath.Count - 1], finalPath[0]) != null);

                        finalPath.Add(finalPath[0]);

                        double quality = CurrentProblem.GetQuality(profit, area);

                        if (quality > bestQuality)
                        {
                            bestQuality = quality;
                            CurrentProblem.Path = finalPath;
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