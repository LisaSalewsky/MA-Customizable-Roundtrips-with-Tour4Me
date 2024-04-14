using System;
using System.Collections.Generic;
using System.Linq;
using Tour4MeAdvancedProject.ObjectClasses;
using static Tour4MeAdvancedProject.Helper.EnumHelper;

namespace Tour4MeAdvancedProject.Solver
{
    public class SimmulatedAnnealingSolver : Selection
    {
        public Algo Algo { get; set; } = Algo.minCost;
        // runs with changing temperature
        public int Runs { get; set; } = 5;
        // repetitions at each temperature
        public int Repetitions { get; set; } = 5;
        public double Temperature { get; set; } = 0.9;
        public Random Random { get; set; } = new Random();



        public SimmulatedAnnealingSolver () { }

        public override SolveStatus Solve ( ref Problem P )
        {
            SolveStatus status = SolveStatus.Unsolved;
            switch (Algo)
            {
                case Algo.Greedy:
                    {
                        // Greedy
                        SelectionSolver solver = new SelectionSolver();
                        status = solver.Solve( ref P );
                        break;
                    }
                case Algo.minCost:
                    {
                        // minCost
                        JoggerSolver solver = new JoggerSolver();
                        status = solver.Solve( ref P );
                        break;
                    }
                case Algo.ILS:
                    {
                        // ILS
                        //ILS solver = new ILS();
                        //status = solver.Solve( ref problem);
                        break;
                    }
                case Algo.AntColony:
                    {
                        // Ant
                        AntSolver solver = new AntSolver();
                        status = solver.Solve( ref P );
                        break;
                    }
                case Algo.AntMinCost:
                    {
                        // Ant Combined
                        AntCombined solver = new AntCombined();
                        status = solver.Solve( ref P, Algo.minCost );
                        break;
                    }
                case Algo.AntGreedy:
                    {
                        // Ant Combined
                        AntCombined solver = new AntCombined();
                        status = solver.Solve( ref P, Algo.Greedy );
                        break;
                    }

            }

            if (status == SolveStatus.Feasible || status == SolveStatus.Optimal)
            {
                double currentQuality = P.Path.Quality;
                double initProb = 0.9;
                double maxTemperature = Temperature;
                for (int i = 0; i < Runs; i++)
                {
                    Problem currentProblem = P;
                    double diff = 0;
                    for (int j = 0; j < Repetitions; j++)
                    {
                        Path neighboringSolution = GenerateNeighborSolution( currentProblem );

                        double neighborQuality = neighboringSolution.Quality;

                        diff = neighborQuality - currentQuality;

                        // if the quality of the initial solution was worse than of the new one, always pick the new one
                        if (diff <= 0)
                        {
                            P = currentProblem;
                        }
                        // if the quality of the new solution was worse than of the initial one, pick the new one with a chance
                        else
                        {
                            // calculate probability of choosing the new ( worse ) soution
                            double probs = Math.Exp( -diff / Temperature );
                            // generate a rondom value between 0 and 1
                            double rand = Random.NextDouble();

                            // if the random value exceeds the probability, choose the new ( worse ) solution
                            if (rand > probs)
                            {
                                P = currentProblem;
                            }
                        }
                    }
                    // change temperature according to difference in quality and normalize by maxTemperetature
                    Temperature = ( -diff ) / Math.Log( initProb ) / maxTemperature;
                }
            }

            return SolveStatus.Feasible;
        }

        private Path GenerateNeighborSolution ( Problem P )
        {
            Path currentPath = new Path( P.Path );

            List<int> nodes = currentPath.Visited;

            Random rnd = new Random();
            List<Edge> incident;
            Node nodeToMove;
            int randomIndex;
            double bestQuality = P.Path.Quality;
            do
            {
                randomIndex = rnd.Next( 1, currentPath.Edges.Count - 1 );
                nodes = currentPath.Visited;
                nodeToMove = P.Graph.VNodes[ nodes[ randomIndex ] ];
                incident = nodeToMove.Incident;
            }
            while (incident.Count( x => x != null ) < 2);
            int nodeInsertCounter = randomIndex;
            int edgeInsertCounter = randomIndex;

            foreach (Edge edge in nodeToMove.Incident)
            {
                if (P.Graph.VEdges[ edge.GraphId ] != null)
                {
                    Node prev = P.Graph.VNodes[ nodes[ randomIndex - 1 ] ];
                    Node succ = P.Graph.VNodes[ nodes[ randomIndex + 1 ] ];
                    if (( edge.SourceNode == nodeToMove && ( edge.TargetNode == prev ) ) ||
                        ( edge.TargetNode == nodeToMove && ( edge.SourceNode == prev ) ))
                    {
                        _ = currentPath.Edges.RemoveAll( x => x.GraphId == edge.GraphId );
                        edgeInsertCounter--;
                    }
                    if (( edge.SourceNode == nodeToMove && ( edge.TargetNode == succ ) ) ||
                        ( edge.TargetNode == nodeToMove && ( edge.SourceNode == succ ) ))
                    {
                        _ = currentPath.Edges.RemoveAll( x => x.GraphId == edge.GraphId );
                    }
                }
            }
            currentPath.Visited.RemoveAt( randomIndex );
            int backupCounter = edgeInsertCounter;
            HashSet<int> visitedNodes = nodes.ToHashSet();

            foreach (Edge edge in incident)
            {
                Node neighbor = edge.SourceNode == nodeToMove ? edge.TargetNode : edge.SourceNode;
                edgeInsertCounter = backupCounter;

                if (!visitedNodes.Contains( neighbor.GraphNodeId ))
                {
                    (List<Edge> resultEdges, List<int> resultNodes) = P.Graph.GetShortestPath( neighbor.GraphNodeId, nodes[ randomIndex - 1 ] );
                    //double shortestPath = P.Graph.ShortestPath( neighbor.GraphNodeId, nodes[ randomIndex + 1 ] );

                    List<bool> visited = new List<bool>();
                    visited.AddRange( new bool[ P.Graph.VEdges.Count ] );

                    int current = neighbor.GraphNodeId;

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
                            double dis = P.Graph.ShortestPath( nodes[ randomIndex ], neigh );

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
                            currentPath.Insert( bestEdge, edgeInsertCounter, bestNeigh, bestProfit );
                            edgeInsertCounter++;
                            resultEdges.Add( bestEdge );
                            resultNodes.Add( current );
                            visited[ bestEdge.GraphId ] = true;
                            length += bestEdge.Cost;
                            current = bestNeigh;
                        }
                    }
                    int count = nodeInsertCounter;
                    foreach (int node in resultNodes)
                    {
                        currentPath.Visited.Insert( count, node );
                        count++;
                        nodeInsertCounter++;
                    }
                    count = Math.Max( 0, edgeInsertCounter + 1 - resultEdges.Count );
                    foreach (Edge edgeToInsert in resultEdges)
                    {
                        currentPath.Edges.Insert( count, edgeToInsert );
                        count++;
                    }

                }

                double quality = P.GetQuality( P.GetProfit( currentPath ), P.GetArea( currentPath ) );
                if (quality > bestQuality)
                {
                    P.Path = currentPath;
                }
            }

            return P.Path;
        }
    }
}