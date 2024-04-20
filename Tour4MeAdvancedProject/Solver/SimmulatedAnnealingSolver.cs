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
        public int NumberWaypoints { get; set; } = 10;



        public SimmulatedAnnealingSolver () { }

        public SimmulatedAnnealingSolver ( int runs, int repetitions, double temp, int waypoints )
        {
            Runs = runs;
            Repetitions = repetitions;
            Temperature = temp;
            NumberWaypoints = waypoints;
        }

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
                        if (diff > 0)
                        {
                            P.Path = neighboringSolution;
                            P.Quality = neighborQuality;
                            currentQuality = neighborQuality;
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
                                P.Path = neighboringSolution;
                                P.Quality = neighborQuality;
                                currentQuality = neighborQuality;
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
            Problem problem = new Problem( P, P.Path );

            Path currentPath = problem.Path;

            // list of the index where to find Node(Id) in the List of all visited nodes
            List<int> waypoints = new List<int>();
            List<int> nodes = P.Path.Visited;
            _ = currentPath.Edges;
            Random rnd = new Random();
            List<Edge> incident;

            // find waypoint idxs
            incident = FindWaypointIndexes( P, waypoints, nodes );

            // find new Waypoint to insert (& preedSucc coordinates)
            _ = FindNewWaypoint( P, waypoints, out Tuple<int, int> predSuccWaypointIdxs, nodes, rnd, incident, out Node newWaypoint );

            // Add / Remove / Move & use new path if quality gets better (or with temperature 

            //P.Path = AddWaypoint( waypoints, predSuccWaypointIdxs, newWaypoint, problem );
            //(P.Path, _) = RemoveWaypoint( waypoints, problem, rnd );
            currentPath = MoveWaypoint( waypoints, problem, rnd, newWaypoint );
            _ = predSuccWaypointIdxs.ToString();

            P.Path = currentPath;
            currentPath.Quality = P.GetQuality( P.GetProfit( currentPath ), P.GetArea( currentPath ) );

            return currentPath;
        }

        private static List<Edge> FindNewWaypoint ( Problem P, List<int> waypoints, out Tuple<int, int> predSuccWaypointIdxs, List<int> nodes, Random rnd, List<Edge> incident, out Node newWaypoint )
        {
            // use random value of all existing waypoints & the point's predecessor (thus start random with 1)
            List<int> availableNodes = new List<int>();
            int newWaypointIdx = 0;

            while (availableNodes.Count == 0)
            {
                newWaypointIdx = rnd.Next( 1, waypoints.Count - 1 );
                for (int nodeId = waypoints[ newWaypointIdx - 1 ] + 1; nodeId < waypoints[ newWaypointIdx ]; nodeId++)
                {
                    Node possibleWaypoint = P.Graph.VNodes[ nodes[ nodeId ] ];
                    incident = possibleWaypoint.Incident;
                    if (incident.Count( x => x != null ) > 2)
                    {
                        availableNodes.Add( nodeId );
                    }
                }
            }
            // select middlemost point as waypoint
            predSuccWaypointIdxs = Tuple.Create( waypoints[ newWaypointIdx - 1 ], waypoints[ newWaypointIdx + 1 ] );
            newWaypoint = P.Graph.VNodes[ nodes[ availableNodes[ Math.Abs( ( availableNodes.Count - 1 ) / 2 ) ] ] ];
            int i = 0;

            bool wayPointIsPred = newWaypoint.GraphNodeId == P.Graph.VNodes[ nodes[ predSuccWaypointIdxs.Item1 ] ].GraphNodeId;
            bool wayPointIsSucc = newWaypoint.GraphNodeId == P.Graph.VNodes[ nodes[ predSuccWaypointIdxs.Item2 ] ].GraphNodeId;
            while (( wayPointIsPred || wayPointIsSucc ) &&
                Math.Abs( ( availableNodes.Count - 1 ) / 2 ) + i + 1 < availableNodes.Count)
            {
                i++;
                newWaypoint = P.Graph.VNodes[ nodes[ availableNodes[ Math.Abs( ( availableNodes.Count - 1 ) / 2 ) + i ] ] ];
            }
            i = 0;
            while (wayPointIsPred || ( wayPointIsSucc && Math.Abs( ( availableNodes.Count - 1 ) / 2 ) - i > 0 ))
            {
                i++;
                newWaypoint = P.Graph.VNodes[ nodes[ availableNodes[ Math.Abs( ( availableNodes.Count - 1 ) / 2 ) - i ] ] ];
            }
            wayPointIsPred = newWaypoint.GraphNodeId == P.Graph.VNodes[ nodes[ predSuccWaypointIdxs.Item1 ] ].GraphNodeId;
            wayPointIsSucc = newWaypoint.GraphNodeId == P.Graph.VNodes[ nodes[ predSuccWaypointIdxs.Item2 ] ].GraphNodeId;

            return incident;
        }

        private List<Edge> FindWaypointIndexes ( Problem P, List<int> waypoints, List<int> nodes )
        {
            List<Edge> incident = new List<Edge>();
            int j = -1;
            int numEdges = nodes.Count - 1;

            while (P.Path.Visited.Count <= NumberWaypoints * 2)
            {
                NumberWaypoints /= 2;
            }

            for (int i = 0; i < NumberWaypoints; i++)
            {
                do
                {
                    j++;
                    Node possibleWaypoint = P.Graph.VNodes[ nodes[ j ] ];
                    incident = possibleWaypoint.Incident;
                }
                while (incident.Count( x => x != null ) < 2 && j < numEdges);
                waypoints.Add( j );

                int step = Math.Abs( ( nodes.Count - j ) / NumberWaypoints );
                if (j + 1 + step < nodes.Count)
                {
                    // make a bigger step to cover as much of the path as possible
                    j += step;
                }
            }

            return incident;
        }

        private (Path, Tuple<int, int>) RemoveWaypoint ( List<int> waypointIdxs, Problem p, Random rnd, Node newWaypoint )
        {
            int randomIndex = rnd.Next( 1, waypointIdxs.Count - 1 );
            int predIdx = waypointIdxs[ randomIndex - 1 ];
            int succIdx = waypointIdxs[ randomIndex + 1 ];

            bool predIsWaypoint = newWaypoint.GraphNodeId == p.Graph.VNodes[ p.Path.Visited[ predIdx ] ].GraphNodeId;
            bool succIsWaypoint = newWaypoint.GraphNodeId == p.Graph.VNodes[ p.Path.Visited[ succIdx ] ].GraphNodeId;

            while (predIsWaypoint || succIsWaypoint)
            {
                randomIndex = rnd.Next( 1, waypointIdxs.Count - 1 );
                predIdx = waypointIdxs[ randomIndex - 1 ];
                succIdx = waypointIdxs[ randomIndex + 1 ];

                predIsWaypoint = newWaypoint.GraphNodeId == p.Graph.VNodes[ p.Path.Visited[ predIdx ] ].GraphNodeId;
                succIsWaypoint = newWaypoint.GraphNodeId == p.Graph.VNodes[ p.Path.Visited[ succIdx ] ].GraphNodeId;

            }

            List<int> visited = p.Path.Visited;
            List<Edge> pathEdges = p.Path.Edges;

            // look at all nodes and edges between pred and succ waypoint & remove them
            for (int i = predIdx + 1; i < succIdx; i++)
            {
                visited.RemoveAt( i );
                pathEdges.RemoveAt( i );
            }

            // calculate new path to fill gap
            (List<Edge> edges, List<int> nodeIds) = p.Graph.GetShortestPath( visited[ predIdx ], visited[ succIdx ] );

            visited.InsertRange( predIdx, nodeIds );
            pathEdges.InsertRange( predIdx, edges );


            p.Path.Visited = visited;
            p.Path.Edges = pathEdges;

            return (p.Path, Tuple.Create( predIdx, succIdx ));
        }

        private Path AddWaypoint ( List<int> waypointIdxs, Tuple<int, int> predSuccWaypointIdxs, Node newWaypoint, Problem p )
        {

            int predIdx = predSuccWaypointIdxs.Item1;
            int succIdx = predSuccWaypointIdxs.Item2;

            List<int> visited = p.Path.Visited;
            List<Edge> pathEdges = p.Path.Edges;

            // look at all nodes and edges between pred and succ waypoint & remove nodes and edges to allow adding new waypoint
            for (int i = predIdx + 1; i < succIdx; i++)
            {
                visited.RemoveAt( predIdx + 1 );
                pathEdges.RemoveAt( predIdx + 1 );
            }


            // calculate new paths to fill gap while using new waypoint
            (List<Edge> edges, List<int> nodeIds) = p.Graph.GetShortestPath( visited[ predIdx ], newWaypoint.GraphNodeId );

            visited.InsertRange( predIdx, nodeIds );
            pathEdges.InsertRange( predIdx, edges );


            (edges, nodeIds) = p.Graph.GetShortestPath( newWaypoint.GraphNodeId, visited[ succIdx + nodeIds.Count() ] );

            visited.InsertRange( predIdx, nodeIds );
            pathEdges.InsertRange( predIdx, edges );

            p.Path.Visited = visited;
            p.Path.Edges = pathEdges;

            return p.Path;
        }

        private Path MoveWaypoint ( List<int> waypointIdxs, Problem p, Random rnd, Node newWaypoint )
        {
            (Path path, Tuple<int, int> predSuccWaypointIdxs) = RemoveWaypoint( waypointIdxs, p, rnd, newWaypoint );

            p.Path = path;
            return AddWaypoint( waypointIdxs, predSuccWaypointIdxs, newWaypoint, p );
        }

    }
}