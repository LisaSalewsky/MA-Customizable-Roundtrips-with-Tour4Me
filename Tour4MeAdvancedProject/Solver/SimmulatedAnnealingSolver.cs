using NetTopologySuite.Geometries;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Web.UI.WebControls;
using Tour4MeAdvancedProject.Helper;
using Tour4MeAdvancedProject.ObjectClasses;
using static Tour4MeAdvancedProject.Helper.EnumHelper;

namespace Tour4MeAdvancedProject.Solver
{
    public class SimmulatedAnnealingSolver : Selection
    {
        public Algo Algo { get; set; } = Algo.Greedy;
        // runs with changing temperature
        public int Runs { get; set; } = 50;
        // repetitions at each temperature
        public int Repetitions { get; set; } = 5;
        public double Temperature { get; set; } = 0.9;
        public Random Random { get; set; } = new Random();
        public int NumberwaypointList { get; set; } = 10;



        public SimmulatedAnnealingSolver () { }

        public SimmulatedAnnealingSolver ( int runs, int repetitions, double temp, int waypointList )
        {
            Runs = runs;
            Repetitions = repetitions;
            Temperature = temp;
            NumberwaypointList = waypointList;
        }

        public SolveStatus Solve ( ref Problem P, Algo algo )
        {
            Algo = algo;

            return Solve( ref P );
        }

        public override SolveStatus Solve ( ref Problem P )
        {
            bool useDatastructure = false;
            bool calculatePointsOfInterest = false;



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
                case Algo.SimulatedAnnealingEmpty:
                case Algo.SimulatedAnnealingFullyRandom:
                default:
                    {
                        Path emptyPath = new Path( Tuple.Create( P.CenterLat, P.CenterLon ) );
                        emptyPath.Visited.Add( P.Start );

                        P.Path = emptyPath;
                        status = SolveStatus.Feasible;
                        break;
                    }

            }

            HashSet<SurfaceTag> addedSurfaceTags = new HashSet<SurfaceTag>();
            HashSet<HighwayTag> addedPathTypes = new HashSet<HighwayTag>();
            HashSet<string> addedSurroundings = new HashSet<string>();

            foreach (Edge edge in P.Path.Edges)
            {
                foreach (string currentTag in edge.Tags)
                {
                    Utils.AddTags( ref addedSurfaceTags, ref addedPathTypes, ref addedSurroundings, currentTag );
                }
            }

            double currentQuality = 0;
            double length = P.Path.Length;
            double currentArea = P.Path.CoveredArea;
            double currentEdgeProfits = P.Path.TotalEdgeProfits;
            double currentPathsMaxSteepness = P.Path.Steepness;
            double currentElevationDiff = P.Path.Elevation;
            Tuple<double, double>[] boudingCoordinates = P.Path.BoundingCoordinates;

            Algo = Algo.SimulatedAnnealingFullyRandom;

            if (status == SolveStatus.Feasible || status == SolveStatus.Optimal)
            {
                #region prepare needed values


                P.Graph.InitializeShortestPath( P.Start );

                //NumberwaypointList = P.Path.Visited.Count() / 10;
                currentQuality = P.Path.Quality;


                Problem problem = new Problem( P, P.Path );
                // dict of the index where to find Node(Id) in the List of all visited nodes
                // and a tuple containing the nodeId and the list of nodes to the next waypoint
                List<Waypoint> waypointList = new List<Waypoint>();

                Random rnd = new Random();

                // find waypoint idxs
                waypointList = FindWaypointIndexes( P, waypointList );

                double lonL = boudingCoordinates[ 0 ].Item2;
                double latB = boudingCoordinates[ 1 ].Item1;
                double latT = boudingCoordinates[ 2 ].Item1;
                double lonR = boudingCoordinates[ 3 ].Item2;


                double targetDist = P.TargetDistance;
                Coordinate pathMiddle = new Coordinate( ( latT + latB ) / 2, ( lonR + lonL ) / 2 );
                List<Node> graphNodes = P.Graph.VNodes.Where( x => x != null && x.ShortestDistance < problem.TargetDistance ).ToList();


                // Query the tree for nearest neighbors
                Node closestNode = graphNodes
                                    .OrderBy( node => pathMiddle.Distance( new Coordinate( node.Lat, node.Lon ) ) )
                                    .FirstOrDefault();
                HashSet<Node> availableNodes = graphNodes.Where( x => x != null &&
                                                                !waypointList.Any( y => y.NodeID == x.GraphNodeId ) ).ToHashSet();
                List<Tuple<double, int>> probabilityList;

                if (Algo == Algo.SimulatedAnnealingFullyRandom)
                {
                    probabilityList = CalculateFullyRandomDistribution( closestNode, P, availableNodes );
                }
                else
                {
                    // calculate cummulative Probabilities for selecting the index
                    probabilityList = CalculateProbabilityDistribution( closestNode, P, availableNodes, calculatePointsOfInterest );
                }
                Dictionary<int, double> cumulativeProbabilities = new Dictionary<int, double>
                {
                    { probabilityList[ 0 ].Item2, probabilityList[ 0 ].Item1 }
                };
                double prevItem = cumulativeProbabilities.FirstOrDefault().Value;
                for (int i = 1; i < probabilityList.Count; i++)
                {
                    if (probabilityList[ i ].Item1 > 0)
                    {
                        prevItem += probabilityList[ i ].Item1;
                        cumulativeProbabilities.Add( i, prevItem );
                    }
                    //cumulativeProbabilities[ i ] = cumulativeProbabilities[ i - 1 ] + probabilityList[ i ].Item1;
                }
                List<Waypoint> waypointListBackup = new List<Waypoint>( waypointList );
                Dictionary<int, double> cumulativeProbabilitiesBackup = new Dictionary<int, double>();
                bool alreadyRecalculated = false;

                bool pathChanged = false;
                bool recalculateProbs = false;

                Dictionary<int, Dictionary<int, double>> waypointAllDistancesForNodeIdDict = new Dictionary<int, Dictionary<int, double>>();


                Node[] viableNodes = availableNodes.Where( x => x != null && probabilityList[ x.GraphNodeId ].Item1 > 0 ).ToArray();

                if (useDatastructure)
                {
                    waypointAllDistancesForNodeIdDict.Add( waypointList[ 0 ].NodeID, new Dictionary<int, double>() );
                    foreach (Node viableNode in viableNodes)
                    {
                        waypointAllDistancesForNodeIdDict[ waypointList[ 0 ].NodeID ].Add( viableNode.GraphNodeId, viableNode.ShortestDistance );
                    }
                }


                Tuple<string, int> waypointChangeAndIdx = Tuple.Create( "", -1 );
                #endregion

                // do the simulated annealing runs

                for (int i = 0; i < Runs; i++)
                {
                    double diff = 0;
                    for (int j = 0; j < Repetitions; j++)
                    {

                        Path neighboringSolution = GenerateNeighborSolution( problem, useDatastructure, ref waypointList, ref cumulativeProbabilities, ref availableNodes, rnd, waypointAllDistancesForNodeIdDict, out recalculateProbs, ref waypointChangeAndIdx, out int currentWaypointId ); // , out alreadyRecalculated );

                        double neighborQuality = neighboringSolution.Quality;

                        diff = neighborQuality - currentQuality;

                        // if the quality of the initial solution was worse than of the new one, always pick the new one
                        if (diff > 0)
                        {
                            problem.Path = new Path( neighboringSolution );
                            problem.Quality = neighborQuality;
                            currentQuality = neighborQuality;
                            waypointListBackup = new List<Waypoint>();
                            foreach (Waypoint wp in waypointList)
                            {
                                waypointListBackup.Add( (Waypoint)wp.Clone() );
                            }

                            cumulativeProbabilitiesBackup = new Dictionary<int, double>( cumulativeProbabilities );
                            pathChanged = true;

                            viableNodes = availableNodes.Where( x => x != null && probabilityList[ x.GraphNodeId ].Item1 > 0 ).ToArray();
                            if (useDatastructure)
                            {
                                UpdateDistances( problem, ref waypointAllDistancesForNodeIdDict, waypointChangeAndIdx, viableNodes, currentWaypointId );
                            }
                        }
                        // if the quality of the new solution was worse than of the initial one, pick the new one with a chance
                        else
                        {
                            // calculate probability of choosing the new ( worse ) soution
                            double probs = Math.Exp( diff / Temperature );
                            // generate a rondom value between 0 and 1
                            double rand = Random.NextDouble();

                            // if the random value exceeds the probability, choose the new ( worse ) solution
                            if (rand < probs)
                            {
                                problem.Path = new Path( neighboringSolution );
                                problem.Quality = neighborQuality;
                                currentQuality = neighborQuality;
                                waypointListBackup = new List<Waypoint>();
                                foreach (Waypoint wp in waypointList)
                                {
                                    waypointListBackup.Add( (Waypoint)wp.Clone() );
                                }

                                viableNodes = availableNodes.Where( x => x != null && probabilityList[ x.GraphNodeId ].Item1 > 0 ).ToArray();
                                if (useDatastructure)
                                {
                                    UpdateDistances( problem, ref waypointAllDistancesForNodeIdDict, waypointChangeAndIdx, viableNodes, currentWaypointId );
                                }

                                cumulativeProbabilitiesBackup = new Dictionary<int, double>( cumulativeProbabilities );
                                pathChanged = true;
                            }
                        }
                        if (!pathChanged)
                        {
                            List<Waypoint> old = new List<Waypoint>( waypointList );
                            waypointList = new List<Waypoint>();
                            foreach (Waypoint wp in waypointListBackup)
                            {
                                waypointList.Add( (Waypoint)wp.Clone() );
                            }
                            cumulativeProbabilitiesBackup = new Dictionary<int, double>( cumulativeProbabilities );

                        }
                        else if (problem.Path.Length > problem.TargetDistance + 1000)
                        {
                            pathMiddle = new Coordinate( ( latT + latB ) / 2, ( lonR + lonL ) / 2 );
                            closestNode = graphNodes
                                    .OrderBy( node => pathMiddle.Distance( new Coordinate( node.Lat, node.Lon ) ) )
                                    .FirstOrDefault();
                            if (Algo == Algo.SimulatedAnnealingFullyRandom)
                            {
                                probabilityList = CalculateFullyRandomDistribution( closestNode, P, availableNodes );
                            }
                            else
                            {
                                // calculate cummulative Probabilities for selecting the index
                                probabilityList = CalculateProbabilityDistribution( closestNode, P, availableNodes, calculatePointsOfInterest );
                            }

                            // calculate cummulative Probabilities for selecting the index
                            cumulativeProbabilities = new Dictionary<int, double>
                            {
                                [ 0 ] = probabilityList[ 0 ].Item1
                            };
                            for (int l = 1; l < probabilityList.Count; l++)
                            {
                                cumulativeProbabilities[ l ] = cumulativeProbabilities[ l - 1 ] + probabilityList[ l ].Item1;
                            }
                            cumulativeProbabilitiesBackup = new Dictionary<int, double>( cumulativeProbabilities );
                            alreadyRecalculated = true;
                        }
                        pathChanged = false;
                    }
                    // change temperature according to difference in quality and normalize by maxTemperetature
                    Temperature *= Math.Exp( -2 );


                    if (recalculateProbs && !alreadyRecalculated)
                    {
                        pathMiddle = new Coordinate( ( latT + latB ) / 2, ( lonR + lonL ) / 2 );
                        closestNode = graphNodes
                                .OrderBy( node => pathMiddle.Distance( new Coordinate( node.Lat, node.Lon ) ) )
                                .FirstOrDefault();
                        if (Algo == Algo.SimulatedAnnealingFullyRandom)
                        {
                            probabilityList = CalculateFullyRandomDistribution( closestNode, P, availableNodes );
                        }
                        else
                        {
                            // calculate cummulative Probabilities for selecting the index
                            probabilityList = CalculateProbabilityDistribution( closestNode, P, availableNodes, calculatePointsOfInterest );
                        }

                        // calculate cummulative Probabilities for selecting the index

                        cumulativeProbabilities = new Dictionary<int, double>
                        {
                            { probabilityList[ 0 ].Item2, probabilityList[ 0 ].Item1 }
                        };
                        prevItem = cumulativeProbabilities.FirstOrDefault().Value;
                        for (int l = 1; l < probabilityList.Count; l++)
                        {
                            if (probabilityList[ l ].Item1 > 0)
                            {
                                prevItem += probabilityList[ l ].Item1;
                                cumulativeProbabilities.Add( l, prevItem );
                            }
                            //cumulativeProbabilities[ i ] = cumulativeProbabilities[ i - 1 ] + probabilityList[ i ].Item1;
                        }
                        cumulativeProbabilitiesBackup = new Dictionary<int, double>( cumulativeProbabilities );
                        alreadyRecalculated = true;
                    }
                }

                P = problem;
            }

            length = P.Path.Length;
            currentArea = P.Path.CoveredArea;
            currentEdgeProfits = P.Path.TotalEdgeProfits;
            currentPathsMaxSteepness = P.Path.Steepness;
            currentElevationDiff = P.Path.Elevation;
            boudingCoordinates = P.Path.BoundingCoordinates;
            //Utils.UpdateCurrentProblemPathMetadata( ref P, addedSurfaceTags, addedPathTypes, addedSurroundings, currentEdgeProfits, currentArea, currentQuality, currentPathsMaxSteepness, currentElevationDiff, boudingCoordinates );

            Utils.UpdateMetadata( P.Path, P );

            return SolveStatus.Feasible;
        }

        private static void UpdateDistances ( Problem problem, ref Dictionary<int, Dictionary<int, double>> waypointAllDistancesForNodeIdDict, Tuple<string, int> waypointChangeAndIdx, Node[] viableNodes, int newWaypointId )
        {
            if (waypointChangeAndIdx.Item1 == "add")
            {
                int wpToAdd = newWaypointId;
                if (!waypointAllDistancesForNodeIdDict.ContainsKey( wpToAdd ))
                {
                    waypointAllDistancesForNodeIdDict.Add( wpToAdd, new Dictionary<int, double>() );
                }
                foreach (Node current in viableNodes)
                {
                    int nodeId = current.GraphNodeId;
                    int distance = (int)problem.Graph.ShortestPath( wpToAdd, nodeId );
                    if (!waypointAllDistancesForNodeIdDict[ wpToAdd ].ContainsKey( nodeId ))
                    {
                        waypointAllDistancesForNodeIdDict[ wpToAdd ].Add( nodeId, distance );
                    }
                    waypointAllDistancesForNodeIdDict[ wpToAdd ][ nodeId ] = distance;
                }

            }
            else if (waypointChangeAndIdx.Item1 == "move")
            {
                int wpToMove = newWaypointId;

                _ = waypointAllDistancesForNodeIdDict.Remove( waypointChangeAndIdx.Item2 );
                if (!waypointAllDistancesForNodeIdDict.ContainsKey( wpToMove ))
                {
                    waypointAllDistancesForNodeIdDict.Add( wpToMove, new Dictionary<int, double>() );
                }

                foreach (Node current in viableNodes)
                {
                    int nodeId = current.GraphNodeId;
                    if (!waypointAllDistancesForNodeIdDict[ wpToMove ].ContainsKey( nodeId ))
                    {
                        waypointAllDistancesForNodeIdDict[ wpToMove ] = new Dictionary<int, double>();
                    }
                    int distance = (int)problem.Graph.ShortestPath( wpToMove, nodeId );
                    waypointAllDistancesForNodeIdDict[ wpToMove ][ nodeId ] = distance;
                }
            }
            else
            {
                _ = waypointAllDistancesForNodeIdDict.Remove( waypointChangeAndIdx.Item2 );
            }
        }

        private List<Tuple<double, int>> CalculateFullyRandomDistribution ( Node pathMiddle, Problem p, HashSet<Node> availableNodes )
        {

            // list containing a probability and the nodeID of the node this probability belongs to
            List<Tuple<double, int>> probNodeIdList = new List<Tuple<double, int>>();
            bool startedWithEmptyPath = p.Path.Visited.Count == 1;
            // calculate average distance of path nodes to the middle
            double avgDist = p.TargetDistance / 4;
            if (startedWithEmptyPath)
            {
                avgDist = 2 * p.TargetDistance / 3;
            }

            // calculate distance of every node from the middle of the path
            double[] dist = new double[ p.Graph.VNodes.Count ];
            double[] actDist = new double[ p.Graph.VNodes.Count ];
            int start = pathMiddle.GraphNodeId;
            PriorityQueue<Tuple<int, double>> queue = new PriorityQueue<Tuple<int, double>>();

            CaculateDistances( p, availableNodes, dist, actDist, start, queue );

            // all nodes have the same probability
            for (int i = 0; i < dist.Length; i++)
            {
                if (dist[ i ] > 0)
                {
                    dist[ i ] = startedWithEmptyPath
                        ? ( dist[ i ] > avgDist * 2 || p.Graph.VNodes[ i ] == null || dist[ i ] < avgDist / 2 ) ? 0 : 1
                        : ( dist[ i ] > avgDist || p.Graph.VNodes[ i ] == null ) ? 0 : 1;
                }
            }


            CaclucateAndSetProbabilities( p, probNodeIdList, dist );

            return probNodeIdList;
        }

        private List<Waypoint> FindWaypointIndexes ( Problem P, List<Waypoint> waypointList )
        {
            List<int> nodes = P.Path.Visited;
            List<Edge> edges = P.Path.Edges;

            if (edges.Count == 0)
            {
                waypointList.Add( new Waypoint( P.Path.Visited.First(), new List<int>(), new List<Edge>() ) );
            }
            else
            {

                // take fewer waypoints if path is way too short
                while (P.Path.Visited.Count <= NumberwaypointList * 2)
                {
                    NumberwaypointList /= 2;
                }


                // space out waypoints evenly alogn the path
                int step = ( edges.Count - 1 ) / NumberwaypointList;
                int remainingNodes = edges.Count % NumberwaypointList;

                // the first waypoint is always the start (aka the first visited node)
                int help = 0;

                // create node and edge lists
                List<int> nodeList;
                List<Edge> edgeList;


                for (int i = 0; i < NumberwaypointList; i++)
                {
                    int actualStep = step;
                    if (remainingNodes > 0)
                    {
                        actualStep++;
                        remainingNodes--;
                    }

                    // generate List of Visited nodes between previous waypoint and currently to be added
                    nodeList = i == NumberwaypointList - 1
                        ? new List<int>( nodes.GetRange( help, actualStep + 1 ) )
                        : new List<int>( nodes.GetRange( help, actualStep ) );
                    edgeList = new List<Edge>( edges.GetRange( help, actualStep ) );

                    waypointList.Add( new Waypoint( nodes[ help ], nodeList, edgeList ) );

                    help += actualStep;
                }
            }

            return waypointList;
        }


        private List<Tuple<double, int>> CalculateProbabilityDistribution ( Node pathMiddle, Problem p, HashSet<Node> availableNodes, bool calculatePointsOfInterest )
        {

            // list containing a probability and the nodeID of the node this probability belongs to
            List<Tuple<double, int>> probNodeIdList = new List<Tuple<double, int>>();
            bool startedWithEmptyPath = p.Path.Visited.Count == 1;
            // calculate average distance of path nodes to the middle
            double avgDist = p.TargetDistance / 2;
            if (startedWithEmptyPath)
            {
                avgDist = 2 * p.TargetDistance / 3;
            }

            // calculate distance of every node from the middle of the path
            double[] dist = new double[ p.Graph.VNodes.Count ];
            double[] actDist = new double[ p.Graph.VNodes.Count ];
            int start = pathMiddle.GraphNodeId;
            PriorityQueue<Tuple<int, double>> queue = new PriorityQueue<Tuple<int, double>>();


            if (calculatePointsOfInterest)
            {
                CaculateDistances( p, availableNodes, dist, actDist, start, queue );
                foreach (Node node in availableNodes)
                {
                    int i = node.GraphNodeId;
                    if (dist[ i ] > 0)
                    {
                        dist[ i ] = startedWithEmptyPath
                            ? ( dist[ i ] > avgDist * 2 || p.Graph.VNodes[ i ] == null || dist[ i ] < avgDist / 2 ) ? 0 : 1 / node.Incident.Sum( x => x.Profit * x.Cost ) / 100
                            : ( dist[ i ] > avgDist * 2 || p.Graph.VNodes[ i ] == null ) ? 0 : 1 / Math.Pow( node.Incident.Sum( x => x.Profit * x.Cost ), 2 );
                    }
                }
            }
            else
            {
                CaculateDistances( p, availableNodes, dist, actDist, start, queue );

                // set distances that are too far to 0
                for (int i = 0; i < dist.Length; i++)
                {
                    if (dist[ i ] != 0)
                    {
                        dist[ i ] = startedWithEmptyPath
                            ? ( dist[ i ] > avgDist * 2 || p.Graph.VNodes[ i ] == null || dist[ i ] < avgDist / 2 ) ? 0 : dist[ i ] / 100
                            : ( dist[ i ] > avgDist * 2 || p.Graph.VNodes[ i ] == null ) ? 0 : actDist[ i ];
                    }
                }
            }

            CaclucateAndSetProbabilities( p, probNodeIdList, dist );

            return probNodeIdList;

        }

        private static void CaclucateAndSetProbabilities ( Problem p, List<Tuple<double, int>> probNodeIdList, double[] dist )
        {

            double sum = 0.0;

            // Calculate sum of distances
            foreach (int distance in dist)
            {
                if (distance != 0)
                {
                    sum += 1.0 / distance;
                }
            }

            // Calculate probabilities
            double[] probabilities = new double[ dist.Length ];
            for (int i = 0; i < dist.Length; i++)
            {
                probabilities[ i ] = dist[ i ] == 0 ? 0 : 1.0 / dist[ i ] / sum;
                probNodeIdList.Add( Tuple.Create( probabilities[ i ], i ) );
            }
        }

        private static void CaculateDistances ( Problem p, HashSet<Node> availableNodes, double[] dist, double[] actDist, int start, PriorityQueue<Tuple<int, double>> queue )
        {
            dist[ start ] = 0.0;
            actDist[ start ] = 0.0;
            queue.Enqueue( 0.0, new Tuple<int, double>( start, 0.0 ) );

            while (queue.Count > 0)
            {
                (double, Tuple<int, double>) current = queue.Dequeue();
                double currentDist = current.Item1;
                int currentNode = current.Item2.Item1;
                double actual = current.Item2.Item2;

                double bestKnownDist = dist[ currentNode ];
                if (bestKnownDist == 0.0 || bestKnownDist == double.MaxValue)
                {
                    dist[ currentNode ] = currentDist;
                    actDist[ currentNode ] = actual;
                    bestKnownDist = currentDist;
                }

                if (bestKnownDist != currentDist)
                {
                    continue;
                }

                List<Edge> currentIncident = p.Graph.VNodes[ currentNode ].Incident;
                foreach (Edge edge in currentIncident)
                {
                    Node neighbor = edge.SourceNode.GraphNodeId == currentNode ? edge.TargetNode : edge.SourceNode;
                    int neighborId = neighbor.GraphNodeId;
                    if (availableNodes.Any( x => x.GraphNodeId == neighborId ))
                    {
                        // weighted distance calculation (cost weighted by profit)
                        double newDistance = bestKnownDist + ( edge.Cost / ( edge.Profit + 0.1 ) );
                        double newActual = bestKnownDist + edge.Cost;

                        // normal distance calculation with unweighted cost
                        //newDistance = bestKnownDist + edge.Cost;
                        double currentNeighborDist = dist[ neighborId ];
                        if (currentNeighborDist == 0.0)
                        {
                            dist[ neighborId ] = double.MaxValue;
                            currentNeighborDist = double.MaxValue;
                        }

                        if (newDistance < currentNeighborDist)
                        {
                            double heuristic = newDistance; /*+ p.Graph.GetDistanceFromLatLon( start, neighborId );*/
                            queue.Enqueue( heuristic, new Tuple<int, double>( neighborId, newActual ) );
                            dist[ neighborId ] = newDistance;
                            actDist[ neighborId ] = newActual;
                        }
                    }
                }
            }
        }

        private Path GenerateNeighborSolution ( Problem problem, bool useDatastructure, ref List<Waypoint> waypointList, ref Dictionary<int, double> cummulativeProbabilities, ref HashSet<Node> availableNodes, Random rnd, Dictionary<int, Dictionary<int, double>> waypointAllDistancesForNodeIdDict, out bool recalculateProbs, ref Tuple<string, int> waypointChangeAndIdx, out int currentWaypointId ) //, out bool alreadyRecalculated )
        {
            // find new Waypoint to insert (& preedSucc coordinates)
            FindNewRandomWaypoint( problem, useDatastructure, ref waypointList, ref cummulativeProbabilities, availableNodes, waypointAllDistancesForNodeIdDict, out Tuple<int, int> predSuccwaypointList, rnd, out Node newWaypoint );
            currentWaypointId = newWaypoint.GraphNodeId;
            recalculateProbs = false;

            Path currentPath;
            double randomNumber = rnd.NextDouble();

            // Add / Remove / Move waypoints
            if (waypointList.Count == 1)
            {
                waypointChangeAndIdx = Tuple.Create( "add", newWaypoint.GraphNodeId );
                currentPath = AddWaypoint( ref availableNodes, ref waypointList, predSuccwaypointList, newWaypoint, problem );

            }
            else if (waypointList.Count < 10 && ( problem.Path.Length < problem.TargetDistance + 1000 ))
            {
                {
                    {
                        waypointChangeAndIdx = randomNumber <= 0.5 ? Tuple.Create( "add", newWaypoint.GraphNodeId ) : Tuple.Create( "move", newWaypoint.GraphNodeId );
                        // if there are less than 10 waypoints, only add or move
                        currentPath = randomNumber <= 0.5
                            ? AddWaypoint( ref availableNodes, ref waypointList, predSuccwaypointList, newWaypoint, problem )
                            : MoveWaypoint( ref availableNodes, ref waypointList, predSuccwaypointList, newWaypoint, problem );
                    }
                }
            }
            else
            {
                int curWaypoint = ( predSuccwaypointList.Item1 + 1 + waypointList.Count ) % waypointList.Count;
                int curWaypointNodeId = waypointList[ curWaypoint ].NodeID;
                waypointChangeAndIdx = randomNumber <= 0.5 ? Tuple.Create( "remove", curWaypointNodeId ) : Tuple.Create( "move", newWaypoint.GraphNodeId );
                // if there are 10 or more, only remove or move
                currentPath = randomNumber <= 0.5
                    ? RemoveWaypoint( ref availableNodes, ref waypointList, problem, predSuccwaypointList )
                    : MoveWaypoint( ref availableNodes, ref waypointList, predSuccwaypointList, newWaypoint, problem );
                recalculateProbs = true;
            }

            currentPath.Quality = problem.GetQuality( problem.GetProfit( currentPath.Visited ), problem.GetArea( currentPath.Visited ), currentPath.Elevation, currentPath.Length );

            return currentPath;
        }

        private List<Edge> FindNewWaypoint ( Problem P, List<Waypoint> waypointList, List<int> nodes, Random rnd, List<Edge> incident, out Node newWaypoint )
        {
            // use random value of all existing waypointList & the point's predecessor (thus start random with 1)
            List<int> availableNodes = new List<int>();
            int newWaypointIdx = 0;

            while (availableNodes.Count == 0)
            {
                newWaypointIdx = rnd.Next( 1, waypointList.Count - 1 );


                //KeyValuePair<int, Tuple<int, List<int>>> startWaypointTuple = waypointList.ElementAt( newWaypointIdx - 1 );
                //KeyValuePair<int, Tuple<int, List<int>>> endWaypointTuple = waypointList.ElementAt( newWaypointIdx );
                //for (int idxInVisited = startWaypointTuple.Key + 1; idxInVisited < endWaypointTuple.Key; idxInVisited++)
                //{
                //    Node possibleWaypoint = P.Graph.VNodes[ nodes[ idxInVisited ] ];
                //    incident = possibleWaypoint.Incident;
                //    if (incident.Count( x => x != null ) > 2)
                //    {
                //        availableNodes.Add( idxInVisited );
                //    }
                //}
            }



            //// select middlemost point as waypoint
            //predSuccwaypointList = Tuple.Create(
            //    waypointList.ElementAt( newWaypointIdx - 1 ).Key,
            //    waypointList.ElementAt( newWaypointIdx + 1 ).Key >= waypointList.Count() ? 0 :
            //    waypointList.ElementAt( newWaypointIdx + 1 ).Key );

            int selector = Math.Abs( ( availableNodes.Count - 1 ) / 2 );
            Node tempWaypoint = P.Graph.VNodes[ nodes[ availableNodes[ selector ] ] ];
            List<Edge> incidentCandidtates;

            if (tempWaypoint.Incident.Count() > 1)
            {
                incidentCandidtates = tempWaypoint.Incident.Where( x =>
                ( x.SourceNode.GraphNodeId == tempWaypoint.GraphNodeId &&
                    ( nodes[ availableNodes[ selector ] - 1 ] != x.TargetNode.GraphNodeId ||
                     nodes[ availableNodes[ selector ] + 1 ] != x.TargetNode.GraphNodeId ) ) ||
                ( x.TargetNode.GraphNodeId == tempWaypoint.GraphNodeId &&
                    ( nodes[ availableNodes[ selector ] - 1 ] != x.SourceNode.GraphNodeId ||
                     nodes[ availableNodes[ selector ] + 1 ] != x.SourceNode.GraphNodeId ) ) ).ToList();
            }
            incidentCandidtates = tempWaypoint.Incident;
            Edge candidateEdge = incidentCandidtates[ rnd.Next( 0, incidentCandidtates.Count - 1 ) ];
            newWaypoint = candidateEdge.SourceNode == tempWaypoint ? candidateEdge.TargetNode : candidateEdge.SourceNode;

            return incident;
        }

        private void FindNewRandomWaypoint ( Problem P, bool useDatastructure, ref List<Waypoint> waypointList, ref Dictionary<int, double> cumulativeProbabilities, HashSet<Node> availableNodes, Dictionary<int, Dictionary<int, double>> waypointAllDistancesForNodeIdDict, out Tuple<int, int> predSuccwaypointList, Random rnd, out Node newWaypoint )
        {
            // pick a random number between 0 and 1
            double randomNumber = rnd.NextDouble();
            // use cummulative probabilities to pick the first matching index
            int chosenIndex = 0;
            foreach (KeyValuePair<int, double> kvp in cumulativeProbabilities)
            {
                Node selected = P.Graph.VNodes[ kvp.Key ];
                if (randomNumber < kvp.Value &&
                    !waypointList.Any( x => x.NodeID == kvp.Key ) &&
                    selected != null &&
                    availableNodes.Contains( selected ))
                {
                    chosenIndex = kvp.Key;
                    break;
                }
            }

            // the new waypoint will be the element at the chosen index
            newWaypoint = P.Graph.VNodes[ chosenIndex ];

            //int j = chosenIndex;
            //while (j > 0 && j + 1 < cumulativeProbabilities.Length && cumulativeProbabilities[ j ] == cumulativeProbabilities[ j + 1 ])
            //{
            //    cumulativeProbabilities[ j ] = cumulativeProbabilities[ j - 1 ];
            //    j++;
            //}
            //if (j > 0)
            //{
            //    // fix the last which will always be skipped in the while loop (since j is updated but not j+1)
            //    cumulativeProbabilities[ j ] = cumulativeProbabilities[ j - 1 ];
            //}
            int closestWaypointIdx = -1;
            double generalShortestDist = double.MaxValue;

            // calculate shortest distance to all waypoints except the first (ensures that the starting point can never be deleted)


            if (!useDatastructure)
            {

                for (int i = 1; i < waypointList.Count; i++)
                {
                    double currentShortestPath = P.Graph.ShortestPath( waypointList[ i ].NodeID, newWaypoint.GraphNodeId );

                    if (currentShortestPath < generalShortestDist)
                    {
                        generalShortestDist = currentShortestPath;
                        closestWaypointIdx = i;
                    }


                }
            }
            else
            {
                foreach (KeyValuePair<int, Dictionary<int, double>> kvp in waypointAllDistancesForNodeIdDict)
                {
                    // make sure we only look at permitted nodes (the ones that have been added to the waypoint
                    // and never use the first waypoint, since that is the starting point and should neither be removed nor moved
                    if (waypointAllDistancesForNodeIdDict[ kvp.Key ].ContainsKey( chosenIndex ) && kvp.Key != waypointList[ 0 ].NodeID)
                    {
                        double shotestDist = waypointAllDistancesForNodeIdDict[ kvp.Key ][ chosenIndex ];
                        if (shotestDist < generalShortestDist)
                        {
                            generalShortestDist = shotestDist;
                            closestWaypointIdx = waypointList.FindIndex( x => x.NodeID == kvp.Key );
                        }
                    }
                }
            }


            // find predecessor and successor
            // (using modulo in case the picked element was the last waypoint and start needs to be selected)
            // idxs are index in waypointList ( = index in node List + 1)
            int predWaypointIdxInWaypointList = ( closestWaypointIdx - 1 + waypointList.Count ) % waypointList.Count;
            int succWaypointIdxInWaypointList = ( closestWaypointIdx + 1 + waypointList.Count ) % waypointList.Count;

            // for locating pred & succ in waypoint List: use normal values
            predSuccwaypointList = Tuple.Create( predWaypointIdxInWaypointList, succWaypointIdxInWaypointList );

        }

        private Path RemoveWaypoint ( ref HashSet<Node> availableNodes, ref List<Waypoint> waypointList, Problem p, Tuple<int, int> predSuccwaypointList )
        {
            int curWaypoint = ( predSuccwaypointList.Item1 + 1 + waypointList.Count ) % waypointList.Count;
            int curWaypointNodeId = waypointList[ curWaypoint ].NodeID;
            int predIdxInWaypointList = predSuccwaypointList.Item1;
            int succIdxInWaypointList = predSuccwaypointList.Item2;

            // removed node is now available again
            _ = availableNodes.Add( p.Graph.VNodes[ curWaypointNodeId ] );
            _ = p.Path.Elevation;
            _ = p.Path.Steepness;
            _ = p.Path.TotalEdgeProfits;
            _ = p.Path.CoveredArea;
            _ = p.Path.Quality;


            Path returnPath = new Path( p.Path );
            _ = new List<int>();
            _ = new List<Edge>();


            // calcualte new path beween pred and succ
            (List<Edge> edges, List<int> nodeIds) = p.Graph.GetShortestPath( waypointList[ predIdxInWaypointList ].NodeID, waypointList[ succIdxInWaypointList ].NodeID );

            // set previously calculated shortest path (nodes and edges) as path values for pred node
            waypointList[ predIdxInWaypointList ].Path = nodeIds;
            waypointList[ predIdxInWaypointList ].Edges = edges;

            // remove current waypoint from WP list
            waypointList.RemoveAt( curWaypoint );

            Utils.UpdateMetadata( p, waypointList, nodeIds, null, ref returnPath );

            return returnPath;
        }

        private Path MoveWaypoint ( ref HashSet<Node> availableNodes, ref List<Waypoint> waypointList, Tuple<int, int> predSuccwaypointList, Node newWaypoint, Problem p )
        {

            int curWaypoint = ( predSuccwaypointList.Item1 + 1 + waypointList.Count ) % waypointList.Count;
            int predIdxInWaypointList = predSuccwaypointList.Item1;
            int succIdxInWaypointList = predSuccwaypointList.Item2;


            // if there are only 2 waypoints, we only have the start (cannot be moved) and one other
            // so, pick the other to move
            if (waypointList.Count == 2)
            {
                predIdxInWaypointList = 0;
                curWaypoint = 1;
                succIdxInWaypointList = 0;
            }

            // removed node is now available again
            _ = availableNodes.Add( p.Graph.VNodes[ waypointList[ curWaypoint ].NodeID ] );
            // new waypoint is not available anymore
            _ = availableNodes.Remove( newWaypoint );


            Path returnPath = new Path( p.Path );
            _ = new List<int>();
            _ = new List<Edge>();
            _ = returnPath.Elevation;
            _ = returnPath.Steepness;
            _ = returnPath.TotalEdgeProfits;
            _ = returnPath.CoveredArea;
            _ = returnPath.Quality;



            List<Edge> edgesL;
            List<int> nodeIdsL;
            List<Edge> edgesR;
            List<int> nodeIdsR;

            // calculate new paths to fill gap while using new waypoint
            (edgesL, nodeIdsL) = p.Graph.GetShortestPath( waypointList[ predIdxInWaypointList ].NodeID, newWaypoint.GraphNodeId );
            (edgesR, nodeIdsR) = p.Graph.GetShortestPath( newWaypoint.GraphNodeId, waypointList[ succIdxInWaypointList ].NodeID );

            // update path from pred to new waypoiunt (change nodes and edges)
            waypointList[ predIdxInWaypointList ].Edges = edgesL;
            waypointList[ predIdxInWaypointList ].Path = nodeIdsL;

            // update whole waypoint at current index (nodeId, path to succ (nodes and edges))
            waypointList[ curWaypoint ].NodeID = newWaypoint.GraphNodeId;
            waypointList[ curWaypoint ].Edges = edgesR;
            waypointList[ curWaypoint ].Path = nodeIdsR;


            Utils.UpdateMetadata( p, waypointList, nodeIdsL, nodeIdsR, ref returnPath );
            return returnPath;
        }

        private Path AddWaypoint ( ref HashSet<Node> availableNodes, ref List<Waypoint> waypointList, Tuple<int, int> predSuccwaypointList, Node newWaypoint, Problem p )
        {
            int predIdx = predSuccwaypointList.Item1;
            int succIdx = predSuccwaypointList.Item2;
            Path returnPath = new Path( p.Path );
            _ = new List<int>();
            _ = new List<Edge>();
            List<Edge> edgesL;
            List<int> nodeIdsL;
            List<Edge> edgesR;
            List<int> nodeIdsR;
            // if the list is too short, at leas one of pred and succ is empty
            // thus, only insert behind predIx
            if (predIdx == 0 && succIdx == 0 && waypointList.Count < 2)
            {

                (edgesL, nodeIdsL) = p.Graph.GetShortestPath( waypointList[ predIdx ].NodeID, newWaypoint.GraphNodeId );
                (edgesR, nodeIdsR) = p.Graph.GetShortestPath( newWaypoint.GraphNodeId, waypointList[ succIdx ].NodeID );
                waypointList[ predIdx ].Edges = edgesL;
                waypointList[ predIdx ].Path = nodeIdsL;

                waypointList.Insert( predIdx + 1, new Waypoint( newWaypoint.GraphNodeId, nodeIdsR, edgesR ) );
            }
            else
            {

                double predShortestPath = p.Graph.ShortestPath( waypointList[ predIdx ].NodeID, newWaypoint.GraphNodeId );
                double succShortestPath = p.Graph.ShortestPath( waypointList[ succIdx ].NodeID, newWaypoint.GraphNodeId );

                // if succ is farther than pred, insert between pred and current
                // thus decrease succIdx by 1 (to end at cur)
                if (predShortestPath < succShortestPath)
                {
                    succIdx = ( succIdx == 0 ? waypointList.Count : succIdx ) - 1;
                }
                // if pred is farther than succ, insert between current and succ
                // thus increase predIdx by 1 (to start at cur)
                else
                {
                    predIdx = predIdx == waypointList.Count - 1 ? 0 : predIdx + 1;
                }


                // calculate new paths to fill gap while using new waypoint
                (edgesL, nodeIdsL) = p.Graph.GetShortestPath( waypointList[ predIdx ].NodeID, newWaypoint.GraphNodeId );
                (edgesR, nodeIdsR) = p.Graph.GetShortestPath( newWaypoint.GraphNodeId, waypointList[ succIdx ].NodeID );


                waypointList[ predIdx ].Edges = edgesL;
                waypointList[ predIdx ].Path = nodeIdsL;

                waypointList.Insert( predIdx + 1, new Waypoint( newWaypoint.GraphNodeId, nodeIdsR, edgesR ) );

            }

            // new waypoint is not available anymore
            _ = availableNodes.Remove( newWaypoint );

            Utils.UpdateMetadata( p, waypointList, nodeIdsL, nodeIdsR, ref returnPath );

            return returnPath;
        }


        private Path RemoveAndAddWaypoint ( ref HashSet<Node> availableNodes, List<Waypoint> waypointList, Problem p, Node newWaypoint, Tuple<int, int> predSuccwaypointList )
        {
            // moving = remove waypoint closest to new waypoint
            Path path = RemoveWaypoint( ref availableNodes, ref waypointList, p, predSuccwaypointList );

            p.Path = path;

            // + add in new one close to pred and succ of it 
            return AddWaypoint( ref availableNodes, ref waypointList, predSuccwaypointList, newWaypoint, p );
        }
    }
}