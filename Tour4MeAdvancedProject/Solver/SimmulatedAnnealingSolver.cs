using NetTopologySuite.Geometries;
using System;
using System.Collections.Generic;
using System.Linq;
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
        public double Temperature { get; set; } = 0.1;
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
                                                                waypointList.Values.Where( nodeIdPathToNextTuple => nodeIdPathToNextTuple.Item1 == x.GraphNodeId ).Count() == 0 ).ToHashSet();

                List<Tuple<double, int>> probabilityList = CalculateProbabilityDistribution( closestNode, P, availableNodes );

                // calculate cummulative Probabilities for selecting the index
                double[] cumulativeProbabilities = new double[ probabilityList.Count ];
                cumulativeProbabilities[ 0 ] = probabilityList[ 0 ].Item1;
                for (int i = 1; i < probabilityList.Count; i++)
                {
                    cumulativeProbabilities[ i ] = cumulativeProbabilities[ i - 1 ] + probabilityList[ i ].Item1;
                }

                List<Waypoint> waypointListBackup = new List<Waypoint>( waypointList );

                bool pathChanged = false;
                #endregion

                // do the simulated annealing runs

                for (int i = 0; i < Runs; i++)
                {
                    double diff = 0;
                    for (int j = 0; j < Repetitions; j++)
                    {
                        Path neighboringSolution = GenerateNeighborSolution( problem, ref waypointList, cumulativeProbabilities, ref availableNodes, rnd, probabilityList, ref addedSurfaceTags, ref addedPathTypes, ref addedSurroundings );

                        double neighborQuality = neighboringSolution.Quality;

                        diff = neighborQuality - currentQuality;

                        // if the quality of the initial solution was worse than of the new one, always pick the new one
                        if (diff > 0)
                        {
                            problem.Path = new Path( neighboringSolution );
                            problem.Quality = neighborQuality;
                            currentQuality = neighborQuality;
                            waypointListBackup = new List<Waypoint>( waypointList );
                            pathChanged = true;
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
                                waypointListBackup = new List<Waypoint>( waypointList );
                                pathChanged = true;
                            }
                        }
                        if (!pathChanged)
                        {
                            waypointList = new List<Waypoint>( waypointListBackup );
                        }
                        pathChanged = false;
                    }
                    // change temperature according to difference in quality and normalize by maxTemperetature
                    Temperature = 1 / ( 1 + Temperature );
                }
                P = problem;
            }

            //P.Path.CoveredArea = P.Quality;
            P.Path.Length = length;
            P.Path.TotalEdgeProfits = currentEdgeProfits;
            P.Path.CoveredArea = currentArea;
            P.Path.Steepness = currentPathsMaxSteepness;
            P.Path.Elevation = currentElevationDiff / 2;
            P.Path.BoundingCoordinates = boudingCoordinates;
            P.Path.PathTypes = string.Join( ", ", addedPathTypes );
            P.Path.Surfaces = string.Join( ", ", addedSurfaceTags );
            P.Path.SurroundingTags = string.Join( ", ", addedSurroundings );


            return SolveStatus.Feasible;
        }


        private List<Waypoint> FindWaypointIndexes ( Problem P, List<Waypoint> waypointList )
        {
            List<int> nodes = P.Path.Visited;
            List<Edge> edges = P.Path.Edges;

            // the first waypoint is always the start (aka the first visited node)
            waypointList.Add( new Waypoint( nodes[ 0 ], new List<int>(), new List<Edge>() ) );

            // take fewer waypoints if path is way too short
            while (P.Path.Visited.Count <= NumberwaypointList * 2)
            {
                NumberwaypointList /= 2;
            }

            List<int> nodeList = new List<int>();
            List<Edge> edgeList = new List<Edge>();

            // space out waypoints evenly alogn the path
            double step = Math.Abs( ( nodes.Count - 1 ) / NumberwaypointList );
            int nodesPerWaypoint = nodes.Count / NumberwaypointList;
            int remainingNodes = nodes.Count % NumberwaypointList;


            for (int i = 1; i < NumberwaypointList; i++)
            {
                int j = (int)Math.Round( i * step, 0 );

                if (remainingNodes > 0)
                {
                    step++;
                    remainingNodes--;
                }

                // generate List of Visited nodes between previous waypoint and currently to be added
                nodeList = new List<int>( nodes.GetRange( i, (int)Math.Round( step ) ) );
                edgeList = new List<Edge>( edges.GetRange( i, (int)Math.Round( step ) ) );

                // update list of nodes that form the path from previous waypoint to the one currently to be added
                waypointList[ i ].Path = nodeList;
                waypointList[ i ].Edges = edgeList;

                waypointList.Add( new Waypoint( j, new List<int>(), new List<Edge>() ) );

            }

            // add last path part
            int lastIdx = NumberwaypointList - 1;
            nodeList = new List<int>( nodes.GetRange( lastIdx, (int)Math.Round( step ) ) );
            edgeList = new List<Edge>( edges.GetRange( lastIdx, (int)Math.Round( step ) ) );

            waypointList[ lastIdx ].Path = nodeList;
            waypointList[ lastIdx ].Edges = edgeList;

            return waypointList;
        }


        private List<Tuple<double, int>> CalculateProbabilityDistribution ( Node pathMiddle, Problem p, HashSet<Node> availableNodes )
        {

            // list containing a probability and the nodeID of the node this probability belongs to
            List<Tuple<double, int>> probNodeIdList = new List<Tuple<double, int>>();

            // calculate average distance of path nodes to the middle
            //double distSum = 0;
            //GeoCoordinate middle = new GeoCoordinate( pathMiddle.Lat, pathMiddle.Lon );
            //List<Node> vNodes = p.Graph.VNodes;

            //foreach (int nodeId in p.Path.Visited)
            //{
            //    Node node = vNodes[ nodeId ];
            //    GeoCoordinate point = new GeoCoordinate( node.Lat, node.Lon );

            //    double distance = middle.GetDistanceTo( point );
            //    distSum += distance;
            //}

            //double avgDist = distSum / p.Path.Visited.Count;
            double avgDist = p.TargetDistance / 4;

            // calculate distance of every node from the middle of the path

            double[] dist = new double[ p.Graph.VNodes.Count + 1 ];
            PriorityQueue<Tuple<int, double>> queue = new PriorityQueue<Tuple<int, double>>();

            int start = pathMiddle.GraphNodeId;
            //List<Node> availableNodesList = availableNodes.ToList();
            //availableNodesList.Add( pathMiddle );
            dist[ start ] = 0.0;
            queue.Enqueue( 0.0, new Tuple<int, double>( start, 0.0 ) );

            while (queue.Count > 0)
            {
                (double, Tuple<int, double>) current = queue.Dequeue();
                _ = current.Item1;
                int currentNode = current.Item2.Item1;
                double actual = current.Item2.Item2;

                double bestKnownDist = dist[ currentNode ];
                if (bestKnownDist == 0.0 && bestKnownDist == double.MaxValue)
                {
                    dist[ currentNode ] = actual;
                    bestKnownDist = actual;
                }

                if (bestKnownDist != actual)
                {
                    continue;
                }

                List<Edge> currentIncident = p.Graph.VNodes[ currentNode ].Incident;
                foreach (Edge edge in currentIncident)
                {
                    Node neighbor = edge.SourceNode.GraphNodeId == currentNode ? edge.TargetNode : edge.SourceNode;
                    int neighborId = neighbor.GraphNodeId;
                    if (neighborId < availableNodes.Count)
                    {

                        double newDistance = bestKnownDist + edge.Cost;
                        double currentNeighborDist = dist[ neighborId ];
                        if (currentNeighborDist == 0.0)
                        {
                            dist[ neighborId ] = double.MaxValue;
                            currentNeighborDist = double.MaxValue;
                        }

                        if (newDistance < currentNeighborDist)
                        {
                            double heuristic = newDistance + p.Graph.GetDistanceFromLatLon( start, neighborId );
                            queue.Enqueue( heuristic, new Tuple<int, double>( neighborId, newDistance ) );
                            dist[ neighborId ] = newDistance;
                        }
                    }
                }
            }

            // TODO Debug only
            Dictionary<int, double> nodeIdDists = new Dictionary<int, double>();


            for (int i = 0; i < dist.Length; i++)
            {
                if (dist[ i ] != 0)
                {
                    //dist[ i ] = ( dist[ i ] < avgDist / 10 || dist[ i ] > 2 * avgDist ) ? 0 : dist[ i ];
                    dist[ i ] = ( dist[ i ] > avgDist ) ? 0 : dist[ i ];
                    nodeIdDists.Add( i, dist[ i ] );
                }
            }
            double sum = 0.0;

            // Calculate sum of distances
            foreach (int distance in dist)
            {
                if (distance != 0)
                {
                    sum += 1.0 / Math.Pow( distance, 2 );
                }
            }

            // TODO debug only
            Dictionary<int, double> nodeIdDistPairs = new Dictionary<int, double>();


            // Calculate probabilities
            double[] probabilities = new double[ dist.Length ];
            for (int i = 0; i < dist.Length; i++)
            {
                probabilities[ i ] = dist[ i ] == 0 ? 0 : 1.0 / dist[ i ] / sum;
                probNodeIdList.Add( Tuple.Create( probabilities[ i ], i ) );
                if (probabilities[ i ] > 0)
                {
                    nodeIdDistPairs.Add( i, 1.0 / Math.Pow( dist[ i ], 2 ) / sum );
                }
            }

            return probNodeIdList;

        }


        private Path GenerateNeighborSolution ( Problem problem, ref List<Waypoint> waypointList, double[] cummulativeProbabilities, ref HashSet<Node> availableNodes, Random rnd, List<Tuple<double, int>> probabilityList, ref HashSet<SurfaceTag> addedSurfaceTags, ref HashSet<HighwayTag> addedPathTypes, ref HashSet<string> addedSurroundings )
        {
            // find new Waypoint to insert (& preedSucc coordinates)
            FindNewRandomWaypoint( problem, ref availableNodes, ref waypointList, cummulativeProbabilities, out Tuple<int, int> predSuccwaypointList, rnd, probabilityList, out Node newWaypoint );

            // Add / Remove / Move waypoints

            Path currentPath = MoveWaypoint( ref waypointList, predSuccwaypointList, newWaypoint, problem, ref addedSurfaceTags, ref addedPathTypes, ref addedSurroundings );
            //currentPath = RemoveWaypoint( waypointList, problem, rnd, newWaypoint, predSuccwaypointList, ref addedSurfaceTags, ref addedPathTypes, ref addedSurroundings);
            //Path currentPath = RemoveAndAddWaypoint( waypointList, problem, rnd, newWaypoint, predSuccwaypointList );
            _ = predSuccwaypointList.ToString();

            //originalProblem.Path = currentPath;
            currentPath.Quality = problem.GetQuality( problem.GetProfit( currentPath.Visited ), problem.GetArea( currentPath.Visited ), currentPath.Elevation );

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

        private void FindNewRandomWaypoint ( Problem P, ref HashSet<Node> availableNodes, ref List<Waypoint> waypointList, double[] cumulativeProbabilities, out Tuple<int, int> predSuccwaypointList, Random rnd, List<Tuple<double, int>> probabilityList, out Node newWaypoint )
        {
            // pick a random number between 0 and 1
            double randomNumber = rnd.NextDouble();
            // use cummulative probabilities to pick the first matching index
            int chosenIndex = 0;
            for (int i = 0; i < cumulativeProbabilities.Length; i++)
            {
                if (randomNumber < cumulativeProbabilities[ i ])
                {
                    chosenIndex = i;
                    break;
                }
            }

            // the new waypoint will be the element at the chosen index
            newWaypoint = availableNodes.ElementAt( chosenIndex );

            double shortestPath = double.MaxValue;
            int closestWaypointIdx = -1;

            // calculate shortest distance to all waypoints except the first (ensures that the starting point can never be deleted)
            for (int i = 1; i < waypointList.Count; i++)
            {
                double currentShortestPath = P.Graph.ShortestPath( waypointList[ i ].NodeID, newWaypoint.GraphNodeId );

                if (currentShortestPath < shortestPath)
                {
                    shortestPath = currentShortestPath;
                    closestWaypointIdx = i;
                }


            }

            // find predecessor and successor
            // (using modulo in case the picked element was the last waypoint and start needs to be selected)
            // idxs are index in waypointList ( = index in node List + 1)
            int predWaypointIdxInWaypointList = ( closestWaypointIdx - 1 + waypointList.Count ) % waypointList.Count;
            int succWaypointIdxInWaypointList = ( closestWaypointIdx + 1 + waypointList.Count ) % waypointList.Count;

            // for locating pred & succ in waypoint List: use normal values
            predSuccwaypointList = Tuple.Create( predWaypointIdxInWaypointList, succWaypointIdxInWaypointList );

            // TODO maybe keep removed node not available
            // removed node is now available again
            _ = availableNodes.Add( P.Graph.VNodes[ waypointList[ closestWaypointIdx ].NodeID ] );
            // new waypoint is not available anymore
            _ = availableNodes.Remove( newWaypoint );
            _ = waypointList.Remove( waypointList.ElementAt( closestWaypointIdx ) );
        }

        private Path RemoveWaypoint ( ref List<Waypoint> waypointList, Problem p, Tuple<int, int> predSuccwaypointList, ref HashSet<SurfaceTag> addedSurfaceTags, ref HashSet<HighwayTag> addedPathTypes, ref HashSet<string> addedSurroundings )
        {
            int curWaypoint = predSuccwaypointList.Item1 + 1;
            int predIdx = predSuccwaypointList.Item1;
            int succIdx = predSuccwaypointList.Item2;

            double elevationDiff = p.Path.Elevation;
            double maxSteepness = p.Path.Steepness;
            double currentEdgeProfits = p.Path.TotalEdgeProfits;
            double currentArea = p.Path.CoveredArea;
            double currentQuality = p.Path.Quality;


            List<int> visited = p.Path.Visited;
            List<Edge> pathEdges = p.Path.Edges;



            // prepare to calculate new path to fill gap
            (List<Edge> edges, List<int> nodeIds) = (new List<Edge>(), new List<int>());
            _ = waypointList.Remove( curWaypoint );

            //check if succ was start (special case; removal step only between pred and end of visited
            if (succIdx == 0)
            {
                // calculate shortest path to fill the new gap
                (edges, nodeIds) = p.Graph.GetShortestPath( visited[ predIdx ], visited[ visited.Count - 1 ] );
                // look at all nodes and edges between pred and end of list & remove them
                for (int i = predIdx + 1; i < visited.Count - 1; i++)
                {
                    Edge currentEdge = pathEdges[ predIdx + 1 ];
                    elevationDiff -= Math.Abs( currentEdge.SourceNode.Elevation - currentEdge.TargetNode.Elevation ) / 2;

                    visited.RemoveAt( predIdx + 1 );
                    pathEdges.RemoveAt( predIdx + 1 );
                }
            }
            else
            {
                // calculate shortest path to fill the new gap
                (edges, nodeIds) = p.Graph.GetShortestPath( visited[ predIdx ], visited[ succIdx ] );
                // look at all nodes and edges between pred and succ waypoint & remove them
                for (int i = predIdx + 1; i < succIdx; i++)
                {
                    Edge currentEdge = pathEdges[ predIdx + 1 ];
                    elevationDiff -= Math.Abs( currentEdge.SourceNode.Elevation - currentEdge.TargetNode.Elevation ) / 2;

                    visited.RemoveAt( predIdx + 1 );
                    pathEdges.RemoveAt( predIdx + 1 );
                }
            }

            // re-calculate steepness (TODO Check this!!)
            maxSteepness = pathEdges.Max( x => Math.Abs( x.SourceNode.Elevation - x.TargetNode.Elevation ) / x.Cost * 100 );
            foreach (Edge edge in edges)
            {
                foreach (string currentTag in edge.Tags)
                {
                    Utils.AddTags( ref addedSurfaceTags, ref addedPathTypes, ref addedSurroundings, currentTag );
                }
                Utils.CalculateElevationDiffAndSteepness( edge, ref maxSteepness, ref elevationDiff );
                Utils.CaculateQualityValues( p, edge, elevationDiff, ref currentEdgeProfits, ref currentArea, ref currentQuality );
            }
            Tuple<double, double>[] boundingCoordinates = p.Path.BoundingCoordinates;

            // re-calculate bounding box coordinates
            foreach (int nodeId in nodeIds)
            {
                p.Path.UpdateBoundingCoordinates( ref boundingCoordinates, p.Graph.VNodes[ nodeId ] );
            }

            // add newly calcualted visited nodes and edges
            visited.InsertRange( predIdx + 1, nodeIds );
            pathEdges.InsertRange( predIdx + 1, edges );

            // TODO for testing only, set breakpoint with conditions to check for a closed path!
            //for (int i = 0; i < visited.Count - 1; i++)
            //{
            //    Edge edge = p.Graph.GetEdge( visited[ i ], visited[ i + 1 ] );
            //}

            // update all current path values
            p.Path.Visited = visited;
            p.Path.Edges = pathEdges;
            p.Path.Elevation = elevationDiff / 2;
            p.Path.Length = pathEdges.Sum( x => x.Cost );
            p.Path.BoundingCoordinates = boundingCoordinates;

            return p.Path;
        }

        private Path MoveWaypoint ( ref List<Waypoint> waypointList, Tuple<int, int> predSuccwaypointList, Node newWaypoint, Problem p, ref HashSet<SurfaceTag> addedSurfaceTags, ref HashSet<HighwayTag> addedPathTypes, ref HashSet<string> addedSurroundings )
        {

            int predIdx = predSuccwaypointList.Item1;
            int succIdx = predSuccwaypointList.Item2;

            Path returnPath = new Path( p.Path );

            List<int> visited = new List<int>( returnPath.Visited );
            List<Edge> pathEdges = new List<Edge>( returnPath.Edges );

            double elevationDiff = returnPath.Elevation;
            double maxSteepness = returnPath.Steepness;
            double currentEdgeProfits = returnPath.TotalEdgeProfits;
            double currentArea = returnPath.CoveredArea;
            double currentQuality = returnPath.Quality;

            // prepare to calculate new path to fill gap
            (List<Edge> edgesL, List<int> nodeIdsL) = (new List<Edge>(), new List<int>());

            // prepare to calculate new path to fill gap
            (List<Edge> edgesR, List<int> nodeIdsR) = (new List<Edge>(), new List<int>());

            // calculate new paths to fill gap while using new waypoint
            (edgesL, nodeIdsL) = p.Graph.GetShortestPath( waypointList[ predIdx ].NodeID, newWaypoint.GraphNodeId );
            (edgesR, nodeIdsR) = p.Graph.GetShortestPath( newWaypoint.GraphNodeId, waypointList[ succIdx ].NodeID );


            int numberRemoved = 0;
            //check if succ was start (special case; removal step only between pred and end of visited
            //if (succIdx == 0)
            //{
            //    // look at all nodes and edges between pred and end of list & remove them
            //    int lastIndex = visited.Count;
            //    for (int i = predIdx + 1; i < lastIndex - 1; i++)
            //    {
            //        Edge currentEdge = pathEdges[ predIdx + 1 ];
            //        elevationDiff -= Math.Abs( currentEdge.SourceNode.Elevation - currentEdge.TargetNode.Elevation ) / 2;

            //        visited.RemoveAt( predIdx + 1 );
            //        pathEdges.RemoveAt( predIdx + 1 );
            //    }
            //    numberRemoved = lastIndex - predIdx;
            //}
            //else
            //{
            //    // look at all nodes and edges between pred and succ waypoint & remove nodes and edges to allow adding new waypoint
            //    for (int i = predIdx + 1; i < succIdx; i++)
            //    {
            //        Edge currentEdge = pathEdges[ predIdx + 1 ];
            //        elevationDiff -= Math.Abs( currentEdge.SourceNode.Elevation - currentEdge.TargetNode.Elevation ) / 2;

            //        visited.RemoveAt( predIdx + 1 );
            //        pathEdges.RemoveAt( predIdx + 1 );
            //    }
            //    numberRemoved = succIdx - predIdx;
            //}

            waypointList[ predIdx ].Edges = edgesL;
            waypointList[ succIdx ].Edges = edgesR;
            waypointList[ predIdx ].Path = nodeIdsL;
            waypointList[ succIdx ].Path = nodeIdsR;



            // re-calculate bounding box coordinates
            Tuple<double, double>[] boundingCoordinates = returnPath.BoundingCoordinates;
            foreach (int nodeId in nodeIdsL)
            {
                returnPath.UpdateBoundingCoordinates( ref boundingCoordinates, p.Graph.VNodes[ nodeId ] );
            }
            foreach (int nodeId in nodeIdsR)
            {
                returnPath.UpdateBoundingCoordinates( ref boundingCoordinates, p.Graph.VNodes[ nodeId ] );
            }

            // add newly calcualted visited nodes and edges
            visited.InsertRange( predIdx + 1, nodeIdsL );
            pathEdges.InsertRange( predIdx + 1, edgesL );

            visited.InsertRange( predIdx + 1 + nodeIdsL.Count(), nodeIdsR );
            pathEdges.InsertRange( predIdx + 1 + edgesL.Count(), edgesR );

            int shiftWaypointsBy = nodeIdsL.Count() + nodeIdsR.Count() - numberRemoved + 1;
            for (int i = 0; i < waypointList.Count; i++)
            {
                KeyValuePair<int, Tuple<int, List<int>>> currentWaypointTuple = waypointList.ElementAt( i );
                int waypoint = currentWaypointTuple.Key;
                if (waypoint > predIdx)
                {
                    waypoint += shiftWaypointsBy;
                    _ = waypointList.Remove( currentWaypointTuple.Key );
                    waypointList.Add( waypoint, currentWaypointTuple.Value );

                }
            }

            // add the new waypoint and reorder
            int newWaypointIdxInVisited = predIdx + 1 + nodeIdsL.Count;
            waypointList.Add( newWaypointIdxInVisited, Tuple.Create( visited[ newWaypointIdxInVisited ], nodeIdsR ) );
            waypointList = waypointList.OrderBy( x => x.Key ).ToDictionary(

            // TODO for testing only, set breakpoint with conditions to check for a closed path!
            //for (int i = 0; i < visited.Count - 1; i++)
            //{
            //    Edge edge = p.Graph.GetEdge( visited[ i ], visited[ i + 1 ] );
            //}

            // update all current path values
            returnPath.Visited = visited;
            returnPath.Edges = pathEdges;
            returnPath.Elevation = elevationDiff / 2;
            returnPath.Steepness = maxSteepness;
            returnPath.BoundingCoordinates = boundingCoordinates;
            returnPath.Length = pathEdges.Sum( x => x.Cost );

            return returnPath;
        }

        private Path RemoveAndAddWaypoint ( List<Waypoint> waypointList, Problem p, Random rnd, Node newWaypoint, Tuple<int, int> predSuccwaypointList, ref HashSet<SurfaceTag> addedSurfaceTags, ref HashSet<HighwayTag> addedPathTypes, ref HashSet<string> addedSurroundings )
        {
            // moving = remove waypoint closest to new waypoint
            Path path = RemoveWaypoint( ref waypointList, p, predSuccwaypointList, ref addedSurfaceTags, ref addedPathTypes, ref addedSurroundings );

            p.Path = path;

            // + add in new one close to pred and succ of it 
            return AddWaypoint( waypointList, predSuccwaypointList, newWaypoint, p );
        }

        private Path AddWaypoint ( List<Waypoint> waypointList, Tuple<int, int> predSuccwaypointList, Node newWaypoint, Problem p )
        {
            throw new NotImplementedException();
        }
    }
}