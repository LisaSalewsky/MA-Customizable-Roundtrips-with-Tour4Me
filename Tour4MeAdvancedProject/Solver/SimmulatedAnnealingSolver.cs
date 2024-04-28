﻿using NetTopologySuite.Geometries;
using NetTopologySuite.Index.Strtree;
using System;
using System.Collections.Generic;
using System.Device.Location;
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
        public int Runs { get; set; } = 5;
        // repetitions at each temperature
        public int Repetitions { get; set; } = 10;
        public double Temperature { get; set; } = 0.3;
        public Random Random { get; set; } = new Random();
        public int NumberWaypointIdxInVisited { get; set; } = 10;



        public SimmulatedAnnealingSolver () { }

        public SimmulatedAnnealingSolver ( int runs, int repetitions, double temp, int waypointIdxInVisited )
        {
            Runs = runs;
            Repetitions = repetitions;
            Temperature = temp;
            NumberWaypointIdxInVisited = waypointIdxInVisited;
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
                #region prepare needed values


                P.Graph.InitializeShortestPath( P.Start );

                //NumberwaypointIdxInVisited = P.Path.Visited.Count() / 10;
                double currentQuality = P.Path.Quality;
                double initProb = 0.6;


                Problem problem = new Problem( P, P.Path );
                // list of the index where to find Node(Id) in the List of all visited nodes
                List<int> waypointIdxInVisited = new List<int>();
                Random rnd = new Random();

                // find waypoint idxs
                _ = FindWaypointIndexes( P, waypointIdxInVisited );

                double lonL = P.Path.BoundingCoordinates[ 0 ].Item2;
                double latB = P.Path.BoundingCoordinates[ 1 ].Item1;
                double latT = P.Path.BoundingCoordinates[ 2 ].Item1;
                double lonR = P.Path.BoundingCoordinates[ 3 ].Item2;

                double targetDist = P.TargetDistance;
                Coordinate pathMiddle = new Coordinate( ( latT + latB ) / 2, ( lonR + lonL ) / 2 );
                STRtree<Node> tree = new STRtree<Node>();
                List<Node> graphNodes = P.Graph.VNodes.Where( x => x != null && x.ShortestDistance < problem.TargetDistance ).ToList();
                foreach (Node node in graphNodes)
                {
                    tree.Insert( new Envelope( node.Lat, node.Lat, node.Lon, node.Lon ), node );
                }
                List<int> waypointNodeIdxs = new List<int>();

                foreach (int idx in waypointIdxInVisited)
                {
                    waypointNodeIdxs.Add( P.Path.Visited[ idx ] );
                }

                // Query the tree for nearest neighbors
                Node closestNode = graphNodes
                                    .OrderBy( node => pathMiddle.Distance( new Coordinate( node.Lat, node.Lon ) ) )
                                    .FirstOrDefault();
                HashSet<Node> availableNodes = P.Graph.VNodes.Where( x => x != null &&
                                                                            !waypointNodeIdxs.Contains( x.GraphNodeId ) &&
                                                                            x.ShortestDistance <= targetDist ).ToHashSet();

                List<Tuple<double, int>> probabilityList = CalculateProbabilityDistribution( closestNode, P, availableNodes );

                // calculate cummulative Probabilities for selecting the index
                double[] cumulativeProbabilities = new double[ probabilityList.Count ];
                cumulativeProbabilities[ 0 ] = probabilityList[ 0 ].Item1;
                for (int i = 1; i < probabilityList.Count; i++)
                {
                    cumulativeProbabilities[ i ] = cumulativeProbabilities[ i - 1 ] + probabilityList[ i ].Item1;
                }

                #endregion

                // do the simulated annealing runs

                for (int i = 0; i < Runs; i++)
                {
                    double diff = 0;
                    for (int j = 0; j < Repetitions; j++)
                    {
                        Path neighboringSolution = GenerateNeighborSolution( problem, ref waypointIdxInVisited, cumulativeProbabilities, availableNodes, rnd, probabilityList );

                        double neighborQuality = neighboringSolution.Quality;

                        diff = neighborQuality - currentQuality;
                        if (Math.Abs( neighboringSolution.Length - P.Path.Length ) < 100)
                        {
                            Console.WriteLine( "test" );
                        }

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
                                P = problem;
                                P.Path = neighboringSolution;
                                P.Quality = neighborQuality;
                                currentQuality = neighborQuality;
                            }
                        }
                    }
                    // change temperature according to difference in quality and normalize by maxTemperetature
                    Temperature = -Math.Abs( diff ) / Math.Log( initProb );
                }
            }
            P.Path.CoveredArea = P.Quality;
            return SolveStatus.Feasible;
        }


        private List<int> FindWaypointIndexes ( Problem P, List<int> waypointIdxInVisited )
        {
            List<int> nodes = P.Path.Visited;

            // the first waypoint is always the start (aka the first visited node)
            waypointIdxInVisited.Add( 0 );

            // take fewer waypoints if path is way too short
            while (P.Path.Visited.Count <= NumberWaypointIdxInVisited * 2)
            {
                NumberWaypointIdxInVisited /= 2;
            }

            // space out waypoints evenly alogn the path
            double step = Math.Abs( ( nodes.Count - 1 ) / NumberWaypointIdxInVisited );
            for (int i = 1; i < NumberWaypointIdxInVisited; i++)
            {
                int j = (int)Math.Round( i * step, 0 );
                waypointIdxInVisited.Add( j );

            }

            return waypointIdxInVisited;
        }


        private List<Tuple<double, int>> CalculateProbabilityDistribution ( Node pathMiddle, Problem p, HashSet<Node> availableNodes )
        {

            // list containing a probability and the nodeID of the node this probability belongs to
            List<Tuple<double, int>> probNodeIdList = new List<Tuple<double, int>>();

            // calculate average distance of path nodes to the middle
            double distSum = 0;
            GeoCoordinate middle = new GeoCoordinate( pathMiddle.Lat, pathMiddle.Lon );
            List<Node> vNodes = p.Graph.VNodes;

            foreach (int nodeId in p.Path.Visited)
            {
                Node node = vNodes[ nodeId ];
                GeoCoordinate point = new GeoCoordinate( node.Lat, node.Lon );

                double distance = middle.GetDistanceTo( point );
                distSum += distance;
            }

            double avgDist = distSum / p.Path.Visited.Count;

            // calculate distance of every node from the middle of the path

            double[] dist = new double[ availableNodes.Count + 1 ];
            PriorityQueue<Tuple<int, double>> queue = new PriorityQueue<Tuple<int, double>>();

            int start = pathMiddle.GraphNodeId;
            List<Node> availableNodesList = availableNodes.ToList();
            availableNodesList.Add( pathMiddle );
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

                List<Edge> currentIncident = availableNodesList[ currentNode ].Incident;
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

            for (int i = 0; i < dist.Length; i++)
            {
                if (dist[ i ] != 0)
                {
                    dist[ i ] += dist[ i ] < avgDist ? avgDist / 4 : 0;
                }
            }
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

            return probNodeIdList;

        }


        private Path GenerateNeighborSolution ( Problem problem, ref List<int> waypointIdxInVisited, double[] cummulativeProbabilities, HashSet<Node> availableNodes, Random rnd, List<Tuple<double, int>> probabilityList )
        {
            // find new Waypoint to insert (& preedSucc coordinates)
            FindNewRandomWaypoint( problem, availableNodes, ref waypointIdxInVisited, cummulativeProbabilities, out Tuple<int, int> predSuccwaypointIdxInVisited, rnd, probabilityList, out Node newWaypoint );

            // Add / Remove / Move waypoints

            Path currentPath = MoveWaypoint( ref waypointIdxInVisited, predSuccwaypointIdxInVisited, newWaypoint, problem );
            //currentPath = RemoveWaypoint( waypointIdxInVisited, problem, rnd, newWaypoint, predSuccwaypointIdxInVisited );
            //Path currentPath = RemoveAndAddWaypoint( waypointIdxInVisited, problem, rnd, newWaypoint, predSuccwaypointIdxInVisited );
            _ = predSuccwaypointIdxInVisited.ToString();

            //originalProblem.Path = currentPath;
            currentPath.Quality = problem.GetQuality( problem.GetProfit( currentPath.Visited ), problem.GetArea( currentPath.Visited ), problem.Path.Elevation );

            return currentPath;
        }

        private List<Edge> FindNewWaypoint ( Problem P, List<int> waypointIdxInVisited, out Tuple<int, int> predSuccwaypointIdxInVisited, List<int> nodes, Random rnd, List<Edge> incident, out Node newWaypoint )
        {
            // use random value of all existing waypointIdxInVisited & the point's predecessor (thus start random with 1)
            List<int> availableNodes = new List<int>();
            int newWaypointIdx = 0;

            while (availableNodes.Count == 0)
            {
                newWaypointIdx = rnd.Next( 1, waypointIdxInVisited.Count - 1 );
                for (int nodeId = waypointIdxInVisited[ newWaypointIdx - 1 ] + 1; nodeId < waypointIdxInVisited[ newWaypointIdx ]; nodeId++)
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
            predSuccwaypointIdxInVisited = Tuple.Create( waypointIdxInVisited[ newWaypointIdx - 1 ], waypointIdxInVisited[ newWaypointIdx + 1 >= waypointIdxInVisited.Count() ? 0 : newWaypointIdx + 1 ] );
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

        private void FindNewRandomWaypoint ( Problem P, HashSet<Node> availableNodes, ref List<int> waypointIdxInVisited, double[] cumulativeProbabilities, out Tuple<int, int> predSuccwaypointIdxInVisited, Random rnd, List<Tuple<double, int>> probabilityList, out Node newWaypoint )
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

            Dictionary<int, double> nodeDistDictionary = new Dictionary<int, double>();

            // calculate shortest distance to all waypoints except the first (ensures that the starting point can never be deleted)
            for (int i = 1; i < waypointIdxInVisited.Count; i++)
            {
                double currentShortestPath = P.Graph.ShortestPath( P.Path.Visited[ waypointIdxInVisited[ i ] ], newWaypoint.GraphNodeId );

                // add at i-1 because i starts at 1 for incexing waypointIdxInVisited but indexing nodeDistDictionary starts at 0
                nodeDistDictionary.Add( i - 1, currentShortestPath );

            }
            // sort to find closest point 
            List<KeyValuePair<int, double>> nodeList = nodeDistDictionary.OrderBy( x => x.Value ).ToList();

            // this is our current waypoint
            int pickedElement = nodeList[ 0 ].Key;

            // find the *two* neighbors by looking at picked elements neighbors
            // adjust picked element to fit waypointIdx
            pickedElement++;


            // find predecessor and successor
            // (using modulo in case the picked element was the last waypoint and start needs to be selected)
            // idxs are index in waypointList ( = index in node List + 1)
            int predWaypointIdxInWaypointList = ( pickedElement - 1 + waypointIdxInVisited.Count ) % waypointIdxInVisited.Count;
            int succWaypointIdxInWaypointList = ( pickedElement + 1 + waypointIdxInVisited.Count ) % waypointIdxInVisited.Count;

            // for locating pred & succ in waypoint List: use normal values
            predSuccwaypointIdxInVisited = Tuple.Create( waypointIdxInVisited[ predWaypointIdxInWaypointList ],
                                                            waypointIdxInVisited[ succWaypointIdxInWaypointList ] );

            _ = waypointIdxInVisited.Remove( waypointIdxInVisited[ pickedElement ] );
        }

        private Path RemoveWaypoint ( ref List<int> waypointIdxInVisited, Problem p, Tuple<int, int> predSuccwaypointIdxInVisited )
        {
            int curWaypoint = predSuccwaypointIdxInVisited.Item1 + 1;
            int predIdx = predSuccwaypointIdxInVisited.Item1;
            int succIdx = predSuccwaypointIdxInVisited.Item2;

            double elevationDiff = p.Path.Elevation;
            double maxSteepness = p.Path.Steepness;


            List<int> visited = p.Path.Visited;
            List<Edge> pathEdges = p.Path.Edges;



            // prepare to calculate new path to fill gap
            (List<Edge> edges, List<int> nodeIds) = (new List<Edge>(), new List<int>());
            _ = waypointIdxInVisited.Remove( curWaypoint );

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
                Utils.CalculateElevationDiffAndSteepness( edge, ref maxSteepness, ref elevationDiff );
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

        private Path MoveWaypoint ( ref List<int> waypointIdxInVisited, Tuple<int, int> predSuccwaypointIdxInVisited, Node newWaypoint, Problem p )
        {

            int predIdx = predSuccwaypointIdxInVisited.Item1;
            int succIdx = predSuccwaypointIdxInVisited.Item2;

            List<int> visited = p.Path.Visited;
            List<Edge> pathEdges = p.Path.Edges;

            double elevationDiff = p.Path.Elevation;
            double maxSteepness = p.Path.Steepness;

            // TODO remove old waypoint; add new one (maybe do this at FindNewRandomWaypoint)

            // prepare to calculate new path to fill gap
            (List<Edge> edgesL, List<int> nodeIdsL) = (new List<Edge>(), new List<int>());

            // prepare to calculate new path to fill gap
            (List<Edge> edgesR, List<int> nodeIdsR) = (new List<Edge>(), new List<int>());

            // calculate new paths to fill gap while using new waypoint
            int succ = visited[ succIdx ];
            List<int> prevVisited = new List<int>( visited );
            (edgesL, nodeIdsL) = p.Graph.GetShortestPath( visited[ predIdx ], newWaypoint.GraphNodeId );
            (edgesR, nodeIdsR) = p.Graph.GetShortestPath( newWaypoint.GraphNodeId, succ );

            // add the new waypoint and reorder
            waypointIdxInVisited.Add( predIdx + 1 + nodeIdsL.Count );
            waypointIdxInVisited = waypointIdxInVisited.OrderBy( x => x ).ToList();

            int numberRemoved = 0;
            //check if succ was start (special case; removal step only between pred and end of visited
            if (succIdx == 0)
            {
                // look at all nodes and edges between pred and end of list & remove them
                int lastIndex = visited.Count;
                for (int i = predIdx + 1; i < lastIndex - 1; i++)
                {
                    Edge currentEdge = pathEdges[ predIdx + 1 ];
                    elevationDiff -= Math.Abs( currentEdge.SourceNode.Elevation - currentEdge.TargetNode.Elevation ) / 2;

                    visited.RemoveAt( predIdx + 1 );
                    pathEdges.RemoveAt( predIdx + 1 );
                }
                numberRemoved = lastIndex - predIdx;
            }
            else
            {
                // look at all nodes and edges between pred and succ waypoint & remove nodes and edges to allow adding new waypoint
                for (int i = predIdx + 1; i < succIdx; i++)
                {
                    Edge currentEdge = pathEdges[ predIdx + 1 ];
                    elevationDiff -= Math.Abs( currentEdge.SourceNode.Elevation - currentEdge.TargetNode.Elevation ) / 2;

                    visited.RemoveAt( predIdx + 1 );
                    pathEdges.RemoveAt( predIdx + 1 );
                }
                numberRemoved = succIdx - predIdx;
            }

            // re-calculate steepness (TODO Check this!!)
            maxSteepness = pathEdges.Max( x => Math.Abs( x.SourceNode.Elevation - x.TargetNode.Elevation ) / x.Cost * 100 );
            foreach (Edge edge in edgesL)
            {
                Utils.CalculateElevationDiffAndSteepness( edge, ref maxSteepness, ref elevationDiff );
            }
            foreach (Edge edge in edgesR)
            {
                Utils.CalculateElevationDiffAndSteepness( edge, ref maxSteepness, ref elevationDiff );
            }

            // re-calculate bounding box coordinates
            Tuple<double, double>[] boundingCoordinates = p.Path.BoundingCoordinates;
            foreach (int nodeId in nodeIdsL)
            {
                p.Path.UpdateBoundingCoordinates( ref boundingCoordinates, p.Graph.VNodes[ nodeId ] );
            }
            foreach (int nodeId in nodeIdsR)
            {
                p.Path.UpdateBoundingCoordinates( ref boundingCoordinates, p.Graph.VNodes[ nodeId ] );
            }

            for (int i = 0; i < waypointIdxInVisited.Count; i++)
            {
                int waypoint = waypointIdxInVisited[ i ];
                if (waypoint > predIdx && waypoint < succIdx)
                {
                    waypoint += nodeIdsL.Count() + nodeIdsR.Count() - numberRemoved;
                }
            }

            // add newly calcualted visited nodes and edges
            visited.InsertRange( predIdx + 1, nodeIdsL );
            pathEdges.InsertRange( predIdx + 1, edgesL );

            visited.InsertRange( predIdx + 1 + nodeIdsL.Count(), nodeIdsR );
            pathEdges.InsertRange( predIdx + 1 + edgesL.Count(), edgesR );


            // TODO for testing only, set breakpoint with conditions to check for a closed path!
            //for (int i = 0; i < visited.Count - 1; i++)
            //{
            //    Edge edge = p.Graph.GetEdge( visited[ i ], visited[ i + 1 ] );
            //}

            // update all current path values
            p.Path.Visited = visited;
            p.Path.Edges = pathEdges;
            p.Path.Elevation = elevationDiff / 2;
            p.Path.Steepness = maxSteepness;
            p.Path.BoundingCoordinates = boundingCoordinates;
            p.Path.Length = pathEdges.Sum( x => x.Cost );

            return p.Path;
        }

        private Path RemoveAndAddWaypoint ( List<int> waypointIdxInVisited, Problem p, Random rnd, Node newWaypoint, Tuple<int, int> predSuccwaypointIdxInVisited )
        {
            // moving = remove waypoint closest to new waypoint
            Path path = RemoveWaypoint( ref waypointIdxInVisited, p, predSuccwaypointIdxInVisited );

            p.Path = path;

            // + add in new one close to pred and succ of it 
            return AddWaypoint( waypointIdxInVisited, predSuccwaypointIdxInVisited, newWaypoint, p );
        }

        private Path AddWaypoint ( List<int> waypointIdxInVisited, Tuple<int, int> predSuccwaypointIdxInVisited, Node newWaypoint, Problem p )
        {
            throw new NotImplementedException();
        }
    }
}