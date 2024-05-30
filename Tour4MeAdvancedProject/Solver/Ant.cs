﻿using System;
using System.Collections.Generic;
using System.Device.Location;
using System.Linq;
using Tour4MeAdvancedProject.Helper;
using static Tour4MeAdvancedProject.Helper.EnumHelper;


namespace Tour4MeAdvancedProject.ObjectClasses
{
    public class Ant
    {
        //public List<Node> Solution { get; set; }

        public List<Edge> SolutionEdges { get; set; }

        // constant for calculating pheromone distribution of  this specific ant
        public int PheromoneAmount { get; set; }
        public double Alpha { get; set; } = 0.3;
        public double Beta { get; set; } = 0.7;
        public Random Random { get; set; }


        public Ant ( int pheromoneAmount )
        {
            //Solution = new List<Node>();
            SolutionEdges = new List<Edge>();
            PheromoneAmount = pheromoneAmount;
            Random = new Random();
        }

        public Ant ( int pheromoneAmount, double alpha, double beta )
        {
            //Solution = new List<Node>();
            SolutionEdges = new List<Edge>();
            PheromoneAmount = pheromoneAmount;
            Alpha = alpha;
            Beta = beta;
            Random = new Random();
        }


        /// <summary>
        /// Conducts a tour of the given problem's graph using an ant, 
        /// applying the Ant Colony Optimization (ACO) algorithm.
        /// </summary>
        /// <param name="CurrentProblem">
        /// The problem instance containing the graph and necessary parameters for the tour.
        /// </param>
        /// <param name="usePenalty">
        /// determines whether the ants should find out that closing the tour is better (usePenalty = true)
        /// or we always enforce closing the tour (usePenalty = false).
        /// </param>
        /// <returns>
        /// A list of edges representing the tour taken by the ant. 
        /// If the tour is incomplete or fails, returns null.
        /// </returns>
        /// <remarks>
        /// The Ant Colony Optimization algorithm aims to find an optimal path in a graph by 
        /// simulating the foraging behavior of ants. 
        /// This method applies the ACO algorithm to navigate the graph and construct a tour based
        /// on pheromone levels, edge visibility, and other parameters specified in the 
        /// <paramref name="CurrentProblem"/>.
        /// </remarks>
        /// <seealso cref="Problem"/>
        /// <seealso cref="Graph"/>
        /// <seealso cref="Edge"/>
        /// <seealso cref="Node"/>
        /// <seealso cref="AntColonyOptimizer"/>
        public (Problem, List<Edge>, List<int>) Tour ( Problem CurrentProblem, bool usePenalty, bool useBacktracking )
        {
            #region intialize needed variables
            SolutionEdges = new List<Edge>();
            Graph graph = CurrentProblem.Graph;
            int start = CurrentProblem.Start;
            Node[] vNodes = graph.VNodes;
            int parent = -1;

            HashSet<Node> visitableNodes = new HashSet<Node>( graph.VNodes );

            //double[] dist = new double[vNodes.Count];
            //dist[start] = 0.0;

            double[] actDist = new double[ vNodes.Length ];
            actDist[ start ] = 0.0;

            //PriorityQueue<Tuple<int, double>> queue = new PriorityQueue<Tuple<int, double>>();
            //queue.Enqueue(0.0, new Tuple<int, double>(start, 0.0));
            List<int> queue = new List<int>() { start };
            HashSet<int> visited = new HashSet<int>();
            List<int> solutionVisited = new List<int>();
            Edge pickedEdge = null;



            HashSet<SurfaceTag> addedSurfaceTags = new HashSet<SurfaceTag>();
            HashSet<HighwayTag> addedPathTypes = new HashSet<HighwayTag>();
            HashSet<string> addedSurroundings = new HashSet<string>();

            double currentDistance = 0;
            double currentEdgeProfits = 0;
            double currentArea = 0;
            double currentQuality = 0;
            _ = visited.Add( start );
            solutionVisited.Add( start );
            #endregion

            // Define the seed
            //System.Diagnostics.Debug.WriteLine( "--------------------------------------" );
            // Initialize the Random object with the seed
            //Random random = new Random( 123 );

            double currentPathsMaxSteepness = 0;
            double currentElevationDiff = 0;
            Tuple<double, double> startCoordinates = Tuple.Create( graph.VNodes[ start ].Lat, graph.VNodes[ start ].Lon );
            Tuple<double, double>[] boudingCoordinates = new Tuple<double, double>[] {
            startCoordinates , startCoordinates , startCoordinates , startCoordinates  };

            while (queue.Count > 0)
            {
                Dictionary<Edge, double> edgeProbabilities = new Dictionary<Edge, double>();
                //(double currentDist, (int currentNode, double currentActual)) = queue.Dequeue();
                int currentNode = queue.First();
                _ = queue.Remove( currentNode );


                // only look into edges where the opposite of currentNode wasn't visited yet
                // one of the nodes has to be in visited (the one we are working from now)
                // the other one may not be in visited
                // so check, if either of the nodes is not yet in visited
                List<Edge> allowed = vNodes[ currentNode ].Incident;

                FindAllowedPath( CurrentProblem, usePenalty, useBacktracking, vNodes, solutionVisited, ref visitableNodes, ref visited, pickedEdge, currentDistance, ref currentNode, ref allowed, ref currentElevationDiff );

                CalculateNewPheromoneValuesIncludeNegative( CurrentProblem, currentElevationDiff, vNodes, visited, currentDistance, currentEdgeProfits, currentArea, currentElevationDiff, ref edgeProbabilities, vNodes[ currentNode ], allowed );

                // randomly generate a random number between [0.0, 1.0) (excluding 1)
                double probability = Random.NextDouble();
                // pick an edge according to edgeProbabilities and calculated probability
                KeyValuePair<Edge, double> pickedPair = ChooseEdge( edgeProbabilities, probability );
                pickedEdge = pickedPair.Key;
                double pickedProbability = pickedPair.Value;
                if (pickedProbability == 0)
                {
                    Console.WriteLine( "ahhhhhhhhhh" );
                }

                // if we picked an edge: move to the respctive neighbor and add it to the queue (= choose next town to move to)
                if (pickedEdge != null && pickedProbability > 0)
                {
                    parent = AddEdgeAndContinue( CurrentProblem, visitableNodes, queue, visited, solutionVisited, pickedEdge, ref boudingCoordinates, ref addedSurfaceTags, ref addedPathTypes, ref addedSurroundings, ref currentPathsMaxSteepness, ref currentElevationDiff, ref currentDistance, ref currentEdgeProfits, ref currentArea, ref currentQuality, currentNode );
                    CurrentProblem.Path.BoundingCoordinates = boudingCoordinates;

                    lock (CurrentProblem.Path)
                    {
                        CurrentProblem.Path.CoveredArea = currentArea;
                    }

                    Utils.UpdateCurrentProblemPathMetadata( ref CurrentProblem, addedSurfaceTags, addedPathTypes, addedSurroundings, currentEdgeProfits, currentArea, currentQuality, currentPathsMaxSteepness, currentElevationDiff, boudingCoordinates );

                }
                else
                {
                    CloseOffPath( CurrentProblem, start, parent, ref addedSurfaceTags, ref addedPathTypes, ref addedSurroundings, ref currentPathsMaxSteepness, currentDistance, ref currentElevationDiff, ref currentEdgeProfits, ref currentArea, ref currentQuality, out List<Edge> edges, out List<int> nodes );

                    CurrentProblem.Path.Visited = solutionVisited.Concat( nodes ).ToList();
                    CurrentProblem.Path.Edges = SolutionEdges.Concat( edges ).ToList();

                    foreach (Edge edge in edges)
                    {
                        edge.Pheromone += edge.Cost * edge.Profit;
                    }

                    Utils.UpdateCurrentProblemPathMetadata( ref CurrentProblem, addedSurfaceTags, addedPathTypes, addedSurroundings, currentEdgeProfits, currentArea, currentQuality, currentPathsMaxSteepness, currentElevationDiff, boudingCoordinates );

                    return (CurrentProblem, CurrentProblem.Path.Edges, CurrentProblem.Path.Visited);
                }
            }

            Utils.UpdateCurrentProblemPathMetadata( ref CurrentProblem, addedSurfaceTags, addedPathTypes, addedSurroundings, currentEdgeProfits, currentArea, currentQuality, currentPathsMaxSteepness, currentElevationDiff, boudingCoordinates );
            //CurrentProblem.Path.CoveredArea = CurrentProblem.Path.Quality;
            return (CurrentProblem, SolutionEdges, solutionVisited);
        }

        private void FindAllowedPath ( Problem CurrentProblem, bool usePenalty, bool useBacktracking, Node[] vNodes, List<int> solutionVisited, ref HashSet<Node> visitableNodes, ref HashSet<int> visited, Edge pickedEdge, double currentDistance, ref int currentNode, ref List<Edge> allowed, ref double currentElevationDiff )
        {

            // if we can't find a next node from the one we picked, choose from the following options:
            //if (allowed.Count == 0 && currentDistance < CurrentProblem.TargetDistance)
            if (currentDistance < CurrentProblem.TargetDistance)
            {
                // if we use a penalty, the ants will figure out what to do by themselves. Just update the trail intensity accordingly
                if (usePenalty)
                {
                    allowed = ScaleTrailIntensity( vNodes, visited, currentNode, CurrentProblem, currentElevationDiff );
                }
                // if not, two more options reamain
                else if (useBacktracking)
                {
                    BackTrackToUsableSolution( pickedEdge, ref visited, ref visitableNodes, solutionVisited, ref currentNode );
                }
                // if we want to use backtracking, just step backwards through the graph and remove edges that didn't result in a working solution
                // otherwise, we allow for edges to be visited more than once, with exception of the edge we just came from
                else
                {
                    allowed = vNodes[ currentNode ].Incident;

                    if (pickedEdge != null)
                    {
                        _ = allowed.Remove( pickedEdge );
                    }
                }
            }
        }

        private void CalculateNewPheromoneValues ( ref Problem CurrentProblem, ref double currentElevationDiff, bool usePenalty, Node[] vNodes, HashSet<int> visited, double currentDistance, Dictionary<Edge, float> edgeProbabilities, Node currentNode, List<Edge> allowed, float sumOfAllowed, double profit, double area, double elevation, out double profitOut, out double areaOut, out double elevationOut )
        {
            List<Edge> scaledEdges = ScaleTrailIntensity( vNodes, visited, currentNode.GraphNodeId, CurrentProblem, currentElevationDiff );
            profitOut = profit;
            areaOut = area;

            double newProfit = profit;
            double newArea = area;
            double newElevation = elevation;

            foreach (Edge edge in allowed)
            {
                Node neighbor = edge.SourceNode == currentNode ? edge.TargetNode : edge.SourceNode;

                //if (neighbor.ShortestDistance == 0)
                //{
                //    neighbor.ShortestDistance = CurrentProblem.Graph.ShortestPath( CurrentProblem.Start, neighbor.GraphNodeId );
                //}

                if (neighbor.ShortestDistance < CurrentProblem.TargetDistance - currentDistance - edge.Cost)
                {
                    newProfit = profit + ( edge.Cost * edge.Profit );
                    newArea = area + ( edge.SourceNode == currentNode ? edge.ShoelaceForward : edge.ShoelaceBackward );
                    newElevation = elevation + ( Math.Abs( edge.SourceNode.Elevation - edge.TargetNode.Elevation ) / 2 );

                    //edge.Quality = CurrentProblem.GetEdgeQuality( newProfit, newArea, newElevation );

                    //double edgeValue = edge.Cost * edge.TrailIntensity;
                    //double edgeVisibility = 1 / edge.Cost;
                    double edgeVisibility = CurrentProblem.GetEdgeQuality( newProfit, newArea, newElevation );

                    int negModifier = newArea < 0 ? -1 : 1;

                    // p_{ij}^k (used for choosing town to move to)
                    float currentProbability = (float)( (float)Math.Pow( scaledEdges.Find( x => x.Id == edge.Id ).TrailIntensity, Alpha ) *
                        negModifier * (float)Math.Pow( negModifier * edgeVisibility, Beta ) / sumOfAllowed );


                    edgeProbabilities.Add( edge, currentProbability );

                    // delta t_{ij}^k (PheromoneAmount / edge.Cost = Q / L_k) & update of delta t_ij (edge.Pheromone)
                    edge.Pheromone += PheromoneAmount * edge.Profit * edge.Cost;
                    if (edge.Pheromone < double.MinValue || edge.Pheromone > double.MaxValue)
                    {
                        Console.WriteLine( "ahhh" );
                    }
                }
                else
                {
                    edge.Pheromone /= 1000;
                    if (edge.Pheromone < double.MinValue || edge.Pheromone > double.MaxValue)
                    {
                        Console.WriteLine( "ahhh" );
                    }
                }

                if (neighbor.Incident.Count == 1 || currentNode.Incident.Count == 1)
                {
                    edge.Pheromone /= 1000;
                    if (edge.Pheromone < double.MinValue || edge.Pheromone > double.MaxValue)
                    {
                        Console.WriteLine( "ahhh" );
                    }
                }
            }

            // Normalize probabilities
            float sumOfProbabilities = edgeProbabilities.Values.Sum();
            foreach (Edge edge in edgeProbabilities.Keys.ToList())
            {
                edgeProbabilities[ edge ] /= sumOfProbabilities;
            }

            profitOut = newProfit;
            areaOut = newArea;
            elevationOut = newElevation;
        }

        private void CalculateNewPheromoneValuesIncludeNegative ( Problem CurrentProblem, double currentElevationDiff, Node[] vNodes, HashSet<int> visited, double currentDistance, double profit, double area, double elevation, ref Dictionary<Edge, double> edgeProbabilities, Node currentNode, List<Edge> allowed )
        {

            List<Edge> scaledEdges = ScaleTrailIntensity( vNodes, visited, currentNode.GraphNodeId, CurrentProblem, currentElevationDiff );

            //double profit = 0;
            //double area = 0;
            //double elevation = 0;

            double newProfit = 0;
            double newArea = 0;
            double newElevation = 0;
            double newDistance = 0;


            for (int i = 0; i < allowed.Count; i++)
            {
                Edge edge = allowed[ i ];
                Node neighbor = edge.SourceNode == currentNode ? edge.TargetNode : edge.SourceNode;


                if (neighbor.ShortestDistance < CurrentProblem.TargetDistance - currentDistance - edge.Cost)
                {
                    double edgeVisibility = 0;
                    //if ()
                    //{
                    //edgeVisibility = edge.Cost * edge.Profit * CurrentProblem.EdgeProfitImportance;
                    newProfit = profit + ( !edge.Visited ? edge.Cost * edge.Profit : 0 );
                    //}
                    newArea = area + ( edge.SourceNode == currentNode ? edge.ShoelaceForward : edge.ShoelaceBackward );
                    newElevation = elevation + Math.Abs( edge.SourceNode.Elevation - edge.TargetNode.Elevation );
                    newDistance = currentDistance + edge.Cost;

                    //edge.Quality = CurrentProblem.GetQuality( newProfit, newArea, newElevation, newDistance );
                    //edge.Quality = CurrentProblem.GetQuality( profit, area, elevation, );
                    //edgeVisibility = edge.Quality;
                    edgeVisibility = CurrentProblem.GetEdgeQuality( newProfit, newArea, newElevation );

                    int negModifier = area < 0 ? -1 : 1;
                    int visNegModifier = edgeVisibility < 0 ? -1 : 1;

                    //float transformedValue = (float)( Math.Exp( Math.Abs( edgeVisibility ) ) * negModifier );

                    //// Calculate probabilities without normalization
                    //float currentProbability = (float)( Math.Pow( scaledEdges.Find( x => x.Id == edge.Id ).TrailIntensity, Alpha ) *
                    //                                    transformedValue );

                    double currentProbability = Math.Pow( scaledEdges.Find( x => x.Id == edge.Id ).TrailIntensity, Alpha ) * Math.Pow( edgeVisibility * visNegModifier, Beta );

                    edgeProbabilities.Add( edge, currentProbability );

                    if (currentProbability != currentProbability || currentProbability == 0 || currentProbability > double.MaxValue || currentProbability < double.MinValue)
                    {
                        Console.WriteLine( "ahhhh" );
                    }
                    if (edge.Pheromone < double.MinValue || edge.Pheromone > double.MaxValue)
                    {
                        Console.WriteLine( "ahhh" );
                    }
                }
                else
                {
                    edge.Pheromone /= 1000;
                    if (edge.Pheromone < double.MinValue || edge.Pheromone > double.MaxValue)
                    {
                        Console.WriteLine( "ahhh" );
                    }
                }

                if (neighbor.Incident.Count == 1 || currentNode.Incident.Count == 1)
                {
                    edge.Pheromone /= 1000;
                    if (edge.Pheromone < double.MinValue || edge.Pheromone > double.MaxValue)
                    {
                        Console.WriteLine( "ahhh" );
                    }
                }
            }

            // Normalize probabilities
            // p_{ij}^k (used for choosing town to move to)
            double sumOfProbabilities = edgeProbabilities.Values.Sum();
            if (sumOfProbabilities == 0 || sumOfProbabilities < double.MinValue || sumOfProbabilities > double.MaxValue)
            {
                sumOfProbabilities = 1;
                for (int i = 0; i < edgeProbabilities.Keys.ToList().Count; i++)
                {
                    Edge edge = edgeProbabilities.ElementAt( i ).Key;
                    edgeProbabilities[ edge ] = 1 / edgeProbabilities.Count;
                }
                Console.WriteLine( "ahhh" );
            }
            for (int i = 0; i < edgeProbabilities.Keys.ToList().Count; i++)
            {
                Edge edge = edgeProbabilities.ElementAt( i ).Key;
                edgeProbabilities[ edge ] /= sumOfProbabilities;
            }

        }
        private static List<Edge> ScaleTrailIntensity ( Node[] vNodes, HashSet<int> visited, int currentNode, Problem currentProblem, double currentElevationDiff )
        {
            // scale trial intensity on edges accordingly for all edges where a node is visited twice
            double penaltyQuotient = currentProblem.TargetDistance / 100;

            //double maxSteepness = currentProblem.MaxSteepness;
            //double elevationDiff = currentElevationDiff;
            //double maxElevationDiff = currentProblem.MaxElevation;


            //List<Edge> applyElevationPenalty = vNodes[ currentNode ].Incident.FindAll( x =>
            //( elevationDiff + Math.Abs( x.TargetNode.Elevation - x.SourceNode.Elevation ) ) / 2 > maxElevationDiff ||
            //Math.Abs( x.TargetNode.Elevation - x.SourceNode.Elevation ) / x.Cost * 100 > maxSteepness
            //);
            List<Edge> allowed = new List<Edge>( vNodes[ currentNode ].Incident );

            //foreach (Edge edge in applyElevationPenalty)
            //{
            //    Edge newEdge = new Edge( edge, edge.Reversed );
            //    newEdge.TrailIntensity /= currentProblem.ElevationImportance == 0 ? 1 : ( penaltyQuotient * currentProblem.ElevationImportance );
            //    _ = allowed.Remove( edge );
            //    allowed.Add( newEdge );
            //}

            penaltyQuotient = 10 * currentProblem.TargetDistance;
            Edge[] applyDoubleVisitPenalty = vNodes[ currentNode ].Incident.FindAll( x =>
            visited.Contains( x.TargetNode.GraphNodeId ) && visited.Contains( x.SourceNode.GraphNodeId ) ).ToArray();

            for (int i = 0; i < applyDoubleVisitPenalty.Length; i++)
            {
                Edge edge = applyDoubleVisitPenalty[ i ];
                // create new list of edges with applied penalties
                Edge newEdge = new Edge( edge, edge.Reversed );
                newEdge.TrailIntensity /= penaltyQuotient;
                if (newEdge.TrailIntensity < double.MinValue || edge.TrailIntensity > double.MaxValue)
                {
                    Console.WriteLine( "ahhh" );
                }
                _ = allowed.Remove( edge );
                allowed.Add( newEdge );
            }
            //allowed = applyPenalty;
            return allowed;
        }

        private float PrecomputeSumOverAllAllowedEdges ( List<Edge> allowed, Node[] vNodes, HashSet<int> visited, int currentNode, double currentDistance, double profit, double area, double elevation, ref Problem currentProblem, ref double currentElevationDiff )
        {

            List<Edge> scaledEdges = ScaleTrailIntensity( vNodes, visited, currentNode, currentProblem, currentElevationDiff );
            // precompute sum of (trailintensity^alpha * edge visibility^beta) over all allowed edges 
            float sumOfAllowed = 0;
            double newProfit = profit;
            double newArea = area;
            double newElevation = elevation;
            for (int i = 0; i < allowed.Count; i++)
            {
                Edge edge = allowed[ i ];
                newProfit = profit + ( edge.Cost * edge.Profit );
                newArea = area + ( edge.ShoelaceForward >= 0 ? edge.ShoelaceForward : edge.ShoelaceBackward );
                newElevation = elevation + ( Math.Abs( edge.SourceNode.Elevation - edge.TargetNode.Elevation ) / 2 );

                //edge.Quality = currentProblem.GetEdgeQuality( newProfit, newArea, newElevation );
                float trailIntensityPowAlpha = (float)Math.Pow( scaledEdges.Find( x => x.Id == edge.Id ).TrailIntensity, Alpha );
                double edgeVisibility = currentProblem.GetEdgeQuality( newProfit, newArea, newElevation ) * currentProblem.EdgeProfitImportance;
                float visibilityPowBeta = (float)Math.Pow( edgeVisibility, Beta );

                sumOfAllowed += trailIntensityPowAlpha * visibilityPowBeta;
            }

            return sumOfAllowed;
        }

        private int AddEdgeAndContinue ( Problem problem, HashSet<Node> visitableNodes, List<int> queue, HashSet<int> visited, List<int> solutionVisited, Edge pickedEdge, ref Tuple<double, double>[] boundingCoordinates, ref HashSet<SurfaceTag> addedSurfaceTags, ref HashSet<HighwayTag> addedPathTypes, ref HashSet<string> addedSurroundings, ref double maxSteepness, ref double currentElevationDiff, ref double currentDistance, ref double currentEdgeProfits, ref double currentArea, ref double currentQuality, int currentNode )
        {
            int parent;
            // now choose next node based on probability
            Node neighbor = pickedEdge.SourceNode.GraphNodeId == currentNode ? pickedEdge.TargetNode : pickedEdge.SourceNode;
            int neighborId = neighbor.GraphNodeId;
            currentDistance += pickedEdge.Cost;

            queue.Add( neighborId );
            parent = currentNode;
            // once a node was visited, it cannot be visited again in this tour by the same ant
            _ = visited.Add( neighborId );
            solutionVisited.Add( neighborId );
            _ = visitableNodes.Remove( neighbor );

            // delta t_{ij}^k (PheromoneAmount / edge.Cost = Q / L_k) & update of delta t_ij (edge.Pheromone)
            //edge.Pheromone += PheromoneAmount * edge.Quality;
            pickedEdge.Pheromone += PheromoneAmount * pickedEdge.Cost * pickedEdge.Profit;


            // Add the picked edge to the solution path
            SolutionEdges.Add( pickedEdge );

            lock (boundingCoordinates)
            {
                problem.Path.UpdateBoundingCoordinates( ref boundingCoordinates, neighbor );
            }
            for (int i = 0; i < pickedEdge.Tags.Count; i++)
            {
                string currentTag = pickedEdge.Tags[ i ];
                Utils.AddTags( ref addedSurfaceTags, ref addedPathTypes, ref addedSurroundings, currentTag );
            }

            Utils.CalculateElevationDiffAndSteepness( pickedEdge, ref maxSteepness, ref currentElevationDiff );
            Utils.CaculateQualityValues( problem, pickedEdge, currentNode, currentDistance, currentElevationDiff, ref currentEdgeProfits, ref currentArea, ref currentQuality );
            pickedEdge.Visited = true;


            return parent;
        }


        private void CloseOffPath ( Problem problem, int start, int parent, ref HashSet<SurfaceTag> addedSurfaceTags, ref HashSet<HighwayTag> addedPathTypes, ref HashSet<string> addedSurroundings, ref double maxSteepness, double currentDistance, ref double currentElevationDiff, ref double currentEdgeProfits, ref double currentArea, ref double currentQuality, out List<Edge> edges, out List<int> nodes )
        {
            // if we need to close the loop since we cannot take any other neighbor:
            // close it using the shortest path from the remaining acceptable node
            edges = new List<Edge>();
            nodes = new List<int>();
            //(edges, nodes) = graph.GetShortestPath( parent, start );
            (edges, nodes) = problem.Graph.DijkstraShortestPath( start, parent );
            edges.Reverse();
            nodes.Reverse();

            int i = 0;
            for (int j = 0; j < edges.Count; j++)
            {
                Edge edge = edges[ j ];
                int startNodeId = nodes[ i ];
                i++;
                for (int x = 0; x < edge.Tags.Count; x++)
                {
                    string currentTag = edge.Tags[ x ];
                    Utils.AddTags( ref addedSurfaceTags, ref addedPathTypes, ref addedSurroundings, currentTag );
                }
                Utils.CalculateElevationDiffAndSteepness( edge, ref maxSteepness, ref currentElevationDiff );
                Utils.CaculateQualityValues( problem, edge, startNodeId, currentDistance + edge.Cost, currentElevationDiff, ref currentEdgeProfits, ref currentArea, ref currentQuality );

            }
            edges.ForEach( x => x.Visited = false );

            double sum = SolutionEdges.Concat( edges ).ToList().Sum( x => x.Cost );
        }

        private void BackTrackToUsableSolution ( Edge pickedEdge, ref HashSet<int> visited, ref HashSet<Node> visitableNodes, List<int> solutionVisited, ref int currentNode )
        {
            if (pickedEdge != null)
            {
                Node neighbor = pickedEdge.SourceNode.GraphNodeId == currentNode ? pickedEdge.TargetNode : pickedEdge.SourceNode;
                int neighborId = neighbor.GraphNodeId;

                // undo the changes made due to picking this edge
                _ = visited.Remove( neighborId );
                _ = visitableNodes.Add( neighbor );
                _ = SolutionEdges.Remove( pickedEdge );

                // make edge impossible to pick again
                // setting trailIntensity to 0 makes the probability of this edge being picked 0
                pickedEdge.TrailIntensity = 0;
            }
        }


        /// <summary>
        /// Updates the pheromone trail on edges in the graph based on evaporation and 
        /// newly placed pheromones.
        /// </summary>
        /// <param name="problem">The graph containing edges with pheromone trails and trail intensities.</param>
        /// <param name="visited">contains a list of all edges the ant has visited for its solution.</param>
        /// <param name="usePenalty">
        /// determines whether the ants should find out that closing the tour is better (usePenalty = true)
        /// or we always enforce closing the tour (usePenalty = false).
        /// </param>
        public void UpdatePheromoneTrail ( Problem problem, List<Edge> visited, double evaporation, bool usePenalty, bool inclueAreaCoverage, double penalty )
        {
            double evaporationRate = evaporation;

            // default set to 1 because that means no addition or reduction of pheromone values to be added
            double penaltyQuotient = penalty;

            bool closedTour = visited.First().Equals( visited.Last() );
            double sum = visited.Sum( x => x.Cost );

            List<Edge> acceptableLentghEdges = new List<Edge>( visited );

            //while (sum > problem.TargetDistance)
            //{
            //    // pick last edge of the path
            //    Edge penaltyEdge = acceptableLentghEdges.Last();
            //    // set trail intensity to 0 to make it as unattractive as possible for future runs
            //    penaltyEdge.TrailIntensity = 0.1;

            //    // remove last edge that added too much to the distance and re-calculate sum
            //    _ = acceptableLentghEdges.Remove( penaltyEdge );
            //    sum = acceptableLentghEdges.Sum( x => x.Cost );

            //}

            if (usePenalty && !closedTour)
            {
                // if we didn't close the tour, use a high quotient as penalty
                // this lowers the increase in the trail intensity heavily for future runs
                penaltyQuotient = 100;
            }

            if (inclueAreaCoverage)
            {
                lock (problem.Path)
                {

                    GeoCoordinate rCoord = new GeoCoordinate( problem.Path.BoundingCoordinates[ 3 ].Item1, problem.Path.BoundingCoordinates[ 3 ].Item2 );
                    GeoCoordinate lCoord = new GeoCoordinate( problem.Path.BoundingCoordinates[ 0 ].Item1, problem.Path.BoundingCoordinates[ 0 ].Item2 );

                    double r1 = lCoord.GetDistanceTo( rCoord );

                    GeoCoordinate uCoord = new GeoCoordinate( problem.Path.BoundingCoordinates[ 2 ].Item1, problem.Path.BoundingCoordinates[ 2 ].Item2 );
                    GeoCoordinate dCoord = new GeoCoordinate( problem.Path.BoundingCoordinates[ 1 ].Item1, problem.Path.BoundingCoordinates[ 1 ].Item2 );

                    double r2 = uCoord.GetDistanceTo( dCoord );

                    double perfectEllipsoidArea = Math.PI * ( r1 / 2 ) * ( r2 / 2 );
                    double boundingArea = r1 * r2;

                    double perfectDiff = boundingArea - perfectEllipsoidArea;

                    double actualDiff = boundingArea - ( ( problem.Path.CoveredArea < 0 ? -1 : 1 ) * problem.Path.CoveredArea );
                    if (boundingArea > 0)
                    {
                        penaltyQuotient *= ( problem.CoveredAreaImportance > 0 ? problem.CoveredAreaImportance : 0.0001 * Math.Abs( actualDiff - perfectDiff ) ) * problem.TargetDistance;
                    }

                }
                //double quality = 0;
                //foreach (Edge edge in visited)
                //{
                //    quality += edge.Quality;
                //}
                //// the bigger the area covered, the smaller our penalty needs to be
                //penaltyQuotient *= problem.CoveredAreaImportance * problem.Path.CoveredArea / ( Math.PI * problem.TargetDistance * problem.TargetDistance );
            }
            for (int i = 0; i < problem.Graph.VEdges.Length; i++)
            {
                Edge edge = problem.Graph.VEdges[ i ];
                if (edge != null)
                {
                    edge.TrailIntensity = ( edge.TrailIntensity * evaporationRate ) + edge.Pheromone;// / penaltyQuotient );
                }
            }
            //for (int i = 0; i < visited.Count; i++)
            //{
            //    Edge visitedEdge = visited[ i ];
            //    // update actual edge of the graph
            //    Edge edge = problem.Graph.VEdges[ visitedEdge.GraphId ];

            //    // trailIntensity contains the actual pheromone left on the trail
            //    // evaporationRate descibes how much of the trail has evaporated during one tour
            //    // pheromone contains the newly placed pheromone from the current ants (updated during tour)
            //    // scale this by a penalty if the ant didn't close the tour
            //    edge.TrailIntensity = ( edge.TrailIntensity * evaporationRate ) + ( edge.Pheromone / penaltyQuotient );
            //    if (edge.TrailIntensity < double.MinValue || edge.TrailIntensity > double.MaxValue)
            //    {
            //        Console.WriteLine( "ahhh" );
            //    }
            //}
        }

        /// <summary>
        /// Generates a random choice based on given probabilities (float) and their respective edge id (int).
        /// </summary>
        /// <param name="probabilities">Array of probabilities for each choice.</param>
        /// <param name="randomNumber"> the random number that was generated.</param>
        /// <returns>The randomly selected choice and its respective edge id.</returns>
        private KeyValuePair<Edge, double> ChooseEdge ( Dictionary<Edge, double> probabilities, double randomNumber )
        {
            if (probabilities.Count == 1)
            {
                return probabilities.First();
            }
            // start with percentage 0
            double sum = 0;
            // step through all possible neighbor edges
            for (int i = 0; i < probabilities.Count; i++)
            {
                KeyValuePair<Edge, double> kvp = probabilities.ElementAt( i );
                // get the current edge percentage
                double percentage = kvp.Value;
                // add this to the previous percentage
                sum += percentage;
                // only if percentage is larger than the random number we generated earlier, 
                // choose this edge 
                if (sum > randomNumber)
                {
                    return kvp;
                }
            }
            // last walk through the loop shouold always check against 100% and thus always return something
            // if nothing was returned, we had an error case
            // represent this with kvp (-1, -1) which is not posible
            return new KeyValuePair<Edge, double>( null, -1 );
        }
    }
}