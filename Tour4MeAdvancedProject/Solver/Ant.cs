using System;
using System.Collections.Generic;
using System.Device.Location;
using System.Linq;
using Tour4MeAdvancedProject.Helper;


namespace Tour4MeAdvancedProject.ObjectClasses
{
    public class Ant
    {
        //public List<Node> Solution { get; set; }

        public List<Edge> SolutionEdges { get; set; }

        // constant for calculating pheromone distribution of  this specific ant
        public int PheromoneAmount { get; set; }
        public double Alpha { get; set; } = 0.5;
        public double Beta { get; set; } = 0.5;
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
        public (List<Edge>, List<int>) Tour ( ref Problem CurrentProblem, bool usePenalty, bool useBacktracking )
        {
            #region intialize needed variables
            SolutionEdges = new List<Edge>();
            Graph graph = CurrentProblem.Graph;
            int start = CurrentProblem.Start;
            List<Node> vNodes = graph.VNodes;
            int parent = -1;

            List<Node> visitableNodes = new List<Node>( graph.VNodes );

            //double[] dist = new double[vNodes.Count];
            //dist[start] = 0.0;

            double[] actDist = new double[ vNodes.Count ];
            actDist[ start ] = 0.0;

            double profit = 0;
            double area = 0;

            //PriorityQueue<Tuple<int, double>> queue = new PriorityQueue<Tuple<int, double>>();
            //queue.Enqueue(0.0, new Tuple<int, double>(start, 0.0));
            List<int> queue = new List<int>() { start };
            HashSet<int> visited = new HashSet<int>();
            List<int> solutionVisited = new List<int>();
            Edge pickedEdge = null;

            double currentDistance = 0;
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
                Dictionary<Edge, float> edgeProbabilities = new Dictionary<Edge, float>();
                //(double currentDist, (int currentNode, double currentActual)) = queue.Dequeue();
                int currentNode = queue.First();
                _ = queue.Remove( currentNode );


                // only look into edges where the opposite of currentNode wasn't visited yet
                // one of the nodes has to be in visited (the one we are working from now)
                // the other one may not be in visited
                // so check, if either of the nodes is not yet in visited
                _ = CurrentProblem.MaxSteepness;
                _ = CurrentProblem.MaxElevation;
                List<Edge> allowed = vNodes[ currentNode ].Incident;
                //    .FindAll( x =>
                //  ( !visited.Contains( x.TargetNode.GraphNodeId ) || !visited.Contains( x.SourceNode.GraphNodeId ) ) &&
                //  ( ( currentElevationDiff + Math.Abs( x.TargetNode.Elevation - x.SourceNode.Elevation ) ) / 2 > maxAllowedElevationDiff ||
                //  Math.Abs( x.TargetNode.Elevation - x.SourceNode.Elevation ) / x.Cost * 100 > maxAllowedSteepness )
                //);

                FindAllowedPath( ref CurrentProblem, usePenalty, useBacktracking, vNodes, ref visitableNodes, ref visited, pickedEdge, currentDistance, ref currentNode, ref allowed, ref currentElevationDiff );

                //float sumOfAllowed = PrecomputeSumOverAllAllowedEdges( allowed, vNodes, visited, currentNode, profit, area, ref CurrentProblem, ref currentElevationDiff );
                //CalculateNewPheromoneValues( ref CurrentProblem, ref currentElevationDiff, usePenalty, vNodes, visited, currentDistance, edgeProbabilities, vNodes[ currentNode ], allowed, sumOfAllowed, profit, area, out profit, out area );

                CalculateNewPheromoneValuesIncludeNegative( ref CurrentProblem, ref currentElevationDiff, vNodes, visited, currentDistance, ref edgeProbabilities, vNodes[ currentNode ], allowed, profit, area, out profit, out area );


                lock (CurrentProblem.Path)
                {
                    CurrentProblem.Path.CoveredArea = area;
                }
                // randomly generate a random number between [0.0, 1.0) (excluding 1)
                float probability = (float)Random.NextDouble();
                // pick an edge according to edgeProbabilities and calculated probability
                KeyValuePair<Edge, float> pickedPair = ChooseEdge( edgeProbabilities, probability );
                pickedEdge = pickedPair.Key;
                float pickedProbability = pickedPair.Value;

                // if we picked an edge: move to the respctive neighbor and add it to the queue (= choose next town to move to)
                if (pickedEdge != null && pickedProbability > 0)
                {
                    parent = AddEdgeAndContinue( visitableNodes, queue, visited, solutionVisited, pickedEdge, ref boudingCoordinates, ref currentPathsMaxSteepness, ref currentElevationDiff, ref currentDistance, currentNode );
                    CurrentProblem.Path.BoundingCoordinates = boudingCoordinates;
                }
                else
                {
                    CloseOffPath( graph, start, parent, ref currentPathsMaxSteepness, ref currentElevationDiff, out List<Edge> edges, out List<int> nodes );

                    CurrentProblem.Path.Steepness = currentPathsMaxSteepness;
                    CurrentProblem.Path.Elevation = currentElevationDiff;
                    return (SolutionEdges.Concat( edges ).ToList(), solutionVisited.Concat( nodes ).ToList());
                }
            }

            CurrentProblem.Path.Steepness = currentPathsMaxSteepness;
            return (SolutionEdges, solutionVisited);
        }

        private void FindAllowedPath ( ref Problem CurrentProblem, bool usePenalty, bool useBacktracking, List<Node> vNodes, ref List<Node> visitableNodes, ref HashSet<int> visited, Edge pickedEdge, double currentDistance, ref int currentNode, ref List<Edge> allowed, ref double currentElevationDiff )
        {

            // if we can't find a next node from the one we picked, choose from the following options:
            //if (allowed.Count == 0 && currentDistance < CurrentProblem.TargetDistance)
            if (currentDistance < CurrentProblem.TargetDistance)
            {
                // if we use a penalty, the ants will figure out what to do by themselves. Just update the trail intensity accordingly
                if (usePenalty)
                {
                    allowed = ScaleTrailIntensity( vNodes, visited, currentNode, ref CurrentProblem, ref currentElevationDiff );
                }
                // if not, two more options reamain
                else if (useBacktracking)
                {
                    BackTrackToUsableSolution( pickedEdge, visited, visitableNodes, currentNode, out visited, out visitableNodes, out currentNode );
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

        private void CalculateNewPheromoneValues ( ref Problem CurrentProblem, ref double currentElevationDiff, bool usePenalty, List<Node> vNodes, HashSet<int> visited, double currentDistance, Dictionary<Edge, float> edgeProbabilities, Node currentNode, List<Edge> allowed, float sumOfAllowed, double profit, double area, out double profitOut, out double areaOut )
        {
            List<Edge> scaledEdges = ScaleTrailIntensity( vNodes, visited, currentNode.GraphNodeId, ref CurrentProblem, ref currentElevationDiff );
            profitOut = profit;
            areaOut = area;

            double newProfit = profit;
            double newArea = area;

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

                    edge.Quality = CurrentProblem.GetQuality( newProfit, newArea );

                    //double edgeValue = edge.Cost * edge.TrailIntensity;
                    //double edgeVisibility = 1 / edge.Cost;
                    double edgeVisibility = edge.Quality * CurrentProblem.EdgeProfitImportance;

                    int negModifier = newArea < 0 ? -1 : 1;

                    // p_{ij}^k (used for choosing town to move to)
                    float currentProbability = (float)( (float)Math.Pow( scaledEdges.Find( x => x.Id == edge.Id ).TrailIntensity, Alpha ) *
                        negModifier * (float)Math.Pow( negModifier * edgeVisibility, Beta ) / sumOfAllowed );


                    edgeProbabilities.Add( edge, currentProbability );

                    // delta t_{ij}^k (PheromoneAmount / edge.Cost = Q / L_k) & update of delta t_ij (edge.Pheromone)
                    edge.Pheromone += PheromoneAmount * edge.Profit * edge.Cost;
                }
                else
                {
                    edge.Pheromone /= 1000;
                }

                if (neighbor.Incident.Count == 1 || currentNode.Incident.Count == 1)
                {
                    edge.Pheromone /= 1000;
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
        }

        private void CalculateNewPheromoneValuesIncludeNegative ( ref Problem CurrentProblem, ref double currentElevationDiff, List<Node> vNodes, HashSet<int> visited, double currentDistance, ref Dictionary<Edge, float> edgeProbabilities, Node currentNode, List<Edge> allowed, double profit, double area, out double profitOut, out double areaOut )
        {

            List<Edge> scaledEdges = ScaleTrailIntensity( vNodes, visited, currentNode.GraphNodeId, ref CurrentProblem, ref currentElevationDiff );
            profitOut = profit;
            areaOut = area;

            double newProfit = profit;
            double newArea = area;

            foreach (Edge edge in allowed)
            {
                Node neighbor = edge.SourceNode == currentNode ? edge.TargetNode : edge.SourceNode;


                if (neighbor.ShortestDistance < CurrentProblem.TargetDistance - currentDistance - edge.Cost)
                {
                    newProfit = profit + ( edge.Cost * edge.Profit );
                    newArea = area + ( edge.SourceNode == currentNode ? edge.ShoelaceForward : edge.ShoelaceBackward );


                    edge.Quality = CurrentProblem.GetQuality( newProfit, newArea );
                    double edgeVisibility = edge.Quality * CurrentProblem.EdgeProfitImportance;

                    int negModifier = newArea < 0 ? -1 : 1;


                    float transformedValue = (float)( Math.Exp( Math.Abs( edgeVisibility ) ) * negModifier );

                    // Calculate probabilities without normalization
                    float currentProbability = (float)( Math.Pow( scaledEdges.Find( x => x.Id == edge.Id ).TrailIntensity, Alpha ) *
                                                        transformedValue );
                    edgeProbabilities.Add( edge, currentProbability );

                    // delta t_{ij}^k (PheromoneAmount / edge.Cost = Q / L_k) & update of delta t_ij (edge.Pheromone)
                    edge.Pheromone += PheromoneAmount * edge.Profit * edge.Cost;
                }
                else
                {
                    edge.Pheromone /= 1000;
                }

                if (neighbor.Incident.Count == 1 || currentNode.Incident.Count == 1)
                {
                    edge.Pheromone /= 1000;
                }
            }

            // Normalize probabilities
            // p_{ij}^k (used for choosing town to move to)
            float sumOfProbabilities = edgeProbabilities.Values.Sum();
            foreach (Edge edge in edgeProbabilities.Keys.ToList())
            {
                edgeProbabilities[ edge ] /= sumOfProbabilities;
            }

            IOrderedEnumerable<KeyValuePair<Edge, float>> sortedProbabilities = from entry in edgeProbabilities orderby entry.Value ascending select entry;
            edgeProbabilities = sortedProbabilities.ToDictionary( pair => pair.Key, pair => pair.Value );
            profitOut = newProfit;
            areaOut = newArea;
        }
        private static List<Edge> ScaleTrailIntensity ( List<Node> vNodes, HashSet<int> visited, int currentNode, ref Problem currentProblem, ref double currentElevationDiff )
        {
            // scale trial intensity on edges accordingly for all edges where a node is visited twice
            double penaltyQuotient = 1000;
            List<Edge> applyDoubleVisitPenalty = vNodes[ currentNode ].Incident.FindAll( x =>
            visited.Contains( x.TargetNode.GraphNodeId ) && visited.Contains( x.SourceNode.GraphNodeId ) );

            double maxSteepness = currentProblem.MaxSteepness;
            double elevationDiff = currentElevationDiff;
            double maxElevationDiff = currentProblem.MaxElevation;


            List<Edge> applyElevationPenalty = vNodes[ currentNode ].Incident.FindAll( x =>
            ( elevationDiff + Math.Abs( x.TargetNode.Elevation - x.SourceNode.Elevation ) ) / 2 > maxElevationDiff ||
            Math.Abs( x.TargetNode.Elevation - x.SourceNode.Elevation ) / x.Cost * 100 > maxSteepness
            );
            List<Edge> allowed = new List<Edge>( vNodes[ currentNode ].Incident );

            foreach (Edge edge in applyDoubleVisitPenalty)
            {
                // create new list of edges with applied penalties
                Edge newEdge = new Edge( edge, edge.Reversed );
                newEdge.TrailIntensity /= penaltyQuotient;
                _ = allowed.Remove( edge );
                allowed.Add( newEdge );
            }

            foreach (Edge edge in applyElevationPenalty)
            {
                Edge newEdge = new Edge( edge, edge.Reversed );
                newEdge.TrailIntensity /= currentProblem.ElevationImportance == 0 ? 1 : penaltyQuotient * ( 1 / currentProblem.ElevationImportance );
                _ = allowed.Remove( edge );
                allowed.Add( newEdge );
            }

            //allowed = applyPenalty;
            return allowed;
        }

        private float PrecomputeSumOverAllAllowedEdges ( List<Edge> allowed, List<Node> vNodes, HashSet<int> visited, int currentNode, double profit, double area, ref Problem currentProblem, ref double currentElevationDiff )
        {

            List<Edge> scaledEdges = ScaleTrailIntensity( vNodes, visited, currentNode, ref currentProblem, ref currentElevationDiff );
            // precompute sum of (trailintensity^alpha * edge visibility^beta) over all allowed edges 
            float sumOfAllowed = 0;
            double newProfit = profit;
            double newArea = area;
            foreach (Edge edge in allowed)
            {
                newProfit = profit + ( edge.Cost * edge.Profit );
                newArea = area + ( edge.ShoelaceForward >= 0 ? edge.ShoelaceForward : edge.ShoelaceBackward );

                edge.Quality = currentProblem.GetQuality( newProfit, newArea );
                float trailIntensityPowAlpha = (float)Math.Pow( scaledEdges.Find( x => x.Id == edge.Id ).TrailIntensity, Alpha );
                double edgeVisibility = edge.Quality * currentProblem.EdgeProfitImportance;
                float visibilityPowBeta = (float)Math.Pow( edgeVisibility, Beta );

                sumOfAllowed += trailIntensityPowAlpha * visibilityPowBeta;
            }

            return sumOfAllowed;
        }

        private int AddEdgeAndContinue ( List<Node> visitableNodes, List<int> queue, HashSet<int> visited, List<int> solutionVisited, Edge pickedEdge, ref Tuple<double, double>[] boundingCoordinates, ref double maxSteepness, ref double currentElevationDfif, ref double currentDistance, int currentNode )
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
            _ = visitableNodes.Remove( visitableNodes.Find( x => x != null && x.GraphNodeId == neighborId ) );

            // Add the picked edge to the solution path
            SolutionEdges.Add( pickedEdge );
            lock (boundingCoordinates)
            {

                // insert the 4 boudning coordinates
                // left
                if (boundingCoordinates[ 0 ].Item1 > neighbor.Lat)
                {
                    boundingCoordinates[ 0 ] = Tuple.Create( neighbor.Lat, neighbor.Lon );
                }
                // right
                else if (boundingCoordinates[ 3 ].Item1 < neighbor.Lat)
                {
                    boundingCoordinates[ 3 ] = Tuple.Create( neighbor.Lat, neighbor.Lon );
                }
                // top
                else if (boundingCoordinates[ 2 ].Item2 > neighbor.Lon)
                {
                    boundingCoordinates[ 2 ] = Tuple.Create( neighbor.Lat, neighbor.Lon );
                }
                //bottom
                else if (boundingCoordinates[ 1 ].Item2 < neighbor.Lon)
                {
                    boundingCoordinates[ 1 ] = Tuple.Create( neighbor.Lat, neighbor.Lon );
                }
            }

            Utils.CalculateElevationDiffAndSteepness( pickedEdge, ref maxSteepness, ref currentElevationDfif );

            return parent;
        }

        private void CloseOffPath ( Graph graph, int start, int parent, ref double maxSteepness, ref double currentElevationDfif, out List<Edge> edges, out List<int> nodes )
        {
            // if we need to close the loop since we cannot take any other neighbor:
            // close it using the shortest path from the remaining acceptable node
            edges = new List<Edge>();
            nodes = new List<int>();
            //(edges, nodes) = graph.GetShortestPath( parent, start );
            (edges, nodes) = graph.DijkstraShortestPath( start, parent );
            edges.Reverse();
            nodes.Reverse();

            foreach (Edge edge in edges)
            {
                Utils.CalculateElevationDiffAndSteepness( edge, ref maxSteepness, ref currentElevationDfif );
            }

            double sum = SolutionEdges.Concat( edges ).ToList().Sum( x => x.Cost );
        }

        private void BackTrackToUsableSolution ( Edge pickedEdge, HashSet<int> visited, List<Node> visitableNodes, int currentNode, out HashSet<int> fixedVisited, out List<Node> fixedVisitable, out int fixedCurrentNode )
        {
            fixedCurrentNode = -1;
            fixedVisitable = null;
            fixedVisited = null;
            if (pickedEdge != null)
            {
                Node neighbor = pickedEdge.SourceNode.GraphNodeId == currentNode ? pickedEdge.TargetNode : pickedEdge.SourceNode;
                int neighborId = neighbor.GraphNodeId;

                // undo the changes made due to picking this edge
                _ = visited.Remove( neighborId );
                fixedVisited = visited;
                visitableNodes.Add( visitableNodes.Find( x => x.GraphNodeId == neighborId ) );
                fixedVisitable = visitableNodes;
                _ = SolutionEdges.Remove( pickedEdge );

                // make edge impossible to pick again
                // setting trailIntensity to 0 makes the probability of this edge being picked 0
                pickedEdge.TrailIntensity = 0;

                Node current = pickedEdge.SourceNode.GraphNodeId == neighborId ? pickedEdge.TargetNode : pickedEdge.SourceNode;
                fixedCurrentNode = current.GraphNodeId;
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
        public void UpdatePheromoneTrail ( ref Problem problem, List<Edge> visited, double evaporation, bool usePenalty, bool inclueAreaCoverage )
        {
            double evaporationRate = evaporation;

            // default set to 1 because that means no addition or reduction of pheromone values to be added
            double penaltyQuotient = 1;

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
                        penaltyQuotient *= problem.CoveredAreaImportance * Math.Abs( actualDiff - perfectDiff );
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

            foreach (Edge visitedEdge in visited)
            {
                // update actual edge of the graph
                Edge edge = problem.Graph.VEdges[ visitedEdge.GraphId ];

                // trailIntensity contains the actual pheromone left on the trail
                // evaporationRate descibes how much of the trail has evaporated during one tour
                // pheromone contains the newly placed pheromone from the current ants (updated during tour)
                // scale this by a penalty if the ant didn't close the tour
                edge.TrailIntensity = ( edge.TrailIntensity * evaporationRate ) + ( edge.Pheromone / penaltyQuotient );
            }
        }

        /// <summary>
        /// Generates a random choice based on given probabilities (float) and their respective edge id (int).
        /// </summary>
        /// <param name="probabilities">Array of probabilities for each choice.</param>
        /// <param name="randomNumber"> the random number that was generated.</param>
        /// <returns>The randomly selected choice and its respective edge id.</returns>
        private KeyValuePair<Edge, float> ChooseEdge ( Dictionary<Edge, float> probabilities, float randomNumber )
        {
            if (probabilities.Count == 1)
            {
                return probabilities.First();
            }
            // start with percentage 0
            float sum = 0;
            // step through all possible neighbor edges
            foreach (KeyValuePair<Edge, float> kvp in probabilities)
            {
                // get the current edge percentage
                float percentage = kvp.Value;
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
            return new KeyValuePair<Edge, float>( null, -1 );
        }
    }
}