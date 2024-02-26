using System;
using System.Collections.Generic;
using System.Linq;

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
        public (List<Edge>, List<int>) Tour ( Problem CurrentProblem, bool usePenalty, bool useBacktracking )
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

            //PriorityQueue<Tuple<int, double>> queue = new PriorityQueue<Tuple<int, double>>();
            //queue.Enqueue(0.0, new Tuple<int, double>(start, 0.0));
            List<int> queue = new List<int>() { start };
            List<int> visited = new List<int>();
            Edge pickedEdge = null;

            double currentDistance = 0;
            visited.Add( start );
            #endregion


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
                List<Edge> allowed = vNodes[ currentNode ].Incident.FindAll( x =>
                 !visited.Contains( x.TargetNode.GraphNodeId ) || !visited.Contains( x.SourceNode.GraphNodeId ) );

                FindAllowedPath( CurrentProblem, usePenalty, useBacktracking, vNodes, ref visitableNodes, ref visited, pickedEdge, currentDistance, ref currentNode, ref allowed );

                float sumOfAllowed = PrecomputeSumOverAllAllowedEdges( allowed );
                CalculateNewPheromoneValues( CurrentProblem, usePenalty, graph, start, currentDistance, edgeProbabilities, currentNode, allowed, sumOfAllowed );

                // randomly generate a random number between [0.0, 1.0) (excluding 1)
                float probability = (float)Random.NextDouble();

                // pick an edge according to edgeProbabilities and calculated probability
                KeyValuePair<Edge, float> pickedPair = ChooseEdge( edgeProbabilities, probability );
                pickedEdge = pickedPair.Key;
                float pickedProbability = pickedPair.Value;

                // if we picked an edge: move to the respctive neighbor and add it to the queue (= choose next town to move to)
                if (pickedEdge != null && pickedProbability > 0)
                {
                    parent = AddEdgeAndContinue( visitableNodes, queue, visited, pickedEdge, ref currentDistance, currentNode );
                }
                else
                {
                    CloseOffPath( graph, start, parent, out List<Edge> edges, out List<int> nodes );

                    return (SolutionEdges.Concat( edges ).ToList(), visited.Concat( nodes ).ToList());
                }
            }

            return (SolutionEdges, visited);
        }

        private void FindAllowedPath ( Problem CurrentProblem, bool usePenalty, bool useBacktracking, List<Node> vNodes, ref List<Node> visitableNodes, ref List<int> visited, Edge pickedEdge, double currentDistance, ref int currentNode, ref List<Edge> allowed )
        {

            // if we can't find a next node from the one we picked, choose from the following options:
            if (allowed.Count == 0 && currentDistance < CurrentProblem.TargetDistance)
            {
                // if we use a penalty, the ants will figure out what to do by themselves. Just update the trail intensity accordingly
                if (usePenalty)
                {
                    allowed = ScaleTrailIntensity( vNodes, visited, currentNode );
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

        private void CalculateNewPheromoneValues ( Problem CurrentProblem, bool usePenalty, Graph graph, int start, double currentDistance, Dictionary<Edge, float> edgeProbabilities, int currentNode, List<Edge> allowed, float sumOfAllowed )
        {
            foreach (Edge edge in allowed)
            {
                int neighborId = edge.SourceNode.GraphNodeId == currentNode ? edge.TargetNode.GraphNodeId : edge.SourceNode.GraphNodeId;


                if (!usePenalty && graph.ShortestPath( start, neighborId ) > CurrentProblem.TargetDistance - currentDistance - edge.Cost)
                {
                    continue;
                }


                //double edgeValue = edge.Cost * edge.TrailIntensity;
                double edgeVisibility = 1 / edge.Cost;

                // p_{ij}^k (used for choosing town to move to)
                float currentProbability = (float)( Math.Pow( edge.TrailIntensity, Alpha ) * Math.Pow( edgeVisibility, Beta ) / sumOfAllowed );
                edgeProbabilities.Add( edge, currentProbability );

                // delta t_{ij}^k (PheromoneAmount / edge.Cost = Q / L_k) & update of delta t_ij (edge.Pheromone)
                edge.Pheromone += PheromoneAmount / edge.Cost;
            }
        }

        private static List<Edge> ScaleTrailIntensity ( List<Node> vNodes, List<int> visited, int currentNode )
        {
            List<Edge> allowed;
            // scale trial intensity on edges accordingly for all edges where a node is visited twice
            double penaltyQuotient = 50;
            List<Edge> applyPenalty = vNodes[ currentNode ].Incident.FindAll( x =>
            visited.Contains( x.TargetNode.GraphNodeId ) && visited.Contains( x.SourceNode.GraphNodeId ) );

            foreach (Edge edge in applyPenalty)
            {
                edge.TrailIntensity /= penaltyQuotient;
            }
            allowed = applyPenalty;
            return allowed;
        }

        private float PrecomputeSumOverAllAllowedEdges ( List<Edge> allowed )
        {

            // precompute sum of (trailintensity^alpha * edge visibility^beta) over all allowed edges 
            float sumOfAllowed = 0;
            foreach (Edge edge1 in allowed)
            {
                float trailIntensityPowAlpha = (float)Math.Pow( edge1.TrailIntensity, Alpha );
                float visibilityPowBeta = (float)Math.Pow( 1 / edge1.Cost, Beta );

                sumOfAllowed += trailIntensityPowAlpha * visibilityPowBeta;
            }

            return sumOfAllowed;
        }

        private int AddEdgeAndContinue ( List<Node> visitableNodes, List<int> queue, List<int> visited, Edge pickedEdge, ref double currentDistance, int currentNode )
        {
            int parent;
            // now choose next node based on probability
            Node neighbor = pickedEdge.SourceNode.GraphNodeId == currentNode ? pickedEdge.TargetNode : pickedEdge.SourceNode;
            int neighborId = neighbor.GraphNodeId;
            currentDistance += pickedEdge.Cost;

            queue.Add( neighborId );
            parent = currentNode;
            // once a node was visited, it cannot be visited again in this tour by the same ant
            visited.Add( neighborId );
            _ = visitableNodes.Remove( visitableNodes.Find( x => x.GraphNodeId == neighborId ) );

            // Add the picked edge to the solution path
            SolutionEdges.Add( pickedEdge );
            return parent;
        }

        private void CloseOffPath ( Graph graph, int start, int parent, out List<Edge> edges, out List<int> nodes )
        {
            // if we need to close the loop since we cannot take any other neighbor:
            // close it using the shortest path from the remaining acceptable node
            edges = new List<Edge>();
            nodes = new List<int>();
            (edges, nodes) = graph.GetShortestPath( parent, start );

            double sum = SolutionEdges.Concat( edges ).ToList().Sum( x => x.Cost );
        }

        private void BackTrackToUsableSolution ( Edge pickedEdge, List<int> visited, List<Node> visitableNodes, int currentNode, out List<int> fixedVisited, out List<Node> fixedVisitable, out int fixedCurrentNode )
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
        public void UpdatePheromoneTrail ( Problem problem, List<Edge> visited, double evaporation, bool usePenalty, bool inclueAreaCoverage )
        {
            double evaporationRate = evaporation;

            // default set to 1 because that means no addition or reduction of pheromone values to be added
            double penaltyQuotient = 1;

            bool closedTour = visited.First().Equals( visited.Last() );
            double sum = visited.Sum( x => x.Cost );

            List<Edge> acceptableLentghEdges = new List<Edge>( visited );

            while (sum > problem.TargetDistance)
            {
                // pick last edge of the path
                Edge penaltyEdge = acceptableLentghEdges.Last();
                // set trail intensity to 0 to make it as unattractive as possible for future runs
                penaltyEdge.TrailIntensity = 0;

                // remove last edge that added too much to the distance and re-calculate sum
                _ = acceptableLentghEdges.Remove( penaltyEdge );
                sum = acceptableLentghEdges.Sum( x => x.Cost );

            }

            if (usePenalty && !closedTour)
            {
                // if we didn't close the tour, use a high quotient as penalty
                // this lowers the increase in the trail intensity heavily for future runs
                penaltyQuotient = 100;
            }

            if (inclueAreaCoverage)
            {
                double area = 0;
                foreach (Edge edge in visited)
                {
                    area += edge.ShoelaceForward >= 0 ? edge.ShoelaceForward : edge.ShoelaceBackward;
                }
                // the bigger the area covered, the smaller our penalty needs to be
                penaltyQuotient *= 1 / ( area * problem.CoveredAreaImportance );
            }

            foreach (Edge visitedEdge in visited)
            {
                // update actual edge of the graph
                Edge edge = problem.Graph.VEdges.Find( x => x.Id == visitedEdge.Id );

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