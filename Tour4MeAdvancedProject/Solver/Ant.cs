﻿using System;
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
        }

        public Ant ( int pheromoneAmount, double alpha, double beta )
        {
            //Solution = new List<Node>();
            SolutionEdges = new List<Edge>();
            PheromoneAmount = pheromoneAmount;
            Alpha = alpha;
            Beta = beta;
        }


        /// <summary>
        /// Conducts a tour of the given problem's graph using an ant, 
        /// applying the Ant Colony Optimization (ACO) algorithm.
        /// </summary>
        /// <param name="CurrentProblem">
        /// The problem instance containing the graph and necessary parameters for the tour.
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
        public List<Edge> Tour ( Problem CurrentProblem )
        {
            Graph graph = CurrentProblem.Graph;
            int start = CurrentProblem.Start;
            List<Node> vNodes = graph.VNodes;
            Tuple<int, Edge>[] parent = new Tuple<int, Edge>[ vNodes.Count ];

            List<Node> visitableNodes = graph.VNodes;

            //double[] dist = new double[vNodes.Count];
            //dist[start] = 0.0;

            double[] actDist = new double[ vNodes.Count ];
            actDist[ start ] = 0.0;

            //PriorityQueue<Tuple<int, double>> queue = new PriorityQueue<Tuple<int, double>>();
            //queue.Enqueue(0.0, new Tuple<int, double>(start, 0.0));
            List<int> queue = new List<int>() { start };
            List<int> visited = new List<int>();

            Dictionary<Edge, float> edgeProbabilities = new Dictionary<Edge, float>();

            double currentDistance = 0;

            // TODO: ensure we close our tour not going over the max length!
            while (queue.Count > 0)
            {
                //(double currentDist, (int currentNode, double currentActual)) = queue.Dequeue();
                int currentNode = queue.First();
                _ = queue.Remove( currentNode );

                //if (currentDistance > CurrentProblem.TargetDistance)
                //{
                //    continue;
                //}
                // only look into edges where the opposite of currentNode wasn't visited yet
                List<Edge> allowed = vNodes[ currentNode ].Incident.FindAll( x =>
                ( x.SourceNode.GraphNodeId == currentNode && !visited.Contains( x.TargetNode.GraphNodeId ) ) ||
                ( x.TargetNode.GraphNodeId == currentNode && !visited.Contains( x.SourceNode.GraphNodeId ) ) );

                // precompute sum of (trailintensity^alpha * edge visibility^beta) over all allowed edges 
                float sumOfAllowed = 0;
                foreach (Edge edge1 in allowed)
                {
                    float trailIntensityPowAlpha = (float)Math.Pow( edge1.TrailIntensity, Alpha );
                    float visibilityPowBeta = (float)Math.Pow( 1 / edge1.Cost, Beta );

                    sumOfAllowed += trailIntensityPowAlpha * visibilityPowBeta;
                }

                foreach (Edge edge in allowed)
                {
                    int neighborId = edge.SourceNode.GraphNodeId == currentNode ? edge.TargetNode.GraphNodeId : edge.SourceNode.GraphNodeId;

                    // TODO @Mart: does this actually provide a path of maximum target distance length?
                    if (graph.ShortestPath( start, neighborId ) > CurrentProblem.TargetDistance - currentDistance - edge.Cost)
                    {
                        continue;
                    }

                    currentDistance += edge.Cost;

                    double edgeValue = edge.Cost * edge.TrailIntensity;
                    double edgeVisibility = 1 / edge.Cost;

                    // p_{ij}^k (used for choosing town to move to)
                    float currentProbability = (float)( Math.Pow( edge.TrailIntensity, Alpha ) * Math.Pow( edgeVisibility, Beta ) / sumOfAllowed );
                    edgeProbabilities.Add( edge, currentProbability );

                    // delta t_{ij}^k (PheromoneAmount / edge.Cost = Q / L_k) & update of delta t_ij (edge.Pheromone)
                    edge.Pheromone += PheromoneAmount / edge.Cost;
                }

                // randomly generate a random number between [0.0, 1.0) (excluding 1)
                float probability = (float)Random.NextDouble();

                // pick an edge according to edgeProbabilities and calculated probability
                KeyValuePair<Edge, float> pickedPair = ChooseEdge( edgeProbabilities, probability );
                Edge pickedEdge = pickedPair.Key;
                float pickedProbability = pickedPair.Value;

                // if we picked an edge: move to the respctive neighbor and add it to the queue (= choose next town to move to)
                if (pickedEdge != null && pickedProbability > 0)
                {
                    // now choose next node based on probability
                    Node neighbor = pickedEdge.SourceNode.GraphNodeId == currentNode ? pickedEdge.TargetNode : pickedEdge.SourceNode;
                    int neighborId = neighbor.GraphNodeId;

                    queue.Add( neighborId );
                    // once a node was visited, it cannot be visited again in this tour by the same ant
                    visited.Add( neighborId );
                    _ = visitableNodes.Remove( visitableNodes.Find( x => x.GraphNodeId == neighborId ) );

                    // Add the picked edge to the solution path
                    SolutionEdges.Add( pickedEdge );
                }
                else
                {
                    return null;
                }
            }

            return SolutionEdges;
        }

        /// <summary>
        /// Updates the pheromone trail on edges in the graph based on evaporation and 
        /// newly placed pheromones.
        /// </summary>
        /// <param name="graph">The graph containing edges with pheromone trails and trail intensities.</param>
        public void UpdatePheromoneTrail ( Graph graph, List<Edge> visited )
        {
            double evaporationRate = 0.9;

            foreach (Edge visitedEdge in visited)
            {
                // update actual edge of the graph
                Edge edge = graph.VEdges.Find( x => x.Id == visitedEdge.Id );

                // trailIntensity contains the actual pheromone left on the trail
                // evaporationRate descibes how much of the trail has evaporated during one tour
                // pheromone contains the newly placed pheromone from the current ants (updated during tour)
                edge.TrailIntensity = ( edge.TrailIntensity * evaporationRate ) + edge.Pheromone;
            }
        }

        /// <summary>
        /// Generates a random choice based on given probabilities (float) and their respective edge id (int).
        /// </summary>
        /// <param name="probabilities">Array of probabilities for each choice.</param>
        /// <returns>The randomly selected choice and its respective edge id.</returns>
        private KeyValuePair<Edge, float> ChooseEdge ( Dictionary<Edge, float> probabilities, float randomNumber )
        {
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