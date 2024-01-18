using System;
using System.Collections.Generic;
using System.Linq;
using Tour4MeAdvancedProject.Solver;
using static Tour4MeAdvancedProject.Helper.EnumHelper;

namespace Tour4MeAdvancedProject.ObjectClasses
{
    public class Ant : Selection
    {
        public List<Node> Solution { get; set; }

        // constant for calculating pheromone distribution of  this specific ant
        public int PheromoneAmount { get; set; }
        public double Alpha { get; set; } = 0.5;
        public double Beta { get; set; } = 0.5;
        public Random Random { get; set; }


        public Ant ( int pheromoneAmount )
        {
            Solution = new List<Node>();
            PheromoneAmount = pheromoneAmount;
        }

        public override SolveStatus Solve ( Problem CurrentProblem )
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

            double currentActual = 0;

            // TODO: ensure we close our tour not going over the max length!
            while (queue.Count > 0)
            {
                //(double currentDist, (int currentNode, double currentActual)) = queue.Dequeue();
                int currentNode = queue.First();
                _ = queue.Remove( currentNode );

                if (currentActual > CurrentProblem.TargetDistance)
                {
                    continue;
                }

                // only look into edges where the opposite of currentNode wasn't visited yet
                List<Edge> allowed = vNodes[ currentNode ].Incident.FindAll( x =>
                ( x.SourceNode.Id == currentNode && !visited.Contains( x.TargetNode.Id ) ) ||
                ( x.TargetNode.Id == currentNode && !visited.Contains( x.SourceNode.Id ) ) );

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
                    int neighborId = edge.SourceNode.Id == currentNode ? edge.TargetNode.Id : edge.SourceNode.Id;
                    currentActual += edge.Cost;

                    // delta t_{ij}^k
                    double edgeValue = edge.Cost * edge.TrailIntensity;
                    double edgeVisibility = 1 / edge.Cost;

                    // p_{ij}^k
                    float currentProbability = (float)( Math.Pow( edge.TrailIntensity, Alpha ) * Math.Pow( edgeVisibility, Beta ) / sumOfAllowed );

                    edgeProbabilities.Add( edge, currentProbability );
                    edge.Pheromone += PheromoneAmount / edge.Cost;
                }

                // randomly generate a random number between [0.0, 1.0) (excluding 1)
                float probability = (float)Random.NextDouble();

                KeyValuePair<Edge, float> pickedPair = ChooseEdge( edgeProbabilities, probability );
                Edge pickedEdge = pickedPair.Key;
                float pickedProbability = pickedPair.Value;

                // if we picked an edge: move to the respctive neighbor and add it to the queue 
                if (pickedEdge != null && pickedProbability > 0)
                {
                    // now choose next node based on probability
                    Node neighbor = pickedEdge.SourceNode.Id == currentNode ? pickedEdge.TargetNode : pickedEdge.SourceNode;
                    int neighborId = neighbor.Id;

                    queue.Add( neighborId );
                    // once a node was visited, it cannot be visited again in this tour by the same ant
                    visited.Add( neighborId );
                    _ = visitableNodes.Remove( visitableNodes.Find( x => x.Id == neighborId ) );

                    Solution.Add( neighbor );
                }
                else
                {
                    return SolveStatus.Unsolved;
                }
            }

            UpdatePheromoneTrail( graph );

            // TODO add option for more tries 
            // TODO add option for more ants

            return SolveStatus.Feasible;
        }

        /// <summary>
        /// Updates the pheromone trail on edges in the graph based on evaporation and 
        /// newly placed pheromones.
        /// </summary>
        /// <param name="graph">The graph containing edges with pheromone trails and trail intensities.</param>
        private void UpdatePheromoneTrail ( Graph graph )
        {
            double evaporationRate = 0.9;

            foreach (Edge edge in graph.VEdges)
            {
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