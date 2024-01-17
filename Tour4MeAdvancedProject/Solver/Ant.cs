using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Web;
using System.Xml.Linq;
using Tour4MeAdvancedProject.Solver;
using static Tour4MeAdvancedProject.Helper.EnumHelper;

namespace Tour4MeAdvancedProject.ObjectClasses
{
    public class Ant : Selection
    {
        public List<Node> Solution { get; set; }

        // constant for calculating pheromone distribution of  this specific ant
        public int PheromoneAmount { get; set; }

        
        public Ant(int pheromoneAmount)
        {
            Solution = new List<Node>();
            PheromoneAmount = pheromoneAmount;
        }

        public override SolveStatus Solve(Problem CurrentProblem)
        {
            Graph graph = CurrentProblem.Graph;
            int start = CurrentProblem.Start;
            List<Node> vNodes = graph.VNodes;
            Tuple<int, Edge>[] parent = new Tuple<int, Edge>[vNodes.Count];

            List<Node> visitableNodes = graph.VNodes;

            double[] dist = new double[vNodes.Count];
            dist[start] = 0.0;
            
            double[] actDist = new double[vNodes.Count];
            actDist[start] = 0.0;

            PriorityQueue<Tuple<int, double>> queue = new PriorityQueue<Tuple<int, double>>();
            queue.Enqueue(0.0, new Tuple<int, double>(start, 0.0));
            List<int> visited = new List<int>();

            while (queue.Count > 0)
            {

                (double currentDist, (int currentNode, double currentActual)) = queue.Dequeue();

                if (currentActual > CurrentProblem.TargetDistance)
                {
                    continue;
                }

                double bestKnownDist = dist[currentNode];
                Edge bestEdge;

                foreach (Edge edge in vNodes[currentNode].Incident)
                {
                    int neighborId = edge.SourceNode.Id == currentNode ? edge.TargetNode.Id : edge.SourceNode.Id;

                    // skip already visited nodes
                    if (visited.Contains(neighborId))
                    {
                        continue;
                    }

                    double edgeValue = edge.Cost * edge.TrailIntensity;


                    double newDist = bestKnownDist + (edge.Cost / (edge.Profit + 0.1));
                    double newCost = currentActual + edge.Cost;

                    if (true)
                    {
                        queue.Enqueue(newDist, new Tuple<int, double>(neighborId, newCost));
                        dist[neighborId] = newDist;
                        actDist[neighborId] = newCost;
                        parent[neighborId] = Tuple.Create(currentNode, edge);

                        // once a node was visited, it cannot be visited again in this tour by the same ant
                        visited.Add(neighborId);
                        visitableNodes.Remove(visitableNodes.Find(x => x.Id == neighborId));
                    }

                    edge.Pheromone = PheromoneAmount / newCost;

                }

            }



            return SolveStatus.Unsolved;
        }


        private void updatePheromoneTrail(Graph graph)
        {
            double evaporationRate = 0.9;

            foreach (Edge edge in graph.VEdges)
            {
                // trailIntensity contains the actual pheromone left on the trail
                // evaporationRate descibes how much of the trail has evaporated during one tour
                // pheromone contains the newly placed pheromone from the current ants (updated during tour)
                edge.TrailIntensity = edge.TrailIntensity * evaporationRate + edge.Pheromone;
            }
        }
    }
}