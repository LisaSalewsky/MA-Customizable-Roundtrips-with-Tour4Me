using System.Collections.Generic;
using Tour4MeAdvancedProject.ObjectClasses;
using static Tour4MeAdvancedProject.Helper.EnumHelper;

namespace Tour4MeAdvancedProject.Solver
{
    public class AntSolver : Selection
    {
        public int NumberTours { get; set; } = 2;
        public int NumberAnts { get; set; } = 10;
        public List<Ant> Ants { get; set; } = new List<Ant>();
        public double Alpha { get; set; } = 0.5;
        public double Beta { get; set; } = 0.5;
        public double EvaporationRate { get; set; } = 0.5;
        public bool UsePenalty { get; set; } = true;
        public bool UseBacktracking { get; set; } = true;
        public bool InclueAreaCoverage { get; set; } = true;

        public AntSolver () { }

        public AntSolver ( int numberTours, int numberAnts, double alpha, double beta )
        {
            NumberTours = numberTours;
            NumberAnts = numberAnts;
            Alpha = alpha;
            Beta = beta;
        }


        /// <summary>
        /// Overrides the base method to solve the given problem using the Ant Colony Optimization (ACO)
        /// algorithm.
        /// </summary>
        /// <param name="P">The problem instance to be solved.</param>
        /// <returns>
        /// The status of the solution attempt. Returns <see cref="SolveStatus.Unsolved"/> 
        /// if the algorithm does not find a solution.
        /// </returns>
        /// <remarks>
        /// This method applies the Ant Colony Optimization (ACO) algorithm to solve the given problem 
        /// <paramref name="P"/>.
        /// It initializes a specified number of ants, and for a specified number of tours, 
        /// each ant conducts a tour of the graph, updating pheromone levels on the edges. 
        /// After all tours are completed, the pheromone levels are reset to zero.
        /// </remarks>
        /// <seealso cref="Problem"/>
        /// <seealso cref="Ant"/>
        /// <seealso cref="Graph"/>
        /// <seealso cref="Edge"/>
        /// <seealso cref="AntColonyOptimizer"/>
        /// </summary>
        public override SolveStatus Solve ( Problem P )
        {
            UsePenalty = true;
            UseBacktracking = false;
            InclueAreaCoverage = true;
            EvaporationRate = 0.6;
            int pheromoneAmount = 1;

            PreprocessEdges( P );


            for (int i = 0; i < NumberAnts; i++)
            {
                Ants.Add( new Ant( pheromoneAmount ) );
            }
            List<Edge> solutionEdges = new List<Edge>();
            List<int> visitedNodes = new List<int>();
            for (int i = 0; i < NumberTours; i++)
            {
                foreach (Ant currentAnt in Ants)
                {
                    // calculate one Tour for the current Ant
                    // save the edges that form the solution path in solutionEdges
                    (solutionEdges, visitedNodes) = currentAnt.Tour( P, UsePenalty, UseBacktracking );

                    // now update the pheromone trail (trailInensity)
                    currentAnt.UpdatePheromoneTrail( P, solutionEdges, EvaporationRate, UsePenalty, InclueAreaCoverage );
                }
                // reset all pheromone values set by ants
                P.Graph.VEdges.ForEach( edge => edge.Pheromone = 0 );
            }


            P.Path = new Path( solutionEdges, visitedNodes, P.GetProfit( visitedNodes ) );

            return SolveStatus.Feasible;
        }


        private void PreprocessEdges ( Problem P )
        {
            double increaseAmount = P.PrefTags.Count == 0 || P.EdgeProfitImportance == 0
                ? 0
                : 1 / ( P.PrefTags.Count * P.EdgeProfitImportance );
            double decreaseAmount = P.AvoidTags.Count == 0 || P.EdgeProfitImportance == 0
                ? 0
                : 1 / ( P.AvoidTags.Count * P.EdgeProfitImportance );

            foreach (Edge edge in P.Graph.VEdges)
            {
                foreach (string tag in P.PrefTags)
                {
                    if (edge.Tags.Contains( tag ))
                    {
                        edge.Pheromone += increaseAmount;
                    }
                }
                foreach (string tag in P.AvoidTags)
                {
                    if (edge.Tags.Contains( tag ))
                    {
                        edge.Pheromone -= decreaseAmount;
                    }
                }
                // don't allow negative pheromone values (for now) TODO maybe change this
                edge.Pheromone = edge.Pheromone < 0 ? 0 : edge.Pheromone;
            }

        }

    }
}