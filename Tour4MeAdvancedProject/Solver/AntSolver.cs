using System.Collections.Generic;
using Tour4MeAdvancedProject.ObjectClasses;
using static Tour4MeAdvancedProject.Helper.EnumHelper;

namespace Tour4MeAdvancedProject.Solver
{
    public class AntSolver : Selection
    {
        public int NumberTours { get; set; } = 1;
        public int NumberAnts { get; set; } = 1;
        public List<Ant> Ants { get; set; } = new List<Ant>();
        public double Alpha { get; set; } = 0.5;
        public double Beta { get; set; } = 0.5;
        public bool UsePenalty { get; set; } = true;

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
            UsePenalty = false;
            int pheromoneAmount = 1;

            for (int i = 0; i < NumberAnts; i++)
            {
                Ants.Add( new Ant( pheromoneAmount ) );
            }
            List<Edge> solutionEdges = new List<Edge>();
            List<int> visitedNodes = new List<int>();
            for (int i = 0; i <= NumberTours; i++)
            {
                foreach (Ant currentAnt in Ants)
                {
                    // calculate one Tour for the current Ant
                    // save the edges that form the solution path in solutionEdges
                    (solutionEdges, visitedNodes) = currentAnt.Tour( P, UsePenalty );

                    // now update the pheromone trail (trailInensity)
                    currentAnt.UpdatePheromoneTrail( P.Graph, solutionEdges, UsePenalty );
                }
                // reset all pheromone values set by ants
                P.Graph.VEdges.ForEach( edge => edge.Pheromone = 0 );
            }


            P.Path = new Path( solutionEdges, visitedNodes, P.GetProfit( visitedNodes ) );

            return SolveStatus.Feasible;
        }

    }
}