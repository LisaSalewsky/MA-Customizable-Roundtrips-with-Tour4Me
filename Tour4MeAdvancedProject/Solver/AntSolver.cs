using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Tour4MeAdvancedProject.Helper;
using Tour4MeAdvancedProject.ObjectClasses;
using static Tour4MeAdvancedProject.Helper.EnumHelper;

namespace Tour4MeAdvancedProject.Solver
{
    public class AntSolver : Selection
    {
        public int NumberTours { get; set; } = 10;
        public int NumberAnts { get; set; } = 50;
        public List<Ant> Ants { get; set; } = new List<Ant>();
        public double Alpha { get; set; } = 0.3;
        public double Beta { get; set; } = 0.7;
        public double EvaporationRate { get; set; } = 0.8;
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
        public override SolveStatus Solve ( ref Problem P )
        {
            UsePenalty = true;
            UseBacktracking = false;
            InclueAreaCoverage = true;
            EvaporationRate = 0.4;
            int pheromoneAmount = 10;

            PreprocessEdges( P );


            for (int i = 0; i < NumberAnts; i++)
            {
                Ants.Add( new Ant( pheromoneAmount ) );
            }
            List<Edge> solutionEdges = new List<Edge>();
            List<int> visitedNodes = new List<int>();

            Problem tempProblem = P;
            for (int i = 0; i < NumberTours; i++)
            {
                //foreach (Ant currentAnt in Ants)
                _ = Parallel.ForEach( Ants, currentAnt =>
                {
                    lock (tempProblem)
                    {
                        // calculate one Tour for the current Ant
                        // save the edges that form the solution path in solutionEdges
                        (solutionEdges, visitedNodes) = currentAnt.Tour( ref tempProblem, UsePenalty, UseBacktracking );

                        // now update the pheromone trail (trailInensity)
                        currentAnt.UpdatePheromoneTrail( ref tempProblem, solutionEdges, EvaporationRate, UsePenalty, InclueAreaCoverage );
                    }
                }
                );
                // reset all pheromone values set by ants
                foreach (Edge edge in P.Graph.VEdges)
                {
                    if (edge != null)
                    {
                        edge.Pheromone = 1;
                    }
                }
            }

            P = tempProblem;

            P.Path = new Path( solutionEdges, visitedNodes, P.GetProfit( visitedNodes.ToList() ), P.Path );


            HashSet<SurfaceTag> addedSurfaceTags = new HashSet<SurfaceTag>();
            HashSet<HighwayTag> addedPathTypes = new HashSet<HighwayTag>();
            HashSet<string> addedSurroundings = new HashSet<string>();

            foreach (Edge edge in solutionEdges)
            {
                for (int i = 0; i < edge.Tags.Count; i++)
                {
                    string currentTag = edge.Tags[ i ];
                    Utils.AddTags( ref addedSurfaceTags, ref addedPathTypes, ref addedSurroundings, currentTag );

                }

            }
            P.Path.Surfaces = string.Join( ", ", addedSurfaceTags );
            P.Path.PathTypes = string.Join( ", ", addedPathTypes );
            P.Path.SurroundingTags = string.Join( ", ", addedSurroundings );
            P.Path.Elevation = P.Path.Elevation / 2;

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

            //HashSet<int> visitedNodes = new HashSet<int>();
            //P.Graph.CalculateShortestDistances( P.Start );
            P.Graph.InitializeShortestPath( P.Start );

            _ = Parallel.ForEach( P.Graph.VEdges, edge =>
            {
                //    if (edge.SourceNode.ShortestDistance == 0)
                //    {
                //        edge.SourceNode.ShortestDistance = P.Graph.ShortestPath( P.Start, edge.SourceNode.GraphNodeId );
                //    }
                //    if (edge.TargetNode.ShortestDistance == 0)
                //    {
                //        edge.TargetNode.ShortestDistance = P.Graph.ShortestPath( P.Start, edge.TargetNode.GraphNodeId );
                //    }
                if (edge != null)
                {
                    foreach (string tag in edge.Tags)
                    {
                        if (P.PrefTags.Contains( tag ))
                        {
                            edge.Pheromone += increaseAmount;
                        }
                        if (P.AvoidTags.Contains( tag ))
                        {
                            edge.Pheromone -= decreaseAmount;
                        }
                    }
                    // don't allow negative pheromone values (for now) TODO maybe change this
                    edge.TrailIntensity = edge.Pheromone < 0 ? 0 : edge.Pheromone;
                }
            } );

        }

    }
}