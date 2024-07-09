using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Tour4MeAdvancedProject.Helper;
using Tour4MeAdvancedProject.ObjectClasses;
using static Tour4MeAdvancedProject.Helper.EnumHelper;

namespace Tour4MeAdvancedProject.Solver
{
    public class AntCombined : Selection
    {
        public int NumberTours { get; set; } = 5;
        public int NumberAnts { get; set; } = 2;
        public List<Ant> Ants { get; set; } = new List<Ant>();
        public double Alpha { get; set; } = 0.8;
        public double Beta { get; set; } = 0.2;
        public double EvaporationRate { get; set; } = 0.6;
        public int EdgeScalingPenalty { get; set; } = 1;
        private double InitTrailIntensity { get; set; } = 0.0001;
        public int TrailPenalty { get; set; } = 1;
        public string NewPheromoneFunction { get; set; } = "Profit";
        public bool UsePenalty { get; set; } = true;
        public bool UseBacktracking { get; set; } = true;
        public bool InclueAreaCoverage { get; set; } = true;
        public Algo AlgoToCombineWith { get; set; }

        public AntCombined () { }

        public AntCombined ( int numberTours, int numberAnts, double alpha, double beta )
        {
            NumberTours = numberTours;
            NumberAnts = numberAnts;
            Alpha = alpha;
            Beta = beta;
        }


        public AntCombined (
            int numberRunsAnt,
            int numberAnts,
            double alpha,
            double beta,
            double evaporationRate,
            int edgeScalingPenalty,
            double initTrailIntensity,
            int trailPenalty,
            string newPheromoneFunction,
            Algo algoToUse ) : this( numberRunsAnt, numberAnts, alpha, beta )
        {
            EvaporationRate = evaporationRate;
            EdgeScalingPenalty = edgeScalingPenalty;
            InitTrailIntensity = (int)initTrailIntensity;
            TrailPenalty = trailPenalty;
            NewPheromoneFunction = newPheromoneFunction;
            AlgoToCombineWith = algoToUse;
        }

        public SolveStatus Solve ( ref Problem problem, Algo baseTour )
        {
            AlgoToCombineWith = baseTour;
            switch (AlgoToCombineWith)
            {
                case Algo.Greedy:
                    {
                        // Greedy
                        SelectionSolver solver = new SelectionSolver();
                        _ = solver.Solve( ref problem );
                        break;
                    }
                case Algo.minCost:
                    {
                        // minCost
                        JoggerSolver solver = new JoggerSolver();
                        _ = solver.Solve( ref problem );
                        break;
                    }
                case Algo.ILS:
                    {
                        // ILS
                        //ILS solver = new ILS();
                        //status = solver.Solve( ref problem);
                        break;
                    }
                case Algo.SimulatedAnnealingMinCost:
                    {
                        // Simmulated Annealing
                        SimmulatedAnnealingSolver solver = new SimmulatedAnnealingSolver();
                        _ = solver.Solve( ref problem, Algo.minCost );
                        break;
                    }
                default:
                    break;
            }


            UsePenalty = true;
            UseBacktracking = false;
            InclueAreaCoverage = true;
            //EvaporationRate = 0.6;
            int pheromoneAmount = 1;


            for (int i = 0; i < NumberAnts; i++)
            {
                Ants.Add( new Ant( pheromoneAmount, Alpha, Beta ) );
            }
            List<Edge> solutionEdges = new List<Edge>();
            List<int> visitedNodes = new List<int>();

            Problem tempProblem = problem;
            PreprocessEdges( tempProblem );

            for (int i = 0; i < NumberTours; i++)
            {
                //foreach (Ant currentAnt in Ants)
                _ = Parallel.ForEach( Ants, currentAnt =>
                {
                    lock (tempProblem)
                    {
                        // calculate one Tour for the current Ant
                        // save the edges that form the solution path in solutionEdges
                        (tempProblem, solutionEdges, visitedNodes) = currentAnt.Tour( tempProblem, UsePenalty, UseBacktracking );

                        // now update the pheromone trail (trailInensity)
                        currentAnt.UpdatePheromoneTrail( tempProblem, solutionEdges, EvaporationRate, UsePenalty, InclueAreaCoverage, TrailPenalty );
                    }
                }
                );
                // reset all pheromone values set by ants
                foreach (Edge edge in problem.Graph.VEdges)
                {
                    if (edge != null)
                    {
                        edge.Visited = false;
                        edge.Pheromone = 1;
                    }
                }
            }

            problem = tempProblem;

            problem.Path = new Path( solutionEdges, visitedNodes, problem.GetProfit( visitedNodes.ToList() ), problem.Path );


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
            problem.Path.Surfaces = string.Join( ", ", addedSurfaceTags );
            problem.Path.PathTypes = string.Join( ", ", addedPathTypes );
            problem.Path.SurroundingTags = string.Join( ", ", addedSurroundings );
            problem.Path.Elevation = problem.Path.Elevation / 2;

            Utils.UpdateMetadata( problem.Path, problem );

            return SolveStatus.Feasible;
        }



        public override int SolveMaxTime ( ref Problem problem, int maxTime )
        {
            switch (AlgoToCombineWith)
            {
                case Algo.Greedy:
                    {
                        // Greedy
                        SelectionSolver solver = new SelectionSolver();
                        _ = solver.Solve( ref problem );
                        break;
                    }
                case Algo.minCost:
                    {
                        // minCost
                        JoggerSolver solver = new JoggerSolver();
                        _ = solver.Solve( ref problem );
                        break;
                    }
                case Algo.ILS:
                    {
                        // ILS
                        //ILS solver = new ILS();
                        //status = solver.Solve( ref problem);
                        break;
                    }
                case Algo.SimulatedAnnealingMinCost:
                    {
                        // Simmulated Annealing
                        SimmulatedAnnealingSolver solver = new SimmulatedAnnealingSolver();
                        _ = solver.Solve( ref problem, Algo.minCost );
                        break;
                    }
                default:
                    break;
            }


            UsePenalty = true;
            UseBacktracking = false;
            InclueAreaCoverage = true;
            //EvaporationRate = 0.6;
            int pheromoneAmount = 1;


            for (int i = 0; i < NumberAnts; i++)
            {
                Ants.Add( new Ant( pheromoneAmount, Alpha, Beta ) );
            }
            List<Edge> solutionEdges = new List<Edge>();
            List<int> visitedNodes = new List<int>();

            Problem tempProblem = problem;
            Problem returnedProb = new Problem( tempProblem );
            List<Problem> allSolutions = new List<Problem>();

            PreprocessEdges( tempProblem );

            DateTime calc_time_1 = DateTime.Now;
            // do the simulated annealing runs
            int time = ( DateTime.Now - calc_time_1 ).Seconds;
            while (time < maxTime)
            {
                //foreach (Ant currentAnt in Ants)
                _ = Parallel.ForEach( Ants, currentAnt =>
                {
                    lock (tempProblem)
                    {
                        // calculate one Tour for the current Ant
                        // save the edges that form the solution path in solutionEdges
                        (tempProblem, solutionEdges, visitedNodes) = currentAnt.Tour( tempProblem, UsePenalty, UseBacktracking );

                        Problem temp = new Problem( returnedProb )
                        {
                            Path = new Path( returnedProb.Path )
                        };
                        allSolutions.Add( temp );
                        // now update the pheromone trail (trailInensity)
                        currentAnt.UpdatePheromoneTrail( tempProblem, solutionEdges, EvaporationRate, UsePenalty, InclueAreaCoverage, TrailPenalty );
                    }
                }
                );
                // reset all pheromone values set by ants
                for (int j = 0; j < problem.Graph.VEdges.Length; j++)
                {
                    Edge edge = problem.Graph.VEdges[ j ];
                    if (edge != null)
                    {
                        edge.Visited = false;
                        edge.Pheromone = 1;
                    }
                }
                tempProblem = allSolutions.First( s => Math.Abs( s.Path.Quality ) == allSolutions.Max( x => Math.Abs( x.Path.Quality ) ) );

                time = ( DateTime.Now - calc_time_1 ).Seconds;
            }

            problem = tempProblem;

            problem.Path = new Path( solutionEdges, visitedNodes, problem.GetProfit( visitedNodes.ToList() ), problem.Path );


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
            problem.Path.Surfaces = string.Join( ", ", addedSurfaceTags );
            problem.Path.PathTypes = string.Join( ", ", addedPathTypes );
            problem.Path.SurroundingTags = string.Join( ", ", addedSurroundings );
            problem.Path.Elevation = problem.Path.Elevation / 2;

            Utils.UpdateMetadata( problem.Path, problem );

            return time;
        }


        private void PreprocessEdges ( Problem P )
        {
            P.Graph.InitializeShortestPath( P.Start );

            _ = Parallel.ForEach( P.Path.Edges, edge =>
            {
                P.Graph.VEdges[ edge.GraphId ].TrailIntensity = 5000;
                P.Graph.VEdges[ edge.GraphId ].Pheromone = 5000;
            } );

        }

    }
}