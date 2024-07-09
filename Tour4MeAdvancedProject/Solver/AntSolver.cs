using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Tour4MeAdvancedProject.Helper;
using Tour4MeAdvancedProject.ObjectClasses;
using static Tour4MeAdvancedProject.Helper.EnumHelper;

namespace Tour4MeAdvancedProject.Solver
{
    public class AntSolver : Selection
    {
        public int NumberTours { get; set; } = 50;
        public int NumberAnts { get; set; } = 100;
        public List<Ant> Ants { get; set; } = new List<Ant>();
        public double Alpha { get; set; } = 2;
        public double Beta { get; set; } = 5;
        public double EvaporationRate { get; set; } = 0.6;
        public int EdgeScalingPenalty { get; set; } = 1;
        public double InitTrailIntensity { get; set; } = 0.0001;
        public int TrailPenalty { get; set; } = 1;
        public string NewPheromoneFunction { get; set; } = "Profit";
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

        public AntSolver (
            int numberRunsAnt,
            int numberAnts,
            double alpha,
            double beta,
            double evaporationRate,
            int edgeScalingPenalty,
            double initTrailIntensity,
            int trailPenalty,
            string newPheromoneFunction ) : this( numberRunsAnt, numberAnts, alpha, beta )
        {
            EvaporationRate = evaporationRate;
            EdgeScalingPenalty = edgeScalingPenalty;
            InitTrailIntensity = initTrailIntensity;
            TrailPenalty = trailPenalty;
            NewPheromoneFunction = newPheromoneFunction;
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
            //EvaporationRate = ;
            int pheromoneAmount = 10;

            PreprocessEdges( P );

            Ants = new List<Ant>();
            for (int i = 0; i < NumberAnts; i++)
            {
                Ants.Add( new Ant( pheromoneAmount ) );
            }
            List<Edge> solutionEdges = new List<Edge>();
            List<int> visitedNodes = new List<int>();

            Problem tempProblem = P;
            Problem returnedProb = new Problem( tempProblem );
            List<Problem> allSolutions = new List<Problem>();


            for (int i = 0; i < NumberTours; i++)
            {
                //foreach (Ant currentAnt in Ants)
                _ = Parallel.ForEach( Ants, currentAnt =>
                {
                    lock (tempProblem)
                    {
                        // calculate one Tour for the current Ant
                        // save the edges that form the solution path in solutionEdges
                        (returnedProb, solutionEdges, visitedNodes) = currentAnt.Tour( tempProblem, UsePenalty, UseBacktracking );

                        if (returnedProb.Path.Quality > 1)
                        {
                            Console.WriteLine( "ahhhhhhh" );
                        }
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
                for (int j = 0; j < P.Graph.VEdges.Length; j++)
                {
                    Edge edge = P.Graph.VEdges[ j ];
                    if (edge != null)
                    {
                        edge.Visited = false;
                        edge.Pheromone = 1;
                    }
                }
                tempProblem = allSolutions.First( s => Math.Abs( s.Path.Quality ) == allSolutions.Max( x => Math.Abs( x.Path.Quality ) ) );

                if (tempProblem.Path.Quality > 1)
                {
                    Console.WriteLine( "ahhhhhhh" );
                }
            }


            returnedProb = allSolutions.First( s => Math.Abs( s.Path.Quality ) == allSolutions.Max( x => Math.Abs( x.Path.Quality ) ) );
            P = returnedProb;

            P.Path = new Path( solutionEdges, visitedNodes, P.GetProfit( visitedNodes.ToList() ), P.Path );
            for (int i = 0; i < P.Graph.VEdges.Length; i++)
            {
                Edge edge = P.Graph.VEdges[ i ];
                if (edge != null)
                {
                    edge.TrailIntensity = InitTrailIntensity;
                }
            }

            HashSet<SurfaceTag> addedSurfaceTags = new HashSet<SurfaceTag>();
            HashSet<HighwayTag> addedPathTypes = new HashSet<HighwayTag>();
            HashSet<string> addedSurroundings = new HashSet<string>();
            double currentEdgeProfits = P.Path.TotalEdgeProfits;
            double currentArea = P.Path.CoveredArea;
            double currentQuality = P.Path.Quality;
            double currentPathsMaxSteepness = P.Path.Steepness;
            double currentElevationDiff = P.Path.Elevation;
            System.Tuple<double, double>[] boudingCoordinates = P.Path.BoundingCoordinates;


            //Utils.UpdateCurrentProblemPathMetadata( ref P, addedSurfaceTags, addedPathTypes, addedSurroundings, currentEdgeProfits, currentArea, currentQuality, currentPathsMaxSteepness, currentElevationDiff, boudingCoordinates );
            Utils.UpdateMetadata( P.Path, P );

            return SolveStatus.Feasible;
        }
        public override int SolveMaxTime ( ref Problem P, int maxTime )
        {
            UsePenalty = true;
            UseBacktracking = false;
            InclueAreaCoverage = true;
            //EvaporationRate = ;
            int pheromoneAmount = 10;
            int time = -1;

            PreprocessEdges( P );

            Ants = new List<Ant>();
            for (int i = 0; i < NumberAnts; i++)
            {
                Ants.Add( new Ant( pheromoneAmount ) );
            }
            List<Edge> solutionEdges = new List<Edge>();
            List<int> visitedNodes = new List<int>();

            Problem tempProblem = P;
            Problem returnedProb = new Problem( tempProblem );
            List<Problem> allSolutions = new List<Problem>();


            DateTime calc_time_1 = DateTime.Now;
            // do the simulated annealing runs
            time = ( DateTime.Now - calc_time_1 ).Seconds;
            while (time < maxTime)
            {
                //foreach (Ant currentAnt in Ants)
                _ = Parallel.ForEach( Ants, currentAnt =>
                {
                    lock (tempProblem)
                    {
                        // calculate one Tour for the current Ant
                        // save the edges that form the solution path in solutionEdges
                        (returnedProb, solutionEdges, visitedNodes) = currentAnt.Tour( tempProblem, UsePenalty, UseBacktracking );

                        if (returnedProb.Path.Quality > 1)
                        {
                            Console.WriteLine( "ahhhhhhh" );
                        }
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
                for (int j = 0; j < P.Graph.VEdges.Length; j++)
                {
                    Edge edge = P.Graph.VEdges[ j ];
                    if (edge != null)
                    {
                        edge.Visited = false;
                        edge.Pheromone = 1;
                    }
                }
                tempProblem = allSolutions.First( s => Math.Abs( s.Path.Quality ) == allSolutions.Max( x => Math.Abs( x.Path.Quality ) ) );

                if (tempProblem.Path.Quality > 1)
                {
                    Console.WriteLine( "ahhhhhhh" );
                }

                time = ( DateTime.Now - calc_time_1 ).Seconds;
            }


            returnedProb = allSolutions.First( s => Math.Abs( s.Path.Quality ) == allSolutions.Max( x => Math.Abs( x.Path.Quality ) ) );
            P = returnedProb;

            P.Path = new Path( solutionEdges, visitedNodes, P.GetProfit( visitedNodes.ToList() ), P.Path );
            for (int i = 0; i < P.Graph.VEdges.Length; i++)
            {
                Edge edge = P.Graph.VEdges[ i ];
                if (edge != null)
                {
                    edge.TrailIntensity = InitTrailIntensity;
                }
            }

            HashSet<SurfaceTag> addedSurfaceTags = new HashSet<SurfaceTag>();
            HashSet<HighwayTag> addedPathTypes = new HashSet<HighwayTag>();
            HashSet<string> addedSurroundings = new HashSet<string>();
            double currentEdgeProfits = P.Path.TotalEdgeProfits;
            double currentArea = P.Path.CoveredArea;
            double currentQuality = P.Path.Quality;
            double currentPathsMaxSteepness = P.Path.Steepness;
            double currentElevationDiff = P.Path.Elevation;
            System.Tuple<double, double>[] boudingCoordinates = P.Path.BoundingCoordinates;


            //Utils.UpdateCurrentProblemPathMetadata( ref P, addedSurfaceTags, addedPathTypes, addedSurroundings, currentEdgeProfits, currentArea, currentQuality, currentPathsMaxSteepness, currentElevationDiff, boudingCoordinates );
            Utils.UpdateMetadata( P.Path, P );

            return time;
        }


        private void PreprocessEdges ( Problem P )
        {
            _ = P.PrefTags.Count == 0 || P.EdgeProfitImportance == 0
                ? 0
                : 1 / ( P.PrefTags.Count * P.EdgeProfitImportance );
            _ = P.AvoidTags.Count == 0 || P.EdgeProfitImportance == 0
                ? 0
                : 1 / ( P.AvoidTags.Count * P.EdgeProfitImportance );

            //HashSet<int> visitedNodes = new HashSet<int>();
            //P.Graph.CalculateShortestDistances( P.Start );

            if (P.Graph.VNodes.Any( x => x != null && x.GraphNodeId != P.Start && x.ShortestDistance == 0 ))
            {
                P.Graph.InitializeShortestPath( P.Start );
            }

            for (int i = 0; i < P.Graph.VEdges.Length; i++)
            {
                Edge edge = P.Graph.VEdges[ i ];
                if (edge != null)
                {
                    edge.Visited = false;
                    edge.Pheromone = 0;
                    edge.TrailIntensity = InitTrailIntensity;
                    edge.Quality = 0;
                }
            }

            //_ = Parallel.ForEach( P.Graph.VEdges, edge =>
            //{
            //    //    if (edge.SourceNode.ShortestDistance == 0)
            //    //    {
            //    //        edge.SourceNode.ShortestDistance = P.Graph.ShortestPath( P.Start, edge.SourceNode.GraphNodeId );
            //    //    }
            //    //    if (edge.TargetNode.ShortestDistance == 0)
            //    //    {
            //    //        edge.TargetNode.ShortestDistance = P.Graph.ShortestPath( P.Start, edge.TargetNode.GraphNodeId );
            //    //    }
            //    if (edge != null)
            //    {
            //        for (int j = 0; j < edge.Tags.Count; j++)
            //        {
            //            string tag = edge.Tags[ j ];
            //            if (P.PrefTags.Contains( tag ))
            //            {
            //                edge.Pheromone += increaseAmount;
            //            }
            //            if (P.AvoidTags.Contains( tag ))
            //            {
            //                edge.Pheromone -= decreaseAmount;
            //            }
            //        }
            //        // don't allow negative pheromone values (for now) TODO maybe change this
            //        edge.TrailIntensity = edge.Pheromone < 0 ? 0 : edge.Pheromone;
            //        if (edge.TrailIntensity < double.MinValue || edge.TrailIntensity > double.MaxValue)
            //        {
            //            Console.WriteLine( "ahhh" );
            //        }
            //    }
            //} );

        }

        public SolveStatus SolveStepwiseFeedback ( ref Problem P, ref StringBuilder[] jsonBuilders )
        {
            UsePenalty = true;
            UseBacktracking = false;
            InclueAreaCoverage = true;
            //EvaporationRate = ;
            int pheromoneAmount = 10;

            PreprocessEdges( P );

            Ants = new List<Ant>();
            for (int i = 0; i < NumberAnts; i++)
            {
                Ants.Add( new Ant( pheromoneAmount ) );
            }
            List<Edge> solutionEdges = new List<Edge>();
            List<int> visitedNodes = new List<int>();

            Problem tempProblem = P;
            Problem returnedProb = new Problem( tempProblem );
            List<Problem> allSolutions = new List<Problem>();


            for (int i = 0; i < NumberTours; i++)
            {
                //foreach (Ant currentAnt in Ants)
                _ = Parallel.ForEach( Ants, currentAnt =>
                {
                    lock (tempProblem)
                    {
                        // calculate one Tour for the current Ant
                        // save the edges that form the solution path in solutionEdges
                        (returnedProb, solutionEdges, visitedNodes) = currentAnt.Tour( tempProblem, UsePenalty, UseBacktracking );

                        if (returnedProb.Path.Quality > 1)
                        {
                            Console.WriteLine( "ahhhhhhh" );
                        }
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
                for (int j = 0; j < P.Graph.VEdges.Length; j++)
                {
                    Edge edge = P.Graph.VEdges[ j ];
                    if (edge != null)
                    {
                        edge.Visited = false;
                        edge.Pheromone = 1;
                    }
                }
                tempProblem = allSolutions.First( s => Math.Abs( s.Path.Quality ) == allSolutions.Max( x => Math.Abs( x.Path.Quality ) ) );

                if (tempProblem.Path.Quality > 1)
                {
                    Console.WriteLine( "ahhhhhhh" );
                }

                _ = jsonBuilders[ i ].Append( "    {\n" );
                _ = jsonBuilders[ i ].Append( "    \"run\": \"Run " + i + "\",\n" );
                _ = jsonBuilders[ i ].Append( "    \"runNumber\": " + i + ",\n" );
                int negModifier = tempProblem.Path.CoveredArea < 0 ? -1 : 1;
                _ = jsonBuilders[ i ].Append( "    \"CoveredArea\": " + ( negModifier * Math.Round( tempProblem.Path.CoveredArea ) ).ToString( System.Globalization.CultureInfo.InvariantCulture ) + ",\n" );
                _ = jsonBuilders[ i ].Append( "    \"Profit\": " + tempProblem.Path.TotalEdgeProfits.ToString( System.Globalization.CultureInfo.InvariantCulture ) + ",\n" );
                _ = jsonBuilders[ i ].Append( "    \"Elevation\": " + ( ( tempProblem.MaxElevation - tempProblem.Path.Elevation ) / tempProblem.MaxElevation ).ToString( System.Globalization.CultureInfo.InvariantCulture ) + ",\n" );
                _ = jsonBuilders[ i ].Append( "    \"Quality\": " + Math.Round( tempProblem.Path.Quality * 1000, 4 ).ToString( System.Globalization.CultureInfo.InvariantCulture ) );

                //_ = i == NumberTours - 1 ? jsonBuilders[ i ].Append( "\n    }\n ]" ) : jsonBuilders[ i ].Append( "\n    },\n" );
            }


            returnedProb = allSolutions.First( s => Math.Abs( s.Path.Quality ) == allSolutions.Max( x => Math.Abs( x.Path.Quality ) ) );
            P = returnedProb;

            P.Path = new Path( solutionEdges, visitedNodes, P.GetProfit( visitedNodes.ToList() ), P.Path );
            for (int i = 0; i < P.Graph.VEdges.Length; i++)
            {
                Edge edge = P.Graph.VEdges[ i ];
                if (edge != null)
                {
                    edge.TrailIntensity = InitTrailIntensity;
                }
            }

            HashSet<SurfaceTag> addedSurfaceTags = new HashSet<SurfaceTag>();
            HashSet<HighwayTag> addedPathTypes = new HashSet<HighwayTag>();
            HashSet<string> addedSurroundings = new HashSet<string>();
            double currentEdgeProfits = P.Path.TotalEdgeProfits;
            double currentArea = P.Path.CoveredArea;
            double currentQuality = P.Path.Quality;
            double currentPathsMaxSteepness = P.Path.Steepness;
            double currentElevationDiff = P.Path.Elevation;
            System.Tuple<double, double>[] boudingCoordinates = P.Path.BoundingCoordinates;


            //Utils.UpdateCurrentProblemPathMetadata( ref P, addedSurfaceTags, addedPathTypes, addedSurroundings, currentEdgeProfits, currentArea, currentQuality, currentPathsMaxSteepness, currentElevationDiff, boudingCoordinates );
            Utils.UpdateMetadata( P.Path, P );

            return SolveStatus.Feasible;
        }


    }
}