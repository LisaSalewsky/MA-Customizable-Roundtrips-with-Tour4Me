﻿using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Tour4MeAdvancedProject.Helper;
using Tour4MeAdvancedProject.ObjectClasses;
using static Tour4MeAdvancedProject.Helper.EnumHelper;

namespace Tour4MeAdvancedProject.Solver
{
    public class AntCombined : Selection
    {
        public int NumberTours { get; set; } = 2;
        public int NumberAnts { get; set; } = 10;
        public List<Ant> Ants { get; set; } = new List<Ant>();
        public double Alpha { get; set; } = 0.3;
        public double Beta { get; set; } = 0.7;
        public double EvaporationRate { get; set; } = 0.8;
        public bool UsePenalty { get; set; } = true;
        public bool UseBacktracking { get; set; } = true;
        public bool InclueAreaCoverage { get; set; } = true;

        public AntCombined () { }

        public AntCombined ( int numberTours, int numberAnts, double alpha, double beta )
        {
            NumberTours = numberTours;
            NumberAnts = numberAnts;
            Alpha = alpha;
            Beta = beta;
        }


        public SolveStatus Solve ( ref Problem problem, Algo baseTour )
        {

            SolveStatus status = SolveStatus.Unsolved;
            switch (baseTour)
            {
                case Algo.Greedy:
                    {
                        // Greedy
                        SelectionSolver solver = new SelectionSolver();
                        status = solver.Solve( ref problem );
                        break;
                    }
                case Algo.minCost:
                    {
                        // minCost
                        JoggerSolver solver = new JoggerSolver();
                        status = solver.Solve( ref problem );
                        break;
                    }
                case Algo.ILS:
                    {
                        // ILS
                        //ILS solver = new ILS();
                        //status = solver.Solve( ref problem);
                        break;
                    }
                case Algo.Genetic:
                    {
                        // Genetic
                        GeneticSolver solver = new GeneticSolver();
                        status = solver.Solve( ref problem );
                        break;
                    }
                default:
                    status = SolveStatus.Unsolved;
                    break;
            }


            UsePenalty = true;
            UseBacktracking = false;
            InclueAreaCoverage = true;
            EvaporationRate = 0.6;
            int pheromoneAmount = 1;

            PreprocessEdges( problem );


            for (int i = 0; i < NumberAnts; i++)
            {
                Ants.Add( new Ant( pheromoneAmount ) );
            }
            List<Edge> solutionEdges = new List<Edge>();
            List<int> visitedNodes = new List<int>();

            Problem tempProblem = problem;
            for (int i = 0; i < NumberTours; i++)
            {
                _ = Parallel.ForEach( Ants, currentAnt =>
                {
                    // calculate one Tour for the current Ant
                    // save the edges that form the solution path in solutionEdges
                    (solutionEdges, visitedNodes) = currentAnt.Tour( ref tempProblem, UsePenalty, UseBacktracking );

                    // now update the pheromone trail (trailInensity)
                    currentAnt.UpdatePheromoneTrail( tempProblem, solutionEdges, EvaporationRate, UsePenalty, InclueAreaCoverage );
                } );
                // reset all pheromone values set by ants
                problem.Graph.VEdges.ForEach( edge => edge.Pheromone = 0 );
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

            return SolveStatus.Feasible;
        }


        private void PreprocessEdges ( Problem P )
        {
            P.Graph.InitializeShortestPath( P.Start );

            _ = Parallel.ForEach( P.Path.Edges, edge =>
            {
                edge.TrailIntensity = 5;
            } );

        }

    }
}