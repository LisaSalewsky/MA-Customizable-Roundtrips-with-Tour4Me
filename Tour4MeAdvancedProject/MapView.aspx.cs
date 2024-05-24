using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Web;
using System.Web.Services;
using System.Web.UI.WebControls;
using Tour4MeAdvancedProject.Helper;
using Tour4MeAdvancedProject.ObjectClasses;
using Tour4MeAdvancedProject.Solver;
using static Tour4MeAdvancedProject.Helper.EnumHelper;
using Path = System.IO.Path;

namespace Tour4MeAdvancedProject
{
    public partial class MapView : System.Web.UI.Page
    {
        //private const double LAT_SEA = 47.45;
        //private const double LON_SEA = -122.4;
        //private const double SIZ_SEA = 2;

        private const double LAT_DOR = 51.3648;
        private const double LON_DOR = 7.2185;


        private static Tuple<double, double> MarkerLatLon;
        //private static readonly bool Zoomed = true;
        //private static readonly List<List<Tuple<double, double>>> OuterBox = new List<List<Tuple<double, double>>>() { };
        //private static readonly List<List<Tuple<double, double>>> InnerBox = new List<List<Tuple<double, double>>>() { };

        //private static readonly double LatGran = 0.5 / 4;
        //private static readonly double LatPad = 0.5 / 6;
        //private static readonly double AbsMinLat = LAT_DOR;

        //private static readonly double LonGran = 0.75 / 4;
        //private static readonly double LonPad = 0.75 / 6;
        //private static readonly double AbsMinLon = LON_DOR;
        //private static readonly string[] COLORS = { "red", "green", "blue", "maroon", "purple", "lime", "navy" };

        //private readonly string SelectedMap = "dor";

        //private readonly int RouteCounter = 0;

        // StoredRoutes: List(RouteID, Path)
        private readonly List<Tuple<int, List<int>>> Stored_routes = new List<Tuple<int, List<int>>>();


        protected void Page_Load ( object sender, EventArgs e )
        {
        }

        protected void GetPathButton_Click ( object sender, EventArgs e )
        {
            Response.Write( "<script>alert('Testing');</script>" );
        }

        protected void ExportTourButton1_Click ( object sender, EventArgs e )
        {

        }

        [WebMethod]
        public static List<Tuple<string, List<object>>> GetPath ( double latIn, double lonIn )
        {
            List<Tuple<string, List<object>>> result = new List<Tuple<string, List<object>>>();
            return result;
        }

        [WebMethod]
        public static Dictionary<string, string> Tour ( double latIn, double lonIn, double distIn,
                                                        string algoIn, string elevationIn, string steepnessIn,
                                                        string surroundingsIn, string[] tagsHIn, List<string> tagsSIn,
                                                        string tourShapeIn, string runningTimeIn,
                                                        double edgeProfitIn, double coveredAreaIn, double elevationImportanceIn
                                                        )
        {
            Process currentProcess = Process.GetCurrentProcess();
            Dictionary<string, string> result = new Dictionary<string, string>();
            long memoryUsage = -1;

            DateTime init_time_1 = DateTime.Now;

            double lat, lon, distance, elevation, steepness, runningTime, edgeProfitImportance, coveredAreaImportance, elevationImportance;
            int algorithm;
            string filename;

            lat = latIn;
            lon = lonIn;
            distance = distIn != 0 ? distIn : 1000;
            algorithm = Convert.ToInt32( algoIn );
            runningTime = Convert.ToDouble( runningTimeIn );
            elevation = elevationIn == "" ? 5000 : Convert.ToDouble( elevationIn );
            steepness = steepnessIn == "" || steepnessIn == "0" ? 100 : Convert.ToDouble( steepnessIn );

            runningTime = runningTime < 5 * 60 ? runningTime : 5 * 60;

            bool noShape = tourShapeIn == "Select " ||
                ( edgeProfitIn == 0 &&
                coveredAreaIn == 0 &&
                elevationImportanceIn == 0 );

            edgeProfitImportance = noShape ? 0.33 : edgeProfitIn;
            coveredAreaImportance = noShape ? 0.33 : coveredAreaIn;
            elevationImportance = noShape ? 0.33 : elevationImportanceIn;


            //double min_lat = lat - (lat - AbsMinLat) % LatGran - LatPad;
            //double max_lat = min_lat + 2 * LatPad + LatGran;
            //double min_lon = lon - (lon - AbsMinLon) % LatGran - LonPad;
            //double max_lon = min_lon + 2 * LonPad + LonGran;

            StringBuilder stream = new StringBuilder();
            //stream.AppendFormat("grid-{0:F4}-{1:F4}-{2:F4}-{3:F4}", max_lat, min_lat, max_lon, min_lon);
            filename = stream.AppendFormat( "grid-dor" ).ToString();//stream.ToString();

            //if (false)
            //{
            //    filename = "grid-sea";
            //}

            FileInfo f = new FileInfo( filename );
            if (f.Exists)
            {
                result.Add( "error", "Grid was not found 404 " );
                return result;
            }

            Problem problem;
            // Locking mechanism (use appropriate synchronization object)
            //lock (problem)
            {
                try
                {

                    string path = Path.GetDirectoryName( Path.GetDirectoryName( Path.GetDirectoryName( System.Reflection.Assembly.GetExecutingAssembly().GetName().CodeBase ) ) );
                    Uri uri = new Uri( path );
                    string localPath = Uri.UnescapeDataString( uri.LocalPath );


                    //problem = new Problem( Path.Combine( localPath, "Tour4MeAdvancedProject", "input", filename + ".txt" ) );
                    problem = new Problem( latIn, lonIn, distIn * 3 / 4, Path.Combine( localPath, "Tour4MeAdvancedProject", "input", filename + ".txt" ) );
                    _ = Guid.TryParse( "123E4567-E89B-12D3-A456-426614174001", out Guid guid );

                    //Problem problemTest = new Problem( guid, out string error );
                    string error = "";
                    if (error != "")
                    {
                        HttpContext currentContext = HttpContext.Current;

                        if (currentContext != null)
                        {
                            // Access the Response property
                            HttpResponse response = currentContext.Response;

                            response.Write( error );
                        }
                    }
                    Console.WriteLine( $"Got request: lat {lat}, lon {lon}, dis {distance}" );
                }
                catch (Exception e)
                {
                    Console.WriteLine( "Something went wrong whilst loading the graph " + e.Message );
                    result.Add( "error", "Something went wrong whilst loading the graph 404 " );
                    return result;
                }
            }


            Console.WriteLine( "tags:" );
            for (int i = 0; i < tagsHIn.Length; i++)
            {
                string[] tagsNameDesireChoice = tagsHIn[ i ].Split( ',' );
                string desireChoice = tagsNameDesireChoice[ 1 ];
                string tagName = tagsNameDesireChoice[ 0 ];
                if (desireChoice == "d")
                {
                    _ = problem.PrefTags.Add( tagName );
                }
                else if (desireChoice == "a")
                {
                    _ = problem.AvoidTags.Add( tagName );
                    Console.WriteLine( tagName );
                }
            }
            for (int i = 0; i < tagsSIn.Count; i++)
            {
                string[] tagsNameDesireChoice = tagsSIn[ i ].Split( ',' );
                string desireChoice = tagsNameDesireChoice[ 1 ];
                string tagName = tagsNameDesireChoice[ 0 ];
                if (desireChoice == "d")
                {
                    _ = problem.PrefTags.Add( tagName );
                }
                else if (desireChoice == "a")
                {
                    _ = problem.AvoidTags.Add( tagName );
                    Console.WriteLine( tagName );
                }
            }
            string[] typeAndValues = surroundingsIn.Split( ':' );
            _ = Enum.TryParse( typeAndValues[ 0 ], true, out Surroundings.SurroundingType surroundingType );
            Console.WriteLine( surroundingType );

            string[] surroundingValues = typeAndValues[ 1 ].Split( ';' );

            if (surroundingValues != null && surroundingValues[ 0 ] != " ")
            {
                for (int i = 0; i < surroundingValues.Length - 1; i++)
                {
                    string[] tagsNameDesireChoice = surroundingValues[ i ].Split( ',' );
                    string desireChoice = tagsNameDesireChoice[ 1 ];
                    string tagName = tagsNameDesireChoice[ 0 ];
                    if (desireChoice == "d")
                    {
                        _ = problem.PrefTags.Add( tagName );
                    }
                    else if (desireChoice == "a")
                    {
                        _ = problem.AvoidTags.Add( tagName );
                        Console.WriteLine( tagName );
                    }
                }
            }

            double best_distance = 1000000000000;
            Node start = null;

            foreach (Node v in problem.Graph.VNodes)
            {
                if (v != null)
                {
                    double dis = v.Distance( lat, lon );

                    if (dis < best_distance)
                    {
                        best_distance = dis;
                        start = v;
                    }
                }
            }

            if (start != null)
            {
                problem.Start = start.GraphNodeId;
                problem.Graph.CenterLon = start.Lon;
                problem.Graph.CenterLat = start.Lat;
            }

            problem.RunningTime = runningTime;

            _ = Enum.TryParse( tourShapeIn, true, out TourShape tourShape );

            switch (tourShape)
            {
                // for a U-Turn, we only need to consider edge profit
                case TourShape.UTurn:
                    problem.EdgeProfitImportance = 0.5;
                    problem.CoveredAreaImportance = 0;
                    problem.ElevationImportance = 0.5;
                    break;
                // for round tours, covered area is very important, edge profit not so much
                case TourShape.Round:
                    problem.EdgeProfitImportance = 0.1;
                    problem.ElevationImportance = 0.1;
                    problem.CoveredAreaImportance = 1 - problem.EdgeProfitImportance - problem.ElevationImportance;
                    break;
                // for complex tours, we prioritize edge profit but try not to generate a U-Turn either 
                case TourShape.Complex:
                    problem.EdgeProfitImportance = 0.4;
                    problem.ElevationImportance = 0.4;
                    problem.CoveredAreaImportance = 1 - problem.EdgeProfitImportance;
                    break;
                // custom tour shapes allow the user to select the values themselves
                case TourShape.Custom:
                default:
                    problem.EdgeProfitImportance = edgeProfitImportance;
                    problem.CoveredAreaImportance = coveredAreaImportance;
                    problem.ElevationImportance = elevationImportance;
                    break;
            }

            problem.MaxElevation = elevation;
            problem.MaxSteepness = steepness;

            problem.Path.Visited.Clear();
            problem.Quality = -1;
            problem.CalculateProfit( problem.Graph );

            problem.TargetDistance = distance;

            DateTime init_time_2 = DateTime.Now;
            DateTime algo_time_1 = DateTime.Now;

            SolveStatus status = SolveStatus.Unsolved;

            string algo = "";


            Console.WriteLine( "start" );
            switch (algorithm)
            {
                case 0:
                    {
                        // Greedy
                        SelectionSolver solver = new SelectionSolver();
                        status = solver.Solve( ref problem );
                        algo = Algo.Greedy.ToString();
                        break;
                    }
                case 1:
                    {
                        // minCost
                        JoggerSolver solver = new JoggerSolver();
                        status = solver.Solve( ref problem );
                        algo = Algo.minCost.ToString();
                        break;
                    }
                case 2:
                    {
                        // ILS
                        //ILS solver = new ILS();
                        //status = solver.Solve( ref problem);
                        algo = Algo.ILS.ToString();
                        break;
                    }
                case 3:
                    {
                        // Ant
                        AntSolver solver = new AntSolver();
                        status = solver.Solve( ref problem );
                        algo = Algo.AntColony.ToString();
                        break;
                    }
                case 4:
                    {
                        // Ant MinCost
                        AntCombined solver = new AntCombined();
                        status = solver.Solve( ref problem, Algo.minCost );
                        algo = Algo.AntMinCost.ToString();
                        break;
                    }
                case 5:
                    {
                        // Ant Greedy
                        AntCombined solver = new AntCombined();
                        status = solver.Solve( ref problem, Algo.Greedy );
                        algo = Algo.AntGreedy.ToString();
                        break;
                    }
                case 6:
                    {
                        // Simmulated Annealing Greedy
                        SimmulatedAnnealingSolver solver = new SimmulatedAnnealingSolver();
                        status = solver.Solve( ref problem, Algo.Greedy );
                        algo = Algo.SimulatedAnnealingGreedy.ToString();
                        break;
                    }
                case 7:
                    {
                        // Simmulated Annealing MinCost
                        SimmulatedAnnealingSolver solver = new SimmulatedAnnealingSolver();
                        status = solver.Solve( ref problem, Algo.minCost );
                        algo = Algo.SimulatedAnnealingMinCost.ToString();
                        break;

                    }
                case 8:
                    {
                        // Simmulated Annealing Ant
                        SimmulatedAnnealingSolver solver = new SimmulatedAnnealingSolver();
                        status = solver.Solve( ref problem, Algo.AntColony );
                        algo = Algo.SimulatedAnnealingAnt.ToString();
                        break;
                    }
                case 9:
                    {
                        // Simmulated Annealing Empty
                        SimmulatedAnnealingSolver solver = new SimmulatedAnnealingSolver();
                        status = solver.Solve( ref problem, Algo.SimulatedAnnealingEmpty );
                        algo = Algo.SimulatedAnnealingEmpty.ToString();
                        break;
                    }
                case 10:
                    {
                        // Genetic
                        SimmulatedAnnealingSolver solver = new SimmulatedAnnealingSolver();
                        status = solver.Solve( ref problem, Algo.SimulatedAnnealingFullyRandom );
                        algo = Algo.SimulatedAnnealingFullyRandom.ToString();
                        break;
                    }
            }
            Console.WriteLine( "end" );

            DateTime algo_time_2 = DateTime.Now;

            int init_time_int = (int)( init_time_2 - init_time_1 ).TotalMilliseconds;
            int algo_time_int = (int)( algo_time_2 - algo_time_1 ).TotalMilliseconds;

            problem.Metadata.Add( "Initialization time (ms): " + init_time_int.ToString() );
            problem.Metadata.Add( "Algorithm computation time (ms): " + algo_time_int.ToString() );
            problem.Metadata.Add( "Algo: " + algo );

            switch (status)
            {
                case SolveStatus.Optimal:
                    problem.Metadata.Add( "Profit: " + problem.GetProfit( problem.Path.Visited ) +
                                         " (theoretical upper bound: " + problem.TargetDistance + ")" );
                    problem.Metadata.Add( "Area: " + Math.Abs( problem.GetArea( problem.Path.Visited ) ) +
                                         " (theoretical upper bound: " +
                                         ( Math.PI * ( problem.TargetDistance / ( 2 * Math.PI ) ) *
                                          ( problem.TargetDistance / ( 2 * Math.PI ) ) ) + ")" );
                    result.Add( "success", "200" );
                    foreach (KeyValuePair<string, string> kv in problem.OutputToResultString())
                    {
                        result.Add( kv.Key, kv.Value );
                    }

                    if (!currentProcess.HasExited)
                    {
                        // Refresh the current process property values.
                        currentProcess.Refresh();

                        memoryUsage = currentProcess.WorkingSet64;

                        Console.WriteLine( $"  Physical memory usage     : {memoryUsage}" );
                    }
                    return result;
                //if (gpx)
                //{
                //    problem.OutputToGPX(filename);
                //}
                case SolveStatus.Feasible:
                    problem.Metadata.Add( "Profit: " + problem.GetProfit( problem.Path.Visited ) +
                                         " (theoretical upper bound: " + problem.TargetDistance + ")" );
                    //problem.Metadata.Add( "Area: " + Math.Abs( problem.GetArea( problem.Path.Visited ) ) +
                    //                     " (theoretical upper bound: " +
                    //                     ( Math.PI * ( problem.TargetDistance / ( 2 * Math.PI ) ) *
                    //                      ( problem.TargetDistance / ( 2 * Math.PI ) ) ) + ")" );
                    problem.Metadata.Add( "Area: " + 1 +
                                         " (theoretical upper bound: " +
                                         ( Math.PI * ( problem.TargetDistance / ( 2 * Math.PI ) ) *
                                          ( problem.TargetDistance / ( 2 * Math.PI ) ) ) + ")" );
                    result.Add( "success", "200" );

                    if (!currentProcess.HasExited)
                    {
                        // Refresh the current process property values.
                        currentProcess.Refresh();
                        //     Gets the amount of physical memory, in bytes
                        memoryUsage = currentProcess.WorkingSet64;

                        Console.WriteLine( $"  Physical memory usage     : {memoryUsage}" );
                    }
                    problem.Metadata.Add( "MemoryUsage: " + memoryUsage );
                    foreach (KeyValuePair<string, string> kv in problem.OutputToResultString())
                    {
                        result.Add( kv.Key, kv.Value );
                    }
                    return result;
                //if (gpx)
                //{
                //    problem.OutputToGPX(filename);
                //}
                case SolveStatus.Unsolved:
                    result.Add( "error", "Not solved 400" );

                    if (!currentProcess.HasExited)
                    {
                        // Refresh the current process property values.
                        currentProcess.Refresh();

                        memoryUsage = currentProcess.WorkingSet64;

                        Console.WriteLine( $"  Physical memory usage     : {memoryUsage}" );
                    }
                    problem.Metadata.Add( "MemoryUsage: " + memoryUsage );
                    return result;
                case SolveStatus.Timeout:
                    result.Add( "error", "Timeout 504" );

                    if (!currentProcess.HasExited)
                    {
                        // Refresh the current process property values.
                        currentProcess.Refresh();

                        memoryUsage = currentProcess.WorkingSet64;

                        Console.WriteLine( $"  Physical memory usage     : {memoryUsage}" );
                    }
                    problem.Metadata.Add( "MemoryUsage: " + memoryUsage );
                    return result;
            }
            result.Add( "error", "Not solved 400" );


            if (!currentProcess.HasExited)
            {
                // Refresh the current process property values.
                currentProcess.Refresh();

                memoryUsage = currentProcess.WorkingSet64;

                Console.WriteLine( $"  Physical memory usage     : {memoryUsage}" );
            }
            problem.Metadata.Add( "MemoryUsage: " + memoryUsage );

            return result;

        }

        //public static string SetLatLon() 
        [WebMethod]
        public static string SetLatLon ( double latIn, double lonIn )
        {
            MarkerLatLon = new Tuple<double, double>( latIn, lonIn );
            return "test";
        }

        [WebMethod]
        public static Dictionary<string, List<KeyValuePair<int, string>>> RenderGraph ()
        {
            Dictionary<string, List<KeyValuePair<int, string>>> result = new Dictionary<string, List<KeyValuePair<int, string>>>();

            List<KeyValuePair<int, string>> tagList = Enum.GetValues( typeof( HighwayTag ) )
            .Cast<HighwayTag>()
            .Select( ( value, index ) => new KeyValuePair<int, string>( index, value.ToString() ) )
            .ToList();
            result.Add( "highway", tagList );

            tagList = Enum.GetValues( typeof( SurfaceTag ) )
           .Cast<SurfaceTag>()
           .Select( ( value, index ) => new KeyValuePair<int, string>( index, value.ToString() ) )
           .ToList();
            result.Add( "surface", tagList );

            List<KeyValuePair<int, string>> algoList = Enum.GetValues( typeof( Algo ) )
            .Cast<Algo>()
            .Select( ( value, index ) => new KeyValuePair<int, string>( index, value.ToString() ) )
            .ToList();
            result.Add( "algorithms", algoList );

            List<KeyValuePair<int, string>> shapeList = Enum.GetValues( typeof( TourShape ) )
            .Cast<TourShape>()
            .Select( ( value, index ) => new KeyValuePair<int, string>( index, value.ToString() ) )
            .ToList();
            result.Add( "shapes", shapeList );

            List<KeyValuePair<Surroundings.SurroundingType, string[]>> surroundingsList = Surroundings.Values
            .Select( pair => new KeyValuePair<Surroundings.SurroundingType, string[]>( pair.Key, pair.Value ) )
            .ToList();
            List<KeyValuePair<int, string>> formattedSurroundingsList = Surroundings.Values
            .Select( ( pair, index ) => new KeyValuePair<int, string>(
                index,
                $"{pair.Key}: {string.Join( ", ", pair.Value )}" ) )
            .ToList();

            result.Add( "surroundings", formattedSurroundingsList );

            return result;
        }

        [WebMethod]
        public static List<string> DrawGraph ( double latIn, double lonIn, double distIn )
        {
            List<string> result = new List<string>();
            double lat, lon, distance;

            lat = latIn;
            lon = lonIn;
            distance = distIn;

            StringBuilder stream = new StringBuilder();

            string filename = stream.AppendFormat( "grid-dor" ).ToString();

            FileInfo f = new FileInfo( filename );
            if (f.Exists)
            {
                result.Add( "error: Grid was not found 404 " );
                return result;
            }

            Problem problem;
            {
                try
                {

                    string path = Path.GetDirectoryName( Path.GetDirectoryName( Path.GetDirectoryName( System.Reflection.Assembly.GetExecutingAssembly().GetName().CodeBase ) ) );
                    Uri uri = new Uri( path );
                    string localPath = Uri.UnescapeDataString( uri.LocalPath );


                    //problem = new Problem( Path.Combine( localPath, "Tour4MeAdvancedProject", "input", filename + ".txt" ) );
                    problem = new Problem( latIn, lonIn, distIn * 3 / 4, Path.Combine( localPath, "Tour4MeAdvancedProject", "input", filename + ".txt" ) );
                    _ = Guid.TryParse( "123E4567-E89B-12D3-A456-426614174001", out Guid guid );

                    //Problem problemTest = new Problem( guid, out string error );
                    string error = "";
                    if (error != "")
                    {
                        HttpContext currentContext = HttpContext.Current;

                        if (currentContext != null)
                        {
                            // Access the Response property
                            HttpResponse response = currentContext.Response;

                            response.Write( error );
                        }
                    }
                    Console.WriteLine( $"Got request: lat {lat}, lon {lon}, dis {distance}" );
                }
                catch (Exception e)
                {
                    Console.WriteLine( "Something went wrong whilst loading the graph " + e.Message );
                    result.Add( "error: Something went wrong whilst loading the graph 404 " );
                    return result;
                }
            }

            StringBuilder outputString = new StringBuilder();

            foreach (Edge e in problem.Graph.VEdges)
            {
                if (e != null)
                {
                    _ = outputString.AppendFormat( CultureInfo.InvariantCulture, "[[{0:F6},{1:F6}],",
                        e.SourceNode.Lat, e.SourceNode.Lon );

                    bool reverse = e.GeoLocations != null && e.GeoLocations.Count > 0 && ( e.SourceNode.Lat != e.GeoLocations.First().Item1 || e.SourceNode.Lon != e.GeoLocations.First().Item2 );
                    List<Tuple<double, double>> locationList = new List<Tuple<double, double>>( e.GeoLocations );

                    if (reverse)
                    {
                        locationList.Reverse();
                        //edge.GeoLocations.Reverse();
                    }
                    foreach ((double locationsLat, double locationsLon) in locationList)
                    {
                        _ = outputString.AppendFormat( CultureInfo.InvariantCulture, "[{0:F6},{1:F6}],",
                            locationsLat, locationsLon );
                    }
                    _ = outputString.AppendFormat( CultureInfo.InvariantCulture, "[{0:F6},{1:F6}]]/",
                        e.TargetNode.Lat, e.TargetNode.Lon );

                }
            }

            result.Add( outputString.ToString().Substring( 0, outputString.Length - 1 ) );

            return result;

        }

    }
}