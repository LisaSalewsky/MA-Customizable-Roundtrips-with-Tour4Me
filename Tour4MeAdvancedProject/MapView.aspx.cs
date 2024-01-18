using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Web;
using System.Web.Services;
using System.Web.UI.WebControls;
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
        private static readonly bool Zoomed = true;
        // OuterBox: List(List(Lat,Lon)) = List((minmin(LatLon), minmax(LatLon), maxmax(LatLon), maxmin(LatLon))
        private static readonly List<List<Tuple<double, double>>> OuterBox = new List<List<Tuple<double, double>>>() { };
        // InnerBox: List(List(Lat,Lon)) = List((minmin(LatLon), minmax(LatLon), maxmax(LatLon), maxmin(LatLon))
        private static readonly List<List<Tuple<double, double>>> InnerBox = new List<List<Tuple<double, double>>>() { };

        private static readonly double LatGran = 0.5 / 4;
        private static readonly double LatPad = 0.5 / 6;
        private static readonly double AbsMinLat = LAT_DOR;
        //private static double AbsMaxLat = AbsMinLat + LatGran;

        private static readonly double LonGran = 0.75 / 4;
        private static readonly double LonPad = 0.75 / 6;
        private static readonly double AbsMinLon = LON_DOR;
        //private static double AbsMaxLon = AbsMinLon + LonGran;
        private static readonly string[] COLORS = { "red", "green", "blue", "maroon", "purple", "lime", "navy" };

        private readonly string SelectedMap = "dor";

        private readonly int RouteCounter = 0;

        // StoredRoutes: List(RouteID, Path)
        private readonly List<Tuple<int, List<int>>> Stored_routes = new List<Tuple<int, List<int>>>();

        //private double CenterLat = 51.489808;
        //private double CenterLon = 7.406319;


        protected void Page_Load ( object sender, EventArgs e )
        {
        }

        protected void GetPathButton_Click ( object sender, EventArgs e )
        {
            Response.Write( "<script>alert('Testing');</script>" );
        }

        [WebMethod]
        public static List<Tuple<string, List<object>>> GetPath ( double latIn, double lonIn )
        {
            List<Tuple<string, List<object>>> result = new List<Tuple<string, List<object>>>();
            return result;
        }

        [WebMethod]
        public static Dictionary<string, string> Tour ( double latIn, double lonIn, double distIn,
                                                                string algoIn, string[] tagsHIn, List<string> tagsSIn,
                                                                string runningTimeIn, double edgeProfitIn, double coveredAreaIn
                                                                )
        {
            Dictionary<string, string> result = new Dictionary<string, string>();


            DateTime init_time_1 = DateTime.Now;

            double lat, lon, distance, runningTime, edgeProfitImportance, coveredAreaImportance;
            int algorithm;
            string filename;

            lat = latIn;
            lon = lonIn;
            distance = distIn;
            algorithm = Convert.ToInt32( algoIn );
            runningTime = Convert.ToDouble( runningTimeIn );

            runningTime = runningTime < 5 * 60 ? runningTime : 5 * 60;

            edgeProfitImportance = edgeProfitIn;
            coveredAreaImportance = coveredAreaIn;


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


                    problem = new Problem( Path.Combine( localPath, "Tour4MeAdvancedProject", "input", filename + ".txt" ) );
                    _ = Guid.TryParse( "123E4567-E89B-12D3-A456-426614174001", out Guid guid );
                    Problem problemTest = new Problem( guid, out string error );
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


            double best_distance = 1000000000000;
            Node start = null;

            foreach (Node v in problem.Graph.VNodes)
            {
                double dis = v.Distance( lat, lon );

                if (dis < best_distance)
                {
                    best_distance = dis;
                    start = v;
                }
            }

            if (start != null)
            {
                problem.Start = start.Id;
                problem.Graph.CenterLon = start.Lon;
                problem.Graph.CenterLat = start.Lat;
            }

            problem.RunningTime = runningTime;
            problem.EdgeProfitImportance = edgeProfitImportance;
            problem.CoveredAreaImportance = coveredAreaImportance;

            problem.Path.Visited.Clear();
            problem.Quality = -1;
            problem.CalculateProfit( problem.Graph );

            problem.TargetDistance = distance;

            DateTime init_time_2 = DateTime.Now;
            DateTime algo_time_1 = DateTime.Now;

            SolveStatus status = SolveStatus.Unsolved;

            Console.WriteLine( "start" );
            switch (algorithm)
            {
                case 0:
                    {
                        SelectionSolver solver = new SelectionSolver();
                        status = solver.Solve( problem );
                        break;
                    }
                case 1:
                    {
                        JoggerSolver solver = new JoggerSolver();
                        status = solver.Solve( problem );
                        break;
                    }
                case 2:
                    {
                        //Jogger jogSolver = new Jogger();
                        //status = jogSolver.Solve(problem);
                        //if (status == SolveStatus.Unsolved)
                        break;

                        //ILS solver = new ILS();
                        //status = solver.Solve(problem);
                        //break;
                    }
            }
            Console.WriteLine( "end" );

            DateTime algo_time_2 = DateTime.Now;

            int init_time_int = (int)( init_time_2 - init_time_1 ).TotalMilliseconds;
            int algo_time_int = (int)( algo_time_2 - algo_time_1 ).TotalMilliseconds;

            problem.Metadata.Add( "Initialization time (ms): " + init_time_int.ToString() );
            problem.Metadata.Add( "Algorithm computation time (ms): " + algo_time_int.ToString() );

            switch (status)
            {
                case SolveStatus.Optimal:
                    problem.Metadata.Add( "Profit: " + problem.GetProfit( problem.Path ) +
                                         " (theoretical upper bound: " + problem.TargetDistance + ")" );
                    problem.Metadata.Add( "Area: " + Math.Abs( problem.GetArea( problem.Path ) ) +
                                         " (theoretical upper bound: " +
                                         ( Math.PI * ( problem.TargetDistance / ( 2 * Math.PI ) ) *
                                          ( problem.TargetDistance / ( 2 * Math.PI ) ) ) + ")" );
                    result.Add( "success", "200" );
                    foreach (KeyValuePair<string, string> kv in problem.OutputToResultString())
                    {
                        result.Add( kv.Key, kv.Value );
                    }
                    return result;
                //if (gpx)
                //{
                //    problem.OutputToGPX(filename);
                //}
                case SolveStatus.Feasible:
                    problem.Metadata.Add( "Profit: " + problem.GetProfit( problem.Path ) +
                                         " (theoretical upper bound: " + problem.TargetDistance + ")" );
                    problem.Metadata.Add( "Area: " + Math.Abs( problem.GetArea( problem.Path ) ) +
                                         " (theoretical upper bound: " +
                                         ( Math.PI * ( problem.TargetDistance / ( 2 * Math.PI ) ) *
                                          ( problem.TargetDistance / ( 2 * Math.PI ) ) ) + ")" );
                    result.Add( "success", "200" );
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
                    return result;
                case SolveStatus.Timeout:
                    result.Add( "error", "Timeout 504" );
                    return result;
            }
            result.Add( "error", "Not solved 400" );
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

            return result;
        }

    }
}