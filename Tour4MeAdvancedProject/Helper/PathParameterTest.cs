﻿
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Web;
using System.Web.Script.Services;
using System.Web.Services;
using Tour4MeAdvancedProject.ObjectClasses;
using Tour4MeAdvancedProject.Solver;
using static Tour4MeAdvancedProject.Helper.EnumHelper;
using Path = System.IO.Path;


namespace Tour4MeAdvancedProject.Helper
{
    [WebService( Namespace = "http://localhost:44323/" )]
    [WebServiceBinding( ConformsTo = WsiProfiles.BasicProfile1_1 )]
    [System.Web.Script.Services.ScriptService]
    public partial class PathParameterTest : WebService
    {

        [WebMethod]
        [ScriptMethod( UseHttpGet = true )]
        public string GeneratePathTestPlot ( int numberPaths )
        {
            StringBuilder jsonBuilder = new StringBuilder();

            _ = jsonBuilder.Append( "[\n" );

            double lat = 51.489808;
            double lon = 7.406319;
            double distance = 5000;
            double elevation = 100;
            double steepness = 0.05;
            string[] tagsHIn = new string[] { };
            List<string> tagsSIn = new List<string>() { "Paved,d",
                                                        "Cobblestone,d",
                                                        "Gravel,d",
                                                        "Unpaved,d",
                                                        "Compacted,d",
                                                        "FineGravel,d",
                                                        "Rock,d",
                                                        "Pebblestone,d" };

            string surroundingsIn = "forest: tree,d";



            double edgeProfitImportance = 0.5;
            double coveredAreaImportance = 0.5;
            double elevationImportance = 0;

            StringBuilder stream = new StringBuilder();
            string filename = stream.AppendFormat( "grid-dor" ).ToString();//stream.ToString();
            _ = new FileInfo( filename );

            Problem problem = null;
            {
                try
                {

                    string path = System.IO.Path.GetDirectoryName( Path.GetDirectoryName( Path.GetDirectoryName( System.Reflection.Assembly.GetExecutingAssembly().GetName().CodeBase ) ) );
                    Uri uri = new Uri( path );
                    string localPath = Uri.UnescapeDataString( uri.LocalPath );


                    //problem = new Problem( Path.Combine( localPath, "Tour4MeAdvancedProject", "input", filename + ".txt" ) );
                    problem = new Problem( lat, lon, distance * 3 / 4, Path.Combine( localPath, "Tour4MeAdvancedProject", "input", filename + ".txt" ) );
                    _ = Guid.TryParse( "123E4567-E89B-12D3-A456-426614174001", out Guid guid );

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
                double dis = v.Distance( lat, lon );

                if (dis < best_distance)
                {
                    best_distance = dis;
                    start = v;
                }
            }

            if (start != null)
            {
                problem.Start = start.GraphNodeId;
                problem.Graph.CenterLon = start.Lon;
                problem.Graph.CenterLat = start.Lat;
            }


            _ = Enum.TryParse( "Round", true, out TourShape tourShape );

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
            _ = DateTime.Now;
            _ = DateTime.Now;
            Console.WriteLine( "start" );

            Color[] colors = GenerateColors( numberPaths );

            for (int i = 1; i <= numberPaths; i++)
            {
                double alpha = i / numberPaths;
                double beta = 1 - alpha;
                AntSolver solver = new AntSolver( 2, 100, alpha, beta );
                SolveStatus status = solver.Solve( ref problem );
                Console.WriteLine( "end" );

                string result = "";
                switch (status)
                {
                    case SolveStatus.Optimal:
                        problem.Metadata.Add( "Profit: " + problem.GetProfit( problem.Path ) +
                                             " (theoretical upper bound: " + problem.TargetDistance + ")" );
                        problem.Metadata.Add( "Area: " + Math.Abs( problem.GetArea( problem.Path ) ) +
                                             " (theoretical upper bound: " +
                                             ( Math.PI * ( problem.TargetDistance / ( 2 * Math.PI ) ) *
                                              ( problem.TargetDistance / ( 2 * Math.PI ) ) ) + ")" );
                        foreach (KeyValuePair<string, string> kv in GenerateOutputString( problem ))
                        {
                            if (kv.Key == "path")
                            {
                                result = kv.Value;
                            }
                        }
                        _ = jsonBuilder.Append( "    {\n" );
                        _ = jsonBuilder.Append( "    \"tour\": \"Tour" + i + "\",\n" );
                        _ = jsonBuilder.Append( "    \"values\": " + result + ",\n" );
                        _ = jsonBuilder.Append( "    \"color\": " + ConvertColorToString( colors[ i - 1 ] ) + ",\n" );
                        _ = jsonBuilder.Append( "    \"opacity\": " + ( 1.0 / numberPaths ).ToString( System.Globalization.CultureInfo.InvariantCulture ) );

                        _ = i == numberPaths ? jsonBuilder.Append( "\n    }\n ]" ) : jsonBuilder.Append( "\n    },\n" );
                        break;
                    case SolveStatus.Feasible:
                        foreach (KeyValuePair<string, string> kv in GenerateOutputString( problem ))
                        {
                            result = kv.Value;
                        }
                        _ = jsonBuilder.Append( "    {\n" );
                        _ = jsonBuilder.Append( "    \"tour\": \"Tour" + i + "\",\n" );
                        _ = jsonBuilder.Append( "    \"values\": \"" + result + "\",\n" );
                        _ = jsonBuilder.Append( "    \"color\": " + ConvertColorToString( colors[ i - 1 ] ) + ",\n" );
                        _ = jsonBuilder.Append( "    \"opacity\": " + ( 1.0 / numberPaths ).ToString( System.Globalization.CultureInfo.InvariantCulture ) );

                        _ = i == numberPaths ? jsonBuilder.Append( "\n    }\n ]" ) : jsonBuilder.Append( "\n    },\n" );
                        break;
                }

            }
            return jsonBuilder.ToString();
        }

        public static string ConvertColorToString ( Color color )
        {
            return $"{{\"A\":{color.A},\"R\":{color.R},\"G\":{color.G},\"B\":{color.B}}}";
        }

        public Color[] GenerateColors ( int count )
        {
            List<Color> colors = new List<Color>();

            // Generate colors evenly distributed between red and blue
            for (int i = 0; i < count; i++)
            {
                float hue = (float)i / ( count - 1 ); // Interpolate hue from red (0) to blue (1)
                colors.Add( ColorFromAhsb( 255, hue, 1.0f, 1.0f ) );
            }

            return colors.ToArray();
        }

        private Color ColorFromAhsb ( int alpha, float hue, float saturation, float brightness )
        {
            float hueScaled = hue * 360; // Scale hue to degrees (0 - 360)

            // Convert HSB (Hue, Saturation, Brightness) to RGB color representation
            int hi = Convert.ToInt32( Math.Floor( hueScaled / 60 ) ) % 6;
            float f = ( hueScaled / 60 ) - (float)Math.Floor( hueScaled / 60 );

            brightness *= 255;
            byte v = Convert.ToByte( brightness );
            byte p = Convert.ToByte( brightness * ( 1 - saturation ) );
            byte q = Convert.ToByte( brightness * ( 1 - ( f * saturation ) ) );
            byte t = Convert.ToByte( brightness * ( 1 - ( ( 1 - f ) * saturation ) ) );

            return hi == 0
                ? Color.FromArgb( alpha, v, t, p )
                : hi == 1
                    ? Color.FromArgb( alpha, q, v, p )
                    : hi == 2
                        ? Color.FromArgb( alpha, p, v, t )
                        : hi == 3
                            ? Color.FromArgb( alpha, p, q, v )
                            : hi == 4
                                ? Color.FromArgb( alpha, t, p, v )
                                : Color.FromArgb( alpha, v, p, q );
        }
        public List<KeyValuePair<string, string>> GenerateOutputString ( Problem P )
        {

            List<KeyValuePair<string, string>> result = new List<KeyValuePair<string, string>>();
            StringBuilder outputString = new StringBuilder( "[" );
            P.Path.Visited.Add( P.Path.Visited.ElementAt( 0 ) );

            for (int i = 0; i < P.Path.Visited.Count() - 1; i++)
            {
                int node = P.Path.Visited.ElementAt( i );
                _ = outputString.AppendFormat( CultureInfo.InvariantCulture, "[{0:F6},{1:F6}],",
                    P.Graph.VNodes[ node ].Lat, P.Graph.VNodes[ node ].Lon );


                // todo check if i can use solution edges instead
                Edge edge = P.Graph.GetEdge( node, P.Path.Visited.ElementAt( i + 1 ) );

                if (edge == null)
                {
                    continue;
                }

                bool reverse = node != edge.SourceNode.GraphNodeId;
                List<Tuple<double, double>> locationList = new List<Tuple<double, double>>( edge.GeoLocations );
                if (reverse)
                {
                    locationList.Reverse();
                }
                foreach ((double lat, double lon) in locationList)
                {
                    _ = outputString.AppendFormat( CultureInfo.InvariantCulture, "[{0:F6},{1:F6}],",
                        lat, lon );
                }
                _ = outputString.AppendFormat( CultureInfo.InvariantCulture, "[{0:F6},{1:F6}],",
                    P.Graph.VNodes[ P.Path.Visited.First() ].Lat, P.Graph.VNodes[ P.Path.Visited.First() ].Lon );

            }
            _ = outputString.Remove( outputString.Length - 1, 1 );
            _ = outputString.Append( "]" );
            result.Add( new KeyValuePair<string, string>( "path", outputString.ToString() ) );
            return result;
        }
    }
}