using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using SysPath = System.IO.Path;


namespace Tour4MeAdvancedProject.ObjectClasses
{

    public class Problem
    {
        protected string graphName;

        private Guid Id;

        public int Start { get; set; }
        public double CenterLat { get; set; }
        public double CenterLon { get; set; }
        public HashSet<string> PrefTags { get; set; }
        public HashSet<string> AvoidTags { get; set; }
        public double MaxElevation { get; set; }
        public double MaxSteepness { get; set; }
        public Graph Graph { get; set; }
        public Graph Backbone { get; set; }
        public double RunningTime { get; set; }
        public double EdgeProfitImportance { get; set; }
        public double CoveredAreaImportance { get; set; }
        public double ElevationImportance { get; set; }
        public List<List<double>> ShortestPath { get; set; }

        public Path Path { get; set; }
        public double Quality { get; set; }

        public double TargetDistance { get; set; }
        public List<string> Metadata { get; set; }

        public Problem ( string fileName )
        {
            Graph = new Graph( fileName );
            Path = new Path();
            Metadata = new List<string>();
            PrefTags = new HashSet<string>();
            AvoidTags = new HashSet<string>();
        }

        //public Problem ( double startLat, double startLon, double givenMaxLat, double givenMaxLon, double givenMinLat, double givenMinLon, double radius, string fileName )
        public Problem ( double startLat, double startLon, double radius, string fileName )
        {
            //Graph = new Graph( startLat, startLon, givenMaxLat, givenMaxLon, givenMinLat, givenMinLon, radius, fileName );
            Graph = new Graph( startLat, startLon, radius, fileName );
            Path = new Path();
            Metadata = new List<string>();
            PrefTags = new HashSet<string>();
            AvoidTags = new HashSet<string>();

        }

        //public Problem ( Guid ProblemId, out string error )
        //{
        //    error = "";
        //    Id = ProblemId;
        //    _ = Guid.TryParse( "123E4567-E89B-12D3-A456-426614174001", out Guid guid );
        //    Graph = new Graph( guid, out string graphError );
        //    Path = new Path();
        //    Metadata = new List<string>();
        //    PrefTags = new HashSet<string>();
        //    AvoidTags = new HashSet<string>();
        //    error += graphError;
        //}

        public void FillShortestPath ( string filename )
        {
            string path = SysPath.GetDirectoryName( SysPath.GetDirectoryName( SysPath.GetDirectoryName( System.Reflection.Assembly.GetExecutingAssembly().GetName().CodeBase ) ) );
            Uri uri = new Uri( path );
            string localPath = Uri.UnescapeDataString( uri.LocalPath );
            string finalPath = SysPath.Combine( localPath, "input", filename, "_sp.txt" );

            using (StreamWriter outputFile = new StreamWriter( finalPath ))
            {

                ShortestPath = new List<List<double>>( Graph.VNodes.Count );

                for (int source = 0; source < Graph.VNodes.Count; source++)
                {
                    ShortestPath.Add( new List<double>( Graph.VNodes.Count ) );
                    Dictionary<int, double> dist = new Dictionary<int, double>();
                    PriorityQueue<Tuple<int, double>> queue = new PriorityQueue<Tuple<int, double>>();

                    dist[ source ] = 0.0;
                    queue.Enqueue( 0.0, new Tuple<int, double>( source, 0.0 ) );

                    while (queue.Count > 0)
                    {
                        (double distance, (int currentNode, double actual)) = queue.Dequeue();

                        if (!dist.TryGetValue( currentNode, out double bestKnownDist ))
                        {
                            dist[ currentNode ] = distance;
                            bestKnownDist = distance;
                        }

                        if (bestKnownDist != distance)
                        {
                            continue;
                        }

                        foreach (Edge edge in Graph.VNodes[ currentNode ].Incident)
                        {
                            int neighborId = edge.SourceNode.GraphNodeId == currentNode ? edge.TargetNode.GraphNodeId : edge.SourceNode.GraphNodeId;
                            double newDistance = bestKnownDist + edge.Cost;

                            if (!dist.TryGetValue( neighborId, out double currentDist ))
                            {
                                dist[ neighborId ] = double.MaxValue;
                            }

                            if (newDistance < currentDist)
                            {
                                queue.Enqueue( newDistance, new Tuple<int, double>( neighborId, 0.0 ) );
                                dist[ neighborId ] = newDistance;
                            }
                        }
                    }

                    for (int target = 0; target < Graph.VNodes.Count; target++)
                    {
                        ShortestPath[ source ].Add( dist.ContainsKey( target ) ? dist[ target ] : double.MaxValue );
                        outputFile.WriteLine( $"c" + source + target + ( dist.ContainsKey( target ) ? dist[ target ].ToString() : "inf" ) );
                    }
                }
            }
        }

        public void OutputPath ( string fileName )
        {

            using (StreamWriter outputFile = new StreamWriter( "C:/Users/Lisa Salewsky/OneDrive" +
                "/Dokumente/Uni/Dortmund/Master/MA/Tour4MeAdvanced/" +
                "Tour4MeExtensions/Tour4MeAdvancedProject/output/" + fileName + ".json" ))
            {
                outputFile.WriteLine( "{" );
                outputFile.WriteLine( "    \"info\": {" );
                outputFile.WriteLine( $"        \"graph_name\": \"{graphName}\"," );
                outputFile.WriteLine( $"        \"n_nodes\": {Graph.VNodes.Count}," );
                outputFile.WriteLine( $"        \"target_distance\": {TargetDistance}," );
                outputFile.WriteLine( $"        \"center_lat\": {Graph.CenterLat}," );
                outputFile.WriteLine( $"        \"center_lon\": {Graph.CenterLon}" );
                outputFile.WriteLine( "    }," );
                outputFile.WriteLine( "    \"node_path\": [" );

                foreach (int node in Path.Visited)
                {
                    outputFile.Write( $"        {Graph.VNodes[ node ].NodeId}, " );
                }

                outputFile.WriteLine( $"{Graph.VNodes[ Path.Visited.First() ].NodeId}], " );
                outputFile.WriteLine( "    \"cord_path\": [" );

                foreach (int node in Path.Visited)
                {
                    outputFile.Write( $"[{Graph.VNodes[ node ].Lat}, {Graph.VNodes[ node ].Lon}], " );
                }

                outputFile.WriteLine( "]" );
                outputFile.WriteLine( "}" );
            }
        }

        public void OutputToGPX ( string fileName )
        {
            using (StreamWriter outputFile = new StreamWriter( "C:/Users/Lisa Salewsky/OneDrive" +
                "/Dokumente/Uni/Dortmund/Master/MA/Tour4MeAdvanced/" +
                "Tour4MeExtensions/Tour4MeAdvancedProject/output/" + fileName + ".gpx" ))
            {
                outputFile.WriteLine( "<?xml version='1.0' encoding='UTF-8'?>" );
                outputFile.WriteLine( "<gpx version=\"1.1\" creator=\"JeBoi\" " +
                    "xmlns=\"http://www.topografix.com/GPX/1/1\" " +
                    "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" " +
                    "xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 " +
                    "http://www.topografix.com/GPX/1/1/gpx.xsd\">" );
                outputFile.WriteLine( "  <metadata>" );
                outputFile.WriteLine( $"    <name>{fileName}</name>" );
                outputFile.WriteLine( "  </metadata>" );
                outputFile.WriteLine( "  <trk>" );
                outputFile.WriteLine( $"    <name>{fileName}</name>" );
                outputFile.WriteLine( "    <trkseg>" );

                foreach (int node in Path.Visited)
                {
                    outputFile.WriteLine( $"      <trkpt lat=\"{Graph.VNodes[ node ].Lat}\" " +
                        $"lon=\"{Graph.VNodes[ node ].Lon}\"></trkpt>" );
                }

                outputFile.WriteLine( "    </trkseg>" );
                outputFile.WriteLine( "  </trk>" );
                outputFile.WriteLine( "</gpx>" );
            }
        }

        public string OutputToString ()
        {
            //StringBuilder outputString = new StringBuilder("{\n    \"path\": [\n");
            StringBuilder outputString = new StringBuilder( "{\n    [\n" );

            foreach (int node in Path.Visited)
            {
                _ = outputString.Append( $"        [{Graph.VNodes[ node ].Lat},{Graph.VNodes[ node ].Lon}], " +
                    $"\n" );

                Edge edge = Graph.GetEdge( node, Path.Visited.ToList().ElementAtOrDefault(
                    Path.Visited.ToList().IndexOf( node ) + 1 ) );

                if (edge == null)
                {
                    continue;
                }

                bool reverse = node < edge.TargetNode.GraphNodeId;
                if (!reverse)
                {
                    edge.GeoLocations.Reverse();
                    reverse = true;
                }
                foreach ((double lat, double lon) in edge.GeoLocations)
                {
                    _ = outputString.Append( $"        [{lat},{lon}], \n" );
                }
            }

            _ = outputString.Append( $"        [{Graph.VNodes[ Path.Visited.First() ].Lat}," +
                $"{Graph.VNodes[ Path.Visited.First() ].Lon}] \n" );
            _ = outputString.Append( "    ],\n" );
            _ = outputString.Append( "    \"meta\": [\n" );

            for (int i = 0; i < Metadata.Count - 1; i++)
            {
                _ = outputString.Append( $"        \"{Metadata[ i ]}\",\n" );
            }

            _ = outputString.Append( $"        \"{Metadata[ Metadata.Count - 1 ]}\"\n" );
            _ = outputString.Append( "    ]\n}" );

            return outputString.ToString();
        }

        public List<KeyValuePair<string, string>> OutputToResultString ()
        {
            List<KeyValuePair<string, string>> result = new List<KeyValuePair<string, string>>();
            StringBuilder outputString = new StringBuilder( "[" );
            Path.Visited.Add( Path.Visited.ElementAt( 0 ) );

            for (int i = 0; i < Path.Visited.Count() - 1; i++)
            {
                int node = Path.Visited.ElementAt( i );
                _ = outputString.AppendFormat( CultureInfo.InvariantCulture, "[{0:F6},{1:F6}],",
                    Graph.VNodes[ node ].Lat, Graph.VNodes[ node ].Lon );


                // todo check if i can use solution edges instead
                Edge edge = Graph.GetEdge( node, Path.Visited.ElementAt( i + 1 ) );

                if (edge == null)
                {
                    continue;
                }

                bool reverse = edge.GeoLocations != null &&
                    edge.GeoLocations.Count > 0 &&
                    ( Graph.VNodes[ node ].Lat != edge.GeoLocations.First().Item1 ||
                    Graph.VNodes[ node ].Lon != edge.GeoLocations.First().Item2 );
                List<Tuple<double, double>> locationList = new List<Tuple<double, double>>( edge.GeoLocations );
                if (reverse)
                {
                    locationList.Reverse();
                    //edge.GeoLocations.Reverse();
                }
                foreach ((double lat, double lon) in locationList)
                {
                    _ = outputString.AppendFormat( CultureInfo.InvariantCulture, "[{0:F6},{1:F6}],",
                        lat, lon );
                }
            }
            _ = outputString.AppendFormat( CultureInfo.InvariantCulture, "[{0:F6},{1:F6}]",
                Graph.VNodes[ Path.Visited.First() ].Lat, Graph.VNodes[ Path.Visited.First() ].Lon );

            _ = outputString.Append( "]" );

            result.Add( new KeyValuePair<string, string>( "path", outputString.ToString() ) );

            outputString = new StringBuilder( "[" );

            for (int i = 0; i < Metadata.Count - 1; i++)
            {
                _ = outputString.Append( $"\"{Metadata[ i ]}\"," );
            }

            _ = outputString.Append( $"\"{Metadata[ Metadata.Count - 1 ]}\"" );
            _ = outputString.Append( "]" );

            result.Add( new KeyValuePair<string, string>( "meta", outputString.ToString() ) );

            return result;
        }

        public void CalculateProfit ( Graph G )
        {
            foreach (Edge edge in G.VEdges)
            {
                edge.Profit = 0.0001;
                foreach (string tag in edge.Tags)
                {
                    if (Math.Abs( edge.Profit - 0.0001 ) < double.Epsilon && PrefTags.Contains( tag ))
                    {
                        edge.Profit = 1;
                    }
                    if (AvoidTags.Contains( tag ))
                    {
                        edge.Profit = -1;
                    }
                }
            }
        }

        public double GetQuality ( double profit, double area )
        {
            return ( EdgeProfitImportance * profit / TargetDistance ) + ( CoveredAreaImportance * area / ( Math.PI * TargetDistance * TargetDistance ) );
        }

        public double GetProfit ( Path path )
        {
            //path.Visited = new List<int>( Graph.VEdges.Count );
            int[] visited = new int[ Graph.VEdges.Count ];
            double quality = 0.0;

            foreach (Edge edge in path.Edges)
            {
                if (visited[ edge.GraphId ] == 0)
                {
                    quality += edge.Cost * edge.Profit;
                    visited[ edge.GraphId ]++;
                }
            }
            //path.Visited = visited.ToList();
            return quality;
        }

        public double GetProfit ( List<int> path )
        {
            HashSet<Edge> edgeSet = new HashSet<Edge>();

            double quality = 0.0;

            for (int i = 0; i < path.Count - 1; i++)
            {
                int current = path[ i ];
                int next = path[ i + 1 ];

                Edge edge = Graph.GetEdge( current, next );

                if (!edgeSet.Contains( edge ) && edge != null)
                {
                    quality += edge.Cost * edge.Profit;
                    _ = edgeSet.Add( edge );
                }
            }

            return quality;
        }

        public double GetArea ( Path path )
        {
            double area = 0.0;

            foreach (Edge edge in path.Edges)
            {
                area += !edge.Reversed ? edge.ShoelaceForward : edge.ShoelaceBackward;
            }

            return area / 2;
        }

        public double GetArea ( List<int> path )
        {
            double area = 0.0;

            int prev = -1;
            for (int i = 0; i < path.Count; i++)
            {
                int current = path[ i ];

                if (prev != -1)
                {
                    Edge edge = Graph.GetEdge( prev, current );

                    area += prev == edge.SourceNode.GraphNodeId ? edge.ShoelaceForward : edge.ShoelaceBackward;
                }

                prev = current;
            }

            if (prev != path.First())
            {
                Edge edge = Graph.GetEdge( prev, path.First() );
                area += prev == edge.SourceNode.GraphNodeId ? edge.ShoelaceForward : edge.ShoelaceBackward;
            }

            return area / 2;
        }

        public double GetLength ( List<int> path )
        {
            double length = 0.0;

            for (int i = 0; i < path.Count - 1; i++)
            {
                Edge edge = Graph.GetEdge( path[ i ], path[ i + 1 ] );

                length += edge.Cost;
            }

            return length;
        }
    }

}