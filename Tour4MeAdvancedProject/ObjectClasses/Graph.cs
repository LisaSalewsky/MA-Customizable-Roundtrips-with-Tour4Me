using Microsoft.SqlServer.Types;
using System;
using System.Collections.Generic;
using System.Configuration;
using System.Data;
using System.Data.SqlClient;
using System.Data.SqlTypes;
using System.Globalization;
using System.IO;
using System.Linq;
using Tour4MeAdvancedProject.Helper;

namespace Tour4MeAdvancedProject.ObjectClasses
{
    public class Graph
    {
        private readonly string dbConString = ConfigurationManager.ConnectionStrings[ "con" ].ConnectionString;
        private SqlConnection DBSqlConnection;
        private readonly SqlCommand DBSqlCommand;

        private Guid Id;

        public double MaxLat { get; set; }
        public double MinLat { get; set; }
        public double MaxLon { get; set; }
        public double MinLon { get; set; }
        public double CenterLat { get; set; }
        public double CenterLon { get; set; }

        public List<Node> VNodes { get; set; }
        public List<Edge> VEdges { get; set; }

        public Dictionary<Guid, int> GIdNode { get; set; }

        public Graph ( Guid InID, out string error )
        {
            error = "";
            Id = InID;
            VNodes = new List<Node>();
            VEdges = new List<Edge>();
            GIdNode = new Dictionary<Guid, int>();
            string connectError = "";
            try
            {
                ConnectToDB( out connectError );

                DBSqlCommand = new SqlCommand( "SELECT * from Graph WHERE Id='" + Id + "';", DBSqlConnection );
                SqlDataAdapter DBSqldataAdapter = new SqlDataAdapter( DBSqlCommand );
                SqlDataReader dataReader = DBSqlCommand.ExecuteReader();

                if (dataReader.HasRows)
                {
                    while (dataReader.Read())
                    {
                        object VNodesTemp = dataReader.GetValue( (int)GraphColumnsEnumHelper.Columns.VNodes );
                        object VEdgesTemp = dataReader.GetValue( (int)GraphColumnsEnumHelper.Columns.VEdges );
                        object GIdNodeTemp = dataReader.GetValue( (int)GraphColumnsEnumHelper.Columns.GraphIdNode );


                        //SqlBytes MaxLatLonBytes = dataReader.GetSqlBytes((int)GraphColumnsEnumHelper.Columns.MaxLatLon);
                        //string MaxLatLonString = BitConverter.ToString(MaxLatLonBytes.Buffer).Replace("-", "");
                        //SqlGeography MaxLatLon = SqlGeography.Parse(MaxLatLonString);
                        //SqlBytes MinLatLonBytes = dataReader.GetSqlBytes((int)GraphColumnsEnumHelper.Columns.MaxLatLon);
                        //string MinLatLonString = BitConverter.ToString(MinLatLonBytes.Buffer).Replace("-", "");
                        //SqlGeography MinLatLon = SqlGeography.Parse(MinLatLonString);
                        //SqlBytes CenterLatLonBytes = dataReader.GetSqlBytes((int)GraphColumnsEnumHelper.Columns.MaxLatLon);
                        //string CenterLatLonString = BitConverter.ToString(CenterLatLonBytes.Buffer).Replace("-", "");
                        //SqlGeography CenterLatLon = SqlGeography.Parse(CenterLatLonString);

                        SqlBytes MaxLatLonBytes = dataReader.GetSqlBytes( (int)GraphColumnsEnumHelper.Columns.MaxLatLon );
                        SqlGeography MaxLatLonTemp = SqlGeography.STGeomFromWKB( MaxLatLonBytes, 4326 );
                        SqlBytes MinLatLonBytes = dataReader.GetSqlBytes( (int)GraphColumnsEnumHelper.Columns.MaxLatLon );
                        SqlGeography MinLatLonTemp = SqlGeography.STGeomFromWKB( MinLatLonBytes, 4326 );
                        SqlBytes CenterLatLonBytes = dataReader.GetSqlBytes( (int)GraphColumnsEnumHelper.Columns.MaxLatLon );
                        SqlGeography CenterLatLonTemp = SqlGeography.STGeomFromWKB( CenterLatLonBytes, 4326 );
                    }
                }
                else
                {
                    error = "<script> alert('No Graph Data available! " +
                        "Please run script for creating and filling the database first';</script>)";
                }
            }
            catch (Exception ex)
            {
                error = "<script> alert('" + ex.ToString() + "';</script>)\");";
            }
            error += connectError;
        }

        private void ConnectToDB ( out string error )
        {
            error = "";
            try
            {
                DBSqlConnection = new SqlConnection( dbConString );
                if (DBSqlConnection.State == ConnectionState.Closed)
                {
                    DBSqlConnection.Open();
                }

            }
            catch (Exception ex)
            {
                error = "<script> alert('" + ex.ToString() + "';</script>)\");";
            }
        }


        public void AddEdge ( Edge edge )
        {
            if (VEdges == null)
            {
                VEdges = new List<Edge>();
            }
            VNodes[ edge.SourceNode.GraphNodeId ].Incident.Add( edge );
            VNodes[ edge.TargetNode.GraphNodeId ].Incident.Add( edge );
            VEdges.Add( edge );
        }
        public Graph ( string fileName )
        {
            using (StreamReader file = new StreamReader( fileName ))
            {
                VNodes = new List<Node>();
                VEdges = new List<Edge>();
                GIdNode = new Dictionary<Guid, int>();
                string str;
                int nNodes = 0;
                int nEdges = 0;
                int cNodes = 0;
                int cEdges = 0;

                while (( str = file.ReadLine() ) != null)
                {
                    char type = str[ 0 ];

                    if (type == 'g')
                    {
                        string t = NextWord( ref str, ' ' );
                        MaxLat = double.Parse( NextWord( ref str, ' ' ), CultureInfo.InvariantCulture );
                        MinLat = double.Parse( NextWord( ref str, ' ' ), CultureInfo.InvariantCulture );
                        MaxLon = double.Parse( NextWord( ref str, ' ' ), CultureInfo.InvariantCulture );
                        MinLon = double.Parse( NextWord( ref str, ' ' ), CultureInfo.InvariantCulture );
                        CenterLat = double.Parse( NextWord( ref str, ' ' ), CultureInfo.InvariantCulture );
                        CenterLon = double.Parse( NextWord( ref str, ' ' ), CultureInfo.InvariantCulture );
                    }
                    else if (type == 'p')
                    {
                        string t = NextWord( ref str, ' ' );
                        nNodes = int.Parse( NextWord( ref str, ' ' ) );
                        nEdges = int.Parse( NextWord( ref str, ' ' ) );

                        VNodes = new List<Node>( nNodes );
                    }
                    else if (type == 'n')
                    {
                        string t = NextWord( ref str, ' ' );
                        long intermediate = long.Parse( NextWord( ref str, ' ' ) );
                        byte[] longBytes = BitConverter.GetBytes( intermediate );
                        byte[] guidBytes = new byte[ 16 ];
                        Buffer.BlockCopy( longBytes, 0, guidBytes, 0, longBytes.Length );
                        Guid id = new Guid( guidBytes );

                        double lat = double.Parse( NextWord( ref str, ' ' ), CultureInfo.InvariantCulture );
                        double lon = double.Parse( NextWord( ref str, ' ' ), CultureInfo.InvariantCulture );

                        Node node = new Node( cNodes, id, lat, lon );
                        VNodes.Add( node );

                        GIdNode.Add( id, cNodes );

                        cNodes++;
                    }
                    else if (type == 'e')
                    {
                        string t = NextWord( ref str, ' ' );
                        long intermediateV = long.Parse( NextWord( ref str, ' ' ) );
                        byte[] longBytesV = BitConverter.GetBytes( intermediateV );
                        byte[] guidBytesV = new byte[ 16 ];
                        Buffer.BlockCopy( longBytesV, 0, guidBytesV, 0, longBytesV.Length );
                        Guid vId = new Guid( guidBytesV );
                        long intermediateW = long.Parse( NextWord( ref str, ' ' ) );
                        byte[] longBytesW = BitConverter.GetBytes( intermediateW );
                        byte[] guidBytesW = new byte[ 16 ];
                        Buffer.BlockCopy( longBytesW, 0, guidBytesW, 0, longBytesW.Length );
                        Guid wId = new Guid( guidBytesW );
                        double cost = double.Parse( NextWord( ref str, ' ' ), CultureInfo.InvariantCulture );

                        int sId = GIdNode[ vId ];
                        int tId = GIdNode[ wId ];

                        if (EdgeExists( sId, tId ))
                        {
                            _ = file.ReadLine();
                            _ = file.ReadLine();
                            continue;
                        }


                        Edge edge = AddEdge( cEdges, new Node( sId, vId, 0, 0 ), new Node( tId, wId, 0, 0 ), cost, new List<string>(), false, false );
                        //Edge edge = AddEdge(cEdges, sId, tId, cost);
                        cEdges++;

                        str = file.ReadLine();
                        if (str[ 0 ] != 'f')
                        {
                            throw new InvalidOperationException( "Error: graph file not in correct format; 'f' expected after 'e'" );
                        }

                        t = NextWord( ref str, ' ' );
                        while (HasWord( ref str, ' ' ))
                        {
                            double lat = double.Parse( NextWord( ref str, ' ' ), CultureInfo.InvariantCulture );
                            double lon = double.Parse( NextWord( ref str, ' ' ), CultureInfo.InvariantCulture );
                            edge.GeoLocations.Add( new Tuple<double, double>( lon, lat ) );
                        }

                        str = file.ReadLine();
                        if (str[ 0 ] != 'g')
                        {
                            throw new InvalidOperationException( "Error: graph file not in correct format; 'g' expected after 'f'" );
                        }

                        _ = NextWord( ref str, ' ' );
                        while (HasWord( ref str, ' ' ))
                        {
                            string word = NextWord( ref str, ' ' );
                            word = char.ToUpper( word[ 0 ] ) + word.Substring( 1 );
                            edge.Tags.Add( word );
                        }
                    }
                }

                //if (cNodes != nNodes)
                //{
                //    throw new InvalidOperationException($"Error: number of nodes ({cNodes}) does not match the file preamble ({nNodes})");
                //}

                //if (cEdges != nEdges)
                //{
                //    throw new InvalidOperationException($"Error: number of edges ({cEdges}) does not match the file preamble ({nEdges})");
                //}
            }
        }

        public Graph ( double startLat, double startLon, double radius, string fileName )
        {
            SqlServerTypes.Utilities.LoadNativeAssemblies( AppDomain.CurrentDomain.BaseDirectory );
            ConnectToDB( out string error );
            if (error != null && error != "")
            {
                Console.WriteLine( error );
                _ = new Graph( fileName );
            }
            else
            {
                CenterLat = startLat;
                CenterLon = startLon;

                VNodes = new List<Node>();
                VEdges = new List<Edge>();
                GIdNode = new Dictionary<Guid, int>();
                int cNodes = 0;
                int cEdges = 0;
                //_ = CreateNodesFromDBMaxMinLatLon( givenMaxLat, givenMaxLon, givenMinLat, givenMinLon, cNodes, out _, out _ );
                //CreateEdgesFromDBMaxMinLatLon( givenMaxLat, givenMaxLon, givenMinLat, givenMinLon, cEdges, out _, out _ );


                CreateNodesAndEdgesFromDBCenterRadius( startLat, startLon, radius, cNodes, cEdges, VNodes, GIdNode );

                RemoveDeadEnds();
            }
        }

        private void CreateNodesAndEdgesFromDBCenterRadius ( double startLat, double startLon, double radius, int cNodes, int cEdges, List<Node> vNodes, Dictionary<Guid, int> gIdNodes )
        {
            // Define SQL query strings
            string nodesQuery = @"DECLARE @CenterPoint geography;" +
            "SET @CenterPoint = geography::Point(@CenterLat, @CenterLon, 4326);" +

            "DECLARE @RadiusInMeters float = @Radius;" +
            "SELECT Id," +
            "GeographyValues.Lat AS Latitude," +
            "GeographyValues.Long AS Longitude," +
            "Elevation," +
            "Surroundings " +
            "FROM dbo.Node " +
            "WHERE GeographyValues.STDistance(@CenterPoint) <= @RadiusInMeters;";

            string edgesQuery = @"DECLARE @CenterPoint geography;" +
            "SET @CenterPoint = geography::Point(@CenterLat, @CenterLon, 4326);" +

            "DECLARE @RadiusInMeters float = @Radius;" +

            "SELECT e.Id," +
                "Tags," +
                "Reversed," +
                "OneWay," +
                "Cost," +
                "SourceNodeId," +
                "TargetNodeId," +
                "GeoLocations.STAsText() AS GeoLocationsText," +
                "GeoLocations " +
            "FROM dbo.Edge e " +
            "INNER JOIN dbo.IncidentEdges ie ON e.Id = ie.edgeId " +
            "INNER JOIN dbo.Node n ON ie.NodeId = n.Id " +
            "WHERE n.GeographyValues.STDistance( @CenterPoint ) <= @RadiusInMeters; ";

            // Define parameters
            _ = new SqlParameter( "@CenterLat", SqlDbType.Float )
            {
                Value = CenterLat
            };
            _ = new SqlParameter( "@CenterLon", SqlDbType.Float )
            {
                Value = CenterLon
            };
            _ = new SqlParameter( "@Radius", SqlDbType.Float )
            {
                Value = (float)radius
            };

            // Execute nodes query
            SqlCommand nodesCommand = new SqlCommand( nodesQuery, DBSqlConnection );
            _ = nodesCommand.Parameters.AddWithValue( "@CenterLat", startLat );
            _ = nodesCommand.Parameters.AddWithValue( "@CenterLon", startLon );
            _ = nodesCommand.Parameters.AddWithValue( "@Radius", radius );

            // Execute edges query
            SqlCommand edgesCommand = new SqlCommand( edgesQuery, DBSqlConnection );
            _ = edgesCommand.Parameters.AddWithValue( "@CenterLat", startLat );
            _ = edgesCommand.Parameters.AddWithValue( "@CenterLon", startLon );
            _ = edgesCommand.Parameters.AddWithValue( "@Radius", radius );


            _ = CreateNodesFromReader( cNodes, nodesCommand, vNodes, gIdNodes );
            CreateEdgesFromReader( cEdges, edgesCommand );
        }

        private void CreateEdgesFromDBMaxMinLatLon ( double givenMaxLat, double givenMaxLon, double givenMinLat, double givenMinLon, int cEdges, out string sqlQuery, out SqlCommand command )
        {

            // Construct and execute SQL query to fetch edges within range
            sqlQuery = @"SELECT e.EdgeId, e.SourceNodeId, e.TargetNodeId" +
                "FROM Edges e" +
                "JOIN Nodes source ON e.SourceNodeId = source.NodeId" +
                "JOIN Nodes target ON e.TargetNodeId = target.NodeId" +
                "WHERE source.Latitude BETWEEN @MinLat AND @MaxLat" +
                "AND source.Longitude BETWEEN @MinLon AND @MaxLon" +
                "AND target.Latitude BETWEEN @MinLat AND @MaxLat" +
                "AND target.Longitude BETWEEN @MinLon AND @MaxLon;";
            command = new SqlCommand( sqlQuery, DBSqlConnection );

            _ = command.Parameters.AddWithValue( "@MinLat", givenMinLat );
            _ = command.Parameters.AddWithValue( "@MaxLat", givenMaxLat );
            _ = command.Parameters.AddWithValue( "@MinLon", givenMinLon );
            _ = command.Parameters.AddWithValue( "@MaxLon", givenMaxLon );

            CreateEdgesFromReader( cEdges, command );
        }

        private void CreateEdgesFromReader ( int cEdges, SqlCommand command )
        {
            using (SqlDataReader reader = command.ExecuteReader())
            {
                //_ = Parallel.ForEach( ParallelizingIterator.IterateUntilFalse( () => reader.Read() ), z =>
                while (reader.Read())
                {
                    _ = reader.GetGuid( reader.GetOrdinal( "Id" ) );
                    _ = reader.GetGuid( 0 );
                    Guid sourceId = reader.GetGuid( reader.GetOrdinal( "SourceNodeId" ) );
                    Guid targetId = reader.GetGuid( reader.GetOrdinal( "TargetNodeId" ) );
                    int idxTags = reader.GetOrdinal( "Tags" );
                    List<string> tags = new List<string>();
                    if (!reader.IsDBNull( idxTags ))
                    {
                        tags = reader.GetString( idxTags )?.Split( ',' ).ToList();
                    }
                    bool reversed = reader.GetBoolean( reader.GetOrdinal( "Reversed" ) );
                    bool oneWay = reader.GetBoolean( reader.GetOrdinal( "OneWay" ) );
                    double cost = (double)reader.GetDecimal( reader.GetOrdinal( "Cost" ) );

                    if (!GIdNode.ContainsKey( sourceId ) || !GIdNode.ContainsKey( targetId ))
                    {
                        continue;
                    }
                    int sId = GIdNode[ sourceId ];
                    int tId = GIdNode[ targetId ];

                    if (EdgeExists( sId, tId ))
                    {
                        //return;
                        continue;
                    }

                    Node source = VNodes[ sId ];
                    Node target = VNodes[ tId ];



                    Edge edge = AddEdge( cEdges, source, target, cost, tags, reversed, oneWay );
                    cEdges++;
                    //edge = new Edge( edge );

                    object geoLocsVal = reader[ "GeoLocationsText" ];

                    string geoLocationsText = geoLocsVal != null ? geoLocsVal.ToString() : "";
                    if (!string.IsNullOrEmpty( geoLocationsText ))
                    {
                        try
                        {
                            //SqlGeography geometry = SqlGeography.STLineFromText( new SqlChars( geoLocationsText ), 4326 );

                            SqlGeography lineString = SqlGeography.STLineFromText( new SqlChars( geoLocationsText ), 4326 );
                            for (int i = 1; i <= lineString.STNumPoints(); i++)
                            {
                                SqlGeography point = lineString.STPointN( i );
                                double latitude = point.Lat.Value;
                                double longitude = point.Long.Value;

                                edge.GeoLocations.Add( new Tuple<double, double>( longitude, latitude ) );
                            }
                        }
                        catch (Exception ex)
                        {
                            string msg = ex.Message;
                            Console.WriteLine( msg );
                        }
                    }

                }
                //);
            }
        }

        private int CreateNodesFromDBMaxMinLatLon ( double givenMaxLat, double givenMaxLon, double givenMinLat, double givenMinLon, int cNodes, List<Node> vNodes, Dictionary<Guid, int> gIdNodes, out string sqlQuery, out SqlCommand command )
        {
            sqlQuery = @"SELECT * FROM Nodes WHERE" +
                "SpatialLocation.STLatitude() BETWEEN @MinLat AND @MaxLat " +
                "AND SpatialLocation.STLongitude() BETWEEN @MinLon AND @MaxLon";
            command = new SqlCommand( sqlQuery, DBSqlConnection );
            _ = command.Parameters.AddWithValue( "@MinLat", givenMinLat );
            _ = command.Parameters.AddWithValue( "@MaxLat", givenMaxLat );
            _ = command.Parameters.AddWithValue( "@MinLon", givenMinLon );
            _ = command.Parameters.AddWithValue( "@MaxLon", givenMaxLon );

            cNodes = CreateNodesFromReader( cNodes, command, vNodes, gIdNodes );

            return cNodes;
        }

        private static int CreateNodesFromReader ( int cNodes, SqlCommand command, List<Node> vNodes, Dictionary<Guid, int> gIdNodes )
        {

            using (SqlDataReader reader = command.ExecuteReader())
            {
                while (reader.Read())
                {
                    Guid nodeId = reader.GetGuid( reader.GetOrdinal( "Id" ) );
                    double elevation = (double)reader.GetDecimal( reader.GetOrdinal( "Elevation" ) );
                    double latitude = reader.GetDouble( reader.GetOrdinal( "Latitude" ) );
                    double longitude = reader.GetDouble( reader.GetOrdinal( "Longitude" ) );

                    int idxSurroundings = reader.GetOrdinal( "Surroundings" );
                    List<string> tags = new List<string>();
                    if (!reader.IsDBNull( idxSurroundings ))
                    {
                        tags = reader.GetString( idxSurroundings )?.Split( ',' ).ToList();
                    }


                    Node node = new Node( cNodes, nodeId, latitude, longitude, elevation );
                    vNodes.Add( node );
                    gIdNodes.Add( nodeId, cNodes );

                    cNodes++;
                }
            }

            return cNodes;
        }

        public Edge AddEdge ( int edgeGraphId, Node s, Node t, double cost, List<string> tags, bool reversed, bool oneWay )
        {
            if (s.GraphNodeId > t.GraphNodeId)
            {
                (t, s) = (s, t);
            }


            Node l = VNodes[ s.GraphNodeId ];
            Node r = VNodes[ t.GraphNodeId ];

            HashSet<string> surroundings = l.Suroundings.Concat( r.Suroundings ).ToHashSet();
            if (surroundings.Count > 0)
            {
                tags = tags.Concat( surroundings ).ToList();
            }


            Edge edge = new Edge( edgeGraphId, new Tuple<Node, Node>( l, r ), cost );

            //double yl = GetDistanceFromLatLon( l.Lat, l.Lon, MinLat, l.Lon ) * ( l.Lat < MinLat ? -1 : 1 );
            //double xl = GetDistanceFromLatLon( l.Lat, l.Lon, l.Lat, MinLon ) * ( l.Lon < MinLon ? -1 : 1 );

            //double yr = GetDistanceFromLatLon( r.Lat, r.Lon, MinLat, r.Lon ) * ( r.Lat < MinLat ? -1 : 1 );
            //double xr = GetDistanceFromLatLon( r.Lat, r.Lon, r.Lat, MinLon ) * ( r.Lon < MinLon ? -1 : 1 );

            double yl = GetDistanceFromLatLon( l.Lat, l.Lon, CenterLat, l.Lon ) * ( l.Lat < CenterLat ? -1 : 1 );
            double xl = GetDistanceFromLatLon( l.Lat, l.Lon, l.Lat, CenterLon ) * ( l.Lon < CenterLon ? -1 : 1 );

            double yr = GetDistanceFromLatLon( r.Lat, r.Lon, CenterLat, r.Lon ) * ( r.Lat < CenterLat ? -1 : 1 );
            double xr = GetDistanceFromLatLon( r.Lat, r.Lon, r.Lat, CenterLon ) * ( r.Lon < CenterLon ? -1 : 1 );

            edge.ShoelaceForward = ( yl + yr ) * ( xl - xr );
            edge.ShoelaceBackward = ( yr + yl ) * ( xr - xl );

            edge.Tags = tags;
            edge.Reversed = edge.GeoLocations != null &&
                edge.GeoLocations.Count > 0 &&
                ( edge.GeoLocations.First().Item1 != edge.SourceNode.Lat ||
                edge.GeoLocations.First().Item2 != edge.SourceNode.Lon )
                ;
            edge.OneWay = oneWay;

            AddEdge( edge );
            return edge;
        }

        public Edge GetEdge ( int sId, int tId )
        {
            foreach (Edge e in VNodes[ sId ].Incident)
            {
                if (( tId == e.SourceNode.GraphNodeId && sId == e.TargetNode.GraphNodeId ) || ( tId == e.TargetNode.GraphNodeId && sId == e.SourceNode.GraphNodeId ))
                {
                    return e;
                }
            }

            return null;
        }

        public bool EdgeExists ( int sId, int tId )
        {
            foreach (Edge e in VNodes[ sId ].Incident)
            {
                if (tId == e.SourceNode.GraphNodeId || tId == e.TargetNode.GraphNodeId)
                {
                    return true;
                }
            }

            return false;
        }

        public double Length ( List<int> path )
        {
            double length = 0.0;

            for (int i = 0; i < path.Count - 1; i++)
            {
                Edge edge = GetEdge( path[ i ], path[ i + 1 ] );
                length += edge.Cost;
            }

            return length;
        }

        public List<Tuple<int, Path>> CalculateRing ( int sourceNode, double innerDistance, double outerDistance,
                                                    int nodeLimit, HashSet<int> contained )
        {
            double[] dist = new double[ VNodes.Count ];
            for (int i = 0; i < dist.Length; i++)
            {
                dist[ i ] = double.MaxValue;
            }

            double[] actDist = new double[ VNodes.Count ];

            PriorityQueue<Tuple<int, double>> queue = new PriorityQueue<Tuple<int, double>>();
            Tuple<int, Edge>[] parent = new Tuple<int, Edge>[ VNodes.Count ];
            dist[ sourceNode ] = 0.0;
            actDist[ sourceNode ] = 0.0;

            queue.Enqueue( 0.0, new Tuple<int, double>( sourceNode, 0.0 ) );

            List<int> visited = new List<int>();

            while (queue.Count > 0)
            {
                (double currentDist, (int currentNode, double currentActual)) = queue.Dequeue();

                if (currentActual > outerDistance)
                {
                    continue;
                }

                double bestKnownDist = dist[ currentNode ];

                if (bestKnownDist == double.MaxValue)
                {
                    dist[ currentNode ] = currentDist;
                    actDist[ currentNode ] = currentActual;
                    bestKnownDist = currentDist;
                }

                if (bestKnownDist != currentDist)
                {
                    continue;
                }


                foreach (Edge edge in VNodes[ currentNode ].Incident)
                {
                    int neighborId = edge.SourceNode.GraphNodeId == currentNode ? edge.TargetNode.GraphNodeId : edge.SourceNode.GraphNodeId;

                    double newDist = bestKnownDist + ( edge.Cost / ( edge.Profit + 0.1 ) );
                    double newActual = currentActual + edge.Cost;

                    if (newDist < dist[ neighborId ])
                    {
                        queue.Enqueue( newDist, new Tuple<int, double>( neighborId, newActual ) );
                        dist[ neighborId ] = newDist;
                        actDist[ neighborId ] = newActual;
                        parent[ neighborId ] = Tuple.Create( currentNode, edge );

                        visited.Add( neighborId );
                    }
                }
            }

            List<Tuple<int, Path>> output = new List<Tuple<int, Path>>();
            int outputSize = 0;

            foreach (int vis in visited)
            {
                if (contained != null && !contained.Contains( vis ))
                {
                    continue;
                }

                if (outputSize > nodeLimit)
                {
                    break;
                }

                if (actDist[ vis ] < innerDistance)
                {
                    continue;
                }

                int current = vis;
                Path path = new Path( Tuple.Create( CenterLat, CenterLon ) );

                bool aborted = false;

                while (current != sourceNode)
                {
                    Edge e = parent[ current ].Item2;
                    path.Edges.Insert( 0, new Edge( e, e.TargetNode.GraphNodeId == current ) );
                    path.Length += e.Cost;

                    // if any of the null checks holds, current was not deeply enough routed
                    if (parent[ current ] != null && // current has no parent (is the start)
                        parent[ parent[ current ].Item1 ] != null)// current has not 2 parent nodes, so it's the first child only
                    {
                        if (current == parent[ parent[ current ].Item1 ].Item1)
                        //if (current == parent[parent[current].Item1].Item1)
                        {
                            aborted = true;
                            break;
                        }

                        if (parent[ parent[ parent[ current ].Item1 ].Item1 ] != null && // current has not 3 parent nodes
                            current == parent[ parent[ parent[ current ].Item1 ].Item1 ].Item1)
                        //if (current == parent[parent[parent[current].Item1].Item1].Item1)
                        {
                            aborted = true;
                            break;
                        }
                    }
                    current = parent[ current ].Item1;
                }

                if (!aborted)
                {
                    output.Add( Tuple.Create( vis, path ) );
                    outputSize++;
                }
            }

            return output;
        }

        private static string NextWord ( ref string line, char delimiter )
        {
            if (line.Length == 0)
            {
                throw new InvalidOperationException( "Error: target line is empty" );
            }

            int pos = line.IndexOf( delimiter );
            string word;

            if (pos != -1)
            {
                word = line.Substring( 0, pos );
                line = line.Remove( 0, pos + 1 );
            }
            else
            {
                word = line;
                line = string.Empty;
            }

            return word;
        }

        private static bool HasWord ( ref string line, char delimiter )
        {
            return line.Length != 0;
        }

        public double GetDistanceFromLatLon ( int node1Id, int node2Id )
        {
            const double R = 6371; // Radius of the earth in km
            Node node1 = VNodes[ node1Id ];
            Node node2 = VNodes[ node2Id ];
            double dLat = Deg2Rad( node2.Lat - node1.Lat );  // deg2rad below
            double dLon = Deg2Rad( node2.Lon - node1.Lon );
            double a =
              ( Math.Sin( dLat / 2 ) * Math.Sin( dLat / 2 ) ) +
              ( Math.Cos( Deg2Rad( node1.Lat ) ) * Math.Cos( Deg2Rad( node2.Lat ) ) *
              Math.Sin( dLon / 2 ) * Math.Sin( dLon / 2 ) )
              ;
            double c = 2 * Math.Atan2( Math.Sqrt( a ), Math.Sqrt( 1 - a ) );
            double distance = R * c; // Distance in km
            return distance;
        }

        public static double GetDistanceFromLatLon ( double node1Lat, double node1Lon, double node2Lat, double node2Lon )
        {
            const double R = 6371000; // Radius of the earth in km
            double dLat = Deg2Rad( node2Lat - node1Lat );  // deg2rad below
            double dLon = Deg2Rad( node2Lon - node1Lon );
            double a =
              ( Math.Sin( dLat / 2 ) * Math.Sin( dLat / 2 ) ) +
              ( Math.Cos( Deg2Rad( node1Lat ) ) * Math.Cos( Deg2Rad( node2Lat ) ) *
              Math.Sin( dLon / 2 ) * Math.Sin( dLon / 2 ) )
              ;
            double c = 2 * Math.Atan2( Math.Sqrt( a ), Math.Sqrt( 1 - a ) );
            double distance = R * c; // Distance in km
            return distance;
        }

        private static double Deg2Rad ( double deg )
        {
            return deg * ( Math.PI / 180 );
        }

        private int MinDistance ( double[] dist, bool[] sptSet )
        {
            // Initialize min value
            double min = double.MaxValue;
            int min_index = -1;

            for (int v = 0; v < VNodes.Count; v++)
            {
                if (sptSet[ v ] == false && dist[ v ] <= min)
                {
                    min = dist[ v ];
                    min_index = v;
                }
            }

            return min_index;
        }
        public void CalculateShortestDistances ( int start )
        {
            double[] dist = new double[ VNodes.Count ];
            double[] queue = new double[ VNodes.Count ];
            bool[] sptSet = new bool[ VNodes.Count ];

            for (int i = 0; i < VNodes.Count; i++)
            {
                dist[ i ] = int.MaxValue;
                sptSet[ i ] = false;
            }


            dist[ start ] = 0.0;
            VNodes[ start ].ShortestDistance = 0.0;
            queue[ start ] = 0.0;

            for (int count = 0; count < VNodes.Count - 1; count++)
            {
                int u = MinDistance( dist, sptSet );
                sptSet[ u ] = true;
                for (int v = 0; v < VNodes.Count; v++)
                {
                    if (!sptSet[ v ] && dist[ u ] != int.MaxValue)
                    {
                        Node cur = VNodes[ u ];
                        Node neighbor = VNodes[ v ];

                        double potShortst = cur.ShortestDistance + dist[ v ];
                        if (cur.Incident
                            .Where( e => e.SourceNode.GraphNodeId == v || e.TargetNode.GraphNodeId == v ).Count() > 0
                            && neighbor.ShortestDistance > potShortst)
                        {
                            neighbor.ShortestDistance = potShortst;
                        }
                    }
                }
            }

        }
        public void InitializeShortestPath ( int start )
        {
            double[] dist = new double[ VNodes.Count ];
            PriorityQueue<Tuple<int, double>> queue = new PriorityQueue<Tuple<int, double>>();


            dist[ start ] = 0.0;
            queue.Enqueue( 0.0, new Tuple<int, double>( start, 0.0 ) );

            while (queue.Count > 0)
            {
                (double, Tuple<int, double>) current = queue.Dequeue();
                _ = current.Item1;
                int currentNode = current.Item2.Item1;
                double actual = current.Item2.Item2;

                double bestKnownDist = dist[ currentNode ];
                if (bestKnownDist == 0.0 && bestKnownDist == double.MaxValue)
                {
                    dist[ currentNode ] = actual;
                    bestKnownDist = actual;
                }

                if (bestKnownDist != actual)
                {
                    continue;
                }

                List<Edge> currentIncident = VNodes[ currentNode ].Incident;
                foreach (Edge edge in currentIncident)
                {
                    Node neighbor = edge.SourceNode.GraphNodeId == currentNode ? edge.TargetNode : edge.SourceNode;
                    int neighborId = neighbor.GraphNodeId;

                    double newDistance = bestKnownDist + edge.Cost;
                    double currentNeighborDist = dist[ neighborId ];
                    if (currentNeighborDist == 0.0)
                    {
                        dist[ neighborId ] = double.MaxValue;
                        currentNeighborDist = double.MaxValue;
                    }

                    if (newDistance < currentNeighborDist)
                    {
                        double heuristic = newDistance + GetDistanceFromLatLon( start, neighborId );
                        queue.Enqueue( heuristic, new Tuple<int, double>( neighborId, newDistance ) );
                        dist[ neighborId ] = newDistance;
                        neighbor.ShortestDistance = newDistance;
                    }
                }
            }
        }


        public double ShortestPath ( int start, int end )
        {
            double[] dist = new double[ VNodes.Count ];
            PriorityQueue<Tuple<int, double>> queue = new PriorityQueue<Tuple<int, double>>();


            dist[ start ] = 0.0;
            queue.Enqueue( 0.0, new Tuple<int, double>( start, 0.0 ) );

            while (queue.Count > 0)
            {
                (double, Tuple<int, double>) current = queue.Dequeue();
                _ = current.Item1;
                int currentNode = current.Item2.Item1;
                double actual = current.Item2.Item2;

                if (currentNode == end)
                {
                    return dist[ currentNode ];
                }

                double bestKnownDist = dist[ currentNode ];
                if (bestKnownDist == 0.0 && bestKnownDist == double.MaxValue)
                {
                    dist[ currentNode ] = actual;
                    bestKnownDist = actual;
                }

                if (bestKnownDist != actual)
                {
                    continue;
                }

                List<Edge> currentIncident = VNodes[ currentNode ].Incident;
                foreach (Edge edge in currentIncident)
                {
                    int neighborId = edge.SourceNode.GraphNodeId == currentNode ? edge.TargetNode.GraphNodeId : edge.SourceNode.GraphNodeId;

                    double newDistance = bestKnownDist + edge.Cost;

                    double currentNeighborDist = dist[ neighborId ];
                    if (currentNeighborDist == 0.0)
                    {
                        dist[ neighborId ] = double.MaxValue;
                        currentNeighborDist = double.MaxValue;
                    }

                    if (newDistance < currentNeighborDist)
                    {
                        double heuristic = newDistance + GetDistanceFromLatLon( neighborId, end );
                        queue.Enqueue( heuristic, new Tuple<int, double>( neighborId, newDistance ) );
                        dist[ neighborId ] = newDistance;
                    }
                }
            }
            return double.MaxValue;
        }

        public (List<Edge>, List<int>) GetShortestPath ( int start, int end )
        {
            Dictionary<int, double> dist = new Dictionary<int, double>();
            PriorityQueue<Tuple<int, double>> queue = new PriorityQueue<Tuple<int, double>>();
            Dictionary<int, Edge> parentEdges = new Dictionary<int, Edge>();


            dist[ start ] = 0.0;
            queue.Enqueue( 0.0, new Tuple<int, double>( start, 0.0 ) );

            while (queue.Count > 0)
            {
                (double, Tuple<int, double>) current = queue.Dequeue();
                _ = current.Item1;
                int currentNode = current.Item2.Item1;
                double actual = current.Item2.Item2;

                if (currentNode == end)
                {
                    List<Edge> path = new List<Edge>();
                    List<int> pathNodes = new List<int>();
                    int node = end;
                    // walk through parent edges and build path
                    while (node != start)
                    {
                        Edge parentEdge = parentEdges[ node ];
                        path.Insert( 0, parentEdge );
                        pathNodes.Insert( 0, node );
                        node = parentEdge.SourceNode.GraphNodeId == node ? parentEdge.TargetNode.GraphNodeId : parentEdge.SourceNode.GraphNodeId;
                    }
                    return (path, pathNodes);
                }

                if (!dist.TryGetValue( currentNode, out double bestKnownDist ))
                {
                    dist[ currentNode ] = actual;
                    bestKnownDist = actual;
                }

                if (bestKnownDist != actual)
                {
                    continue;
                }

                List<Edge> incident = VNodes[ currentNode ]?.Incident;

                if (incident != null && incident.Count > 0)
                {
                    foreach (Edge edge in incident)
                    {
                        int neighborId = edge.SourceNode.GraphNodeId == currentNode ? edge.TargetNode.GraphNodeId : edge.SourceNode.GraphNodeId;

                        double newDistance = bestKnownDist + edge.Cost;

                        if (!dist.TryGetValue( neighborId, out double currentNeighborDist ))
                        {
                            dist[ neighborId ] = double.MaxValue;
                            currentNeighborDist = double.MaxValue;
                        }

                        if (newDistance < currentNeighborDist)
                        {
                            double heuristic = newDistance + GetDistanceFromLatLon( neighborId, end );
                            queue.Enqueue( heuristic, new Tuple<int, double>( neighborId, newDistance ) );
                            dist[ neighborId ] = newDistance;
                            parentEdges[ neighborId ] = edge;
                        }
                    }
                }
            }
            return (new List<Edge>(), new List<int>());
        }
        public (List<Edge>, List<int>) DijkstraShortestPath ( int start, int end )
        {
            List<Edge> path = new List<Edge>();
            List<int> pathNodes = new List<int>();

            int currentNode = end;
            do
            {
                Node currentNodeObj = VNodes[ currentNode ];
                double shortestDist = currentNodeObj.ShortestDistance;
                int prevNode = -1;
                foreach (Edge edge in currentNodeObj.Incident)
                {
                    int neighborId = edge.SourceNode.GraphNodeId == currentNode ? edge.TargetNode.GraphNodeId : edge.SourceNode.GraphNodeId;
                    if (shortestDist == VNodes[ neighborId ].ShortestDistance + edge.Cost)
                    {
                        prevNode = neighborId;
                        path.Insert( 0, edge );
                        break;
                    }
                }
                if (prevNode == -1)
                {
                    // No valid previous node found
                    break;
                }
                pathNodes.Insert( 0, currentNode );
                currentNode = prevNode;
            }
            while (currentNode != start);
            return (path, pathNodes);
        }

        public void RemoveDeadEnds ()
        {
            HashSet<Node> deadEndNodes = new HashSet<Node>();
            HashSet<Edge> deadEndEdges = new HashSet<Edge>();
            foreach (Edge edge in VEdges)
            {
                bool nodeOrNeighborDeleted = false;
                Node node = edge.SourceNode;
                Node neighbor = edge.TargetNode;

                // an edge is a connection from junction to junction
                // so if you count all edges from a certain junction
                // and it is exactly 1 you have found a dead end
                if (node.Incident.Count() == 1)
                {
                    _ = deadEndNodes.Add( node );
                    nodeOrNeighborDeleted = true;
                }
                if (neighbor.Incident.Count() == 1)
                {
                    _ = deadEndNodes.Add( neighbor );
                    nodeOrNeighborDeleted = true;

                }
                if (nodeOrNeighborDeleted)
                {
                    _ = deadEndEdges.Add( edge );
                }
            }

            foreach (Node node in deadEndNodes)
            {
                if (node.Incident.Count == 1)
                {
                    Edge incident = node.Incident[ 0 ];
                    Node neighbor = incident.SourceNode.GraphNodeId == node.GraphNodeId ? incident.TargetNode : incident.SourceNode;
                    _ = neighbor.Incident.Remove( incident );
                    _ = VNodes[ node.GraphNodeId ] = null;

                }
            }
            foreach (Edge currentEdge in deadEndEdges)
            {
                _ = VEdges[ currentEdge.GraphId ] = null;

            }
        }

    }

}