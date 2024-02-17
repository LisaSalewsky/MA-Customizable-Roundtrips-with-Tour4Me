using Microsoft.SqlServer.Types;
using System;
using System.Collections.Generic;
using System.Configuration;
using System.Data;
using System.Data.SqlClient;
using System.Globalization;
using System.IO;
using Tour4MeAdvancedProject.ObjectClasses;

namespace Tour4MeAdvancedProject.Helper
{
    public static class ImportGraphDataToSQL
    {
        private static readonly string connectionString = ConfigurationManager.ConnectionStrings[ "con" ].ConnectionString;
        private static readonly SqlConnection connection = new SqlConnection( connectionString );
        private static readonly StreamReader file = new StreamReader( "your_data_file.txt" );
        private static readonly Dictionary<Guid, Node> GId = new Dictionary<Guid, Node>();
        private static Guid GraphId;
        private static double maxLat = 0;
        private static double minLat = 0;
        private static double maxLon = 0;
        private static double minLon = 0;
        private static double centerLat = 0;
        private static double centerLon = 0;
        private static List<Node> VNodes;
        private static readonly List<Edge> VEdges;


        public static void ReadDataFile ()
        {
            int cNodes = 0;
            string line;
            while (( line = file.ReadLine() ) != null)
            {
                string[] lineData = line.Split();
                string dataType = lineData[ 0 ];
                if (dataType == "g")
                {
                    maxLat = double.Parse( lineData[ 1 ], CultureInfo.InvariantCulture );
                    minLat = double.Parse( lineData[ 2 ], CultureInfo.InvariantCulture );
                    maxLon = double.Parse( lineData[ 3 ], CultureInfo.InvariantCulture );
                    minLon = double.Parse( lineData[ 4 ], CultureInfo.InvariantCulture );
                    centerLat = double.Parse( lineData[ 5 ], CultureInfo.InvariantCulture );
                    centerLon = double.Parse( lineData[ 6 ], CultureInfo.InvariantCulture );
                    GraphId = Guid.NewGuid();

                    SqlGeography maxLatLon = SqlGeography.Point( maxLat, maxLon, 4326 );
                    SqlGeography minLatLon = SqlGeography.Point( minLat, minLon, 4326 );
                    SqlGeography centerLatLon = SqlGeography.Point( centerLat, centerLon, 4326 );

                    InsertGraph( GraphId, maxLatLon, minLatLon, centerLatLon );
                }
                else if (dataType == "p")
                {
                    int nNodes = int.Parse( lineData[ 1 ] );

                    VNodes = new List<Node>( nNodes );
                }
                else if (dataType == "n")
                {
                    Guid id = Guid.Parse( lineData[ 1 ] );
                    double lat = double.Parse( lineData[ 2 ], CultureInfo.InvariantCulture );
                    double lon = double.Parse( lineData[ 3 ], CultureInfo.InvariantCulture );
                    SqlGeography geography = SqlGeography.Point( lat, lon, 4326 );
                    InsertNode( id, geography );

                    Node node = new Node( cNodes, id, lat, lon );

                    GId.Add( id, node );

                    VNodes.Add( node );
                    cNodes++;
                }
                else if (dataType == "e")
                {
                    string vId = lineData[ 1 ];
                    string wId = lineData[ 2 ];
                    double eCost = double.Parse( lineData[ 3 ], CultureInfo.InvariantCulture );

                    Guid sourceNode = Guid.Parse( vId );
                    Guid targetNode = Guid.Parse( wId );

                    Guid? currentEdgeId = GenerateGuidFromGuids( sourceNode, targetNode );


                    if (EdgeExists( currentEdgeId.Value ))
                    {
                        _ = file.ReadLine();
                        _ = file.ReadLine();
                        continue;
                    }

                    InsertEdge( currentEdgeId.Value, sourceNode, targetNode, eCost );
                    AddIncidentEdgeRelation( Guid.Parse( vId ), currentEdgeId.Value );

                    lineData = file.ReadLine().Split();
                    dataType = lineData[ 0 ];
                    if (dataType != "f")
                    {
                        throw new InvalidOperationException( "Error: graph file not in correct format; 'f' expected after 'e'" );
                    }

                    double[] nodes = new double[ lineData.Length - 1 ];
                    for (int i = 1; i < lineData.Length; i += 2)
                    {

                        nodes[ i ] = double.Parse( lineData[ i ], CultureInfo.InvariantCulture );
                        nodes[ i + 1 ] = double.Parse( lineData[ i + 1 ], CultureInfo.InvariantCulture );


                        _ = lineData[ 1 ];  // Assuming v_id is available in the previous 'e' line
                        InsertIntermediateNodes( currentEdgeId.Value, nodes );
                    }

                    lineData = file.ReadLine().Split();
                    dataType = lineData[ 0 ];
                    if (dataType != "g")
                    {
                        throw new InvalidOperationException( "Error: graph file not in correct format; 'g' expected after 'f'" );
                    }

                    string tags = "";
                    for (int i = 1; i < lineData.Length; i++)
                    {
                        string word = lineData[ i ];
                        tags += "," + char.ToUpper( word[ 0 ] ) + word.Substring( 1 );
                    }
                    AssociateTagsWithEdge( currentEdgeId.Value, tags );
                }
            }

            foreach (Node node in VNodes)
            {
                AddNodeIntoGraph( node.NodeId, GraphId );
            }

            foreach (Edge edge in VEdges)
            {

                AddEdgeIntoGraph( edge.Id, GraphId );
            }

        }

        private static void InsertGraph ( Guid graphId, SqlGeography maxLatLon, SqlGeography minLatLon, SqlGeography centerLatLon )
        {

            try
            {
                connection.Open();

                string query = $"INSERT INTO Graph (Id, MaxLatLon, MinLatLon, CenterLatLon) VALUES ('{graphId}', @maxLatLon, @minLatLon, @centerLatLon)";
                using (SqlCommand command = new SqlCommand( query, connection ))
                {
                    _ = command.Parameters.Add( new SqlParameter( "@maxLatLon", System.Data.SqlDbType.Udt ) { UdtTypeName = "maxLatLon", Value = maxLatLon } );
                    _ = command.Parameters.Add( new SqlParameter( "@minLatLon", System.Data.SqlDbType.Udt ) { UdtTypeName = "minLatLon", Value = minLatLon } );
                    _ = command.Parameters.Add( new SqlParameter( "@centerLatLon", System.Data.SqlDbType.Udt ) { UdtTypeName = "centerLatLon", Value = centerLatLon } );

                    _ = command.ExecuteNonQuery();
                }
            }
            finally
            {
                connection.Close();
            }
        }

        private static Guid GenerateGuidFromGuids ( Guid guid1, Guid guid2 )
        {
            string combinedString = $"{guid1}-{guid2}";
            byte[] combinedBytes = Guid.NewGuid().ToByteArray(); // Initialize with a new GUID
            byte[] stringBytes = System.Text.Encoding.UTF8.GetBytes( combinedString );

            for (int i = 0; i < Math.Min( combinedBytes.Length, stringBytes.Length ); i++)
            {
                combinedBytes[ i ] ^= stringBytes[ i ];
            }

            return new Guid( combinedBytes );
        }

        // Function to insert node data
        private static void InsertNode ( Guid id, SqlGeography geography )
        {
            try
            {
                connection.Open();

                string query = $"INSERT INTO Node (Id, GeographyValues) VALUES ('{id}', @geography)";
                using (SqlCommand command = new SqlCommand( query, connection ))
                {
                    _ = command.Parameters.Add( new SqlParameter( "@geography", System.Data.SqlDbType.Udt ) { UdtTypeName = "geography", Value = geography } );

                    _ = command.ExecuteNonQuery();
                }
            }
            finally
            {
                connection.Close();
            }
        }

        private static void AddIncidentEdgeRelation ( Guid vId, Guid edgeId )
        {
            try
            {
                connection.Open();

                string query = $"INSERT INTO IncidentEdges (NodeId, EdgeId) VALUES (' {vId} ', '{edgeId} ');";
                using (SqlCommand command = new SqlCommand( query, connection ))
                {
                    _ = command.ExecuteNonQuery();
                }
            }
            finally
            {
                connection.Close();
            }
        }


        public static bool EdgeExists ( Guid edgeId )
        {

            try
            {
                connection.Open();

                string query = $"SELECT Id FROM Edges WHERE Id = @Id";
                using (SqlCommand command = new SqlCommand( query, connection ))
                {
                    _ = command.Parameters.AddWithValue( "@Id", edgeId );

                    using (SqlDataAdapter adapter = new SqlDataAdapter( command ))
                    {
                        DataTable dataTable = new DataTable();
                        _ = adapter.Fill( dataTable );
                        // if there are rows, (count > 0), a matching edge was found
                        return dataTable.Rows.Count > 0;
                    }
                }
            }
            finally
            {
                connection.Close();
            }
        }

        // Function to insert edge data
        private static void InsertEdge ( Guid edgeId, Guid vId, Guid wId, double eCost )
        {
            if (!( GId.ContainsKey( vId ) && GId.ContainsKey( wId ) ))
            {
                throw new Exception( "Ids have to have been added into GIDs!" );
            }
            Node source = GId[ vId ];
            Node target = GId[ wId ];

            if (source.GraphNodeId > target.GraphNodeId)
            {
                (target, source) = (source, target);
            }

            double yl = Graph.GetDistanceFromLatLon( source.Lat, source.Lon, minLat, source.Lon ) * ( source.Lat < minLat ? -1 : 1 );
            double xl = Graph.GetDistanceFromLatLon( source.Lat, source.Lon, source.Lat, minLon ) * ( source.Lon < minLon ? -1 : 1 );

            double yr = Graph.GetDistanceFromLatLon( target.Lat, target.Lon, minLat, target.Lon ) * ( target.Lat < minLat ? -1 : 1 );
            double xr = Graph.GetDistanceFromLatLon( target.Lat, target.Lon, target.Lat, minLon ) * ( target.Lon < minLon ? -1 : 1 );

            double ShoelaceForward = ( yl + yr ) * ( xl - xr );
            double ShoelaceBackward = ( yr + yl ) * ( xr - xl );

            try
            {
                connection.Open();

                string query = $"INSERT INTO Edge (Id, ShoelaceForward, ShoelaceBackward, " +
                    $"SourceNode, TargetNode, Reversed, Pheromone, Cost, Profit) VALUES " +
                    $"('{edgeId}', '{ShoelaceForward}', '{ShoelaceBackward}', '{vId}', {wId}, '{false}', '{1}', '{eCost}', '{0}')";
                using (SqlCommand command = new SqlCommand( query, connection ))
                {
                    _ = command.ExecuteNonQuery();
                }
            }
            finally
            {
                connection.Close();

                //Edge edge = new Edge( edgeId, source, target, eCost );

                //VNodes[ edge.SourceNode.GraphNodeId ].Incident.Add( edge );
                //VNodes[ edge.TargetNode.GraphNodeId ].Incident.Add( edge );

                //if (VEdges == null)
                //{
                //    VEdges = new List<Edge>();
                //}
                //VEdges.Add( edge );
            }
        }

        // Function to insert intermediate nodes of an edge
        private static void InsertIntermediateNodes ( Guid edgeId, double[] nodes )
        {
            try
            {
                connection.Open();
                for (int i = 0; i < nodes.Length; i += 2)
                {
                    double lat = nodes[ i ];
                    double lon = nodes[ i + 1 ];

                    SqlGeography geography = SqlGeography.Point( lat, lon, 4326 );


                    string query = $"UPDATE Edges SET GeoLocations =  ISNULL(GeoLocations, '') + @geography WHERE  Id = '{edgeId}'";
                    using (SqlCommand command = new SqlCommand( query, connection ))
                    {
                        // Add parameter for the geography
                        _ = command.Parameters.Add( new SqlParameter( "@geography", System.Data.SqlDbType.Udt ) { UdtTypeName = "geography", Value = geography } );

                        _ = command.ExecuteNonQuery();
                    }
                }
            }
            catch (Exception e)
            {
                Console.WriteLine( e.ToString() );
            }
            finally
            {
                connection.Close();
            }
        }

        private static void AssociateTagsWithEdge ( Guid edgeId, string tags )
        {

            try
            {
                connection.Open();

                // Assuming you have a table for storing edge tags
                string query = $"UPDATE Edges SET Tags = ISNULL(Tags, '') + '{tags}' WHERE Id = '{edgeId}'";
                using (SqlCommand command = new SqlCommand( query, connection ))
                {
                    _ = command.ExecuteNonQuery();
                }
            }
            finally
            {
                connection.Close();
            }
        }

        private static void AddEdgeIntoGraph ( Guid edgeId, Guid graphId )
        {
            try
            {
                connection.Open();

                // Assuming you have a table for storing edge tags
                string query = $"INSERT INTO VEdges (EdgeId, GraphId) VALUES (' {edgeId} ', ' {graphId} ')";
                using (SqlCommand command = new SqlCommand( query, connection ))
                {
                    _ = command.ExecuteNonQuery();
                }
            }
            finally
            {
                connection.Close();
            }
        }

        private static void AddNodeIntoGraph ( Guid nodeId, Guid graphId )
        {
            try
            {
                connection.Open();

                // Assuming you have a table for storing edge tags
                string query = $"INSERT INTO VNodes (NodeId, GraphId) VALUES (' {nodeId}', ' {graphId} ')";
                using (SqlCommand command = new SqlCommand( query, connection ))
                {
                    _ = command.ExecuteNonQuery();
                }
            }
            finally
            {
                connection.Close();
            }
        }
    }
}

