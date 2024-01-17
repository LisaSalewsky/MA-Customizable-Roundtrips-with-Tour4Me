using System.Collections.Generic;
using System.IO;
using System;
using System.Globalization;
using System.Configuration;
using System.Data.SqlClient;
using System.Data;
using System.Web;
using System.Web.UI;
using Tour4MeAdvancedProject.Helper;
using System.Web.Caching;
using Microsoft.SqlServer.Types;
using System.Data.SqlTypes;


namespace Tour4MeAdvancedProject.ObjectClasses
{
    public class Graph
    {
        private string dbConString = ConfigurationManager.ConnectionStrings["con"].ConnectionString;
        private SqlConnection DBSqlConnection;
        private SqlCommand DBSqlCommand;

        private Guid Id;

        public double MaxLat { get; set; }
        public double MinLat { get; set; }
        public double MaxLon { get; set; }
        public double MinLon { get; set; }
        public double CenterLat { get; set; }
        public double CenterLon { get; set; }

        public List<Node> VNodes { get; set; }
        public List<Edge> VEdges { get; set; }

        public Dictionary<long, int> GIdNode { get; set; }

        public Graph(Guid InID, out string error)
        {
            error = "";
            Id = InID;
            VNodes = new List<Node>();
            VEdges = new List<Edge>();
            GIdNode = new Dictionary<long, int>();
            string connectError = "";
            try
            {
                connectToDB(out connectError);

                DBSqlCommand = new SqlCommand("SELECT * from Graph WHERE Id='" + Id + "';", DBSqlConnection);
                SqlDataAdapter DBSqldataAdapter = new SqlDataAdapter(DBSqlCommand);
                SqlDataReader dataReader = DBSqlCommand.ExecuteReader();

                if (dataReader.HasRows)
                {
                    while (dataReader.Read())
                    {
                        var VNodesTemp = dataReader.GetValue((int)GraphColumnsEnumHelper.Columns.VNodes);
                        var VEdgesTemp = dataReader.GetValue((int)GraphColumnsEnumHelper.Columns.VEdges);
                        var GIdNodeTemp = dataReader.GetValue((int)GraphColumnsEnumHelper.Columns.GraphIdNode);


                        //SqlBytes MaxLatLonBytes = dataReader.GetSqlBytes((int)GraphColumnsEnumHelper.Columns.MaxLatLon);
                        //string MaxLatLonString = BitConverter.ToString(MaxLatLonBytes.Buffer).Replace("-", "");
                        //SqlGeography MaxLatLon = SqlGeography.Parse(MaxLatLonString);
                        //SqlBytes MinLatLonBytes = dataReader.GetSqlBytes((int)GraphColumnsEnumHelper.Columns.MaxLatLon);
                        //string MinLatLonString = BitConverter.ToString(MinLatLonBytes.Buffer).Replace("-", "");
                        //SqlGeography MinLatLon = SqlGeography.Parse(MinLatLonString);
                        //SqlBytes CenterLatLonBytes = dataReader.GetSqlBytes((int)GraphColumnsEnumHelper.Columns.MaxLatLon);
                        //string CenterLatLonString = BitConverter.ToString(CenterLatLonBytes.Buffer).Replace("-", "");
                        //SqlGeography CenterLatLon = SqlGeography.Parse(CenterLatLonString);

                        var MaxLatLonBytes = dataReader.GetSqlBytes((int)GraphColumnsEnumHelper.Columns.MaxLatLon);
                        var MaxLatLonTemp = SqlGeography.STGeomFromWKB(MaxLatLonBytes, 4326);
                        var MinLatLonBytes = dataReader.GetSqlBytes((int)GraphColumnsEnumHelper.Columns.MaxLatLon);
                        var MinLatLonTemp = SqlGeography.STGeomFromWKB(MinLatLonBytes, 4326);
                        var CenterLatLonBytes = dataReader.GetSqlBytes((int)GraphColumnsEnumHelper.Columns.MaxLatLon);
                        var CenterLatLonTemp = SqlGeography.STGeomFromWKB(CenterLatLonBytes, 4326);
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
            error = error + connectError;
        }

        private void connectToDB(out string error)
        {
            error = "";
            try
            {
                DBSqlConnection = new SqlConnection(dbConString);
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


        public void AddEdge(Edge edge)
        {
            if (VEdges == null)
            {
                VEdges = new List<Edge>();
            }
            VNodes[edge.SourceNode.Id].Incident.Add(edge);
            VNodes[edge.TargetNode.Id].Incident.Add(edge);
            VEdges.Add(edge);
        }
        public Graph(string fileName)
        {
            using (StreamReader file = new StreamReader(fileName))
            {
                VNodes = new List<Node>();
                VEdges = new List<Edge>();
                GIdNode = new Dictionary<long, int>();
                string str;
                int nNodes = 0;
                int nEdges = 0;
                int cNodes = 0;
                int cEdges = 0;

                while ((str = file.ReadLine()) != null)
                {
                    char type = str[0];

                    if (type == 'g')
                    {
                        string t = NextWord(ref str, ' ');
                        MaxLat = double.Parse(NextWord(ref str, ' '), CultureInfo.InvariantCulture);
                        MinLat = double.Parse(NextWord(ref str, ' '), CultureInfo.InvariantCulture);
                        MaxLon = double.Parse(NextWord(ref str, ' '), CultureInfo.InvariantCulture);
                        MinLon = double.Parse(NextWord(ref str, ' '), CultureInfo.InvariantCulture);
                        CenterLat = double.Parse(NextWord(ref str,' '), CultureInfo.InvariantCulture);
                        CenterLon = double.Parse(NextWord(ref str, ' '), CultureInfo.InvariantCulture);
                        double test = double.Parse("51.57313333", CultureInfo.InvariantCulture);
                    }
                    else if (type == 'p')
                    {
                        string t = NextWord(ref str, ' ');
                        nNodes = int.Parse(NextWord(ref str, ' '));
                        nEdges = int.Parse(NextWord(ref str, ' '));

                        VNodes = new List<Node>(nNodes);
                    }
                    else if (type == 'n')
                    {
                        string t = NextWord(ref str, ' ');
                        long id = long.Parse(NextWord(ref str, ' '));
                        double lat = double.Parse(NextWord(ref str, ' '), CultureInfo.InvariantCulture);
                        double lon = double.Parse(NextWord(ref str, ' '), CultureInfo.InvariantCulture);

                        Node node = new Node(cNodes, id, lat, lon);
                        VNodes.Add(node);

                        GIdNode.Add(id, cNodes);

                        cNodes++;
                    }
                    else if (type == 'e')
                    {
                        string t = NextWord(ref str, ' ');
                        long vId = long.Parse(NextWord(ref str, ' '));
                        long wId = long.Parse(NextWord(ref str, ' '));
                        double cost = double.Parse(NextWord(ref str, ' '), CultureInfo.InvariantCulture);

                        int sId = GIdNode[vId];
                        int tId = GIdNode[wId];

                        if (EdgeExists(sId, tId))
                        {
                            file.ReadLine();
                            file.ReadLine();
                            continue;
                        }
                        // TODO: DB Access to get Node and correct Data
                        Edge edge = AddEdge(new Node(sId, vId,0,0), new Node(tId,wId,0,0), cost);
                        //Edge edge = AddEdge(cEdges, sId, tId, cost);
                        cEdges++;

                        str = file.ReadLine();
                        if (str[0] != 'f')
                        {
                            throw new InvalidOperationException("Error: graph file not in correct format; 'f' expected after 'e'");
                        }

                        t = NextWord(ref str, ' ');
                        while (HasWord(ref str, ' '))
                        {
                            double lat = double.Parse(NextWord(ref str, ' '), CultureInfo.InvariantCulture);
                            double lon = double.Parse(NextWord(ref str, ' '), CultureInfo.InvariantCulture);
                            edge.GeoLocations.Add(new Tuple<double, double>(lon, lat));
                        }

                        str = file.ReadLine();
                        if (str[0] != 'g')
                        {
                            throw new InvalidOperationException("Error: graph file not in correct format; 'g' expected after 'f'");
                        }

                        t = NextWord(ref str, ' ');
                        while (HasWord(ref str, ' '))
                        {
                            string word = NextWord(ref str, ' ');
                            word = char.ToUpper(word[0]) + word.Substring(1);
                            edge.Tags.Add(word);
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

        public Edge AddEdge(Node s, Node t, double cost)
        {
            if (s.Id > t.Id)
            {
                Node temp = s;
                s = t;
                t = temp;
            }

            Edge edge = new Edge(s, t, cost);

            Node l = VNodes[s.Id];
            Node r = VNodes[t.Id];

            Node c = new Node(-1, -1, MinLat, MinLon);
            double yl = GetDistanceFromLatLon(l, new Node(-1, -1, c.Lat, l.Lon)) * (l.Lat < c.Lat ? -1 : 1);
            double xl = GetDistanceFromLatLon(l, new Node(-1, -1, l.Lat, c.Lon)) * (l.Lon < c.Lon ? -1 : 1);

            double yr = GetDistanceFromLatLon(r, new Node(-1, -1, c.Lat, r.Lon)) * (r.Lat < c.Lat ? -1 : 1);
            double xr = GetDistanceFromLatLon(r, new Node(-1, -1, r.Lat, c.Lon)) * (r.Lon < c.Lon ? -1 : 1);

            edge.ShoelaceForward = (yl + yr) * (xl - xr);
            edge.ShoelaceBackward = (yr + yl) * (xr - xl);

            AddEdge(edge);
            return edge;
        }


        public Edge GetEdge(int sId, int tId)
        {
            foreach (Edge e in VNodes[sId].Incident)
            {
                if (tId == e.SourceNode.Id || tId == e.TargetNode.Id)
                {
                    return e;
                }
            }

            return null;
        }

        public bool EdgeExists(int sId, int tId)
        {
            foreach (Edge e in VNodes[sId].Incident)
            {
                if (tId == e.SourceNode.Id || tId == e.TargetNode.Id)
                {
                    return true;
                }
            }

            return false;
        }

        public double Length(List<int> path)
        {
            double length = 0.0;

            for (int i = 0; i < path.Count - 1; i++)
            {
                Edge edge = GetEdge(path[i], path[i + 1]);
                length += edge.Cost;
            }

            return length;
        }


        public List<Tuple<int, Path>> CalculateRing(int sourceNode, double innerDistance, double outerDistance,
                                                    int nodeLimit, HashSet<int> contained)
        {
            double[] dist = new double[VNodes.Count];
            for (int i = 0; i < dist.Length; i++)
            {
                dist[i] = double.MaxValue;
            }

            double[] actDist = new double[VNodes.Count];

            PriorityQueue<Tuple<int, double>> queue = new PriorityQueue<Tuple<int, double>>();
            Tuple<int, Edge>[] parent = new Tuple<int, Edge>[VNodes.Count];
            List<int> neighbors = new List<int>();
            dist[sourceNode] = 0.0;
            actDist[sourceNode] = 0.0;

            queue.Enqueue(0.0, new Tuple<int, double>(sourceNode, 0.0));

            List<int> visited = new List<int>();

            while (queue.Count > 0)
            {
                (double currentDist, (int currentNode, double currentActual)) = queue.Dequeue();

                if (currentActual > outerDistance)
                {
                    continue;
                }

                double bestKnownDist = dist[currentNode];

                if (bestKnownDist == double.MaxValue)
                {
                    dist[currentNode] = currentDist;
                    actDist[currentNode] = currentActual;
                    bestKnownDist = currentDist;
                }

                if (bestKnownDist != currentDist)
                {
                    continue;
                }


                neighbors.Add(VNodes[currentNode].Incident.Count);
               
                foreach (Edge edge in VNodes[currentNode].Incident)
                {
                    int neighborId = edge.SourceNode.Id  == currentNode ? edge.TargetNode.Id : edge.SourceNode.Id;

                    double newDist = bestKnownDist + (edge.Cost / (edge.Profit + 0.1));
                    double newActual = currentActual + edge.Cost;

                    if (newDist < dist[neighborId])
                    {
                        queue.Enqueue(newDist, new Tuple<int, double>(neighborId, newActual));
                        dist[neighborId] = newDist;
                        actDist[neighborId] = newActual;
                        parent[neighborId] = Tuple.Create(currentNode, edge);

                        visited.Add(neighborId);
                    }
                }
            }

            List<Tuple<int, Path>> output = new List<Tuple<int, Path>>();
            int outputSize = 0;

            foreach (int vis in visited)
            {
                if (contained != null && !contained.Contains(vis))
                {
                    continue;
                }

                if (outputSize > nodeLimit)
                {
                    break;
                }

                if (actDist[vis] < innerDistance)
                {
                    continue;
                }

                int current = vis;
                Path path = new Path();

                bool aborted = false;

                while (current != sourceNode)
                {
                    Edge e = parent[current].Item2;
                    path.Edges.Insert(0, new Edge(e, e.TargetNode.Id == current));
                    path.Length += e.Cost;

                    // if any of the null checks holds, current was not deeply enough routed
                    if (parent[current] != null && // current has no parent (is the start)
                        parent[parent[current].Item1] != null)// current has not 2 parent nodes, so it's the first child only
                    {
                        if (current == parent[parent[current].Item1].Item1)
                        //if (current == parent[parent[current].Item1].Item1)
                        {
                            aborted = true;
                            break;
                        }

                        if (parent[parent[parent[current].Item1].Item1] != null && // current has not 3 parent nodes
                            current == parent[parent[parent[current].Item1].Item1].Item1)
                        //if (current == parent[parent[parent[current].Item1].Item1].Item1)
                        {
                            aborted = true;
                            break;
                        }
                    }
                    current = parent[current].Item1;
                }

                if (!aborted)
                {
                    output.Add(Tuple.Create(vis, path));
                    outputSize++;
                }
            }

            return output;
        }

        private static string NextWord(ref string line, char delimiter)
        {
            if (line.Length == 0)
            {
                throw new InvalidOperationException("Error: target line is empty");
            }

            int pos = line.IndexOf(delimiter);
            string word;

            if (pos != -1)
            {
                word = line.Substring(0, pos);
                line = line.Remove(0, pos + 1);
            }
            else
            {
                word = line;
                line = string.Empty;
            }

            return word;
        }

        private static bool HasWord(ref string line, char delimiter)
        {
            return line.Length != 0;
        }

        public double GetDistanceFromLatLon(int node1Id, int node2Id)
        {
            const double R = 6371; // Radius of the earth in km
            Node node1 = VNodes[node1Id];
            Node node2 = VNodes[node2Id];
            double dLat = Deg2Rad(node2.Lat - node1.Lat);  // deg2rad below
            double dLon = Deg2Rad(node2.Lon - node1.Lon);
            double a =
              Math.Sin(dLat / 2) * Math.Sin(dLat / 2) +
              Math.Cos(Deg2Rad(node1.Lat)) * Math.Cos(Deg2Rad(node2.Lat)) *
              Math.Sin(dLon / 2) * Math.Sin(dLon / 2)
              ;
            double c = 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a));
            double distance = R * c; // Distance in km
            return distance;
        }

        public double GetDistanceFromLatLon(Node node1, Node node2)
        {
            const double R = 6371000; // Radius of the earth in km
            double dLat = Deg2Rad(node2.Lat - node1.Lat);  // deg2rad below
            double dLon = Deg2Rad(node2.Lon - node1.Lon);
            double a =
              Math.Sin(dLat / 2) * Math.Sin(dLat / 2) +
              Math.Cos(Deg2Rad(node1.Lat)) * Math.Cos(Deg2Rad(node2.Lat)) *
              Math.Sin(dLon / 2) * Math.Sin(dLon / 2)
              ;
            double c = 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a));
            double distance = R * c; // Distance in km
            return distance;
        }

        private double Deg2Rad(double deg)
        {
            return deg * (Math.PI / 180);
        }


        public double ShortestPath(int start, int end)
        {
            Dictionary<int, double> dist = new Dictionary<int, double>();
            PriorityQueue<Tuple<int, double>> queue = new PriorityQueue<Tuple<int, double>>();
            

            dist[start] = 0.0;
            queue.Enqueue(0.0, new Tuple<int,double>(start, 0.0));

            while (queue.Count > 0)
            {
                var current = queue.Dequeue();

                double actualHeuristic = current.Item1;
                int currentNode = current.Item2.Item1;
                double actual = current.Item2.Item2;

                if (currentNode == end)
                    return dist[currentNode];

                if (!dist.TryGetValue(currentNode, out double bestKnownDist))
                {
                    dist[currentNode] = actual;
                    bestKnownDist = actual;
                }

                if (bestKnownDist != actual)
                    continue;

                foreach (Edge edge in VNodes[currentNode].Incident)
                {
                    int neighborId = edge.SourceNode.Id == currentNode ? edge.TargetNode.Id : edge.SourceNode.Id;

                    double newDistance = bestKnownDist + edge.Cost;

                    if (!dist.TryGetValue(neighborId, out double currentNeighborDist))
                    {
                        dist[neighborId] = double.MaxValue;
                        currentNeighborDist = double.MaxValue;
                    }

                    if (newDistance < currentNeighborDist)
                    {
                        double heuristic = newDistance + GetDistanceFromLatLon(neighborId, end);
                        queue.Enqueue(heuristic, new Tuple<int, double>(neighborId, newDistance));
                        dist[neighborId] = newDistance;
                    }
                }
            }
            return double.MaxValue;
        }

    }

}