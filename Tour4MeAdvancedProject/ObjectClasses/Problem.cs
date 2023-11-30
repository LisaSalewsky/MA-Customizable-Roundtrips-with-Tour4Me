using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using SysPath = System.IO.Path;


namespace Tour4MeAdvancedProject.ObjectClasses
{

    public class Problem
    {
        protected string graphName;

        public int Start { get; set; }
        public double CenterLat { get; set; }
        public double CenterLon { get; set; }
        public HashSet<string> PrefTags { get; set; }
        public HashSet<string> AvoidTags { get; set; }
        public Graph Graph { get; set; }
        public Graph Backbone { get; set; }
        public double RunningTime { get; set; }
        public double EdgeProfitImportance { get; set; }
        public double CoveredAreaImportance { get; set; }
        public List<List<double>> ShortestPath { get; set; }

        public List<int> Path { get; set; }
        public double Quality { get; set; }

        public double TargetDistance { get; set; }
        public List<string> Metadata { get; set; }

        public Problem(string fileName)
        {
            Graph = new Graph(fileName);
            Path = new List<int>();
            Metadata = new List<string>();
            PrefTags = new HashSet<string>();
            AvoidTags = new HashSet<string>();
        }

        public void FillShortestPath(string filename)
        {
            var path = SysPath.GetDirectoryName(SysPath.GetDirectoryName(SysPath.GetDirectoryName(System.Reflection.Assembly.GetExecutingAssembly().GetName().CodeBase)));
            Uri uri = new Uri(path);
            string localPath = Uri.UnescapeDataString(uri.LocalPath);
            string finalPath = SysPath.Combine(localPath, "input", filename, "_sp.txt");

            using (StreamWriter outputFile = new StreamWriter(finalPath))
            {

                ShortestPath = new List<List<double>>(Graph.VNodes.Count);

                for (int source = 0; source < Graph.VNodes.Count; source++)
                {
                    ShortestPath.Add(new List<double>(Graph.VNodes.Count));
                    Dictionary<int, double> dist = new Dictionary<int, double>();
                    PriorityQueue<Tuple<int, double>> queue = new PriorityQueue<Tuple<int, double>>((x, y) => x.Item1.CompareTo(y.Item1));

                    dist[source] = 0.0;
                    queue.Enqueue(0.0, new Tuple<int, double>(source, 0.0));

                    while (queue.Count > 0)
                    {
                        (double distance, (int currentNode, double actual)) = queue.Dequeue();

                        if (!dist.TryGetValue(currentNode, out double bestKnownDist))
                        {
                            dist[currentNode] = distance;
                            bestKnownDist = distance;
                        }

                        if (bestKnownDist != distance)
                            continue;

                        foreach (Edge edge in Graph.VNodes[currentNode].Incident)
                        {
                            int neighborId = edge.S == currentNode ? edge.T : edge.S;
                            double newDistance = bestKnownDist + edge.Cost;

                            if (!dist.TryGetValue(neighborId, out double currentDist))
                                dist[neighborId] = double.MaxValue;

                            if (newDistance < currentDist)
                            {
                                queue.Enqueue(newDistance, new Tuple<int, double>(neighborId, 0.0));
                                dist[neighborId] = newDistance;
                            }
                        }
                    }

                    for (int target = 0; target < Graph.VNodes.Count; target++)
                    {
                        ShortestPath[source].Add(dist.ContainsKey(target) ? dist[target] : double.MaxValue);
                        outputFile.WriteLine($"c" + source + target + (dist.ContainsKey(target) ? dist[target].ToString() : "inf"));
                    }
                }
            }
        }

        public void OutputPath(string fileName)
        {
            using (StreamWriter outputFile = new StreamWriter("/home/hagedoorn/Documents/TUD/Code/AOPcpp/output/" + fileName + ".json"))
            {
                outputFile.WriteLine("{");
                outputFile.WriteLine("    \"info\": {");
                outputFile.WriteLine($"        \"graph_name\": \"{graphName}\",");
                outputFile.WriteLine($"        \"n_nodes\": {Graph.VNodes.Count},");
                outputFile.WriteLine($"        \"target_distance\": {TargetDistance},");
                outputFile.WriteLine($"        \"center_lat\": {Graph.CenterLat},");
                outputFile.WriteLine($"        \"center_lon\": {Graph.CenterLon}");
                outputFile.WriteLine("    },");
                outputFile.WriteLine("    \"node_path\": [");

                foreach (int node in Path)
                {
                    outputFile.Write($"        {Graph.VNodes[node].GId}, ");
                }

                outputFile.WriteLine($"{Graph.VNodes[Path.First()].GId}], ");
                outputFile.WriteLine("    \"cord_path\": [");

                foreach (int node in Path)
                {
                    outputFile.Write($"[{Graph.VNodes[node].Lat}, {Graph.VNodes[node].Lon}], ");
                }

                outputFile.WriteLine("]");
                outputFile.WriteLine("}");
            }
        }

        public void OutputToGPX(string fileName)
        {
            using (StreamWriter outputFile = new StreamWriter("/home/hagedoorn/Documents/TUD/Code/AOPcpp/output/gpx/" + fileName + ".gpx"))
            {
                outputFile.WriteLine("<?xml version='1.0' encoding='UTF-8'?>");
                outputFile.WriteLine("<gpx version=\"1.1\" creator=\"JeBoi\" xmlns=\"http://www.topografix.com/GPX/1/1\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">");
                outputFile.WriteLine("  <metadata>");
                outputFile.WriteLine($"    <name>{fileName}</name>");
                outputFile.WriteLine("  </metadata>");
                outputFile.WriteLine("  <trk>");
                outputFile.WriteLine($"    <name>{fileName}</name>");
                outputFile.WriteLine("    <trkseg>");

                foreach (int node in Path)
                {
                    outputFile.WriteLine($"      <trkpt lat=\"{Graph.VNodes[node].Lat}\" lon=\"{Graph.VNodes[node].Lon}\"></trkpt>");
                }

                outputFile.WriteLine("    </trkseg>");
                outputFile.WriteLine("  </trk>");
                outputFile.WriteLine("</gpx>");
            }
        }

        public string OutputToString()
        {
            //StringBuilder outputString = new StringBuilder("{\n    \"path\": [\n");
            StringBuilder outputString = new StringBuilder("{\n    [\n");

            foreach (int node in Path)
            {
                outputString.Append($"        [{Graph.VNodes[node].Lat},{Graph.VNodes[node].Lon}], \n");

                Edge edge = Graph.GetEdge(node, Path.ElementAtOrDefault(Path.IndexOf(node) + 1));

                if (edge == null)
                    continue;

                bool reverse = Graph.VNodes[node].GId < Graph.VNodes[edge.T].GId;
                if (!reverse) 
                {
                    edge.GeoLocations.Reverse();
                    reverse = true;
                }
                foreach ((double lat, double lon) in edge.GeoLocations)
                {
                    outputString.Append($"        [{lat},{lon}], \n");
                }
            }

            outputString.Append($"        [{Graph.VNodes[Path.First()].Lat},{Graph.VNodes[Path.First()].Lon}] \n");
            outputString.Append("    ],\n");
            outputString.Append("    \"meta\": [\n");

            for (int i = 0; i < Metadata.Count - 1; i++)
            {
                outputString.Append($"        \"{Metadata[i]}\",\n");
            }

            outputString.Append($"        \"{Metadata[Metadata.Count - 1]}\"\n");
            outputString.Append("    ]\n}");

            return outputString.ToString();
        }

        public List<KeyValuePair<string, string>> OutputToResultString()
        {
            List<KeyValuePair<string, string>> result = new List<KeyValuePair<string, string>>();
            StringBuilder outputString = new StringBuilder("[");

            foreach (int node in Path)
            {
                outputString.Append($"[{Graph.VNodes[node].Lat},{Graph.VNodes[node].Lon}],");

                Edge edge = Graph.GetEdge(node, Path.ElementAtOrDefault(Path.IndexOf(node) + 1));

                if (edge == null)
                    continue;

                bool reverse = Graph.VNodes[node].GId < Graph.VNodes[edge.T].GId;
                if (!reverse)
                {
                    edge.GeoLocations.Reverse();
                    reverse = true;
                }
                foreach ((double lat, double lon) in edge.GeoLocations)
                {
                    outputString.Append($"[{lat},{lon}],");
                }
            }
            //outputString = outputString.Remove(outputString.Length - 1, 1);
            outputString.Append($"[{Graph.VNodes[Path.First()].Lat},{Graph.VNodes[Path.First()].Lon}]");
            outputString.Append("]");

            result.Add(new KeyValuePair<string, string>("path", outputString.ToString()));

            outputString = new StringBuilder("[");

            for (int i = 0; i < Metadata.Count - 1; i++)
            {
                outputString.Append($"\"{Metadata[i]}\",");
            }

            outputString.Append($"\"{Metadata[Metadata.Count - 1]}\"");
            outputString.Append("]");

            result.Add(new KeyValuePair<string, string>("meta", outputString.ToString()));

            return result;
        }

        public void CalculateProfit(Graph G)
        {
            foreach (Edge edge in G.VEdges)
            {
                edge.Profit = 0.0001;
                foreach (string tag in edge.Tags)
                {
                    if (Math.Abs(edge.Profit - 0.0001) < double.Epsilon && PrefTags.Contains(tag))
                    {
                        edge.Profit = 1;
                    }
                    if (AvoidTags.Contains(tag))
                    {
                        edge.Profit = -1;
                    }
                }
            }
        }

        public double GetQuality(double profit, double area)
        {
            return (EdgeProfitImportance * profit / TargetDistance) + (CoveredAreaImportance * area / (Math.PI * TargetDistance * TargetDistance));
        }

        public double GetProfit(Path path)
        {
            path.Visited = new List<int>(Graph.VEdges.Count);

            double quality = 0.0;

            foreach (DirEdge dEdge in path.Edges)
            {
                if (path.Visited[dEdge.Edge.Id] == 0)
                {
                    quality += dEdge.Edge.Cost * dEdge.Edge.Profit;
                    path.Visited[dEdge.Edge.Id]++;
                }
            }

            return quality;
        }

        public double GetProfit(List<int> path)
        {
            HashSet<Edge> edgeSet = new HashSet<Edge>();

            double quality = 0.0;

            for (int i = 0; i < path.Count - 1; i++)
            {
                int current = path[i];
                int next = path[i + 1];

                Edge edge = Graph.GetEdge(current, next);

                if (!edgeSet.Contains(edge) && edge != null)
                {
                    quality += edge.Cost * edge.Profit;
                    edgeSet.Add(edge);
                }
            }

            return quality;
        }

        public double GetArea(Path path)
        {
            double area = 0.0;

            foreach (DirEdge dEdge in path.Edges)
            {
                area += !dEdge.Reversed ? dEdge.Edge.ShoelaceForward : dEdge.Edge.ShoelaceBackward;
            }

            return area / 2;
        }

        public double GetArea(List<int> path)
        {
            double area = 0.0;

            int prev = -1;
            for (int i = 0; i < path.Count; i++)
            {
                int current = path[i];

                if (prev != -1)
                {
                    Edge edge = Graph.GetEdge(prev, current);

                    area += prev == edge.S ? edge.ShoelaceForward : edge.ShoelaceBackward;
                }

                prev = current;
            }

            if (prev != path.First())
            {
                Edge edge = Graph.GetEdge(prev, path.First());
                area += prev == edge.S ? edge.ShoelaceForward : edge.ShoelaceBackward;
            }

            return area / 2;
        }

        public double GetLength(List<int> path)
        {
            double length = 0.0;

            for (int i = 0; i < path.Count - 1; i++)
            {
                Edge edge = Graph.GetEdge(path[i], path[i + 1]);

                length += edge.Cost;
            }

            return length;
        }
    }

}