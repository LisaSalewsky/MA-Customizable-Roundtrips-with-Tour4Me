using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Web;
using System.Web.Services;
using System.Web.UI;
using System.Web.UI.WebControls;
using static Tour4MeAdvancedProject.Helper.EnumHelper;

namespace Tour4MeAdvancedProject
{
    public partial class MapView : System.Web.UI.Page
    {

        private const double LAT_SEA = 47.45;
        private const double LON_SEA = -122.4;
        private const double SIZ_SEA = 2;

        private const double LAT_DOR = 51.3648;
        private const double LON_DOR = 7.2185;


        private static Tuple<double, double> MarkerLatLon;
        private static bool Zoomed = true;
        // OuterBox: List(List(Lat,Lon)) = List((minmin(LatLon), minmax(LatLon), maxmax(LatLon), maxmin(LatLon))
        private static List<List<Tuple<double, double>>> OuterBox = new List<List<Tuple<double, double>>>() { };
        // InnerBox: List(List(Lat,Lon)) = List((minmin(LatLon), minmax(LatLon), maxmax(LatLon), maxmin(LatLon))
        private static List<List<Tuple<double, double>>> InnerBox= new List<List<Tuple<double, double>>>() { };


        private HighwayTag[] HighwayTags;
        private SurfaceTag[] SurfaceTags;
        private Algo[] algos;

        private static double LatGran = 0.5 / 4;
        private static double LatPad = 0.5 / 6;
        private static double AbsMinLat = LAT_DOR;
        private static double AbsMaxLat = AbsMinLat + LatGran;

        private static double LonGran = 0.75 / 4;
        private static double LonPad = 0.75 / 6;
        private static double AbsMinLon = LON_DOR;
        private static double AbsMaxLon = AbsMinLon + LonGran;
        private static readonly string[] COLORS = {"red", "green", "blue", "maroon", "purple", "lime", "navy" };

        private string SelectedMap = "dor";

        private int RouteCounter = 0;
        
        // StoredRoutes: List(RouteID, Path)
        private List<Tuple<int, List<int>>> Stored_routes = new List<Tuple<int, List<int>>>();

        private double CenterLat = 51.489808;
        private double CenterLon = 7.406319;


        protected void Page_Load(object sender, EventArgs e)
        {

        }

        protected void GetPathButton_Click(object sender, EventArgs e)
        {
            Response.Write("<script>alert('Testing');</script>");

            
        }

        protected static List<Tuple<string, List<object>>> GetPath(double lat, double lon, string map)
        {
            List<Tuple<string, List<object>>> result = new List<Tuple<string, List<object>>>();

            return result;
        }

        //public static string SetLatLon() 
        [WebMethod]
        public static string SetLatLon(double latIn, double lonIn)
        {
            MarkerLatLon = new Tuple<double, double>(latIn, lonIn);
            return "test";
        }

        protected static List<Tuple<string, List<object>>> RenderGraph()
        {
            List<Tuple<string, List<object>>> result = new List<Tuple<string, List<object>>>();

            List<object> tagList = new List<object>();
            tagList.Add(HighwayTag.Footway);
            tagList.Add(HighwayTag.Cycleway);
            tagList.Add(HighwayTag.Unclassified);
            tagList.Add(HighwayTag.Residential);
            tagList.Add(HighwayTag.Path);
            tagList.Add(HighwayTag.Track);
            tagList.Add(HighwayTag.Secondary);
            tagList.Add(SurfaceTag.Paved);
            tagList.Add(SurfaceTag.Cobblestone);
            tagList.Add(SurfaceTag.Gravel);
            tagList.Add(SurfaceTag.Unpaved);
            tagList.Add(SurfaceTag.Compacted);
            tagList.Add(SurfaceTag.FineGravel);
            tagList.Add(SurfaceTag.Rock);
            tagList.Add(SurfaceTag.Pebblestone);

            result.Add(new Tuple<string, List<object>>("tags", tagList));

            List<object> algoList = new List<object>();
            algoList.Add(Algo.Greedy);
            algoList.Add(Algo.minCost);
            algoList.Add(Algo.ILS);

            result.Add(new Tuple<string, List<object>>("algorithms", algoList));

            return result;
        }

    }
}