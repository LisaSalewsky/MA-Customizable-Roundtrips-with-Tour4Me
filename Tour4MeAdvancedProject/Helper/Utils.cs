using System;
using System.Collections.Generic;
using System.Linq;
using Tour4MeAdvancedProject.ObjectClasses;
using static Tour4MeAdvancedProject.Helper.EnumHelper;

namespace Tour4MeAdvancedProject.Helper
{
    public static class Utils
    {

        public static void AddTags ( ref HashSet<SurfaceTag> addedSurfaceTags, ref HashSet<HighwayTag> addedPathTypes, ref HashSet<string> addedSurroundings, string currentTag )
        {
            if (Enum.TryParse<SurfaceTag>( currentTag, true, out SurfaceTag surfaceTag ))
            {
                _ = addedSurfaceTags.Add( surfaceTag );
            }
            if (Enum.TryParse<HighwayTag>( currentTag, true, out HighwayTag pathType ))
            {
                _ = addedPathTypes.Add( pathType );
            }
            bool tagFound = false;
            foreach (string[] tagsArray in Surroundings.Values.Values)
            {
                if (tagsArray.Contains( currentTag ))
                {
                    tagFound = true;
                    // Tag found in one of the arrays
                    break;
                }
            }

            if (tagFound)
            {
                _ = addedSurroundings.Add( currentTag );
            }
        }

        public static void CalculateElevationDiffAndSteepness ( Edge edge, ref double maxSteepness, ref double currentElevationDiff )
        {
            double elevationDiff = Math.Abs( edge.SourceNode.Elevation - edge.TargetNode.Elevation );

            if (( elevationDiff / edge.Cost * 100 ) > maxSteepness)
            {
                maxSteepness = elevationDiff / edge.Cost * 100;
            }

            currentElevationDiff += elevationDiff;
        }

    }
}