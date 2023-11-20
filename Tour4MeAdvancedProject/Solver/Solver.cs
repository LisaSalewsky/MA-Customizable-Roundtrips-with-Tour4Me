using System.Collections.Generic;
using static Tour4MeAdvancedProject.Helper.EnumHelper;
using Tour4MeAdvancedProject.ObjectClasses;

namespace Tour4MeAdvancedProject.Solver
{
    public class Solver
    {
        protected List<Node> outputPath = new List<Node>();

        public virtual SolveStatus Solve(Problem p)
        {
            // Implement the solving logic in derived classes
            return SolveStatus.Unsolved;
        }

        public List<Node> GetPath()
        {
            return outputPath;
        }
    }
}