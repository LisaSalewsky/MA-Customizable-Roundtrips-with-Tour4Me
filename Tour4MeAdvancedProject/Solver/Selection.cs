using System.Collections.Generic;
using static Tour4MeAdvancedProject.Helper.EnumHelper;
using Tour4MeAdvancedProject.ObjectClasses;

namespace Tour4MeAdvancedProject.Solver
{
    public class Selection : Solver
    {
        protected List<int> OutputPath;


        public virtual SolveStatus Solve(Problem P)
        {
            return SolveStatus.Unsolved; // Placeholder, please replace with actual result
        }

        public new List<int> GetPath()
        {
            return OutputPath;
        }
    }

}