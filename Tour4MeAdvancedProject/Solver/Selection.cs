using System.Collections.Generic;
using Tour4MeAdvancedProject.ObjectClasses;
using static Tour4MeAdvancedProject.Helper.EnumHelper;

namespace Tour4MeAdvancedProject.Solver
{
    public class Selection
    {
        protected List<int> OutputPath;

        public Selection () { }

        public virtual SolveStatus Solve ( Problem p )
        {
            return SolveStatus.Unsolved;
        }

        public List<int> GetPath ()
        {
            return OutputPath;
        }
    }

}
