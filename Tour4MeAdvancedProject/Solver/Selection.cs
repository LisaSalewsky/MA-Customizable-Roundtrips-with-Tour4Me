using System.Collections.Generic;
using Tour4MeAdvancedProject.ObjectClasses;
using static Tour4MeAdvancedProject.Helper.EnumHelper;

namespace Tour4MeAdvancedProject.Solver
{
    public class Selection
    {
        protected List<int> OutputPath;

        public Selection () { }

        public virtual SolveStatus Solve ( ref Problem p )
        {
            return SolveStatus.Unsolved;
        }

        public virtual int SolveMaxTime ( ref Problem p, int maxTime )
        {
            return -1;
        }

        public List<int> GetPath ()
        {
            return OutputPath;
        }
    }

}
