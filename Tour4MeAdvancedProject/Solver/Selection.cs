using System.Collections.Generic;
using static Tour4MeAdvancedProject.Helper.EnumHelper;
using Tour4MeAdvancedProject.ObjectClasses;
using System;

namespace Tour4MeAdvancedProject.Solver
{
    public class Selection : Solver
    {
        protected List<int> OutputPath;

        public Selection() { }
        
        public SolveStatus Solver()
        {
            return SolveStatus.Unsolved;
        }

        public new List<int> GetPath()
        {
            return OutputPath;
        }
    }

}
