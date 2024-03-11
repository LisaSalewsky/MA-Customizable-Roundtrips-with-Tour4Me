using Tour4MeAdvancedProject.ObjectClasses;
using static Tour4MeAdvancedProject.Helper.EnumHelper;

namespace Tour4MeAdvancedProject.Solver
{
    public class GeneticSolver : Selection
    {
        public GeneticSolver () { }




        public override SolveStatus Solve ( Problem P )
        {


            return SolveStatus.Feasible;
        }


    }
}