using System;
using System.Collections.Generic;

namespace Tour4MeAdvancedProject.Helper
{
    public class ParallelizingIterator
    {
        public static IEnumerable<bool> IterateUntilFalse ( Func<bool> condition )
        {
            while (condition())
            {
                yield return true;
            }
        }
    }
}