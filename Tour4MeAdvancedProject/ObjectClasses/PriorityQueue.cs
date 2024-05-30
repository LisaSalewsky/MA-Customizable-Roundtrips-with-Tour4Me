using System;
using System.Collections.Generic;

namespace Tour4MeAdvancedProject.ObjectClasses
{
    public class PriorityQueue<T>
    {
        private readonly List<(double, T)> sortedList = new List<(double, T)>();

        public int Count => sortedList.Count;

        public void Enqueue ( double key, T value )
        {
            int index = sortedList.BinarySearch( (key, value), Comparer<(double, T)>.Create( ( x, y ) => x.Item1.CompareTo( y.Item1 ) ) );
            if (index < 0)
            {
                index = ~index; // If not found, BinarySearch returns the bitwise complement of the index of the next element that is larger
            }
            sortedList.Insert( index, (key, value) );
        }

        public (double, T) Dequeue ()
        {
            if (sortedList.Count == 0)
            {
                throw new InvalidOperationException( "Queue is empty" );
            }

            (double, T) top = sortedList[ 0 ];
            sortedList.RemoveAt( 0 );

            return top;
        }

    }

}