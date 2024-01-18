using System;
using System.Collections.Generic;

namespace Tour4MeAdvancedProject.ObjectClasses
{
    public class PriorityQueue<T>
    {
        private readonly List<(double, T)> heap = new List<(double, T)>();

        public int Count => heap.Count;

        public void Enqueue ( double priority, T item )
        {
            heap.Add( (priority, item) );
            HeapifyUp( heap.Count - 1 );
        }

        public (double, T) Dequeue ()
        {
            if (heap.Count == 0)
            {
                throw new InvalidOperationException( "Queue is empty" );
            }

            (double, T) top = heap[ 0 ];
            heap[ 0 ] = heap[ heap.Count - 1 ];
            heap.RemoveAt( heap.Count - 1 );

            if (heap.Count > 1)
            {
                HeapifyDown( 0 );
            }

            return top;
        }

        private void HeapifyUp ( int index )
        {
            while (index > 0)
            {
                int parentIndex = ( index - 1 ) / 2;

                if (heap[ index ].Item1 < heap[ parentIndex ].Item1)
                {
                    Swap( index, parentIndex );
                    index = parentIndex;
                }
                else
                {
                    break;
                }
            }
        }

        private void HeapifyDown ( int index )
        {
            while (true)
            {
                int leftChild = ( 2 * index ) + 1;
                int rightChild = ( 2 * index ) + 2;
                int smallest = index;

                if (leftChild < heap.Count && heap[ leftChild ].Item1 < heap[ smallest ].Item1)
                {
                    smallest = leftChild;
                }

                if (rightChild < heap.Count && heap[ rightChild ].Item1 < heap[ smallest ].Item1)
                {
                    smallest = rightChild;
                }

                if (smallest != index)
                {
                    Swap( index, smallest );
                    index = smallest;
                }
                else
                {
                    break;
                }
            }
        }

        private void Swap ( int i, int j )
        {
            (heap[ j ], heap[ i ]) = (heap[ i ], heap[ j ]);
        }
    }

}