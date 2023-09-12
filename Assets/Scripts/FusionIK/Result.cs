using System.Collections.Generic;

namespace FusionIK
{
    /// <summary>
    /// Store a result of a robot moving to a target.
    /// </summary>
    public class Result
    {
        /// <summary>
        /// If the move was successful.
        /// </summary>
        public bool Success => success[^1];
        
        /// <summary>
        /// The time it took for the joints to reach their destinations.
        /// </summary>
        public double Time => time[^1];

        /// <summary>
        /// The fitness score of the result.
        /// </summary>
        public double Fitness => fitness[^1];
        
        /// <summary>
        /// The robot that did the move.
        /// </summary>
        public readonly Robot robot;
        
        /// <summary>
        /// The time the algorithm is allowed to run for.
        /// </summary>
        public readonly long[] milliseconds;
        
        /// <summary>
        /// If the move was successful.
        /// </summary>
        public readonly bool[] success;

        /// <summary>
        /// The time it took for the joints to reach their destinations.
        /// </summary>
        public readonly double[] time;

        /// <summary>
        /// The fitness score of the result.
        /// </summary>
        public readonly double[] fitness;

        /// <summary>
        /// The joints of the best move.
        /// </summary>
        public List<float> joints;

        /// <summary>
        /// Store a move result.
        /// </summary>
        /// <param name="robot">The robot that did the move.</param>
        /// <param name="milliseconds">The time the algorithm is allowed to run for.</param>
        public Result(Robot robot, long[] milliseconds)
        {
            this.robot = robot;
            this.milliseconds = new long[milliseconds.Length];
            success = new bool[this.milliseconds.Length];
            time = new double[this.milliseconds.Length];
            fitness = new double[this.milliseconds.Length];
            for (int i = 0; i < this.milliseconds.Length; i++)
            {
                this.milliseconds[i] = milliseconds[i];
                success[i] = false;
                time[i] = 0;
                fitness[i] = double.MaxValue;
            }
        }

        /// <summary>
        /// Set values for the results.
        /// </summary>
        /// <param name="index">The milliseconds index the results are for and higher.</param>
        /// <param name="s">If the move was successful.</param>
        /// <param name="t">The time it took for the joints to reach their destinations.</param>
        /// <param name="f">The fitness score of the result.</param>
        public void Set(int index, bool s, double t, double f)
        {
            for (int i = index; i < milliseconds.Length; i++)
            {
                success[i] = s;
                time[i] = t;
                fitness[i] = f;
            }
        }
    }
}