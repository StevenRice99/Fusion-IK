using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace FusionIK
{
    /// <summary>
    /// Store a result of a robot moving to a target.
    /// </summary>
    public class Result
    {
        /// <summary>
        /// If the algorithm should end.
        /// </summary>
        public bool Done => _stopwatch.ElapsedMilliseconds >= milliseconds[^1];
        
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
        /// Convert joints for use with articulation bodies.
        /// </summary>
        public List<float> Floats => Joints.Select(t => (float) t).ToList();
        
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
        public double[] Joints { get; private set; }

        /// <summary>
        /// Used to time algorithms.
        /// </summary>
        private readonly Stopwatch _stopwatch = new();

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
        /// Reset the stopwatch.
        /// </summary>
        public void Reset()
        {
            _stopwatch.Reset();
            
            for (int i = 0; i < milliseconds.Length; i++)
            {
                success[i] = false;
                time[i] = 0;
                fitness[i] = double.MaxValue;
            }

            Joints ??= new double[robot.Ghost.dof];
            List<float> j = robot.GetJoints();
            for (int i = 0; i < Joints.Length; i++)
            {
                Joints[i] = j[i];
            }
        }

        /// <summary>
        /// Start the stopwatch.
        /// </summary>
        public void Start()
        {
            _stopwatch.Start();
        }

        /// <summary>
        /// Stop the stopwatch.
        /// </summary>
        public void Stop()
        {
            _stopwatch.Stop();
        }

        /// <summary>
        /// Set values for the results.
        /// </summary>
        /// <param name="s">If the move was successful.</param>
        /// <param name="t">The time it took for the joints to reach their destinations.</param>
        /// <param name="f">The fitness score of the result.</param>
        /// <param name="j">The joints of the robot.</param>
        public void Set(bool s, double t, double f, double[] j = null)
        {
            if (Success)
            {
                if (!s || t >= Time)
                {
                    return;
                }
            }
            else
            {
                if (f >= Fitness)
                {
                    return;
                }
            }

            if (s)
            {
                f = 0;
            }
            
            bool wasRunning = false;
            if (_stopwatch.IsRunning)
            {
                _stopwatch.Stop();
                wasRunning = true;
            }
            
            for (int i = 0; i < milliseconds.Length; i++)
            {
                if (i != milliseconds.Length - 1 && _stopwatch.ElapsedMilliseconds > milliseconds[i])
                {
                    continue;
                }
                
                success[i] = s;
                time[i] = t;
                fitness[i] = f;
            }

            if (j != null)
            {
                for (int i = 0; i < Joints.Length; i++)
                {
                    Joints[i] = j[i];
                }
            }

            if (wasRunning)
            {
                _stopwatch.Start();
            }
        }
    }
}