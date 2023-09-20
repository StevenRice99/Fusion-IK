using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using UnityEngine;

namespace FusionIK
{
    /// <summary>
    /// Store settings and results of a robot moving to a target.
    /// </summary>
    public class Details
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
        /// <param name="increment">The milliseconds to increment by.</param>
        /// <param name="maximum">The maximum milliseconds to run for.</param>
        public Details(Robot robot, long increment, long maximum)
        {
            this.robot = robot;
            if (increment > maximum)
            {
                increment = maximum;
            }

            List<long> timeouts = new();
            for (long i = increment; i <= maximum; i += increment)
            {
                timeouts.Add(i);
            }
            
            milliseconds = timeouts.ToArray();
            success = new bool[milliseconds.Length];
            time = new double[milliseconds.Length];
            fitness = new double[milliseconds.Length];
            for (int i = 0; i < milliseconds.Length; i++)
            {
                milliseconds[i] = milliseconds[i];
                success[i] = false;
                time[i] = 0;
                fitness[i] = double.MaxValue;
            }
        }

        /// <summary>
        /// Reset the stopwatch.
        /// <param name="position">The position to move the robot to.</param>
        /// <param name="rotation">The rotation to move the robot to.</param>
        /// </summary>
        public void Reset(Vector3 position, Quaternion rotation)
        {
            _stopwatch.Reset();

            Joints ??= new double[robot.Virtual.dof];
            List<float> j = robot.GetJoints();
            for (int i = 0; i < Joints.Length; i++)
            {
                Joints[i] = j[i];
            }

            robot.Virtual.SetTargetPosition(position);
            robot.Virtual.SetTargetRotation(rotation);
            bool s = robot.Virtual.CheckConvergence(Joints, position, rotation);
            double f = s ? 0 : robot.Virtual.ComputeLoss(j);
            
            for (int i = 0; i < milliseconds.Length; i++)
            {
                success[i] = s;
                time[i] = 0;
                fitness[i] = f;
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