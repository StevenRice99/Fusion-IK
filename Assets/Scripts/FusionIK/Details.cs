using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;

namespace FusionIK
{
    /// <summary>
    /// Store settings and results of a robot moving to a target.
    /// </summary>
    public class Details
    {
        /// <summary>
        /// One rotation in radians.
        /// </summary>
        public const double C = 2.0 * math.PI_DBL;
        
        /// <summary>
        /// If the algorithm should end.
        /// </summary>
        public bool Done => _stopwatch.ElapsedMilliseconds >= milliseconds;
        
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
        public readonly long milliseconds;
        
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
        /// The position to solve for.
        /// </summary>
        private Vector3 _position;

        /// <summary>
        /// The rotation to solve for.
        /// </summary>
        private Quaternion _rotation;

        /// <summary>
        /// The starting joints.
        /// </summary>
        private double[] _starting;
        
        /// <summary>
        /// Used to time algorithms.
        /// </summary>
        private readonly Stopwatch _stopwatch = new();

        /// <summary>
        /// Store a move result.
        /// </summary>
        /// <param name="robot">The robot that did the move.</param>
        /// <param name="milliseconds">The maximum milliseconds to run for.</param>
        public Details(Robot robot, long milliseconds)
        {
            this.robot = robot;
            
            this.milliseconds = milliseconds;
            success = new bool[this.milliseconds + 1];
            time = new double[success.Length];
            fitness = new double[success.Length];
            for (int i = 0; i < success.Length; i++)
            {
                success[i] = false;
                time[i] = double.MaxValue;
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
            _starting ??= new double[Joints.Length];
            List<float> j = robot.GetJoints();
            for (int i = 0; i < Joints.Length; i++)
            {
                _starting[i] = Joints[i] = j[i];
            }

            _position = position;
            _rotation = rotation;
            robot.Virtual.SetTargetPosition(_position);
            robot.Virtual.SetTargetRotation(_rotation);
            bool s = robot.Virtual.CheckConvergence(Joints, _position, _rotation);
            double f = s ? 0 : robot.Virtual.ComputeLoss(j);
            
            for (int i = 0; i < success.Length; i++)
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
        /// Set values for the results.
        /// </summary>
        /// <param name="joints">The joints of the robot.</param>
        public void Set(double[] joints)
        {
            bool wasRunning = _stopwatch.IsRunning;
            _stopwatch.Stop();
            
            Improve(joints);
            
            // Check if successful.
            bool s = robot.Virtual.CheckConvergence(joints, _position, _rotation);
            double t;
            double f;
            
            if (s)
            {
                // Get the time of the successful move.
                t = robot.CalculateTime(_starting, joints);
                
                // If there is a previously successful move and the new time is worse, discard it.
                if (Success && t >= Time)
                {
                    if (wasRunning)
                    {
                        _stopwatch.Start();
                    }
                    
                    return;
                }
                
                // Zero the fitness score for successful moves since it is not needed.
                f = 0;
            }
            else
            {
                // If there have been successful moves already, discard this unsuccessful move.
                if (Success)
                {
                    if (wasRunning)
                    {
                        _stopwatch.Start();
                    }
                    
                    return;
                }
                
                // Get the fitness score of unsuccessful moves.
                f = robot.Virtual.ComputeLoss(joints);
                
                // If the existing fitness is better, discard it.
                if (f >= Fitness)
                {
                    if (wasRunning)
                    {
                        _stopwatch.Start();
                    }
                    
                    return;
                }
                
                // Calculate the time.
                t = robot.CalculateTime(_starting, joints);
            }
            
            for (long i = _stopwatch.ElapsedMilliseconds <= milliseconds ? _stopwatch.ElapsedMilliseconds : milliseconds; i <= milliseconds; i++)
            {
                success[i] = s;
                time[i] = t;
                fitness[i] = f;
            }

            for (int i = 0; i < Joints.Length; i++)
            {
                Joints[i] = joints[i];
            }

            if (wasRunning)
            {
                _stopwatch.Start();
            }
        }


        /// <summary>
        /// Run the robot's network.
        /// </summary>
        /// <param name="floats">The joint values to run from as a list of floats.</param>
        /// <returns>The solved joint values.</returns>
        public double[] RunNetwork(List<float> floats)
        {
            bool wasRunning = _stopwatch.IsRunning;
            _stopwatch.Stop();

            double[] joints;
            if (floats == null)
            {
                joints = null;
            }
            else
            {
                joints = new double[floats.Count];
                for (int i = 0; i < joints.Length; i++)
                {
                    joints[i] = floats[i];
                }
            }

            joints = RunNetwork(joints);

            if (wasRunning)
            {
                _stopwatch.Start();
            }

            return joints;
        }

        /// <summary>
        /// Run the robot's network.
        /// </summary>
        /// <param name="joints">The joint values to run from.</param>
        /// <returns>The solved joint values.</returns>
        public double[] RunNetwork(double[] joints = null)
        {
            bool wasRunning = _stopwatch.IsRunning;
            _stopwatch.Stop();
            List<float> floats;

            if (joints != null)
            {
                floats = new(joints.Length);
                for (int i = 0; i < joints.Length; i++)
                {
                    floats.Add((float) joints[i]);
                }
            }
            else
            {
                floats = null;
            }
            
            _stopwatch.Start();
            floats = robot.RunNetwork(_position, _rotation, floats);
            _stopwatch.Stop();

            joints ??= new double[floats.Count];

            for (int i = 0; i < joints.Length; i++)
            {
                joints[i] = floats[i];
            }
            
            Improve(joints);

            if (wasRunning)
            {
                _stopwatch.Start();
            }

            return joints;
        }

        /// <summary>
        /// Minimize the joint movements, ensuring no extra full rotations.
        /// </summary>
        /// <param name="joints">The joints to improve.</param>
        private void Improve(double[] joints)
        {
            for (int i = 0; i < joints.Length; i++)
            {
                if (joints[i] > _starting[i])
                {
                    while (joints[i] - C >= _starting[i])
                    {
                        joints[i] -= C;
                    }

                    if (joints[i] - C < robot.Limits[i].lower)
                    {
                        continue;
                    }

                    double radians = joints[i] - C;
                    if (_starting[i] - radians < joints[i] - _starting[i])
                    {
                        joints[i] = radians;
                    }
                }
                else
                {
                    while (joints[i] + C <= _starting[i])
                    {
                        joints[i] += C;
                    }
                    
                    if (joints[i] + C > robot.Limits[i].upper)
                    {
                        continue;
                    }

                    double radians = joints[i] + C;
                    if (radians - _starting[i] < _starting[i] - joints[i])
                    {
                        joints[i] = radians;
                    }
                }
            }
        }
    }
}