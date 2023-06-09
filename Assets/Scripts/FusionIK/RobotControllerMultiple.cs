﻿using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Random = UnityEngine.Random;

namespace FusionIK
{
    /// <summary>
    /// Control robots of every movement type.
    /// </summary>
    [DisallowMultipleComponent]
    public class RobotControllerMultiple : RobotController
    {
        /// <summary>
        /// All robots to control.
        /// </summary>
        protected Robot[] robots;

        /// <summary>
        /// Easy getter for a single robot.
        /// </summary>
        protected Robot Robot => robots[^1];

        protected override void Awake()
        {
            base.Awake();

            // Create a robot for every movement type.
            int types = Enum.GetNames(typeof(Robot.SolverMode)).Length;
            List<Robot> robotsLists = new(types);
            for (int i = 0; i < types; i++)
            {
                GameObject go = Instantiate(robotPrefab, Vector3.zero, Quaternion.identity);
                Robot r = go.GetComponent<Robot>();
                if (r == null)
                {
                    continue;
                }

                r.mode = (Robot.SolverMode) i;

                // Can only create robots other than Bio IK if there is a network to run inference.
                if (r.mode != Robot.SolverMode.BioIk && !r.Properties.NetworksCheck)
                {
                    Destroy(r.gameObject);
                    continue;
                }
            
                go.name = $"{r.Properties.name} {Robot.Name(r.mode)}";
                robotsLists.Add(r);
            }

            robots = robotsLists.ToArray();
        }

        /// <summary>
        /// Get the results of all robots randomly moving.
        /// </summary>
        /// <param name="starting">The starting joint values.</param>
        /// <param name="position">The position to reach.</param>
        /// <param name="rotation">The rotation to reach.</param>
        /// <param name="maxGenerations">The maximum number of generations to run each algorithm for.</param>
        /// <returns>The results of all robots.</returns>
        protected Result[] RandomMoveResults(List<float> starting, out Vector3 position, out Quaternion rotation, int maxGenerations)
        {
            return RandomMoveResults(starting, out position, out rotation, new[] {maxGenerations});
        }

        /// <summary>
        /// Get the results of all robots randomly moving for multiple generation values.
        /// </summary>
        /// <param name="starting">The starting joint values.</param>
        /// <param name="position">The position to reach.</param>
        /// <param name="rotation">The rotation to reach.</param>
        /// <param name="maxGenerations">All the maximum number of generations to run each algorithm for.</param>
        /// <returns>The results of all robots.</returns>
        protected Result[] RandomMoveResults(List<float> starting, out Vector3 position, out Quaternion rotation, int[] maxGenerations)
        {
            // Move to robot to a random position.
            Robot.SnapRadians(Robot.RandomJoints());
            Robot.PhysicsStep();

            // Get the position and rotation of the robot to be those to reach.
            (Vector3 position, Quaternion rotation) target = Robot.EndTransform;
            position = target.position;
            rotation = target.rotation;

            // Generate the seed for random number generation.
            uint seed = (uint) Random.Range(1, int.MaxValue);

            // Test at every generation value.
            List<Result> results = new(robots.Length);
            for (int i = 0; i < maxGenerations.Length; i++)
            {
                // Move every robot to the starting position.
                for (int j = 0; j < robots.Length; j++)
                {
                    robots[j].SnapRadians(starting);
                }
                Robot.PhysicsStep();
                
                // Test every robot.
                for (int j = 0; j < robots.Length; j++)
                {
                    // Only test the network once as it is the same regardless.
                    if (i > 0 && robots[j].mode == Robot.SolverMode.Network)
                    {
                        continue;
                    }

                    // Can only test the network if zero max generations.
                    if (maxGenerations[i] == 0 && robots[j].mode != Robot.SolverMode.Network)
                    {
                        continue;
                    }
                    
                    // Can only test Greedy and Exhaustive Fusion IK versions if more than one generation.
                    if (maxGenerations[i] == 1 && robots[j].mode > Robot.SolverMode.FusionIk)
                    {
                        continue;
                    }
                    
                    results.Add(EvaluateRobot(robots[j], target.position, target.rotation, maxGenerations[i], seed));
                }
            }

            return results.ToArray();
        }

        /// <summary>
        /// Determine which robot was the best.
        /// </summary>
        /// <param name="results">The results to check.</param>
        /// <param name="ordered">The ordered results based off how they performed.</param>
        /// <returns>The robot which did the best.</returns>
        protected static Robot Best(Result[] results, out Result[] ordered)
        {
            // Get the robots that reached ordered by their move time, then generations, then mode.
            Result[] reached = results.Where(x => x.success).OrderBy(x => x.time).ThenBy(x => x.generations).ThenBy(x => x.robot.mode).ToArray();
            
            // Get the robots that did not reached ordered by their distance, then angle, then mode.
            Result[] notReached = results.Where(x => !x.success).OrderBy(x => x.distance).ThenBy(x => x.angle).ThenBy(x => x.robot.mode).ToArray();
            
            // Combine both and return the first robot.
            ordered = reached.Concat(notReached).ToArray();
            return ordered[0].robot;
        }

        /// <summary>
        /// Get the result of a robot.
        /// </summary>
        /// <param name="r">The robot to test.</param>
        /// <param name="position">The position to reach.</param>
        /// <param name="rotation">The rotation to reach.</param>
        /// <param name="maxGenerations">All the maximum number of generations to run each algorithm for.</param>
        /// <param name="seed">The seed for random number generation.</param>
        /// <returns>The results of the robot trying to reach the target.</returns>
        private static Result EvaluateRobot(Robot r, Vector3 position, Quaternion rotation, int maxGenerations, uint seed)
        {
            r.Snap(position, rotation, maxGenerations, out bool reached, out float moveTime, out int generations, seed);
            Robot.PhysicsStep();

            return new(r.mode == Robot.SolverMode.Network ? 0 : maxGenerations, r, reached, moveTime, generations, Robot.PositionAccuracy(position, r.EndTransform.position), r.RotationAccuracy(rotation, r.EndTransform.rotation));
        }
    }
}