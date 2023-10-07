using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace FusionIK
{
    /// <summary>
    /// Control robots of every movement type.
    /// </summary>
    [DisallowMultipleComponent]
    public class ControllerMultiple : Controller
    {
        protected Robot[] CreateRobots()
        {
            // Create a robot for every movement type.
            List<Robot> robots = new();

            Robot r = CreateRobot(Robot.SolverMode.BioIk);
            if (r != null)
            {
                robots.Add(r);
                if (r.Properties.NetworksValid)
                {
                    for (Robot.SolverMode mode = Robot.SolverMode.Network; mode <= Robot.SolverMode.IterativeFusionIk; mode++)
                    {
                        r = CreateRobot(mode);
                        if (r != null)
                        {
                            robots.Add(r);
                        }
                    }
                }
            }

            return robots.ToArray();
        }

        private Robot CreateRobot(Robot.SolverMode solverMode)
        {
            GameObject go = Instantiate(robotPrefab, Vector3.zero, Quaternion.identity);
            Robot r = go.GetComponent<Robot>();
            if (r == null)
            {
                Destroy(go);
                return null;
            }

            r.mode = solverMode;
            go.name = $"{r.Properties.name} {Robot.Name(r.mode)}";
            
            return r;
        }

        /// <summary>
        /// Get the results of all robots randomly moving for multiple generation values.
        /// </summary>
        /// <param name="starting">The starting joint values.</param>
        /// <param name="position">The position to reach.</param>
        /// <param name="rotation">The rotation to reach.</param>
        protected void RandomMoveResults(List<float> starting, out Vector3 position, out Quaternion rotation)
        {
            // Move to robot to a random position.
            R.Snap(R.RandomJoints());
            Robot.PhysicsStep();

            // Get the position and rotation of the robot to be those to reach.
            (Vector3 position, Quaternion rotation) target = R.EndTransform;
            position = target.position;
            rotation = target.rotation;

            MoveResults(starting, position, rotation);
        }

        /// <summary>
        /// Get the results of all robots moving for multiple generation values.
        /// </summary>
        /// <param name="starting">The starting joint values.</param>
        /// <param name="position">The position to reach.</param>
        /// <param name="rotation">The rotation to reach.</param>
        protected void MoveResults(List<float> starting, Vector3 position, Quaternion rotation)
        {
            for (int i = 0; i < results.Length; i++)
            {
                results[i].robot.Snap(starting);
            }
            
            Robot.PhysicsStep();

            for (int i = 0; i < results.Length; i++)
            {
                Solver.Run(ref position, ref rotation, ref results[i]);
            }
        }

        /// <summary>
        /// Determine which robot was the best.
        /// </summary>
        /// <param name="results">The results to check.</param>
        /// <param name="ordered">The ordered results based off how they performed.</param>
        /// <returns>The robot which did the best.</returns>
        protected static Details Best(Details[] results, out Details[] ordered)
        {
            // Get the robots that reached ordered by their move time, then solutions, then mode.
            Details[] reached = results.Where(x => x.Success).OrderBy(x => x.Time).ThenBy(x => x.robot.mode).ToArray();
            
            // Get the robots that did not reached ordered by their fitness, then distance, then angle, then mode.
            Details[] notReached = results.Where(x => !x.Success).OrderBy(x => x.Fitness).ThenBy(x => x.robot.mode).ToArray();
            
            // Combine both and return the first robot.
            ordered = reached.Concat(notReached).ToArray();
            return ordered[0];
        }
    }
}