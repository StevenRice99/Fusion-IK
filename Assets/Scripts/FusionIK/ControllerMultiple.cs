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
        /// <summary>
        /// Create the robots.
        /// </summary>
        /// <returns>The robots that were created.</returns>
        protected Robot[] CreateRobots()
        {
            // Create a robot for every movement type.
            List<Robot> robots = new();

            Robot r = CreateRobot(Robot.SolverMode.BioIk);
            if (r != null)
            {
                robots.Add(r);
            }

            r = CreateRobot(Robot.SolverMode.FusionIk);
            if (r != null)
            {
                robots.Add(r);
            }

            r = CreateRobot(Robot.SolverMode.Network);
            if (r != null)
            {
                robots.Add(r);
            }

            r = CreateRobot(Robot.SolverMode.Network, minimal:true);
            if (r != null)
            {
                robots.Add(r);
            }

            r = CreateRobot(Robot.SolverMode.FusionIk, true);
            if (r != null)
            {
                robots.Add(r);
            }

            r = CreateRobot(Robot.SolverMode.FusionIk, iterative:true);
            if (r != null)
            {
                robots.Add(r);
            }

            r = CreateRobot(Robot.SolverMode.FusionIk, true, true);
            if (r != null)
            {
                robots.Add(r);
            }

            r = CreateRobot(Robot.SolverMode.FusionIk, minimal:true);
            if (r != null)
            {
                robots.Add(r);
            }

            return robots.ToArray();
        }

        /// <summary>
        /// Create a robot.
        /// </summary>
        /// <param name="solverMode">The mode to solve in.</param>
        /// <param name="exhaustive">If exhaustive Fusion IK should be used.</param>
        /// <param name="iterative">If iterative Fusion IK should be used.</param>
        /// <param name="minimal">If it is a minimal network.</param>
        /// <returns>The robot if it was created, false otherwise.</returns>
        private Robot CreateRobot(Robot.SolverMode solverMode, bool exhaustive = false, bool iterative = false, bool minimal = false)
        {
            if (solverMode == Robot.SolverMode.BioIk && minimal)
            {
                return null;
            }
            
            if (solverMode != Robot.SolverMode.FusionIk && (exhaustive || iterative))
            {
                return null;
            }
            
            if (minimal && (exhaustive || iterative))
            {
                return null;
            }
            
            GameObject go = Instantiate(robotPrefab, Vector3.zero, Quaternion.identity);
            Robot r = go.GetComponent<Robot>();
            if (r == null)
            {
                Destroy(go);
                return null;
            }

            if (solverMode != Robot.SolverMode.BioIk && ((!minimal && !r.Properties.StandardNetworkValid) || (minimal && !r.Properties.MinimalNetworkValid)))
            {
                Destroy(go);
                return null;
            }

            r.mode = solverMode;
            r.exhaustive = exhaustive;
            r.iterative = iterative;
            r.minimal = minimal;
            go.name = $"{r.Properties.name} {r}";
            
            return r;
        }

        /// <summary>
        /// Get the results of all robots randomly moving for multiple generation values.
        /// </summary>
        /// <param name="position">The position to reach.</param>
        /// <param name="rotation">The rotation to reach.</param>
        protected void RandomMoveResults(out Vector3 position, out Quaternion rotation)
        {
            // Move to robot to a random position.
            R.Snap(R.RandomJoints());
            Robot.PhysicsStep();

            // Get the position and rotation of the robot to be those to reach.
            (Vector3 position, Quaternion rotation) target = R.EndTransform;
            position = target.position;
            rotation = target.rotation;

            MoveResults(position, rotation);
        }

        /// <summary>
        /// Get the results of all robots moving for multiple generation values.
        /// </summary>
        /// <param name="position">The position to reach.</param>
        /// <param name="rotation">The rotation to reach.</param>
        protected void MoveResults(Vector3 position, Quaternion rotation)
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
            Details[] reached = results.Where(x => x.Success).OrderBy(x => x.Time).ThenBy(x => x.robot.mode).ThenByDescending(x => x.robot.minimal).ThenBy(x => x.robot.iterative).ThenBy(x => x.robot.exhaustive).ToArray();
            
            // Get the robots that did not reached ordered by their fitness, then move time, then mode.
            Details[] notReached = results.Where(x => !x.Success).OrderBy(x => x.Fitness).ThenBy(x => x.Time).ThenBy(x => x.robot.mode).ThenByDescending(x => x.robot.minimal).ThenBy(x => x.robot.iterative).ThenBy(x => x.robot.exhaustive).ToArray();
            
            // Combine both and return the first robot.
            ordered = reached.Concat(notReached).ToArray();
            return ordered[0];
        }
    }
}