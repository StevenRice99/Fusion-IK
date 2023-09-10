using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;

namespace FusionIK
{
    /// <summary>
    /// Generate training data for Fusion-IK.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class Generator : Controller
    {
        [Tooltip("The time the algorithm is allowed to run for.")]
        [Min(1)]
        [SerializeField]
        private long milliseconds = 100;
        
        // The joints to start at for the next attempt.
        private List<float> _starting;

        private void Start()
        {
            // Spawn the robot.
            GameObject go = Instantiate(robotPrefab, Vector3.zero, Quaternion.identity);
            go.name = robotPrefab.name;

            Robot robot = go.GetComponent<Robot>();
            if (robot == null)
            {
                Destroy(gameObject);
            }

            // Start in Bio IK mode.
            robot.mode = Robot.SolverMode.BioIk;
            
            SetResult(new [] { robot }, new [] { milliseconds });
            
            // Don't need visuals during this process.
            NoVisuals();
        }

        private void Update()
        {
            // If no previous starting values, load last known values.
            if (_starting == null)
            {
                List<float> lastOutput = R.Properties.LastPose ?? Load();
                _starting = lastOutput ?? R.GetJoints();
            }
            
            // Randomly move the robot.
            R.Snap(R.RandomJoints());
            Robot.PhysicsStep();

            // Get the position and rotation to reach.
            (Vector3 position, Quaternion rotation) target = results[0].robot.EndTransform;
            
            // Reset the robot back to its starting position.
            R.Snap(_starting);
            Robot.PhysicsStep();

            // Get the best result to reach the target.
            Robot.Solve(target.position, target.rotation, ref results[0]);
            
            // If failed to reach, don't use this data.
            if (!results[0].Success)
            {
                R.Snap(_starting);
                Robot.PhysicsStep();
                return;
            }
            
            // Snap to the results.
            R.Snap(results[0].joints);
            Robot.PhysicsStep();

            // If reached, add the result, update the last pose, and set the start of the next generation to the result.
            R.Properties.AddTrainingData(R.PrepareInputs(R.EndTransform.position, R.EndTransform.rotation, _starting), R.NetScaledJoints(results[0].joints).ToArray());
            R.Properties.SetLastPose(_starting);
            _starting = results[0].joints;
        }

        /// <summary>
        /// Attempt to restore any previous joints from the CSV data.
        /// </summary>
        /// <returns></returns>
        private List<float> Load()
        {
            string path = Properties.DirectoryPath(new[] { "Testing" });
            if (path == null)
            {
                return null;
            }
            
            path = Path.Combine(path, $"{R.Properties.Name}.csv");
            if (!File.Exists(path))
            {
                return null;
            }

            string[] lines = File.ReadLines(path).ToArray();

            if (lines.Length <= 1)
            {
                return null;
            }

            string[] strings = lines[0].Split(',');
            int joints = strings.Count(s => s.Contains("I")) - 7;
            if (joints <= 0)
            {
                return null;
            }
            
            strings = lines[^1].Split(',').Skip(joints + 7).ToArray();
            List<float> lastPose = new(joints);
            for (int i = 0; i < joints; i++)
            {
                lastPose.Add(float.Parse(strings[i]));
            }

            // The CSV results are relative to joint values so scale them back to joint values.
            R.Properties.SetLastPose(R.ResultsScaled(lastPose));
            return R.Properties.LastPose;
        }
    }
}