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
        
        // The robot to generate data for.
        private Robot _robot;

        private void Start()
        {
            // Spawn the robot.
            GameObject go = Instantiate(robotPrefab, Vector3.zero, Quaternion.identity);
            go.name = robotPrefab.name;

            _robot = go.GetComponent<Robot>();
            if (_robot == null)
            {
                Destroy(gameObject);
            }

            // Start in Bio IK mode.
            _robot.mode = Robot.SolverMode.BioIk;
            
            // Don't need visuals during this process.
            NoVisuals();
        }

        private void Update()
        {
            // If no previous starting values, load last known values.
            if (_starting == null)
            {
                List<float> lastOutput = _robot.Properties.LastPose ?? Load();
                _starting = lastOutput ?? _robot.GetJoints();
            }
            
            // Randomly move the robot.
            _robot.Snap(_robot.RandomJoints());
            Robot.PhysicsStep();

            // Get the position and rotation to reach.
            (Vector3 position, Quaternion rotation) target = _robot.EndTransform;
            
            // Reset the robot back to its starting position.
            _robot.Snap(_starting);
            Robot.PhysicsStep();

            // Get the best result to reach the target.
            List<float> results = _robot.Solve(target.position, target.rotation, milliseconds, out bool reached, out double _, out double _);
            
            // If failed to reach, don't use this data.
            if (!reached)
            {
                _robot.Snap(_starting);
                Robot.PhysicsStep();
                return;
            }
            
            // Snap to the results.
            _robot.Snap(results);
            Robot.PhysicsStep();

            // If reached, add the result, update the last pose, and set the start of the next generation to the result.
            _robot.Properties.AddTrainingData(_robot.PrepareInputs(_robot.EndTransform.position, _robot.EndTransform.rotation, _starting), _robot.NetScaledJoints(results).ToArray());
            _robot.Properties.SetLastPose(_starting);
            _starting = _robot.GetJoints();
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
            
            path = Path.Combine(path, $"{_robot.Properties.Name}.csv");
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
            _robot.Properties.SetLastPose(_robot.ResultsScaled(lastPose));
            return _robot.Properties.LastPose;
        }
    }
}