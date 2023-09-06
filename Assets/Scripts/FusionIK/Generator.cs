using System.Collections.Generic;
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
            // Randomly move the robot.
            _robot.Snap(_robot.RandomJoints());
            Robot.PhysicsStep();

            // Get the position and rotation to reach.
            (Vector3 position, Quaternion rotation) target = _robot.EndTransform;

            // Snap to the starting middle pose.
            _robot.SnapMiddle();
            Robot.PhysicsStep();
            
            // Get the best result to reach the target.
            List<float> results = _robot.Solve(target.position, target.rotation, milliseconds, out bool reached, out double _, out double _);
            
            // If failed to reach, don't use this data.
            if (!reached)
            {
                return;
            }
            
            // Snap to the results.
            _robot.Snap(results);
            Robot.PhysicsStep();

            // If reached, add the result, update the last pose, and set the start of the next generation to the result.
            _robot.Properties.AddTrainingData(_robot.PrepareInputs(_robot.EndTransform.position, _robot.EndTransform.rotation), _robot.NetScaledJoints(results).ToArray(), _robot);
        }
    }
}