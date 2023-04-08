using System;
using UnityEngine;

namespace FusionIK
{
    /// <summary>
    /// Evaluate all robot solving methods against each other.
    /// </summary>
    [DisallowMultipleComponent]
    public class RobotEvaluator : RobotControllerMultiple
    {
        [Tooltip("The maximum number of generations Bio IK is allowed to run for.")]
        [SerializeField]
        protected int[] maxGenerations = Array.Empty<int>();
        
        private void Start()
        {
            // Don't need visuals during this process.
            NoVisuals();
        }

        private void Update()
        {
            // Get all results.
            Result[] results = RandomMoveResults(Robot.Properties.LastPose ?? Robot.GetJoints(), out Vector3 _, out Quaternion _, maxGenerations);

            // Export results.
            Robot.Properties.AddResultsData(results);

            // Start at the best result for the next test.
            Robot.Properties.SetLastPose(Best(results, out _).GetJoints());
        }
    }
}