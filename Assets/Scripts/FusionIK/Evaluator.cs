using System;
using UnityEngine;

namespace FusionIK
{
    /// <summary>
    /// Evaluate all robot solving methods against each other.
    /// </summary>
    [DisallowMultipleComponent]
    public class Evaluator : ControllerMultiple
    {
        [Tooltip("The number of generations Bio IK is allowed to run for.")]
        [SerializeField]
        protected int[] generations = Array.Empty<int>();
        
        private void Start()
        {
            // Don't need visuals during this process.
            NoVisuals();
        }

        private void Update()
        {
            // Get all results.
            Result[] results = RandomMoveResults(Robot.Properties.LastPose ?? Robot.GetJoints(), out Vector3 _, out Quaternion _, generations);

            // Export results.
            Robot.Properties.AddTestingData(results);

            // Start at the best result for the next test.
            Robot.Properties.SetLastPose(Best(results, out _).GetJoints());
        }
    }
}