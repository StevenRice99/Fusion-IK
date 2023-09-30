using UnityEngine;

namespace FusionIK
{
    /// <summary>
    /// Generate testing data for the algorithms.
    /// </summary>
    [DisallowMultipleComponent]
    public class Tester : ControllerMultiple
    {
        [Tooltip("The time the algorithms are allowed to run for.")]
        [SerializeField]
        protected long milliseconds = 5000;
        
        private void Start()
        {
            SetResult(CreateRobots(), milliseconds);
            
            // Don't need visuals during this process.
            NoVisuals();
        }

        private void Update()
        {
            // Get all results.
            RandomMoveResults(R.Properties.LastPose ?? R.GetJoints(), out Vector3 _, out Quaternion _);

            // Export results.
            R.Properties.AddTestingData(ref results);

            // Start at the best result for the next test.
            R.Properties.SetLastPose(Best(results, out _).Floats);
        }
    }
}