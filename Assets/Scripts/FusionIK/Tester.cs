using System;
using System.Collections.Generic;
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
        protected long[] milliseconds = Array.Empty<long>();
        
        private void Start()
        {
            // Don't need visuals during this process.
            NoVisuals();
        }

        private void Update()
        {
            // Export results.
            Robot.Properties.AddTestingData(RandomMoveResults(out Vector3 _, out Quaternion _, milliseconds));
        }
    }
}