using UnityEditor;
using UnityEngine;

namespace FusionIK
{
    /// <summary>
    /// Control a robot.
    /// </summary>
    [DisallowMultipleComponent]
    public abstract class Controller : MonoBehaviour
    {
        [Tooltip("The robot to control.")]
        [SerializeField]
        protected GameObject robotPrefab;

        /// <summary>
        /// Quick access to a robot for controlling.
        /// </summary>
        protected Robot R => results[^1].robot;

        /// <summary>
        /// Store move data.
        /// </summary>
        protected Result[] results;

        /// <summary>
        /// Setup the results storing capability.
        /// </summary>
        /// <param name="robots">The robots to store results for.</param>
        /// <param name="milliseconds">The milliseconds to test against.</param>
        protected void SetResult(Robot[] robots, long[] milliseconds)
        {
            results = new Result[robots.Length];
            for (int i = 0; i < results.Length; i++)
            {
                results[i] = new(robots[i], robots[i].mode == Robot.SolverMode.Network ? new long[] { 0 } : milliseconds);
            }
        }

        protected virtual void Awake()
        {
            if (robotPrefab == null)
            {
#if UNITY_EDITOR
                EditorApplication.ExitPlaymode();
#else
                Application.Quit();
#endif
            }
        }

        /// <summary>
        /// For scenes that require no visuals, disable them, even if it is only a marginal speed up.
        /// </summary>
        protected static void NoVisuals()
        {
            MeshRenderer[] meshRenderers = FindObjectsOfType<MeshRenderer>();
            for (int i = 0; i < meshRenderers.Length; i++)
            {
                Destroy(meshRenderers[i]);
            }

            MeshFilter[] meshFilters = FindObjectsOfType<MeshFilter>();
            for (int i = 0; i < meshFilters.Length; i++)
            {
                if (!meshFilters[i].gameObject.GetComponent<MeshCollider>())
                {
                    Destroy(meshFilters[i]);
                }
            }
        }
    }
}