using UnityEditor;
using UnityEngine;

namespace FusionIK
{
    /// <summary>
    /// Control a robot.
    /// </summary>
    [DisallowMultipleComponent]
    public abstract class RobotController : MonoBehaviour
    {
        [Tooltip("The robot to control.")]
        [SerializeField]
        protected GameObject robotPrefab;

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