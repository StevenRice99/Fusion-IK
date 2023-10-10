using System.Collections.Generic;
using System.IO;
using System.Linq;
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
        protected Details[] results;

        /// <summary>
        /// The last pose moved to.
        /// </summary>
        protected List<float> lastPose;

        /// <summary>
        /// Setup the results storing capability.
        /// </summary>
        /// <param name="robots">The robots to store results for.</param>
        /// <param name="maximum">The maximum milliseconds to run for.</param>
        protected void SetResult(Robot[] robots, long maximum)
        {
            results = new Details[robots.Length];
            for (int i = 0; i < results.Length; i++)
            {
                results[i] = new(robots[i], maximum);
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
        
        /// <summary>
        /// Ensure a directory exists.
        /// </summary>
        /// <param name="directories">The names of the directories.</param>
        /// <returns>The path to the directory if it exists or was made, null otherwise.</returns>
        protected static string DirectoryPath(string[] directories)
        {
            DirectoryInfo full = Directory.GetParent(Application.dataPath);
            if (full == null)
            {
                Debug.LogError($"Directory {Application.dataPath} does not exist, this should not be possible!");
#if UNITY_EDITOR
                EditorApplication.ExitPlaymode();
#else
                Application.Quit();
#endif
                return null;
            }

            string path = full.FullName;

            foreach (string directory in directories)
            {
                path = Path.Combine(path, directory);
                if (Directory.Exists(path))
                {
                    continue;
                }

                DirectoryInfo result = Directory.CreateDirectory(path);
                if (result.Exists)
                {
                    continue;
                }

                Debug.LogError($"Cannot find or create directory {path}.");
#if UNITY_EDITOR
                EditorApplication.ExitPlaymode();
#else
                    Application.Quit();
#endif
                return null;
            }

            return path;
        }

        /// <summary>
        /// Count the number of lines in a file.
        /// </summary>
        /// <param name="path">The file path.</param>
        /// <returns>The number of lines in the file less one for the header or zero if the file does not exist.</returns>
        protected static int CountLines(string path)
        {
            return !File.Exists(path) ? 0 : File.ReadLines(path).Count() - 1;
        }
    }
}