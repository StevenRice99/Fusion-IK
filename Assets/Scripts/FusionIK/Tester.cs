using System.IO;
using UnityEditor;
using UnityEngine;

namespace FusionIK
{
    /// <summary>
    /// Generate testing data for the algorithms.
    /// </summary>
    [DisallowMultipleComponent]
    public class Tester : ControllerMultiple
    {
        /// <summary>
        /// The total number of results to generate for testing.
        /// </summary>
        [Tooltip("The total number of results to generate for testing.")]
        [Min(1)]
        [SerializeField]
        private int testingTotal = 10000;

        /// <summary>
        /// The time the algorithms are allowed to run for.
        /// </summary>
        [Tooltip("The time the algorithms are allowed to run for.")]
        [Min(1)]
        [SerializeField]
        protected long milliseconds = 1000;

        /// <summary>
        /// How many results have been run.
        /// </summary>
        private int _testingCount = -1;

        /// <summary>
        /// The path to write data to.
        /// </summary>
        private string _path;
        
        /// <summary>
        /// Start is called on the frame when a script is enabled just before any of the Update methods are called the first time. This function can be a coroutine.
        /// </summary>
        private void Start()
        {
            SetResult(CreateRobots(), milliseconds);
            
            // Ensure folder exists.
            _path = DirectoryPath(new[] {"Testing", R.Properties.name});
            if (_path == null)
            {
#if UNITY_EDITOR
                EditorApplication.ExitPlaymode();
#else
                Application.Quit();
#endif
                return;
            }

            foreach (Details result in results)
            {
                // Don't need folders for network-only robots as they simply write to a single CSV file.
                if (result.robot.mode == Robot.SolverMode.Network)
                {
                    continue;
                }
                
                string file = DirectoryPath(new[] {"Testing", R.Properties.name, result.robot.ToString()});
                if (file != null)
                {
                    continue;
                }
                
#if UNITY_EDITOR
                EditorApplication.ExitPlaymode();
#else
                Application.Quit();
#endif
                return;
            }
            
            // Don't need visuals during this process.
            NoVisuals();
        }

        /// <summary>
        /// Update is called every frame, if the MonoBehaviour is enabled.
        /// </summary>
        private void Update()
        {
            // If already evaluated required amount, exit.
            if (_testingCount >= testingTotal)
            {
                Debug.Log("Finished testing.");
#if UNITY_EDITOR
                EditorApplication.ExitPlaymode();
#else
                Application.Quit();
#endif
                return;
            }
            
            // Get all results.
            starting ??= R.Middle;
            RandomMoveResults(out Vector3 _, out Quaternion _);

            // Add all results.
            foreach (Details result in results)
            {
                for (long i = 0; i <= result.milliseconds; i++)
                {
                    string file;
                    if (result.robot.mode == Robot.SolverMode.Network)
                    {
                        if (i > 0)
                        {
                            continue;
                        }
                        
                        file = Path.Combine(_path, $"{result.robot}.csv");
                    }
                    else
                    {
                        file = Path.Combine(_path, result.robot.ToString(), $"{i}.csv");
                    }

                    // If file exceeds what is needed, return.
                    if (_testingCount < 0)
                    {
                        _testingCount = CountLines(file);
                        if (_testingCount >= testingTotal)
                        {
                            return;
                        }
                    }

                    if (!File.Exists(file))
                    {
                        File.WriteAllText(file, "Success,Time,Fitness");
                    }
                
                    File.AppendAllText(file, $"\n{result.success[i]},{result.time[i]},{result.fitness[i]}");
                }
            }
            
            Debug.Log($"{R.Properties.name} | Tested {++_testingCount} of {testingTotal}.");

            // Start at the best result for the next test.
            starting = Best(results, out _).Floats;
        }
    }
}