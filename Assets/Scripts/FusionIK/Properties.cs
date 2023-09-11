using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Unity.Barracuda;
using UnityEditor;
using UnityEngine;

namespace FusionIK
{
    /// <summary>
    /// Handle properties for a robot.
    /// </summary>
    [CreateAssetMenu(fileName = "Properties", menuName = "Fusion-IK/Properties", order = 0)]
    public class Properties : ScriptableObject
    {
        /// <summary>
        /// How accurate in meters the robot can repeat a movement.
        /// </summary>
        public float Repeatability => repeatability;

        /// <summary>
        /// The population size of each generation during Bio IK evolution.
        /// </summary>
        public int Population => population;

        /// <summary>
        /// The number of elites in each generation during Bio IK evolution.
        /// </summary>
        public int Elites => elites;

        /// <summary>
        /// Material to apply to the best robot during visualization
        /// </summary>
        public Material Normal => normal;

        /// <summary>
        /// Material to apply to the non-best robots during visualization.
        /// </summary>
        public Material Transparent => transparent;

        [Header("Movement")]
        [Tooltip("How accurate in meters the robot can repeat a movement.")]
        [Min(0)]
        [SerializeField]
        private float repeatability = 8e-5f;

        [Tooltip("Network to control the robot.")]
        public NNModel[] networks;

        [Header("Bio IK")]
        [Tooltip("The population size of each generation during Bio IK evolution.")]
        [Min(1)]
        [SerializeField]
        private int population = 120;
        
        [Tooltip("The number of elites in each generation during Bio IK evolution.")]
        [Min(1)]
        [SerializeField]
        private int elites = 3;

        [Header("Datasets")]
        [Tooltip("The total number of entries to generate for training.")]
        [Min(1)]
        [SerializeField]
        private int trainingTotal = 250000;
        
        [Tooltip("The total number of results to generate for testing.")]
        [Min(1)]
        [SerializeField]
        private int testingTotal = 100000;

        [Header("Materials")]
        [Tooltip("Material to apply to the best robot during visualization.")]
        [SerializeField]
        private Material normal;
        
        [Tooltip("Material to apply to the non-best robots during visualization.")]
        [SerializeField]
        private Material transparent;

        /// <summary>
        /// How much training data has been generated.
        /// </summary>
        [NonSerialized]
        private int _generatedCount = -1;

        /// <summary>
        /// How many results have been run.
        /// </summary>
        [NonSerialized]
        private int _resultsCount = -1;

        /// <summary>
        /// Formatted name for the robot.
        /// </summary>
        public string Name => name.Replace(" ", "-");

        /// <summary>
        /// Check if networks are valid.
        /// </summary>
        public bool NetworksValid => networks is {Length: > 0} && networks.All(network => network != null);
        
        /// <summary>
        /// A network to run inference on.
        /// </summary>
        /// <param name="index">The network index.</param>
        /// <returns>The joint network at a given index that is desired.</returns>
        public Model CompiledNetwork(int index) => networks is {Length: > 0} && index >= 0 && index < networks.Length && networks[index] != null ? ModelLoader.Load(networks[index]) : null;
        
        /// <summary>
        /// The last pose the robot was in.
        /// </summary>
        [field: NonSerialized]
        public List<float> LastPose { get; private set; }
        
        /// <summary>
        /// Ensure a directory exists.
        /// </summary>
        /// <param name="directories">The names of the directories.</param>
        /// <returns>The path to the directory if it exists or was made, null otherwise.</returns>
        public static string DirectoryPath(string[] directories)
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
        /// Set the last pose the robot was in.
        /// </summary>
        /// <param name="joints">Joint values.</param>
        public void SetLastPose(List<float> joints)
        {
            LastPose = joints;
        }

        /// <summary>
        /// Write training data to CSV.
        /// </summary>
        /// <param name="inputs">The inputs to write.</param>
        /// <param name="outputs">The outputs to write.</param>
        public void AddTrainingData(float[] inputs, float[] outputs)
        {
            // If already generated required amount, exit.
            if (_generatedCount >= trainingTotal)
            {
                Debug.Log("Finished generation.");
#if UNITY_EDITOR
                EditorApplication.ExitPlaymode();
#else
                Application.Quit();
#endif
                return;
            }
            
            // Ensure folder exists.
            string path = DirectoryPath(new[] { "Training" });
            if (path == null)
            {
#if UNITY_EDITOR
                EditorApplication.ExitPlaymode();
#else
                Application.Quit();
#endif
                return;
            }

            path = Path.Combine(path, $"{Name}.csv");
            
            // Read total from file in case it exceeds amount.
            if (_generatedCount < 0)
            {
                _generatedCount = CountLines(path);
                if (_generatedCount >= trainingTotal)
                {
                    return;
                }
            }

            string s = string.Empty;
            
            // Write header if new file.
            if (!File.Exists(path))
            {
                for (int i = 0; i < inputs.Length; i++)
                {
                    s += $"I{i + 1},";
                }
                
                for (int i = 0; i < outputs.Length; i++)
                {
                    s += $"O{i + 1}";
                    if (i < outputs.Length - 1)
                    {
                        s += ",";
                    }
                }
            
                File.WriteAllText(path, s);
            }

            // Write data.
            s = "\n";
            for (int j = 0; j < inputs.Length; j++)
            {
                s += $"{inputs[j]},";
            }
            for (int j = 0; j < outputs.Length; j++)
            {
                s += $"{outputs[j]}";
                if (j < outputs.Length - 1)
                {
                    s += ",";
                }
            }
                
            File.AppendAllText(path, s);
            
            Debug.Log($"{Name} | Generated {++_generatedCount} of {trainingTotal}.");
        }
        
        /// <summary>
        /// Write testing data to CSV.
        /// </summary>
        /// <param name="results"></param>
        public void AddTestingData(ref Result[] results)
        {
            // If already evaluated required amount, exit.
            if (_resultsCount >= testingTotal)
            {
                Debug.Log("Finished evaluation.");
#if UNITY_EDITOR
                EditorApplication.ExitPlaymode();
#else
                Application.Quit();
#endif
                return;
            }
            
            // Ensure folder exists.
            string path = DirectoryPath(new[] {"Testing", Name});
            if (path == null)
            {
#if UNITY_EDITOR
                EditorApplication.ExitPlaymode();
#else
                Application.Quit();
#endif
                return;
            }

            // Add all results.
            foreach (Result result in results)
            {
                for (int i = 0; i < result.milliseconds.Length; i++)
                {
                    string file = Path.Combine(path, $"{result.milliseconds[i]:0000} {Robot.Name(result.robot.mode).Replace(" ", "-")}.csv");

                    // If file exceeds what is needed, return.
                    if (_resultsCount < 0)
                    {
                        _resultsCount = CountLines(file);
                        if (_resultsCount >= testingTotal)
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
            
            Debug.Log($"{Name} | Evaluated {++_resultsCount} of {testingTotal}.");
        }

        /// <summary>
        /// Count the number of lines in a file.
        /// </summary>
        /// <param name="path">The file path.</param>
        /// <returns>The number of lines in the file less one for the header or zero if the file does not exist.</returns>
        private static int CountLines(string path)
        {
            return !File.Exists(path) ? 0 : File.ReadLines(path).Count() - 1;
        }

        private void OnValidate()
        {
            // Cannot have more elites than the population.
            if (elites > population)
            {
                elites = population;
            }
        }
    }
}