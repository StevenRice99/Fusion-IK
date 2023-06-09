﻿using System;
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
    [CreateAssetMenu(fileName = "Robot Properties", menuName = "Rapid-Sim/Robot Properties", order = 0)]
    public class RobotProperties : ScriptableObject
    {
        /// <summary>
        /// A network to run inference on.
        /// </summary>
        /// <param name="index">The joint index for the network.</param>
        /// <returns>The network for the joint.</returns>
        public Model CompiledNetwork(int index) => networkModels[index] != null ? ModelLoader.Load(networkModels[index]) : null;
        
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
        /// The number of steps during the Bio IK minimise process.
        /// </summary>
        public int Steps => steps;

        /// <summary>
        /// Material to apply to the best robot during visualization
        /// </summary>
        public Material Normal => normal;

        /// <summary>
        /// Material to apply to the non-best robots during visualization.
        /// </summary>
        public Material Transparent => transparent;

        /// <summary>
        /// The last pose the robot was in.
        /// </summary>
        [field: NonSerialized]
        public List<float> LastPose { get; private set; }

        [Header("Movement")]
        [Tooltip("How accurate in meters the robot can repeat a movement.")]
        [Min(0)]
        [SerializeField]
        private float repeatability = 8e-5f;

        [Tooltip("Model for the network to control the robot.")]
        [SerializeField]
        private NNModel[] networkModels;

        [Header("Bio IK")]
        [Tooltip("The population size of each generation during Bio IK evolution.")]
        [Min(1)]
        [SerializeField]
        private int population = 120;
        
        [Tooltip("The number of elites in each generation during Bio IK evolution.")]
        [Min(1)]
        [SerializeField]
        private int elites = 3;

        [Tooltip("The number of steps during the Bio IK minimise process.")]
        [Min(1)]
        [SerializeField]
        private int steps = 100;

        [Header("Datasets")]
        [Tooltip("The total number of entries to generate.")]
        [Min(1)]
        [SerializeField]
        private int generationTotal = 250000;
        
        [Tooltip("The total number of results to test.")]
        [Min(1)]
        [SerializeField]
        private int resultsTotal = 100000;

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
        /// Check if all networks are present.
        /// </summary>
        public bool NetworksCheck => networkModels.All(model => model != null);

        /// <summary>
        /// Set the last pose the robot was in.
        /// </summary>
        /// <param name="joints">Joint values.</param>
        public void SetLastPose(List<float> joints)
        {
            LastPose = joints;
        }

        /// <summary>
        /// Write generated training data to CSV.
        /// </summary>
        /// <param name="inputs">The inputs to write.</param>
        /// <param name="outputs">The outputs to write.</param>
        public void AddGenerationData(float[] inputs, float[] outputs)
        {
            // If already generated required amount, exit.
            if (_generatedCount >= generationTotal)
            {
#if UNITY_EDITOR
                EditorApplication.ExitPlaymode();
#else
                Application.Quit();
#endif
                return;
            }
            
            // Ensure folder exists.
            string path = DirectoryPath("Datasets");
            if (path == null)
            {
                Debug.Log("Finished generation.");
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
                if (_generatedCount >= generationTotal)
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
            
            Debug.Log($"{name} - Generated {++_generatedCount} of {generationTotal}.");
        }
        
        /// <summary>
        /// Write evaluation data to CSV.
        /// </summary>
        /// <param name="results"></param>
        public void AddResultsData(Result[] results)
        {
            // If already evaluated required amount, exit.
            if (_resultsCount >= resultsTotal)
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
            string path = DirectoryPath("Evaluation");
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
                string file = Path.Combine(path, $"{Name} {Robot.Name(result.robot.mode).Replace(" ", "-")} {result.maxGenerations}.csv");

                // If file exceeds what is needed, return.
                if (_resultsCount < 0)
                {
                    _resultsCount = CountLines(file);
                    if (_resultsCount >= resultsTotal)
                    {
                        return;
                    }
                }

                if (!File.Exists(file))
                {
                    File.WriteAllText(file, "Success,Time,Generations,Distance,Angle");
                }
                
                File.AppendAllText(file, $"\n{result.success},{result.time},{result.generations},{result.distance},{result.angle}");
            }
            
            Debug.Log($"{name} - Evaluated {++_resultsCount} of {resultsTotal}.");
        }

        private void OnValidate()
        {
            // Cannot have more elites than the population.
            if (elites > population)
            {
                elites = population;
            }
        }

        /// <summary>
        /// Ensure a directory exists.
        /// </summary>
        /// <param name="directory">The name of the directory.</param>
        /// <returns>The path to the directory if it exists or was made, null otherwise.</returns>
        public string DirectoryPath(string directory)
        {
            DirectoryInfo full = Directory.GetParent(Application.dataPath);
            if (full == null)
            {
                Debug.LogError($"{name} - Directory {Application.dataPath} does not exist, this should not be possible!");
#if UNITY_EDITOR
                EditorApplication.ExitPlaymode();
#else
                Application.Quit();
#endif
                return null;
            }
            
            string path = Path.Combine(full.FullName, "IK-Trainer");
            if (!Directory.Exists(path))
            {
                DirectoryInfo result = Directory.CreateDirectory(path);
                if (!result.Exists)
                {
                    Debug.LogError($"{name} - Cannot find or create directory {path}.");
#if UNITY_EDITOR
                    EditorApplication.ExitPlaymode();
#else
                    Application.Quit();
#endif
                    return null;
                }
            }

            path = Path.Combine(path, directory);
            if (!Directory.Exists(path))
            {
                DirectoryInfo result = Directory.CreateDirectory(path);
                if (!result.Exists)
                {
                    Debug.LogError($"{name} - Cannot find or create directory {path}.");
#if UNITY_EDITOR
                    EditorApplication.ExitPlaymode();
#else
                    Application.Quit();
#endif
                    return null;
                }
            }

            return path;
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
    }
}