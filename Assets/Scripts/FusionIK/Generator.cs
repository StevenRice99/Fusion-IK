using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace FusionIK
{
    /// <summary>
    /// Generate training data for Fusion-IK.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class Generator : Controller
    {
        [Tooltip("The total number of entries to generate for training.")]
        [Min(1)]
        [SerializeField]
        private int generatedTotal = 10000;
        
        [Tooltip("The time the algorithm is allowed to run for.")]
        [Min(1)]
        [SerializeField]
        private long milliseconds = 1000;

        /// <summary>
        /// How much training data has been generated.
        /// </summary>
        private int _generatedCount = -1;

        /// <summary>
        /// The path to write data to.
        /// </summary>
        private string _path;

        private void Start()
        {
            // Spawn the robot.
            GameObject go = Instantiate(robotPrefab, Vector3.zero, Quaternion.identity);
            go.name = robotPrefab.name;

            Robot robot = go.GetComponent<Robot>();
            if (robot == null)
            {
                Destroy(gameObject);
            }

            // Ensure in Bio IK mode.
            robot.mode = Robot.SolverMode.BioIk;
            
            SetResult(new [] { robot }, milliseconds);
            
            // Don't need visuals during this process.
            NoVisuals();
            
            Load(robot);
        }

        private void Update()
        {
            // If already generated required amount, exit.
            if (_generatedCount >= generatedTotal)
            {
                Debug.Log("Finished generation.");
#if UNITY_EDITOR
                EditorApplication.ExitPlaymode();
#else
                    Application.Quit();
#endif
                return;
            }
            
            // If no previous starting values, load last known values.
            starting ??= R.Middle;
            
            // Randomly move the robot.
            R.Snap(R.RandomJoints());
            Robot.PhysicsStep();

            // Get the position and rotation to reach.
            (Vector3 position, Quaternion rotation) target = results[0].robot.EndTransform;
            
            // Reset the robot back to its starting position.
            R.Snap(starting);
            Robot.PhysicsStep();

            // Get the best result to reach the target.
            Solver.Run(ref target.position, ref target.rotation, ref results[0]);
            
            // If failed to reach, don't use this data.
            if (!results[0].Success)
            {
                R.Snap(starting);
                Robot.PhysicsStep();
                return;
            }
            
            // Snap to the results.
            R.Snap(results[0].Floats);
            Robot.PhysicsStep();

            // If reached, add the result, update the last pose, and set the start of the next generation to the result.
            float[] inputs = R.PrepareInputs(R.EndTransform.position, R.EndTransform.rotation, starting);
            float[] inputsMinimal = R.PrepareInputs(R.EndTransform.position, R.EndTransform.rotation);
            float[] outputs = R.NetScaledJoints(results[0].Floats).ToArray();
            float[] outputsMinimal = R.NetScaledJoints(results[0].FloatsMiddle).ToArray();

            string path = Path.Combine(_path, "Standard.csv");
            string data = string.Empty;
            
            // Write header if new file.
            if (!File.Exists(path))
            {
                for (int i = 0; i < inputs.Length; i++)
                {
                    data += $"I{i + 1},";
                }
                
                for (int i = 0; i < outputs.Length; i++)
                {
                    data += $"O{i + 1}";
                    if (i < outputs.Length - 1)
                    {
                        data += ",";
                    }
                }
            
                File.WriteAllText(path, data);
            }

            // Write data.
            data = "\n";
            for (int j = 0; j < inputs.Length; j++)
            {
                data += $"{inputs[j]},";
            }
            for (int j = 0; j < outputs.Length; j++)
            {
                data += $"{outputs[j]}";
                if (j < outputs.Length - 1)
                {
                    data += ",";
                }
            }
            
            File.AppendAllText(path, data);
            
            string pathMinimal = Path.Combine(_path, "Minimal.csv");
            string dataMinimal = string.Empty;
            
            // Write header if new file.
            if (!File.Exists(pathMinimal))
            {
                for (int i = 0; i < inputsMinimal.Length; i++)
                {
                    dataMinimal += $"I{i + 1},";
                }
                
                for (int i = 0; i < outputsMinimal.Length; i++)
                {
                    dataMinimal += $"O{i + 1}";
                    if (i < outputsMinimal.Length - 1)
                    {
                        dataMinimal += ",";
                    }
                }
            
                File.WriteAllText(pathMinimal, dataMinimal);
            }

            // Write data.
            dataMinimal = "\n";
            for (int j = 0; j < inputsMinimal.Length; j++)
            {
                dataMinimal += $"{inputsMinimal[j]},";
            }
            for (int j = 0; j < outputsMinimal.Length; j++)
            {
                dataMinimal += $"{outputsMinimal[j]}";
                if (j < outputsMinimal.Length - 1)
                {
                    dataMinimal += ",";
                }
            }
            
            File.AppendAllText(pathMinimal, dataMinimal);
            
            Debug.Log($"{R.Properties.name} | Generated {++_generatedCount} of {generatedTotal}.");
            
            // Update the pose to start at.
            starting = results[0].Floats;
        }

        private void Load(Robot robot)
        {
            // Ensure folder exists.
            _path = DirectoryPath(new[] { "Training", robot.Properties.name });
            if (_path == null)
            {
#if UNITY_EDITOR
                EditorApplication.ExitPlaymode();
#else
                Application.Quit();
#endif
                return;
            }

            string path = Path.Combine(_path, "Standard.csv");
            
            // Read total from file in case it exceeds amount.
            _generatedCount = CountLines(path);

            if (_generatedCount <= 0)
            {
                return;
            }

            // Attempt to load the last pose.
            string[] lines = File.ReadLines(path).ToArray();
            if (lines.Length <= 1)
            {
                return;
            }

            // Count the joints.
            string[] strings = lines[0].Split(',');
            int joints = strings.Count(s => s.Contains("I")) - 6;
            if (joints <= 0)
            {
                return;
            }
            
            // Create the joints.
            strings = lines[^1].Split(',').Skip(joints + 6).ToArray();
            starting = new(joints);
            for (int i = 0; i < joints; i++)
            {
                starting.Add(float.Parse(strings[i]));
            }
        }
    }
}