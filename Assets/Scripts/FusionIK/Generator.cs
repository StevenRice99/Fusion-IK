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
        private int generatedTotal = 250000;
        
        [Tooltip("The time the algorithm is allowed to run for.")]
        [Min(1)]
        [SerializeField]
        private long milliseconds = 500;

        /// <summary>
        /// How much training data has been generated.
        /// </summary>
        private int _generatedCount = -1;

        /// <summary>
        /// The path to write data to.
        /// </summary>
        private string _path;

        /// <summary>
        /// If minimal mode is being used.
        /// </summary>
        private bool _minimal;

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
                if (!_minimal)
                {
                    _minimal = true;
                    Load(R);
                }
                else
                {
                    Debug.Log("Finished generation.");
#if UNITY_EDITOR
                    EditorApplication.ExitPlaymode();
#else
                    Application.Quit();
#endif
                    return;
                }
            }
            
            // If no previous starting values, load last known values.
            starting = _minimal ? R.Middle : starting ??= R.GetJoints();
            
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
            float[] inputs = R.PrepareInputs(R.EndTransform.position, R.EndTransform.rotation, _minimal ? null : starting);
            float[] outputs = R.NetScaledJoints(results[0].Floats).ToArray();

            string s = string.Empty;
            
            // Write header if new file.
            if (!File.Exists(_path))
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
            
                File.WriteAllText(_path, s);
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
            
            File.AppendAllText(_path, s);
            
            string doing = _minimal ? "Minimal" : "Normal";
            Debug.Log($"{R.Properties.name} | {doing} | Generated {++_generatedCount} of {generatedTotal}.");
            
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

            string doing = _minimal ? "Minimal" : "Normal";
            _path = Path.Combine(_path, $"{doing}.csv");
            
            // Read total from file in case it exceeds amount.
            _generatedCount = CountLines(_path);

            if (_minimal)
            {
                return;
            }

            // Attempt to load the last pose.
            string[] lines = File.ReadLines(_path).ToArray();
            if (lines.Length <= 1)
            {
                return;
            }

            // Count the joints.
            string[] strings = lines[0].Split(',');
            int joints = strings.Count(s => s.Contains("I")) - 7;
            if (joints <= 0)
            {
                return;
            }
            
            // Create the joints.
            strings = lines[^1].Split(',').Skip(joints + 7).ToArray();
            starting = new(joints);
            for (int i = 0; i < joints; i++)
            {
                starting.Add(float.Parse(strings[i]));
            }
        }
    }
}