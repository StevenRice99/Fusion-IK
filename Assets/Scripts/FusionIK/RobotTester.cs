using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Rendering;

namespace FusionIK
{
    [DisallowMultipleComponent]
    public class RobotTester : RobotControllerMultiple
    {
        /// <summary>
        /// Cached shader value for use with line rendering.
        /// </summary>
        private static readonly int SrcBlend = Shader.PropertyToID("_SrcBlend");

        /// <summary>
        /// Cached shader value for use with line rendering.
        /// </summary>
        private static readonly int DstBlend = Shader.PropertyToID("_DstBlend");

        /// <summary>
        /// Cached shader value for use with line rendering.
        /// </summary>
        private static readonly int Cull = Shader.PropertyToID("_Cull");

        /// <summary>
        /// Cached shader value for use with line rendering.
        /// </summary>
        private static readonly int ZWrite = Shader.PropertyToID("_ZWrite");
        
        /// <summary>
        /// The auto-generated material for displaying lines.
        /// </summary>
        private static Material _lineMaterial;
        
        [Tooltip("The maximum number of generations Bio IK is allowed to run for.")]
        [Min(1)]
        [SerializeField]
        private int maxGenerations = 100;
        
        private readonly List<Material> _normalMaterials = new();
        
        private readonly List<Material> _transparentMaterials = new();

        private readonly List<MeshRenderer[]> _meshRenderers = new();

        private Vector3? _endPosition;

        private Quaternion? _endRotation;

        private Result[] _ordered;

        private List<Vector3>[] _paths;

        /// <summary>
        /// Setup the material for line rendering.
        /// </summary>
        private static void LineMaterial()
        {
            if (_lineMaterial)
            {
                return;
            }

            // Unity has a built-in shader that is useful for drawing simple colored things.
            _lineMaterial = new(Shader.Find("Hidden/Internal-Colored"))
            {
                hideFlags = HideFlags.HideAndDontSave
            };
            
            // Turn on alpha blending.
            _lineMaterial.SetInt(SrcBlend, (int)BlendMode.SrcAlpha);
            _lineMaterial.SetInt(DstBlend, (int)BlendMode.OneMinusSrcAlpha);
            
            // Turn backface culling off.
            _lineMaterial.SetInt(Cull, (int)CullMode.Off);
            
            // Turn off depth writes.
            _lineMaterial.SetInt(ZWrite, 0);
        }

        private void Start()
        {
            // Create path visualization lists for every robot.
            _paths = new List<Vector3>[robots.Length];
            for (int i = 0; i < robots.Length; i++)
            {
                _paths[i] = new();
            }
            
            SetMaterials();
        }

        /// <summary>
        /// Set the materials to be used for the visualization.
        /// </summary>
        private void SetMaterials()
        {
            // Apply for every robot.
            for (int i = 0; i < robots.Length; i++)
            {
                // Get the color for the robot.
                Color color = robots[i].RobotColor();
                
                // Create the normal material for the robot when it is the best.
                Material material = Instantiate(Robot.Properties.Normal);
                material.color = new(color.r, color.g, color.b, material.color.a);
                _normalMaterials.Add(material);
                
                // Create the transparent material for the robot when it is not the best.
                material = Instantiate(Robot.Properties.Transparent);
                material.color = new(color.r, color.g, color.b, material.color.a);
                _transparentMaterials.Add(material);
                
                // Set the materials for all the mesh renderers and store the renderers.
                _meshRenderers.Add(robots[i].GetComponentsInChildren<MeshRenderer>());
                for (int j = 0; j < _meshRenderers[i].Length; j++)
                {
                    Material[] materials = _meshRenderers[i][j].materials;
                    for (int k = 0; k < materials.Length; k++)
                    {
                        materials[k] = material;
                    }

                    _meshRenderers[i][j].materials = materials;
                }
            }
        }

        /// <summary>
        /// Add a point to the path for a robot.
        /// </summary>
        /// <param name="robot">The robot.</param>
        /// <param name="index">The path index of the robot.</param>
        private void AddToPath(Robot robot, int index)
        {
            // Only add new points from if it was moving.
            if (!robot.IsMoving)
            {
                return;
            }
            
            // Ensure no duplicates are added.
            Vector3 end = robot.EndTransform.position;
            if (!_paths[index].Contains(end))
            {
                _paths[index].Add(end);
            }
        }

        private void Update()
        {
            // Create a new movement when the space key is pressed.
            if (Keyboard.current.spaceKey.wasPressedThisFrame)
            {
                RandomMove();
            }
        }

        private void FixedUpdate()
        {
            // Add to paths as needed.
            for (int i = 0; i < robots.Length; i++)
            {
                AddToPath(robots[i], i);
            }
        }

        private List<float> GetStarting()
        {
            // Clear old paths.
            foreach (List<Vector3> path in _paths)
            {
                path.Clear();
            }
            
            // Start at the last position.
            return Robot.Properties.LastPose ?? Robot.GetJoints();
        }

        private void MovePerform(List<float> starting, Result[] results)
        {
            // Get the best robot and order the rest.
            Robot best = Best(results, out _ordered);

            // Apply materials to all robots.
            for (int i = 0; i < robots.Length; i++)
            {
                // Regular if best, transparent otherwise.
                Material material = robots[i] == best ? _normalMaterials[i] : _transparentMaterials[i];
                for (int j = 0; j < _meshRenderers[i].Length; j++)
                {
                    Material[] materials = _meshRenderers[i][j].materials;
                    for (int k = 0; k < materials.Length; k++)
                    {
                        materials[k] = material;
                    }

                    _meshRenderers[i][j].materials = materials;
                }
            }

            // Store the best result.
            Robot.Properties.SetLastPose(best.GetJoints());

            // Store the endings and snap back to the start.
            List<float>[] endings = new List<float>[robots.Length];
            for (int i = 0; i < robots.Length; i++)
            {
                endings[i] = robots[i].GetJoints();
                robots[i].SnapRadians(starting);
            }
            Robot.PhysicsStep();

            // Call to move robots in real time.
            for (int i = 0; i < robots.Length; i++)
            {
                robots[i].MoveRadians(endings[i]);
            }
        }

        private void Move()
        {
            List<float> starting = GetStarting();
            
            MovePerform(starting, MoveResults(starting, _endPosition.Value, _endRotation.Value, new [] {maxGenerations}));
        }

        /// <summary>
        /// Move all robots to a random target.
        /// </summary>
        private void RandomMove()
        {
            List<float> starting = GetStarting();

            // Solve for random target.
            Result[] results = RandomMoveResults(starting, out Vector3 position, out Quaternion rotation, maxGenerations);
            _endPosition = position;
            _endRotation = rotation;

            MovePerform(starting, results);
        }

        /// <summary>
        /// Draw an axis gizmos at a robot's tooling.
        /// </summary>
        /// <param name="robot">The robot to draw it for.</param>
        private static void DrawAxis(Robot robot)
        {
            (Vector3 position, Quaternion rotation) end = robot.EndTransform;
            DrawAxis(end.position, end.rotation);
        }

        /// <summary>
        /// Draw an axis at a given position and rotation.
        /// </summary>
        /// <param name="position">The position to draw it at.</param>
        /// <param name="rotation">The rotation to draw it at.</param>
        private static void DrawAxis(Vector3 position, Quaternion rotation)
        {
            const float length = 0.3f;
            
            GL.Color(Color.blue);
            GL.Vertex(position);
            GL.Vertex(position + rotation * (Vector3.forward * length));
            
            GL.Color(Color.green);
            GL.Vertex(position);
            GL.Vertex(position + rotation * (Vector3.up * length));
            
            GL.Color(Color.red);
            GL.Vertex(position);
            GL.Vertex(position + rotation * (Vector3.right * length));
        }

        /// <summary>
        /// Create a label for a robot.
        /// </summary>
        /// <param name="y">The Y position.</param>
        /// <param name="data">The data of its last move.</param>
        /// <param name="title">The title of the robot.</param>
        /// <param name="color">The color to make it.</param>
        private static void RobotLabel(float y, Result? data, string title, Color color)
        {
            if (data == null)
            {
                return;
            }
            
            GUI.color = color;
            GUI.Label(new(10, y, 135, 20), $"{title}");

            string success;
            string description;

            string generations = data.Value.generations > 0 ? $" | {data.Value.generations} Generations" : string.Empty;
            
            if (data.Value.success)
            {
                success = "Success";
                description = $"{data.Value.time} Seconds{generations}";
            }
            else
            {
                success = "Failed";
                description = $"{data.Value.distance} Meters | {data.Value.angle} Degrees{generations}";
            }
            
            GUI.Label(new(145, y, 55, 20), success);
            GUI.Label(new(210, y, Screen.width - 220, 20), description);
        }

        private void OnGUI()
        {
            // Display input to change the max number of generations.
            GUI.color = Color.white;
            
            GUI.Label(new(10, 10, 100, 20), "Max Generations");
            string s = maxGenerations.ToString();
            s = GUI.TextField(new(10, 30, 100, 20), s, 5);
            s = new(s.Where(char.IsDigit).ToArray());
            if (string.IsNullOrWhiteSpace(s))
            {
                maxGenerations = 1;
            }
            else
            {
                try
                {
                    int input = int.Parse(s);
                    maxGenerations = input <= 0 ? 1 : input;
                }
                catch
                {
                    maxGenerations = 1;
                }
            }

            if (_endPosition == null || _endRotation == null)
            {
                _endPosition = Robot.EndTransform.position;
                _endRotation = Robot.EndTransform.rotation;
            }

            // Button to move the robot randomly.
            if (GUI.Button(new(10, 55, 100, 20), "Random Move"))
            {
                RandomMove();
            }

            // Button to move the robot.
            if (GUI.Button(new(10, 75, 100, 20), "Move"))
            {
                Move();
            }

            const int controlsWidth = 200;
            const int controlsHeight = 20;
            const int labelWidth = 90;
            
            GUI.Label(new(Screen.width - controlsWidth - labelWidth - 10, 10, labelWidth, controlsHeight), "Position X");
            GUI.Label(new(Screen.width - controlsWidth - labelWidth - 10, controlsHeight + 10 * 2, labelWidth, controlsHeight), "Position Y");
            GUI.Label(new(Screen.width - controlsWidth - labelWidth - 10, controlsHeight * 2 + 10 * 3, labelWidth, controlsHeight), "Position Z");
            GUI.Label(new(Screen.width - controlsWidth - labelWidth - 10, controlsHeight * 3 + 10 * 4, labelWidth, controlsHeight), "Rotation X");
            GUI.Label(new(Screen.width - controlsWidth - labelWidth - 10, controlsHeight * 4 + 10 * 5, labelWidth, controlsHeight), "Rotation Y");
            GUI.Label(new(Screen.width - controlsWidth - labelWidth - 10, controlsHeight * 5 + 10 * 6, labelWidth, controlsHeight), "Rotation Z");

            Transform robotTransform = Robot.transform;
            Vector3 robotPosition = robotTransform.position;
            
            float x = GUI.HorizontalSlider(new(Screen.width - controlsWidth - 10, 10, controlsWidth, controlsHeight), _endPosition.Value.x, robotPosition.x - Robot.ChainLength, robotPosition.x + Robot.ChainLength);
            float y = GUI.HorizontalSlider(new(Screen.width - controlsWidth - 10, controlsHeight + 10 * 2, controlsWidth, controlsHeight), _endPosition.Value.y, robotPosition.y - Robot.ChainLength, robotPosition.y + Robot.ChainLength);
            float z = GUI.HorizontalSlider(new(Screen.width - controlsWidth - 10, controlsHeight * 2 + 10 * 3, controlsWidth, controlsHeight), _endPosition.Value.z, robotPosition.z - Robot.ChainLength, robotPosition.z + Robot.ChainLength);
            
            _endPosition = new(x, y, z);

            Vector3 euler = _endRotation.Value.eulerAngles;
            
            x = GUI.HorizontalSlider(new(Screen.width - controlsWidth - 10, controlsHeight * 3 + 10 * 4, controlsWidth, controlsHeight), euler.x, 0, 360);
            y = GUI.HorizontalSlider(new(Screen.width - controlsWidth - 10, controlsHeight * 4 + 10 * 5, controlsWidth, controlsHeight), euler.y, 0, 360);
            z = GUI.HorizontalSlider(new(Screen.width - controlsWidth - 10, controlsHeight * 5 + 10 * 6, controlsWidth, controlsHeight), euler.z, 0, 360);
            
            _endRotation = Quaternion.Euler(x, y, z);
            
            if (_ordered == null)
            {
                return;
            }

            // Display robot labels.
            float offset = robots.Length * 20 + 10;
            foreach (Result data in _ordered)
            {
                RobotLabel(Screen.height - offset, data, Robot.Name(data.robot.mode), Robot.RobotColor(data.robot.mode));
                offset -= 20;
            }
        }

        private void OnRenderObject()
        {
            LineMaterial();
            _lineMaterial.SetPass(0);

            GL.PushMatrix();
            GL.MultMatrix(Matrix4x4.identity);
            GL.Begin(GL.LINES);

            // Draw the end target.
            if (_endPosition != null && _endRotation != null)
            {
                DrawAxis(_endPosition.Value, _endRotation.Value);
            }

            // Draw all robot paths and their axis gizmos.
            if (_ordered != null)
            {
                foreach (Result data in _ordered.Reverse())
                {
                    Robot robot = robots.FirstOrDefault(r => r == data.robot);
                    if (robot == null)
                    {
                        continue;
                    }

                    int i = Array.IndexOf(robots, robot);
                    if (i < 0)
                    {
                        continue;
                    }
                
                    DrawAxis(robots[i]);
                
                    GL.Color(robots[i].RobotColor());
                
                    for (int j = 1; j < _paths[i].Count; j++)
                    {
                        GL.Vertex(_paths[i][j - 1]);
                        GL.Vertex(_paths[i][j]);
                    }
                }
            }

            GL.End();
            GL.PopMatrix();
        }
    }
}