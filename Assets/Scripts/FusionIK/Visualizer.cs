using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Rendering;

namespace FusionIK
{
    /// <summary>
    /// Visualize the movements of the algorithms.
    /// </summary>
    [DisallowMultipleComponent]
    public class Visualizer : ControllerMultiple
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
        
        [Tooltip("The time the algorithm is allowed to run for.")]
        [Min(0)]
        [SerializeField]
        private long milliseconds = 100;
        
        /// <summary>
        /// Normal robot materials.
        /// </summary>
        private readonly List<Material> _normalMaterials = new();
        
        /// <summary>
        /// Transparent robot materials.
        /// </summary>
        private readonly List<Material> _transparentMaterials = new();

        /// <summary>
        /// Robot mesh renderers.
        /// </summary>
        private readonly List<MeshRenderer[]> _meshRenderers = new();

        /// <summary>
        /// Target position.
        /// </summary>
        private Vector3? _targetPosition;

        /// <summary>
        /// Target rotation.
        /// </summary>
        private Quaternion? _targetRotation;

        /// <summary>
        /// Robots ordered by performance.
        /// </summary>
        private Result[] _ordered;

        /// <summary>
        /// Robot movement paths.
        /// </summary>
        private List<Vector3>[] _paths;

        /// <summary>
        /// Which robots are visible.
        /// </summary>
        private bool[] _visible;

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
            _visible = new bool[robots.Length];
            _paths = new List<Vector3>[robots.Length];
            for (int i = 0; i < robots.Length; i++)
            {
                _paths[i] = new();
            }

            if (robots.Length <= 1)
            {
                return;
            }

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
            if (!robot.IsMoving || !_visible[index])
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
            return lastPose ?? Robot.GetJoints();
        }

        private void MovePerform(List<float> starting, Result[] results)
        {
            milliseconds = math.max(milliseconds, 1);
            
            // Get the best robot and order the rest.
            Robot best = Best(results, out _ordered);
            
            bool bestNetwork = false;
            bool bestFusionIk = false;
            
            // Only show the best robot for network and Fusion-IK moves to reduce visual clutter.
            foreach (Result result in _ordered)
            {
                int index = Array.IndexOf(robots, result.robot);
                
                switch (robots[index].mode)
                {
                    case Robot.SolverMode.Network:
                        _visible[index] = !bestNetwork;
                        bestNetwork = true;
                        break;
                    case Robot.SolverMode.FusionIk:
                        _visible[index] = !bestFusionIk;
                        bestFusionIk = true;
                        break;
                    case Robot.SolverMode.BioIk:
                    default:
                        _visible[index] = true;
                        break;
                }
            }

            if (robots.Length > 1)
            {
                // Apply materials to all robots.
                for (int i = 0; i < robots.Length; i++)
                {
                    // Regular if best, transparent otherwise.
                    Material material = robots[i] == best ? _normalMaterials[i] : _transparentMaterials[i];
                    
                    for (int j = 0; j < _meshRenderers[i].Length; j++)
                    {
                        _meshRenderers[i][j].enabled = _visible[i];
                        
                        Material[] materials = _meshRenderers[i][j].materials;
                        for (int k = 0; k < materials.Length; k++)
                        {
                            materials[k] = material;
                        }

                        _meshRenderers[i][j].materials = materials;
                    }
                }
            }

            // Store the best result.
            lastPose = best.GetJoints();

            // Store the endings and snap back to the start.
            List<float>[] endings = new List<float>[robots.Length];
            for (int i = 0; i < robots.Length; i++)
            {
                endings[i] = robots[i].GetJoints();
                robots[i].Snap(starting);
            }
            Robot.PhysicsStep();

            // Call to move robots in real time.
            for (int i = 0; i < robots.Length; i++)
            {
                robots[i].Move(endings[i]);
            }
        }

        private void Move()
        {
            if (_targetPosition == null || _targetRotation == null)
            {
                return;
            }
            
            List<float> starting = GetStarting();
            MovePerform(starting, MoveResults(_targetPosition.Value, _targetRotation.Value, new [] {milliseconds}));
        }

        /// <summary>
        /// Move all robots to a random target.
        /// </summary>
        private void RandomMove()
        {
            List<float> starting = GetStarting();

            // Solve for random target.
            Result[] results = RandomMoveResults(out Vector3 position, out Quaternion rotation, new [] {milliseconds});
            _targetPosition = position;
            _targetRotation = rotation;

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
            
            if (data.Value.success)
            {
                success = "Success";
                description = $"{data.Value.time} Seconds from Middle";
            }
            else
            {
                success = "Failed";
                description = $"{data.Value.distance} Meters | {data.Value.angle} Degrees";
            }
            
            GUI.Label(new(145, y, 55, 20), success);
            GUI.Label(new(210, y, Screen.width - 220, 20), description);
        }

        private void OnGUI()
        {
            GUI.color = Color.white;
            
            // Display input to change the milliseconds.
            GUI.Label(new(10, 10, 100, 20), "Milliseconds");
            string s = milliseconds > 0 ? milliseconds.ToString() : string.Empty;
            s = GUI.TextField(new(10, 30, 100, 20), s, 5);
            s = new(s.Where(char.IsDigit).ToArray());
            if (string.IsNullOrWhiteSpace(s))
            {
                milliseconds = 1;
            }
            else
            {
                try
                {
                    int input = int.Parse(s);
                    milliseconds = input;
                }
                catch
                {
                    milliseconds = 0;
                }
            }

            // At the beginning, just call to move.
            if (_targetPosition == null || _targetRotation == null)
            {
                Robot.SnapMiddle();
                Robot.PhysicsStep();
                _targetPosition = Robot.EndTransform.position;
                _targetRotation = Robot.EndTransform.rotation;
                Move();
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

            // Cartesian jog constants.
            const int controlsWidth = 200;
            const int controlsHeight = 20;
            const int labelWidth = 90;
            
            // Cartesian jog labels.
            GUI.Label(new(Screen.width - controlsWidth - labelWidth - 10, 10, labelWidth, controlsHeight), "Position X");
            GUI.Label(new(Screen.width - controlsWidth - labelWidth - 10, controlsHeight + 10 * 2, labelWidth, controlsHeight), "Position Y");
            GUI.Label(new(Screen.width - controlsWidth - labelWidth - 10, controlsHeight * 2 + 10 * 3, labelWidth, controlsHeight), "Position Z");
            GUI.Label(new(Screen.width - controlsWidth - labelWidth - 10, controlsHeight * 3 + 10 * 4, labelWidth, controlsHeight), "Rotation X");
            GUI.Label(new(Screen.width - controlsWidth - labelWidth - 10, controlsHeight * 4 + 10 * 5, labelWidth, controlsHeight), "Rotation Y");
            GUI.Label(new(Screen.width - controlsWidth - labelWidth - 10, controlsHeight * 5 + 10 * 6, labelWidth, controlsHeight), "Rotation Z");

            // Cartesian jog position sliders.
            Transform robotTransform = Robot.transform;
            Vector3 robotPosition = robotTransform.position;
            float x = GUI.HorizontalSlider(new(Screen.width - controlsWidth - 10, 10, controlsWidth, controlsHeight), _targetPosition.Value.x, robotPosition.x - Robot.ChainLength, robotPosition.x + Robot.ChainLength);
            float y = GUI.HorizontalSlider(new(Screen.width - controlsWidth - 10, controlsHeight + 10 * 2, controlsWidth, controlsHeight), _targetPosition.Value.y, robotPosition.y - Robot.ChainLength, robotPosition.y + Robot.ChainLength);
            float z = GUI.HorizontalSlider(new(Screen.width - controlsWidth - 10, controlsHeight * 2 + 10 * 3, controlsWidth, controlsHeight), _targetPosition.Value.z, robotPosition.z - Robot.ChainLength, robotPosition.z + Robot.ChainLength);
            _targetPosition = new(x, y, z);

            // Cartesian jog rotation sliders.
            Vector3 euler = _targetRotation.Value.eulerAngles;
            x = GUI.HorizontalSlider(new(Screen.width - controlsWidth - 10, controlsHeight * 3 + 10 * 4, controlsWidth, controlsHeight), euler.x, 0, 360);
            y = GUI.HorizontalSlider(new(Screen.width - controlsWidth - 10, controlsHeight * 4 + 10 * 5, controlsWidth, controlsHeight), euler.y, 0, 360);
            z = GUI.HorizontalSlider(new(Screen.width - controlsWidth - 10, controlsHeight * 5 + 10 * 6, controlsWidth, controlsHeight), euler.z, 0, 360);
            _targetRotation = Quaternion.Euler(x, y, z);
            
            if (_ordered == null)
            {
                return;
            }

            // Display robot labels.
            float offset = robots.Length * 20 + 10;
            foreach (Result data in _ordered)
            {
                string title = Robot.Name(data.robot.mode);
                RobotLabel(Screen.height - offset, data, title, Robot.RobotColor(data.robot.mode));
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
            if (_targetPosition != null && _targetRotation != null)
            {
                DrawAxis(_targetPosition.Value, _targetRotation.Value);
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
                    if (i < 0 || !_visible[i])
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