using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;

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
        
        /// <summary>
        /// The time the algorithm is allowed to run for.
        /// </summary>
        [Tooltip("The time the algorithm is allowed to run for.")]
        [Min(1)]
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
        private Details[] _ordered;

        /// <summary>
        /// Robot movement paths.
        /// </summary>
        private List<Vector3>[] _paths;

        /// <summary>
        /// The next scene to load.
        /// </summary>
        private int _next;

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

        /// <summary>
        /// Start is called on the frame when a script is enabled just before any of the Update methods are called the first time. This function can be a coroutine.
        /// </summary>
        private void Start()
        {
            // Get the next scene to load.
            _next = SceneManager.GetActiveScene().buildIndex + 1;
            if (_next >= SceneManager.sceneCountInBuildSettings)
            {
                _next = 0;
            }
            
            SetResult(CreateRobots(), milliseconds);
            
            // Create path visualization lists for every robot.
            _paths = new List<Vector3>[results.Length];
            for (int i = 0; i < results.Length; i++)
            {
                _paths[i] = new();
            }

            if (results.Length <= 1)
            {
                return;
            }

            // Apply for every robot.
            for (int i = 0; i < results.Length; i++)
            {
                // Get the color for the robot.
                Color color = results[i].robot.ToColor();
            
                // Create the normal material for the robot when it is the best.
                Material material = Instantiate(R.Properties.Normal);
                material.color = new(color.r, color.g, color.b, material.color.a);
                _normalMaterials.Add(material);
            
                // Create the transparent material for the robot when it is not the best.
                material = Instantiate(R.Properties.Transparent);
                material.color = new(color.r, color.g, color.b, material.color.a);
                _transparentMaterials.Add(material);
            
                // Set the materials for all the mesh renderers and store the renderers.
                _meshRenderers.Add(results[i].robot.GetComponentsInChildren<MeshRenderer>());
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

        /// <summary>
        /// Update is called every frame, if the MonoBehaviour is enabled.
        /// </summary>
        private void Update()
        {
            // Create a new movement when the space key is pressed.
            if (Keyboard.current.spaceKey.wasPressedThisFrame)
            {
                Move();
            }
        }

        /// <summary>
        /// Frame-rate independent MonoBehaviour.FixedUpdate message for physics calculations.
        /// </summary>
        private void FixedUpdate()
        {
            // Add to paths as needed.
            for (int i = 0; i < results.Length; i++)
            {
                AddToPath(results[i].robot, i);
            }
        }

        /// <summary>
        /// Get the starting position of the robots.
        /// </summary>
        private void GetStarting()
        {
            // Clear old paths.
            foreach (List<Vector3> path in _paths)
            {
                path.Clear();
            }
            
            // Start at the last position.
            starting ??= R.Middle;
        }

        /// <summary>
        /// Perform a move of the robots.
        /// </summary>
        private void MovePerform()
        {
            // Get the best robot and order the rest.
            Details best = Best(results, out _ordered);

            if (results.Length > 1)
            {
                // Apply materials to all robots.
                for (int i = 0; i < results.Length; i++)
                {
                    // Regular if best, transparent otherwise.
                    Material material = results[i] == best ? _normalMaterials[i] : _transparentMaterials[i];
                    
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

            // Store the endings and snap back to the start.
            for (int i = 0; i < results.Length; i++)
            {
                results[i].robot.Snap(starting);
            }
            Robot.PhysicsStep();

            // Call to move robots in real time.
            for (int i = 0; i < results.Length; i++)
            {
                results[i].robot.Move(results[i].Floats);
            }

            // Store the best result.
            starting = best.Floats;
        }

        /// <summary>
        /// Move all robots to a random target.
        /// </summary>
        private void Move()
        {
            GetStarting();

            // Solve for random target.
            RandomMoveResults(out Vector3 position, out Quaternion rotation);
            _targetPosition = position;
            _targetRotation = rotation;

            MovePerform();
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
        /// <param name="color">The color to make it.</param>
        private void RobotLabel(float y, Details data, Color color)
        {
            if (data == null)
            {
                return;
            }
            
            GUI.color = color;
            int offset;
            if (results.Length > 1)
            {
                offset = 200;
                GUI.Label(new(10, y, 190, 20), data.robot.ToString());
            }
            else
            {
                offset = 10;
            }

            string success;
            string description;
            
            if (data.Success)
            {
                success = "Success";
                description = $"{data.Time} Seconds";
            }
            else
            {
                success = "Failed";
                description = $"{data.Fitness} Fitness Score";
            }
            
            GUI.Label(new(offset, y, 55, 20), success);
            GUI.Label(new(offset + 65, y, Screen.width - offset - 10, 20), description);
        }

        /// <summary>
        /// OnGUI is called for rendering and handling GUI events.
        /// </summary>
        private void OnGUI()
        {
            GUI.color = Color.white;
            
            // Display input to change the milliseconds.
            GUI.Label(new(10, 10, 80, 20), "Milliseconds");
            string s = milliseconds.ToString();
            s = GUI.TextField(new(10, 30, 80, 20), s, 5);
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
                    milliseconds = input <= 0 ? 1 : input;
                }
                catch
                {
                    milliseconds = 1;
                }
            }

            if (milliseconds != results[0].milliseconds)
            {
                Robot[] robots = new Robot[results.Length];
                for (int i = 0; i < robots.Length; i++)
                {
                    robots[i] = results[i].robot;
                }

                SetResult(robots, milliseconds);
            }

            // At the beginning, just set up the target.
            if (_targetPosition == null || _targetRotation == null)
            {
                GetStarting();
                
                for (int i = 0; i < results.Length; i++)
                {
                    results[i].robot.Snap(starting);
                }
            
                Robot.PhysicsStep();
                
                _targetPosition = R.EndTransform.position;
                _targetRotation = R.EndTransform.rotation;
            }

            // Button to move the robot randomly.
            if (GUI.Button(new(10, 55, 80, 20), "Move"))
            {
                Move();
            }
            
            // View the next scene if pressed.
            if (SceneManager.sceneCountInBuildSettings > 1 && GUI.Button(new(Screen.width - 120, 10, 110, 20), "Test Next Robot"))
            {
                SceneManager.LoadScene(_next);
            }
            
            if (_ordered == null)
            {
                return;
            }

            // Display robot labels.
            float offset = results.Length * 20 + 10;
            foreach (Details data in _ordered)
            {
                RobotLabel(Screen.height - offset, data, data.robot.ToColor());
                offset -= 20;
            }
        }

        /// <summary>
        /// OnRenderObject is called after camera has rendered the Scene.
        /// </summary>
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
                foreach (Details data in _ordered.Reverse())
                {
                    Details details = results.FirstOrDefault(r => r.robot == data.robot);
                    if (details == null)
                    {
                        continue;
                    }

                    int i = Array.IndexOf(results, details);
                    if (i < 0)
                    {
                        continue;
                    }
                
                    DrawAxis(results[i].robot);
                
                    GL.Color(results[i].robot.ToColor());
                
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