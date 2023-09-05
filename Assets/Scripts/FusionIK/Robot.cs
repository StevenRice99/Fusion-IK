using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using FusionIK.Evolution;
using Unity.Barracuda;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;
using Debug = UnityEngine.Debug;
using Random = UnityEngine.Random;

namespace FusionIK
{
    /// <summary>
    /// The main robot class.
    /// </summary>
    [DisallowMultipleComponent]
    public class Robot : MonoBehaviour
    {
        /// <summary>
        /// The mode the robot is solving in.
        /// </summary>
        public enum SolverMode
        {
            Network,
            BioIk,
            FusionIk
        }

        /// <summary>
        /// The properties of the robot.
        /// </summary>
        public Properties Properties => properties;
        
        /// <summary>
        /// The ghost joints of the robot.
        /// </summary>
        public GhostJoint[] GhostJoints { get; private set; }
        
        /// <summary>
        /// Rescaling value used for Bio IK.
        /// </summary>
        public float Rescaling { get; private set; }

        /// <summary>
        /// If the robot is currently moving.
        /// </summary>
        public bool IsMoving { get; private set; }

        /// <summary>
        /// How long the robot is.
        /// </summary>
        public float ChainLength { get; private set; }

        /// <summary>
        /// Get the end position and rotation of the robot.
        /// </summary>
        public (Vector3 position, Quaternion rotation) EndTransform => (LastJoint.position, LastJoint.rotation);
        
        /// <summary>
        /// The last joint of the robot.
        /// </summary>
        private Transform LastJoint => _joints[^1].transform;

        [Tooltip("The robot properties to use.")]
        [SerializeField]
        private Properties properties;

        [Tooltip("Mode to perform solving in.")]
        public SolverMode mode = SolverMode.BioIk;

        [Tooltip("The index of neural networks to use for Fusion-IK")]
        [Min(0)]
        public int networkIndex;

        /// <summary>
        /// The root joint of the robot.
        /// </summary>
        private ArticulationBody Root => _joints[0].Joint;

        /// <summary>
        /// The joints.
        /// </summary>
        private RobotJoint[] _joints;

        /// <summary>
        /// The joint limits of the robot.
        /// </summary>
        private JointLimit[] _limits;

        /// <summary>
        /// Simple reference zero values for the robot.
        /// </summary>
        private List<float> _zeros;

        /// <summary>
        /// The current joint targets.
        /// </summary>
        private List<float> _targets;

        /// <summary>
        /// The max speeds of the joints.
        /// </summary>
        private float[] _maxSpeeds;

        /// <summary>
        /// How fast the joints are currently moving.
        /// </summary>
        private float[] _currentSpeeds;

        /// <summary>
        /// The networks to control the joints.
        /// </summary>
        private Model[][] _networks;

        /// <summary>
        /// The network workers.
        /// </summary>
        private IWorker[][] _workers;

        /// <summary>
        /// The Bio IK controller.
        /// </summary>
        private BioIk _bioIk;

        /// <summary>
        /// Get a name for display.
        /// </summary>
        /// <param name="mode">The mode of a robot.</param>
        /// <returns>A name for a robot.</returns>
        public static string Name(SolverMode mode)
        {
            switch (mode)
            {
                case SolverMode.Network:
                    return "Network";
                case SolverMode.BioIk:
                    return "Bio IK";
                case SolverMode.FusionIk:
                default:
                    return "Fusion IK";
            }
        }

        /// <summary>
        /// Get a color for display.
        /// </summary>
        /// <param name="mode">The mode of a robot.</param>
        /// <returns>A color for a robot.</returns>
        public static Color RobotColor(SolverMode mode)
        {
            switch (mode)
            {
                case SolverMode.Network:
                    return Color.magenta;
                case SolverMode.BioIk:
                    return Color.cyan;
                case SolverMode.FusionIk:
                default:
                    return Color.white;
            }
        }

        /// <summary>
        /// Get the color for the robot.
        /// </summary>
        /// <returns>The color for the robot.</returns>
        public Color RobotColor()
        {
            return RobotColor(mode);
        }

        /// <summary>
        /// Set the radians to move to.
        /// </summary>
        /// <param name="radians">The radians to move to.</param>
        public void Move(List<float> radians)
        {
            // Set the targets.
            _targets = radians;

            // Calculate the slowest joint.
            List<float> angles = GetJoints();
            float time = 0;
            for (int i = 0; i < angles.Count; i++)
            {
                angles[i] = math.abs(angles[i] - _targets[i]);
                if (angles[i] / _maxSpeeds[i] > time)
                {
                    time = angles[i] / _maxSpeeds[i];
                }
            }

            // Limit the speeds of faster joints so they all arrive at the same time.
            if (time > 0)
            {
                for (int i = 0; i < _currentSpeeds.Length; i++)
                {
                    _currentSpeeds[i] = angles[i] / time;
                }
            }

            // Flag to move during fixed update.
            IsMoving = true;
        }
    
        /// <summary>
        /// Snap to a target.
        /// </summary>
        /// <param name="position">The position to reach.</param>
        /// <param name="rotation">The rotation to reach.</param>
        /// <param name="milliseconds">The time the algorithm is allowed to run for.</param>
        /// <param name="reached">If the target was reached.</param>
        /// <param name="moveTime">The time for the joints to reach the destination.</param>
        /// <param name="fitness">The fitness score of the result.</param>
        /// <param name="seed">The seed for the random numbers of the solver.</param>
        public void Snap(Vector3 position, Quaternion rotation, long milliseconds, out bool reached, out double moveTime, out double fitness, uint seed = 0)
        {
            Snap(Solve(position, rotation, milliseconds, out reached, out moveTime, out fitness, seed));
        }

        /// <summary>
        /// Snap joints to radian values.
        /// </summary>
        /// <param name="radians">The radians to snap to.</param>
        public void Snap(List<float> radians)
        {
            IsMoving = false;
            Snap((IEnumerable<float>) radians);
        }

        /// <summary>
        /// Get the current joints of the robot.
        /// </summary>
        /// <returns>The joints of the robot.</returns>
        public List<float> GetJoints()
        {
            List<float> angles = new(_limits.Length);
            Root.GetJointPositions(angles);
            return angles;
        }

        /// <summary>
        /// Assign random joint values.
        /// </summary>
        /// <returns>Random joint values.</returns>
        public List<float> RandomJoints()
        {
            List<float> angles = new(_limits.Length);
            for (int i = 0; i < _limits.Length; i++)
            {
                angles.Add(Random.Range(_limits[i].lower, _limits[i].upper));
            }

            return angles;
        }
        
        /// <summary>
        /// Step the physics simulation.
        /// </summary>
        public static void PhysicsStep()
        {
            Physics.simulationMode = SimulationMode.Script;
            Physics.Simulate(1);
            Physics.simulationMode = SimulationMode.FixedUpdate;
        }

        /// <summary>
        /// If a position and rotation were reached.
        /// </summary>
        /// <param name="targetPosition">The position to check.</param>
        /// <param name="targetRotation">The rotation to check.</param>
        /// <returns>True if reached, false otherwise.</returns>
        private bool Reached(Vector3 targetPosition, Quaternion targetRotation) => Reached(targetPosition, targetRotation, LastJoint.position, LastJoint.rotation);
        
        /// <summary>
        /// If a position and rotation were reached relative to a root position and rotation.
        /// </summary>
        /// <param name="targetPosition">The position to check.</param>
        /// <param name="targetRotation">The rotation to check.</param>
        /// <param name="position">The root position.</param>
        /// <param name="rotation">The root rotation.</param>
        /// <returns>True if reached, false otherwise.</returns>
        public bool Reached(Vector3 targetPosition, Quaternion targetRotation, Vector3 position, Quaternion rotation) => PositionAccuracy(targetPosition, position) + RotationAccuracy(targetRotation, rotation) <= properties.Repeatability;

        /// <summary>
        /// If the position was reached.
        /// </summary>
        /// <param name="targetPosition">The position to check.</param>
        /// <param name="position">The root position.</param>
        /// <returns>True if reached, false otherwise.</returns>
        public static float PositionAccuracy(Vector3 targetPosition, Vector3 position) => Vector3.Distance(position, targetPosition);

        /// <summary>
        /// If the rotation was reached.
        /// </summary>
        /// <param name="targetRotation">The rotation to check.</param>
        /// <param name="rotation">The root rotation.</param>
        /// <returns>True if reached, false otherwise.</returns>
        public static float RotationAccuracy(Quaternion targetRotation, Quaternion rotation) => Quaternion.Angle(targetRotation, rotation) / 360;

        /// <summary>
        /// Calculate the time needed for a robot to move from its starting to ending joint values.
        /// </summary>
        /// <param name="starting">Starting joint position.s</param>
        /// <param name="ending">Ending joint positions.</param>
        /// <returns>The time to complete the move.</returns>
        public float CalculateTime(IEnumerable<float> starting, IReadOnlyList<float> ending) => starting.Select((t, i) => math.abs(t - ending[i]) / _maxSpeeds[i]).Prepend(0).Max();
        
        public double CalculateTime(double[] starting, double[] ending)
        {
            float[] startingFloats = new float[starting.Length];
            float[] endingFloats = new float[starting.Length];
            for (int i = 0; i < starting.Length; i++)
            {
                startingFloats[i] = (float) starting[i];
                endingFloats[i] = (float) ending[i];
            }
            
            return CalculateTime(startingFloats, endingFloats);
        }

        /// <summary>
        /// Get a position relative to the root of the robot.
        /// </summary>
        /// <param name="position">The position in global position.</param>
        /// <returns>The relative position.</returns>
        private Vector3 RelativePosition(Vector3 position) => Root.transform.InverseTransformPoint(position) / ChainLength;

        /// <summary>
        /// Get a rotation relative to the root of the robot.
        /// </summary>
        /// <param name="rotation">The rotation in global rotation.</param>
        /// <returns>The relative rotation.</returns>
        private Quaternion RelativeRotation(Quaternion rotation) => Quaternion.Inverse(Root.transform.rotation) * rotation;

        /// <summary>
        /// Stop the robot at a position.
        /// </summary>
        /// <param name="radians">The joint values in radians to stop at.</param>
        private void Snap(IEnumerable<float> radians)
        {
            List<float> list = radians.ToList();
            Root.SetDriveTargets(list);
            Root.SetJointPositions(list);
            Root.SetJointVelocities(_zeros);
            Root.SetJointForces(_zeros);
        }

        /// <summary>
        /// Scale joint values between zero and one for network inference.
        /// </summary>
        /// <param name="joints">The joint values.</param>
        /// <returns>The joints scaled for network inference.</returns>
        public List<float> NetScaledJoints(List<float> joints)
        {
            List<float> scaled = new(joints.Count);
            for (int i = 0; i < joints.Count; i++)
            {
                scaled.Add((joints[i] - _limits[i].lower) / (_limits[i].upper - _limits[i].lower));
            }

            return scaled;
        }
        
        /// <summary>
        /// Scale network inference back to joint values.
        /// </summary>
        /// <param name="joints">The joints as received from network inference.</param>
        /// <returns>The joints scaled to their real values.</returns>
        public List<float> ResultsScaled(List<float> joints)
        {
            List<float> scaled = new(joints.Count);
            for (int i = 0; i < joints.Count; i++)
            {
                scaled.Add(math.clamp(joints[i] * (_limits[i].upper - _limits[i].lower) + _limits[i].lower, _limits[i].lower, _limits[i].upper));
            }

            return scaled;
        }

        /// <summary>
        /// Prepare inputs for networks.
        /// </summary>
        /// <param name="targetPosition">The position to reach.</param>
        /// <param name="targetRotation">The rotation to reach.</param>
        /// <param name="starting">The starting joint values.</param>
        /// <returns>The values to be passed to the networks.</returns>
        public float[] PrepareInputs(Vector3 targetPosition, Quaternion targetRotation, List<float> starting)
        {
            // Scale and add joints.
            List<float> scaled = NetScaledJoints(starting);
            float[] inputs = new float[scaled.Count + 7];
            for (int i = 0; i < scaled.Count; i++)
            {
                inputs[i] = scaled[i];
            }

            // Get relative values.
            targetPosition = RelativePosition(targetPosition);
            targetRotation = RelativeRotation(targetRotation);
            
            // Add scaled position.
            inputs[scaled.Count] = (targetPosition.x + 1) / 2;
            inputs[scaled.Count + 1] = (targetPosition.y + 1) / 2;
            inputs[scaled.Count + 2] = (targetPosition.z + 1) / 2;
            
            // Add scaled rotation.
            inputs[scaled.Count + 3] = (targetRotation.x + 1) / 2;
            inputs[scaled.Count + 4] = (targetRotation.y + 1) / 2;
            inputs[scaled.Count + 5] = (targetRotation.z + 1) / 2;
            inputs[scaled.Count + 6] = (targetRotation.w + 1) / 2;

            return inputs;
        }

        /// <summary>
        /// Request a solution.
        /// </summary>
        /// <param name="targetPosition">The position to reach.</param>
        /// <param name="targetRotation">The rotation to reach.</param>
        /// <param name="milliseconds">The time the algorithm is allowed to run for.</param>
        /// <param name="reached">If the robot reached the destination.</param>
        /// <param name="moveTime">How long it took for the robot to perform the move.</param>
        /// <param name="fitness">The fitness score of the result.</param>
        /// <param name="seed">The seed for random number generation</param>
        /// <returns>The joints to move the robot to.</returns>
        public List<float> Solve(Vector3 targetPosition, Quaternion targetRotation, long milliseconds, out bool reached, out double moveTime, out double fitness, uint seed = 0)
        {
            Stopwatch stopwatch = Stopwatch.StartNew();
            
            // If already at the destination do nothing.
            reached = Reached(targetPosition, targetRotation);
            if (reached)
            {
                moveTime = 0;
                fitness = 0;
                return GetJoints();
            }
            
            // Initialize other variables.
            List<float> starting = GetJoints();
            List<float> results = null;
            moveTime = float.MaxValue;
            fitness = double.MaxValue;

            // Run through neural networks if it should.
            if (mode != SolverMode.BioIk)
            {
                results = RunNetwork(PrepareInputs(targetPosition, targetRotation, starting));
            }

            // Use Bio IK if it should.
            if (stopwatch.ElapsedMilliseconds < milliseconds && mode != SolverMode.Network)
            {
                // Convert to doubles.
                double[] doubles = new double[starting.Count];
                double[] bioSeed = new double[starting.Count];
                double[] solution = new double[starting.Count];
                for (int i = 0; i < starting.Count; i++)
                {
                    doubles[i] = starting[i];
                    bioSeed[i] = results != null ? results[i] : starting[i];
                    solution[i] = bioSeed[i];
                }
            
                // If no seed was passed, create a random one.
                if (seed == 0)
                {
                    seed = (uint) Random.Range(1, int.MaxValue);
                }
            
                // Initialize random number generation.
                Unity.Mathematics.Random random = new(seed);
            
                // Utilize all available generations.
                do
                {
                    // Run Bio IK.
                    _bioIk = new(this, properties.Population, properties.Elites);
                    double[] attemptSolution = _bioIk.Optimise(bioSeed, targetPosition, targetRotation, milliseconds - stopwatch.ElapsedMilliseconds, ref random, out bool attemptReached, out double attemptFitness);
                
                    // If have already reached.
                    if (reached)
                    {
                        // If the current attempt did not reach, discard it.
                        if (!attemptReached)
                        {
                            continue;
                        }

                        // If the new solutions is faster than the existing solution, update it.
                        double attemptMoveTime = CalculateTime(doubles, attemptSolution);
                        if (attemptMoveTime >= moveTime)
                        {
                            continue;
                        }

                        solution = attemptSolution;
                        moveTime = attemptMoveTime;
                        continue;
                    }

                    // If the attempt did not reach the target.
                    if (!attemptReached)
                    {
                        // If the fitness of this attempt was worse than other failed attempts, discard it.
                        if (attemptFitness >= fitness)
                        {
                            continue;
                        }

                        // Otherwise this is the best failure so store it.
                        solution = attemptSolution;
                        moveTime = CalculateTime(doubles, solution);
                        fitness = attemptFitness;
                        continue;
                    }

                    // This is the first successful reach so store it.
                    reached = true;
                    solution = attemptSolution;
                    moveTime = CalculateTime(doubles, solution);
                    fitness = 0;
                } while (stopwatch.ElapsedMilliseconds < milliseconds);

                results = solution.Select(t => (float) t).ToList();
            }
            
            // Fitness should be zero if it was reached.
            if (reached)
            {
                fitness = 0;
            }

            return results;
        }

        /// <summary>
        /// Run the network inference.
        /// </summary>
        /// <param name="inputs">The inputs to the networks.</param>
        /// <returns>The joints to move the robot to.</returns>
        private List<float> RunNetwork(IEnumerable<float> inputs)
        {
            // Get initial input values and prepare for outputs.
            float[] forwards = inputs.ToArray();
            List<float> outputs = new(_workers[networkIndex].Length);
            
            // Go through every joint's network.
            for (int i = 0; i < _workers[networkIndex].Length; i++)
            {
                // Run the current joint network.
                Tensor input = new(1, 1, 1, forwards.Length, forwards, "INPUTS");
                Tensor output = _workers[networkIndex][i].Execute(input).PeekOutput();
                input.Dispose();
                
                // Add the output and replace the input for the next joint's network.
                outputs.Add(math.clamp(output[0, 0, 0, 0], 0, 1));
                output.Dispose();
                forwards[i] = outputs[i];
            }

            return ResultsScaled(outputs);
        }

        private void Start()
        {
            // Ensure there are properties attached.
            if (properties == null)
            {
                Debug.LogError($"No robot properties attached to {name}.");
#if UNITY_EDITOR
                EditorApplication.ExitPlaymode();
#endif
                Destroy(gameObject);
                return;
            }

            RobotJoint rootJoint = GetComponent<RobotJoint>();

            RobotJoint[] children = GetComponentsInChildren<RobotJoint>();

            // Ensure there are joints.
            if (rootJoint == null)
            {
                if (children.Length == 0)
                {
                    Debug.LogError($"No articulation bodies attached to {name}.");
#if UNITY_EDITOR
                    EditorApplication.ExitPlaymode();
#endif
                    Destroy(gameObject);
                    return;
                }

                _joints = children;
            }
            else if (children.Length > 0)
            {
                _joints = new RobotJoint[children.Length + 1];
                _joints[0] = rootJoint;
                for (int i = 1; i < _joints.Length - 1; i++)
                {
                    _joints[i] = children[i - 1];
                }
            }
            else
            {
                _joints = new RobotJoint[1];
                _joints[0] = rootJoint;
            }
            
            // Setup all joints.
            foreach (RobotJoint robotJoint in _joints)
            {
                robotJoint.Setup();
            }

            _joints = _joints.OrderBy(j => j.Joint.index).ToArray();
            
            // Calculate the chain length.
            ChainLength = 0;
            List<JointLimit> limits = new();
            for (int i = 0; i < _joints.Length; i++)
            {
                limits.AddRange(_joints[i].Limits());
                if (i > 0)
                {
                    ChainLength += Vector3.Distance(_joints[i - 1].transform.position, _joints[i].transform.position);
                }
            }

            Rescaling = math.PI * math.PI / (ChainLength * ChainLength);

            _limits = limits.ToArray();
            List<float> joints = GetJoints();
            
            // Get the max speeds.
            List<float> speeds = new(_limits.Length);
            foreach (RobotJoint j in _joints)
            {
                if (!j.HasMotion)
                {
                    continue;
                }

                if (j.XMotion)
                {
                    speeds.Add(j.SpeedX);
                }

                if (j.YMotion)
                {
                    speeds.Add(j.SpeedY);
                }

                if (j.ZMotion)
                {
                    speeds.Add(j.SpeedZ);
                }
            }
            _maxSpeeds = speeds.ToArray();

            // Ensure every joint had limits assigned.
            if (joints.Count != _limits.Length)
            {
                Debug.LogError($"Ensure all joints on {name} have limits defined.");
#if UNITY_EDITOR
                EditorApplication.ExitPlaymode();
#endif
                Destroy(gameObject);
                return;
            }
        
            // Create helper zero list.
            _zeros = new(joints.Count);
            for (int i = 0; i < joints.Count; i++)
            {
                _zeros.Add(0);
            }
            
            // Create data to hold current move speeds.
            _currentSpeeds = new float[_maxSpeeds.Length];
            for (int i = 0; i < _currentSpeeds.Length; i++)
            {
                _currentSpeeds[i] = _maxSpeeds[i];
            }

            // Error check.
            if (joints.Count != _maxSpeeds.Length)
            {
                Debug.LogError($"{name} has {joints.Count} degrees of freedom but {_maxSpeeds.Length} speeds defined.");
#if UNITY_EDITOR
                EditorApplication.ExitPlaymode();
#endif
                Destroy(gameObject);
                return;
            }

            List<GhostJoint> bioIkJoints = new();
            GhostJoint previousJoint = null;
            Transform parent = transform;

            int jointNumber = 1;

            // Create ghost joints for Bio IK.
            foreach (RobotJoint j in _joints)
            {
                // Only for joints that have motion.
                if (!j.HasMotion)
                {
                    continue;
                }

                Transform jointTransform = j.transform;
                GameObject go = new($"Bio IK Joint {jointNumber++}")
                {
                    transform =
                    {
                        parent = parent,
                        position = jointTransform.position,
                        rotation = jointTransform.rotation
                    }
                };

                GhostJoint ghostJoint = go.AddComponent<GhostJoint>();
                if (previousJoint != null)
                {
                    ghostJoint.parent = previousJoint;
                    previousJoint.child = ghostJoint;
                }
                ghostJoint.Setup();
                previousJoint = ghostJoint;
                parent = go.transform;
                
                ghostJoint.rotational = j.Type != ArticulationJointType.PrismaticJoint;
                ghostJoint.SetRotation(Vector3.zero);
                bioIkJoints.Add(ghostJoint);

                if (j.XMotion)
                {
                    ghostJoint.y.enabled = true;
                    if (!ghostJoint.rotational)
                    {
                        ghostJoint.y.SetLowerLimit(j.LimitX.lower);
                        ghostJoint.y.SetUpperLimit(j.LimitX.upper);
                    }
                    else
                    {
                        ghostJoint.y.SetLowerLimit(math.degrees(j.LimitX.lower));
                        ghostJoint.y.SetUpperLimit(math.degrees(j.LimitX.upper));
                    }
                }
                else
                {
                    ghostJoint.y.enabled = false;
                }

                if (j.YMotion)
                {
                    ghostJoint.z.enabled = true;
                    if (!ghostJoint.rotational)
                    {
                        ghostJoint.z.SetLowerLimit(j.LimitY.lower);
                        ghostJoint.z.SetUpperLimit(j.LimitY.upper);
                    }
                    else
                    {
                        ghostJoint.z.SetLowerLimit(math.degrees(j.LimitY.lower));
                        ghostJoint.z.SetUpperLimit(math.degrees(j.LimitY.upper));
                    }
                }
                else
                {
                    ghostJoint.z.enabled = false;
                }
                
                if (j.ZMotion)
                {
                    ghostJoint.x.enabled = true;
                    if (!ghostJoint.rotational)
                    {
                        ghostJoint.x.SetLowerLimit(j.LimitZ.lower);
                        ghostJoint.x.SetUpperLimit(j.LimitZ.upper);
                    }
                    else
                    {
                        ghostJoint.x.SetLowerLimit(math.degrees(j.LimitZ.lower));
                        ghostJoint.x.SetUpperLimit(math.degrees(j.LimitZ.upper));
                    }
                }
                else
                {
                    ghostJoint.x.enabled = false;
                }
            }

            GhostJoints = bioIkJoints.ToArray();

            // Configure ghost.
            foreach (GhostJoint j in GhostJoints)
            {
                j.UpdateData();
            }

            // Setup networks.
            if (properties.networks.Length > 0)
            {
                _networks = new Model[properties.networks.Length][];
                _workers = new IWorker[_networks.Length][];
                bool networkValid = true;
                for (int i = 0; i < _networks.Length; i++)
                {
                    _networks[i] = new Model[properties.networks[i].networks.Length];
                    _workers[i] = new IWorker[_networks[i].Length];

                    for (int j = 0; j < _networks[i].Length; j++)
                    {
                        _networks[i][j] = properties.CompiledNetwork(i, j);
                        if (_networks[i][j] != null)
                        {
                            _workers[i][j] = WorkerFactory.CreateWorker(_networks[i][j]);
                            continue;
                        }

                        networkValid = false;
                    }
                }

                // Do an initial network run as it is slower on the first inference.
                if (networkValid)
                {
                    if (networkIndex >= _networks.Length)
                    {
                        networkIndex = _networks.Length - 1;
                    }
                    
                    // Save what network we currently want.
                    int currentNetworkIndex = networkIndex;
                
                    // Run every network for the initial speed limits.
                    for (int i = 0; i < _networks.Length; i++)
                    {
                        networkIndex = i;
                        RunNetwork(PrepareInputs(Vector3.zero, Quaternion.identity, GetJoints()).ToList());
                    }

                    // Switch to the desired network.
                    networkIndex = currentNetworkIndex;
                }
                else
                {
                    mode = SolverMode.BioIk;
                    networkIndex = 0;
                    _networks = null;
                    CleanupWorkers();
                    _workers = null;
                }
            }
            else
            {
                networkIndex = 0;
                mode = SolverMode.BioIk;
            }
        }

        private void FixedUpdate()
        {
            if (!IsMoving)
            {
                return;
            }

            IsMoving = false;

            // Move the robot towards the target joint values.
            List<float> delta = GetJoints();
            for (int i = 0; i < delta.Count; i++)
            {
                if (delta[i] >= _targets[i])
                {
                    delta[i] -= _currentSpeeds[i] * Time.fixedDeltaTime;
                    if (delta[i] <= _targets[i])
                    {
                        delta[i] = _targets[i];
                    }
                    else
                    {
                        IsMoving = true;
                    }
                }
                else
                {
                    delta[i] += _currentSpeeds[i] * Time.fixedDeltaTime;
                    if (delta[i] >= _targets[i])
                    {
                        delta[i] = _targets[i];
                    }
                    else
                    {
                        IsMoving = true;
                    }
                }
            }

            // Set to position.
            Snap((IEnumerable<float>) delta);
        }

        private void OnDestroy()
        {
            CleanupWorkers();
        }

        /// <summary>
        /// Cleanup all network workers.
        /// </summary>
        private void CleanupWorkers()
        {
            if (_workers  == null)
            {
                return;
            }
            
            // Clean up network runners.
            for (int i = 0; i < _workers.Length; i++)
            {
                for (int j = 0; j < _workers[i].Length; j++)
                {
                    _workers[i][j]?.Dispose();
                }
            }
        }
    }
}