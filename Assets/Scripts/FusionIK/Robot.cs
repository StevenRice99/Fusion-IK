using System;
using System.Collections.Generic;
using System.Linq;
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
            BioIk,
            Network,
            FusionIk
        }

        /// <summary>
        /// The properties of the robot.
        /// </summary>
        public Properties Properties => properties;
        
        /// <summary>
        /// The ghost joints of the robot.
        /// </summary>
        public VirtualJoint[] GhostJoints { get; private set; }
        
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
        /// Ghost for calculations.
        /// </summary>
        public VirtualRobot Virtual { get; private set; }
        
        /// <summary>
        /// Middle joint values.
        /// </summary>
        public List<float> Middle { get; private set; }

        /// <summary>
        /// The joint limits of the robot.
        /// </summary>
        public JointLimit[] Limits { get; private set; }

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

        /// <summary>
        /// The mode to solve in.
        /// </summary>
        [NonSerialized]
        public SolverMode mode;

        [NonSerialized]
        public bool exhaustive;

        [NonSerialized]
        public bool iterative;

        [NonSerialized]
        public bool minimal;

        /// <summary>
        /// The root joint of the robot.
        /// </summary>
        private ArticulationBody Root => _joints[0].Joint;

        /// <summary>
        /// The joints.
        /// </summary>
        private RobotJoint[] _joints;

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
        /// Worker for the standard network.
        /// </summary>
        [NonSerialized]
        private IWorker _standardWorker;

        /// <summary>
        /// Worker for the minimal network.
        /// </summary>
        [NonSerialized]
        private IWorker _minimalWorker;

        /// <summary>
        /// Get a name for display.
        /// </summary>
        /// <returns>A name for a robot.</returns>
        public override string ToString()
        {
            return (exhaustive ? "Exhaustive " : string.Empty) + (iterative ? "Iterative " : string.Empty) +
                   (minimal ? "Minimal " : string.Empty) + mode switch
                   {
                       SolverMode.BioIk => "Bio IK",
                       SolverMode.Network => "Network",
                       _ => "Fusion IK"
                   };
        }

        /// <summary>
        /// Get a color for display.
        /// </summary>
        /// <returns>A color for a robot.</returns>
        public Color ToColor()
        {
            if (mode == SolverMode.Network)
            {
                return minimal ? Color.magenta : new(0.5f, 0, 1);
            }
            
            if (minimal)
            {
                return Color.white;
            }
            
            if (exhaustive)
            {
                return iterative ? new(1, 0.71f, 0.008f) : Color.yellow;
            }

            if (exhaustive)
            {
                return new(1, 0.5f, 0);
            }

            return mode == SolverMode.BioIk ? Color.cyan : new(0.5f, 1, 1);
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
        /// Snap joints to radian values.
        /// </summary>
        /// <param name="radians">The radians to snap to.</param>
        public void Snap(IEnumerable<float> radians)
        {
            IsMoving = false;
            SnapPerform(radians);
        }
        
        /// <summary>
        /// Snap joints to radian values.
        /// </summary>
        /// <param name="radians">The radians to snap to.</param>
        private void SnapPerform(IEnumerable<float> radians)
        {
            List<float> list = radians.ToList();
            Root.SetDriveTargets(list);
            Root.SetJointPositions(list);
            Root.SetJointVelocities(_zeros);
            Root.SetJointForces(_zeros);
        }

        /// <summary>
        /// Get the current joints of the robot.
        /// </summary>
        /// <returns>The joints of the robot.</returns>
        public List<float> GetJoints()
        {
            List<float> angles = new(Limits.Length);
            Root.GetJointPositions(angles);
            return angles;
        }

        /// <summary>
        /// Assign random joint values.
        /// </summary>
        /// <returns>Random joint values.</returns>
        public List<float> RandomJoints()
        {
            List<float> angles = new(Limits.Length);
            for (int i = 0; i < Limits.Length; i++)
            {
                angles.Add(Random.Range(Limits[i].lower, Limits[i].upper));
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
        private static float PositionAccuracy(Vector3 targetPosition, Vector3 position) => Vector3.Distance(position, targetPosition);

        /// <summary>
        /// If the rotation was reached.
        /// </summary>
        /// <param name="targetRotation">The rotation to check.</param>
        /// <param name="rotation">The root rotation.</param>
        /// <returns>True if reached, false otherwise.</returns>
        private static float RotationAccuracy(Quaternion targetRotation, Quaternion rotation) => Quaternion.Angle(targetRotation, rotation) / 360;

        /// <summary>
        /// Calculate the time needed for a robot to move from its middle to ending joint values.
        /// </summary>
        /// <param name="starting">Ending joint positions.</param>
        /// <param name="ending">Ending joint positions.</param>
        /// <returns>The time to complete the move.</returns>
        public double CalculateTime(IEnumerable<double> starting, IReadOnlyList<double> ending)
        {
            return starting.Select((t, i) => math.abs(t - ending[i]) / _maxSpeeds[i]).Prepend(0).Max();
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
        /// Scale joint values between zero and one for network inference.
        /// </summary>
        /// <param name="joints">The joint values.</param>
        /// <returns>The joints scaled for network inference.</returns>
        public List<float> NetScaledJoints(List<float> joints)
        {
            List<float> scaled = new(joints.Count);
            for (int i = 0; i < joints.Count; i++)
            {
                scaled.Add((joints[i] - Limits[i].lower) / (Limits[i].upper - Limits[i].lower));
            }

            return scaled;
        }

        /// <summary>
        /// Run the network inference.
        /// </summary>
        /// <param name="position">The position to reach.</param>
        /// <param name="rotation">The rotation to reach.</param>
        /// <param name="starting">The joints to start at.</param>
        /// <returns>The joints to move the robot to.</returns>
        public List<float> RunNetwork(Vector3 position, Quaternion rotation, List<float> starting = null)
        {
            // Get initial input values and prepare for outputs.
            float[] inputs = PrepareInputs(position, rotation, starting);
            Tensor input = new(1, 1, 1, inputs.Length, inputs, "INPUTS");

            IWorker worker = minimal ? _minimalWorker : _standardWorker;

            // Run the current joint network.
            worker.Execute(input);
            worker.FlushSchedule(true);
            Tensor output = worker.PeekOutput();
            input.Dispose();
            
            // Add the output and replace the input for the next joint's network.
            List<float> outputs = new(Limits.Length);
            for (int i = 0; i < Limits.Length; i++)
            {
                outputs.Add(math.clamp(output[0, 0, 0, i], 0, 1));
            }
            output.Dispose();

            return ResultsScaled(outputs);
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
                scaled.Add(math.clamp(joints[i] * (Limits[i].upper - Limits[i].lower) + Limits[i].lower, Limits[i].lower, Limits[i].upper));
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
        public float[] PrepareInputs(Vector3 targetPosition, Quaternion targetRotation, List<float> starting = null)
        {
            int offset = starting?.Count ?? 0;
            float[] inputs = new float[offset + 7];
            
            // Scale and add joints.
            if (offset > 0)
            {
                List<float> scaled = NetScaledJoints(starting);
                for (int i = 0; i < offset; i++)
                {
                    inputs[i] = scaled[i];
                }
            }

            // Get relative values.
            targetPosition = RelativePosition(targetPosition);
            targetRotation = RelativeRotation(targetRotation);
            
            // Add scaled position.
            inputs[offset] = (targetPosition.x + 1) / 2;
            inputs[offset + 1] = (targetPosition.y + 1) / 2;
            inputs[offset + 2] = (targetPosition.z + 1) / 2;
            
            // Add scaled rotation.
            inputs[offset + 3] = (targetRotation.x + 1) / 2;
            inputs[offset + 4] = (targetRotation.y + 1) / 2;
            inputs[offset + 5] = (targetRotation.z + 1) / 2;
            inputs[offset + 6] = (targetRotation.w + 1) / 2;

            return inputs;
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

            Limits = limits.ToArray();

            Middle = new(Limits.Length);
            for (int i = 0; i < Limits.Length; i++)
            {
                Middle.Add((Limits[i].lower + Limits[i].upper) / 2);
            }
            
            List<float> joints = GetJoints();
            
            // Get the max speeds.
            List<float> speeds = new(Limits.Length);
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
            if (joints.Count != Limits.Length)
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

            List<VirtualJoint> bioIkJoints = new();
            VirtualJoint previousJoint = null;
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

                VirtualJoint virtualJoint = go.AddComponent<VirtualJoint>();
                if (previousJoint != null)
                {
                    virtualJoint.parent = previousJoint;
                    previousJoint.child = virtualJoint;
                }
                virtualJoint.Setup();
                previousJoint = virtualJoint;
                parent = go.transform;
                
                virtualJoint.rotational = j.Type != ArticulationJointType.PrismaticJoint;
                virtualJoint.SetRotation(Vector3.zero);
                bioIkJoints.Add(virtualJoint);

                if (j.XMotion)
                {
                    virtualJoint.y.enabled = true;
                    if (!virtualJoint.rotational)
                    {
                        virtualJoint.y.SetLowerLimit(j.LimitX.lower);
                        virtualJoint.y.SetUpperLimit(j.LimitX.upper);
                    }
                    else
                    {
                        virtualJoint.y.SetLowerLimit(math.degrees(j.LimitX.lower));
                        virtualJoint.y.SetUpperLimit(math.degrees(j.LimitX.upper));
                    }
                }
                else
                {
                    virtualJoint.y.enabled = false;
                }

                if (j.YMotion)
                {
                    virtualJoint.z.enabled = true;
                    if (!virtualJoint.rotational)
                    {
                        virtualJoint.z.SetLowerLimit(j.LimitY.lower);
                        virtualJoint.z.SetUpperLimit(j.LimitY.upper);
                    }
                    else
                    {
                        virtualJoint.z.SetLowerLimit(math.degrees(j.LimitY.lower));
                        virtualJoint.z.SetUpperLimit(math.degrees(j.LimitY.upper));
                    }
                }
                else
                {
                    virtualJoint.z.enabled = false;
                }
                
                if (j.ZMotion)
                {
                    virtualJoint.x.enabled = true;
                    if (!virtualJoint.rotational)
                    {
                        virtualJoint.x.SetLowerLimit(j.LimitZ.lower);
                        virtualJoint.x.SetUpperLimit(j.LimitZ.upper);
                    }
                    else
                    {
                        virtualJoint.x.SetLowerLimit(math.degrees(j.LimitZ.lower));
                        virtualJoint.x.SetUpperLimit(math.degrees(j.LimitZ.upper));
                    }
                }
                else
                {
                    virtualJoint.x.enabled = false;
                }
            }

            GhostJoints = bioIkJoints.ToArray();

            // Configure ghost.
            foreach (VirtualJoint j in GhostJoints)
            {
                j.UpdateData();
            }

            bool original = minimal;
            
            // Setup large network.
            if (properties.StandardNetworkValid)
            {
                _standardWorker = WorkerFactory.CreateWorker(WorkerFactory.Type.CSharpBurst, ModelLoader.Load(properties.standardNetwork));

                if (_standardWorker != null)
                {
                    minimal = false;
                    RunNetwork(Vector3.zero, Quaternion.identity, GetJoints());
                }
                else
                {
                    _standardWorker?.Dispose();
                }
            }

            // Setup minimal network.
            if (properties.MinimalNetworkValid)
            {
                _minimalWorker = WorkerFactory.CreateWorker(WorkerFactory.Type.CSharpBurst, ModelLoader.Load(properties.minimalNetwork));

                if (_minimalWorker != null)
                {
                    minimal = true;
                    RunNetwork(Vector3.zero, Quaternion.identity);
                }
                else
                {
                    _minimalWorker?.Dispose();
                }
            }

            minimal = original;

            Virtual = new(this);
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
            SnapPerform(delta);
        }

        public void OnDestroy()
        {
            _standardWorker?.Dispose();
            _minimalWorker?.Dispose();
        }
    }
}