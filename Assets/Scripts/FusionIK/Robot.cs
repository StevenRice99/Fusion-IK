using System.Collections.Generic;
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
            FusionIk,
            GreedyFusionIk,
            ExhaustiveFusionIk
        }

        /// <summary>
        /// The properties of the robot.
        /// </summary>
        public RobotProperties Properties => properties;
        
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
        private RobotProperties properties;

        [Tooltip("Mode to perform solving in.")]
        public SolverMode mode = SolverMode.BioIk;

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
        private Model[] _networks;

        /// <summary>
        /// The network workers.
        /// </summary>
        private IWorker[] _workers;

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
                    return "Fusion IK";
                case SolverMode.GreedyFusionIk:
                    return "Greedy Fusion IK";
                case SolverMode.ExhaustiveFusionIk:
                default:
                    return "Exhaustive Fusion IK";
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
                    return new(1, 0.65f, 0);
                case SolverMode.FusionIk:
                    return Color.white;
                case SolverMode.GreedyFusionIk:
                    return Color.cyan;
                case SolverMode.ExhaustiveFusionIk:
                default:
                    return Color.yellow;
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
        public void MoveRadians(List<float> radians)
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
        /// <param name="maxGenerations">The maximum generations for the solving algorithm.</param>
        /// <param name="reached">If the target was reached.</param>
        /// <param name="moveTime">The time for the joints to reach the destination.</param>
        /// <param name="generations">The generations required.</param>
        /// <param name="seed">The seed for the random numbers of the solver.</param>
        public void Snap(Vector3 position, Quaternion rotation, int maxGenerations, out bool reached, out float moveTime, out int generations, uint seed = 0)
        {
            SnapRadians(RequestSolution(position, rotation, maxGenerations, out reached, out moveTime, out generations, seed));
        }

        /// <summary>
        /// Snap joints to radian values.
        /// </summary>
        /// <param name="radians">The radians to snap to.</param>
        public void SnapRadians(List<float> radians)
        {
            IsMoving = false;
            Snap(radians);
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
        public float RotationAccuracy(Quaternion targetRotation, Quaternion rotation) => Quaternion.Angle(targetRotation, rotation) / 360;

        /// <summary>
        /// Calculate the time needed for a robot to move from its starting to ending joint values.
        /// </summary>
        /// <param name="starting">Starting joint position.s</param>
        /// <param name="ending">Ending joint positions.</param>
        /// <returns>The time to complete the move.</returns>
        private float CalculateTime(IEnumerable<float> starting, IReadOnlyList<float> ending) => starting.Select((t, i) => math.abs(t - ending[i]) / _maxSpeeds[i]).Prepend(0).Max();

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
        /// Optimize a robot movement from a starting position to a destination.
        /// </summary>
        /// <param name="targetPosition">The position to reach.</param>
        /// <param name="targetRotation">The rotation to reach.</param>
        /// <param name="maxGenerations">The number of generations for each attempt.</param>
        /// <param name="starting">The starting joint values.</param>
        /// <param name="attempts">The number of Bio IK attempts to perform.</param>
        /// <param name="hasReached">If the robot reached the destination.</param>
        /// <returns>The move to the destination which takes the least amount of time.</returns>
        public List<float> BioIkOptimize(Vector3 targetPosition, Quaternion targetRotation, int maxGenerations, float[] starting, int attempts, out bool hasReached)
        {
            // Move to the start.
            SnapRadians(starting.ToList());
            
            // If already at the destination, no need to move and return.
            if (Reached(targetPosition, targetRotation))
            {
                hasReached = true;
                return starting.ToList();
            }
            
            // Default values to check against.
            List<float>best = null;
            float bestTime = float.MaxValue;

            // Go for every Bio IK attempt.
            for (int attempt = 0; attempt < attempts; attempt++)
            {
                // Move to the Bio IK solution.
                SnapRadians(BioIkSolve(targetPosition, targetRotation, starting, maxGenerations, new((uint) Random.Range(1, int.MaxValue)), out bool reached, out _, out _));
                PhysicsStep();

                // Only care if reached.
                if (!reached)
                {
                    continue;
                }

                List<float> solution = GetJoints();

                // If the new joints move faster than the exiting best solution, update it.
                float time = CalculateTime(starting, solution);
                if (time >= bestTime)
                {
                    continue;
                }

                best = solution;
                bestTime = time;
            }

            // No solution was found.
            if (best == null)
            {
                hasReached = false;
                return starting.ToList();
            }
            
            // Move to the best solution.
            hasReached = true;
            SnapRadians(best);
            PhysicsStep();
            return GetJoints();
        }

        /// <summary>
        /// Run Bio IK.
        /// </summary>
        /// <param name="targetPosition">The position to reach.</param>
        /// <param name="targetRotation">The rotation to reach.</param>
        /// <param name="starting">The starting joint values.</param>
        /// <param name="maxGenerations">The number of generations for each attempt.</param>
        /// <param name="random">The random number generator.</param>
        /// <param name="reached">If the robot reached the destination.</param>
        /// <param name="generations">The number of generations needed to reach the solution.</param>
        /// <param name="fitness">How fit the result was with smaller values being closer to the solution.</param>
        /// <returns>The joints to move the robot to.</returns>
        private List<float> BioIkSolve(Vector3 targetPosition, Quaternion targetRotation, IReadOnlyList<float> starting, int maxGenerations, Unity.Mathematics.Random random, out bool reached, out int generations, out float fitness)
        {
            double[] seed = new double[_limits.Length];
            for (int i = 0; i < _limits.Length; i++)
            {
                seed[i] = starting[i];
            }
            
            double[] solution = _bioIk.Optimise(seed, targetPosition, targetRotation, maxGenerations, random, out reached, out generations, out double f);
            fitness = (float) f;
            return solution.Select(t => (float) t).ToList();
        }

        /// <summary>
        /// Stop the robot at a position.
        /// </summary>
        /// <param name="radians">The joint values in radians to stop at.</param>
        private void Snap(IEnumerable<float> radians)
        {
            List<float> list = radians.ToList();
            Root.SetDriveTargets(list);
            Root.SetJointVelocities(_zeros);
            Root.SetJointForces(_zeros);
            Root.SetJointPositions(list);
        }

        /// <summary>
        /// Scale joint values between zero and one for network inference.
        /// </summary>
        /// <param name="joints">The joint values.</param>
        /// <returns>The joints scaled for network inference.</returns>
        public List<float> NetScaledJoints(List<float> joints)
        {
            for (int i = 0; i < joints.Count; i++)
            {
                joints[i] = (joints[i] - _limits[i].lower) / (_limits[i].upper - _limits[i].lower);
            }

            return joints;
        }
        
        /// <summary>
        /// Scale network inference back to network values.
        /// </summary>
        /// <param name="joints">The joints as received from network inference.</param>
        /// <returns>The joints scaled to their real values.</returns>
        public List<float> ResultsScaled(List<float> joints)
        {
            for (int i = 0; i < joints.Count; i++)
            {
                joints[i] = math.clamp(joints[i] * (_limits[i].upper - _limits[i].lower) + _limits[i].lower, _limits[i].lower, _limits[i].upper);
            }

            return joints;
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
            starting = NetScaledJoints(starting);
            float[] inputs = new float[starting.Count + 7];
            for (int i = 0; i < starting.Count; i++)
            {
                inputs[i] = starting[i];
            }

            // Get relative values.
            targetPosition = RelativePosition(targetPosition);
            targetRotation = RelativeRotation(targetRotation);
            
            // Add scaled position.
            inputs[starting.Count] = (targetPosition.x + 1) / 2;
            inputs[starting.Count + 1] = (targetPosition.y + 1) / 2;
            inputs[starting.Count + 2] = (targetPosition.z + 1) / 2;
            
            // Add scaled rotation.
            inputs[starting.Count + 3] = (targetRotation.x + 1) / 2;
            inputs[starting.Count + 4] = (targetRotation.y + 1) / 2;
            inputs[starting.Count + 5] = (targetRotation.z + 1) / 2;
            inputs[starting.Count + 6] = (targetRotation.w + 1) / 2;

            return inputs;
        }

        /// <summary>
        /// Request a solution.
        /// </summary>
        /// <param name="targetPosition">The position to reach.</param>
        /// <param name="targetRotation">The rotation to reach.</param>
        /// <param name="maxGenerations">The number of generations the solving mode can be run for.</param>
        /// <param name="reached">If the robot reached the destination.</param>
        /// <param name="moveTime">How long it took for the robot to perform the move.</param>
        /// <param name="generations">The number of generations needed to reach the solution.</param>
        /// <param name="seed">The seed for random number generation</param>
        /// <returns>The joints to move the robot to.</returns>
        private List<float> RequestSolution(Vector3 targetPosition, Quaternion targetRotation, int maxGenerations, out bool reached, out float moveTime, out int generations, uint seed = 0)
        {
            // If no seed was passed, create a random one.
            if (seed == 0)
            {
                seed = (uint) Random.Range(1, int.MaxValue);
            }
            
            // Initialize random number generation.
            Unity.Mathematics.Random random = new(seed);
            
            // Call the requested mode.
            switch (mode)
            {
                case SolverMode.Network:
                    generations = 0;
                    return SolutionNetwork(targetPosition, targetRotation, out reached, out moveTime);
                case SolverMode.BioIk:
                    return SolutionBioIk(targetPosition, targetRotation, maxGenerations, random, out reached, out moveTime, out generations);
                case SolverMode.FusionIk:
                    return SolutionFusionIk(targetPosition, targetRotation, maxGenerations, random, out reached, out moveTime, out generations);
                case SolverMode.GreedyFusionIk:
                    return SolutionGreedyFusionIk(targetPosition, targetRotation, maxGenerations, random, out reached, out moveTime, out generations);
                case SolverMode.ExhaustiveFusionIk:
                default:
                    return SolutionExhaustiveFusionIk(targetPosition, targetRotation, maxGenerations, random, out reached, out moveTime, out generations);
            }
        }

        /// <summary>
        /// Perform inference on the networks.
        /// </summary>
        /// <param name="targetPosition">The position to reach.</param>
        /// <param name="targetRotation">The rotation to reach.</param>
        /// <param name="reached">If the robot reached the destination.</param>
        /// <param name="moveTime">How long it took for the robot to perform the move.</param>
        /// <returns>The joints to move the robot to.</returns>
        private List<float> SolutionNetwork(Vector3 targetPosition, Quaternion targetRotation, out bool reached, out float moveTime)
        {
            List<float> starting = GetJoints();

            List<float> results = RunNetwork(PrepareInputs(targetPosition, targetRotation, GetJoints()).ToList());
            reached = Reached(targetPosition, targetRotation);
            
            moveTime = CalculateTime(starting, results);
            return results;
        }

        /// <summary>
        /// Perform Bio IK.
        /// </summary>
        /// <param name="targetPosition">The position to reach.</param>
        /// <param name="targetRotation">The rotation to reach.</param>
        /// <param name="maxGenerations">The number of generations the solving mode can be run for.</param>
        /// <param name="random">The random number generator.</param>
        /// <param name="reached">If the robot reached the destination.</param>
        /// <param name="moveTime">How long it took for the robot to perform the move.</param>
        /// <param name="generations">The number of generations needed to reach the solution.</param>
        /// <returns>The joints to move the robot to.</returns>
        private List<float> SolutionBioIk(Vector3 targetPosition, Quaternion targetRotation, int maxGenerations, Unity.Mathematics.Random random, out bool reached, out float moveTime, out int generations)
        {
            List<float> starting = GetJoints();
            
            List<float> results = BioIkSolve(targetPosition, targetRotation, starting, maxGenerations, random, out reached, out generations, out _);
            
            moveTime = CalculateTime(starting, results);
            return results;
        }

        /// <summary>
        /// Perform Exhaustive Fusion IK.
        /// </summary>
        /// <param name="targetPosition">The position to reach.</param>
        /// <param name="targetRotation">The rotation to reach.</param>
        /// <param name="maxGenerations">The number of generations the solving mode can be run for.</param>
        /// <param name="random">The random number generator.</param>
        /// <param name="reached">If the robot reached the destination.</param>
        /// <param name="moveTime">How long it took for the robot to perform the move.</param>
        /// <param name="generations">The number of generations needed to reach the solution.</param>
        /// <returns>The joints to move the robot to.</returns>
        private List<float> SolutionExhaustiveFusionIk(Vector3 targetPosition, Quaternion targetRotation, int maxGenerations, Unity.Mathematics.Random random, out bool reached, out float moveTime, out int generations)
        {
            // If only one generation, it is the same as standard Fusion IK so simply run it.
            if (maxGenerations == 1)
            {
                return SolutionFusionIk(targetPosition, targetRotation, maxGenerations, random, out reached, out moveTime, out generations);
            }
            
            List<float> starting = GetJoints();

            // Split the generations.
            int networkGenerations = maxGenerations / 2;
            
            // Get the results of both Fusion IK and Bio IK.
            List<float> fusion = BioIkSolve(targetPosition, targetRotation, RunNetwork(PrepareInputs(targetPosition, targetRotation, GetJoints())), networkGenerations, random, out bool fusionReached, out networkGenerations, out float fusionFitness);
            List<float> regular = BioIkSolve(targetPosition, targetRotation, starting, maxGenerations - networkGenerations, random, out bool regularReached, out generations, out float regularFitness);

            generations += networkGenerations;

            reached = fusionReached || regularReached;

            // Return the best option.
            switch (fusionReached)
            {
                // Fusion reached and regular did not.
                case true when !regularReached:
                    moveTime = CalculateTime(starting, fusion);
                    return fusion;
                // Fusion did not reach and regular did.
                case false when regularReached:
                    moveTime = CalculateTime(starting, regular);
                    return regular;
                // Both reached so choose the fastest one.
                case true:
                    float fusionTime = CalculateTime(starting, fusion);
                    float regularTime = CalculateTime(starting, regular);
                    moveTime = fusionTime <= regularTime ? fusionTime : regularTime;
                    return fusionTime <= regularTime ? fusion : regular;
                // Neither reached so choose the closest one.
                default:
                    moveTime = fusionFitness <= regularFitness ? CalculateTime(starting, fusion) : CalculateTime(starting, regular);
                    return fusionFitness <= regularFitness ? fusion : regular;
            }
        }

        /// <summary>
        /// Perform Greedy Fusion IK.
        /// </summary>
        /// <param name="targetPosition">The position to reach.</param>
        /// <param name="targetRotation">The rotation to reach.</param>
        /// <param name="maxGenerations">The number of generations the solving mode can be run for.</param>
        /// <param name="random">The random number generator.</param>
        /// <param name="reached">If the robot reached the destination.</param>
        /// <param name="moveTime">How long it took for the robot to perform the move.</param>
        /// <param name="generations">The number of generations needed to reach the solution.</param>
        /// <returns>The joints to move the robot to.</returns>
        private List<float> SolutionGreedyFusionIk(Vector3 targetPosition, Quaternion targetRotation, int maxGenerations, Unity.Mathematics.Random random, out bool reached, out float moveTime, out int generations)
        {
            // If only one generation, it is the same as standard Fusion IK so simply run it.
            if (maxGenerations == 1)
            {
                return SolutionFusionIk(targetPosition, targetRotation, maxGenerations, random, out reached, out moveTime, out generations);
            }
            
            List<float> starting = GetJoints();

            // Split the generations.
            int networkGenerations = maxGenerations / 2;
            
            // Run the Fusion IK.
            List<float> fusion = BioIkSolve(targetPosition, targetRotation, RunNetwork(PrepareInputs(targetPosition, targetRotation, GetJoints())), networkGenerations, random, out reached, out networkGenerations, out float fusionFitness);

            // If it reached, return.
            if (reached)
            {
                generations = networkGenerations;
                moveTime = CalculateTime(starting, fusion);
                return fusion;
            }

            // Otherwise, run Bio IK.
            List<float> regular = BioIkSolve(targetPosition, targetRotation, starting, maxGenerations - networkGenerations, random, out reached, out generations, out float regularFitness);
            generations += networkGenerations;
            
            // If it reached, return.
            if (reached)
            {
                moveTime = CalculateTime(starting, regular);
                return regular;
            }
            
            // Otherwise, return whichever move was closest.
            moveTime = fusionFitness <= regularFitness ? CalculateTime(starting, fusion) : CalculateTime(starting, regular);
            return fusionFitness <= regularFitness ? fusion : regular;
        }
        
        /// <summary>
        /// Perform Fusion IK.
        /// </summary>
        /// <param name="targetPosition">The position to reach.</param>
        /// <param name="targetRotation">The rotation to reach.</param>
        /// <param name="maxGenerations">The number of generations the solving mode can be run for.</param>
        /// <param name="random">The random number generator.</param>
        /// <param name="reached">If the robot reached the destination.</param>
        /// <param name="moveTime">How long it took for the robot to perform the move.</param>
        /// <param name="generations">The number of generations needed to reach the solution.</param>
        /// <returns>The joints to move the robot to.</returns>
        private List<float> SolutionFusionIk(Vector3 targetPosition, Quaternion targetRotation, int maxGenerations, Unity.Mathematics.Random random, out bool reached, out float moveTime, out int generations)
        {
            List<float> starting = GetJoints();
            
            List<float> results = BioIkSolve(targetPosition, targetRotation, RunNetwork(PrepareInputs(targetPosition, targetRotation, GetJoints())), maxGenerations, random, out reached, out generations, out _);

            moveTime = CalculateTime(starting, results);
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
            List<float> outputs = new(_workers.Length);
            
            // Go through every joint's network.
            for (int i = 0; i < _workers.Length; i++)
            {
                // Run the current joint network.
                Tensor input = new(1, 1, 1, forwards.Length, forwards, "INPUTS");
                Tensor output = _workers[i].Execute(input).PeekOutput();
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
            _networks = new Model[_limits.Length];
            _workers = new IWorker[_networks.Length];
            bool networkValid = true;
            for (int i = 0; i < _networks.Length; i++)
            {
                _networks[i] = properties.CompiledNetwork(i);
                if (_networks[i] != null)
                {
                    _workers[i] = WorkerFactory.CreateWorker(_networks[i]);
                    continue;
                }

                networkValid = false;
                break;
            }

            // Do an initial network run as it is slower on the first inference.
            if (networkValid)
            {
                RunNetwork(PrepareInputs(Vector3.zero, Quaternion.identity, GetJoints()).ToList());
            }

            // Configure the Bio IK solver.
            _bioIk = new(this, properties.Population, properties.Elites, properties.Steps);
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
            Snap(delta);
        }

        private void OnDestroy()
        {
            // Clean up network runners.
            for (int i = 0; i < _workers.Length; i++)
            {
                _workers[i]?.Dispose();
            }
        }
    }
}