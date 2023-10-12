using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Barracuda;
using Unity.Mathematics;
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
        /// The number of generations after which to do the iterative process in iterative Fusion IK.
        /// </summary>
        public int Generations => generations;

        /// <summary>
        /// The number of the best population members to keep during each iterative process in iterative Fusion IK.
        /// </summary>
        public int Kept => kept;

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
        
        [Tooltip("The population size of each generation during Bio IK evolution.")]
        [Min(1)]
        [SerializeField]
        private int population = 120;
        
        [Tooltip("The number of elites in each generation during Bio IK evolution.")]
        [Min(1)]
        [SerializeField]
        private int elites = 3;

        [Tooltip("The number of generations after which to do the iterative process in iterative Fusion IK.")]
        [Min(1)]
        [SerializeField]
        private int generations = 10;

        [Tooltip("The number of the best population members to keep during each iterative process in iterative Fusion IK.")]
        [Min(1)]
        [SerializeField]
        private int kept = 10;

        [Header("Networks")]
        [Tooltip("Standard network.")]
        public NNModel standardNetwork;

        [Tooltip("Minimal network.")]
        public NNModel minimalNetwork;

        [Header("Materials")]
        [Tooltip("Material to apply to the best robot during visualization.")]
        [SerializeField]
        private Material normal;
        
        [Tooltip("Material to apply to the non-best robots during visualization.")]
        [SerializeField]
        private Material transparent;

        /// <summary>
        /// Check if the standard network is valid.
        /// </summary>
        public bool StandardNetworkValid => standardNetwork != null;

        /// <summary>
        /// Check if the minimal network is valid.
        /// </summary>
        public bool MinimalNetworkValid => minimalNetwork != null;

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
        /// Setup the workers.
        /// </summary>
        /// <param name="robot">The robot.</param>
        public void SetupWorkers(Robot robot)
        {
            // Setup large network.
            if (StandardNetworkValid)
            {
                _standardWorker = WorkerFactory.CreateWorker(WorkerFactory.Type.CSharpBurst, ModelLoader.Load(standardNetwork));

                if (_standardWorker != null)
                {
                    RunNetwork(robot, Vector3.zero, Quaternion.identity, robot.GetJoints());
                }
                else
                {
                    _standardWorker?.Dispose();
                }
            }

            // Setup minimal network.
            if (MinimalNetworkValid)
            {
                _minimalWorker = WorkerFactory.CreateWorker(WorkerFactory.Type.CSharpBurst, ModelLoader.Load(minimalNetwork));

                if (_minimalWorker != null)
                {
                    RunNetwork(robot, Vector3.zero, Quaternion.identity);
                }
                else
                {
                    _minimalWorker?.Dispose();
                }
            }
        }

        /// <summary>
        /// Run the network inference.
        /// </summary>
        /// <param name="robot">The robot to process.</param>
        /// <param name="position">The position to reach.</param>
        /// <param name="rotation">The rotation to reach.</param>
        /// <param name="starting">The joints to start at.</param>
        /// <returns>The joints to move the robot to.</returns>
        public List<float> RunNetwork(Robot robot, Vector3 position, Quaternion rotation, List<float> starting = null)
        {
            // Get initial input values and prepare for outputs.
            float[] inputs = robot.PrepareInputs(position, rotation, starting);
            Tensor input = new(1, 1, 1, inputs.Length, inputs, "INPUTS");

            IWorker worker = robot.minimal ? _minimalWorker : _standardWorker;

            // Run the current joint network.
            worker.Execute(input);
            worker.FlushSchedule(true);
            Tensor output = worker.PeekOutput();
            input.Dispose();
            
            // Add the output and replace the input for the next joint's network.
            List<float> outputs = new(robot.Limits.Length);
            for (int i = 0; i < robot.Limits.Length; i++)
            {
                outputs.Add(math.clamp(output[0, 0, 0, i], 0, 1));
            }
            output.Dispose();

            return robot.ResultsScaled(outputs);
        }

        private void OnValidate()
        {
            // Cannot have more elites than the population.
            if (elites > population)
            {
                elites = population;
            }

            if (kept > population)
            {
                kept = population;
            }
        }
    }
}