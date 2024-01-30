using Unity.Barracuda;
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

        /// <summary>
        /// How accurate in meters the robot can repeat a movement.
        /// </summary>
        [Header("Movement")]
        [Tooltip("How accurate in meters the robot can repeat a movement.")]
        [Min(0)]
        [SerializeField]
        private float repeatability = 8e-5f;
        
        /// <summary>
        /// The population size of each generation during Bio IK evolution.
        /// </summary>
        [Tooltip("The population size of each generation during Bio IK evolution.")]
        [Min(1)]
        [SerializeField]
        private int population = 120;
        
        /// <summary>
        /// The number of elites in each generation during Bio IK evolution.
        /// </summary>
        [Tooltip("The number of elites in each generation during Bio IK evolution.")]
        [Min(1)]
        [SerializeField]
        private int elites = 3;

        /// <summary>
        /// The number of generations after which to do the iterative process in iterative Fusion IK.
        /// </summary>
        [Tooltip("The number of generations after which to do the iterative process in iterative Fusion IK.")]
        [Min(1)]
        [SerializeField]
        private int generations = 10;

        /// <summary>
        /// The number of the best population members to keep during each iterative process in iterative Fusion IK.
        /// </summary>
        [Tooltip("The number of the best population members to keep during each iterative process in iterative Fusion IK.")]
        [Min(1)]
        [SerializeField]
        private int kept = 10;

        /// <summary>
        /// Standard network.
        /// </summary>
        [Header("Networks")]
        [Tooltip("Standard network.")]
        public NNModel standardNetwork;

        /// <summary>
        /// Minimal network.
        /// </summary>
        [Tooltip("Minimal network.")]
        public NNModel minimalNetwork;

        /// <summary>
        /// Material to apply to the best robot during visualization.
        /// </summary>
        [Header("Materials")]
        [Tooltip("Material to apply to the best robot during visualization.")]
        [SerializeField]
        private Material normal;
        
        /// <summary>
        /// Material to apply to the non-best robots during visualization.
        /// </summary>
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