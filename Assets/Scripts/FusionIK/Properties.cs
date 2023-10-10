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

        [Header("Movement")]
        [Tooltip("How accurate in meters the robot can repeat a movement.")]
        [Min(0)]
        [SerializeField]
        private float repeatability = 8e-5f;

        [Tooltip("Network to control the robot.")]
        public NNModel network;

        [Header("Bio IK")]
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

        [Header("Materials")]
        [Tooltip("Material to apply to the best robot during visualization.")]
        [SerializeField]
        private Material normal;
        
        [Tooltip("Material to apply to the non-best robots during visualization.")]
        [SerializeField]
        private Material transparent;

        /// <summary>
        /// Check if networks are valid.
        /// </summary>
        public bool NetworksValid => network != null;
        
        /// <summary>
        /// A network to run inference on.
        /// </summary>
        /// <returns>The joint network at a given index that is desired.</returns>
        public Model CompiledNetwork => network != null ? ModelLoader.Load(network) : null;

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