using Unity.Barracuda;
using UnityEngine;

namespace FusionIK
{
    /// <summary>
    /// Store the networks for a robot.
    /// </summary>
    [CreateAssetMenu(fileName = "Inverse Kinematics Network", menuName = "Fusion-IK/Inverse Kinematics Network", order = 1)]
    public class InverseKinematicsNetwork : ScriptableObject
    {
        /// <summary>
        /// A network to run inference on.
        /// </summary>
        /// <param name="joint">The joint network that is desired.</param>
        /// <returns>The joint network that is desired.</returns>
        public Model CompiledNetwork(int joint) => networks.Length > 0 && joint < networks.Length && networks[joint] != null ? ModelLoader.Load(networks[joint]) : null;
        
        [Tooltip("Networks to control the joints.")]
        public NNModel[] networks;
    }
}