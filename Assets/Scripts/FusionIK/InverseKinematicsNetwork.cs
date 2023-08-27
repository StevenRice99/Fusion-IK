using System;
using System.Linq;
using Unity.Barracuda;
using UnityEngine;

namespace FusionIK
{
    /// <summary>
    /// Store the networks for a robot.
    /// </summary>
    [Serializable]
    public class InverseKinematicsNetwork
    {
        /// <summary>
        /// A network to run inference on.
        /// </summary>
        /// <param name="joint">The joint network that is desired.</param>
        /// <returns>The joint network that is desired.</returns>
        public Model CompiledNetwork(int joint) => networks.Length > 0 && joint < networks.Length && networks[joint] != null ? ModelLoader.Load(networks[joint]) : null;
        
        /// <summary>
        /// Check if all networks are present.
        /// </summary>
        public bool NetworksCheck => networks.Length > 0 && networks.All(model => model != null);
        
        [Tooltip("Network to control the joints.")]
        public NNModel[] networks;
    }
}