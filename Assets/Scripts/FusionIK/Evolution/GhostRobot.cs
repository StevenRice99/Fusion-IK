using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace FusionIK.Evolution
{
	/// <summary>
	/// Used to help with Bio IK calculations.
	/// </summary>
	public class GhostRobot
	{
		/// <summary>
		/// All motions in the Bio IK chain.
		/// </summary>
		public MotionPtr[] motionPointers = Array.Empty<MotionPtr>();

		/// <summary>
		/// Degrees of freedom.
		/// </summary>
		public readonly int dof;
		
		/// <summary>
		/// The robot.
		/// </summary>
		private readonly Robot _robot;

		/// <summary>
		/// World offset position.
		/// </summary>
		private float _opx, _opy, _opz;
		
		/// <summary>
		/// World offset rotation.
		/// </summary>
		private float _orx, _ory, _orz, _orw;
		
		/// <summary>
		/// All nodes that make up the ghost robot.
		/// </summary>
		private Node[] _nodes = Array.Empty<Node>();

		/// <summary>
		/// Current configuration.
		/// </summary>
		private readonly float[] _configuration;
		
		/// <summary>
		/// Current gradient.
		/// </summary>
		private readonly double[] _gradient;
		
		/// <summary>
		/// Current loss.
		/// </summary>
		private double _loss;

		/// <summary>
		/// Simulated position.
		/// </summary>
		private float _px, _py, _pz;
		
		/// <summary>
		/// Simulated rotation.
		/// </summary>
		private float _rx, _ry, _rz, _rw;
		
		/// <summary>
		/// Simulated loss.
		/// </summary>
		private double _simulatedLoss;
		
		/// <summary>
		/// Target position.
		/// </summary>
		private float _tpx, _tpy, _tpz;

		/// <summary>
		/// Target rotation.
		/// </summary>
		private float _trx, _try, _trz, _trw;

		/// <summary>
		/// Configure a new ghost robot based off an actual robot chain.
		/// </summary>
		/// <param name="robot"></param>
		public GhostRobot(Robot robot)
		{
			_robot = robot;

			// Add a node for every ghost joint.
			AddNode(_robot.GhostJoints[0], null);
			GhostJoint current = _robot.GhostJoints[0].child;
			while (current != null)
			{
				AddNode(current, _nodes[^1]);
				current = current.child;
			}

			dof = motionPointers.Length;
			_configuration = new float[motionPointers.Length];
			_gradient = new double[motionPointers.Length];

			// Update the configuration.
			for (int i = 0; i < _configuration.Length; i++)
			{
				_configuration[i] = motionPointers[i].motion.GetTargetValue();
			}

			// Update offset from world to the root.
			if (_robot.GhostJoints[0].transform.root == _robot.transform)
			{
				_opx = _opy = _opz = _orx = _ory = _orz = 0f;
				_orw = 1f;
			}
			else
			{
				Transform parent = _robot.GhostJoints[0].transform.parent;
				Vector3 p = parent.position;
				Quaternion r = parent.rotation;
				_opx = p.x; _opy = p.y; _opz = p.z;
				_orx = r.x; _ory = r.y; _orz = r.z; _orw = r.w;
			}

			// Update the nodes.
			_nodes[0].Refresh();
		}

		/// <summary>
		/// Set the target position.
		/// </summary>
		/// <param name="position">The position the robot should try to reach.</param>
		public void SetTargetPosition(Vector3 position)
		{
			_tpx = position.x;
			_tpy = position.y;
			_tpz = position.z;
		}

		/// <summary>
		/// Set the target rotation.
		/// </summary>
		/// <param name="rotation">The rotation the robot should try to reach.</param>
		public void SetTargetRotation(Quaternion rotation)
		{
			_trx = rotation.x;
			_try = rotation.y;
			_trz = rotation.z;
			_trw = rotation.w;
		}

		/// <summary>
		/// Create a copy of another ghost robot.
		/// </summary>
		/// <param name="ghostRobot">The ghost robot to copy.</param>
		public void CopyFrom(GhostRobot ghostRobot)
		{
			_opx = ghostRobot._opx;
			_opy = ghostRobot._opy;
			_opz = ghostRobot._opz;
			_orx = ghostRobot._orx;
			_ory = ghostRobot._ory;
			_orz = ghostRobot._orz;
			_orw = ghostRobot._orw;
			for (int i = 0; i < dof; i++)
			{
				_configuration[i] = ghostRobot._configuration[i];
				_gradient[i] = ghostRobot._gradient[i];
			}
			_px = ghostRobot._px;
			_py = ghostRobot._py;
			_pz = ghostRobot._pz;
			_rx = ghostRobot._rx;
			_ry = ghostRobot._ry;
			_rz = ghostRobot._rz;
			_rw = ghostRobot._rw;
			_loss = ghostRobot._loss;
			_simulatedLoss = ghostRobot._simulatedLoss;
			for (int i = 0; i < _nodes.Length; i++)
			{
				_nodes[i].wpx = ghostRobot._nodes[i].wpx;
				_nodes[i].wpy = ghostRobot._nodes[i].wpy;
				_nodes[i].wpz = ghostRobot._nodes[i].wpz;
				_nodes[i].wrx = ghostRobot._nodes[i].wrx;
				_nodes[i].wry = ghostRobot._nodes[i].wry;
				_nodes[i].wrz = ghostRobot._nodes[i].wrz;
				_nodes[i].wrw = ghostRobot._nodes[i].wrw;
				_nodes[i].wsx = ghostRobot._nodes[i].wsx;
				_nodes[i].wsy = ghostRobot._nodes[i].wsy;
				_nodes[i].wsz = ghostRobot._nodes[i].wsz;

				_nodes[i].lpx = ghostRobot._nodes[i].lpx;
				_nodes[i].lpy = ghostRobot._nodes[i].lpy;
				_nodes[i].lpz = ghostRobot._nodes[i].lpz;
				_nodes[i].lrx = ghostRobot._nodes[i].lrx;
				_nodes[i].lry = ghostRobot._nodes[i].lry;
				_nodes[i].lrz = ghostRobot._nodes[i].lrz;
				_nodes[i].lrw = ghostRobot._nodes[i].lrw;
				
				_nodes[i].xValue = ghostRobot._nodes[i].xValue;
				_nodes[i].yValue = ghostRobot._nodes[i].yValue;
				_nodes[i].zValue = ghostRobot._nodes[i].zValue;
			}
			_tpx = ghostRobot._tpx;
			_tpy = ghostRobot._tpy;
			_tpz = ghostRobot._tpz;
			_trx = ghostRobot._trx;
			_try = ghostRobot._try;
			_trz = ghostRobot._trz;
			_trw = ghostRobot._trw;
		}

		/// <summary>
		/// Check if the ghost robot has reached the target.
		/// </summary>
		/// <param name="configuration">The joint values.</param>
		/// <param name="targetPosition">The target position.</param>
		/// <param name="targetRotation">The target rotation.</param>
		/// <returns>True if reached, false otherwise.</returns>
		public bool CheckConvergence(double[] configuration, Vector3 targetPosition, Quaternion targetRotation)
		{
			ForwardKinematics(configuration);

			Vector3 position = new((float) _nodes[^1].wpx, (float) _nodes[^1].wpy, (float) _nodes[^1].wpz);
			Quaternion rotation = new((float) _nodes[^1].wrx, (float) _nodes[^1].wry, (float) _nodes[^1].wrz, (float) _nodes[^1].wrw);

			return _robot.Reached(targetPosition, targetRotation, position, rotation);
		}

		/// <summary>
		/// Computes the loss as the root mean error squared.
		/// </summary>
		/// <param name="configuration">The joint values.</param>
		/// <returns>The loss.</returns>
		public double ComputeLoss(double[] configuration)
		{
			ForwardKinematics(configuration);
			Node node = motionPointers[^1].node;
			_loss = ComputeLoss(node.wpx, node.wpy, node.wpz, node.wrx, node.wry, node.wrz, node.wrw);
			return math.sqrt(_loss);
		}

		/// <summary>
		/// Computes the loss as the root mean error squared.
		/// </summary>
		/// <param name="configuration">The joint values.</param>
		/// <returns>The loss.</returns>
		public double ComputeLoss(List<float> configuration)
		{
			ForwardKinematics(configuration);
			Node node = motionPointers[^1].node;
			_loss = ComputeLoss(node.wpx, node.wpy, node.wpz, node.wrx, node.wry, node.wrz, node.wrw);
			return math.sqrt(_loss);
		}

		/// <summary>
		/// Computes the loss as the root mean error squared.
		/// </summary>
		/// <param name="apx">X position.</param>
		/// <param name="apy">Y position.</param>
		/// <param name="apz">Z position.</param>
		/// <param name="arx">X rotation.</param>
		/// <param name="ary">Y rotation.</param>
		/// <param name="arz">Z rotation.</param>
		/// <param name="arw">W rotation.</param>
		/// <returns>The loss squared.</returns>
		private double ComputeLoss(double apx, double apy, double apz, double arx, double ary, double arz, double arw)
		{
			double pos = _robot.Rescaling * ((_tpx - apx) * (_tpx - apx) + (_tpy - apy) * (_tpy - apy) + (_tpz - apz) * (_tpz - apz));
			
			double d = arx * _trx + ary * _try + arz * _trz + arw * _trw;
			switch (d)
			{
				case < 0.0:
					d = -d;
					if (d > 1.0)
					{
						d = 1.0;
					}

					break;
				case > 1.0:
					d = 1.0;
					break;
			}
			double rot = 2.0 * math.acos(d);
			rot *= rot;
			
			return pos + rot;
		}

		/// <summary>
		/// Compute the gradient.
		/// </summary>
		/// <param name="configuration">The joint values.</param>
		/// <param name="resolution">The error resolution.</param>
		/// <returns>The gradient for each joint value.</returns>
		public double[] ComputeGradient(double[] configuration, double resolution)
		{
			double oldLoss = ComputeLoss(configuration);
			for (int j = 0; j < dof; j++)
			{
				_configuration[j] += (float) resolution;
				motionPointers[j].node.SimulateModification(_configuration);
				_configuration[j] -= (float) resolution;
				_gradient[j] = (math.sqrt(_simulatedLoss) - oldLoss) / resolution;
			}
			return _gradient;
		}

		/// <summary>
		/// Run forward kinematics on the ghost robot.
		/// </summary>
		/// <param name="configuration">The joint values.</param>
		private void ForwardKinematics(double[] configuration)
		{
			for (int i = 0; i < _configuration.Length; i++)
			{
				// Cast to float to limit accuracy to how it would be in Unity.
				_configuration[i] = (float) configuration[i];
			}
			_nodes[0].FeedForwardConfiguration(configuration);
		}

		/// <summary>
		/// Run forward kinematics on the ghost robot.
		/// </summary>
		/// <param name="configuration">The joint values.</param>
		private void ForwardKinematics(List<float> configuration)
		{
			double[] doubles = new double[_configuration.Length];
			for (int i = 0; i < _configuration.Length; i++)
			{
				// Cast to float to limit accuracy to how it would be in Unity.
				_configuration[i] = configuration[i];
				doubles[i] = configuration[i];
			}
			_nodes[0].FeedForwardConfiguration(doubles);
		}

		/// <summary>
		/// Add a node to the ghost robot.
		/// </summary>
		/// <param name="joint">The joint attached to this node.</param>
		/// <param name="parent">The parent node.</param>
		private void AddNode(GhostJoint joint, Node parent)
		{
			Node node = new(this, parent, joint);

			if (node.ghostJoint != null)
			{
				if (node.ghostJoint.GetDoF() == 0)
				{
					node.ghostJoint = null;
				}
				else
				{
					if (node.ghostJoint.x.enabled)
					{
						MotionPtr motionPtr = new(node.ghostJoint.x, node, motionPointers.Length);
						Array.Resize(ref motionPointers, motionPointers.Length + 1);
						motionPointers[^1] = motionPtr;
						node.xEnabled = true;
						node.xIndex = motionPtr.index;
					}
					if (node.ghostJoint.y.enabled)
					{
						MotionPtr motionPtr = new(node.ghostJoint.y, node, motionPointers.Length);
						Array.Resize(ref motionPointers, motionPointers.Length + 1);
						motionPointers[^1] = motionPtr;
						node.yEnabled = true;
						node.yIndex = motionPtr.index;
					}
					if (node.ghostJoint.z.enabled)
					{
						MotionPtr motionPtr = new(node.ghostJoint.z, node, motionPointers.Length);
						Array.Resize(ref motionPointers, motionPointers.Length + 1);
						motionPointers[^1] = motionPtr;
						node.zEnabled = true;
						node.zIndex = motionPtr.index;
					}
				}
			}

			Array.Resize(ref _nodes, _nodes.Length + 1);
			_nodes[^1] = node;
		}

		/// <summary>
		/// Store data for a node in the ghost robot chain.
		/// </summary>
		public class Node
		{
			/// <summary>
			/// The ghost robot this node is a part of.
			/// </summary>
			private readonly GhostRobot _ghostRobot;
			
			/// <summary>
			/// Parent node in the chain.
			/// </summary>
			private readonly Node _parent;
			
			/// <summary>
			/// Child node in the chain.
			/// </summary>
			private Node _child;
			
			/// <summary>
			/// Transform of the node.
			/// </summary>
			private readonly Transform _transform;
			
			/// <summary>
			/// Joint attached to the node.
			/// </summary>
			public GhostJoint ghostJoint;

			/// <summary>
			/// World position.
			/// </summary>
			public double wpx, wpy, wpz;
			
			/// <summary>
			/// World rotation.
			/// </summary>
			public double wrx, wry, wrz, wrw;
			
			/// <summary>
			/// World scale.
			/// </summary>
			public double wsx, wsy, wsz;
			
			/// <summary>
			/// Local position.
			/// </summary>
			public double lpx, lpy, lpz;
			
			/// <summary>
			/// Local rotation.
			/// </summary>
			public double lrx, lry, lrz, lrw;

			/// <summary>
			/// Which axis are enabled.
			/// </summary>
			public bool xEnabled, yEnabled, zEnabled;
			
			/// <summary>
			/// The index of each axis.
			/// </summary>
			public int xIndex = -1, yIndex = -1, zIndex = -1;
			
			/// <summary>
			/// The value of each axis.
			/// </summary>
			public double xValue, yValue, zValue;

			/// <summary>
			/// Setup the node.
			/// </summary>
			/// <param name="ghostRobot">The ghost robot this node is a part of.</param>
			/// <param name="parent">The parent node.</param>
			/// <param name="ghostJoint">The joint attached to this node.</param>
			public Node(GhostRobot ghostRobot, Node parent, GhostJoint ghostJoint)
			{
				_ghostRobot = ghostRobot;
				_parent = parent;
				if (_parent != null)
				{
					_parent._child = this;
				}

				_transform = ghostJoint.transform;
				this.ghostJoint = ghostJoint;
			}

			/// <summary>
			/// Update transform data.
			/// </summary>
			public void Refresh()
			{
				xValue = ghostJoint.x.GetTargetValue();
				yValue = ghostJoint.y.GetTargetValue();
				zValue = ghostJoint.z.GetTargetValue();
				ghostJoint.ComputeLocalTransformation(xValue, yValue, zValue, out lpx, out lpy, out lpz, out lrx, out lry, out lrz, out lrw);
				
				Vector3 ws = _transform.lossyScale;
				wsx = ws.x;
				wsy = ws.y;
				wsz = ws.z;
				
				ComputeWorldTransformation();
				
				_child?.Refresh();
			}

			/// <summary>
			/// Updates local and world transform and feeds forward to the child node.
			/// </summary>
			/// <param name="configuration">The joint configuration.</param>
			/// <param name="updateWorld">If world position should be updated.</param>
			public void FeedForwardConfiguration(double[] configuration, bool updateWorld = false)
			{
				// Assume no local update is required unless an axis is enabled.
				bool updateLocal = false;

				if (xEnabled)
				{
					xValue = configuration[xIndex];
					updateLocal = true;
				}
				
				if (yEnabled)
				{
					yValue = configuration[yIndex];
					updateLocal = true;
				}
				
				if (zEnabled)
				{
					zValue = configuration[zIndex];
					updateLocal = true;
				}
				
				// Only update local transformation if a joint value has changed.
				if (updateLocal)
				{
					ghostJoint.ComputeLocalTransformation(xValue, yValue, zValue, out lpx, out lpy, out lpz, out lrx, out lry, out lrz, out lrw);
					updateWorld = true;
				}

				// Only update world transformation if local transformation (in this or parent node) has changed.
				if (updateWorld)
				{
					ComputeWorldTransformation();
				}

				_child?.FeedForwardConfiguration(configuration, updateWorld);
			}

			/// <summary>
			/// Simulates a single transform modification without changing actual values.
			/// </summary>
			/// <param name="configuration">The joint configuration.</param>
			public void SimulateModification(float[] configuration)
			{
				Node node = _ghostRobot.motionPointers[^1].node;
				ghostJoint.ComputeLocalTransformation(
					xEnabled ? configuration[xIndex] : xValue,
					yEnabled ? configuration[yIndex] : yValue, 
					zEnabled ? configuration[zIndex] : zValue, 
					out double lpX, out double lpY, out double lpZ, out double lrX, out double lrY, out double lrZ, out double lrW
				);
				double localRx, localRy, localRz, localRw, localX, localY, localZ;
				double px;
				double py;
				double pz;
				if (_parent == null)
				{
					px = _ghostRobot._opx;
					py = _ghostRobot._opy;
					pz = _ghostRobot._opz;
					localRx = _ghostRobot._orx;
					localRy = _ghostRobot._ory;
					localRz = _ghostRobot._orz;
					localRw = _ghostRobot._orw;
					localX = lpX;
					localY = lpY;
					localZ = lpZ;
				}
				else
				{
					px = _parent.wpx;
					py = _parent.wpy;
					pz = _parent.wpz;
					localRx = _parent.wrx;
					localRy = _parent.wry;
					localRz = _parent.wrz;
					localRw = _parent.wrw;
					localX = _parent.wsx * lpX;
					localY = _parent.wsy * lpY;
					localZ = _parent.wsz * lpZ;
				}
				double qx = localRx * lrW + localRy * lrZ - localRz * lrY + localRw * lrX;
				double qy = -localRx * lrZ + localRy * lrW + localRz * lrX + localRw * lrY;
				double qz = localRx * lrY - localRy * lrX + localRz * lrW + localRw * lrZ;
				double qw = -localRx * lrX - localRy * lrY - localRz * lrZ + localRw * lrW;
				double dot = wrx*wrx + wry*wry + wrz*wrz + wrw*wrw;
				double x = qx / dot; double y = qy / dot; double z = qz / dot; double w = qw / dot;
				qx = x * wrw + y * -wrz - z * -wry + w * -wrx;
				qy = -x * -wrz + y * wrw + z * -wrx + w * -wry;
				qz = x * -wry - y * -wrx + z * wrw + w * -wrz;
				qw = -x * -wrx - y * -wry - z * -wrz + w * wrw;
				px +=
						+ 2.0 * ((0.5 - localRy * localRy - localRz * localRz) * localX + (localRx * localRy - localRw * localRz) * localY + (localRx * localRz + localRw * localRy) * localZ)
						+ 2.0 * ((0.5 - qy * qy - qz * qz) * (node.wpx-wpx) + (qx * qy - qw * qz) * (node.wpy-wpy) + (qx * qz + qw * qy) * (node.wpz-wpz));
				py += 
						+ 2.0 * ((localRx * localRy + localRw * localRz) * localX + (0.5 - localRx * localRx - localRz * localRz) * localY + (localRy * localRz - localRw * localRx) * localZ)
						+ 2.0 * ((qx * qy + qw * qz) * (node.wpx-wpx) + (0.5 - qx * qx - qz * qz) * (node.wpy-wpy) + (qy * qz - qw * qx) * (node.wpz-wpz));
				pz += 
						+ 2.0 * ((localRx * localRz - localRw * localRy) * localX + (localRy * localRz + localRw * localRx) * localY + (0.5 - (localRx * localRx + localRy * localRy)) * localZ)
						+ 2.0 * ((qx * qz - qw * qy) * (node.wpx-wpx) + (qy * qz + qw * qx) * (node.wpy-wpy) + (0.5 - qx * qx - qy * qy) * (node.wpz-wpz));
				double rx = qx * node.wrw + qy * node.wrz - qz * node.wry + qw * node.wrx;
				double ry = -qx * node.wrz + qy * node.wrw + qz * node.wrx + qw * node.wry;
				double rz = qx * node.wry - qy * node.wrx + qz * node.wrw + qw * node.wrz;
				double rw = -qx * node.wrx - qy * node.wry - qz * node.wrz + qw * node.wrw;
				_ghostRobot._simulatedLoss = _ghostRobot.ComputeLoss(px, py, pz, rx, ry, rz, rw);
			}

			/// <summary>
			/// Computes the world transformation using the current joint variable configuration.
			/// </summary>
			private void ComputeWorldTransformation()
			{
				double rx, ry, rz, rw, x, y, z;
				if (_parent == null)
				{
					wpx = _ghostRobot._opx;
					wpy = _ghostRobot._opy;
					wpz = _ghostRobot._opz;
					rx = _ghostRobot._orx;
					ry = _ghostRobot._ory;
					rz = _ghostRobot._orz;
					rw = _ghostRobot._orw;
					x = lpx;
					y = lpy;
					z = lpz;
				}
				else
				{
					wpx = _parent.wpx;
					wpy = _parent.wpy;
					wpz = _parent.wpz;
					rx = _parent.wrx;
					ry = _parent.wry;
					rz = _parent.wrz;
					rw = _parent.wrw;
					x = _parent.wsx * lpx;
					y = _parent.wsy * lpy;
					z = _parent.wsz * lpz;
				}
				wpx += 2.0 * ((0.5 - ry * ry - rz * rz) * x + (rx * ry - rw * rz) * y + (rx * rz + rw * ry) * z);
				wpy += 2.0 * ((rx * ry + rw * rz) * x + (0.5 - rx * rx - rz * rz) * y + (ry * rz - rw * rx) * z);
				wpz += 2.0 * ((rx * rz - rw * ry) * x + (ry * rz + rw * rx) * y + (0.5 - rx * rx - ry * ry) * z);
				wrx = rx * lrw + ry * lrz - rz * lry + rw * lrx;
				wry = -rx * lrz + ry * lrw + rz * lrx + rw * lry;
				wrz = rx * lry - ry * lrx + rz * lrw + rw * lrz;
				wrw = -rx * lrx - ry * lry - rz * lrz + rw * lrw;
			}
		}

		/// <summary>
		/// Store joint motion data.
		/// </summary>
		public struct MotionPtr
		{
			/// <summary>
			/// The motion to reference.
			/// </summary>
			public readonly GhostJoint.Motion motion;
			
			/// <summary>
			/// The node this motion is attached to.
			/// </summary>
			public readonly Node node;
			
			/// <summary>
			/// The index of this motion.
			/// </summary>
			public readonly int index;
			
			/// <summary>
			/// Setup this motion pointer.
			/// </summary>
			/// <param name="motion">The motion.</param>
			/// <param name="node">The node.</param>
			/// <param name="index">The motion index.</param>
			public MotionPtr(GhostJoint.Motion motion, Node node, int index)
			{
				this.motion = motion;
				this.node = node;
				this.index = index;
			}
		}
	}
}