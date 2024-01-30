using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace FusionIK
{
    /// <summary>
    /// Joint handling.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(ArticulationBody))]
    public class RobotJoint : MonoBehaviour
    {
        /// <summary>
        /// The speed in meters per second (for prismatic joints) or in degrees per second (for rotational joints).
        /// </summary>
        [Tooltip("The speed in meters per second (for prismatic joints) or in degrees per second (for rotational joints).")]
        [Min(float.Epsilon)]
        [SerializeField]
        private float3 speed;
        
        /// <summary>
        /// The articulation body being the actual joint controller.
        /// </summary>
        public ArticulationBody Joint { get; private set; }
        
        /// <summary>
        /// The X movement limit.
        /// </summary>
        public JointLimit LimitX { get; private set; }

        /// <summary>
        /// The Y movement limit.
        /// </summary>
        public JointLimit LimitY { get; private set; }
        
        /// <summary>
        /// The Z movement limit.
        /// </summary>
        public JointLimit LimitZ { get; private set; }

        /// <summary>
        /// If this joint has any kind of movement.
        /// </summary>
        public bool HasMotion => Type != ArticulationJointType.FixedJoint;

        /// <summary>
        /// If there is motion along the X axis.
        /// </summary>
        public bool XMotion => XDrive.lowerLimit != 0 && XDrive.upperLimit != 0;

        /// <summary>
        /// If there is motion along the Y axis.
        /// </summary>
        public bool YMotion => YDrive.lowerLimit != 0 && YDrive.upperLimit != 0;

        /// <summary>
        /// If there is motion along the Z axis.
        /// </summary>
        public bool ZMotion => ZDrive.lowerLimit != 0 && ZDrive.upperLimit != 0;

        /// <summary>
        /// The speed of the X axis.
        /// </summary>
        public float SpeedX => Type == ArticulationJointType.PrismaticJoint ? speed.x : math.radians(speed.x);

        /// <summary>
        /// The speed of the XY axis.
        /// </summary>
        public float SpeedY => Type == ArticulationJointType.PrismaticJoint ? speed.y : math.radians(speed.y);

        /// <summary>
        /// The speed of the Z axis.
        /// </summary>
        public float SpeedZ => Type == ArticulationJointType.PrismaticJoint ? speed.z : math.radians(speed.z);

        /// <summary>
        /// The type of joint.
        /// </summary>
        public ArticulationJointType Type => Joint.jointType;

        /// <summary>
        /// The drive for the X axis.
        /// </summary>
        private ArticulationDrive XDrive => Joint.xDrive;

        /// <summary>
        /// The drive for the Y axis.
        /// </summary>
        private ArticulationDrive YDrive => Joint.yDrive;

        /// <summary>
        /// The drive for the Z axis.
        /// </summary>
        private ArticulationDrive ZDrive => Joint.zDrive;

        /// <summary>
        /// Define limits based on the joint type.
        /// </summary>
        public void Setup()
        {
            OnValidate();
            
            switch (Type)
            {
                case ArticulationJointType.FixedJoint:
                    break;
                case ArticulationJointType.PrismaticJoint:
                    LimitX = new(XDrive.lowerLimit, XDrive.upperLimit);
                    LimitY = new(YDrive.lowerLimit, YDrive.upperLimit);
                    LimitZ = new(ZDrive.lowerLimit, ZDrive.upperLimit);
                    break;
                case ArticulationJointType.RevoluteJoint:
                    LimitX = new(math.radians(XDrive.lowerLimit), math.radians(XDrive.upperLimit));
                    LimitY = new(0, 0);
                    LimitZ = new(0, 0);
                    break;
                case ArticulationJointType.SphericalJoint:
                default:
                    if (XMotion)
                    {
                        LimitX = new(math.radians(XDrive.lowerLimit), math.radians(XDrive.upperLimit));
                    }
                    else
                    {
                        LimitX = new(0, 0);
                    }
                    if (YMotion)
                    {
                        LimitY = new(math.radians(YDrive.lowerLimit), math.radians(YDrive.upperLimit));
                    }
                    else
                    {
                        LimitY = new(0, 0);
                    }
                    if (ZMotion)
                    {
                        LimitY = new(math.radians(ZDrive.lowerLimit), math.radians(ZDrive.upperLimit));
                    }
                    else
                    {
                        LimitZ = new(0, 0);
                    }
                    break;
            }
        }

        private void OnValidate()
        {
            Joint = GetComponent<ArticulationBody>();
            if (Joint == null)
            {
                return;
            }

            ArticulationDrive drive = XDrive;
            drive.stiffness = 100000;
            drive.damping = 100;
            drive.forceLimit = float.MaxValue;
            drive.targetVelocity = 0;
            Joint.xDrive = drive;
            
            drive = YDrive;
            drive.stiffness = 100000;
            drive.damping = 100;
            drive.forceLimit = float.MaxValue;
            drive.targetVelocity = 0;
            Joint.yDrive = drive;
            
            drive = ZDrive;
            drive.stiffness = 100000;
            drive.damping = 100;
            drive.forceLimit = float.MaxValue;
            drive.targetVelocity = 0;
            Joint.zDrive = drive;
        }

        /// <summary>
        /// Get the limits of the joint.
        /// </summary>
        /// <returns>All limits of the joint.</returns>
        public IEnumerable<JointLimit> Limits()
        {
            List<JointLimit> limits = new();
            if (XMotion)
            {
                limits.Add(LimitX);
            }
            if (YMotion)
            {
                limits.Add(LimitY);
            }
            if (ZMotion)
            {
                limits.Add(LimitZ);
            }

            return limits;
        }
    }
}