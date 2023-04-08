namespace FusionIK
{
    /// <summary>
    /// Store the limits of a joint axis.
    /// </summary>
    public struct JointLimit
    {
        /// <summary>
        /// The lower limit.
        /// </summary>
        public readonly float lower;

        /// <summary>
        /// The upper limit.
        /// </summary>
        public readonly float upper;

        /// <summary>
        /// Configure a joint axis limit.
        /// </summary>
        /// <param name="lower">The lower limit.</param>
        /// <param name="upper">The upper limit.</param>
        public JointLimit(float lower, float upper)
        {
            this.lower = lower;
            this.upper = upper;
        }
    }
}