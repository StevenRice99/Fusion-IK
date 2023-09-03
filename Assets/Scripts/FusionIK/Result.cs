namespace FusionIK
{
    /// <summary>
    /// Store a result of a robot moving to a target.
    /// </summary>
    public struct Result
    {
        /// <summary>
        /// The time the algorithm is allowed to run for.
        /// </summary>
        public readonly long milliseconds;
        
        /// <summary>
        /// The robot that did the move.
        /// </summary>
        public readonly Robot robot;
        
        /// <summary>
        /// If the move was successful.
        /// </summary>
        public readonly bool success;

        /// <summary>
        /// The time it took for the joints to reach their destinations.
        /// </summary>
        public readonly double time;

        /// <summary>
        /// How far off in meters was the end effectors from reaching the target.
        /// </summary>
        public readonly float distance;

        /// <summary>
        /// How far off in degrees was the end effector from reaching the target.
        /// </summary>
        public readonly float angle;

        /// <summary>
        /// The fitness score of the result.
        /// </summary>
        public readonly double fitness;

        /// <summary>
        /// Store a move result.
        /// </summary>
        /// <param name="milliseconds">The time the algorithm is allowed to run for.</param>
        /// <param name="robot">The robot that did the move.</param>
        /// <param name="success">If the move was successful.</param>
        /// <param name="time">The time it took for the joints to reach their destinations.</param>
        /// <param name="distance">How far off in meters was the end effectors from reaching the target.</param>
        /// <param name="angle">How far off in degrees was the end effector from reaching the target.</param>
        /// <param name="fitness">The fitness score of the result.</param>
        public Result(long milliseconds, Robot robot, bool success, double time, float distance, float angle, double fitness)
        {
            this.milliseconds = milliseconds;
            this.robot = robot;
            this.success = success;
            this.time = time;
            this.distance = distance;
            this.angle = angle;
            this.fitness = fitness;
        }
    }
}