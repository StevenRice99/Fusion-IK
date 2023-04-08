namespace FusionIK
{
    /// <summary>
    /// Store a result of a robot moving to a target.
    /// </summary>
    public struct Result
    {
        /// <summary>
        /// How many generations were allowed.
        /// </summary>
        public readonly int maxGenerations;
        
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
        public readonly float time;

        /// <summary>
        /// The number of generations required to reach the solution.
        /// </summary>
        public readonly int generations;

        /// <summary>
        /// How far off in meters was the end effectors from reaching the target.
        /// </summary>
        public readonly float distance;

        /// <summary>
        /// How far off in degrees was the end effector from reaching the target.
        /// </summary>
        public readonly float angle;

        /// <summary>
        /// Store a move result.
        /// </summary>
        /// <param name="maxGenerations">How many generations were allowed.</param>
        /// <param name="robot">The robot that did the move.</param>
        /// <param name="success">If the move was successful.</param>
        /// <param name="time">The time it took for the joints to reach their destinations.</param>
        /// <param name="generations">The number of generations required to reach the solution.</param>
        /// <param name="distance">How far off in meters was the end effectors from reaching the target.</param>
        /// <param name="angle">How far off in degrees was the end effector from reaching the target.</param>
        public Result(int maxGenerations, Robot robot, bool success, float time, int generations, float distance, float angle)
        {
            this.maxGenerations = maxGenerations;
            this.robot = robot;
            this.success = success;
            this.time = time;
            this.generations = generations;
            this.distance = distance;
            this.angle = angle;
        }
    }
}