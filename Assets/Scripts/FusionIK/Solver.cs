using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using Random = UnityEngine.Random;

namespace FusionIK
{
    /// <summary>
    /// Inverse kinematics solver.
    /// </summary>
	public static class Solver
    {
        /// <summary>
        /// Ghost robot to perform calculations on.
        /// </summary>
        private static VirtualRobot _virtual;
        
        /// <summary>
        /// Population size.
        /// </summary>
		private static int _populationSize;
        
        /// <summary>
        /// Number of elites.
        /// </summary>
		private static int _elites;
        
        /// <summary>
        /// Number of joints.
        /// </summary>
		private static int _dimensionality;

        /// <summary>
        /// Lower limits of joints.
        /// </summary>
		private static double[] _lowerBounds;
        
        /// <summary>
        /// Upper limits of joints.
        /// </summary>
		private static double[] _upperBounds;

        /// <summary>
        /// The population.
        /// </summary>
		private static Individual[] _population;
        
        /// <summary>
        /// The offspring for the next population.
        /// </summary>
		private static Individual[] _offspring;

        /// <summary>
        /// Selection pool for recombination for the next generation.
        /// </summary>
		private static readonly List<Individual> Pool = new();
        
        /// <summary>
        /// Number of members in the pool.
        /// </summary>
		private static int _poolCount;
        
        /// <summary>
        /// Likelihood of each member being selected for recombination.
        /// </summary>
		private static double[] _probabilities;
        
        /// <summary>
        /// Helper gene storage variable.
        /// </summary>
		private static double _gene;
        
        /// <summary>
        /// Helper weight storage variable.
        /// </summary>
		private static double _weight;

        /// <summary>
        /// Which members improved.
        /// </summary>
        private static bool[] _improved;
        
        /// <summary>
        /// Elite models operations can be run in parallel.
        /// </summary>
        private static VirtualRobot[] _ghosts;
        
        /// <summary>
        /// Optimizers for every elite.
        /// </summary>
        private static Optimizer[] _optimizers;

        /// <summary>
        /// Current best joint values.
        /// </summary>
		private static double[] _solution;
        
        /// <summary>
        /// Current best fitness.
        /// </summary>
		private static double _fitness;

        /// <summary>
        /// Random number generator.
        /// </summary>
        private static Unity.Mathematics.Random _random;

        /// <summary>
        /// The duration to optimise for.
        /// </summary>
        private static double _duration;

        /// <summary>
        /// Run the algorithm to solve for a solution.
        /// </summary>
        /// <param name="targetPosition">The position to reach.</param>
        /// <param name="targetRotation">The rotation to reach.</param>
        /// <param name="details">The solving parameters to save to.</param>
        /// <returns>The results to update.</returns>
        public static void Run(ref Vector3 targetPosition, ref Quaternion targetRotation, ref Details details)
        {
            // Reset the values.
            details.Reset(targetPosition, targetRotation);
            
            // If already at the destination do nothing.
            if (details.Success)
            {
                return;
            }

            double[][] seed = new double[details.robot.mode != Robot.SolverMode.BioIk ? 2 : 1][];
            seed[0] = new double[details.Joints.Length];
            for (int i = 0; i < seed[0].Length; i++)
            {
                seed[0][i] = details.Joints[i];
            }

            // Run through neural networks if it should.
            if (details.robot.mode != Robot.SolverMode.BioIk)
            {
                List<float> starting = details.robot.minimal ? null : details.robot.GetJoints();
                
                details.Start();
                List<float> joints = details.robot.RunNetwork(targetPosition, targetRotation, starting);
                details.Stop();

                seed[1] = new double[joints.Count];
                for (int i = 0; i < seed[1].Length; i++)
                {
                    seed[1][i] = joints[i];
                }

                // Get the existing fitness.
                details.Set(seed[1]);
                
                // If it was reached, don't use it in future attempts for even faster times as it will always be the exact same.
                if (details.Success)
                {
                    double[][] temp = new double[1][];
                    temp[0] = new double[seed[0].Length];
                    for (int i = 0; i < temp[0].Length; i++)
                    {
                        temp[0][i] = seed[0][i];
                    }

                    seed = temp;
                }
            }

            // Use Bio IK if it should.
            if (details.robot.mode == Robot.SolverMode.Network)
            {
                return;
            }
            
            _random = new((uint) Random.Range(1, int.MaxValue));

            _populationSize = details.robot.Properties.Population;
            _elites = details.robot.Properties.Elites;
            _virtual = details.robot.Virtual;
            _dimensionality = _virtual.dof;

            _population = new Individual[_populationSize];
            _offspring = new Individual[_populationSize];
            for (int i = 0; i < _populationSize; i++)
            {
                _population[i] = new(_dimensionality);
                _offspring[i] = new(_dimensionality);
            }

            _lowerBounds = new double[_dimensionality];
            _upperBounds = new double[_dimensionality];
            _probabilities = new double[_populationSize];
            _solution = new double[_dimensionality];

            _ghosts = new VirtualRobot[_elites];
            _optimizers = new Optimizer[_elites];
            _improved = new bool[_elites];
            for (int i = 0; i < _elites; i++)
            {
                int member = i;
                _ghosts[member] = new(details.robot);
                _optimizers[member] = new(_dimensionality, x => _ghosts[member].ComputeLoss(x), y => _ghosts[member].ComputeGradient(y, 1e-5));
            }
            
            // Set the limits.
            for (int i = 0; i < _dimensionality; i++)
            {
                _lowerBounds[i] = _virtual.motionPointers[i].motion.GetLowerLimit();
                _upperBounds[i] = _virtual.motionPointers[i].motion.GetUpperLimit();
            }
            
            // The number of generations that have passed. Used in iterative Fusion IK.
            int generations = 0;
            
            details.Start();

            // Loop for all available time.
            do
            {
                // Reset the fitness.
                _fitness = double.MaxValue;
            
                // Reset any previous values.
                for (int i = 0; i < _populationSize; i++)
                {
                    _population[i].Reset();
                    _offspring[i].Reset();
                }
                for (int i = 0; i < _elites; i++)
                {
                    _optimizers[i].Reset();
                }
            
                // Set the seed as the first member of the population.
                for (int i = 0; i < seed.Length; i++)
                {
                    for (int j = 0; j < _dimensionality; j++)
                    {
                        _population[i].genes[j] = seed[i][j];
                        _population[i].momentum[j] = 0.0;
                    }
                
                    _population[i].fitness = _virtual.ComputeLoss(_population[i].genes);
                }

                // Randomize the rest of the population.
                for (int i = seed.Length; i < _populationSize; i++)
                {
                    RandomMember(_population[i]);
                }

                SortByFitness();
                ComputeExtinctions();
                TryUpdateSolution();
            
                // Start from the best seed.
                for (int i = 0; i < _dimensionality; i++)
                {
                    _virtual.motionPointers[i].motion.SetTargetValue((float) _solution[i]);
                }
            
                // Setup the elites.
                for (int i = 0; i < _elites; i++)
                {
                    _ghosts[i].CopyFrom(_virtual);
                    _optimizers[i].lowerBounds = _lowerBounds;
                    _optimizers[i].upperBounds = _upperBounds;
                }
                
                // Loop until a solution is reached.
                do
                {
                    // Create the mating pool.
			        Pool.Clear();
			        Pool.AddRange(_population);
			        _poolCount = _populationSize;
                    
                    DateTime timestamp = DateTime.Now;
                    
                    // Evolve offspring.
                    for (int i = _elites; i < _populationSize; i++)
                    {
                        if (_poolCount > 0)
                        {
                            Individual parentA = Select(Pool);
                            Individual parentB = Select(Pool);
                            Individual prototype = Select(Pool);

                            // Recombination and adoption.
                            Reproduce(_offspring[i], parentA, parentB, prototype);
                            
                            if (_offspring[i].fitness < parentA.fitness)
                            {
                                Pool.Remove(parentA);
                                _poolCount -= 1;
                            }
                            
                            if (_offspring[i].fitness < parentB.fitness)
                            {
                                Pool.Remove(parentB);
                                _poolCount -= 1;
                            }
                        }
                        else
                        {
                            // Fill the rest of the population.
                            RandomMember(_offspring[i]);
                        }
                    }

                    _duration = ElapsedTime(timestamp);

                    // Exploit the elites.
#if UNITY_WEBGL
                    for (int i = 0; i < _elites; i++)
                    {
                        Survive(i);
                    }
#else
                    System.Threading.Tasks.Parallel.For(0, _elites, Survive);
#endif
			        // Re-roll elite if exploitation was not successful.
			        for (int i = 0; i < _elites; i++)
                    {
                        if (_improved[i])
                        {
                            continue;
                        }

                        // In Bio IK, standard Fusion IK, and iterative Fusion IK, create a random member.
                        if (!details.robot.exhaustive)
                        {
                            RandomMember(_offspring[i]);
                            continue;
                        }

                        // Otherwise, pass the current value through the network.
                        List<float> starting = new(_dimensionality);
                        for (int j = 0; j < _dimensionality; j++)
                        {
                            starting.Add((float) _offspring[i].genes[j]);
                        }

                        // Run the network.
                        List<float> results = details.robot.RunNetwork(targetPosition, targetRotation, starting);
                        
                        // Update the values.
                        for (int j = 0; j < _dimensionality; j++)
                        {
                            _offspring[i].genes[j] = results[j];
                            _offspring[i].momentum[j] = 0;
                        }

                        _offspring[i].fitness = _virtual.ComputeLoss(_offspring[i].genes);
                    }
                    
                    // Swap population and offspring.
                    (_population, _offspring) = (_offspring, _population);

			        SortByFitness();

                    // Check for improvement.
                    bool improvement = TryUpdateSolution() || HasAnyEliteImproved();
                    
                    details.Set(_solution);
                    
                    // If we reached the target, finish this attempt.
                    if (_virtual.CheckConvergence(_solution, targetPosition, targetRotation))
                    {
                        break;
                    }
                    
                    // Break out if there was no improvement or out of time.
                    if ((details.robot.iterative && generations >= details.robot.Properties.Generations) || !improvement || details.Done)
                    {
                        // Keep the best values and pass then into the network.
                        if (details.robot.iterative)
                        {
                            // Reset the generations.
                            generations = 0;
                            
                            // Keep the best of the population along with the original seed.
                            double[][] temp = new double[2 + details.robot.Properties.Kept][];
                            
                            // Copy the original seed values.
                            temp[0] = seed[0];
                            temp[1] = seed[1];
                            
                            // Copy the best of the population.
                            for (int i = 0; i < details.robot.Properties.Kept; i++)
                            {
                                // Convert to floats.
                                List<float> starting = new(_dimensionality);
                                for (int j = 0; j < _dimensionality; j++)
                                {
                                    starting.Add((float) _population[i].genes[j]);
                                }
                                
                                // Run the network.
                                List<float> results = details.robot.RunNetwork(targetPosition, targetRotation, starting);
                                temp[i + 2] = new double[_dimensionality];
                                
                                // Copy to the new seeds.
                                for (int j = 0; j < _dimensionality; j++)
                                {
                                    temp[i + 2][j] = results[j];
                                }
                            }

                            // Update the seed.
                            seed = temp;
                        }
                        
                        break;
                    }
                    
                    // Evolve the next generation;
                    ComputeExtinctions();
                } while (true);

            } while (!details.Done);
        }

        /// <summary>
        /// Get the time elapsed since a timestamp
        /// </summary>
        /// <param name="timestamp">The timestamp to check.</param>
        /// <returns>The time in seconds since the timestamp.</returns>
        public static double ElapsedTime(DateTime timestamp)
        {
            return (DateTime.Now - timestamp).Duration().TotalSeconds;
        }

		/// <summary>
		/// Get the mutation probability from two parents.
		/// </summary>
		/// <param name="parentA">The first parent.</param>
		/// <param name="parentB">The second parent.</param>
		/// <returns>The mutation probability between two parents.</returns>
		private static double GetMutationProbability(Individual parentA, Individual parentB)
        {
			double inverse = 1.0 / _dimensionality;
			return 0.5 * (parentA.extinction + parentB.extinction) * (1.0 - inverse) + inverse;
		}

		/// <summary>
		/// Get the mutation strength from two parents
		/// </summary>
        /// <param name="parentA">The first parent.</param>
        /// <param name="parentB">The second parent.</param>
		/// <returns>The mutation strength between two parents.</returns>
		private static double GetMutationStrength(Individual parentA, Individual parentB)
        {
			return 0.5 * (parentA.extinction + parentB.extinction);
		}

		/// <summary>
		/// Computes the extinction factors for all members of the population.
		/// </summary>
		private static void ComputeExtinctions()
        {
			for (int i = 0; i < _populationSize; i++)
            {
				_population[i].extinction = (_population[i].fitness + _population[0].fitness * (i / ((double) _populationSize - 1) - 1.0)) / _population[_populationSize - 1].fitness;
			}
        }
        
        /// <summary>
		/// Get if any could be improved by the exploitation.
		/// </summary>
		/// <returns>True if any elite could be improved, false otherwise.</returns>
		private static bool HasAnyEliteImproved()
        {
			for (int i = 0; i < _elites; i++)
            {
				if (_improved[i])
                {
					return true;
				}
			}
			return false;
		}
        
        /// <summary>
        /// Tries to improve the evolutionary solution by the population, and returns whether it was successful
        /// </summary>
        /// <returns>Try if the solution was improved, false otherwise.</returns>
		private static bool TryUpdateSolution()
        {
			double candidateFitness = _population[0].fitness;
            if (candidateFitness >= _fitness)
            {
                return false;
            }

            for (int i = 0; i < _dimensionality; i++)
            {
                _solution[i] = _population[0].genes[i];
            }
            _fitness = candidateFitness;
            return true;
        }

        /// <summary>
        /// Test an elite for its survival.
        /// </summary>
        /// <param name="index">The index of the elite.</param>
		private static void Survive(int index)
        {
            // Copy the elite.
            Individual survivor = _population[index];
            Individual elite = _offspring[index];
            for (int i = 0; i < _dimensionality; i++)
            {
                elite.genes[i] = survivor.genes[i];
                elite.momentum[i] = survivor.momentum[i];
            }

            // Exploit.
            double fitness = _ghosts[index].ComputeLoss(elite.genes);
            _optimizers[index].Minimise(elite.genes, _duration);
            if (_optimizers[index].value < fitness)
            {
                for (int i = 0; i < _dimensionality; i++)
                {
                    elite.momentum[i] = _optimizers[index].solution[i] - elite.genes[i];
                    elite.genes[i] = _optimizers[index].solution[i];
                }
                elite.fitness = _optimizers[index].value;        
                _improved[index] = true;
                return;
            }

            elite.fitness = fitness;
            _improved[index] = false;
        }

		/// <summary>
		/// Evolve a new member of the population.
		/// </summary>
		/// <param name="offspring">The new offspring.</param>
		/// <param name="parentA">The first parent.</param>
		/// <param name="parentB">The second parent.</param>
		/// <param name="prototype">The individual to base off of.</param>
		private static void Reproduce(Individual offspring, Individual parentA, Individual parentB, Individual prototype)
        {
            double mutationProbability = GetMutationProbability(parentA, parentB);
			double mutationStrength = GetMutationStrength(parentA, parentB);

			for (int i = 0; i < _dimensionality; i++)
            {
				// Recombination.
				_weight = _random.NextFloat();
				double momentum = _random.NextFloat() * parentA.momentum[i] + _random.NextFloat() * parentB.momentum[i];
				offspring.genes[i] = _weight * parentA.genes[i] + (1.0 - _weight)*parentB.genes[i] + momentum;

				// Store.
				_gene = offspring.genes[i];

				// Mutation.
                if (_random.NextFloat() < mutationProbability)
                {
                    offspring.genes[i] += (_random.NextFloat() * (_upperBounds[i] - _lowerBounds[i]) + _lowerBounds[i]) * mutationStrength;
                }

				// Adoption.
				_weight = _random.NextFloat();
				offspring.genes[i] += 
					_weight * _random.NextFloat() * (0.5 * (parentA.genes[i] + parentB.genes[i]) - offspring.genes[i])
					+ (1.0 - _weight) * _random.NextFloat() * (prototype.genes[i] - offspring.genes[i]);

				// Project.
                if (offspring.genes[i] < _lowerBounds[i])
                {
                    offspring.genes[i] = _lowerBounds[i];
                }
                else if (offspring.genes[i] > _upperBounds[i])
                {
                    offspring.genes[i] = _upperBounds[i];
                }

				// Momentum.
				offspring.momentum[i] = _random.NextFloat() * momentum + (offspring.genes[i] - _gene);
			}
            
            // Fitness.
			offspring.fitness = _virtual.ComputeLoss(offspring.genes);
		}

		/// <summary>
		/// Generate a random individual.
		/// </summary>
		/// <param name="individual">The individual to generate.</param>
		private static void RandomMember(Individual individual)
        {
			for (int i = 0; i < _dimensionality; i++)
            {
                individual.genes[i] = _random.NextDouble(_lowerBounds[i], _upperBounds[i]);
                individual.momentum[i] = 0.0;
			}
			individual.fitness = _virtual.ComputeLoss(individual.genes);
		}

		/// <summary>
		/// Rank-based selection of an individual.
		/// </summary>
		/// <param name="pool">The selection pool.</param>
		/// <returns>An individual from the population.</returns>
		private static Individual Select(List<Individual> pool)
        {
			double rankSum = _poolCount * (_poolCount + 1) / 2.0;
			for (int i = 0; i < _poolCount; i++)
            {
				_probabilities[i] = (_poolCount - i) / rankSum;
			}
			return pool[GetRandomWeightedIndex(_probabilities, _poolCount)];
		}
		
		/// <summary>
		/// Returns a random index with respect to the probability weights.
		/// </summary>
		/// <param name="probabilities">The probabilities of each weight.</param>
		/// <param name="count">The number of weights.</param>
		/// <returns>The random weight value.</returns>
		private static int GetRandomWeightedIndex(double[] probabilities, int count)
        {
			double weightSum = 0.0;
			for (int i = 0; i < count; i++)
            {
				weightSum += probabilities[i];
			}
			double rVal = _random.NextFloat() * weightSum;
			for (int i = 0; i < count; i++)
            {
				rVal -= probabilities[i];
				if (rVal <= 0.0)
                {
					return i;
				}
			}
			return count-1;
		}

        /// <summary>
        /// Sort the population by their fitness values.
        /// </summary>
		private static void SortByFitness()
        {
			Array.Sort(_population, (a, b) => a.fitness.CompareTo(b.fitness));
		}

		/// <summary>
		/// Member of the population.
		/// </summary>
        private class Individual
        {
            /// <summary>
            /// Genes.
            /// </summary>
			public readonly double[] genes;
            
            /// <summary>
            /// Evolution momentum.
            /// </summary>
			public readonly double[] momentum;
            
            /// <summary>
            /// Fitness.
            /// </summary>
			public double fitness;
            
            /// <summary>
            /// Extinction chance.
            /// </summary>
            public double extinction;

            /// <summary>
            /// Setup a new member.
            /// </summary>
            /// <param name="dimensionality">The degrees of freedom.</param>
			public Individual(int dimensionality)
            {
				genes = new double[dimensionality];
				momentum = new double[dimensionality];
				fitness = 0.0;
                extinction = 0.0;
			}

            /// <summary>
            /// Reset the member.
            /// </summary>
            public void Reset()
            {
                fitness = 0.0;
                extinction = 0.0;
            }
		}
	}

    /// <summary>
    /// Based on http://www.netlib.org/opt/lbfgs_um.shar
    /// </summary>
    public class Optimizer
    {
        /// <summary>
        /// The current task being performed.
        /// </summary>
        private enum Task { None, Start, NewX, Fg, FgLN, FgSt, Abnormal, Convergence, Error, RestartLN, Warning }

        /// <summary>
        /// Function to perform.
        /// </summary>
        private readonly Func<double[], double> _function;
        
        /// <summary>
        /// Gradient call.
        /// </summary>
        private readonly Func<double[], double[]> _gradient;
        
        /// <summary>
        /// The degrees of freedom of the robot.
        /// </summary>
        private readonly int _dimensionality;
        
        /// <summary>
        /// The current best solution.
        /// </summary>
        public readonly double[] solution;
        
        /// <summary>
        /// Helper value.
        /// </summary>
        public double value;
        
        /// <summary>
        /// Lower limits.
        /// </summary>
        public double[] lowerBounds;
        
        /// <summary>
        /// Upper limits.
        /// </summary>
        public double[] upperBounds;

        /// <summary>
        /// F value.
        /// </summary>
        private double _f;
        
        /// <summary>
        /// G values.
        /// </summary>
        private readonly double[] _g;

        /// <summary>
        /// Bool values.
        /// </summary>
        private readonly bool[] _lSave;
        
        /// <summary>
        /// Integer values.
        /// </summary>
        private readonly int[] _save;
        
        /// <summary>
        /// Double values.
        /// </summary>
        private readonly double[] _dSave;

        /// <summary>
        /// Value used in minimization.
        /// </summary>
        private readonly int[] _iwa;

        /// <summary>
        /// Value used in minimization.
        /// </summary>
        private readonly int[] _nbd;
        
        /// <summary>
        /// Work values being performed on.
        /// </summary>
        private readonly double[] _work;

        /// <summary>
        /// Current task.
        /// </summary>
        private Task _task;
        
        /// <summary>
        /// Currently saved task.
        /// </summary>
        private Task _cSave;

        /// <summary>
        /// New F value applied.
        /// </summary>
        private double _newF;
        
        /// <summary>
        /// New G values applied.
        /// </summary>
        private double[] _newG;

        /// <summary>
        /// Initialize optimization.
        /// </summary>
        /// <param name="dimensionality">The degrees of freedom.</param>
        /// <param name="function">Function to perform.</param>
        /// <param name="gradient">Gradient call.</param>
        public Optimizer(int dimensionality, Func<double[], double> function, Func<double[], double[]> gradient)
        {
            _dimensionality = dimensionality;
            _function = function;
            _gradient = gradient;
            upperBounds = new double[_dimensionality];
            lowerBounds = new double[_dimensionality];
            solution = new double[_dimensionality];

            int totalSize = 2 * _dimensionality + 11 + 5 * _dimensionality + 8;
            _nbd = new int[_dimensionality];
            for (int i = 0; i < _dimensionality; i++)
            {
                _nbd[i] = 2;
            }

            _f = 0.0;
            _g = new double[_dimensionality];
            _lSave = new bool[4];
            _save = new int[44];
            _dSave = new double[29];
            _iwa = new int[3 * _dimensionality];
            _work = new double[totalSize];
            _task = Task.Start;
            _cSave = Task.None;
            _newF = 0;
            _newG = null;
        }

        /// <summary>
        /// Reset the optimization.
        /// </summary>
        public void Reset()
        {
            for (int i = 0; i < _dimensionality; i++)
            {
                _nbd[i] = 2;
            }

            _f = 0.0;
            _task = Task.Start;
            _cSave = Task.None;
            _newF = 0;
            _newG = null;
        }

        /// <summary>
        /// Perform minimization.
        /// </summary>
        /// <param name="values">The values to perform.</param>
        /// <param name="duration">The duration to optimise for.</param>
        public void Minimise(double[] values, double duration)
        {
            for (int i = 0; i < _dimensionality; i++)
            {
                solution[i] = values[i];
            }

            _f = 0.0;
            Array.Clear(_g, 0, _g.Length);
            Array.Clear(_lSave, 0, _lSave.Length);
            Array.Clear(_save, 0, _save.Length);
            Array.Clear(_dSave, 0, _dSave.Length);
            Array.Clear(_iwa, 0, _iwa.Length);
            Array.Clear(_work, 0, _work.Length);
            _task = Task.Start;
            _cSave = Task.None;
            _newF = 0;
            _newG = null;

            DateTime timestamp = DateTime.Now;
            while (Solver.ElapsedTime(timestamp) < duration)
            {
                Setup(_dimensionality, solution, lowerBounds, upperBounds, _nbd, ref _f, _g, _work, _iwa, ref _task, ref _cSave, _lSave, _save, _dSave);

                if (_task is not (Task.FgLN or Task.FgSt))
                {
                    continue;
                }

                _newF = _function(solution);
                _newG = _gradient(solution);
                _f = _newF;
                for (int i = 0; i < _dimensionality; i++)
                {
                    _g[i] = _newG[i];
                }
            }

            value = _function(solution);
        }
        
        /// <summary>
        /// Initializes where and projects the initial X to the feasible set if necessary.
        /// </summary>
        private static void Active(int n, double[] l, double[] u, int[] nbd, double[] x, int[] where, int whereOffset, out bool projected, out bool constrained, out bool boxed)
        {
            int i;
            projected = false;
            constrained = false;
            boxed = true;
            
            // Project the initial X to the feasible set if necessary.
            for (i = 1; i <= n; i++)
            {
                switch (nbd[i - 1])
                {
                    case <= 0:
                        continue;
                    case <= 2 when x[i - 1] <= l[i - 1]:
                    {
                        if (x[i - 1] < l[i - 1])
                        {
                            projected = true;
                            x[i - 1] = l[i - 1];
                        }

                        break;
                    }
                    case >= 2 when x[i - 1] >= u[i - 1]:
                    {
                        if (x[i - 1] > u[i - 1])
                        {
                            projected = true;
                            x[i - 1] = u[i - 1];
                        }

                        break;
                    }
                }
            }

            // Initialize where and assign values to constrained and boxed.
            for (i = 1; i <= n; i++)
            {
                if (nbd[i - 1] != 2)
                {
                    boxed = false;
                }

                if (nbd[i - 1] == 0)
                {
                    // This variable is always free.
                    where[i - 1 + whereOffset] = -1;
                }
                else
                {
                    constrained = true;
                    if (nbd[i - 1] == 2 && u[i - 1] - l[i - 1] <= 0.0)
                    {
                        // This variable is always fixed.
                        where[i - 1 + whereOffset] = 3;
                    }
                    else
                    {
                        where[i - 1 + whereOffset] = 0;
                    }
                }
            }
        }

        /// <summary>
        /// Solves systems of the form t * x = b trans(t) * x = b where t is a triangular matrix of order n.
        /// </summary>
        private static void Distribute(double[] t, int tOffset, int ldt, int n, double[] b, int bOffset, int job)
        {
            double temp;
            int j;
            int jj;

            // Determine the task and go to it.
            int @case = 1;

            if (job % 10 != 0)
            {
                @case = 2;
            }

            if (job % 100 / 10 != 0)
            {
                @case += 2;
            }

            switch (@case)
            {
                case 1:
                    goto L20;
                case 2:
                    goto L50;
                case 3:
                    goto L80;
            }

            goto L110;

        // Solve t * x = b for t lower triangular.
        L20:

            b[bOffset] /= t[tOffset];

            if (n < 2)
            {
                return;
            }

            for (j = 2; j <= n; j++)
            {
                temp = -b[j - 2 + bOffset];
                ConstantMultiply(n - j + 1, temp, t, j - 1 + (j - 2) * ldt + tOffset, 1, b, j - 1 + bOffset, 1);
                b[j - 1 + bOffset] /= t[j - 1 + (j - 1) * ldt + tOffset];
            }

            return;

        // Solve t * x = b for t upper triangular.
        L50:

            b[n - 1 + bOffset] /= t[n - 1 + (n - 1) * ldt + tOffset];

            if (n < 2)
            {
                return;
            }

            for (jj = 2; jj <= n; jj++)
            {
                j = n - jj + 1;
                temp = -b[j + bOffset];
                ConstantMultiply(j, temp, t, j * ldt + tOffset, 1, b, bOffset, 1);

                b[j - 1 + bOffset] /= t[j - 1 + (j - 1) * ldt + tOffset];
            }

            return;
            
        // Solve trans(t) * x = b for t lower triangular.
        L80:
            b[n - 1 + bOffset] /= t[n - 1 + (n - 1) * ldt + tOffset];

            if (n < 2)
            {
                return;
            }

            for (jj = 2; jj <= n; jj++)
            {
                j = n - jj + 1;
                b[j - 1 + bOffset] -= Dot(jj - 1, t, j + (j - 1) * ldt + tOffset, 1, b, j + bOffset, 1);

                b[j - 1 + bOffset] /= t[j - 1 + (j - 1) * ldt + tOffset];
            }

            return;

        // Solve trans(t)*x=b for t upper triangular.
        L110:
            b[bOffset] /= t[tOffset];

            if (n < 2)
            {
                return;
            }

            for (j = 2; j <= n; j++)
            {
                b[j - 1 + bOffset] -= Dot(j - 1, t, (j - 1) * ldt + tOffset, 1, b, bOffset, 1);

                b[j - 1 + bOffset] /= t[j - 1 + (j - 1) * ldt + tOffset];
            }
        }

        /// <summary>
        /// Computes the product of the 2m x 2m middle matrix in the compact formula of B and a 2m vector v it returns the product in p
        /// </summary>
        private static void Bmv(double[] sy, int syOffset, double[] wt, int wtOffset, int col, double[] v, int vOffset, double[] p, int pOffset)
        {
            int i;
            int k;
            double sum;

            if (col == 0)
            {
                return;
            }
            
            // Solve [  D^(1/2)      O ] [ p1 ] = [ v1 ]
            //       [ -L*D^(-1/2)   J ] [ p2 ]   [ v2 ]
            // Solve Jp2 = v2 + LD^(-1) v1
            p[col + pOffset] = v[col + vOffset];
            for (i = 2; i <= col; i++)
            {
                int i2 = col + i;
                sum = 0.0e0;
                for (k = 1; k <= i - 1; k++)
                {
                    sum += sy[i - 1 + (k - 1) + syOffset] * v[k - 1 + vOffset] / sy[k - 1 + (k - 1) + syOffset];
                }

                p[i2 - 1 + pOffset] = v[i2 - 1 + vOffset] + sum;
            }

            // Solve the triangular system.
            Distribute(wt, wtOffset, 1, col, p, col + pOffset, 11);

            // Solve D^(1 / 2) p1 = v1.
            for (i = 1; i <= col; i++)
            {
                p[i - 1 + pOffset] = v[i - 1 + vOffset] / math.sqrt(sy[i - 1 + (i - 1) + syOffset]);
            }

            // Solve [ -D^(1/2)   D^(-1/2)*L'  ] [ p1 ] = [ p1 ]
            //       [  0         J'           ] [ p2 ]   [ p2 ]
            // Solve J^Tp2 = p2
            Distribute(wt, wtOffset, 1, col, p, col + pOffset, 01);

            // Compute p1 = -D^(-1 / 2) (p1 - D^(-1 / 2) L'p2) = -D^(-1 / 2) p1 + D^(-1) L'p2
            for (i = 1; i <= col; i++)
            {
                p[i - 1 + pOffset] = -(p[i - 1 + pOffset] / math.sqrt(sy[i - 1 + (i - 1) + syOffset]));
            }

            for (i = 1; i <= col; i++)
            {
                sum = 0.0e0;
                for (k = i + 1; k <= col; k++)
                {
                    sum += sy[k - 1 + (i - 1) + syOffset] * p[col + k - 1 + pOffset] / sy[i - 1 + (i - 1) + syOffset];
                }

                p[i - 1 + pOffset] += sum;
            }
        }

        /// <summary>
        /// Computes the generalized Cauchy point
        /// </summary>
        private static void Cauchy(int n, double[] x, double[] l, double[] u, int[] nbd, double[] g, int[] order, int orderOffset, int[] where, int whereOffset, double[] t, int tOffset, double[] d, int dOffset, double[] xcp, int xcpOffset, double[] wy, int wyOffset, double[] ws, int wsOffset, double[] sy, int syOffset, double[] wt, int wtOffset, double theta, int col, int head, double[] p, int pOffset, double[] c, int cOffset, double[] wbp, int wbpOffset, double[] v, int vOffset, ref int numberSegment, double sbgNumber, double e)
        {
            int i;
            int j;
            int pointer;
            int ibp;
            double z;
            double tu = 0.0d;
            double tl = 0.0d;

            if (sbgNumber <= 0.0)
            {
                Copy(n, x, 0, 1, xcp, xcpOffset, 1);
                return;
            }

            bool bounded = true;
            int numberFree = n + 1;
            int numberBreak = 0;
            int iMin = 0;
            double min = 0.0;
            int col2 = 2 * col;
            double f1 = 0.0;

            // We set p to zero and build it up as we determine d.
            for (i = 1; i <= col2; i++)
            {
                p[i - 1 + pOffset] = 0.0;
            }
            
            // In the following loop we determine for each variable its bound status and its breakpoint, and update p accordingly.
            for (i = 1; i <= n; i++)
            {
                double neg = -g[i - 1];

                if (where[i - 1 + whereOffset] != 3 && where[i - 1 + whereOffset] != -1)
                {
                    // If x(i) is not a constant and has bounds, compute the difference between x(i) and its bounds.
                    if (nbd[i - 1] <= 2)
                    {
                        tl = x[i - 1] - l[i - 1];
                    }

                    if (nbd[i - 1] >= 2)
                    {
                        tu = u[i - 1] - x[i - 1];
                    }

                    // If a variable is close enough to a bound we treat it as at bound.
                    bool lower = nbd[i - 1] <= 2 && tl <= 0.0;
                    bool upper = nbd[i - 1] >= 2 && tu <= 0.0;

                    where[i - 1 + whereOffset] = 0;
                    if (lower)
                    {
                        if (neg <= 0.0)
                        {
                            where[i - 1 + whereOffset] = 1;
                        }
                    }
                    else if (upper)
                    {
                        if (neg >= 0.0)
                        {
                            where[i - 1 + whereOffset] = 2;
                        }
                    }
                    else
                    {
                        if (math.abs(neg) <= 0.0)
                        {
                            where[i - 1 + whereOffset] = -3;
                        }
                    }
                }

                pointer = head;
                if (where[i - 1 + whereOffset] != 0 && where[i - 1 + whereOffset] != -1)
                {
                    d[i - 1 + dOffset] = 0.0;
                }
                else
                {
                    d[i - 1 + dOffset] = neg;
                    f1 -= neg * neg;

                    // Calculate p = p - W'e_i * (g_i)
                    for (j = 1; j <= col; j++)
                    {
                        p[j - 1 + pOffset] += wy[i - 1 + (pointer - 1) * n + wyOffset] * neg;
                        p[col + j - 1 + pOffset] += ws[i - 1 + (pointer - 1) * n + wsOffset] * neg;
                        pointer += 1;
                    }

                    switch (nbd[i - 1])
                    {
                        case <= 2 when nbd[i - 1] != 0 && neg < 0.0:
                        {
                            // x(i) + d(i) is bounded.

                            numberBreak += 1;
                            order[numberBreak - 1 + orderOffset] = i;
                            t[numberBreak - 1 + tOffset] = tl / -neg;
                            if (numberBreak == 1 || t[numberBreak - 1 + tOffset] < min)
                            {
                                min = t[numberBreak - 1 + tOffset];
                                iMin = numberBreak;
                            }

                            break;
                        }
                        case >= 2 when neg > 0.0:
                        {
                            // x(i) + d(i) is bounded.

                            numberBreak += 1;
                            order[numberBreak - 1 + orderOffset] = i;
                            t[numberBreak - 1 + tOffset] = tu / neg;
                            if (numberBreak == 1 || t[numberBreak - 1 + tOffset] < min)
                            {
                                min = t[numberBreak - 1 + tOffset];
                                iMin = numberBreak;
                            }

                            break;
                        }
                        default:
                        {
                            // x(i) + d(i) is not bounded.
                            numberFree -= 1;
                            order[numberFree - 1 + orderOffset] = i;
                            if (math.abs(neg) > 0.0)
                            {
                                bounded = false;
                            }

                            break;
                        }
                    }
                }
            }
            
            // Complete the initialization of p for theta not= one.
            Scale(col, theta, p, col + pOffset, 1);
            
            // Initialize GCP xcp = x.
            Copy(n, x, 0, 1, xcp, xcpOffset, 1);

            if (numberBreak == 0 && numberFree == n + 1)
            {
                // Is a zero vector, return with the initial xcp as GCP.
                return;
            }

            // Initialize c = W'(xcp - x) = 0
            for (j = 1; j <= col2; j++)
            {
                c[j - 1 + cOffset] = 0.0;
            }

            // Initialize derivative f2.
            double f2 = -(theta * f1);
            double f2Org = f2;
            if (col > 0)
            {
                Bmv(sy, syOffset, wt, wtOffset, col, p, pOffset, v, vOffset);

                f2 -= Dot(col2, v, vOffset, 1, p, pOffset, 1);
            }

            double dtm = -(f1 / f2);
            double sum = 0.0;
            numberSegment = 1;

            // If there are no breakpoints, locate the GCP and return.
            if (numberBreak == 0)
            {
                goto L888;
            }

            int left = numberBreak;
            int iter = 1;
            double tj = 0.0;
            
            L777:
            // Find the next smallest breakpoint;
            double tj0 = tj;

            if (iter == 1)
            {
                // Since we already have the smallest breakpoint we need not do heapsort yet. Often only one breakpoint is used and the cost of heapsort is avoided.
                tj = min;
                ibp = order[iMin - 1 + orderOffset];
            }
            else
            {
                if (iter == 2)
                {
                    // Replace the already used smallest breakpoint with the breakpoint numbered break > last, before heapsort call.
                    if (iMin != numberBreak)
                    {
                        t[iMin - 1 + tOffset] = t[numberBreak - 1 + tOffset];
                        order[iMin - 1 + orderOffset] = order[numberBreak - 1 + orderOffset];
                    }
                }
                HeapSort(left, t, tOffset, order, orderOffset, iter - 2);
                tj = t[left - 1 + tOffset];
                ibp = order[left - 1 + orderOffset];
            }

            double dt = tj - tj0;

            // If a minimizer is within this interval, locate the GCP and return.
            if (dtm < dt)
            {
                goto L888;
            }

            // Otherwise fix one variable and reset the corresponding component of d to zero.
            sum += dt;
            left -= 1;
            iter += 1;
            double dib = d[ibp - 1 + dOffset];
            d[ibp - 1 + dOffset] = 0.0;

            if (dib > 0.0)
            {
                z = u[ibp - 1] - x[ibp - 1];
                xcp[ibp - 1 + xcpOffset] = u[ibp - 1];
                where[ibp - 1 + whereOffset] = 2;
            }
            else
            {
                z = l[ibp - 1] - x[ibp - 1];
                xcp[ibp - 1 + xcpOffset] = l[ibp - 1];
                where[ibp - 1 + whereOffset] = 1;
            }

            if (left == 0 && numberBreak == n)
            {
                // All n variables are fixed, return with xcp as GCP.
                dtm = dt;
                goto L999;
            }

            // Update the derivative information.
            numberSegment += 1;
            double dib2 = math.pow(dib, 2);

            // Update f1 and f2.
            f1 = f1 + dt * f2 + dib2 - theta * dib * z;
            f2 -= theta * dib2;
            if (col > 0)
            {
                // Update c = c + dt * p
                ConstantMultiply(col2, dt, p, pOffset, 1, c, cOffset, 1);

                // Choose wbp, the row of W corresponding to the breakpoint encountered.
                pointer = head;
                for (j = 1; j <= col; j++)
                {
                    wbp[j - 1 + wbpOffset] = wy[ibp - 1 + (pointer - 1) * n + wyOffset];

                    wbp[col + j - 1 + wbpOffset] = theta * ws[ibp - 1 + (pointer - 1) * n + wsOffset];

                    pointer += 1;
                }

                // Compute (wbp)Mc, (wbp)Mp, and (wbp)M(wbp)'
                Bmv(sy, syOffset, wt, wtOffset, col, wbp, wbpOffset, v, vOffset);

                double wmc = Dot(col2, c, cOffset, 1, v, vOffset, 1);
                double wmp = Dot(col2, p, pOffset, 1, v, vOffset, 1);
                double wmw = Dot(col2, wbp, wbpOffset, 1, v, vOffset, 1);

                // Update p = p - dip * wbp.
                ConstantMultiply(col2, -dib, wbp, wbpOffset, 1, p, pOffset, 1);

                // Complete updating f1 and f2 while col > 0.
                f1 += dib * wmc;
                f2 = f2 + 2.0e0 * dib * wmp - dib2 * wmw;
            }


            f2 = math.max(e * f2Org, f2);

            if (left > 0)
            {
                dtm = -(f1 / f2);
                goto L777;
            }

            if (bounded)
            {
                dtm = 0.0;
            }
            else
            {
                dtm = -(f1 / f2);
            }
            
            L888:
            if (dtm <= 0.0)
            {
                dtm = 0.0;
            }
            sum += dtm;

            // Move free variables (i.e., the ones w/o breakpoints) and the variables whose breakpoints haven't been reached.
            ConstantMultiply(n, sum, d, dOffset, 1, xcp, xcpOffset, 1);
            
            L999:
            // Update c = c + dtm*p = W'(x^c - x) which will be used in computing r = Z'(B(x^c - x) + g)
            if (col > 0)
            {
                ConstantMultiply(col2, dtm, p, pOffset, 1, c, cOffset, 1);
            }
        }

        /// <summary>
        /// This subroutine computes r=-Z'B(xcp-xk)-Z'g by using wa(2m+1)=W'(xcp-x) from subroutine cauchy.
        /// </summary>
        private static void Compromise(int n, int m, double[] x, double[] g, double[] ws, int wsOffset, double[] wy, int wyOffset, double[] sy, int syOffset, double[] wt, int wtOffset, double[] z, int zOffset, double[] r, int rOffset, double[] wa, int waOffset, int[] index, int indexOffset, double theta, int col, int head, int free, bool constrained)
        {
            int i;

            if (!constrained && col > 0)
            {
                for (i = 1; i <= n; i++)
                {
                    r[i - 1 + rOffset] = -g[i - 1];
                }
            }
            else
            {
                int k;
                for (i = 1; i <= free; i++)
                {
                    k = index[i - 1 + indexOffset];
                    r[i - 1 + rOffset] = -(theta * (z[k - 1 + zOffset] - x[k - 1])) - g[k - 1];
                }

                Bmv(sy, syOffset, wt, wtOffset, col, wa, 2 * m + waOffset, wa, waOffset);

                int pointer = head;

                int j;
                for (j = 1; j <= col; j++)
                {
                    double a1 = wa[j - 1 + waOffset];
                    double a2 = theta * wa[col + j - 1 + waOffset];
                    for (i = 1; i <= free; i++)
                    {
                        k = index[i - 1 + indexOffset];
                        r[i - 1 + rOffset] = r[i - 1 + rOffset] + wy[k - 1 + (pointer - 1) * n + wyOffset] * a1 + ws[k - 1 + (pointer - 1) * n + wsOffset] * a2;
                    }

                    pointer = pointer % m + 1;
                }
            }
        }

        /// <summary>
        /// Finds a step that satisfies a sufficient decrease condition and a curvature condition.
        /// </summary>
        private static void Decrease(double f, double g, ref double stp, double tolF, double tolG, double tolX, double stopMin, double stopMan, ref Task task, int[] save, int saveOffset, double[] doubleSave, int doubleSaveOffset)
        {
            bool bracket;
            int stage;
            double finite;
            double fx;
            double fy;
            double gInit;
            double gTest;
            double gx;
            double gy;
            double stx;
            double sty;
            double min;
            double max;
            double width;
            double width1;

            if (task == Task.Start)
            {
                // Initialize local variables.
                bracket = false;
                stage = 1;
                finite = f;
                gInit = g;
                gTest = tolF * gInit;
                width = stopMan - stopMin;
                width1 = width / 0.5;

                // The variables stx, fx, gx contain the values of the step, function, and derivative at the best step. 
                // The variables sty, fy, gy contain the value of the step, function, and derivative at sty.
                // The variables stp, f, g contain the values of the step, function, and derivative at stp.
                stx = 0.0;
                fx = finite;
                gx = gInit;
                sty = 0.0;
                fy = finite;
                gy = gInit;
                min = 0.0;
                max = stp + 4.0 * stp;
                task = Task.Fg;
                goto L1000;
            }

            // Restore local variables.
            bracket = save[saveOffset] == 1;
            stage = save[1 + saveOffset];
            gInit = doubleSave[doubleSaveOffset];
            gTest = doubleSave[1 + doubleSaveOffset];
            gx = doubleSave[2 + doubleSaveOffset];
            gy = doubleSave[3 + doubleSaveOffset];
            finite = doubleSave[4 + doubleSaveOffset];
            fx = doubleSave[5 + doubleSaveOffset];
            fy = doubleSave[6 + doubleSaveOffset];
            stx = doubleSave[7 + doubleSaveOffset];
            sty = doubleSave[8 + doubleSaveOffset];
            min = doubleSave[9 + doubleSaveOffset];
            max = doubleSave[10 + doubleSaveOffset];
            width = doubleSave[11 + doubleSaveOffset];
            width1 = doubleSave[12 + doubleSaveOffset];

            // If psi(stp) <= 0 and f'(stp) >= 0 for some step, then the algorithm enters the second stage.
            double test = finite + stp * gTest;
            if (stage == 1 && f <= test && g >= 0.0)
            {
                stage = 2;
            }

            // Test for warnings.
            if (bracket && (stp <= min || stp >= max))
            {
                task = Task.Warning;
            }
            if (bracket && max - min <= tolX * max)
            {
                task = Task.Warning;
            }

            // Test for convergence.
            if (f <= test && math.abs(g) <= tolG * -gInit)
            {
                task = Task.Convergence;
            }
            
            // Test for termination.
            if (task is Task.Warning or Task.Convergence)
            {
                goto L1000;
            }

            // A modified function is used to predict the step during the first stage if a lower function value has been obtained but the decrease is not sufficient.
            if (stage == 1 && f <= fx && f > test)
            {
                // Define the modified function and derivative values.
                double fm = f - stp * gTest;
                double fxm = fx - stx * gTest;
                double fym = fy - sty * gTest;
                double gm = g - gTest;
                double gxm = gx - gTest;
                double gym = gy - gTest;

                // Call to update stx, sty, and to compute the new step.
                SafeguardStep(ref stx, ref fxm, ref gxm, ref sty, ref fym, ref gym, ref stp, fm, gm, ref bracket, min, max);

                // Reset the function and derivative values for f.
                fx = fxm + stx * gTest;
                fy = fym + sty * gTest;
                gx = gxm + gTest;
                gy = gym + gTest;
            }
            else
            {
                // Call to update stx, sty, and to compute the new step.
                SafeguardStep(ref stx, ref fx, ref gx, ref sty, ref fy, ref gy, ref stp, f, g, ref bracket, min, max);
            }

            // Decide if a bisection step is needed.
            if (bracket)
            {
                if (math.abs(sty - stx) >= 0.6600000000000000310862446895043831318617 * width1)
                {
                    stp = stx + 0.5 * (sty - stx);
                }
                width1 = width;
                width = math.abs(sty - stx);
            }
            
            // Set the minimum and maximum steps allowed for stp.
            if (bracket)
            {
                min = math.min(stx, sty);
                max = math.max(stx, sty);
            }
            else
            {
                min = stp + 1.100000000000000088817841970012523233891 * (stp - stx);
                max = stp + 4.0 * (stp - stx);
            }

            // Force the step to be within the bounds max and min.
            stp = math.max(stp, stopMin);
            stp = math.min(stp, stopMan);
            
            // If further progress is not possible, let stp be the best point obtained during the search.
            if ((bracket && (stp <= min || stp >= max)) || (bracket && max - min <= tolX * max))
            {
                stp = stx;
            }
            
            // Obtain another function and derivative.
            task = Task.Fg;

        L1000:

            // Save local variables.
            save[saveOffset] = bracket ? 1 : 0;

            save[1 + saveOffset] = stage;
            doubleSave[doubleSaveOffset] = gInit;
            doubleSave[1 + doubleSaveOffset] = gTest;
            doubleSave[2 + doubleSaveOffset] = gx;
            doubleSave[3 + doubleSaveOffset] = gy;
            doubleSave[4 + doubleSaveOffset] = finite;
            doubleSave[5 + doubleSaveOffset] = fx;
            doubleSave[6 + doubleSaveOffset] = fy;
            doubleSave[7 + doubleSaveOffset] = stx;
            doubleSave[8 + doubleSaveOffset] = sty;
            doubleSave[9 + doubleSaveOffset] = min;
            doubleSave[10 + doubleSaveOffset] = max;
            doubleSave[11 + doubleSaveOffset] = width;
            doubleSave[12 + doubleSaveOffset] = width1;
        }
        
        /// <summary>
        /// Computes a safeguarded step for a search procedure and updates an interval that contains a step that satisfies a sufficient decrease and a curvature condition.
        /// </summary>
        private static void SafeguardStep(ref double stx, ref double fx, ref double dx, ref double sty, ref double fy, ref double dy, ref double stp, double fp, double dp, ref bool bracket, double stepMin, double stepMax)
        {
            double gamma;
            double p;
            double q;
            double r;
            double s;
            double stopC;
            double stopF;
            double stopQ;
            double theta;
            double signed = dp * (dx / math.abs(dx));

            // First case: A higher function value. The minimum is bracketed. 
            if (fp > fx)
            {
                theta = 3.0 * (fx - fp) / (stp - stx) + dx + dp;

                s = math.max(math.abs(theta), math.max(math.abs(dx), math.abs(dp)));

                gamma = s * math.sqrt(math.pow(theta / s, 2) - dx / s * (dp / s));
                if (stp < stx)
                {
                    gamma = -gamma;
                }
                p = gamma - dx + theta;
                q = gamma - dx + gamma + dp;
                r = p / q;
                stopC = stx + r * (stp - stx);
                stopQ = stx + dx / ((fx - fp) / (stp - stx) + dx) / 2.0 * (stp - stx);

                if (math.abs(stopC - stx) < math.abs(stopQ - stx))
                {
                    stopF = stopC;
                }
                else
                {
                    stopF = stopC + (stopQ - stopC) / 2.0;
                }

                bracket = true;
            }
            // Second case: A lower function value and derivatives of opposite 
            else if (signed < 0.0)
            {
                theta = 3.0 * (fx - fp) / (stp - stx) + dx + dp;
                s = math.max(math.abs(theta), math.max(math.abs(dx), math.abs(dp)));
                gamma = s * math.sqrt(math.pow(theta / s, 2) - dx / s * (dp / s));

                if (stp > stx)
                {
                    gamma = -gamma;
                }

                p = gamma - dp + theta;
                q = gamma - dp + gamma + dx;
                r = p / q;
                stopC = stp + r * (stx - stp);
                stopQ = stp + dp / (dp - dx) * (stx - stp);

                stopF = math.abs(stopC - stp) > math.abs(stopQ - stp) ? stopC : stopQ;

                bracket = true;
            }
            // Third case: A lower function value, derivatives of the same sign, and the magnitude of the derivative decreases.
            else if (math.abs(dp) < math.abs(dx))
            {
                theta = 3.0 * (fx - fp) / (stp - stx) + dx + dp;
                s = math.max(math.abs(theta), math.max(math.abs(dx), math.abs(dp)));

                // The case gamma = 0 only arises if the cubic does not tend to infinity in the direction of the step.
                gamma = s * math.sqrt(math.max(0.0, math.pow(theta / s, 2) - dx / s * (dp / s)));

                if (stp > stx)
                {
                    gamma = -gamma;
                }

                p = gamma - dp + theta;
                q = gamma + (dx - dp) + gamma;
                r = p / q;

                if (r < 0.0 && gamma != 0.0)
                {
                    stopC = stp + r * (stx - stp);
                }
                else if (stp > stx)
                {
                    stopC = stepMax;
                }
                else
                {
                    stopC = stepMin;
                }

                stopQ = stp + dp / (dp - dx) * (stx - stp);

                if (bracket)
                {
                    // A minimizer has been bracketed. If the cubic step is closer to stp than the secant step, the cubic step is taken, otherwise the secant step is taken.
                    stopF = math.abs(stopC - stp) < math.abs(stopQ - stp) ? stopC : stopQ;
                    stopF = stp > stx ? math.min(stp + 0.6600000000000000310862446895043831318617 * (sty - stp), stopF) : math.max(stp + 0.6600000000000000310862446895043831318617 * (sty - stp), stopF);
                }
                else
                {
                    // A minimizer has not been bracketed. If the cubic step is farther from stp than the secant step, the cubic step is taken, otherwise the secant step is taken.
                    stopF = math.abs(stopC - stp) > math.abs(stopQ - stp) ? stopC : stopQ;
                    stopF = math.min(stepMax, stopF);
                    stopF = math.max(stepMin, stopF);
                }
            }
            // Fourth case: A lower function value, derivatives of the same sign, and the magnitude of the derivative does not decrease.
            else
            {
                if (bracket)
                {
                    theta = 3.0 * (fp - fy) / (sty - stp) + dy + dp;
                    s = math.max(math.abs(theta), math.max(math.abs(dy), math.abs(dp)));
                    gamma = s * math.sqrt(math.pow(theta / s, 2) - dy / s * (dp / s));

                    if (stp > sty)
                    {
                        gamma = -gamma;
                    }

                    p = gamma - dp + theta;
                    q = gamma - dp + gamma + dy;
                    r = p / q;
                    stopC = stp + r * (sty - stp);
                    stopF = stopC;
                }
                else if (stp > stx)
                {
                    stopF = stepMax;
                }
                else
                {
                    stopF = stepMin;
                }
            }

            // Update the interval which contains a minimizer.
            if (fp > fx)
            {
                sty = stp;
                fy = fp;
                dy = dp;
            }
            else
            {
                if (signed < 0.0)
                {
                    sty = stx;
                    fy = fx;
                    dy = dx;
                }
                stx = stp;
                fx = fp;
                dx = dp;
            }

            // Compute the new step.
            stp = stopF;
        }
        
        /// <summary>
        /// Checks the validity of the input data.
        /// </summary>
        private static void ErrorCheck(int n, double[] l, double[] u, int[] nbd, ref Task task, ref int k)
        {
            if (n <= 0)
            {
                task = Task.Error;
            }

            // Check the validity of the arrays nbd(i), u(i), and l(i).
            for (int i = 1; i <= n; i++)
            {
                if (nbd[i - 1] < 0 || nbd[i - 1] > 3)
                {
                    task = Task.Error;
                    k = i;
                }

                if (nbd[i - 1] == 2 && l[i - 1] > u[i - 1])
                {
                    task = Task.Error;
                    k = i;
                }
            }

        }

        /// <summary>
        /// Forms  the LEL^T factorization of the indefinite.
        /// </summary>
        private static void FormK(int n, int nsub, int[] ind, int indOffset, int enter, int leave, int[] index2, int index2Offset, int update, bool updated, double[] wn, int wnOffset, double[] wn1, int wn1Offset, int m, double[] ws, int wsOffset, double[] wy, int wyOffset, double[] sy, int syOffset, double theta, int col, int head)
        {
            int iPointer;
            int jPointer;
            int iy;
            int is2;
            int jy;
            int js;
            int k1;
            int k;
            int upper;
            double temp1;
            double temp2;
            double temp3;

            if (updated)
            {
                if (update > m)
                {
                    // Shift old part of WN1.
                    for (jy = 1; jy <= m - 1; jy++)
                    {
                        js = m + jy;
                        Copy(m - jy, wn1, jy + jy * 2 * m + wn1Offset, 1, wn1, jy - 1 + (jy - 1) * 2 * m + wn1Offset, 1);
                        Copy(m - jy, wn1, js + js * 2 * m + wn1Offset, 1, wn1, js - 1 + (js - 1) * 2 * m + wn1Offset, 1);
                        Copy(m - 1, wn1, m + 1 + jy * 2 * m + wn1Offset, 1, wn1, m + (jy - 1) * 2 * m + wn1Offset, 1);
                    }
                }

                // Put new rows in blocks (1,1), (2,1) and (2,2).
                int begin = nsub + 1;
                iy = col;
                is2 = m + col;
                iPointer = head + col - 1;
                if (iPointer > m)
                {
                    iPointer -= m;
                }
                jPointer = head;
                for (jy = 1; jy <= col; jy++)
                {
                    js = m + jy;
                    temp1 = 0.0;
                    temp2 = 0.0;
                    temp3 = 0.0;

                    // Compute element jy of row 'col'.
                    for (k = 1; k <= nsub; k++)
                    {
                        k1 = ind[k - 1 + indOffset];
                        temp1 += wy[k1 - 1 + (iPointer - 1) * n + wyOffset] * wy[k1 - 1 + (jPointer - 1) * n + wyOffset];
                    }

                    // Compute elements jy of row 'col'.
                    for (k = begin; k <= n; k++)
                    {
                        k1 = ind[k - 1 + indOffset];
                        temp2 += ws[k1 - 1 + (iPointer - 1) * n + wsOffset] * ws[k1 - 1 + (jPointer - 1) * n + wsOffset];
                        temp3 += ws[k1 - 1 + (iPointer - 1) * n + wsOffset] * wy[k1 - 1 + (jPointer - 1) * n + wyOffset];
                    }

                    wn1[iy - 1 + (jy - 1) * 2 * m + wn1Offset] = temp1;
                    wn1[is2 - 1 + (js - 1) * 2 * m + wn1Offset] = temp2;
                    wn1[is2 - 1 + (jy - 1) * 2 * m + wn1Offset] = temp3;
                    jPointer = jPointer % m + 1;
                }

                // Put new column in block (2,1).
                jy = col;
                jPointer = head + col - 1;
                if (jPointer > m)
                {
                    jPointer -= m;
                }
                iPointer = head;
                for (int i = 1; i <= col; i++)
                {
                    is2 = m + i;
                    temp3 = 0.0;
                    // Compute element i of column 'col'.
                    for (k = 1; k <= nsub; k++)
                    {
                        k1 = ind[k - 1 + indOffset];
                        temp3 += ws[k1 - 1 + (iPointer - 1) * n + wsOffset] * wy[k1 - 1 + (jPointer - 1) * n + wyOffset];
                    }

                    iPointer = iPointer % m + 1;
                    wn1[is2 - 1 + (jy - 1) * 2 * m + wn1Offset] = temp3;
                }

                upper = col - 1;
            }
            else
            {
                upper = col;
            }

            // Modify the old parts in blocks (1,1) and (2,2) due to changes in the set of free variables.
            iPointer = head;
            for (iy = 1; iy <= upper; iy++)
            {
                is2 = m + iy;
                jPointer = head;
                for (jy = 1; jy <= iy; jy++)
                {
                    js = m + jy;
                    temp1 = 0.0;
                    temp2 = 0.0;
                    temp3 = 0.0;
                    double temp4 = 0.0;
                    for (k = 1; k <= enter; k++)
                    {
                        k1 = index2[k - 1 + index2Offset];
                        temp1 += wy[k1 - 1 + (iPointer - 1) * n + wyOffset] * wy[k1 - 1 + (jPointer - 1) * n + wyOffset];
                        temp2 += ws[k1 - 1 + (iPointer - 1) * n + wsOffset] * ws[k1 - 1 + (jPointer - 1) * n + wsOffset];
                    }

                    for (k = leave; k <= n; k++)
                    {
                        k1 = index2[k - 1 + index2Offset];
                        temp3 += wy[k1 - 1 + (iPointer - 1) * n + wyOffset] * wy[k1 - 1 + (jPointer - 1) * n + wyOffset];
                        temp4 += ws[k1 - 1 + (iPointer - 1) * n + wsOffset] * ws[k1 - 1 + (jPointer - 1) * n + wsOffset];
                    }

                    wn1[iy - 1 + (jy - 1) * 2 * m + wn1Offset] = wn1[iy - 1 + (jy - 1) * 2 * m + wn1Offset] + temp1 - temp3;
                    wn1[is2 - 1 + (js - 1) * 2 * m + wn1Offset] = wn1[is2 - 1 + (js - 1) * 2 * m + wn1Offset] - temp2 + temp4;
                    jPointer = jPointer % m + 1;
                }

                iPointer = iPointer % m + 1;
            }

            // Modify the old parts in block (2, 1).
            iPointer = head;
            for (is2 = m + 1; is2 <= m + upper; is2++)
            {
                jPointer = head;
                for (jy = 1; jy <= upper; jy++)
                {
                    temp1 = 0.0;
                    temp3 = 0.0;
                    for (k = 1; k <= enter; k++)
                    {
                        k1 = index2[k - 1 + index2Offset];
                        temp1 += ws[k1 - 1 + (iPointer - 1) * n + wsOffset] * wy[k1 - 1 + (jPointer - 1) * n + wyOffset];
                    }

                    for (k = leave; k <= n; k++)
                    {
                        k1 = index2[k - 1 + index2Offset];
                        temp3 += ws[k1 - 1 + (iPointer - 1) * n + wsOffset] * wy[k1 - 1 + (jPointer - 1) * n + wyOffset];
                    }

                    if (is2 <= jy + m)
                    {
                        wn1[is2 - 1 + (jy - 1) * 2 * m + wn1Offset] = wn1[is2 - 1 + (jy - 1) * 2 * m + wn1Offset] + temp1 - temp3;
                    }
                    else
                    {
                        wn1[is2 - 1 + (jy - 1) * 2 * m + wn1Offset] = wn1[is2 - 1 + (jy - 1) * 2 * m + wn1Offset] - temp1 + temp3;
                    }

                    jPointer = jPointer % m + 1;
                }

                iPointer = iPointer % m + 1;
            }

            // Form the upper triangle of WN.
            int m2 = 2 * m;
            for (iy = 1; iy <= col; iy++)
            {
                is2 = col + iy;
                int is1 = m + iy;
                for (jy = 1; jy <= iy; jy++)
                {
                    js = col + jy;
                    int js1 = m + jy;
                    wn[jy - 1 + (iy - 1) * 2 * m + wnOffset] = wn1[iy - 1 + (jy - 1) * 2 * m + wn1Offset] / theta;
                    wn[js - 1 + (is2 - 1) * 2 * m + wnOffset] = wn1[is1 - 1 + (js1 - 1) * 2 * m + wn1Offset] * theta;
                }

                for (jy = 1; jy <= iy - 1; jy++)
                {
                    wn[jy - 1 + (is2 - 1) * 2 * m + wnOffset] = -wn1[is1 - 1 + (jy - 1) * 2 * m + wn1Offset];
                }

                for (jy = iy; jy <= col; jy++)
                {
                    wn[jy - 1 + (is2 - 1) * 2 * m + wnOffset] = wn1[is1 - 1 + (jy - 1) * 2 * m + wn1Offset];
                }

                wn[iy - 1 + (iy - 1) * 2 * m + wnOffset] += sy[iy - 1 + (iy - 1) * m + syOffset];
            }

            // Form the upper triangle of WN.
            Factor(wn, wnOffset, m2, col);

            int col2 = 2 * col;
            for (js = col + 1; js <= col2; js++)
            {
                Distribute(wn, wnOffset, m2, col, wn, (js - 1) * 2 * m + wnOffset, 11);
            }
            
            for (is2 = col + 1; is2 <= col2; is2++)
            {
                for (js = is2; js <= col2; js++)
                {
                    wn[is2 - 1 + (js - 1) * 2 * m + wnOffset] += Dot(col, wn, (is2 - 1) * 2 * m + wnOffset, 1, wn, (js - 1) * 2 * m + wnOffset, 1);
                }
            }

            // Factorization of (2,2) block of wn.
            Factor(wn, col + col * 2 * m + wnOffset, m2, col);
        }

        /// <summary>
        /// Forms the upper half of the pos.
        /// </summary>
        private static void FormT(int m, double[] wt, int wtOffset, double[] sy, int syOffset, double[] ss, int ssOffset, int col, double theta)
        {
            int j;

            for (j = 1; j <= col; j++)
            {
                wt[(j - 1) * m + wtOffset] = theta * ss[(j - 1) * m + ssOffset];
            }

            int i;
            for (i = 2; i <= col; i++)
            {
                for (j = i; j <= col; j++)
                {
                    int k1 = math.min(i, j) - 1;

                    double dum = 0.0;
                    int k;
                    for (k = 1; k <= k1; k++)
                    {
                        dum += sy[i - 1 + (k - 1) * m + syOffset] * sy[j - 1 + (k - 1) * m + syOffset] / sy[k - 1 + (k - 1) * m + syOffset];
                    }

                    wt[i - 1 + (j - 1) * m + wtOffset] = dum + theta * ss[i - 1 + (j - 1) * m + ssOffset];
                }
            }
            
            Factor(wt, wtOffset, m, col);
        }

        /// <summary>
        /// Counts the entering and leaving variables when and finds the index set of free and active variables at the GCP.
        /// </summary>
        private static void FreeV(int n, ref int free, int[] index, int indexOffset, out int enter, out int leave, int[] index2, int index2Offset, int[] where, int whereOffset, out bool wrk, bool updated, bool constrained, int iter)
        {
            int i;

            enter = 0;
            leave = n + 1;

            if (iter > 0 && constrained)
            {
                // Count the entering and leaving variables.
                int k;
                for (i = 1; i <= free; i++)
                {
                    k = index[i - 1 + indexOffset];

                    if (where[k - 1 + whereOffset] > 0)
                    {
                        leave -= 1;
                        index2[leave - 1 + index2Offset] = k;
                    }
                }

                for (i = 1 + free; i <= n; i++)
                {
                    k = index[i - 1 + indexOffset];
                    if (where[k - 1 + whereOffset] <= 0)
                    {
                        enter += 1;
                        index2[enter - 1 + index2Offset] = k;
                    }
                }
            }

            wrk = leave < n + 1 || enter > 0 || updated;

            // Find the index set of free and active variables at the GCP.
            free = 0;
            int act = n + 1;
            for (i = 1; i <= n; i++)
            {
                if (where[i - 1 + whereOffset] <= 0)
                {
                    free += 1;
                    index[free - 1 + indexOffset] = i;
                }
                else
                {
                    act -= 1;
                    index[act - 1 + indexOffset] = i;
                }
            }
        }

        /// <summary>
        /// Sorts out the least element of t, and puts the remaining elements of t in a heap.
        /// </summary>
        private static void HeapSort(int n, double[] t, int tOffset, int[] order, int orderOffset, int heap)
        {
            int i;
            int j;
            int index;
            double dum;

            if (heap == 0)
            {
                // Rearrange the elements t(1) to t(n) to form a heap.
                int k;
                for (k = 2; k <= n; k++)
                {
                    dum = t[k - 1 + tOffset];
                    index = order[k - 1 + orderOffset];
                    i = k;

                    L10:
                    if (i > 1)
                    {
                        j = i / 2;
                        if (dum < t[j - 1 + tOffset])
                        {
                            t[i - 1 + tOffset] = t[j - 1 + tOffset];
                            order[i - 1 + orderOffset] = order[j - 1 + orderOffset];
                            i = j;
                            goto L10;
                        }
                    }

                    t[i - 1 + tOffset] = dum;
                    order[i - 1 + orderOffset] = index;
                }
            }
            // Assign to 'out' the value of t(1), the least member of the heap, and rearrange the remaining members to form a heap as elements 1 to n-1 of t.
            if (n > 1)
            {
                i = 1;
                double out2 = t[tOffset];
                int indexOut = order[orderOffset];
                dum = t[n - 1 + tOffset];
                index = order[n - 1 + orderOffset];

            // Restore the heap 
            L30:
                j = i + i;
                if (j <= n - 1)
                {
                    if (t[j + tOffset] < t[j - 1 + tOffset])
                    {
                        j += 1;
                    }
                    if (t[j - 1 + tOffset] < dum)
                    {
                        t[i - 1 + tOffset] = t[j - 1 + tOffset];
                        order[i - 1 + orderOffset] = order[j - 1 + orderOffset];
                        i = j;
                        goto L30;
                    }
                }
                t[i - 1 + tOffset] = dum;
                order[i - 1 + orderOffset] = index;
                
                // Put the least member in t(n). 
                t[n - 1 + tOffset] = out2;
                order[n - 1 + orderOffset] = indexOut;
            }
        }

        /// <summary>
        /// Perform the line search
        /// </summary>
        private static void LineSearch(int n, double[] l, double[] u, int[] nbd, double[] x, double f, ref double fold, out double gd, ref double gold, double[] g, double[] d, int dOffset, double[] r, int rOffset, double[] t, int tOffset, ref double stp, ref double norm, ref double dtd, ref double xStep, ref double stepMax, int iter, ref int fun, ref int back, ref int negative, ref Task task, bool boxed, bool constrained, ref Task cSave, int[] save, int saveOffset, double[] doubleSave, int doubleSaveOffset)
        {
            int i;

            if (task == Task.FgLN)
            {
                goto L556;
            }

            dtd = Dot(n, d, dOffset, 1, d, dOffset, 1);
            norm = math.sqrt(dtd);
            
            // Determine the maximum step length.
            stepMax = 10000000000.0;
            if (constrained)
            {
                if (iter == 0)
                {
                    stepMax = 1.0;
                }
                else
                {
                    for (i = 1; i <= n; i++)
                    {
                        double a1 = d[i - 1 + dOffset];
                        if (nbd[i - 1] != 0)
                        {
                            double a2;
                            switch (a1)
                            {
                                case < 0.0 when nbd[i - 1] <= 2:
                                {
                                    a2 = l[i - 1] - x[i - 1];
                                    if (a2 >= 0.0)
                                    {
                                        stepMax = 0.0;
                                    }
                                    else if (a1 * stepMax < a2)
                                    {
                                        stepMax = a2 / a1;
                                    }

                                    break;
                                }
                                case > 0.0 when nbd[i - 1] >= 2:
                                {
                                    a2 = u[i - 1] - x[i - 1];
                                    if (a2 <= 0.0)
                                    {
                                        stepMax = 0.0;
                                    }
                                    else if (a1 * stepMax > a2)
                                    {
                                        stepMax = a2 / a1;
                                    }

                                    break;
                                }
                            }
                        }
                    }
                }
            }

            stp = iter == 0 && !boxed ? double.IsNaN(norm) ? stepMax : math.min(1.0 / norm, stepMax) : 1.0;

            Copy(n, x, 0, 1, t, tOffset, 1);
            Copy(n, g, 0, 1, r, rOffset, 1);

            fold = f;
            fun = 0;
            back = 0;
            cSave = Task.Start;

        L556:
            gd = Dot(n, g, 0, 1, d, dOffset, 1);

            if (fun == 0)
            {
                gold = gd;
                if (gd >= 0.0)
                {
                    // Line search is impossible.
                    return;
                }
            }

            Decrease(f, gd, ref stp, 0.001000000000000000020816681711721685132943, 0.9000000000000000222044604925031308084726, 0.1000000000000000055511151231257827021182, 0.0, stepMax, ref cSave, save, saveOffset, doubleSave, doubleSaveOffset);

            xStep = stp * norm;
            if (cSave != Task.Convergence && cSave != Task.Warning)
            {
                task = Task.FgLN;
                fun += 1;
                negative += 1;
                back = fun - 1;
                for (i = 1; i <= n; i++)
                {
                    x[i - 1] = stp * d[i - 1 + dOffset] + t[i - 1 + tOffset];
                }
            }
            else
            {
                task = Task.NewX;
            }
        }

        /// <summary>
        /// Solves bound constrained optimization problems by using the compact formula of the limited memory updates.
        /// </summary>
        private static void Main(int n, double[] x, double[] l, double[] u, int[] nbd, ref double f, double[] g, double[] ws, int wsOffset, double[] wy, int wyOffset, double[] sy, int syOffset, double[] ss, int ssOffset, double[] wt, int wtOffset, double[] wn, int wnOffset, double[] snd, int sndOffset, double[] z, int zOffset, double[] r, int rOffset, double[] d, int dOffset, double[] t, int tOffset, double[] xp, int xpOffset, double[] wa, int waOffset, int[] index, int indexOffset, int[] where, int whereOffset, int[] index2, int index2Offset, ref Task task, ref Task cSave, bool[] lSave, int[] save, double[] doubleSave)
        {
            bool projected;
            bool constrained;
            bool boxed;
            bool updated;
            int i;
            int k = 0;
            int inter;
            int file;
            int back;
            int skip;
            int head;
            int col;
            int iter;
            int tail;
            int update;
            int segment;
            int numberFigure;
            int fun;
            int word;
            int free;
            int act;
            int leave;
            int enter;
            double theta;
            double fold;
            double tol;
            double step = 0.0d;
            double sbgNumber;
            double norm;
            double dtd;
            double e;
            double cpu1;
            const double cpu2 = 0.0d;
            double catchy;
            double time;
            double length;
            double time1 = 0.0d;
            double gd;
            double gold;
            double stp;
            double stepMax;

            if (task == Task.Start)
            {
                e = 1.11022302462516E-16;

                // Initialize counters and scalars when task='START'.
                col = 0;
                head = 1;
                theta = 1.0;
                update = 0;
                updated = false;
                back = 0;
                tail = 0;
                word = 0;
                act = 0;
                leave = 0;
                enter = 0;
                fold = 0.0;
                norm = 0.0;
                cpu1 = 0.0;
                gd = 0.0;
                stepMax = 0.0;
                sbgNumber = 0.0;
                stp = 0.0;
                gold = 0.0;
                dtd = 0.0;
                iter = 0;
                numberFigure = 0;
                segment = 0;
                inter = 0;
                skip = 0;
                free = n;
                fun = 0;
                tol = 1e+5 * e;
                catchy = 0;
                time = 0;
                length = 0;
                file = 8;
                
                //Check the input arguments for errors.
                ErrorCheck(n, l, u, nbd, ref task, ref k);

                if (task == Task.Error)
                {
                    return;
                }

                // Initialize where & project x onto the feasible set.
                Active(n, l, u, nbd, x, where, whereOffset, out projected, out constrained, out boxed);
            }
            else
            {
                // Restore local variables.
                projected = lSave[0];
                constrained = lSave[1];
                boxed = lSave[2];
                updated = lSave[3];
                inter = save[21];
                file = save[23];
                back = save[24];
                skip = save[25];
                head = save[26];
                col = save[27];
                tail = save[28];
                iter = save[29];
                update = save[30];
                segment = save[32];
                numberFigure = save[33];
                fun = save[35];
                word = save[36];
                free = save[37];
                act = save[38];
                leave = save[39];
                enter = save[40];
                theta = doubleSave[0];
                fold = doubleSave[1];
                tol = doubleSave[2];
                norm = doubleSave[3];
                e = doubleSave[4];
                cpu1 = doubleSave[5];
                catchy = doubleSave[6];
                time = doubleSave[7];
                length = doubleSave[8];
                time1 = doubleSave[9];
                gd = doubleSave[10];
                stepMax = doubleSave[11];
                sbgNumber = doubleSave[12];
                stp = doubleSave[13];
                gold = doubleSave[14];
                dtd = doubleSave[15];
                switch (task)
                {
                    // After returning from the driver go to the point where execution is to resume.
                    case Task.FgLN:
                        goto L666;
                    case Task.NewX:
                        goto L777;
                    case Task.None:
                    case Task.Start:
                    case Task.Fg:
                    case Task.Abnormal:
                    case Task.Convergence:
                    case Task.Error:
                    case Task.RestartLN:
                    case Task.Warning:
                    default:
                    case Task.FgSt:
                        goto L111;
                }
            }

            // Compute f0 and g0.
            task = Task.FgSt;

            // Return to the driver to calculate f and g; reenter at 111.
            goto L1000;

        L111:

            numberFigure = 1;

            // Compute the infinity norm of the (-) projected gradient.
            Project(n, l, u, nbd, x, g, out sbgNumber);

            if (sbgNumber <= 0.0)
            {
                // Terminate the algorithm.
                task = Task.Convergence;
                goto L999;
            }
            
            L222:
            word = -1;
            // Compute the Generalized Cauchy Point (GCP).
            Cauchy(n, x, l, u, nbd, g, index2, index2Offset, where, whereOffset, t, tOffset, d, dOffset, z, zOffset,  wy, wyOffset, ws, wsOffset, sy, syOffset, wt, wtOffset, theta, col, head, wa, waOffset, wa, 2 + waOffset, wa, 4 + waOffset, wa, 6 + waOffset, ref segment, sbgNumber, e);

            catchy = catchy + cpu2 - cpu1;
            inter += segment;
            // Count the entering and leaving variables for iter > 0.
            FreeV(n, ref free, index, indexOffset, out enter, out leave, index2, index2Offset, where, whereOffset, out bool wrk, updated, constrained, iter);
            act = n - free;
            // If there are no free variables or B=theta*I, then skip the subspace minimization.
            if (free == 0 || col == 0)
            {
                goto L555;
            }
            // Subspace minimization.
            if (wrk)
            {
                FormK(n, free, index, indexOffset, enter, leave, index2, index2Offset, update, updated, wn, wnOffset, snd, sndOffset, 1, ws, wsOffset, wy, wyOffset, sy, syOffset, theta, col, head);
            }
            Compromise(n, 1, x, g, ws, wsOffset, wy, wyOffset, sy, syOffset, wt, wtOffset, z, zOffset, r, rOffset, wa, waOffset, index, indexOffset, theta, col, head, free, constrained);
            // Call the direct method. 
            Subspace(n, 1, free, index, indexOffset, l, u, nbd, z, zOffset, r, rOffset, xp, xpOffset, ws, wsOffset, wy, wyOffset, theta, col, head, ref word, wa, waOffset, wn, wnOffset);          

        L555:
            // Line search and optimality tests.
            for (i = 1; i <= n; i++)
            {
                d[i - 1 + dOffset] = z[i - 1 + zOffset] - x[i - 1];
            }

            L666:
            LineSearch(n, l, u, nbd, x, f, ref fold, out gd, ref gold, g, d, dOffset, r, rOffset, t, tOffset, ref stp, ref norm, ref dtd, ref step, ref stepMax, iter, ref fun, ref back, ref numberFigure, ref task, boxed, constrained, ref cSave, save, 42, doubleSave, 16);

            if (back >= 20)
            {
                // Restore the previous iterate.
                Copy(n, t, tOffset, 1, x, 0, 1);
                Copy(n, r, rOffset, 1, g, 0, 1);
                f = fold;
                if (col == 0)
                {
                    // Restore the actual number of f and g evaluations etc.
                    numberFigure -= 1;
                    fun -= 1;
                    back -= 1;
                    task = Task.Abnormal;
                    iter += 1;
                    goto L999;
                }

                numberFigure -= 1;
                col = 0;
                head = 1;
                theta = 1.0;
                update = 0;
                updated = false;
                task = Task.RestartLN;
                goto L222;
            }

            if (task == Task.FgLN)
            {
                // Return to the driver for calculating f and g; reenter at 666.
                goto L1000;
            }
            // Calculate and print out the quantities related to the new X.
            iter += 1;
            // Compute the infinity norm of the projected (-)gradient.
            Project(n, l, u, nbd, x, g, out sbgNumber);
            goto L1000;

            L777:
            // Test for termination.
            if (sbgNumber <= 0.0)
            {
                // Terminate the algorithm.
                task = Task.Convergence;
                goto L999;
            }

            double dum = math.max(math.abs(fold), math.max(math.abs(f), 1.0));

            if (fold - f <= tol * dum)
            {
                // terminate the algorithm.
                task = Task.Convergence;

                // i.e., to issue a warning if back>10 in the line search.
                goto L999;
            }

            for (i = 1; i <= n; i++)
            {
                r[i - 1 + rOffset] = g[i - 1] - r[i - 1 + rOffset];
            }

            double rr = Dot(n, r, rOffset, 1, r, rOffset, 1);

            double dr = (gd - gold) * stp;
            Scale(n, stp, d, dOffset, 1);
            dum = -(gold * stp);

            if (dr <= e * dum)
            {
                // Skip the update.
                skip += 1;
                updated = false;

                goto L888;
            }

            // Update the matrix.
            updated = true;
            update += 1;

            // Update matrices WS and WY and form the middle matrix in B.
            MatrixUpdate(n, 1, ws, wsOffset, wy, wyOffset, sy, syOffset, ss, ssOffset, d, dOffset, r, rOffset, ref tail, update, ref col, ref head, out theta, rr, dr, stp, dtd);
            
            FormT(1, wt, wtOffset, sy, syOffset, ss, ssOffset, col, theta);
            
        L888:
        goto L222;

        L999:
        L1000:
            // Save local variables.
            lSave[0] = projected;
            lSave[1] = constrained;
            lSave[2] = boxed;
            lSave[3] = updated;
            save[21] = inter;
            save[23] = file;
            save[24] = back;
            save[25] = skip;
            save[26] = head;
            save[27] = col;
            save[28] = tail;
            save[29] = iter;
            save[30] = update;
            save[32] = segment;
            save[33] = numberFigure;
            save[35] = fun;
            save[36] = word;
            save[37] = free;
            save[38] = act;
            save[39] = leave;
            save[40] = enter;
            doubleSave[0] = theta;
            doubleSave[1] = fold;
            doubleSave[2] = tol;
            doubleSave[3] = norm;
            doubleSave[4] = e;
            doubleSave[5] = cpu1;
            doubleSave[6] = catchy;
            doubleSave[7] = time;
            doubleSave[8] = length;
            doubleSave[9] = time1;
            doubleSave[10] = gd;
            doubleSave[11] = stepMax;
            doubleSave[12] = sbgNumber;
            doubleSave[13] = stp;
            doubleSave[14] = gold;
            doubleSave[15] = dtd;
        }

        /// <summary>
        /// Updates matrices WS and WY, and forms the middle matrix in B.
        /// </summary>
        private static void MatrixUpdate(int n, int m, double[] ws, int wsOffset, double[] wy, int wyOffset, double[] sy, int syOffset, double[] ss, int ssOffset, double[] d, int dOffset, double[] r, int rOffset, ref int tail, int update, ref int col, ref int head, out double theta, double rr, double dr, double stp, double dtd)
        {
            int j;

            if (update <= m)
            {
                col = update;
                tail = (head + update - 2) % m + 1;
            }
            else
            {
                tail = tail % m + 1;
                head = head % m + 1;
            }

            // Update matrices WS and WY.
            Copy(n, d, dOffset, 1, ws, (tail - 1) * n + wsOffset, 1);
            Copy(n, r, rOffset, 1, wy, (tail - 1) * n + wyOffset, 1);

            // Set theta=yy/ys.
            theta = rr / dr;

            // Form the middle matrix in B.
            if (update > m)
            {
                // Move old information
                for (j = 1; j <= col - 1; j++)
                {
                    Copy(j, ss, 1 + j * m + ssOffset, 1, ss, (j - 1) * m + ssOffset, 1);
                    Copy(col - j, sy, j + j * m + syOffset, 1, sy, j - 1 + (j - 1) * m + syOffset, 1);
                }
            }

            // Add new information: the last row of SY and the last column of SS.
            int pointer = head;
            for (j = 1; j <= col - 1; j++)
            {
                sy[col - 1 + (j - 1) * m + syOffset] = Dot(n, d, dOffset, 1, wy, (pointer - 1) * n + wyOffset, 1);
                ss[j - 1 + (col - 1) * m + ssOffset] = Dot(n, ws, (pointer - 1) * n + wsOffset, 1, d, dOffset, 1);
                pointer = pointer % m + 1;
            }

            ss[col - 1 + (col - 1) * m + ssOffset] = stp * stp * dtd;
            sy[col - 1 + (col - 1) * m + syOffset] = dr;
        }

        /// <summary>
        /// Computes the infinity norm of the projected gradient.
        /// </summary>
        private static void Project(int n, double[] l, double[] u, int[] nbd, double[] x, double[] g, out double number)
        {
            number = 0.0;
            int i;
            for (i = 1; i <= n; i++)
            {
                double gi = g[i - 1];
                if (nbd[i - 1] != 0)
                {
                    if (gi < 0.0)
                    {
                        if (nbd[i - 1] >= 2)
                        {
                            gi = math.max(x[i - 1] - u[i - 1], gi);
                        }
                    }
                    else
                    {
                        if (nbd[i - 1] <= 2)
                        {
                            gi = math.min(x[i - 1] - l[i - 1], gi);
                        }
                    }
                }

                number = math.max(number, math.abs(gi));
            }
        }

        /// <summary>
        /// Partitions the working arrays and  then uses the limited memory method to solve the bound constrained optimization problem.
        /// </summary>
        private static void Setup(int n, double[] x, double[] l, double[] u, int[] nbd, ref double f, double[] g, double[] wa, int[] iwa, ref Task task, ref Task cSave, bool[] lSave, int[] save, double[] doubleSave)
        {
            if (task == Task.Start)
            {
                save[0] = n;
                save[1] = 1;
                save[2] = 4;
                save[3] = 1;
                save[4] = save[3] + save[0];
                save[5] = save[4] + save[0];
                save[6] = save[5] + save[1];
                save[7] = save[6] + save[1];
                save[8] = save[7] + save[1];
                save[9] = save[8] + save[2];
                save[10] = save[9] + save[2];
                save[11] = save[10] + n;
                save[12] = save[11] + n;
                save[13] = save[12] + n;
                save[14] = save[13] + n;
                save[15] = save[14] + n;
            }
            Main(n, x, l, u, nbd, ref f, g, wa, save[3] - 1, wa, save[4] - 1, wa, save[5] - 1, wa, save[6] - 1, wa, save[7] - 1, wa, save[8] - 1, wa, save[9] - 1, wa, save[10] - 1, wa, save[11] - 1, wa, save[12] - 1, wa, save[13] - 1, wa, save[14] - 1, wa, save[15] - 1, iwa, 0, iwa, n, iwa, 2 * n, ref task, ref cSave, lSave, save, doubleSave);
        }
        
        /// <summary>
        /// Computes an approximate solution of the subspace problem.
        /// </summary>
        private static void Subspace(int n, int m, int nsub, int[] ind, int indOffset, double[] l, double[] u, int[] nbd, double[] x, int xOffset, double[] d, int dOffset, double[] xp, int xpOffset, double[] ws, int wsOffset, double[] wy, int wyOffset, double theta, int col, int head, ref int word, double[] wv, int wvOffset, double[] wn, int wnOffset)
        {
            int i;
            int k;

            if (nsub <= 0)
            {
                return;
            }

            // 
            // Compute wv = W'Zd.
            // 
            int pointer = head;
            for (i = 1; i <= col; i++)
            {
                double temp1 = 0.0;
                double temp2 = 0.0;
                int j;
                for (j = 1; j <= nsub; j++)
                {
                    k = ind[j - 1 + indOffset];
                    temp1 += wy[k - 1 + (pointer - 1) * n + wyOffset] * d[j - 1 + dOffset];
                    temp2 += ws[k - 1 + (pointer - 1) * n + wsOffset] * d[j - 1 + dOffset];
                }

                wv[i - 1 + wvOffset] = temp1;
                wv[col + i - 1 + wvOffset] = theta * temp2;
                pointer = pointer % m + 1;
            }

            int m2 = 2 * m;
            int col2 = 2 * col;

            Distribute(wn, wnOffset, m2, col2, wv, wvOffset, 11);

            for (i = 1; i <= col; i++)
            {
                wv[i - 1 + wvOffset] = -wv[i - 1 + wvOffset];
            }

            Distribute(wn, wnOffset, m2, col2, wv, wvOffset, 1);

            pointer = head;
            int jy;
            for (jy = 1; jy <= col; jy++)
            {
                int js = col + jy;
                for (i = 1; i <= nsub; i++)
                {
                    k = ind[i - 1 + indOffset];
                    d[i - 1 + dOffset] = d[i - 1 + dOffset] + wy[k - 1 + (pointer - 1) * n + wyOffset] * wv[jy - 1 + wvOffset] / theta + ws[k - 1 + (pointer - 1) * n + wsOffset] * wv[js - 1 + wvOffset];
                }

                pointer = pointer % m + 1;
            }


            Scale(nsub, 1.0 / theta, d, dOffset, 1);

            // Let us try the projection, d is the Newton direction.
            word = 0;

            Copy(n, x, xOffset, 1, xp, xpOffset, 1);

            for (i = 1; i <= nsub; i++)
            {
                k = ind[i - 1 + indOffset];
                double dk = d[i - 1 + dOffset];
                double xk = x[k - 1 + xOffset];
                if (nbd[k - 1] != 0)
                {
                    switch (nbd[k - 1])
                    {
                        case 1:
                        {
                            x[k - 1 + xOffset] = math.max(l[k - 1], xk + dk);

                            break;
                        }
                        case 2:
                        {
                            xk = math.max(l[k - 1], xk + dk);
                            x[k - 1 + xOffset] = math.min(u[k - 1], xk);

                            break;
                        }
                        case 3:
                        {
                            x[k - 1 + xOffset] = math.min(u[k - 1], xk + dk);

                            break;
                        }
                    }
                }
                else
                {
                    x[k - 1 + xOffset] = xk + dk;
                }
            }
        }

        /// <summary>
        /// Factors a double precision symmetric positive definite matrix.
        /// </summary>
        private static void Factor(double[] a, int aOffset, int lda, int n)
        {
            int j;
            for (j = 1; j <= n; j++)
            {
                double s = 0.0e0;
                int jm1 = j - 1;
                if (jm1 < 1)
                {
                    goto L20;
                }

                int k;
                for (k = 1; k <= jm1; k++)
                {
                    double t = a[k - 1 + (j - 1) * lda + aOffset] - Dot(k - 1, a, (k - 1) * lda + aOffset, 1, a, (j - 1) * lda + aOffset, 1);

                    t /= a[k - 1 + (k - 1) * lda + aOffset];
                    a[k - 1 + (j - 1) * lda + aOffset] = t;
                    s += t * t;
                }

                L20:
                s = a[j - 1 + (j - 1) * lda + aOffset] - s;
                if (s <= 0.0e0)
                {
                    return;
                }

                a[j - 1 + (j - 1) * lda + aOffset] = math.sqrt(s);
            }
        }

        /// <summary>
        /// Scales a vector by a constant.
        /// </summary>
        private static void Scale(int n, double da, double[] dx, int dxOffset, int increment)
        {
            int i;

            if (n <= 0 || increment <= 0)
            {
                return;
            }

            if (increment == 1)
            {
                goto L20;
            }
            
            int numberIncrement = n * increment;
            for (i = 1; i <= numberIncrement; i += increment)
            {
                dx[i - 1 + dxOffset] = da * dx[i - 1 + dxOffset];
            }

            return;
            
            // Code for increment equal to 1.
            L20:
            int m = n % 5;

            if (m == 0)
            {
                goto L40;
            }

            for (i = 1; i <= m; i++)
            {
                dx[i - 1 + dxOffset] = da * dx[i - 1 + dxOffset];
            }

            if (n < 5)
            {
                return;
            }

            L40:
            int mp1 = m + 1;
            const int iInc = 5;
            for (i = mp1; i <= n; i += iInc)
            {
                dx[i - 1 + dxOffset] = da * dx[i - 1 + dxOffset];
                dx[i + dxOffset] = da * dx[i + dxOffset];
                dx[i + 1 + dxOffset] = da * dx[i + 1 + dxOffset];
                dx[i + 2 + dxOffset] = da * dx[i + 2 + dxOffset];
                dx[i + 3 + dxOffset] = da * dx[i + 3 + dxOffset];
            }
        }

        /// <summary>
        /// Forms the dot product of two vectors.
        /// </summary>
        /// <returns>The dot product of two vectors.</returns>
        private static double Dot(int n, double[] dx, int dxOffset, int incrementX, double[] dy, int dyOffset, int incrementY)
        {
            int i;
            double dot = 0.0e0;
            double temp = 0.0e0;
            if (n <= 0)
            {
                return dot;
            }

            if (incrementX == 1 && incrementY == 1)
            {
                goto L20;
            }

            // For unequal increments or equal increments not equal to 1.
            int ix = 1;
            int iy = 1;
            if (incrementX < 0)
            {
                ix = (-n + 1) * incrementX + 1;
            }
            if (incrementY < 0)
            {
                iy = (-n + 1) * incrementY + 1;
            }

            for (i = 1; i <= n; i++)
            {
                temp += dx[ix - 1 + dxOffset] * dy[iy - 1 + dyOffset];
                ix += incrementX;
                iy += incrementY;
            }

            dot = temp;
            return dot;

            // For both increments equal to 1 clean-up loop.
            L20:
            int m = n % 5;

            if (m == 0)
            {
                goto L40;
            }

            for (i = 1; i <= m; i++)
            {
                temp += dx[i - 1 + dxOffset] * dy[i - 1 + dyOffset];
            }

            if (n < 5)
            {
                goto L60;
            }

            L40:
            int mp1 = m + 1;
            const int iInc = 5;
            for (i = mp1; i <= n; i += iInc)
            {
                temp = temp + dx[i - 1 + dxOffset] * dy[i - 1 + dyOffset] + dx[i + dxOffset] * dy[i + dyOffset] + dx[i + 1 + dxOffset] * dy[i + 1 + dyOffset] + dx[i + 2 + dxOffset] * dy[i + 2 + dyOffset] + dx[i + 3 + dxOffset] * dy[i + 3 + dyOffset];
            }

            L60:
            dot = temp;
            return dot;
        }

        /// <summary>
        /// Copy values.
        /// </summary>
        private static void Copy(int n, double[] dx, int dxOffset, int incrementX, double[] dy, int dyOffset, int incrementY)
        {
            int i;
            if (n <= 0)
            {
                return;
            }

            if (incrementX == 1 && incrementY == 1)
            {
                goto L20;
            }

            // Code for unequal increments or equal increments not equal to 1.
            int ix = 1;
            int iy = 1;
            if (incrementX < 0)
            {
                ix = (-n + 1) * incrementX + 1;
            }
            if (incrementY < 0)
            {
                iy = (-n + 1) * incrementY + 1;
            }

            for (i = 1; i <= n; i++)
            {
                dy[iy - 1 + dyOffset] = dx[ix - 1 + dxOffset];
                ix += incrementX;
                iy += incrementY;
            }

            return;
            
            // Code for both increments equal to 1 clean-up loop.
            L20:
            int m = n % 7;
            if (m == 0)
            {
                goto L40;
            }

            for (i = 1; i <= m; i++)
            {
                dy[i - 1 + dyOffset] = dx[i - 1 + dxOffset];
            }

            if (n < 7)
            {
                return;
            }

            L40:
            int mp1 = m + 1;
            const int iInc = 7;
            for (i = mp1; i <= n; i += iInc)
            {
                dy[i - 1 + dyOffset] = dx[i - 1 + dxOffset];
                dy[i + dyOffset] = dx[i + dxOffset];
                dy[i + 1 + dyOffset] = dx[i + 1 + dxOffset];
                dy[i + 2 + dyOffset] = dx[i + 2 + dxOffset];
                dy[i + 3 + dyOffset] = dx[i + 3 + dxOffset];
                dy[i + 4 + dyOffset] = dx[i + 4 + dxOffset];
                dy[i + 5 + dyOffset] = dx[i + 5 + dxOffset];
            }
        }

        /// <summary>
        /// Constant times a vector plus a vector.
        /// </summary>
        private static void ConstantMultiply(int n, double da, double[] dx, int dxOffset, int incrementX, double[] dy, int dyOffset, int incrementY)
        {
            int i;
            if (n <= 0 || da == 0.0e0)
            {
                return;
            }

            if (incrementX == 1 && incrementY == 1)
            {
                // Code for both increments equal to 1 clean-up loop.
                int m = n % 4;
                if (m != 0)
                {
                    for (i = 1; i <= m; i++)
                    {
                        dy[i - 1 + dyOffset] += da * dx[i - 1 + dxOffset];
                    }
                }

                if (n < 4)
                {
                    return;
                }

                int mp1 = m + 1;
                const int iInc = 4;
                for (i = mp1; i <= n; i += iInc)
                {
                    dy[i - 1 + dyOffset] += da * dx[i - 1 + dxOffset];
                    dy[i + dyOffset] += da * dx[i + dxOffset];
                    dy[i + 1 + dyOffset] += da * dx[i + 1 + dxOffset];
                    dy[i + 2 + dyOffset] += da * dx[i + 2 + dxOffset];
                }

                return;
            }

            // Code for unequal increments or equal increments not equal to 1.
            int ix = 1;
            int iy = 1;
            if (incrementX < 0)
            {
                ix = (-n + 1) * incrementX + 1;
            }
            if (incrementY < 0)
            {
                iy = (-n + 1) * incrementY + 1;
            }

            for (i = 1; i <= n; i++)
            {
                dy[iy - 1 + dyOffset] += da * dx[ix - 1 + dxOffset];
                ix += incrementX;
                iy += incrementY;
            }
        }
    }
}