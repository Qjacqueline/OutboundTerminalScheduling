import random
import time
from copy import deepcopy

import matplotlib.pyplot as plt
import numpy as np

import conf.configs as Cf
from algorithm_factory.algo_utils.machine_cal_methods import output_solution
from common.iter_solution import IterSolution
from data_process.input_process import write_json_to_file
from utils.log import Logger

logger = Logger().get_logger()
random.seed(Cf.RANDOM_SEED)
np.random.seed(Cf.RANDOM_SEED)


class GA(object):
    def __init__(self, population_size: int, iter_solu: IterSolution, crossover_rate: float = 0.8,
                 mutation_rate: float = 0.1, max_generations: int = 100):
        self.population_size: int = population_size
        self.crossover_rate: float = crossover_rate
        self.mutation_rate: float = mutation_rate
        self.max_generations: int = max_generations
        self.iter_solu: IterSolution = iter_solu
        self.population: list = [deepcopy(self.iter_solu) for _ in range(population_size)]
        self.best_solution: IterSolution = deepcopy(self.iter_solu)
        self.best_makespan: float = float('inf')
        self.iter_x: list = list()
        self.iter_y: list = list()

    def run(self, stop_time=float('inf')):
        T1 = time.time()
        count = 0
        tf1 = True
        record = []

        # Initialize population and evaluate fitness
        self.evaluate_population()

        while count < self.max_generations:
            count += 1

            # Selection
            parents = self.selection()

            # Crossover
            offspring = self.crossover(parents)

            # Mutation
            offspring = self.mutation(offspring)

            # Evaluate the new generation
            self.population = offspring
            self.evaluate_population()

            self.iter_x.append(count)
            self.iter_y.append(self.best_makespan)

            if time.time() - T1 > stop_time and tf1:
                record.append(self.best_makespan)
                print("best_makespan: " + str(self.best_makespan), end=" ")
                tf1 = False

        return record

    def evaluate_population(self):
        """Evaluate fitness (makespan) for each solution in the population."""
        for solution in self.population:
            if solution.last_step_makespan < self.best_makespan:
                self.best_makespan = solution.last_step_makespan
                self.best_solution = deepcopy(solution)

    def selection(self):
        """Select individuals for crossover using tournament selection."""
        selected = []
        for _ in range(self.population_size):
            ind1, ind2 = random.sample(self.population, 2)
            selected.append(min(ind1, ind2, key=lambda sol: sol.last_step_makespan))
        return selected

    def crossover(self, parents):
        """Perform crossover with the given parents."""
        offspring = []
        for i in range(0, len(parents), 2):
            parent1 = parents[i]
            if i + 1 < len(parents):
                parent2 = parents[i + 1]
                if random.random() < self.crossover_rate:
                    child1, child2 = self.single_point_crossover(parent1, parent2)
                    offspring.append(child1)
                    offspring.append(child2)
                else:
                    offspring.append(deepcopy(parent1))
                    offspring.append(deepcopy(parent2))
            else:
                offspring.append(deepcopy(parent1))
        return offspring

    @staticmethod
    def single_point_crossover(parent1, parent2):
        """Single-point crossover."""
        child1, child2 = deepcopy(parent1), deepcopy(parent2)
        crossover_point = random.randint(0, 5)
        for i in range(crossover_point, len(child1.actions)):
            child1.actions[i], child2.actions[i] = child2.actions[i], child1.actions[i]
        return child1, child2

    def mutation(self, offspring):
        """Perform mutation on the offspring."""
        for individual in offspring:
            if random.random() < self.mutation_rate:
                self.mutate(individual)
        return offspring

    @staticmethod
    def mutate(solution):
        """Apply mutation by randomly changing an action."""
        mutation_point = random.randint(0, 5)
        solution.step_v1(mutation_point)

    def plot_result(self):
        plt.rcParams["font.sans-serif"] = ["SimHei"]
        plt.rcParams["font.family"] = "sans-serif"
        plt.rcParams['axes.unicode_minus'] = False
        iterations = self.iter_x
        best_record = self.iter_y
        plt.plot(iterations, best_record)
        plt.title('iteration')
        plt.show()
