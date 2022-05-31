import sys
import time
import pickle
import numpy as np
import math
import random
import cv2


class MarkovDecisionProcess:
    """A Markov Decision Process"""
    def __init__(self):
        self.states = set()
        self.transitions = {}
        self.costs = {}
        self.actions = set()
        self.rewards = {}

    def value_iteration(self, x_goal, y_goal, goal_radiance):
        V = {s: 100 for s in self.states}   # Denote the cost (distance)
        V2 = {s: 0 for s in self.states}    # Denotes the probability of reaching a goal state

        goal_region_states = self._sample_states_in_goal_region(x_goal, y_goal, goal_radiance)
        for xg, yg in goal_region_states:
            V[(xg, yg)] = 0
            V2[(xg, yg)] = 1
        eps = 1
        eps2 = 0.01
        max_residual = 100
        while True:
            V1 = V.copy()
            V21 = V2.copy()
            residual = 0
            residual2 = 0
            for s in self.states.difference(goal_region_states):
                transition_prob_dict = self.transitions[s]
                actions = transition_prob_dict.keys()
                bellman_update = min([sum([p * (self.C(s, a)[s1] + V1[s1]) for (s1, p) in self.T(s, a).items()])
                                      for a in actions])
                bellman_update2 = max([sum([p * (self.R(s, a)[s1] + V21[s1]) for (s1, p) in self.T(s, a).items()])
                                       for a in actions])

                V[s] = bellman_update
                V2[s] = bellman_update2
                residual = max(residual, abs(V1[s] - V[s]))
                residual2 = max(residual2, abs(V21[s] - V2[s]))

            if residual < eps and residual2 < eps2:
                return V

    def T(self, s, a):
        return self.transitions[s][a]

    def C(self, s, a):
        return self.costs[s][a]

    def R(self, s, a):
        return self.rewards[s][a]

    def _sample_states_in_goal_region(self, x_goal, y_goal, goal_radiance):
        rst = set()
        for xs, ys in self.states:
            dist = distance(xs, ys, x_goal, y_goal)
            if dist <= goal_radiance:
                rst.add((xs, ys))
        return rst


# return dist and angle b/w new point and nearest node
def dist_and_angle(x1,y1,x2,y2):
    dist = math.sqrt( ((x1-x2)**2)+((y1-y2)**2) )
    angle = math.atan2(y2-y1, x2-x1)
    return dist, angle


def distance(x1, y1, x2, y2):
    return math.sqrt( ((x1-x2)**2)+((y1-y2)**2) )


def calculate_mean(s):
    x_total = 0
    y_total = 0
    for x, y in s:
        x_total += x
        y_total += y
    x_mean = x_total / len(s)
    y_mean = y_total / len(s)
    return int(x_mean), int(y_mean)


def force_generation():
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0),
                  (1, 1), (1, -1), (-1, 1), (-1, -1)]
    x_magnitude = random.randint(1, 10)
    y_magnitude = random.randint(1, 10)
    magnitudes = (x_magnitude, y_magnitude)

    direct_idx = random.randint(0, 7)
    direction_sampled = directions[direct_idx]
    return direction_sampled, magnitudes


def nearest_mean(means, x, y):
    temp_d_min = -1
    closest_idx = -1
    for idx in range(len(means)):
        xm, ym = means[idx]
        d = distance(x, y, xm, ym)
        if temp_d_min == -1 or d < temp_d_min:
            temp_d_min = d
            closest_idx = idx
    return closest_idx


class RRMDP:
    def __init__(self, img, img2, step_size):
        self.mdp = MarkovDecisionProcess()
        self.img = img
        self.img2 = img2
        self.build_iters = 300        # value N in the paper
        self.num_particles_sampled = 20          # value k in the paper
        self.step_size = step_size
        self.neighbour_radiance = 20    # Used for the near function
        self.num_clusters = 4
        self.clustering_iters = 15
        self.goal_region_radius = 15

    def collision(self, x1, y1, x2, y2):
        color = []
        try:
            x = list(np.arange(x1, x2, (x2 - x1) / 100))

            y = list(((y2 - y1) / (x2 - x1)) * (x - x1) + y1)
            for i in range(len(x)):
                # print(int(x[i]),int(y[i]))
                color.append(self.img[int(y[i]), int(x[i])])
            if 0 in color:
                return True  # collision
            else:
                return False  # no-collision
        except ValueError:
            print("Value error thrown...")
            print("When it is thrown, the value of x1: ", x1)
            print("When it is thrown, the value of x2: ", x2)
            print("When it is thrown: the value of (x2-x1)/100: ", (x2 - x1) / 100)

    def nearest_state(self, x, y):
        temp_dist = []
        states = self.mdp.states
        for x1, y1 in states:
            dist, _ = dist_and_angle(x, y, x1, y1)
            temp_dist.append(dist)
        return temp_dist.index(min(temp_dist))

    def rnd_point(self):
        h, l = self.img.shape
        new_y = random.randint(0, h)
        new_x = random.randint(0, l)
        return new_x, new_y

    def steer_towards(self, nearest_x, nearest_y, nx, ny, step_size):
        # TODO: need to change the generation of the force
        direction_sampled, magnitudes = force_generation()
        x_dir, y_dir = direction_sampled
        x_magnitude, y_magnitude = magnitudes

        dis, theta = dist_and_angle(nearest_x, nearest_y, nx, ny)
        if dis <= step_size:
            tx = nx
            ty = ny
        else:
            tx = nearest_x + step_size * math.cos(theta)
            ty = nearest_y + step_size * math.sin(theta)
        print("tx, ty before applying the force: ", tx, ty)
        tx += x_dir * x_magnitude
        ty += y_dir * y_magnitude
        print("x_dir, y_dir: ", x_dir, y_dir)
        print("x_magnitude, y_magnitude: ", x_magnitude, y_magnitude)
        print("tx, ty after applying the force: ", tx, ty)
        return tx, ty

    def sample_neighbours(self, x_n, y_n):
        """
        Returns all the neighbours of (x, y) within the radiance
        self.neighbour_radiance.
        """
        rst = []
        for x, y in self.mdp.states:
            dist, _ = dist_and_angle(x, y, x_n, y_n)
            if dist < self.neighbour_radiance:
                rst.append((x, y))
        return rst

    def build_mdp(self, x_init, y_init, x_goal, y_goal):
        self.mdp.states.add((-1, -1))        # Dead state
        self.mdp.states.add((x_init, y_init))
        for _ in range(self.build_iters):
            x_rand, y_rand = self.rnd_point()
            self.extend_mdp(x_rand, y_rand, self.step_size)

    def extend_mdp(self, x_rand, y_rand, x_goal, y_goal, step_size):
        # TODO: the main extension algorithm starts here
        x_nearest, y_nearest = self.nearest_state(x_rand, y_rand)
        state_particle_sets = dict()
        for x, y in self.mdp.states:
            state_particle_sets[(x, y)] = set()
        nearest_state_particles = state_particle_sets[(x_nearest, y_nearest)]
        for _ in range(self.num_particles_sampled):
            tx, ty = self.steer_towards(x_nearest, y_nearest, x_rand, y_rand, step_size)
            if not self.collision(tx, ty, x_nearest, y_nearest):
                nearest_state_particles.add((tx, ty))
        if len(nearest_state_particles) != 0:
            x_mean, y_mean = calculate_mean(nearest_state_particles)
            mean_neighbours = self.sample_neighbours(x_mean, y_mean)
            try:
                mean_neighbours.remove((x_nearest, y_nearest))
            except ValueError:
                pass
            for xn, yn in mean_neighbours:
                near_particles = state_particle_sets.setdefault((xn, yn), set())
                for _ in range(self.num_particles_sampled):
                    x_new, y_new = self.steer_towards(xn, yn, x_mean, y_mean)
                    if not self.collision(x_nearest, y_nearest, x_new, y_new):
                        near_particles.add((x_new, y_new))

            particle_set_new = set()
            for particles in state_particle_sets.values():
                particle_set_new.union(particles)

            clusters, means = self.k_means(particle_set_new)
            # means_ = set(means)
            states_without_means = self.mdp.states
            self.mdp.states = self.mdp.states.union(set(means))
            for x, y in states_without_means:
                particle_set = state_particle_sets[(x, y)]
                if len(particle_set) > 0:
                    prob_fail = 1
                    action = ((x, y), (x_rand, y_rand))
                    self.mdp.actions.add(action)
                    trans_prob_dict = self.mdp.transitions.setdefault((x, y), dict())
                    trans_action_dict = trans_prob_dict.setdefault(action, dict())
                    cost_dict = self.mdp.costs.setdefault((x, y), dict())
                    cost_action_dict = cost_dict.setdefault(action, dict())

                    # The cost dictionary for the probability
                    reward_dict = self.mdp.rewards.setdefault((x, y), dict())
                    reward_action_dict = reward_dict.setdefault(action, dict())

                    for i in range(len(means)):
                        cluster = clusters[i]
                        x_mean, y_mean = means[i]
                        inter_with_particle = cluster.intersection(particle_set)
                        num_inters = len(inter_with_particle)
                        prob = num_inters / self.num_particles_sampled
                        # trans_prob_dict = self.mdp.transitions.setdefault((x, y), dict())
                        trans_action_dict[(x_mean, y_mean)] = prob
                        prob_fail -= prob
                        dist_between = distance(x, y, x_mean, y_mean)
                        cost_action_dict[(x_mean, y_mean)] = dist_between

                        dist_to_goal = distance(x_mean, y_mean, x_goal, y_goal)
                        if dist_to_goal <= self.goal_region_radius:
                            reward_action_dict[(x_mean, y_mean)] = 1
                        else:
                            reward_action_dict[(x_mean, y_mean)] = 0

                    trans_action_dict[(-1, -1)] = prob_fail

    def plan(self, x_init, y_init, x_goal, y_goal):
        self.build_mdp(x_init, y_init)
        self.mdp.value_iteration(x_goal, y_goal, self.goal_region_radius)

    def k_means(self, particle_set):
        means = [self.rnd_point() for _ in range(self.num_clusters)]
        # clusters = [set() for _ in range(self.num_clusters)]
        clusters = []
        for _ in range(self.clustering_iters):
            clusters = [set() for _ in range(self.num_clusters)]
            for xp, yp in particle_set:
                nearest_idx = nearest_mean(means, xp, yp)
                clusters[nearest_idx].add((xp, yp))
            # Update the means of each cluster
            for i in range(len(clusters)):
                cluster = clusters[i]
                new_mean = calculate_mean(cluster)
                means[i] = new_mean
        return clusters, means

