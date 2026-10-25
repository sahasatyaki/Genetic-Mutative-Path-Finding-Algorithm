from pathlib import Path
import time

import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

from Random_Mutation import Random_Mutation
from Rank_Selection import Rank_Selection
from Roulette_Wheel_Selection import Roulette_Wheel_Selection
from Swap_Mutation import Swap_Mutation
from Tournament_Selection import Tournament_Selection
from TwoPoint_CrossOver import TwoPoint_CrossOver
from Uniform_CrossOver import Uniform_CrossOver

def load_map(filename):
    with Image.open(filename) as image:
        # Threshold grayscale values so both binary and grayscale images use the same map convention: True = free space and False = obstacle.
        return np.asarray(image.convert("L")) >= 128

def path_length(path):
    differences = np.diff(path, axis=0)
    return float(np.sum(np.linalg.norm(differences, axis=1)))

def segment_obstacle_length(point1, point2, map_data):
    # Floating-point coordinates let me check partial segments as well as lines between pixel centres.
    point1 = np.asarray(point1, dtype=float)
    point2 = np.asarray(point2, dtype=float)
    difference = point2 - point1
    distance = float(np.linalg.norm(difference))

    # A repeated point has no length, but a blocked point must still be reported as a collision.
    if distance == 0:
        row, column = np.rint(point1).astype(int)
        if not map_data[row, column]:
            return 1e-6
        return 0.0

    # Represent the segment by fractions from 0 to 1. Splitting at every pixel boundary lets me
    # measure obstacle distance without skipping thin obstacles.
    crossings = [np.array([0.0, 1.0])]

    for axis in range(2):
        if difference[axis] == 0:
            continue

        lower = min(point1[axis], point2[axis])
        upper = max(point1[axis], point2[axis])
        # Pixel centres have integer coordinates, so their edges are at half integers.
        edges = np.arange(np.floor(lower) + 0.5, upper, 1.0)
        parameters = (edges - point1[axis]) / difference[axis]
        parameters = parameters[(parameters > 0) & (parameters < 1)]
        crossings.append(parameters)

    # A corner crossing can appear on both axes, remove duplicates before measuring interval lengths.
    parameters = np.unique(np.concatenate(crossings))
    # Each interval stays inside one pixel, so its midpoint identifies whether that entire interval is blocked.
    middle = (parameters[:-1] + parameters[1:]) / 2.0
    points = point1 + middle[:, None] * difference
    pixels = np.floor(points + 0.5).astype(int)
    blocked = ~map_data[pixels[:, 0], pixels[:, 1]]
    # The intervals are fractions of the segment, so scale their sum by its Euclidean length.
    obstacle_length = float(np.sum(np.diff(parameters)[blocked]) * distance)

    # A line may touch an obstacle corner without travelling a positive distance inside it. I count
    # these contacts as collisions too.
    if obstacle_length == 0:
        points = point1 + parameters[:, None] * difference

        # Tiny offsets inspect cells on all sides of a boundary crossing, closing diagonal gaps between touching obstacles.
        for row_shift, column_shift in [(-1e-9, -1e-9), (-1e-9, 1e-9), (1e-9, -1e-9), (1e-9, 1e-9)]:
            pixels = np.floor(points + 0.5 + [row_shift, column_shift]).astype(int)
            pixels = np.clip(pixels, [0, 0], np.array(map_data.shape) - 1)

            # Use a tiny positive value to flag contact even when the geometric overlap has zero length.
            if np.any(~map_data[pixels[:, 0], pixels[:, 1]]):
                return 1e-6
            
    return obstacle_length

def evaluate_individual(individual, map_data, start_point, end_point):
    path = np.vstack((start_point, individual, end_point))

    if np.any(path < 0) or np.any(path > np.array(map_data.shape) - 1):
        raise ValueError("All path coordinates must lie inside the map.")
    
    distance = path_length(path)
    obstacle_length = 0.0

    for index in range(len(path) - 1):
        obstacle_length += segment_obstacle_length(path[index], path[index + 1], map_data)

    # I penalise distance inside obstacles rather than using one flat penalty for every colliding
    # route. This gives the search a reason to reduce the blocked part of a route.
    penalty = 1000.0 * obstacle_length

    # This extra penalty exceeds the longest possible valid route with the same number of segments, so a colliding route cannot outrank a valid one.
    if obstacle_length > 0:
        maximum_length = (len(path) - 1) * np.linalg.norm(np.array(map_data.shape) - 1)
        penalty += maximum_length + 1.0

    # Selection favours higher fitness, so shorter routes and smaller penalties must give less negative scores.
    return -distance - penalty, distance, obstacle_length

def fitness_evaluation(population, map_data, start_point, end_point):
    scores = []

    for individual in population:
        fitness, distance, obstacle_length = evaluate_individual(individual, map_data, start_point, end_point)
        scores.append(fitness)

    return np.array(scores)

def waypoints_from_anchors(anchors, num_way_points):
    sections = len(anchors) - 1
    pieces = []

    for index in range(sections):
        # Share the waypoint budget between sections, assigning any remainder to the earlier ones.
        count = num_way_points // sections
        if index < num_way_points % sections:
            count += 1
        # The finish is stored separately. Earlier sections include their ending anchor to preserve the
        # turn, while the final section leaves out the finish.
        if index == sections - 1:
            points = np.linspace(anchors[index], anchors[index + 1], count + 2)[1:-1]
        else:
            points = np.linspace(anchors[index], anchors[index + 1], count + 1)[1:]
        pieces.append(points)

    return np.rint(np.vstack(pieces)).astype(int)

# I use visible random bends to seed the population. This favours progress towards the goal, so it
# can struggle with routes that need substantial backtracking.
def guided_individual(map_data, start_point, end_point, num_way_points, rng):
    free_cells = np.argwhere(map_data)
    anchors = [start_point.copy()]

    # Limit the bends to keep initialisation inexpensive and within the waypoint budget.
    for turn in range(min(num_way_points, 5)):
        current = anchors[-1]
        if segment_obstacle_length(current, end_point, map_data) == 0:
            break

        candidates = free_cells[rng.integers(len(free_cells), size=60)]
        remaining = np.linalg.norm(candidates - end_point, axis=1)
        travelled = np.linalg.norm(candidates - current, axis=1)
        estimates = travelled + remaining
        # Random variation helps separate seeds explore different bends instead of repeatedly taking the same route.
        estimates += rng.uniform(0, max(map_data.shape) * 0.15, len(candidates))
        found = False
        current_remaining = np.linalg.norm(current - end_point)

        for index in np.argsort(estimates):
            candidate = candidates[index]
            # Avoid tiny steps and require progress towards the finish for this greedy initialisation.
            if travelled[index] < 2 or remaining[index] >= current_remaining:
                continue
            if segment_obstacle_length(current, candidate, map_data) == 0:
                anchors.append(candidate.copy())
                found = True
                break

        if not found:
            break

    # The remaining section may still collide. Keep it for normal fitness evaluation rather than treating the heuristic as a 
    # guarantee of success.
    anchors.append(end_point.copy())
    return waypoints_from_anchors(anchors, num_way_points)

def random_population_generation(population_size, num_way_points, map_data, start_point, end_point, rng, guided_count=0):
    free_cells = np.argwhere(map_data)
    population = []
    direction = end_point - start_point
    fractions = np.linspace(0, 1, num_way_points + 2)[1:-1]
    straight_points = start_point + fractions[:, None] * direction

    for index in range(population_size):
        # I mix global random routes, smooth bends and perturbed straight lines so the population explores different ways around obstacles.
        if index % 4 == 0:
            chosen = rng.integers(len(free_cells), size=num_way_points)
            individual = free_cells[chosen].copy()
            # Sorting by progress along the start-to-finish direction reduces needless zigzags in random seeds.
            progress = (individual - start_point) @ direction
            individual = individual[np.argsort(progress)]

        elif index % 4 in (1, 2) and num_way_points >= 3:
            anchor_count = index % 4
            chosen = rng.integers(len(free_cells), size=anchor_count)
            anchors = free_cells[chosen]
            progress = (anchors - start_point) @ direction
            anchors = anchors[np.argsort(progress)]
            anchors = np.vstack((start_point, anchors, end_point))
            individual = waypoints_from_anchors(anchors, num_way_points)

        else:
            scale = max(map_data.shape) * rng.uniform(0.02, 0.35)
            individual = np.rint(straight_points + rng.normal(0, scale, (num_way_points, 2)))
            individual = np.clip(individual, [0, 0], np.array(map_data.shape) - 1).astype(int)

            for waypoint in range(num_way_points):
                row, column = individual[waypoint]

                # Move blocked waypoints to nearby free cells without discarding the intended shape of the seed.
                if not map_data[row, column]:
                    distances = np.sum((free_cells - individual[waypoint]) ** 2, axis=1)
                    individual[waypoint] = free_cells[np.argmin(distances)]

        population.append(individual)

    # Boundary routes help on maps with clear edges. They still go through normal evaluation because arbitrary input maps may block those edges.
    if num_way_points >= 1:

        for index, corner in enumerate([[end_point[0], start_point[1]], [start_point[0], end_point[1]]]):
            if index >= population_size:
                break

            corner = np.array(corner)
            first_count = (num_way_points + 1) // 2
            first_half = np.linspace(start_point, corner, first_count + 1)[1:]
            second_half = np.linspace(corner, end_point, num_way_points - first_count + 2)[1:-1]
            population[index] = np.rint(np.vstack((first_half, second_half))).astype(int)

    # Keep the two boundary seeds and replace only the following positions with guided routes.
    for index in range(2, min(population_size, guided_count + 2)):
        population[index] = guided_individual(map_data, start_point, end_point, num_way_points, rng)

    return population

def parent_selection(fitness, method, rng, tournament_size=5, tournament_probability=0.8):
    if method == 0:
        return Roulette_Wheel_Selection(fitness, rng)
    if method == 1:
        return Tournament_Selection(fitness, tournament_size, tournament_probability, rng)
    if method == 2:
        return Rank_Selection(fitness, rng)
    
    raise ValueError("Selection must be among: 0, 1, or 2.")

def crossover_method(parent1, parent2, method, rng):
    if method == 0:
        return Uniform_CrossOver(parent1, parent2, len(parent1), rng)
    if method == 1:
        return TwoPoint_CrossOver(parent1, parent2, len(parent1), rng)
    
    raise ValueError("Crossover must be 0 or 1.")

def mutation_method(individual, mutation_rate, map_data, method, rng, step_size):
    if method == 0:
        return Random_Mutation(individual, mutation_rate, map_data, map_data.shape, 2, rng, step_size)
    if method == 1:
        return Swap_Mutation(individual, mutation_rate, rng)
    
    raise ValueError("Mutation must be 0 or 1.")

def refine_individual(individual, map_data, start_point, end_point, rng, step_size):
    # A rejected trial must not change the route being refined.
    best = individual.copy()
    best_fitness, distance, obstacle_length = evaluate_individual(best, map_data, start_point, end_point)

    # Random visit order avoids always giving the same waypoint the first chance to improve.
    for index in rng.permutation(len(best)):
        path = np.vstack((start_point, best, end_point))
        # The midpoint offers a possible shortcut, the random move offers an alternative when obstacles block that shortcut.
        midpoint = np.rint((path[index] + path[index + 2]) / 2).astype(int)
        random_move = np.rint(best[index] + rng.normal(0, step_size, 2)).astype(int)

        for point in [midpoint, random_move]:
            point = np.clip(point, [0, 0], np.array(map_data.shape) - 1)

            if not map_data[point[0], point[1]]:
                continue

            candidate = best.copy()
            candidate[index] = point
            fitness, distance, obstacle_length = evaluate_individual(candidate, map_data, start_point, end_point)

            # Only accept an improvement above this tolerance, rather than reacting to floating-point noise.
            if fitness > best_fitness + 1e-9:
                best = candidate
                best_fitness = fitness

    return best

def run_genetic_algorithm(map_data, selection=1, crossover=1, mutation=0,
                          population_size=80, num_way_points=10, max_generations=300, crossover_rate=0.8,
                          mutation_rate=0.3, no_change_limit=70, seed=None, tournament_size=5, tournament_probability=0.8,
                          start_point=None, end_point=None, verbose=True, show_live=False, live_interval=5, live_delay=0.02):
    
    map_data = np.asarray(map_data, dtype=bool)

    if map_data.ndim != 2 or min(map_data.shape) < 2:
        raise ValueError("The map must be a 2D array with both dimensions >= 2.")
    if population_size < 4 or num_way_points < 1 or max_generations < 1:
        raise ValueError("Use population_size >= 4, num_way_points >= 1, max_generations >= 1.")
    if selection not in (0, 1, 2) or crossover not in (0, 1) or mutation not in (0, 1):
        raise ValueError("Invalid selection, crossover, or mutation choice.")
    if not 0 <= crossover_rate <= 1 or not 0 <= mutation_rate <= 1:
        raise ValueError("Crossover and mutation rates must lie between 0 and 1.")
    if no_change_limit < 1 or tournament_size < 1 or not 0 <= tournament_probability <= 1:
        raise ValueError("Invalid stopping or tournament parameters.")
    if live_interval < 1 or live_delay <= 0:
        raise ValueError("Use live_interval >= 1 and live_delay > 0.")
    # I use zero-based row/column coordinates, with the top-left pixel as the default start.
    if start_point is None:
        start_point = [0, 0]
    if end_point is None:
        end_point = np.array(map_data.shape) - 1

    start_point = np.asarray(start_point)
    end_point = np.asarray(end_point)

    for point in [start_point, end_point]:
        if point.shape != (2,) or np.any(point != np.rint(point)):
            raise ValueError("Endpoints must be integer (row, column) pairs.")
        if np.any(point < 0) or np.any(point >= map_data.shape):
            raise ValueError("Endpoints must lie inside the map.")
        if not map_data[int(point[0]), int(point[1])]:
            raise ValueError("The start and finish must be free pixels.")
        
    start_point = start_point.astype(int)
    end_point = end_point.astype(int)
    # One generator controls all random choices, making runs reproducible when a seed is supplied.
    rng = np.random.default_rng(seed)
    # Timing includes initialisation and live drawing, so enabling animation adds to the reported time.
    begin = time.perf_counter()
    population = random_population_generation(population_size, num_way_points, map_data, start_point, end_point, rng, guided_count=population_size // 4)

    history = []
    best_fitness = -np.inf
    best_individual = None
    best_generation = 0
    stagnant = 0
    # Cache repeated routes because elites and copied parents can appear in several generations.
    cache = {}
    live_plot = None

    if show_live:
        live_plot = create_live_plot(map_data, start_point, end_point)

    for generation in range(1, max_generations + 1):
        if live_plot is not None:
            # Preview a candidate before sorting. After the first generation, the first two positions are
            # preserved elites, showing only those would hide failed trials.
            trial_path = np.vstack((start_point, population[2], end_point))

        scores = []

        for individual in population:
            # The waypoint bytes identify a route within this run, the map and endpoints stay fixed.
            key = individual.tobytes()

            if key not in cache:
                cache[key] = evaluate_individual(individual, map_data, start_point, end_point)[0]
            scores.append(cache[key])

        order = np.argsort(scores)[::-1]
        population = [population[int(index)] for index in order]
        scores = np.asarray(scores)[order]

        progress = (generation - 1) / max(1, max_generations - 1)
        # I reduce movement size over time to shift from exploration towards finer adjustments.
        step_size = max(1.0, max(map_data.shape) * (0.08 * (1 - progress) + 0.004))

        # Occasional larger moves give the search another chance to leave a local minimum.
        if generation % 10 == 0:
            step_size = max(map_data.shape) * 0.12

        # Local refinement is shared by both mutation choices. It is particularly useful with swap
        # mutation, which cannot introduce new coordinates on its own.
        refined = refine_individual(population[0], map_data, start_point, end_point, rng, step_size)
        refined_score = evaluate_individual(refined, map_data, start_point, end_point)[0]
        population[0] = refined
        scores[0] = refined_score

        if scores[0] > best_fitness + 1e-9:
            best_fitness = float(scores[0])
            # Keep an independent best-so-far copy so later population changes cannot overwrite it.
            best_individual = population[0].copy()
            best_generation = generation
            stagnant = 0
        else:
            stagnant += 1
        history.append(best_fitness)

        if verbose and (generation == 1 or generation % 25 == 0):
            print(f"Generation {generation}: best fitness = {best_fitness:.3f}")

        best_path = np.vstack((start_point, best_individual, end_point))

        if live_plot is not None:
            # Show early trials, then sample at intervals to avoid spending most of the run on animation.
            animate = generation <= 5 or generation % live_interval == 0
            update_live_plot(live_plot, trial_path, history, map_data, generation, time.perf_counter() - begin, animate=animate, frame_delay=live_delay)

        if generation == max_generations:
            break

        # Stagnation only ends the search early if a valid route exists. A stalled colliding route is not a successful result.
        if stagnant >= no_change_limit:
            best_obstacle_length = evaluate_individual(
                best_individual, map_data, start_point, end_point)[2]
            
            if best_obstacle_length == 0:
                break

        # I preserve the two strongest routes so selection and variation cannot lose the best solution.
        next_population = [population[0].copy(), population[1].copy()]
        # Reserve roughly 10% of the population for fresh routes to reduce premature convergence.
        immigrant_count = max(1, population_size // 10)

        while len(next_population) < population_size - immigrant_count:
            first = parent_selection(scores, selection, rng, tournament_size, tournament_probability)
            second = parent_selection(scores, selection, rng, tournament_size, tournament_probability)

            if rng.random() < crossover_rate:
                children = crossover_method(population[first], population[second], crossover, rng)
            else:
                children = (population[first].copy(), population[second].copy())
            # Increase mutation during stagnation, but cap the rate so it does not overwhelm inheritance.
            effective_rate = min(0.8, mutation_rate + 0.3 * min(stagnant / no_change_limit, 1))

            for child in children:
                child = mutation_method(child, effective_rate, map_data, mutation, rng, step_size)
                next_population.append(child)

                # Stop after the final available slot, even if only one child of a pair fits.
                if len(next_population) >= population_size - immigrant_count:
                    break

        immigrants = random_population_generation(immigrant_count + 2, num_way_points, map_data, start_point, end_point, rng, guided_count=1)
        # Skip the two boundary seeds generated at the start of the immigrant list, keep the fresh and
        # guided routes.
        next_population.extend(immigrants[2:])
        population = next_population

        # Bound cache growth during longer runs. Clearing it affects speed, not fitness values.
        if len(cache) > 20000:
            cache.clear()

    fitness, distance, obstacle_length = evaluate_individual(best_individual, map_data, start_point, end_point)

    if live_plot is not None:
        # End the animation with the actual best route, since the last displayed candidate may be worse.
        update_live_plot(live_plot, best_path, history, map_data, generation, time.perf_counter() - begin, frame_delay=live_delay, final_attempt=True)

    elapsed = time.perf_counter() - begin
    result = {"path": best_path, "fitness": fitness, "distance": distance,
              "obstacle_length": obstacle_length, "success": obstacle_length == 0,
              "generations": generation, "best_generation": best_generation,
              "elapsed_seconds": elapsed, "fitness_history": history, "seed": seed}
    
    if live_plot is not None:
        result["live_figure"] = live_plot[0]
    return result

def create_live_plot(map_data, start_point, end_point):

    figure, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].imshow(map_data, cmap="gray", vmin=0, vmax=1)
    # Start with an empty trail so the route is revealed as the robot moves.
    route_line, = axes[0].plot([], [], "-", color="tab:orange", label="Trial trail")

    axes[0].plot(start_point[1], start_point[0], "go", label="Start")
    axes[0].plot(end_point[1], end_point[0], "ro", label="Finish")
    robot, = axes[0].plot([start_point[1]], [start_point[0]], "o", color="cyan", markeredgecolor="black", markersize=8, label="Robot")

    # Keep earlier collision locations visible while testing later candidates.
    collision_marks, = axes[0].plot([], [], "rx", markersize=7, label="Collisions")
    axes[0].set_title("Testing candidate routes")
    # axes[0].legend()

    fitness_line, = axes[1].plot([], [])
    axes[1].set_xlabel("Generation")
    axes[1].set_ylabel("Best fitness (Higher Is Better)")

    figure.tight_layout()
    # A blocking window would prevent the algorithm from running until the window was closed.
    plt.show(block=False)

    return figure, axes, route_line, fitness_line, robot, collision_marks

# This replays a candidate, it does not train the robot on contact. The GA evaluates the full route, while the display stops at its first collision.
def robot_trail(path, map_data):
    trail = [np.asarray(path[0], dtype=float)]

    for index in range(len(path) - 1):
        start = np.asarray(path[index], dtype=float)
        finish = np.asarray(path[index + 1], dtype=float)
        # Check the full segment so a thin obstacle cannot be skipped between animation frames.
        collided = segment_obstacle_length(start, finish, map_data) > 0

        if collided:
            safe_fraction = 0.0
            hit_fraction = 1.0

            # Binary search narrows the first contact between a safe prefix and a colliding prefix.
            for attempt in range(25):
                fraction = (safe_fraction + hit_fraction) / 2
                point = start + fraction * (finish - start)

                if segment_obstacle_length(start, point, map_data) > 0:
                    hit_fraction = fraction
                else:
                    safe_fraction = fraction

            finish = start + hit_fraction * (finish - start)

        samples = max(2, int(np.ceil(np.linalg.norm(finish - start) / 2)) + 1)
        # Leave out the first sample because the previous segment already added that point.
        trail.extend(np.linspace(start, finish, samples)[1:])

        if collided:
            return np.asarray(trail), True
        
    return np.asarray(trail), False

def update_live_plot(live_plot, path, history, map_data, generation, elapsed, animate=True, frame_delay=0.02, final_attempt=False):

    figure, axes, route_line, fitness_line, robot, collision_marks = live_plot

    if not plt.fignum_exists(figure.number):
        return
    
    # The graph tracks the best route found, the animated candidate may have lower fitness.
    fitness_line.set_data(range(1, len(history) + 1), history)
    axes[1].relim()
    axes[1].autoscale_view()
    axes[1].set_title(f"Best fitness | Generation: {generation} | Time: {elapsed:.2f} s")

    if animate:
        trail, collided = robot_trail(path, map_data)
        if final_attempt:
            label = "Best route replay"
        else:
            label = "Candidate trial"
        route_line.set_color("tab:orange")

        axes[0].set_title(f"{label} | Generation: {generation}\nRobot Moving")
        # At most 20 frames keep long routes from making each trial too slow to watch.
        frames = np.unique(np.linspace(0, len(trail) - 1, min(20, len(trail)), dtype=int))

        for index in frames:
            if not plt.fignum_exists(figure.number):
                return
            
            # Reveal only the travelled portion, rather than drawing a complete connection to the finish.
            route_line.set_data(trail[:index + 1, 1], trail[:index + 1, 0])
            robot.set_data([trail[index, 1]], [trail[index, 0]])
            figure.canvas.draw_idle()
            # Process window events as well as delaying the next frame, otherwise the display can freeze.
            plt.pause(frame_delay)

        if collided:
            route_line.set_color("tab:red")
            previous_x, previous_y = collision_marks.get_data()
            collision_marks.set_data(list(previous_x) + [trail[-1, 1]], list(previous_y) + [trail[-1, 0]])
            status = "Collision Encountered. Stopping the trial and penalising the route."
        else:
            route_line.set_color("tab:blue")
            status = "Robot Reached the Target."

        axes[0].set_title(f"{label} | Generation: {generation}\n{status}")
        figure.canvas.draw_idle()
        # Leave the outcome visible briefly before moving on to the next trial.
        plt.pause(max(0.15, frame_delay))

    figure.canvas.draw_idle()
    plt.pause(0.001)

def plot_result(map_data, result, output_path=None, show=True):

    figure = result.get("live_figure")

    # Reuse the live window when it is still open, instead of opening a second result window.
    if figure is not None and plt.fignum_exists(figure.number):
        axes = figure.axes

        for axis in axes:
            axis.clear()

    else:
        figure, axes = plt.subplots(1, 2, figsize=(12, 5))
        
    axes[0].imshow(map_data, cmap="gray", vmin=0, vmax=1)
    path = result["path"]

    if result["success"]:
        color = "tab:blue"
    else:
        color = "tab:red"

    # Image coordinates use column for x and row for y, reversing them would transpose the route.
    axes[0].plot(path[:, 1], path[:, 0], "o-", color=color, markersize=3)
    axes[0].plot(path[0, 1], path[0, 0], "go", label="Start")
    axes[0].plot(path[-1, 1], path[-1, 0], "ro", label="Finish")

    if result["success"]:
        status = "Collision-free route"
    else:
        status = "No collision-free route found"

    axes[0].set_title(f"{status}\nDistance: {result['distance']:.2f} pixels")
    # axes[0].legend()
    axes[1].plot(range(1, len(result["fitness_history"]) + 1), result["fitness_history"])

    axes[1].set_xlabel("Generation")
    axes[1].set_ylabel("Best fitness (higher is better)")
    axes[1].set_title(f"Time: {result['elapsed_seconds']:.2f} s | Best generation: {result['best_generation']}")

    figure.tight_layout()

    if output_path is not None:
        figure.savefig(output_path, dpi=150)
    if show:
        plt.show()
    else:
        plt.close(figure)

def ask_choice(prompt, allowed):
    while True:
        try:
            value = int(input(prompt))

            if value in allowed:
                return value
        except ValueError:
            pass

        print(f"Please enter one of: {allowed}")

def main():
    # Resolve files relative to the script so running it from another directory still finds the map.
    folder = Path(__file__).resolve().parent
    map_data = load_map(folder / "random_map.bmp")
    selection = ask_choice("Selection (0: roulette, 1: tournament, 2: rank): ", (0, 1, 2))
    crossover = ask_choice("Crossover (0: uniform, 1: two-point): ", (0, 1))
    mutation = ask_choice("Mutation (0: random, 1: swap): ", (0, 1))
    display = ask_choice("Display (0: final result only, 1: live robot trials): ", (0, 1))

    # I leave the seed unset for different runs. Set an integer seed here when comparing settings reproducibly.
    result = run_genetic_algorithm(map_data, selection, crossover, mutation, population_size=80, num_way_points=10, max_generations=300, seed=None, show_live=(display == 1))

    print(f"Collision-free: {result['success']}")
    print(f"Distance: {result['distance']:.3f} pixels")
    print(f"Distance inside obstacles: {result['obstacle_length']:.6f} pixels")

    print(f"Time: {result['elapsed_seconds']:.3f} seconds")
    print(f"Generations: {result['generations']}; best generation: {result['best_generation']}")

    if not result["success"]:
        print("No valid route found. Try more generations, more waypoints, or probably another seed.")

    np.savetxt(folder / "best_path.csv", result["path"], fmt="%d", delimiter=",", header="row,column (zero-based)", comments="")
    plot_result(map_data, result, folder / "best_path.png")

if __name__ == "__main__":
    main()