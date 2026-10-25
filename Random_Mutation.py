import numpy as np

def Random_Mutation(individual, mutation_rate, map_data, map_size, minimum_distance, rng, step_size=None):
    # Work on a copy so a mutation cannot alter a parent that is still in the population.
    mutated = individual.copy()

    if len(mutated) == 0 or rng.random() >= mutation_rate:
        return mutated
    if tuple(map_size) != map_data.shape:
        raise ValueError("map_size MUST match the map dimensions.")
    if step_size is None:
        step_size = max(map_size) * 0.08

    index = int(rng.integers(len(mutated)))

    # I retry rejected positions so one blocked cell does not waste the mutation. The limit prevents an endless search on crowded maps.
    for attempt in range(30):
        # Nearby moves help refine a route, occasional moves across the map keep some exploration.
        if rng.random() < 0.85:
            candidate = np.rint(mutated[index] + rng.normal(0, step_size, 2))
            candidate = candidate.astype(int)
        else:
            candidate = np.array([rng.integers(map_size[0]), rng.integers(map_size[1])])

        candidate = np.clip(candidate, [0, 0], np.array(map_size) - 1)

        if not map_data[candidate[0], candidate[1]]:
            continue

        # Exclude the waypoint being replaced, otherwise it would count against its own spacing check.
        other_points = np.delete(mutated, index, axis=0)
        distances = np.linalg.norm(other_points - candidate, axis=1)

        if np.all(distances >= minimum_distance):
            mutated[index] = candidate
            break

    return mutated