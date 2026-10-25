def TwoPoint_CrossOver(parent1, parent2, num_way_points, rng):
    if parent1.shape != parent2.shape or parent1.shape != (num_way_points, 2):
        raise ValueError("Both parents must have shape (num_way_points, 2).")
    
    # Copy both parents so crossover cannot modify routes that are still needed elsewhere.
    child1 = parent1.copy()
    child2 = parent2.copy()

    if num_way_points == 0:
        return child1, child2
    
    # I choose distinct boundaries, including either end of the route, so the exchanged section cannot be empty.
    cuts = rng.choice(num_way_points + 1, size=2, replace=False)
    first = int(min(cuts))
    last = int(max(cuts))

    # Exchange complete waypoints to keep each row/column pair together.
    child1[first:last] = parent2[first:last]
    child2[first:last] = parent1[first:last]

    return child1, child2