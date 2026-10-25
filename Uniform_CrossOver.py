def Uniform_CrossOver(parent1, parent2, num_way_points, rng):
    if parent1.shape != parent2.shape or parent1.shape != (num_way_points, 2):
        raise ValueError("Both parents must have shape (num_way_points, 2).")
    
    # Copies prevent changes to the children from affecting their parents.
    chlid1 = parent1.copy()
    child2 = parent2.copy()

    for index in range(num_way_points):
        # Each position has an independent 50% chance of exchange. I swap whole waypoints so their two coordinates stay together.
        if rng.random() < 0.5:
            chlid1[index] = parent2[index]
            child2[index] = parent1[index]

    return chlid1, child2