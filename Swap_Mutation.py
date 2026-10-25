def Swap_Mutation(individual, mutation_rate, rng):
    # Keeps the parent intact because it may be selected again.
    mutated = individual.copy()

    if len(mutated) >= 2 and rng.random() < mutation_rate:
        # Sampling without replacement guarantees two different positions, so the swap is not a no-op.
        first, second = rng.choice(len(mutated), size=2, replace=False)
        mutated[[first, second]] = mutated[[second, first]]

    return mutated