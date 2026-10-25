import numpy as np

def Rank_Selection(fitnesses, rng):
    fitnesses = np.asarray(fitnesses, dtype=float)

    if fitnesses.size == 0:
        raise ValueError("The population cannot be empty.")

    # Ranking makes selection depend on fitness order rather than the size of the fitness differences. Keeping the original
    # indices so sorting does not change which parent is returned.
    order = np.argsort(fitnesses)
    weights = np.empty(len(fitnesses), dtype=float)
    first = 0

    # Equal fitness values get the same average rank, avoiding an arbitrary advantage from sort order.
    while first < len(order):
        last = first + 1

        while last < len(order) and fitnesses[order[last]] == fitnesses[order[first]]:
            last += 1

        weights[order[first:last]] = (first + 1 + last) / 2.0
        first = last

    return int(rng.choice(len(fitnesses), p=weights / np.sum(weights)))