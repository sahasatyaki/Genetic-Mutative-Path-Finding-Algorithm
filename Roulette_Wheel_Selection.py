import numpy as np

def Roulette_Wheel_Selection(fitnesses, rng):
    fitnesses = np.asarray(fitnesses, dtype=float)

    if fitnesses.size == 0:
        raise ValueError("The population CANNOT be empty.")
    
    # Fitness can be negative, so I shift the weights above zero. Adding 1 also gives the weakest individual a chance of selection.
    weights = fitnesses - np.min(fitnesses) + 1.0
    probabilities = weights / np.sum(weights)
    
    return int(rng.choice(len(fitnesses), p=probabilities))