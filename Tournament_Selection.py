import numpy as np

def Tournament_Selection(fitnesses, size_of_tournament=5, p=0.8, rng=None):
    if rng is None:
        rng = np.random.default_rng()
    if len(fitnesses) == 0 or size_of_tournament < 1:
        raise ValueError("The population and tournament must be nonempty.")
    if not 0 <= p <= 1:
        raise ValueError("Tournament probability must be between 0 and 1.")

    # The tournament cannot have more distinct participants than the population.
    size = min(size_of_tournament, len(fitnesses))
    contenders = rng.choice(len(fitnesses), size=size, replace=False)
    order = np.argsort(np.asarray(fitnesses)[contenders])[::-1]
    contenders = contenders[order]

    # I try contenders from best to worst rather than always choosing the best. This keeps weaker routes in 
    # contention when p is below 1.
    for contender in contenders[:-1]:
        if rng.random() < p:
            return int(contender)
        
    # The last contender is the fallback, including when the tournament has only one participant.
    return int(contenders[-1])