"""Choosing a night's toys (sim-slice [Afy]): a fixed mix of [QyZ]'s tiers, in proportion to the house's floor, binning
being what the nights show."""

SQ_FT_PER_TOY_M2 = 200 * 0.0929   # one toy per 200 sq ft of floor
MIX = {"too_big": 0.125, "hard": 0.25}   # the rest easy
MIN_TOYS = 4


def toys_for_floor(area_m2):
    """how many toys a night for a house with `area_m2` of floor inside its outer walls"""
    return max(MIN_TOYS, round(area_m2 / SQ_FT_PER_TOY_M2))


def night_mix(n):
    """a night of `n` toys by tier: 12.5% too big (at least one), 25% hard, the rest easy"""
    big = max(1, round(MIX["too_big"] * n)); hard = round(MIX["hard"] * n)
    return {"too_big": big, "hard": hard, "easy": n - big - hard}


def pick_night(rng, names, tier, n):
    """`n` different toys from `names` in night_mix(n) by `tier(name)`, in a random order (the order they are placed)"""
    out = []
    for t, k in night_mix(n).items():
        pool = [x for x in names if tier(x) == t]
        if len(pool) < k: raise ValueError(f"only {len(pool)} {t} toys for {k}")
        out += [pool[i] for i in rng.choice(len(pool), k, replace=False)]
    return [out[i] for i in rng.permutation(len(out))]
