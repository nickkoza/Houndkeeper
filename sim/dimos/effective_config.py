"""What DimOS will run with, without running it ([UvE]): the effective GlobalConfig and CostMapper config for a
`dimos run` command line, resolved by DimOS's own config parser from this process's environment, as `dimos run` does.

  python effective_config.py --check-costmap <blueprint>...   exits 1 unless the costmap has our settings: the
  pass-under rule off (>= 2.4 m, the house's ceiling) and the ceiling dropped (0 < ignore_above < 2.4)."""
import os
import sys

CEILING_M = 2.4   # the indoor house's ceiling (make_nights.py)


def effective(blueprints, tokens=(), environ=None):
    """(GlobalConfig, the CostMapper's config instance or None) for `dimos run <blueprints> <tokens>`"""
    from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
    from dimos.core.coordination.blueprints import autoconnect
    from dimos.core.global_config import GlobalConfig, global_config
    from dimos.robot.get_all_blueprints import get_by_name
    environ = os.environ if environ is None else environ
    global_config.update(**BlueprintConfigParser.preparse_global_config(tokens, environ=environ))   # as `dimos run`
    parsed = BlueprintConfigParser(autoconnect(*map(get_by_name, blueprints))).parse(tokens, environ=environ)
    g = GlobalConfig.model_validate(dict(parsed._global_config_values))
    cm = parsed._module_config_values.get("costmapper")
    if cm is None:
        return g, None
    from dimos.mapping.costmapper import Config
    return g, Config(**dict(cm)).config


def costmap_problem(cfg) -> str | None:
    """what is wrong with a CostMapper config for our houses, or None"""
    pu, ia = getattr(cfg, "can_pass_under", None), getattr(cfg, "ignore_above", None)
    if pu is None or pu < CEILING_M or ia is None or not 0 < ia < CEILING_M:
        return f"pass-under {pu} (needs >= {CEILING_M}, off), ignore_above {ia} (needs 0 < it < {CEILING_M})"
    return None


if __name__ == "__main__":
    if len(sys.argv) < 3 or sys.argv[1] != "--check-costmap":
        sys.exit(__doc__)
    _, cfg = effective(sys.argv[2:])
    problem = "no CostMapper in the blueprint" if cfg is None else costmap_problem(cfg)
    print(f"DimOS costmap: {problem or 'pass-under %s, ignore_above %s' % (cfg.can_pass_under, cfg.ignore_above)}")
    sys.exit(1 if problem else 0)
