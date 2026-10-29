"""Item library (spec [QxY]): every simulated thing left on the floor, defined once and used by every scenario.

Each item: how it is built (primitive shape, a scanned household object, cloth or rope), its mass range, and what the
right outcome is ("bin", or "review" with the reason a person should handle it). Masses are estimates of the real
object (the scanned models assume solid density, which is wrong for hollow toys). Scanned objects come from Google
Scanned Objects via mujoco_scanned_objects (fetch with tools/fetch_items.sh); their sizes are the real object's."""
from pathlib import Path
import numpy as np, mujoco
import jaws as jw

GSO = Path(__file__).resolve().parents[2] / "models/items/gso"
# name: (kind, spec, mass range kg (est.), expected outcome, why it is hard or None)
ITEMS = {
    # rigid primitives (the original grasp-bench set)
    "sock_box":   ("box", {"half": [0.09, 0.035, 0.008], "rgba": [0.25, 0.35, 0.75, 1]}, (0.03, 0.06), "bin", None),
    "block":      ("box", {"half": [0.03, 0.03, 0.03], "rgba": [0.9, 0.2, 0.2, 1]}, (0.03, 0.15), "bin", None),
    "brick":      ("box", {"half": [0.05, 0.025, 0.02], "rgba": [0.95, 0.75, 0.1, 1]}, (0.05, 0.3), "bin", None),
    "puck":       ("cylinder", {"half": [0.035, 0.035, 0.015], "rgba": [0.2, 0.7, 0.3, 1]}, (0.03, 0.12), "bin", None),
    # hard for known reasons
    "ball":       ("sphere", {"half": [0.035, 0.035, 0.035], "rgba": [0.95, 0.4, 0.1, 1]}, (0.03, 0.08), "bin", "rolls away when touched"),
    "card":       ("box", {"half": [0.05, 0.075, 0.0015], "rgba": [0.95, 0.95, 0.9, 1]}, (0.005, 0.02), "review", "too flat to pinch from the floor (tissue, paper, card)"),
    "wide_box":   ("box", {"half": [0.07, 0.07, 0.04], "rgba": [0.5, 0.3, 0.7, 1]}, (0.1, 0.3), "review", "wider than the open gripper"),
    "heavy":      ("box", {"half": [0.045, 0.03, 0.03], "rgba": [0.4, 0.4, 0.42, 1]}, (2.0, 2.5), "review", "heavier than the arm's rated 1.5 kg"),
    "pencil":     ("cylinder", {"half": [0.0035, 0.0035, 0.085], "rgba": [0.95, 0.85, 0.2, 1], "lying": True}, (0.005, 0.01), "bin", "thin: hard to see and to grip"),
    # soft
    "cloth_sock": ("cloth", {"size": [0.18, 0.07], "count": [10, 5], "rgba": [0.3, 0.3, 0.6, 1]}, (0.03, 0.05), "bin", "deforms when gripped"),
    "lego_brick": ("box", {"half": [0.016, 0.008, 0.00575], "rgba": [0.8, 0.1, 0.1, 1]}, (0.002, 0.003), "bin", "flat: a 2 x 4 brick, 11.5 mm tall"),   # waymark 0011 [A8K]
    "lego_1x1":   ("box", {"half": [0.0039, 0.0039, 0.0048], "rgba": [0.1, 0.3, 0.8, 1]}, (0.0004, 0.0006), "bin", "tiny: a 1 x 1 brick, 9.6 mm tall"),   # [A8K]
    "lego_plate_1x2": ("box", {"half": [0.0079, 0.0039, 0.0016], "rgba": [0.95, 0.8, 0.1, 1]}, (0.0004, 0.0006), "bin", "flat: a 1 x 2 plate, 3.2 mm tall"),   # [A8K]
    "rope":       ("rope", {"length": 0.4, "count": 16, "radius": 0.004, "rgba": [0.8, 0.2, 0.2, 1]}, (0.01, 0.03), "bin", "long and floppy"),
    # scanned household objects and toys (real sizes)
    "triceratops": ("gso", {"dir": "Great_Dinos_Triceratops_Toy"}, (0.04, 0.08), "bin", None),
    "dino_small":  ("gso", {"dir": "Dino_4"}, (0.05, 0.1), "bin", None),
    "dino_long":   ("gso", {"dir": "Dino_3"}, (0.1, 0.2), "bin", None),
    "figure":      ("gso", {"dir": "Android_Figure_Orange"}, (0.04, 0.08), "bin", None),
    "teether":     ("gso", {"dir": "Kong_Puppy_Teething_Rubber_Small_Pink"}, (0.05, 0.1), "bin", None),
    "toy_taxi":    ("gso", {"dir": "CITY_TAXI_POLICE_CAR"}, (0.04, 0.08), "bin", None),
    "fire_truck":  ("gso", {"dir": "FIRE_TRUCK"}, (0.1, 0.2), "bin", None),
    "baby_car":    ("gso", {"dir": "BABY_CAR"}, (0.1, 0.2), "bin", None),
    "bird_rattle": ("gso", {"dir": "BIRD_RATTLE"}, (0.04, 0.08), "bin", None),
    "bunny_rattle": ("gso", {"dir": "BUNNY_RATTLE"}, (0.04, 0.08), "bin", None),
    "lacing_sheep": ("gso", {"dir": "LACING_SHEEP"}, (0.06, 0.12), "bin", None),
    "rolling_lion": ("gso", {"dir": "My_First_Rolling_Lion"}, (0.12, 0.25), "bin", "has wheels: may roll"),
    "pencil_case": ("gso", {"dir": "Horse_Dreams_Pencil_Case"}, (0.06, 0.15), "bin", None),
    "ballet_flat": ("gso", {"dir": "Cream_Tieks_Italian_Leather_Ballet_Flats"}, (0.15, 0.25), "bin", None),
    "game_case":   ("gso", {"dir": "Mario_Party_9_Wii_Game"}, (0.1, 0.15), "review", "flat and wider than the open gripper"),
    "slip_on_shoe": ("gso", {"dir": "Mens_Billfish_Slip_On_in_Coffee_e8bPKE9Lfgo"}, (0.3, 0.45), "review", "wider than the open gripper"),
    "teddy_bear":  ("gso", {"dir": "Lovable_Huggable_Cuddly_Boutique_Teddy_Bear_Beige"}, (0.2, 0.4), "review", "too big for the gripper"),
    "toy_dog":     ("gso", {"dir": "Dog"}, (0.2, 0.5), "review", "too big for the gripper"),
}
# more scanned floor clutter ([CaM]): mass from size (est.), pick-or-leave judged per trial from how it landed (judge())
CLUTTER = """Android_Figure_Chrome Android_Figure_Panda Dino_5 FARM_ANIMAL Schleich_Lion_Action_Figure Schleich_Spinosaurus_Action_Figure
Breyer_Horse_Of_The_Year_2015 My_Little_Pony_Princess_Celestia Nintendo_Mario_Action_Figure Nintendo_Yoshi_Action_Figure
Nickelodeon_Teenage_Mutant_Ninja_Turtles_Leonardo Transformers_Age_of_Extinction_Mega_1Step_Bumblebee_Figure
Guardians_of_the_Galaxy_Galactic_Battlers_Rocket_Raccoon_Figure Lego_Friends_Mia Toysmith_Windem_Up_Flippin_Animals_Dog
GRANDFATHER_DOLL DOLL_FAMILY 3D_Dollhouse_Happy_Brother 3D_Dollhouse_Sofa Animal_Planet_Foam_2Headed_Dragon
SpiderMan_Titan_Hero_12Inch_Action_Figure_5Hnn4mtkFsP Sootheze_Cold_Therapy_Elephant Elephant Android_Lego
BUNNY_RACER CAR_CARRIER_TRAIN COAST_GUARD_BOAT HELICOPTER SCHOOL_BUS Sonny_School_Bus SORTING_BUS SORTING_TRAIN SPEED_BOAT
TURBOPROP_AIRPLANE_WITH_PILOT Air_Hogs_Wind_Flyers_Set_Airplane_Red Vtech_Roll_Learn_Turtle Lalaloopsy_Peanut_Big_Top_Tricycle
STACKING_BEAR STACKING_RING Vtech_Stack_Sing_Rings_636_Months OWL_SORTER SHAPE_SORTER TWIST_SHAPE MONKEY_BOWLING
My_First_Animal_Tower OVAL_XYLOPHONE HAMMER_PEG HAMMER_BALL Fisher_price_Classic_Toys_Buzzy_Bee Melissa_Doug_Shape_Sorting_Clock
SNAIL_MEASURING_TAPE TOP_TEN_HI CASTLE_BLOCKS FAIRY_TALE_BLOCKS GEARS_PUZZLES_STANDARD_gcYxhNHhKlI TWISTED_PUZZLE
Melissa_Doug_Chunky_Puzzle_Vehicles BUILD_A_ROBOT FISHING_GAME Baby_Elements_Stacking_Cups Melissa_Doug_Cart_Turtle_Block
Crayola_Crayons_24_count Big_Dot_Pink_Pencil_Case Olive_Kids_Robots_Pencil_Case Leap_Frog_Paint_Dabber_Dot_Art_5_paint_bottles
Cootie_Game Simon_Swipe_Game Pokmon_X_Nintendo_3DS_Game Kid_Icarus_Uprising_Nintendo_3DS_Game DPC_Handmade_Hat_Brown
DPC_Thinsulate_Isolate_Gloves_Brown Toys_R_Us_Treat_Dispenser_Smart_Puzzle_Foobler Olive_Kids_Birdie_Lunch_Box""".split()
# Left out after checking the contact sheet (item_sheet.py): scans of several separate pieces fused into one rigid body
# (animal sets, a doll family, bowling pins, block sets, puzzle boards with loose pieces, instruments with their mallet,
# a pair of gloves). Nothing on a real floor moves like that. (toy_taxi, above, is two cars too: out of the pool since
# 29 Sep, SEPARATE_PIECES.)
FUSED = """FARM_ANIMAL DOLL_FAMILY Lego_Friends_Mia MONKEY_BOWLING My_First_Animal_Tower CASTLE_BLOCKS FAIRY_TALE_BLOCKS
GEARS_PUZZLES_STANDARD_gcYxhNHhKlI Cootie_Game FISHING_GAME SORTING_BUS HAMMER_BALL HAMMER_PEG OVAL_XYLOPHONE SPEED_BOAT
SHAPE_SORTER TWIST_SHAPE BUILD_A_ROBOT DPC_Thinsulate_Isolate_Gloves_Brown""".split()
CLUTTER = [c for c in CLUTTER if c not in FUSED]
CLUTTER_KG = {"DPC_Handmade_Hat_Brown": (0.06, 0.15), "DPC_Thinsulate_Isolate_Gloves_Brown": (0.05, 0.12)}   # soft: size says little (est.)
# Small household things left on floors, beyond toys (other non-toy items from the collection, small
# household ones only): shoes, mugs and bowls, towels, a glove, stationery, games and discs, computer mice, a
# toothbrush holder, a spatula, lunch boxes, pet things. Multi-packs (fused pieces) left out, as FUSED below says.
HOUSEHOLD = """Cream_Tieks_Italian_Leather_Ballet_Flats Matte_Black_Tieks_Italian_Leather_Ballet_Flats
Starstruck_Tieks_Glittery_Gold_Italian_Leather_Ballet_Flats Mens_ASV_Billfish_Boat_Shoe_in_Tan_Leather_wmUJ5PbwANc
Womens_Bluefish_2Eye_Boat_Shoe_in_Tan Timberland_Mens_Classic_2Eye_Boat_Shoe
Mens_Tremont_Kiltie_Tassel_Loafer_in_Black_Amaretto Mens_Gold_Cup_ASV_Capetown_Penny_Loafer_in_Black_EjPnk3E8fCs
Mens_Striper_Sneaker_in_White_rnp8HUli59Y TZX_Runner Reebok_REESCULPT_TRAINER_II NAPA_VALLEY_NAVAJO_SANDAL
UGG_Bailey_Bow_Womens_Clogs_Black_7 ASICS_GELDirt_Dog_4_SunFlameBlack ACE_Coffee_Mug_Kristen_16_oz_cup
Room_Essentials_Mug_White_Yellow Cole_Hardware_Mug_Classic_Blue Threshold_Ramekin_White_Porcelain
Room_Essentials_Bowl_Turquiose Threshold_Bead_Cereal_Bowl_White
Ecoforms_Cup_B4_SAN Cole_Hardware_Dishtowel_Red Tag_Dishtowel_Green Threshold_Hand_Towel_Blue_Medallion_16_x_27
Tag_Dishtowel_Waffle_Gray_Checks_18_x_26 Rexy_Glove_Heavy_Duty_Gloves_Medium Diamond_Visions_Scissors_Red
3M_Vinyl_Tape_Green_1_x_36_yd Shurtape_Tape_Purple_CP28 Crayola_Bonus_64_Crayons Crayola_Crayons_Washable_24_crayons
Pinwheel_Pencil_Case Wishbone_Pencil_Case Horse_Dreams_Pencil_Case Pokmon_Y_Nintendo_3DS_Game Mario_Party_9_Wii_Game
House_of_Cards_The_Complete_First_Season_4_Discs_DVD Mattel_SKIP_BO_Card_Game Mad_Gab_Refresh_Card_Game
Razer_Abyssus_Ambidextrous_Gaming_Mouse Razer_Taipan_White_Ambidextrous_Gaming_Mouse
Circo_Fish_Toothbrush_Holder_14995988 OXO_Cookie_Spatula Olive_Kids_Dinosaur_Land_Lunch_Box Paul_Frank_Dot_Lunch_Box
Granimals_20_Wooden_ABC_Blocks_Wagon_g2TinmUGGHI Melissa_Doug_Jumbo_Knob_Puzzles_Barnyard_Animals Dog PETS_ACCESSORIES""".split()
FUSED += """Rexy_Glove_Heavy_Duty_Gloves_Medium PETS_ACCESSORIES Granimals_20_Wooden_ABC_Blocks_Wagon_g2TinmUGGHI
Melissa_Doug_Jumbo_Knob_Puzzles_Barnyard_Animals""".split()                # seen on the sheet, 26 Sep: a pair, a set, not
HOUSEHOLD = [d for d in HOUSEHOLD if d not in FUSED]                        # the object named, a board with its pieces
CLUTTER = CLUTTER + HOUSEHOLD
CLUTTER_KG_SOFT = {d: (0.03, 0.09) for d in HOUSEHOLD if "towel" in d.lower()}   # est.
for _d in CLUTTER:
    _n = _d.lower()[:28]
    if _n not in ITEMS and not any(v[1].get("dir") == _d for v in ITEMS.values()): ITEMS[_n] = ("gso", {"dir": _d}, CLUTTER_KG.get(_d, CLUTTER_KG_SOFT.get(_d)), "auto", None)
# Towels are cloth (MuJoCo flex, drawn by Filament since the 26 Sep mujofil patch): one sheet the size of the folded
# towel as scanned (measured from the scan, 26 Sep), wearing the scan's photo as a repeating texture (stripes and
# checks read right tiled; a folded towel's layers are not modelled)
TOWELS = {"Cole_Hardware_Dishtowel_Red": (0.236, 0.172), "Tag_Dishtowel_Green": (0.237, 0.179),
          "Threshold_Hand_Towel_Blue_Medallion_16_x_27": (0.244, 0.217), "Tag_Dishtowel_Waffle_Gray_Checks_18_x_26": (0.199, 0.198)}
for _d, (_w, _h) in TOWELS.items():
    ITEMS[_d.lower()[:28]] = ("cloth", {"size": [_w, _h], "count": [10, 8], "rgba": [1, 1, 1, 1], "tex": f"{_d}/texture_1k.png",
                                        "gso_dir": _d}, (0.03, 0.09), "auto", "a towel: cloth")
GSO_NAMES = sorted({s["dir"] for k, s, *_ in ITEMS.values() if k == "gso"} | {s["gso_dir"] for k, s, *_ in ITEMS.values() if s.get("gso_dir")})
# Held-out items ([nuQ]): only for judging, never for training, demonstrations or tuning. Drawn
# 26 Sep over the whole library (toys and household things) with random.Random(20260926): about 20% of each kind of 3
# or more (kinds by name, sim-items [DGv]), leaving out the benches' fixtures (sock_box, block, card, wide_box, heavy).
# Every item before 26 Sep was used everywhere, so these count as unseen only from then on.
HELD_OUT = frozenset("""ace_coffee_mug_kristen_16_oz android_figure_chrome big_dot_pink_pencil_case breyer_horse_of_the_year_201
    cloth_sock coast_guard_boat cole_hardware_dishtowel_red cole_hardware_mug_classic_bl crayola_crayons_24_count
    diamond_visions_scissors_red guardians_of_the_galaxy_gala lalaloopsy_peanut_big_top_tr
    mens_asv_billfish_boat_shoe_ olive_kids_dinosaur_land_lun snail_measuring_tape sorting_train
    spiderman_titan_hero_12inch_ stacking_bear teddy_bear teether tzx_runner ugg_bailey_bow_womens_clogs_
    vtech_roll_learn_turtle vtech_stack_sing_rings_636_m""".split())
assert HELD_OUT <= set(ITEMS), HELD_OUT - set(ITEMS)


# Things that live on the floor and are never picked up: a pet's food and
# water bowls, part of the house every night, nudged about by the pet ([nAH] TJu). Never in the pool.
FIXTURES = {
    "dog_bowl_plastic": ("gso", {"dir": "Grreatv_Choice_Dog_Bowl_Gray_Bones_Plastic_20_fl_oz_total"}, (0.10, 0.20), "fixture", "a pet's bowl"),
    "dog_bowl_ceramic": ("gso", {"dir": "Top_Paw_Dog_Bow_Bone_Ceramic_13_fl_oz_total"}, (0.35, 0.60), "fixture", "a pet's bowl"),
}


SEPARATE_PIECES = frozenset({"toy_taxi", "car_carrier_train"})   # separate_objects() > 1 (checked 29 Sep; sim-items [w8q])


def separate_objects(name, touch_m=0.01, big=0.02):
    """a scan's surface split into objects: its mesh pieces (each at least `big` of the surface) grouped where they come
    within `touch_m`; each group's share of the surface. One entry for one object (sim-items [w8q])"""
    import trimesh
    from scipy.spatial import cKDTree
    d = GSO / ITEMS[name][1]["dir"]; f = d / "meshes" / "model.obj"
    m = trimesh.load(f if f.exists() else d / "model.obj", force="mesh")
    parts = [p for p in m.split(only_watertight=False) if p.area > big * m.area]; lab = list(range(len(parts)))
    def root(i):
        while lab[i] != i: i = lab[i]
        return i
    trees = [cKDTree(p.vertices) for p in parts]
    for i in range(len(parts)):
        for j in range(i + 1, len(parts)):
            if trees[j].query(parts[i].vertices, distance_upper_bound=touch_m)[0].min() < touch_m: lab[root(i)] = root(j)
    share = {}
    for i, p in enumerate(parts): share[root(i)] = share.get(root(i), 0.0) + p.area / m.area
    return sorted(share.values(), reverse=True)


def pool(split="train"):
    """the items a run may use: "train" (everything but the held-out set) or "heldout" (judging only); never a scan of
    separate objects fused into one body ([w8q])"""
    if split not in ("train", "heldout"): raise ValueError(split)
    return [n for n in ITEMS if (n in HELD_OUT) == (split == "heldout") and n not in SEPARATE_PIECES]


PICKABLE = [n for n in pool("train") if ITEMS[n][3] == "bin"]              # training items only: every trainer and bench
PICKABLE_HELD_OUT = [n for n in pool("heldout") if ITEMS[n][3] == "bin"]   # draws from PICKABLE, so none can reach one


def add(spec, name, pos, uid=None):
    """add item `name` to an MjSpec as a free body called uid (default name) at pos; returns the body name"""
    kind, s, mr, expect, why = ITEMS[name] if name in ITEMS else FIXTURES[name]; uid = uid or name
    if kind in ("box", "cylinder", "sphere"):
        b = spec.worldbody.add_body(); b.name = uid; b.pos = list(pos); b.add_freejoint()
        g = b.add_geom(); g.name = uid + "_g"; h = s["half"]
        if kind == "box": g.type = mujoco.mjtGeom.mjGEOM_BOX; g.size = h
        elif kind == "sphere": g.type = mujoco.mjtGeom.mjGEOM_SPHERE; g.size = [h[0], 0, 0]
        else:
            g.type = mujoco.mjtGeom.mjGEOM_CYLINDER; g.size = [h[0], h[2], 0]
            if s.get("lying"): g.quat = [0.7071068, 0.7071068, 0, 0]
        g.mass = float(np.mean(mr)); g.rgba = s["rgba"]; g.friction = [1.0, 0.02, 0.001]; g.condim = 4
        if kind == "sphere": g.condim = 6; g.friction = [1.0, 0.005, 0.0005]     # rolling resistance: it rolls
        for k in spec.keys: k.qpos = list(k.qpos) + list(pos) + [1, 0, 0, 0]       # keyframes grow with the new free joint
    elif kind == "gso":
        it = mujoco.MjSpec.from_file(str(GSO / s["dir"] / "model.xml"))
        for m_ in it.meshes: m_.file = str(GSO / s["dir"] / m_.file)
        for t_ in it.textures:                                                      # 1024 px copies: plenty at wrist range, far less GPU memory
            small = GSO / s["dir"] / t_.file.replace(".png", "_1k.png"); t_.file = str(small if small.exists() else GSO / s["dir"] / t_.file)
        f = spec.worldbody.add_frame(pos=list(pos)); bb = f.attach_body(it.body("model"), uid + "/", "")
        bb.name = uid; bb.add_freejoint()
        for g in bb.geoms:
            g.friction = [1.0, 0.02, 0.001]; g.condim = 4
            if g.contype != 0 or g.conaffinity != 0: g.density = 300.0                # hollow-toy density (est.); set_mass rescales per trial
    elif kind in ("cloth", "rope"):                          # soft: built from MJCF (flexcomp cloth, cable rope), then attached
        top = "holder" if kind == "cloth" else uid
        mat = uid.replace("/", "_") + "_mat" if s.get("tex") else None             # a textured cloth (a towel): its photo
        asset = (f'<asset><texture name="{mat}_t" type="2d" file="{GSO / s["tex"]}"/><material name="{mat}" texture="{mat}_t" '
                 f'texrepeat="2 2"/></asset>') if mat else ""
        xml = ('<mujoco><extension><plugin plugin="mujoco.elasticity.cable"/></extension>' + asset + '<worldbody>'
               + (f'<body name="holder">{xml_snippet(name, uid, pos, mat)}</body>' if kind == "cloth" else xml_snippet(name, uid, pos)) + '</worldbody></mujoco>')
        ch = mujoco.MjSpec.from_string(xml); f = spec.worldbody.add_frame(); f.attach_body(ch.body(top), uid + "/" if kind == "cloth" else "", "")
    return uid


def xml_snippet(name, uid, pos, material=None):
    """soft items as MJCF text (flexcomp / cable composite), for models built from XML; material: a cloth's texture"""
    kind, s, mr, expect, why = ITEMS[name]
    rgba = " ".join(map(str, s["rgba"])); p = " ".join(f"{v:.4f}" for v in pos)
    if kind == "cloth":
        cx, cy = s["count"]; sx, sy = s["size"][0] / (cx - 1), s["size"][1] / (cy - 1)
        return (f'<flexcomp name="{uid}" type="grid" count="{cx} {cy} 1" spacing="{sx:.4f} {sy:.4f} 0.01" pos="{p}" '
                f'radius="0.003" rgba="{rgba}" dim="2" mass="{np.mean(mr):.3f}"' + (f' material="{material}"' if material else "") + '>'
                f'<contact condim="4" solref="0.02 1" selfcollide="none"/><edge equality="true" damping="0.05"/>'
                f'<elasticity young="3e5" poisson="0.3" thickness="0.002" elastic2d="bend"/></flexcomp>')
    if kind == "rope":
        n = s["count"]; L = s["length"]
        return (f'<body name="{uid}" pos="{p}"><freejoint/><composite type="cable" prefix="{uid}_" count="{n} 1 1" '
                f'curve="s" size="{L}" initial="none"><plugin plugin="mujoco.elasticity.cable"><config key="twist" value="1e4"/>'
                f'<config key="bend" value="1e4"/></plugin><joint kind="main" damping="0"/>'   # no damping: DimOS's Go1 scene disables implicit damping, and damped light bodies go unstable
                f'<geom type="capsule" size="{s["radius"]}" rgba="{rgba}" density="300" condim="4"/></composite></body>')
    raise ValueError(name)


JAW_MAX, ARM_KG = jw.max_m(), 1.5   # widest item the open jaws take (m, less any pads); the arm's rated payload (kg)
BAND = 0.04                      # the jaws close on the lowest 4 cm of an item (they grip ~1 cm up)


_SURF = {}


SURF_PER_CM2, SURF_MIN, SURF_MAX = 8, 1500, 30000


def _mesh_surface(m, mid, n=None):
    """points spread over a mesh's surface by area (its vertices alone miss the middle of a box's big flat faces), 8 a
    square centimetre (at least 1500): at a fixed 1500 a 16 cm box had about one a cm2, and the gaps between them
    passed for gaps between parts, so the box was "gripped by a part" (29 Sep, too-big bench)"""
    v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
    key = (int(m.mesh_vertnum[mid]), int(m.mesh_facenum[mid]), float(v.sum()), float(np.abs(v).sum()))   # the mesh's own content:
    if key not in _SURF:                           # keyed by id(m), a freed model's address reused by the next handed it the
                                                   # last toy's points (29 Sep: the taxi measured 26.6 x 26.5 cm after other
                                                   # toys, 12.7 x 5.9 alone; every round's tiers had been scrambled by it)
        f = m.mesh_face[m.mesh_faceadr[mid]:m.mesh_faceadr[mid] + m.mesh_facenum[mid]]; a, b_, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
        area = np.linalg.norm(np.cross(b_ - a, c - a), axis=1); rng = np.random.default_rng(mid)
        n = n or int(np.clip(area.sum() / 2 * 1e4 * SURF_PER_CM2, SURF_MIN, SURF_MAX))
        k = rng.choice(len(f), n, p=area / area.sum()); r1, r2 = rng.random((2, n)); flip = r1 + r2 > 1; r1[flip], r2[flip] = 1 - r1[flip], 1 - r2[flip]
        _SURF[key] = np.vstack([v, a[k] + r1[:, None] * (b_[k] - a[k]) + r2[:, None] * (c[k] - a[k])])
    return _SURF[key]


def item_points(m, d, b):
    """world points on item body b's surface: spread over each mesh, or the corners of each primitive geom's box"""
    P = []
    for i in range(m.ngeom):
        if m.body_rootid[m.geom_bodyid[i]] != b: continue
        Rg = d.geom_xmat[i].reshape(3, 3)
        if m.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH:
            v = _mesh_surface(m, m.geom_dataid[i])
        else:
            c, h = m.geom_aabb[i][:3], m.geom_aabb[i][3:]
            v = c + h * np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        P.append(d.geom_xpos[i] + v @ Rg.T)
    return np.vstack(P)


_FOOT: dict = {}




def footprint_half(name):
    """an item's half-extents on the floor (m, largest first) once it has landed and settled, for scoring a find against it
    (the house nights had scored every toy as a 12 cm square, so a long toy found at its end counted as missed)"""
    if name not in _FOOT:
        spec = mujoco.MjSpec(); spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1, 1, .1])
        add(spec, name, [0, 0, 0.1], uid="it"); m = spec.compile(); d = mujoco.MjData(m)
        for _ in range(1500): mujoco.mj_step(m, d)                            # resting as it lands, not as catalogued
        P = item_points(m, d, m.body("it").id)[:, :2]; P = P - P.mean(0)
        ev = np.linalg.eigh(np.cov(P.T))[1]; Q = P @ ev
        _FOOT[name] = sorted((float((Q[:, k].max() - Q[:, k].min()) / 2) for k in range(2)), reverse=True)
        L, S_ = (Q[:, 1], Q[:, 0]) if np.ptp(Q[:, 1]) >= np.ptp(Q[:, 0]) else (Q[:, 0], Q[:, 1])   # long axis, short axis
        bands = [S_[(L >= a) & (L < a + 2 * STRIP)] for a in np.arange(L.min(), L.max() - 2 * STRIP + 1e-9, 0.005)]
        _GRIP[name] = min([float(np.ptp(b)) for b in bands if len(b) >= 20] or [2 * _FOOT[name][1]])
    return _FOOT[name]


_GRIP: dict = {}
_LOCAL: dict = {}


def grip_width_at(name, quat):
    """grip_width for the item lying as `quat` says (w, x, y, z; the sim's truth), not as it lands in a test drop: a toy
    on its side and the same toy upright differ by centimetres (a car carrier: 3.9 vs 8.3 cm)"""
    if name not in _LOCAL:                                       # its surface points in its own frame, measured once
        spec = mujoco.MjSpec(); add(spec, name, [0, 0, 0.5], uid="it"); m = spec.compile(); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
        b = m.body("it").id; P = item_points(m, d, b)
        _LOCAL[name] = (P - d.xpos[b]) @ d.xmat[b].reshape(3, 3)
    Rm = np.zeros(9); mujoco.mju_quat2Mat(Rm, np.asarray(quat, float)); P = (_LOCAL[name] @ Rm.reshape(3, 3).T)[:, :2]
    P = P - P.mean(0); ev = np.linalg.eigh(np.cov(P.T))[1]; Q = P @ ev
    L, S_ = (Q[:, 1], Q[:, 0]) if np.ptp(Q[:, 1]) >= np.ptp(Q[:, 0]) else (Q[:, 0], Q[:, 1])
    bands = [S_[(L >= a) & (L < a + 2 * STRIP)] for a in np.arange(L.min(), L.max() - 2 * STRIP + 1e-9, 0.005)]
    return min([float(np.ptp(b_)) for b_ in bands if len(b_) >= 20] or [float(np.ptp(S_))])


def grip_width(name):
    """the narrowest the jaws can find across an item (m): its narrowest band of the width the jaws close on (2 x STRIP)
    along its length, settled on the floor. A tapered item (a pencil case's zipped end, a shoe's heel) fits where the
    whole-item width says it doesn't"""
    footprint_half(name); return _GRIP[name]


_SETTLED: dict = {}
_TIER: dict = {}
TINY_M, LONG_M, SLENDER, THIN_M, NEAR_LIMIT_M, FLAT_M, PART_ROOM_M, APPENDAGE = 0.06, 0.20, 3.5, 0.02, 0.075, 0.02, 0.02, 0.65


def _points(name, quat=None):
    """an item's surface points, centred over its middle on the floor: as it lies by `quat` (w, x, y, z), or as it
    lands in a test drop"""
    if quat is not None:
        grip_width_at(name, quat); Rm = np.zeros(9); mujoco.mju_quat2Mat(Rm, np.asarray(quat, float)); P = _LOCAL[name] @ Rm.reshape(3, 3).T
    else:
        if name not in _SETTLED:
            spec = mujoco.MjSpec(); spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1, 1, .1])
            add(spec, name, [0, 0, 0.1], uid="it"); m = spec.compile(); d = mujoco.MjData(m)
            for _ in range(1500): mujoco.mj_step(m, d)
            _SETTLED[name] = item_points(m, d, m.body("it").id)
        P = _SETTLED[name]
    return P - np.r_[P[:, :2].mean(0), P[:, 2].min()]


def tier_of(narrow, length, breadth, height, part, jaw_max, body=None):
    """[QyZ]'s tier from an item's measures (m): `narrow` its narrowest band, `length` x `breadth` its footprint,
    `height`, `part` its narrowest grippable part (None if none), `body` its bands' median width (its body, not an
    appendage; None: taken as `narrow`)"""
    import bin_geom
    if not bin_geom.footprint_fits(length, breadth): return "too_big"
    if narrow > jaw_max:                      # awkward: taken only by a part, with room to spare, and not lying flat
        return "hard" if part is not None and part <= jaw_max - PART_ROOM_M and height >= FLAT_M else "too_big"
    tiny = max(length, height) < TINY_M                  # small every way: a toy standing up is taken readily
    long_ = length > LONG_M and length >= SLENDER * breadth
    by_appendage = body is not None and body > NEAR_LIMIT_M and narrow <= APPENDAGE * body   # a thin part of a wide body
                                                          # (a mug's handle, a plane's wing; not a shoe's taper, 6.3 of 7.6)
    return "hard" if tiny or long_ or by_appendage or narrow < THIN_M or narrow > NEAR_LIMIT_M or height < FLAT_M else "easy"


def _body_width(Q):
    """the median width of an item's bands (the jaws' 4 cm across its long axis), Q its footprint on its own axes: the
    width of its body, where the narrowest band may be a leg or a horn"""
    L, S_ = (Q[:, 1], Q[:, 0]) if np.ptp(Q[:, 1]) >= np.ptp(Q[:, 0]) else (Q[:, 0], Q[:, 1])
    bands = [S_[(L >= a) & (L < a + 2 * STRIP)] for a in np.arange(L.min(), L.max() - 2 * STRIP + 1e-9, 0.005)]
    w = [float(np.ptp(b)) for b in bands if len(b) >= 20]
    return float(np.median(w)) if w else float(np.ptp(S_))


def grasp_tier(name, quat=None):
    """the slice's grasp tier for an item ([QyZ], set from geometry before any binning result): as it lies by `quat`
    (the sim's truth), or as it lands in a test drop. "too_big" if its footprint doesn't fit the bin or no part of it
    fits the jaws with room; "hard" if awkward (only a part fits, or it fits only across an appendage of a body at the
    jaws' limit), tiny every way, long and slender, thin, near the jaws' limit or flat; else "easy" (the winged pony, tiny toys and long ones that rotate are hard)"""
    key = (name, None if quat is None else tuple(np.round(np.asarray(quat, float), 3)))
    if key not in _TIER:
        kept = _tier_kept(name) if quat is None else None          # a test drop's tier, measured before (TIER_CACHE)
        if kept is not None: _TIER[key] = kept
        else:
            import jaws
            from wrist_refine import part_grip_options
            P = _points(name, quat); Q = P[:, :2] @ np.linalg.eigh(np.cov(P[:, :2].T))[1]
            L, B = sorted((float(np.ptp(Q[:, k])) for k in range(2)), reverse=True)
            narrow = grip_width_at(name, quat) if quat is not None else grip_width(name)
            J = jaws.max_m(); parts = [o[0] for o in part_grip_options(P[:, :2]) if 0.005 <= o[0] <= J - 0.01]
            _TIER[key] = tier_of(narrow, L, B, float(np.ptp(P[:, 2])), min(parts) if parts else None, J, _body_width(Q))
            if quat is None: _tier_keep(name, _TIER[key])
    return _TIER[key]


# A test drop's tier kept on disk (gitignored, sim/arm/_*): measuring one settles the toy in a small sim and tries every
# part of it between the jaws, seconds a toy, so the whole library took tens of minutes in every fresh process
# (house_items.py, each night and test). Keyed by everything the tier depends on: the item's entry, its scan's geometry
# files, the whole source of the modules that measure and sort it (any edit to them measures afresh), the jaws fitted
# and the simulator's version; a key that matches is the same tier the measurement gives. HK_TIER_CACHE=0 measures
# without it; warm_tiers.py fills it (in parallel) and clears stale entries.
TIER_CACHE = Path(__file__).resolve().parent / "_cache" / "grasp_tier"
_TIER_CODE: list = []


def tier_key(name):
    """the drop-test tier's cache key for item `name` (hex): what grasp_tier(name) depends on, hashed"""
    import hashlib, sys, jaws, bin_geom, wrist_refine
    if not _TIER_CODE:
        _TIER_CODE.append(hashlib.sha256(b"".join(Path(m.__file__).read_bytes()
                                                  for m in (sys.modules[__name__], wrist_refine, jaws, bin_geom))).hexdigest())
    entry = ITEMS[name] if name in ITEMS else FIXTURES[name]
    h = hashlib.sha256(f"{name}|{entry!r}|{_TIER_CODE[0]}|{jaws.JAWS}|{jaws.max_m()!r}|{mujoco.__version__}|{np.__version__}".encode())
    if entry[0] == "gso":
        d = GSO / entry[1]["dir"]
        for f in sorted(d.iterdir()):
            if f.suffix in (".obj", ".xml", ".stl", ".msh"): h.update(f.name.encode()); h.update(f.read_bytes())
    return h.hexdigest()


def _tier_on(): return __import__("os").environ.get("HK_TIER_CACHE", "1") != "0"


def _tier_kept(name):
    if not _tier_on(): return None
    import json
    try: t = json.loads((TIER_CACHE / f"{tier_key(name)}.json").read_text())["tier"]
    except (OSError, ValueError, KeyError): return None
    return t if t in ("easy", "hard", "too_big") else None


def _tier_keep(name, tier):
    import json, os
    if not _tier_on(): return
    TIER_CACHE.mkdir(parents=True, exist_ok=True); f = TIER_CACHE / f"{tier_key(name)}.json"
    tmp = f.with_suffix(f".{os.getpid()}.tmp"); tmp.write_text(json.dumps({"name": name, "tier": tier}))
    os.replace(tmp, f)                                     # whole or not at all: nights run side by side


def grippable(split="train"):
    """the items the arm's demonstrations and grasp benches use: every item of the split that is one rigid body and fits
    between the jaws across its narrow side (a 5 mm margin), whatever its source (fixed-shape, toy or household). Widths
    are measured once and kept beside the scans (models/items/footprints.json)"""
    import json, jaws
    cache = GSO.parent / "footprints.json"; foot = json.loads(cache.read_text()) if cache.exists() else {}
    names = [n for n in pool(split) if ITEMS[n][3] in ("bin", "auto") and ITEMS[n][0] not in ("cloth", "rope")]
    todo = [n for n in names if n not in foot]
    for n in todo: foot[n] = footprint_half(n)
    if todo: cache.write_text(json.dumps(foot, indent=0))
    return [n for n in names if grip_width(n) <= jaws.max_m() - 0.005]


STRIP = 0.02                     # the jaws close within +-2 cm of the grasp point (8 mm fingers, and where they land)


def true_width(P):
    """How wide the item is where the jaws would close: over closing directions 5 deg apart, the narrowest extent of the
    item's lowest 4 cm within a finger-wide strip (+-2 cm) through the point nearest its middle, or 4 cm either side of
    it along a long item, as the pick routine may grip (wrist_refine.grip_options) (m); and its height (m). A long
    dinosaur is gripped across its body, not across the curl of its tail."""
    z0 = P[:, 2].min(); low = P[P[:, 2] <= z0 + BAND][:, :2]; g0 = low[np.argmin(np.linalg.norm(low - low.mean(0), axis=1))]
    ws = []
    for a in np.radians(np.arange(0, 180, 5)):
        jaw = np.array([np.cos(a), np.sin(a)]); along = np.array([-jaw[1], jaw[0]]); t = (low - g0) @ along
        for slide in (0.0, -0.04, 0.04):
            if slide and not (t.min() <= slide - STRIP if slide < 0 else t.max() >= slide + STRIP): continue
            sel = low[np.abs(t - slide) < STRIP]; ws.append(np.ptp((sel if len(sel) >= 3 else low) @ jaw))
    return float(min(ws)), float(np.ptp(P[:, 2]))


def judge(m, d, uid):
    """the right outcome for an item as it lies now: ("bin", None) or ("review", why). Scanned items are judged from how
    they landed (a toy on its side can be wider than the jaws: then leaving it is right) by the true width across the
    jaws, height and mass; primitives, cloth and rope keep their fixed outcome"""
    name = uid.split("#")[0]; kind, expect, why = ITEMS[name][0], ITEMS[name][3], ITEMS[name][4]
    if kind != "gso": return expect, why
    b = m.body(uid).id; w, h = true_width(item_points(m, d, b))
    if m.body_mass[m.body_rootid == b].sum() > ARM_KG:                  # live masses (set_mass changes them; subtreemass is compile-time)
        return "review", f"heavier than the arm's rated {ARM_KG} kg"
    if w >= JAW_MAX - 0.005: return "review", f"{w * 100:.0f} cm across as it landed: wider than the open gripper"
    if h < 0.006: return "review", "too flat to pinch from the floor"
    return "bin", None


def set_mass(m, uid, rng, name=None):
    """draw this trial's mass within the item's range (est.) and scale the body's inertia to match. Items without a
    range: bounding-box volume x 60-200 kg/m3 (est.: hollow plastic toys, some air inside), 10 g to 1 kg"""
    name = name or uid.split("#")[0]; mr = (ITEMS.get(name) or FIXTURES[name])[2]; b = m.body(uid).id   # name: the library item, when uid differs
    if mr is None:
        d0 = mujoco.MjData(m); mujoco.mj_kinematics(m, d0); P = item_points(m, d0, b)
        vol = float(np.prod(np.ptp(P, axis=0))); mr = (float(np.clip(vol * 60, 0.01, 1.0)), float(np.clip(vol * 200, 0.012, 1.0)))
    target = rng.uniform(*mr)
    bs = np.nonzero(m.body_rootid == b)[0]; total = float(m.body_mass[bs].sum())     # every body of the item (a rope is 16),
    if total > 0:                                                               # scaled together: scaling only the first left
        k = target / total; m.body_mass[bs] *= k; m.body_inertia[bs] *= k       # the rest ~0.4 g each, and squeezing those
    return target                                                               # blew the solver up and flung the robot
