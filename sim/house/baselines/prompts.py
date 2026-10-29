"""What the detectors are asked ([XEi], [DZr]), fixed on 4 Oct 2026 before any detector's result was seen.

PROMPTS: local detection's prompt set (OWLv2's text queries). GENERIC: the prompts that name no kind ("toy" covers
anything). KINDS: which specific prompt names each toy kind in the sim's library, by its name; a kind no specific
prompt names is an unknown object ([tWE]): a detector can still catch it through a generic prompt, the floor
comparison needs no prompt at all. FRONTIER: the fixed instruction the frontier model gets with each frame.
STRUCTURE ([En1], fixed 4 Oct 2026 before any prompt-free detector's result): labels a prompt-free detector (YOLO-E
prompt-free, Florence-2's object detection) gives the house itself, its furniture and fittings, dropped before
placement; every other label is kept (a thing on the floor need not be a toy to be found)."""

PROMPTS = ["toy", "stuffed animal", "ball", "block", "shoe", "cup", "card game", "figure", "toy car", "doll", "sock", "book"]
GENERIC = {"toy"}

KINDS = {                              # toy kind (the library's name, or its start) -> the specific prompt naming it
    "figure": "figure", "schleich_lion_action_figure": "figure", "nintendo_mario_action_figure": "figure",
    "mattel_skip_bo_card_game": "card game",
    "brick": "block", "lego_1x1": "block",
    "room_essentials_mug_white_ye": "cup",
    "mens_gold_cup_asv_capetown_p": "shoe", "ballet_flat": "shoe", "womens_bluefish_2eye_boat_sh": "shoe",
    # named by none: turboprop_airplane_with_pilo (an aeroplane), melissa_doug_cart_turtle_blo (a pull-along cart),
    # lacing_sheep (a wooden lacing toy), triceratops and dino_5 (dinosaurs), threshold_bead_cereal_bowl_w (a bowl)
}


def named(name):
    """whether a specific prompt names this toy kind ([tWE]: if not, it is an unknown object)"""
    return name in KINDS


FRONTIER = """You are looking at one frame from a small robot dog's head camera, about 30 cm above the floor of a house, at night.
The robot tidies toys and other small objects off the floor.

List every toy or other loose object lying on the floor in this image: anything that does not belong on an empty floor
(toys, shoes, cups, cards, blocks, figures, anything small someone left there). Do not list furniture, walls,
rugs, doors, floor patterns, stains or shadows.

For each one give a short label and a tight pixel box around it in this image: [x0, y0, x1, y1], x to the right and y
down from the top-left corner, the image being {width} x {height} pixels.

Answer with JSON only, nothing else: a list like
[{{"label": "toy car", "box": [412, 230, 468, 262]}}]
or [] if there is nothing on the floor."""

STRUCTURE = ["floor", "flooring", "hardwood", "parquet", "tile", "carpet", "rug", "mat", "doormat", "wall", "ceiling",
             "door", "doorway", "window", "curtain", "blind", "baseboard", "skirting board", "molding", "stairs",
             "room", "living room", "bedroom", "kitchen", "hallway", "interior", "house", "home", "furniture", "table",
             "desk", "chair", "couch", "sofa", "bed", "cabinet", "cupboard", "drawer", "shelf", "bookshelf", "wardrobe",
             "dresser", "counter", "lamp", "light", "radiator", "fireplace", "television", "plant", "houseplant",
             "shadow", "corner"]
