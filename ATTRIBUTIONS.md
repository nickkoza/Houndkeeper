# Attributions

Third-party assets the code uses, their licences and the credit they ask for. The project's own work is under the MIT
licence (`LICENSE`); everything here keeps its own. The per-asset record, with sources, pinned checksums and what was
changed, is `sim/house/benchmark/asset_manifest.json` (written by `sim/house/benchmark_release.py manifest`). Most of
these are fetched rather than carried in git; `models/README.md` says how.

## Robot models (in git)

- **Unitree Go2** (`models/go2/`): from DeepMind's MuJoCo Menagerie `unitree_go2`. Copyright Unitree Robotics,
  BSD-3-Clause (`models/go2/LICENSE`).
- **Maker Arm** (`models/maker_arm/`): from MakerMods' `maker-arm-sdk` at b30d05a. Apache-2.0
  (`models/maker_arm/LICENSE`).

## Scanned objects (fetched by `tools/fetch_items.sh`)

The sims' toys and household items are scans from **Google Scanned Objects** by Google Research, licensed
[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/), as MuJoCo models converted by
[kevinzakka/mujoco_scanned_objects](https://github.com/kevinzakka/mujoco_scanned_objects) (MIT). Changed: each
texture is scaled to 1024 px. The 115 objects used:

`3D_Dollhouse_Happy_Brother`, `3D_Dollhouse_Sofa`, `3M_Vinyl_Tape_Green_1_x_36_yd`, `ACE_Coffee_Mug_Kristen_16_oz_cup`, `ASICS_GELDirt_Dog_4_SunFlameBlack`, `Air_Hogs_Wind_Flyers_Set_Airplane_Red`, `Android_Figure_Chrome`, `Android_Figure_Orange`, `Android_Figure_Panda`, `Android_Lego`, `Animal_Planet_Foam_2Headed_Dragon`, `BABY_CAR`, `BIRD_RATTLE`, `BUNNY_RACER`, `BUNNY_RATTLE`, `Baby_Elements_Stacking_Cups`, `Big_Dot_Pink_Pencil_Case`, `Breyer_Horse_Of_The_Year_2015`, `CAR_CARRIER_TRAIN`, `CITY_TAXI_POLICE_CAR`, `COAST_GUARD_BOAT`, `Circo_Fish_Toothbrush_Holder_14995988`, `Cole_Hardware_Dishtowel_Red`, `Cole_Hardware_Mug_Classic_Blue`, `Crayola_Bonus_64_Crayons`, `Crayola_Crayons_24_count`, `Crayola_Crayons_Washable_24_crayons`, `Cream_Tieks_Italian_Leather_Ballet_Flats`, `DPC_Handmade_Hat_Brown`, `Diamond_Visions_Scissors_Red`, `Dino_3`, `Dino_4`, `Dino_5`, `Dog`, `Ecoforms_Cup_B4_SAN`, `Elephant`, `FIRE_TRUCK`, `Fisher_price_Classic_Toys_Buzzy_Bee`, `GRANDFATHER_DOLL`, `Great_Dinos_Triceratops_Toy`, `Grreatv_Choice_Dog_Bowl_Gray_Bones_Plastic_20_fl_oz_total`, `Guardians_of_the_Galaxy_Galactic_Battlers_Rocket_Raccoon_Figure`, `HELICOPTER`, `Horse_Dreams_Pencil_Case`, `House_of_Cards_The_Complete_First_Season_4_Discs_DVD`, `Kid_Icarus_Uprising_Nintendo_3DS_Game`, `Kong_Puppy_Teething_Rubber_Small_Pink`, `LACING_SHEEP`, `Lalaloopsy_Peanut_Big_Top_Tricycle`, `Leap_Frog_Paint_Dabber_Dot_Art_5_paint_bottles`, `Lovable_Huggable_Cuddly_Boutique_Teddy_Bear_Beige`, `Mad_Gab_Refresh_Card_Game`, `Mario_Party_9_Wii_Game`, `Matte_Black_Tieks_Italian_Leather_Ballet_Flats`, `Mattel_SKIP_BO_Card_Game`, `Melissa_Doug_Cart_Turtle_Block`, `Melissa_Doug_Chunky_Puzzle_Vehicles`, `Melissa_Doug_Shape_Sorting_Clock`, `Mens_ASV_Billfish_Boat_Shoe_in_Tan_Leather_wmUJ5PbwANc`, `Mens_Billfish_Slip_On_in_Coffee_e8bPKE9Lfgo`, `Mens_Gold_Cup_ASV_Capetown_Penny_Loafer_in_Black_EjPnk3E8fCs`, `Mens_Striper_Sneaker_in_White_rnp8HUli59Y`, `Mens_Tremont_Kiltie_Tassel_Loafer_in_Black_Amaretto`, `My_First_Rolling_Lion`, `My_Little_Pony_Princess_Celestia`, `NAPA_VALLEY_NAVAJO_SANDAL`, `Nickelodeon_Teenage_Mutant_Ninja_Turtles_Leonardo`, `Nintendo_Mario_Action_Figure`, `Nintendo_Yoshi_Action_Figure`, `OWL_SORTER`, `OXO_Cookie_Spatula`, `Olive_Kids_Birdie_Lunch_Box`, `Olive_Kids_Dinosaur_Land_Lunch_Box`, `Olive_Kids_Robots_Pencil_Case`, `Paul_Frank_Dot_Lunch_Box`, `Pinwheel_Pencil_Case`, `Pokmon_X_Nintendo_3DS_Game`, `Pokmon_Y_Nintendo_3DS_Game`, `Razer_Abyssus_Ambidextrous_Gaming_Mouse`, `Razer_Taipan_White_Ambidextrous_Gaming_Mouse`, `Reebok_REESCULPT_TRAINER_II`, `Room_Essentials_Bowl_Turquiose`, `Room_Essentials_Mug_White_Yellow`, `SCHOOL_BUS`, `SNAIL_MEASURING_TAPE`, `SORTING_TRAIN`, `STACKING_BEAR`, `STACKING_RING`, `Schleich_Lion_Action_Figure`, `Schleich_Spinosaurus_Action_Figure`, `Shurtape_Tape_Purple_CP28`, `Simon_Swipe_Game`, `Sonny_School_Bus`, `Sootheze_Cold_Therapy_Elephant`, `SpiderMan_Titan_Hero_12Inch_Action_Figure_5Hnn4mtkFsP`, `Starstruck_Tieks_Glittery_Gold_Italian_Leather_Ballet_Flats`, `TOP_TEN_HI`, `TURBOPROP_AIRPLANE_WITH_PILOT`, `TWISTED_PUZZLE`, `TZX_Runner`, `Tag_Dishtowel_Green`, `Tag_Dishtowel_Waffle_Gray_Checks_18_x_26`, `Threshold_Bead_Cereal_Bowl_White`, `Threshold_Hand_Towel_Blue_Medallion_16_x_27`, `Threshold_Ramekin_White_Porcelain`, `Timberland_Mens_Classic_2Eye_Boat_Shoe`, `Top_Paw_Dog_Bow_Bone_Ceramic_13_fl_oz_total`, `Toys_R_Us_Treat_Dispenser_Smart_Puzzle_Foobler`, `Toysmith_Windem_Up_Flippin_Animals_Dog`, `Transformers_Age_of_Extinction_Mega_1Step_Bumblebee_Figure`, `UGG_Bailey_Bow_Womens_Clogs_Black_7`, `Vtech_Roll_Learn_Turtle`, `Vtech_Stack_Sing_Rings_636_Months`, `Wishbone_Pencil_Case`, `Womens_Bluefish_2Eye_Boat_Shoe_in_Tan`.

## Imported models (Sketchfab, CC BY 4.0)

Each converted from glTF to OBJ and PNG, scaled and re-oriented by `tools/import_glb.py`. Licensed
[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/); credit them wherever a render or video shows them.

| Model | Author | Source |
|---|---|---|
| `bananas` | ShekhirevaVictoria | https://sketchfab.com/3d-models/banana-e0c2951ac0ca47b8978b2a6eca426c3f |
| `salt_pepper` | Replexx | https://sketchfab.com/3d-models/salt-and-pepper-shakers-de57284dcdc941d1b728f91fd60654a4 |
| `snake_plant` | FranklySketch | https://sketchfab.com/3d-models/snake-plant-in-pot-68b8acaeb29e402ea4dc7c981cb5b497 |
| `table_plant` | januart1st0101 | https://sketchfab.com/3d-models/small-potted-plants-9b598061fd644696a63e059e64f164ea |
| `wing_chair` | pranavranjan2212 | https://sketchfab.com/3d-models/vintage-wingback-armchair-d606a3eb868d4122855bc7a82d2889a9 |

## Textures and floor photos (ambientCG, CC0)

The house's surface textures and floor photos are from [ambientCG](https://ambientcg.com), released under CC0 (public
domain): no credit is required, and we give it gladly. Each material is named in the manifest.

## Tools carried with the repository

- The Railyard Method skill (`.claude/skills/spec-driven-change/`) lists its own third-party notices in
  `THIRD_PARTY_NOTICES.md`.
