## base (google/owlv2-base-patch16-ensemble, threshold 0.2, false positives over six nights 3)

Reproduction: {'published_found': 41, 'published_fp': 3, 'rerun_found': 41, 'rerun_fp': 3, 'same_found_set': True}

| Miss | Class | Frames in view | Best frame: long side px, range m | 12 prompts on the toy (anywhere in reach) | Generic, on the toy | Own name, on the toy | Found by (matched threshold) | The 12 lowered to its score |
|---|---|---|---|---|---|---|---|---|
| s71_t2/lego_1x1_1 | resolution/size | 10 | 6, 0.79 | 0.026 (0.026) | object on the floor 0.057, small object 0.030, clutter 0.002, item 0.004, thing on the floor 0.031 | lego brick 0.022 |  | - |
| s71_t2/dino_5_4 | prompt, generic | 7 | 109, 0.47 | 0.062 (0.062) | object on the floor 0.429, small object 0.183, clutter 0.024, item 0.018, thing on the floor 0.239 | dinosaur 0.018 | 12+object on the floor, 12+all five | 0.061: 26 FP, finds it True |
| s71_t2/threshold_bead_cereal_bowl_w_6 | prompt, generic | 10 | 52, 1.23 | 0.067 (0.067) | object on the floor 0.605, small object 0.354, clutter 0.018, item 0.036, thing on the floor 0.236 | bowl 0.615 | 12+object on the floor, 12+small object, 12+all five, 12+bowl | 0.067: 21 FP, finds it True |
| s76_t2/lego_1x1_1 | resolution/size | 3 | 3, 1.65 | 0.000 (0.083) | object on the floor 0.000, small object 0.000, clutter 0.000, item 0.000, thing on the floor 0.000 | lego brick 0.000 |  | - |
| s76_t2/schleich_lion_action_figure_3 | threshold | 11 | 31, 1.05 | 0.149 (0.149) | object on the floor 0.203, small object 0.188, clutter 0.016, item 0.042, thing on the floor 0.140 | lion 0.277 | 12+lion | 0.149: 6 FP, finds it True |
| s76_t2/dino_5_4 | threshold | 10 | 51, 1.02 | 0.191 (0.191) | object on the floor 0.169, small object 0.185, clutter 0.019, item 0.030, thing on the floor 0.120 | dinosaur 0.353 | 12+small object, 12+clutter, 12+item, 12+dinosaur | 0.191: 3 FP, finds it True |
| s76_t2/threshold_bead_cereal_bowl_w_6 | prompt, generic | 2 | 55, 1.16 | 0.077 (0.077) | object on the floor 0.544, small object 0.347, clutter 0.018, item 0.091, thing on the floor 0.269 | bowl 0.559 | 12+object on the floor, 12+small object, 12+all five, 12+bowl | 0.077: 14 FP, finds it True |

| Prompt set | Matched threshold | False positives | Found | Gained | Lost |
|---|---|---|---|---|---|
| 12 | 0.19 | 3 | 42 | s76_t2/dino_5_4 |  |
| 12+object on the floor | 0.32 | 3 | 42 | s71_t2/dino_5_4, s71_t2/threshold_bead_cereal_bowl_w_6, s76_t2/threshold_bead_cereal_bowl_w_6 | s96_t1/lacing_sheep_3, s96_t1/mattel_skip_bo_card_game_4 |
| 12+small object | 0.19 | 3 | 44 | s71_t2/threshold_bead_cereal_bowl_w_6, s76_t2/dino_5_4, s76_t2/threshold_bead_cereal_bowl_w_6 |  |
| 12+clutter | 0.19 | 3 | 42 | s76_t2/dino_5_4 |  |
| 12+item | 0.19 | 3 | 42 | s76_t2/dino_5_4 |  |
| 12+thing on the floor | 0.39 | 2 | 30 |  | s70_t1/room_essentials_mug_white_ye_6, s70_t1/turboprop_airplane_with_pilo_0, s71_t2/ballet_flat_0, s71_t2/schleich_lion_action_figure_3, s73_t1/mattel_skip_bo_card_game_4, s73_t1/room_essentials_mug_white_ye_6, s76_t2/ballet_flat_0, s77_t1/mattel_skip_bo_card_game_4, s77_t1/turboprop_airplane_with_pilo_0, s96_t1/lacing_sheep_3, s96_t1/mattel_skip_bo_card_game_4 |
| 12+all five | 0.39 | 3 | 37 | s71_t2/dino_5_4, s71_t2/threshold_bead_cereal_bowl_w_6, s76_t2/threshold_bead_cereal_bowl_w_6 | s70_t1/turboprop_airplane_with_pilo_0, s71_t2/ballet_flat_0, s71_t2/schleich_lion_action_figure_3, s77_t1/mattel_skip_bo_card_game_4, s77_t1/turboprop_airplane_with_pilo_0, s96_t1/lacing_sheep_3, s96_t1/mattel_skip_bo_card_game_4 |
| object on the floor alone | 0.28 | 3 | 33 | s71_t2/dino_5_4, s71_t2/threshold_bead_cereal_bowl_w_6, s76_t2/threshold_bead_cereal_bowl_w_6 | s70_t1/turboprop_airplane_with_pilo_0, s71_t2/nintendo_mario_action_figure_5, s71_t2/schleich_lion_action_figure_3, s71_t2/triceratops_2, s73_t1/lacing_sheep_3, s73_t1/melissa_doug_cart_turtle_blo_1, s73_t1/turboprop_airplane_with_pilo_0, s76_t2/nintendo_mario_action_figure_5, s76_t2/triceratops_2, s77_t1/turboprop_airplane_with_pilo_0, s96_t1/figure_2 |
| small object alone | 0.16 | 3 | 43 | s71_t2/dino_5_4, s71_t2/threshold_bead_cereal_bowl_w_6, s76_t2/dino_5_4, s76_t2/schleich_lion_action_figure_3, s76_t2/threshold_bead_cereal_bowl_w_6 | s76_t2/nintendo_mario_action_figure_5, s76_t2/triceratops_2, s96_t1/mattel_skip_bo_card_game_4 |
| clutter alone | 0.11 | 2 | 3 |  | s70_t1/brick_5, s70_t1/figure_2, s70_t1/lacing_sheep_3, s70_t1/mattel_skip_bo_card_game_4, s70_t1/mens_gold_cup_asv_capetown_p_7, s70_t1/room_essentials_mug_white_ye_6, s70_t1/turboprop_airplane_with_pilo_0, s71_t2/ballet_flat_0, s71_t2/nintendo_mario_action_figure_5, s71_t2/schleich_lion_action_figure_3, s71_t2/triceratops_2, s73_t1/brick_5, s73_t1/figure_2, s73_t1/lacing_sheep_3, s73_t1/mattel_skip_bo_card_game_4, s73_t1/melissa_doug_cart_turtle_blo_1, s73_t1/mens_gold_cup_asv_capetown_p_7, s73_t1/room_essentials_mug_white_ye_6, s73_t1/turboprop_airplane_with_pilo_0, s76_t2/ballet_flat_0, s76_t2/nintendo_mario_action_figure_5, s76_t2/triceratops_2, s76_t2/womens_bluefish_2eye_boat_sh_7, s77_t1/brick_5, s77_t1/figure_2, s77_t1/lacing_sheep_3, s77_t1/mattel_skip_bo_card_game_4, s77_t1/mens_gold_cup_asv_capetown_p_7, s77_t1/room_essentials_mug_white_ye_6, s77_t1/turboprop_airplane_with_pilo_0, s96_t1/brick_5, s96_t1/figure_2, s96_t1/lacing_sheep_3, s96_t1/mattel_skip_bo_card_game_4, s96_t1/melissa_doug_cart_turtle_blo_1, s96_t1/mens_gold_cup_asv_capetown_p_7, s96_t1/room_essentials_mug_white_ye_6, s96_t1/turboprop_airplane_with_pilo_0 |
| item alone | 0.05 | 1 | 36 | s76_t2/threshold_bead_cereal_bowl_w_6 | s71_t2/triceratops_2, s73_t1/melissa_doug_cart_turtle_blo_1, s76_t2/nintendo_mario_action_figure_5, s76_t2/triceratops_2, s96_t1/lacing_sheep_3, s96_t1/mattel_skip_bo_card_game_4 |
| thing on the floor alone | 0.36 | 3 | 5 |  | s70_t1/brick_5, s70_t1/figure_2, s70_t1/lacing_sheep_3, s70_t1/mens_gold_cup_asv_capetown_p_7, s70_t1/room_essentials_mug_white_ye_6, s70_t1/turboprop_airplane_with_pilo_0, s71_t2/ballet_flat_0, s71_t2/nintendo_mario_action_figure_5, s71_t2/schleich_lion_action_figure_3, s71_t2/triceratops_2, s71_t2/womens_bluefish_2eye_boat_sh_7, s73_t1/brick_5, s73_t1/figure_2, s73_t1/lacing_sheep_3, s73_t1/mattel_skip_bo_card_game_4, s73_t1/melissa_doug_cart_turtle_blo_1, s73_t1/room_essentials_mug_white_ye_6, s73_t1/turboprop_airplane_with_pilo_0, s76_t2/ballet_flat_0, s76_t2/nintendo_mario_action_figure_5, s76_t2/triceratops_2, s76_t2/womens_bluefish_2eye_boat_sh_7, s77_t1/brick_5, s77_t1/figure_2, s77_t1/lacing_sheep_3, s77_t1/mattel_skip_bo_card_game_4, s77_t1/melissa_doug_cart_turtle_blo_1, s77_t1/room_essentials_mug_white_ye_6, s77_t1/turboprop_airplane_with_pilo_0, s96_t1/brick_5, s96_t1/figure_2, s96_t1/lacing_sheep_3, s96_t1/mattel_skip_bo_card_game_4, s96_t1/melissa_doug_cart_turtle_blo_1, s96_t1/room_essentials_mug_white_ye_6, s96_t1/turboprop_airplane_with_pilo_0 |
| 12+action figure | 0.19 | 3 | 42 | s76_t2/dino_5_4 |  |
| 12+ballet flat | 0.19 | 3 | 42 | s76_t2/dino_5_4 |  |
| 12+bowl | 0.19 | 3 | 44 | s71_t2/threshold_bead_cereal_bowl_w_6, s76_t2/dino_5_4, s76_t2/threshold_bead_cereal_bowl_w_6 |  |
| 12+box of cards | 0.19 | 3 | 42 | s76_t2/dino_5_4 |  |
| 12+cart | 0.19 | 3 | 42 | s76_t2/dino_5_4 |  |
| 12+dinosaur | 0.19 | 3 | 42 | s76_t2/dino_5_4 |  |
| 12+lacing sheep | 0.19 | 3 | 42 | s76_t2/dino_5_4 |  |
| 12+lego brick | 0.19 | 3 | 42 | s76_t2/dino_5_4 |  |
| 12+lion | 0.19 | 3 | 43 | s76_t2/dino_5_4, s76_t2/schleich_lion_action_figure_3 |  |
| 12+mug | 0.19 | 3 | 42 | s76_t2/dino_5_4 |  |
| 12+sheep | 0.19 | 3 | 42 | s76_t2/dino_5_4 |  |
| 12+toy airplane | 0.19 | 3 | 42 | s76_t2/dino_5_4 |  |
| 12+toy brick | 0.19 | 3 | 42 | s76_t2/dino_5_4 |  |

Best-frame long side (px), found toys (41): [0, 11.1, 16.1, 20.1, 20.1, 20.2, 21.8, 26.3, 26.4, 27.1, 27.8, 28.2, 30.6, 30.9, 31.5, 34.3, 34.7, 36.6, 37.4, 41.8, 43.0, 46.6, 46.7, 49.6, 54.7, 57.4, 62.5, 65.3, 66.5, 67.3, 71.5, 72.0, 75.3, 81.3, 96.5, 124.9, 129.3, 138.4, 152.9, 193.8, 257.0]
missed toys (7): [2.7, 6.0, 31.1, 50.6, 51.7, 54.9, 108.5]

## large (google/owlv2-large-patch14-ensemble, threshold 0.15, false positives over six nights 4)

Reproduction: {'published_found': 42, 'published_fp': 4, 'rerun_found': 42, 'rerun_fp': 4, 'same_found_set': True}

| Miss | Class | Frames in view | Best frame: long side px, range m | 12 prompts on the toy (anywhere in reach) | Generic, on the toy | Own name, on the toy | Found by (matched threshold) | The 12 lowered to its score |
|---|---|---|---|---|---|---|---|---|
| s71_t2/dino_5_4 | threshold | 7 | 109, 0.47 | 0.126 (0.126) | object on the floor 0.508, small object 0.118, clutter 0.017, item 0.024, thing on the floor 0.258 | dinosaur 0.010 | 12+object on the floor, 12+all five | 0.126: 5 FP, finds it True |
| s71_t2/threshold_bead_cereal_bowl_w_6 | prompt, generic | 10 | 52, 1.23 | 0.060 (0.060) | object on the floor 0.400, small object 0.248, clutter 0.022, item 0.081, thing on the floor 0.211 | bowl 0.647 | 12+object on the floor, 12+small object, 12+bowl | 0.059: 25 FP, finds it True |
| s76_t2/lego_1x1_1 | resolution/size | 3 | 3, 1.65 | 0.000 (0.101) | object on the floor 0.000, small object 0.000, clutter 0.000, item 0.000, thing on the floor 0.000 | lego brick 0.000 |  | - |
| s76_t2/threshold_bead_cereal_bowl_w_6 | threshold | 2 | 55, 1.16 | 0.090 (0.090) | object on the floor 0.497, small object 0.286, clutter 0.033, item 0.114, thing on the floor 0.293 | bowl 0.558 | 12+object on the floor, 12+small object, 12+all five, 12+bowl | 0.089: 10 FP, finds it True |
| s96_t1/lacing_sheep_3 | threshold | 2 | 57, 0.66 | 0.088 (0.088) | object on the floor 0.358, small object 0.378, clutter 0.004, item 0.024, thing on the floor 0.128 | lacing sheep 0.017, sheep 0.004 | 12+object on the floor, 12+small object | 0.088: 10 FP, finds it True |
| s96_t1/mattel_skip_bo_card_game_4 | not in view | 0 | 0, 0.00 | 0.000 (0.000) | object on the floor 0.000, small object 0.000, clutter 0.000, item 0.000, thing on the floor 0.000 | card game 0.000, box of cards 0.000 | 12+object on the floor | - |

| Prompt set | Matched threshold | False positives | Found | Gained | Lost |
|---|---|---|---|---|---|
| 12 | 0.14 | 4 | 42 |  |  |
| 12+object on the floor | 0.35 | 4 | 39 | s71_t2/dino_5_4, s71_t2/threshold_bead_cereal_bowl_w_6, s76_t2/threshold_bead_cereal_bowl_w_6, s96_t1/lacing_sheep_3, s96_t1/mattel_skip_bo_card_game_4 | s70_t1/turboprop_airplane_with_pilo_0, s71_t2/lego_1x1_1, s76_t2/dino_5_4, s76_t2/nintendo_mario_action_figure_5, s76_t2/schleich_lion_action_figure_3, s77_t1/mattel_skip_bo_card_game_4, s77_t1/melissa_doug_cart_turtle_blo_1, s77_t1/turboprop_airplane_with_pilo_0 |
| 12+small object | 0.14 | 4 | 45 | s71_t2/threshold_bead_cereal_bowl_w_6, s76_t2/threshold_bead_cereal_bowl_w_6, s96_t1/lacing_sheep_3 |  |
| 12+clutter | 0.14 | 4 | 42 |  |  |
| 12+item | 0.14 | 4 | 42 |  |  |
| 12+thing on the floor | 0.41 | 4 | 26 |  | s70_t1/figure_2, s70_t1/lacing_sheep_3, s70_t1/room_essentials_mug_white_ye_6, s70_t1/turboprop_airplane_with_pilo_0, s71_t2/lego_1x1_1, s71_t2/nintendo_mario_action_figure_5, s71_t2/schleich_lion_action_figure_3, s73_t1/mens_gold_cup_asv_capetown_p_7, s73_t1/turboprop_airplane_with_pilo_0, s76_t2/dino_5_4, s76_t2/nintendo_mario_action_figure_5, s76_t2/schleich_lion_action_figure_3, s77_t1/mattel_skip_bo_card_game_4, s77_t1/melissa_doug_cart_turtle_blo_1, s77_t1/turboprop_airplane_with_pilo_0, s96_t1/turboprop_airplane_with_pilo_0 |
| 12+all five | 0.42 | 4 | 30 | s71_t2/dino_5_4, s76_t2/threshold_bead_cereal_bowl_w_6 | s70_t1/figure_2, s70_t1/room_essentials_mug_white_ye_6, s70_t1/turboprop_airplane_with_pilo_0, s71_t2/lego_1x1_1, s71_t2/nintendo_mario_action_figure_5, s71_t2/schleich_lion_action_figure_3, s73_t1/turboprop_airplane_with_pilo_0, s76_t2/dino_5_4, s76_t2/nintendo_mario_action_figure_5, s76_t2/schleich_lion_action_figure_3, s77_t1/mattel_skip_bo_card_game_4, s77_t1/melissa_doug_cart_turtle_blo_1, s77_t1/turboprop_airplane_with_pilo_0, s96_t1/turboprop_airplane_with_pilo_0 |
| object on the floor alone | 0.36 | 4 | 26 | s71_t2/dino_5_4, s71_t2/threshold_bead_cereal_bowl_w_6, s76_t2/threshold_bead_cereal_bowl_w_6 | s70_t1/mens_gold_cup_asv_capetown_p_7, s70_t1/turboprop_airplane_with_pilo_0, s71_t2/lego_1x1_1, s71_t2/nintendo_mario_action_figure_5, s71_t2/schleich_lion_action_figure_3, s71_t2/triceratops_2, s71_t2/womens_bluefish_2eye_boat_sh_7, s73_t1/brick_5, s73_t1/melissa_doug_cart_turtle_blo_1, s76_t2/dino_5_4, s76_t2/nintendo_mario_action_figure_5, s76_t2/schleich_lion_action_figure_3, s76_t2/triceratops_2, s77_t1/mattel_skip_bo_card_game_4, s77_t1/melissa_doug_cart_turtle_blo_1, s77_t1/room_essentials_mug_white_ye_6, s77_t1/turboprop_airplane_with_pilo_0, s96_t1/room_essentials_mug_white_ye_6, s96_t1/turboprop_airplane_with_pilo_0 |
| small object alone | 0.09 | 3 | 45 | s71_t2/dino_5_4, s71_t2/threshold_bead_cereal_bowl_w_6, s76_t2/threshold_bead_cereal_bowl_w_6, s96_t1/lacing_sheep_3 | s71_t2/lego_1x1_1 |
| clutter alone | 0.12 | 2 | 7 |  | s70_t1/brick_5, s70_t1/figure_2, s70_t1/lacing_sheep_3, s70_t1/mens_gold_cup_asv_capetown_p_7, s70_t1/room_essentials_mug_white_ye_6, s70_t1/turboprop_airplane_with_pilo_0, s71_t2/ballet_flat_0, s71_t2/lego_1x1_1, s71_t2/nintendo_mario_action_figure_5, s71_t2/schleich_lion_action_figure_3, s71_t2/triceratops_2, s71_t2/womens_bluefish_2eye_boat_sh_7, s73_t1/brick_5, s73_t1/figure_2, s73_t1/lacing_sheep_3, s73_t1/mattel_skip_bo_card_game_4, s73_t1/melissa_doug_cart_turtle_blo_1, s73_t1/mens_gold_cup_asv_capetown_p_7, s73_t1/room_essentials_mug_white_ye_6, s73_t1/turboprop_airplane_with_pilo_0, s76_t2/ballet_flat_0, s76_t2/dino_5_4, s76_t2/nintendo_mario_action_figure_5, s76_t2/schleich_lion_action_figure_3, s76_t2/triceratops_2, s76_t2/womens_bluefish_2eye_boat_sh_7, s77_t1/brick_5, s77_t1/figure_2, s77_t1/lacing_sheep_3, s77_t1/mattel_skip_bo_card_game_4, s77_t1/room_essentials_mug_white_ye_6, s77_t1/turboprop_airplane_with_pilo_0, s96_t1/figure_2, s96_t1/room_essentials_mug_white_ye_6, s96_t1/turboprop_airplane_with_pilo_0 |
| item alone | 0.05 | 1 | 38 | s71_t2/threshold_bead_cereal_bowl_w_6, s76_t2/threshold_bead_cereal_bowl_w_6 | s70_t1/mens_gold_cup_asv_capetown_p_7, s71_t2/lego_1x1_1, s71_t2/womens_bluefish_2eye_boat_sh_7, s76_t2/dino_5_4, s76_t2/nintendo_mario_action_figure_5, s77_t1/turboprop_airplane_with_pilo_0 |
| thing on the floor alone | 0.41 | 4 | 8 |  | s70_t1/brick_5, s70_t1/figure_2, s70_t1/lacing_sheep_3, s70_t1/mens_gold_cup_asv_capetown_p_7, s70_t1/room_essentials_mug_white_ye_6, s70_t1/turboprop_airplane_with_pilo_0, s71_t2/ballet_flat_0, s71_t2/lego_1x1_1, s71_t2/nintendo_mario_action_figure_5, s71_t2/schleich_lion_action_figure_3, s71_t2/triceratops_2, s71_t2/womens_bluefish_2eye_boat_sh_7, s73_t1/brick_5, s73_t1/figure_2, s73_t1/lacing_sheep_3, s73_t1/melissa_doug_cart_turtle_blo_1, s73_t1/mens_gold_cup_asv_capetown_p_7, s73_t1/room_essentials_mug_white_ye_6, s73_t1/turboprop_airplane_with_pilo_0, s76_t2/dino_5_4, s76_t2/nintendo_mario_action_figure_5, s76_t2/schleich_lion_action_figure_3, s76_t2/triceratops_2, s76_t2/womens_bluefish_2eye_boat_sh_7, s77_t1/brick_5, s77_t1/figure_2, s77_t1/lacing_sheep_3, s77_t1/mattel_skip_bo_card_game_4, s77_t1/melissa_doug_cart_turtle_blo_1, s77_t1/room_essentials_mug_white_ye_6, s77_t1/turboprop_airplane_with_pilo_0, s96_t1/figure_2, s96_t1/room_essentials_mug_white_ye_6, s96_t1/turboprop_airplane_with_pilo_0 |
| 12+action figure | 0.14 | 4 | 42 |  |  |
| 12+ballet flat | 0.14 | 4 | 42 |  |  |
| 12+bowl | 0.14 | 4 | 44 | s71_t2/threshold_bead_cereal_bowl_w_6, s76_t2/threshold_bead_cereal_bowl_w_6 |  |
| 12+box of cards | 0.14 | 4 | 42 |  |  |
| 12+cart | 0.14 | 4 | 42 |  |  |
| 12+dinosaur | 0.14 | 4 | 42 |  |  |
| 12+lacing sheep | 0.14 | 4 | 42 |  |  |
| 12+lego brick | 0.14 | 4 | 42 |  |  |
| 12+lion | 0.14 | 4 | 42 |  |  |
| 12+mug | 0.14 | 4 | 42 |  |  |
| 12+sheep | 0.14 | 4 | 42 |  |  |
| 12+toy airplane | 0.14 | 4 | 42 |  |  |
| 12+toy brick | 0.14 | 4 | 42 |  |  |

Best-frame long side (px), found toys (42): [6.0, 11.1, 16.1, 20.1, 20.1, 20.2, 21.8, 26.3, 26.4, 27.1, 27.8, 28.2, 30.6, 30.9, 31.1, 31.5, 34.3, 34.7, 36.6, 37.4, 41.8, 43.0, 46.6, 46.7, 49.6, 50.6, 54.7, 62.5, 65.3, 66.5, 67.3, 71.5, 72.0, 75.3, 81.3, 96.5, 124.9, 129.3, 138.4, 152.9, 193.8, 257.0]
missed toys (6): [0, 2.7, 51.7, 54.9, 57.4, 108.5]

