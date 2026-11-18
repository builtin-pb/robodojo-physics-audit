# Caption refinement of RoboDojo object models (revision 1127bda9)

Files: `classes.json` (same schema plus `caption_name`, `caption_material`, `evidence`, `caption_status`, `prior_class`, `prior_confidence`, `prior_identity`, flag `caption_mismatch`), `descriptions/` (mirrored raw captions; `*.404` marker files record packages with none).

## Coverage

- 724 models; description.json exists for 411 (340 with a structured caption of name/material/size lists, 71 with a one-line description only); 313 have none (404) and are unchanged.
- Class list unchanged: no class was clearly missing (carbon-fibre and gold-plated items fit the nearest metal class; a rubber-mallet and a gas mask fit `rubber_elastomer`).
- Method: caption material compared with the current class family (earliest keyword per phrase). Agreement raised confidence one step; a consistent conflict lowered it one step; otherwise the class was kept. Then every category was read by hand and overrides recorded with a reason in `evidence`. Bbox density, declared mass and category outranked the caption wherever they disagreed.

## Confidence

| | low | medium | high |
|---|---|---|---|
| before | 236 | 278 | 210 |
| after | 188 | 283 | 253 |

- Low-confidence models resolved (low to medium or high): **70 of 236**. Still low: 166 (106 of them have no description at all). Newly low from medium/high: 22 (caption contradicted category; see mismatches). Total raised 134, lowered 43.

## Counts per class

| class | before | after |
|---|---|---|
| pet_plastic | 13 | 11 |
| abs_toy_plastic | 221 | 206 |
| abs_device_housing | 101 | 96 |
| polyolefin_plastic | 23 | 24 |
| softwood | 65 | 64 |
| hardwood | 5 | 7 |
| engineered_wood | 5 | 5 |
| glazed_ceramic | 54 | 60 |
| glass | 10 | 10 |
| steel | 54 | 61 |
| aluminium | 12 | 23 |
| rubber_elastomer | 6 | 8 |
| paper_cardboard | 55 | 55 |
| foam | 4 | 4 |
| fabric_textile | 34 | 35 |
| food_organic | 25 | 24 |
| liquid | 4 | 4 |
| mixed_unclear | 33 | 27 |

42 models changed class; 76 changed identity text (mostly the generic ones: figures, tokens, basket, toy).

Class moves: abs_toy_plastic to aluminium x7; abs_toy_plastic to glazed_ceramic x5; abs_device_housing to aluminium x4; abs_toy_plastic to steel x4; mixed_unclear to steel x3; mixed_unclear to glazed_ceramic x2; softwood to abs_toy_plastic x2; abs_toy_plastic to fabric_textile x2; fabric_textile to rubber_elastomer x1; fabric_textile to abs_toy_plastic x1; abs_device_housing to steel x1; mixed_unclear to hardwood x1; aluminium to abs_toy_plastic x1; pet_plastic to steel x1; pet_plastic to aluminium x1; steel to rubber_elastomer x1; glazed_ceramic to hardwood x1; food_organic to paper_cardboard x1; paper_cardboard to fabric_textile x1; steel to polyolefin_plastic x1; abs_toy_plastic to softwood x1.

## Caption mismatches (79, flagged `caption_mismatch`)

Caption ignored or down-weighted because it contradicts bbox/category. Grouped by category:

- `ice_cream` x1: "Dairy Queen Blizzard" - caption metal can contradicts ice cream
- `tissue` x1: "Nestea tea carton" - caption iced-tea carton contradicts tissue box; paperboard either way
- `checkerboard` x1: "tic-tac-toe board" - caption glass/ceramic tic-tac-toe board conflicts with paper board (0.08 kg over 22x22x1.6 cm)
- `cube_cushion` x2: "disc" - caption rigid plastic disc/plate contradicts cushion category; bbox thickness ~0
- `cup_holder` x2: "smartphone" - caption smartphone/flashlight of 18x18x35 cm does not match a holder or coaster
- `monitor` x2: "smartwatch" - caption "smartwatch" (brushed aluminum) is unrelated to a monitor of this size; caption ignored, class from category/bbox/mass
- `mousemat` x1: "smartphone" - caption "smartphone" (brushed aluminum) is unrelated to a mousemat of this size; caption ignored, class from category/bbox/mass
- `number_card` x9: "coaster" - caption coaster/matte plastic vs numbered card (paper); 10x10x0.5 cm fits both
- `t_cushion` x1: "T-shaped bracket" - caption rigid plastic bracket contradicts cushion category; bbox thickness ~0
- `vase` x1: "suction cup" - caption suction cup/rubber contradicts vase category
- `vertical_coin_stand` x1: "business card holder" - caption business-card holder, wood/painted metal; scale (3 cm) and material both unreliable
- `vertical_laptop_storage_rack` x1: "business card holder" - caption identical business-card-holder text; not a laptop rack
- `block` x24: "smartwatch" - caption "smartwatch" (brushed aluminum) is unrelated to a block of this size; caption ignored, class from category/bbox/mass
- `bottle` x4: "finger cap" - caption finger cap/rubber contradicts 17 cm bottle
- `bread` x1: "smartphone" - caption "smartphone" (brushed aluminum) is unrelated to a bread of this size; caption ignored, class from category/bbox/mass
- `chessman` x2: "washer" - caption washer/plus sign, polished metal; 0.1 kg over 5x5x1 cm implies dense metal; contradicts chess-piece category
- `coin` x1: "smartphone" - caption "smartphone" (brushed aluminum) is unrelated to a coin of this size; caption ignored, class from category/bbox/mass
- `division` x1: "black disc" - caption names button/cap/washer, material alternates plastic/metal across tokens; not a math symbol
- `equal` x1: "pause button" - caption names button/cap/washer, material alternates plastic/metal across tokens; not a math symbol
- `keyboard` x3: "smartphone" - caption "smartphone" (brushed aluminum) is unrelated to a keyboard of this size; caption ignored, class from category/bbox/mass
- `mallet_stand` x1: "chopstick rest" - caption chopstick rest/ceramic contradicts wooden mallet-stand category
- `matryoshka_dolls` x10: "rubber duck" - caption duck/penguin plastic contradicts matryoshka category (same nesting-size series though)
- `minus` x1: "flathead screw cap" - caption names button/cap/washer, material alternates plastic/metal across tokens; not a math symbol
- `multiplication` x1: "button" - caption names button/cap/washer, material alternates plastic/metal across tokens; not a math symbol
- `plus` x1: "button" - caption names button/cap/washer, material alternates plastic/metal across tokens; not a math symbol
- `puppet` x1: "frog rattle" - caption plastic frog rattle contradicts hand puppet category
- `triangular_prism` x1: "triangular tile marker" - caption tile spacer ceramic contradicts a 55 cm long prism
- `wooden_toy` x3: "lion cub" - caption ceramic/plastic contradicts wooden-toy category; category kept

Other noisy captions that did not change a class (not flagged): car (painted-metal die-cast vs 0.45 kg default mass), toothpaste (aluminium tube), paper_ball/00001 (ceramic), pepper/00002 (plastic), mug/00005 (Lego head), tissue/00003 (wood veneer).

## Remaining unresolvable (166 low)

- 106: no description.json (kept prior bbox/mass/category guess)
- 34: caption contradicts category/bbox, no usable evidence
- 26: caption gives no material or conflicting materials

Largest low groups: number x50, block x29, garage x8, cup x5, toaster x4, racket x4, basket x4, stackingblocks x4, toy x3, cube x3, SpringButton x2, ball x2.

Caveats: captions are auto-generated; material changes are plausible readings, not truth. Several declared masses stay spurious (cubes 2.5 cm at 0.35 kg, house 1200 kg, robot dog 18 kg) and those models stay flagged. Captioned models are 411 of 724, so the 313 uncaptioned ones (106 of them low-confidence: 61 Rigid number/mahjong/stacking-type rows, 33 Clutter, 7 Articulation, 5 Geometry) could not be refined.
