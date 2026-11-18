# Draft plausible-range basis lines: 27 unsourced pair bands (54 leaves), 6 class restitutions, and 25 estimated pair friction leaves (#59, #173, #175, #200)

Scope: every `unsourced` leaf in `audit/out/63/class_values.json` that carries no physical basis (`27` class pairs × `2` friction leaves = **54 leaves**, including `paper_cardboard__paper_cardboard` whose `class_values.json` `why` was only `"No fetched paper-on-paper value"` / `"same"`, plus **6 class `restitution` leaves** = **60 `unsourced` leaves**), plus all **25 `estimated` pair friction leaves** in `docs/params/pairs.json` (`17` new T1 leaves in G2 and `8` pre-existing G1/G2 leaves on `main`) whose `class_values.json` rows carried `sources: []` without a per-value basis = **85 leaves** total. Nothing in `audit/out/63/class_values.json` is modified here; `audit/tools/build_docs_params.py` attaches these per-value `basis` entries into `docs/params/{classes,pairs}.json` and fails if any `estimated` leaf lacks one.

Sourcing rule (common rule 4, #173): **zero external citations are added or invented**. Every basis below is **physical reasoning** (plus cross-checks against neighbouring `class_values.json` entries already audited in `audit/out/63/citation_check.md`), stating what real objects in RoboDojo's population span and why the `[min, max]` band covers that span while keeping `status = unsourced`.

---

## 1. Unsourced pair friction bands (27 pairs, 54 leaves)

### 1.1 Hard thermoplastics (`abs_device_housing`, `abs_toy_plastic`) on tabletops and elastomer pads (12 leaves)

1. **`pairs.abs_device_housing__table_laminate.mu_static` — `0.30 [0.15, 0.50]`** *(default; 47/49 in-scope configs, 14/14 T1)*
   - **Basis (physical reasoning, no matched source fetched):** Smooth or satin ABS/PC electronics shells (X5 wrist-camera housing, mice, keyboards, alarm clock, earbuds, tape measure) resting on a dry hard melamine-laminate tabletop (`table_laminate`). Low end `0.15` covers dusty or glossy low-adhesion ABS/PC or PTFE-like mouse feet on hard cured melamine resin; nominal `0.30` sits at typical dry rigid-polymer-on-laminate breakaway; high end `0.50` covers clean textured or matte ABS/PC housings before rubberized feet dominate (`engineered_wood__table_laminate` is `0.30 [0.15, 0.50]` in `class_values.json`).

2. **`pairs.abs_device_housing__table_laminate.mu_kinetic` — `0.25 [0.12, 0.40]`** *(default; 47/49 configs, 14/14 T1)*
   - **Basis (physical reasoning, no matched source fetched):** Steady sliding of rigid ABS/PC device shells across dry melamine laminate after breakaway. Kinetic friction of dry unlubricated hard polymers on hard thermoset resin runs ~15–20% below static as micro-asperity junctions shear without ploughing; `[0.12, 0.40]` spans dusty/low-friction polymer glides (`0.12`) to clean matte ABS shells (`0.40`).

3. **`pairs.abs_device_housing__table_wood.mu_static` — `0.35 [0.20, 0.55]`** *(sensitivity; 47/49 configs, 14/14 T1)*
   - **Basis (physical reasoning, no matched source fetched):** Rigid ABS/PC device housings resting on a finished/lacquered hardwood tabletop (`table_wood`, simulation shader `Mahogany_Planks.mdl`). Polyurethane or nitrocellulose wood finishes are slightly more compliant and adhesive against rigid thermoplastics than hard melamine resin, shifting `[0.15, 0.50]` upward by `0.05` to `[0.20, 0.55]` (consistent with `abs_device_housing__hardwood` `[0.15, 0.65]` in `class_values.json`).

4. **`pairs.abs_device_housing__table_wood.mu_kinetic` — `0.30 [0.15, 0.45]`** *(sensitivity; 47/49 configs, 14/14 T1)*
   - **Basis (physical reasoning, no matched source fetched):** Steady sliding of rigid ABS/PC housings on lacquered wood. Kept ~15% below the static band `[0.20, 0.55]`; `[0.15, 0.45]` spans slick waxed wood lacquer (`0.15`) to clean satin varnish (`0.45`).

5. **`pairs.abs_device_housing__pad_elastomer.mu_static` — `0.70 [0.40, 1.20]`** *(sensitivity; 49/49 configs, 14/14 T1)*
   - **Basis (physical reasoning, no matched source fetched):** Shore A 60–90 silicone/TPU/rubber gripper pad (`pad_elastomer`) pinching smooth or textured ABS/PC device shells (mice, earbuds, keyboards, camera housing). Compliant elastomer drapes over polymer surface roughness and develops adhesive shear above rigid polymer-on-polymer contacts: `0.40` covers dusty or hard-durometer TPU on textured matte ABS, `0.70` represents clean medium-durometer elastomer grip, and `1.20` covers clean soft silicone on glossy PC/ABS (`abs_device_housing__rubber_elastomer` has the same `1.20` ceiling in `class_values.json`).

6. **`pairs.abs_device_housing__pad_elastomer.mu_kinetic` — `0.60 [0.30, 1.00]`** *(sensitivity; 49/49 configs, 14/14 T1)*
   - **Basis (physical reasoning, no matched source fetched):** Slip of an ABS/PC device housing under a Shore A 60–90 elastomer finger pad. Viscoelastic hysteresis and interfacial shear during slip keep kinetic friction high (`0.60` nominal) while ~15% below static breakaway; `[0.30, 1.00]` spans dusty hard TPU (`0.30`) to clean high-grip silicone (`1.00`).

7. **`pairs.abs_toy_plastic__table_laminate.mu_static` — `0.30 [0.15, 0.50]`** *(default; 23/49 configs, 7/14 T1 incl. `swap_T`, `push_T`, `arrange_largest_number`, `solve_equation`)*
   - **Basis (physical reasoning, no matched source fetched):** Moulded ABS/PS/PVC blocks, T-blocks, math/digit tokens, and figurines resting on a dry smooth melamine-laminate tabletop. Both surfaces are hard glassy/semi-gloss polymers with small real contact area: `0.15` covers dusty or glossy injection-moulded PS/ABS on hard melamine, `0.30` is typical dry rigid-plastic breakaway, and `0.50` covers clean satin ABS or plasticized PVC tokens (below `abs_toy_plastic__abs_toy_plastic` ceiling `0.80` because cured melamine thermoset resists polymer ploughing).

8. **`pairs.abs_toy_plastic__table_laminate.mu_kinetic` — `0.25 [0.12, 0.40]`** *(default; 23/49 configs, 7/14 T1)*
   - **Basis (physical reasoning, no matched source fetched):** Sliding of moulded ABS/PS/PVC blocks, T-blocks, and tokens pushed or set down across melamine laminate. Rigid glassy polymer sliding drops ~15–20% below static breakaway; `[0.12, 0.40]` spans dusty low-friction moulded parts (`0.12`) to clean satin ABS/PVC (`0.40`).

9. **`pairs.abs_toy_plastic__table_wood.mu_static` — `0.35 [0.20, 0.55]`** *(sensitivity; 23/49 configs, 7/14 T1)*
   - **Basis (physical reasoning, no matched source fetched):** Moulded ABS/PS/PVC toys and tokens resting on a lacquered/finished wood tabletop. Finished wood coatings exhibit slightly higher adhesion against glossy thermoplastics than melamine laminate, spanning `0.20` (hard waxed finish) to `0.55` (clean polyurethane lacquer against glossy PVC/ABS; inside `abs_toy_plastic__hardwood` `[0.15, 0.70]`).

10. **`pairs.abs_toy_plastic__table_wood.mu_kinetic` — `0.30 [0.15, 0.45]`** *(sensitivity; 23/49 configs, 7/14 T1)*
    - **Basis (physical reasoning, no matched source fetched):** Steady sliding of ABS/PS/PVC blocks and tokens on finished wood, ~15% below static `[0.20, 0.55]`; `[0.15, 0.45]` covers waxed wood (`0.15`) to clean lacquered wood (`0.45`).

11. **`pairs.abs_toy_plastic__pad_elastomer.mu_static` — `0.70 [0.40, 1.20]`** *(sensitivity; 29/49 configs, 8/14 T1)*
    - **Basis (physical reasoning, no matched source fetched):** Shore A 60–90 elastomer gripper pad pinching moulded ABS/PS/PVC blocks, T-blocks, bricks, and digit tokens. Matches `abs_device_housing__pad_elastomer` because the polymer counterface family is the same rigid styrenic/vinyl plastic: `0.40` covers dusty/matte toy plastic on firmer TPU, `0.70` nominal dry elastomer pinch, and `1.20` clean soft silicone on smooth glossy ABS/PVC.

12. **`pairs.abs_toy_plastic__pad_elastomer.mu_kinetic` — `0.60 [0.30, 1.00]`** *(sensitivity; 29/49 configs, 8/14 T1)*
    - **Basis (physical reasoning, no matched source fetched):** In-grasp slip of moulded ABS/PS/PVC parts against an elastomer finger pad. Spanning `[0.30, 1.00]` (~15% below static `[0.40, 1.20]`) covers firmer/dusty TPU (`0.30`) through tacky silicone (`1.00`).

---

### 1.2 Packaging polymers (`pet_plastic`, `polyolefin_plastic`) on tabletops and elastomer pads (12 leaves)

13. **`pairs.pet_plastic__table_laminate.mu_static` — `0.25 [0.15, 0.45]`** *(default; 10/49 configs, `put_bottles_into_dustbin`)*
    - **Basis (physical reasoning, no matched source fetched):** Thin-wall blow-moulded PET beverage bottles (glossy bare PET petaloid bases or BOPP/PET shrink sleeves) resting on dry melamine laminate. Semi-crystalline biaxially oriented PET is harder and lower-adhesion than amorphous ABS (`0.30`), giving `0.25` nominal; `0.15` covers dusty or curved low-area petaloid feet on melamine, and `0.45` covers clean flat-contact or printed polymer shrink-sleeve rims.

14. **`pairs.pet_plastic__table_laminate.mu_kinetic` — `0.20 [0.12, 0.35]`** *(default; 10/49 configs)*
    - **Basis (physical reasoning, no matched source fetched):** Sliding of upright or lying PET bottles across melamine laminate during table grazes or pushes. Rigid semi-crystalline PET slides smoothly once static junctions break, spanning `0.12` (dusty petaloid foot) to `0.35` (clean sleeve/body contact).

15. **`pairs.pet_plastic__table_wood.mu_static` — `0.30 [0.20, 0.50]`** *(sensitivity; 10/49 configs)*
    - **Basis (physical reasoning, no matched source fetched):** Blow-moulded PET bottles on a lacquered/finished wood tabletop. Shifted `+0.05` above `pet_plastic__table_laminate` (`[0.15, 0.45]`) for slightly softer wood lacquer adhesion; spans `0.20` (waxed finish) to `0.50` (clean glossy lacquer, inside `pet_plastic__hardwood` `[0.15, 0.58]`).

16. **`pairs.pet_plastic__table_wood.mu_kinetic` — `0.25 [0.15, 0.40]`** *(sensitivity; 10/49 configs)*
    - **Basis (physical reasoning, no matched source fetched):** Sliding of PET bottles on finished wood, ~15–20% below static `[0.20, 0.50]`; spans `0.15` to `0.40` across waxed to clean lacquered wood finishes.

17. **`pairs.pad_elastomer__pet_plastic.mu_static` — `0.70 [0.40, 1.20]`** *(sensitivity; 10/49 configs)*
    - **Basis (physical reasoning, no matched source fetched):** Shore A 60–90 elastomer finger pad gripping smooth glossy PET bottles or shrink-sleeved beverage walls. Smooth glossy PET allows high real contact area against a compliant elastomer (`1.20` upper bound for clean silicone on smooth PET), while condensation/dust or stiffer TPU drops breakaway to `0.40` (`0.70` nominal).

18. **`pairs.pad_elastomer__pet_plastic.mu_kinetic` — `0.60 [0.30, 1.00]`** *(sensitivity; 10/49 configs)*
    - **Basis (physical reasoning, no matched source fetched):** Slip of a PET bottle wall or label sleeve against an elastomer finger pad under load; `[0.30, 1.00]` sits ~15% below static `[0.40, 1.20]` across stiffer/dusty TPU (`0.30`) to clean soft elastomer (`1.00`).

19. **`pairs.polyolefin_plastic__table_laminate.mu_static` — `0.20 [0.10, 0.35]`** *(default; 14/49 configs, 2/14 T1: `cover_blocks` PP cup, `sweep_blocks*` PP dustpan/brush)*
    - **Basis (physical reasoning, no matched source fetched):** Moulded PP/HDPE/LDPE cups, dustpans, brushes, and containers on smooth melamine laminate. Non-polar waxy polyolefins have the lowest surface energy among RoboDojo's rigid plastics (consistent with `polyolefin_plastic__polyolefin_plastic` floor `0.10` in `class_values.json`): `0.10` covers waxy/textured HDPE or PP rims on hard melamine, `0.20` nominal moulded PP cup/dustpan breakaway, and `0.35` clean smooth LDPE/PP at higher real contact area.

20. **`pairs.polyolefin_plastic__table_laminate.mu_kinetic` — `0.15 [0.08, 0.30]`** *(default; 14/49 configs, 2/14 T1)*
    - **Basis (physical reasoning, no matched source fetched):** Steady sliding of waxy moulded PP/PE cups and dustpans pushed across melamine laminate (directly exercised when `cover_blocks` sets a PP cup down or `sweep_blocks` pushes cubes into the dustpan). Non-polar polyolefins exhibit low adhesive shear in sliding, spanning `0.08` (waxy HDPE/PP rim) to `0.30` (smooth LDPE/PP).

21. **`pairs.polyolefin_plastic__table_wood.mu_static` — `0.25 [0.12, 0.40]`** *(sensitivity; 14/49 configs, 2/14 T1)*
    - **Basis (physical reasoning, no matched source fetched):** Moulded PP/PE containers and tools on finished/lacquered wood. Slightly above melamine (`0.20 [0.10, 0.35]`) due to lacquer compliance, while remaining below polar plastics (`PET` `0.30`, `ABS` `0.35`) because of polyolefin's waxy low-surface-energy skin; `[0.12, 0.40]` spans textured HDPE on waxed wood to smooth PP/LDPE on clean lacquer.

22. **`pairs.polyolefin_plastic__table_wood.mu_kinetic` — `0.20 [0.10, 0.35]`** *(sensitivity; 14/49 configs, 2/14 T1)*
    - **Basis (physical reasoning, no matched source fetched):** Sliding of moulded PP/PE parts across finished wood, ~15–20% below static `[0.12, 0.40]`; spans `0.10` to `0.35`.

23. **`pairs.pad_elastomer__polyolefin_plastic.mu_static` — `0.50 [0.30, 1.00]`** *(sensitivity; 20/49 configs, 3/14 T1)*
    - **Basis (physical reasoning, no matched source fetched):** Shore A 60–90 elastomer finger pad gripping waxy PP/HDPE cups, brushes, dustpans, or bins. Lower surface energy of PP/PE reduces adhesive elastomer grip relative to PET/ABS (`0.70 [0.40, 1.20]`): `0.30` covers textured waxy HDPE against firmer TPU, `0.50` nominal moulded PP in an elastomer jaw, and `1.00` clean soft silicone conforming to smooth PP.

24. **`pairs.pad_elastomer__polyolefin_plastic.mu_kinetic` — `0.45 [0.25, 0.90]`** *(sensitivity; 20/49 configs, 3/14 T1)*
    - **Basis (physical reasoning, no matched source fetched):** Sliding of PP/HDPE parts against an elastomer gripper pad during slip; `[0.25, 0.90]` sits ~10–15% below static `[0.30, 1.00]` from waxy HDPE on TPU (`0.25`) to smooth PP on silicone (`0.90`).

---

### 1.3 Metals (`aluminium`, `steel`) and brittle silicates (`glass`, `glazed_ceramic`) on tabletops and hard pads (14 leaves)

25. **`pairs.aluminium__table_laminate.mu_static` — `0.35 [0.20, 0.50]`** *(default; 47/49 configs, 14/14 T1: X5 arm links, camera/coin stands, cans, cubes)*
    - **Basis (physical reasoning, no matched source fetched):** Anodised or painted/machined aluminium parts (X5 arm links, stands, cans) against hard melamine laminate. Anodised `Al2O3` and painted aluminium behave as hard smooth oxide/polymer-coated surfaces without bare-metal galling (`aluminium__aluminium` `0.90`): `0.20` covers smooth anodised/painted aluminium on dusty melamine, `0.35` nominal dry contact, and `0.50` clean bare or brushed aluminium edge contact (aligned with `aluminium__table_wood` `[0.20, 0.60]` in `class_values.json`).

26. **`pairs.aluminium__table_laminate.mu_kinetic` — `0.30 [0.15, 0.45]`** *(default; 47/49 configs, 14/14 T1)*
    - **Basis (physical reasoning, no matched source fetched):** Sliding of anodised/machined aluminium parts on melamine laminate, ~15% below static `[0.20, 0.50]` without bare-metal seizure; spans `0.15` (smooth anodised) to `0.45` (clean brushed alloy).

27. **`pairs.steel__table_laminate.mu_static` — `0.35 [0.20, 0.50]`** *(default; 20/49 configs, 4/14 T1: coins, keys, hand tools, coated steel bottle `00022`)*
    - **Basis (physical reasoning, no matched source fetched):** Plated, stainless, brass, or powder-coated steel objects (coins, keys, wrenches, pliers, hammers, insulated bottle `00022`) on hard melamine laminate. Hard thermoset melamine prevents metal-to-metal junction growth while resisting ploughing under light manipulation loads: `0.20` covers polished chrome/nickel plating on smooth melamine, `0.35` nominal dry steel/coated-metal on laminate, and `0.50` rougher machined or brushed tool edges (matching `steel__table_wood` `[0.20, 0.60]` at the lower end).

28. **`pairs.steel__table_laminate.mu_kinetic` — `0.30 [0.15, 0.45]`** *(default; 20/49 configs, 4/14 T1)*
    - **Basis (physical reasoning, no matched source fetched):** Steady sliding of steel/brass coins, keys, and tools across melamine laminate; ~15% below static `[0.20, 0.50]`, spanning `0.15` (polished plated coin/key) to `0.45` (unpolished steel tool).

29. **`pairs.glass__table_laminate.mu_static` — `0.30 [0.20, 0.50]`** *(default; 9/49 T2/T3 configs: wine bottles, beakers, test tubes, cups)*
    - **Basis (physical reasoning, no matched source fetched):** Soda-lime or borosilicate glass vessels resting on dry melamine laminate in normal room air (adsorbed moisture/organic film, not nascent clean glass-on-glass `0.80`). Smooth fire-polished glass against hard cured melamine spans `0.20` (slightly dusty/humid glass base) to `0.50` (clean dry glass rim on smooth laminate), centred at `0.30`.

30. **`pairs.glass__table_laminate.mu_kinetic` — `0.25 [0.15, 0.40]`** *(default; 9/49 configs)*
    - **Basis (physical reasoning, no matched source fetched):** Sliding of ordinary room-exposed glass vessels on melamine laminate without surface scoring; ~15–20% below static `[0.20, 0.50]`, spanning `0.15` to `0.40`.

31. **`pairs.glass__table_wood.mu_static` — `0.30 [0.20, 0.50]`** *(sensitivity; 9/49 configs)*
    - **Basis (physical reasoning, no matched source fetched):** Glass vessels resting on lacquered/finished wood (`table_wood`). Smooth hard glass against cured wood lacquer spans the same room-air dry contact regime `[0.20, 0.50]` as melamine laminate (and sits inside `glass__hardwood` `[0.20, 0.60]` in `class_values.json`).

32. **`pairs.glass__table_wood.mu_kinetic` — `0.25 [0.15, 0.40]`** *(sensitivity; 9/49 configs)*
    - **Basis (physical reasoning, no matched source fetched):** Sliding of smooth glass vessels across lacquered wood; ~15–20% below static `[0.20, 0.50]`, spanning `0.15` (waxed wood) to `0.40` (clean lacquer).

33. **`pairs.glazed_ceramic__pad_hard.mu_static` — `0.30 [0.15, 0.50]`** *(default; 16/49 configs, 2/14 T1: mugs, bowls, piggy bank, ceramic figurine)*
    - **Basis (physical reasoning, no matched source fetched):** Hard thermoplastic X5 finger face (`pad_hard`, ABS/PLA analogue) pinching vitreous glazed porcelain or stoneware mugs, bowls, and vessels. Hard polymer against glossy silicate glaze has small real contact area (like `abs_toy_plastic__glazed_ceramic` `[0.15, 0.70]` capped for smooth moulded pads): `0.15` covers slightly moist/dusty glossy glaze, `0.30` nominal dry hard-pad pinch, and `0.50` unglazed foot-ring or matte stoneware contact.

34. **`pairs.glazed_ceramic__pad_hard.mu_kinetic` — `0.25 [0.12, 0.45]`** *(default; 16/49 configs, 2/14 T1)*
    - **Basis (physical reasoning, no matched source fetched):** Slip of a glazed ceramic mug or bowl in the hard thermoplastic X5 jaw; ~15–20% below static `[0.15, 0.50]`, spanning `0.12` (glossy glaze) to `0.45` (matte stoneware or unglazed foot).

35. **`pairs.glazed_ceramic__table_laminate.mu_static` — `0.30 [0.15, 0.50]`** *(default; 13/49 configs, 1/14 T1)*
    - **Basis (physical reasoning, no matched source fetched):** Ceramic mugs, bowls, and figurines resting on melamine laminate. Real ceramic tableware contacts the table either on a glossy glazed body (`~0.15–0.30`) or on an unglazed vitrified foot ring (`~0.35–0.50`), so `[0.15, 0.50]` centred at `0.30` covers both contact geometries.

36. **`pairs.glazed_ceramic__table_laminate.mu_kinetic` — `0.25 [0.12, 0.45]`** *(default; 13/49 configs, 1/14 T1)*
    - **Basis (physical reasoning, no matched source fetched):** Sliding of glazed ceramic vessels (including unglazed foot rings) across melamine laminate; ~15–20% below static `[0.15, 0.50]`, spanning `0.12` to `0.45`.

37. **`pairs.glazed_ceramic__table_wood.mu_static` — `0.35 [0.20, 0.50]`** *(sensitivity; 13/49 configs, 1/14 T1)*
    - **Basis (physical reasoning, no matched source fetched):** Glazed porcelain/stoneware vessels (and unglazed foot rings) on lacquered wood. Slightly higher low-end breakaway (`0.20` vs `0.15`) than melamine because an unglazed ceramic foot ring bites lightly into softer wood varnish; `[0.20, 0.50]` sits inside `glazed_ceramic__hardwood` `[0.15, 0.58]`.

38. **`pairs.glazed_ceramic__table_wood.mu_kinetic` — `0.30 [0.15, 0.45]`** *(sensitivity; 13/49 configs, 1/14 T1)*
    - **Basis (physical reasoning, no matched source fetched):** Sliding of glazed ceramic vessels across finished wood; ~15% below static `[0.20, 0.50]`, spanning `0.15` (glossy glaze on waxed wood) to `0.45` (unglazed foot ring on lacquer).

---

### 1.4 Compliant polymers (`foam`, `rubber_elastomer`) and fibrous cellulose (`paper_cardboard`) on pads, tabletops, and self (16 leaves)

39. **`pairs.foam__foam.mu_static` — `0.80 [0.40, 1.30]`** *(all scenarios; 0/49 configs currently form dynamic foam–foam, carried as class-table self-pair)*
    - **Basis (physical reasoning, no matched source fetched):** Open- or closed-cell polyurethane/EVA foam pads and stuffed cushions pressed against each other. Cellular asperities interlock and compliant ribs drape under normal load, giving high apparent breakaway friction: `0.40` covers smooth closed-cell EVA skin or fabric-faced mousemat tops, `0.80` nominal cellular/EVA contact, and `1.30` tacky open-cell PU foam interlocking.

40. **`pairs.foam__foam.mu_kinetic` — `0.70 [0.30, 1.20]`** *(all scenarios; 0/49 configs)*
    - **Basis (physical reasoning, no matched source fetched):** Continuous sliding of two PU/EVA foam surfaces; cellular deformation hysteresis sustains high drag (`0.70` nominal), spanning `0.30` (smooth fabric-backed mousemat skin) to `1.20` (open-cell PU foam shear).

41. **`pairs.foam__pad_hard.mu_static` — `0.80 [0.40, 1.30]`** *(default; 5/49 configs, 3/14 T1: X5 hard finger pad touching static `cube_cushion` or `mousemat`)*
    - **Basis (physical reasoning, no matched source fetched):** Hard thermoplastic X5 finger face pressing against static EVA/PU foam cushions or mousemats (`arrange_largest_number`, `solve_equation`, `organize_table`). Even though the finger is rigid, the foam counterface conforms around the pad edges and develops viscoelastic/adhesive grip: `0.40` covers smooth cloth-topped mousemats, `0.80` bare closed-cell EVA cushion, and `1.30` soft high-tack PU foam.

42. **`pairs.foam__pad_hard.mu_kinetic` — `0.70 [0.30, 1.20]`** *(default; 5/49 configs, 3/14 T1)*
    - **Basis (physical reasoning, no matched source fetched):** Sliding of a hard thermoplastic finger tip across a static EVA/PU cushion or mousemat; ploughing/hysteresis in the foam keeps kinetic friction close to static across `[0.30, 1.20]` (`0.70` nominal).

43. **`pairs.foam__pad_elastomer.mu_static` — `0.90 [0.40, 1.30]`** *(sensitivity; 5/49 configs, 3/14 T1)*
    - **Basis (physical reasoning, no matched source fetched):** Shore A 60–90 elastomer finger pad pressing against an EVA/PU foam cushion or mousemat. Both surfaces are compliant polymers, raising nominal breakaway to `0.90` (above `foam__pad_hard` `0.80`), while `[0.40, 1.30]` spans cloth-topped low-tack mousemats (`0.40`) to tacky elastomer-on-EVA/PU (`1.30`).

44. **`pairs.foam__pad_elastomer.mu_kinetic` — `0.80 [0.30, 1.20]`** *(sensitivity; 5/49 configs, 3/14 T1)*
    - **Basis (physical reasoning, no matched source fetched):** Sliding of an elastomer finger pad across an EVA/PU cushion or mousemat; dual-elastomer/foam hysteresis sustains `0.80` nominal within `[0.30, 1.20]`.

45. **`pairs.foam__table_laminate.mu_static` — `0.60 [0.40, 1.30]`** *(default; 0/49 configs — all `foam` models in scope are static `Geometry`)*
    - **Basis (physical reasoning, no matched source fetched):** PU/EVA foam pads, cushions, or rubber-foam mousemat bases resting on smooth melamine laminate. Anti-slip EVA/rubber-foam bases are specifically formulated to grip smooth desks (`0.60` nominal, up to `1.30` for tacky closed-cell elastomer foam), while dusty or low-density stiff PU foam drops to `0.40`.

46. **`pairs.foam__table_laminate.mu_kinetic` — `0.50 [0.30, 1.20]`** *(default; 0/49 configs)*
    - **Basis (physical reasoning, no matched source fetched):** Sliding of PU/EVA foam or anti-slip pad bases on melamine laminate; spans `0.30` (stiff/dusty open-cell foam) to `1.20` (high-damping anti-slip foam), centred at `0.50`.

47. **`pairs.foam__table_wood.mu_static` — `0.70 [0.40, 1.30]`** *(sensitivity; 0/49 configs)*
    - **Basis (physical reasoning, no matched source fetched):** PU/EVA foam pads and anti-slip cushion bases on lacquered wood. Nominal `0.70` is slightly above melamine (`0.60`) for clean wood lacquer adhesion, spanning `[0.40, 1.30]` from dusty stiff foam to tacky anti-slip EVA/rubber foam.

48. **`pairs.foam__table_wood.mu_kinetic` — `0.60 [0.30, 1.20]`** *(sensitivity; 0/49 configs)*
    - **Basis (physical reasoning, no matched source fetched):** Sliding of PU/EVA foam across lacquered wood; viscoelastic drag spans `[0.30, 1.20]` (`0.60` nominal).

49. **`pairs.rubber_elastomer__table_laminate.mu_static` — `0.60 [0.35, 1.00]`** *(default; 10/49 T2/T3 configs: rubber balls, eraser, dog toy, conveyor belt)*
    - **Basis (physical reasoning, no matched source fetched):** Shore A 50–90 rubber/TPR/PVC elastomer objects (balls, erasers, dog toys, conveyor belts) resting on dry melamine laminate. Compliant elastomer conforms to smooth laminate for high adhesive friction (`0.60` nominal, up to `1.00` for clean soft rubber), while chalky/dusty TPR erasers or harder PVC elastomers drop breakaway to `0.35`.

50. **`pairs.rubber_elastomer__table_laminate.mu_kinetic` — `0.50 [0.30, 0.80]`** *(default; 10/49 configs)*
    - **Basis (physical reasoning, no matched source fetched):** Sliding of rubber/TPR/PVC elastomer parts across melamine laminate; hysteretic + adhesive shear keeps kinetic friction at `0.50` nominal within `[0.30, 0.80]` (dusty/hard TPR `0.30` to clean rubber `0.80`).

51. **`pairs.rubber_elastomer__table_wood.mu_static` — `0.70 [0.40, 1.10]`** *(sensitivity; 10/49 configs)*
    - **Basis (physical reasoning, no matched source fetched):** Rubber/TPR/PVC elastomer balls, erasers, and toys on lacquered/finished wood. Shifted `+0.05` to `+0.10` above melamine (`0.60 [0.35, 1.00]`) for higher elastomer adhesion on cured wood finishes (matching `hardwood__pad_elastomer` `0.70 [0.40, 1.10]` in `class_values.json`).

52. **`pairs.rubber_elastomer__table_wood.mu_kinetic` — `0.60 [0.35, 0.90]`** *(sensitivity; 10/49 configs)*
    - **Basis (physical reasoning, no matched source fetched):** Sliding of rubber/TPR/PVC elastomer objects on finished wood; ~15% below static `[0.40, 1.10]`, spanning `0.35` (dusty/hard TPR) to `0.90` (clean soft rubber).

53. **`pairs.paper_cardboard__paper_cardboard.mu_static` — `0.40 [0.20, 0.70]`** *(all scenarios; 14/49 configs, 1/14 T1: `sweep_blocks_random` crumpled paper balls)*
    - **Basis (physical reasoning, no matched source fetched):** Dry cellulose-on-cellulose contact between crumpled office paper balls (`Rigid/paper_ball/00000..00002` in `sweep_blocks_random`), cards, and paperboard cartons. Two fibrous cellulose surfaces interlock more strongly than paper on smooth melamine resin (`paper_cardboard__table_laminate` `0.30 [0.15, 0.50]`): `0.20` covers clay-coated or calendered glossy cardstock and laminated pouches, `0.40` nominal dry uncoated 80 gsm bond paper or paperboard breakaway, and `0.70` rough unbleached Kraft board or interlocking crease ridges on crumpled paper balls.

54. **`pairs.paper_cardboard__paper_cardboard.mu_kinetic` — `0.30 [0.15, 0.60]`** *(all scenarios; 14/49 configs, 1/14 T1: `sweep_blocks_random`)*
    - **Basis (physical reasoning, no matched source fetched):** Steady sliding of dry paper, cardstock, or crumpled paper balls against another paper/cardboard surface after breakaway, ~15–25% below static `[0.20, 0.70]` once surface cellulose fibres and shallow creases disengage; `[0.15, 0.60]` spans slick coated cardstock (`0.15`) to rough Kraft board and creased paper facets (`0.60`).

---

## 2. Unsourced class restitution bands without basis text (6 classes, 6 leaves)

Note on regime: RoboDojo's PhysX scene sets `bounce_threshold_velocity = 0.5 m/s` (`env/environment/base_env.py:27`), so restitution is **dormant** during gentle grasps, pushes, and table set-downs (`< 0.5 m/s`, ~13 mm free-fall) and **active** only on drops into receptacles, knocked-off objects, or conveyor falls. Under `restitution_combine_mode = "average"` against `table_laminate` (`0.10 [0.05, 0.40]`), a class unary `e_c` realises `(e_c + 0.10) / 2` on the tabletop.

1. **`classes.abs_device_housing.restitution` — `0.40 [0.20, 0.60]`** *(49/49 configs, 14/14 T1)*
   - **Basis (physical reasoning, no matched <1 m/s source fetched):** Hollow thin-wall ABS/PC consumer-electronics shells containing loose or screw-mounted PCBs, switches, and batteries (mice, keyboards, earbuds, alarm clocks, X5 wrist camera). Structural shell flexing and internal component rattling dissipate much more impact energy than a solid polymer block (`abs_toy_plastic` `0.60`): `0.20` covers multi-part keyboards/clocks that rattle and flex inelastically on drop, `0.40` nominal compact mouse/earbud shell rebound (realising `0.25` against the `0.10` laminate table), and `0.60` small stiff thick-walled earbud pods.

2. **`classes.abs_toy_plastic.restitution` — `0.60 [0.40, 0.80]`** *(29/49 configs, 8/14 T1)*
   - **Basis (physical reasoning, no matched <1 m/s source fetched):** Solid or thick-walled glassy/semi-rigid moulded ABS, PS, and PVC blocks, bricks, tokens, and figurines at low manipulation drop speeds (`0.5–3 m/s`). Unfilled rigid glassy polymers stay mostly elastic below yield in compact geometries (`0.60` nominal, realising `0.35` on the `0.10` table), with `0.40` covering hollow/thin-wall figurines that lose energy in local shell bending and `0.80` compact solid cubes/tokens rebounding against another hard block.

3. **`classes.aluminium.restitution` — `0.50 [0.30, 0.80]`** *(49/49 configs, 14/14 T1)*
   - **Basis (physical reasoning, no matched <1 m/s source fetched):** Wrought/cast aluminium alloy links, stands, cubes, and thin-wall cans. Aluminium alloys have lower yield strength than hardened steel (`steel` `0.60 [0.30, 0.90]` in `class_values.json`), so local elasto-plastic indentation at contact corners and flexural vibration in thin-wall cans or extruded stands dissipate moderate impact energy: `0.30` covers thin-wall hollow cans and corner-denting impacts, `0.50` nominal machined/extruded alloy parts, and `0.80` low-speed elastic impacts of solid rounded aluminium bodies.

4. **`classes.foam.restitution` — `0.10 [0.02, 0.40]`** *(5/49 configs, 3/14 T1)*
   - **Basis (physical reasoning, no matched <1 m/s source fetched):** Soft polyurethane, EVA, and rubber-foam cushions and mousemats. Pneumatic air expulsion in open-cell PU foam and viscoelastic hysteresis in damped EVA/memory-foam absorb nearly all low-speed impact energy (`0.02`–`0.10`, realising `0.06`–`0.10` against the table), while denser closed-cell resilient EVA pads can rebound modestly up to `0.40`.

5. **`classes.glazed_ceramic.restitution` — `0.50 [0.30, 0.70]`** *(16/49 configs, 2/14 T1)*
   - **Basis (physical reasoning, no matched <1 m/s source fetched):** Glazed porcelain and stoneware mugs, bowls, piggy banks, and figurines below fracture thresholds. Fired ceramic is stiff and brittle like `glass` (`0.66 [0.40, 0.85]` in `class_values.json`), but hollow cup/bowl walls convert normal impact momentum into flexural bell-mode ringing rather than rigid-body rebound: `0.30` covers thin open rims and hollow piggy banks, `0.50` nominal thick stoneware/mug bases (realising `0.30` on the `0.10` table), and `0.70` compact thick-walled porcelain.

6. **`classes.rubber_elastomer.restitution` — `0.60 [0.30, 0.85]`** *(12/49 T2/T3 configs)*
   - **Basis (physical reasoning, no matched <1 m/s source fetched):** Molded rubber, TPR, and plasticized PVC elastomers (balls, erasers, dog toys, mallet heads, conveyor belts). Elastomer rebound spans a wide viscoelastic range set by crosslink density and filler damping: `0.30` covers highly damped filled PVC/TPR erasers and conveyor belts, `0.60` general-purpose Shore A 60–80 moulded TPR/rubber toys, and `0.85` high-resilience natural/polybutadiene rubber balls.

---

## 3. Estimated pair friction leaves without per-value basis (25 leaves: 17 new T1 in G2 + 8 pre-existing on `main`)

### 3.1 Seventeen new T1 `estimated` friction leaves (`engineered_wood`, `paper_cardboard`, `softwood` in G2)

1. **`pairs.engineered_wood__engineered_wood.mu_kinetic` — `0.28 [0.15, 0.45]`** *(default)*
   - **Basis (physical reasoning + G2 static anchor):** Steady sliding of melamine-faced or veneered MDF/particleboard panels (`Geometry/drawer/00000` in `organize_table`) against another engineered-wood surface. Anchored at `0.80×` the pair's static breakaway `0.35 [0.20, 0.55]` (`Engineering ToolBox` bare dry wood-on-wood `0.25–0.50` proxy in `mu_static`); `[0.15, 0.45]` spans slick cured melamine foil (`0.15`) to unfinished MDF/plywood edge contact (`0.45`).

2. **`pairs.engineered_wood__pad_elastomer.mu_static` — `0.65 [0.40, 1.00]`** *(sensitivity)*
   - **Basis (physical reasoning):** Shore A 60–90 silicone/TPU/rubber finger pad (`pad_elastomer`) pinching melamine-faced or lacquered MDF/particleboard (`engineered_wood`). Smooth low-porosity melamine/veneer skins develop slightly less elastomer ploughing/adhesion than finished solid wood (`hardwood__pad_elastomer` `0.70 [0.40, 1.10]`): `0.40` covers dusty firmer TPU on slick melamine foil, `0.65` nominal clean elastomer pinch, and `1.00` soft silicone conforming to smooth veneer.

3. **`pairs.engineered_wood__pad_elastomer.mu_kinetic` — `0.55 [0.30, 0.90]`** *(sensitivity)*
   - **Basis (physical reasoning):** In-grasp slip of engineered-wood panels/drawers against a Shore A 60–90 elastomer finger pad. Set ~15% below static breakaway `0.65 [0.40, 1.00]` as viscoelastic asperity junctions shear continuously; `[0.30, 0.90]` spans dusty harder TPU (`0.30`) to clean soft silicone (`0.90`).

4. **`pairs.engineered_wood__pad_hard.mu_kinetic` — `0.25 [0.12, 0.45]`** *(default)*
   - **Basis (physical reasoning + G2 static anchor):** Sliding of engineered wood (MDF/particleboard/plywood) against a hard thermoplastic X5 finger face (`pad_hard`). Anchored at ~`0.83×` the pair's static band `0.30 [0.15, 0.58]` (`arXiv:1405.3049` dry wood-on-PMMA inclined-plane static `0.31–0.58` proxy in `mu_static`, extended down to `0.15` for smooth melamine foil); `[0.12, 0.45]` spans slick melamine film (`0.12`) to bare sanded MDF (`0.45`).

5. **`pairs.engineered_wood__table_laminate.mu_static` — `0.30 [0.15, 0.50]`** *(default)*
   - **Basis (physical reasoning):** Melamine-faced or veneered MDF/particleboard (`Geometry/drawer/00000`) resting on a dry hard melamine-laminate tabletop (`table_laminate`). Two hard cured resin/veneered board faces touch over small real contact area without ploughing: `0.15` covers smooth dusty HPL/melamine foil on laminate, `0.30` nominal dry melamine/veneer breakaway (below bare `Wood Clean and Dry` `0.25–0.50` midpoint `0.35` on `table_wood`), and `0.50` clean unfinished MDF base edges.

6. **`pairs.engineered_wood__table_laminate.mu_kinetic` — `0.25 [0.12, 0.45]`** *(default)*
   - **Basis (physical reasoning):** Steady sliding of engineered-wood bases across dry melamine laminate after breakaway, ~15–20% below static `0.30 [0.15, 0.50]`; `[0.12, 0.45]` spans smooth melamine foil glides (`0.12`) to raw MDF bottom edges (`0.45`).

7. **`pairs.engineered_wood__table_wood.mu_kinetic` — `0.28 [0.15, 0.45]`** *(sensitivity)*
   - **Basis (physical reasoning + G2 static anchor):** Steady sliding of engineered wood across a lacquered/finished wood tabletop (`table_wood`). Anchored at `0.80×` the pair's static band `0.35 [0.20, 0.55]` (`Engineering ToolBox` dry wood-on-wood `0.25–0.50` static proxy in `mu_static`); `[0.15, 0.45]` spans waxed wood finish against melamine foil (`0.15`) to satin lacquer against wood veneer (`0.45`).

8. **`pairs.pad_elastomer__softwood.mu_static` — `0.70 [0.40, 1.10]`** *(sensitivity)*
   - **Basis (physical reasoning):** Shore A 60–90 silicone/TPU/rubber finger pad (`pad_elastomer`) pinching painted, sealed, or sanded softwood Jenga planks and blocks (`Rigid/block/00000..00019` in `build_tower`, `stack_blocks`). Compliant elastomer conforms to fine wood grain or cured paint lacquer: `0.40` covers dusty firmer TPU on sanded/waxed pine, `0.70` nominal clean elastomer grip, and `1.10` soft tacky silicone on smooth lacquered wood.

9. **`pairs.pad_elastomer__softwood.mu_kinetic` — `0.60 [0.30, 1.00]`** *(sensitivity)*
   - **Basis (physical reasoning):** Slip of painted or sanded softwood blocks against an elastomer finger pad under pinch load. Kept ~15% below static `0.70 [0.40, 1.10]`; `[0.30, 1.00]` spans dusty firmer TPU (`0.30`) to clean high-hysteresis silicone (`1.00`).

10. **`pairs.pad_hard__paper_cardboard.mu_static` — `0.30 [0.15, 0.50]`** *(default)*
    - **Basis (physical reasoning):** Hard thermoplastic X5 finger face (`pad_hard`, ABS/PLA analogue) pinching dry crumpled office paper (`Rigid/paper_ball/00000..00002` in `sweep_blocks_random`) or paperboard. Cellulose fibres against smooth rigid polymer exhibit moderate dry adhesion without interlock unless creased edges catch: `0.15` covers clay-coated glossy cardstock on smooth polymer, `0.30` nominal uncoated 80 gsm bond paper / Kraft board on hard plastic, and `0.50` rough fibrous Kraft or crease-edge contact (below `pad_elastomer__paper_cardboard` `0.68 [0.50, 0.90]` sourced from rubber-on-cardboard).

11. **`pairs.pad_hard__paper_cardboard.mu_kinetic` — `0.25 [0.12, 0.45]`** *(default)*
    - **Basis (physical reasoning):** Sliding of dry paper/cardboard against a hard thermoplastic finger face after breakaway, ~15–20% below static `0.30 [0.15, 0.50]`; `[0.12, 0.45]` spans slick coated paperboard (`0.12`) to rough uncoated paper folds (`0.45`).

12. **`pairs.pad_hard__softwood.mu_kinetic` — `0.28 [0.15, 0.45]`** *(default)*
    - **Basis (physical reasoning + G2 static anchor):** Sliding of sanded or painted softwood blocks (`Rigid/block/*`) against a hard thermoplastic finger pad (`pad_hard`). Anchored at `0.80×` the pair's static band `0.35 [0.20, 0.58]` (`arXiv:1405.3049` dry wood-on-PMMA static `0.31–0.58` proxy in `mu_static`, widened down to `0.20` for smooth lacquer); `[0.15, 0.45]` spans glossy painted wood (`0.15`) to bare sanded softwood (`0.45`).

13. **`pairs.paper_cardboard__table_laminate.mu_static` — `0.30 [0.15, 0.50]`** *(default)*
    - **Basis (physical reasoning):** Dry crumpled paper balls (`Rigid/paper_ball/00000..00002` in `sweep_blocks_random`) and paperboard resting on a smooth hard melamine-laminate tabletop (`table_laminate`). Uncoated cellulose facets on cured melamine resin touch at discrete paper folds under millinewton-to-newton loads: `0.15` covers glossy/calendered paper on dusty laminate, `0.30` nominal dry 80 gsm office paper / carton breakaway on melamine, and `0.50` rough fibrous Kraft board on clean laminate.

14. **`pairs.paper_cardboard__table_laminate.mu_kinetic` — `0.25 [0.12, 0.45]`** *(default)*
    - **Basis (physical reasoning):** Steady sliding of crumpled paper balls and paperboard swept or pushed across dry melamine laminate, ~15–20% below static `0.30 [0.15, 0.50]`; `[0.12, 0.45]` spans glossy coated stock (`0.12`) to rough fibrous paperboard (`0.45`).

15. **`pairs.paper_cardboard__table_wood.mu_static` — `0.35 [0.20, 0.60]`** *(sensitivity)*
    - **Basis (physical reasoning):** Dry paper balls and paperboard resting on a finished/lacquered wood tabletop (`table_wood`). Wood varnish/lacquer and residual grain micro-texture offer slightly higher breakaway resistance against fibrous cellulose than hard melamine laminate (`0.30 [0.15, 0.50]`): `0.20` covers glossy coated stock on waxed wood, `0.35` nominal dry bond paper / cardboard on lacquered wood, and `0.60` rough corrugated/Kraft board on satin wood finish (inside `hardwood__paper_cardboard` `[0.15, 0.62]`).

16. **`pairs.paper_cardboard__table_wood.mu_kinetic` — `0.28 [0.15, 0.50]`** *(sensitivity)*
    - **Basis (physical reasoning):** Steady sliding of dry paper and cardboard across finished wood (`table_wood`), set at `0.80×` static `0.35 [0.20, 0.60]`; `[0.15, 0.50]` spans slick coated paper on waxed wood (`0.15`) to rough Kraft board on satin lacquer (`0.50`).

17. **`pairs.softwood__table_laminate.mu_kinetic` — `0.28 [0.15, 0.45]`** *(default)*
    - **Basis (physical reasoning + G2 static anchor):** Steady sliding of painted or sanded softwood planks and blocks (`Rigid/block/*`) across dry melamine laminate (`table_laminate`). Anchored at `0.80×` the pair's static band `0.35 [0.20, 0.50]` (`Engineering ToolBox` dry wood `0.25–0.50` proxy in `mu_static`, extended down to `0.20` for smooth laminate); `[0.15, 0.45]` spans slick painted blocks on melamine (`0.15`) to bare end-grain softwood (`0.45`).

### 3.2 Eight pre-existing `estimated` friction leaves on `main` (`abs_device_housing`, `abs_toy_plastic`, `hardwood`, `polyolefin_plastic` in G1/G2)

18. **`pairs.abs_device_housing__abs_device_housing.mu_static` — `0.40 [0.25, 0.70]`** *(default)*
    - **Basis (physical reasoning + G1 polymer anchor):** Rigid satin/matte ABS/PC electronics enclosures (keyboards, mice, earbuds, alarm clock, wrist-camera housing) in dry polymer-on-polymer contact. Anchored slightly below smooth glossy `abs_toy_plastic__abs_toy_plastic` (`0.50 [0.30, 0.80]`, `Engineering ToolBox` PS-on-PS `0.50` and `Professional Plastics` ABS `0.5` proxy in `sourcing_G1.md`) because textured matte PC/ABS device shells reduce real adhesive contact area: `[0.25, 0.70]` spans textured/dusty PC-ABS (`0.25`) to clean smooth glossy ABS (`0.70`).

19. **`pairs.abs_device_housing__pad_hard.mu_static` — `0.40 [0.25, 0.65]`** *(default)*
    - **Basis (physical reasoning + G1 polymer self-pair analogue):** Hard thermoplastic X5 finger face (`pad_hard`, ABS/PLA analogue) pinching rigid ABS/PC device housings (`sourcing_G1.md` §8 polymer self-pair pad analogue). Matches `abs_device_housing__abs_device_housing` nominal `0.40`, with `[0.25, 0.65]` spanning textured matte ABS/PC shells (`0.25`) to smooth glossy polymer housings (`0.65`).

20. **`pairs.abs_toy_plastic__pad_hard.mu_static` — `0.45 [0.30, 0.70]`** *(default)*
    - **Basis (physical reasoning + G1 polymer self-pair analogue):** Hard thermoplastic X5 finger face (`pad_hard`, ABS/PLA analogue) pinching moulded ABS/PS/PVC blocks, T-blocks, bricks, and tokens (`swap_T`, `push_T`, `stack_blocks_by_language`). Anchored from the `abs_toy_plastic__abs_toy_plastic` hard-polymer self-pair (`0.50 [0.30, 0.80]`, `Engineering ToolBox` PS-on-PS `0.50` proxy in `sourcing_G1.md`), trimmed to `0.45 [0.30, 0.70]` for flat moulded jaw pads against smooth block faces.

21. **`pairs.hardwood__pad_elastomer.mu_static` — `0.70 [0.40, 1.10]`** *(sensitivity)*
    - **Basis (physical reasoning):** Shore A 60–90 silicone/TPU/rubber finger pad (`pad_elastomer`) pinching painted or lacquered hardwood blocks and mallets (`cover_blocks` cubes, `play_Xylophone` mallet). Compliant elastomer conforms to smooth cured wood paint/varnish: `0.40` covers dusty firmer TPU on waxed hardwood, `0.70` nominal clean elastomer pinch, and `1.10` soft tacky silicone on glossy painted hardwood.

22. **`pairs.hardwood__pad_elastomer.mu_kinetic` — `0.60 [0.30, 1.00]`** *(sensitivity)*
    - **Basis (physical reasoning):** In-grasp slip of painted or finished hardwood blocks against an elastomer finger pad (`cover_blocks` sensitivity). Kept ~15% below static breakaway `0.70 [0.40, 1.10]`; `[0.30, 1.00]` spans dusty firmer TPU (`0.30`) to clean soft silicone (`1.00`).

23. **`pairs.hardwood__pad_hard.mu_kinetic` — `0.28 [0.15, 0.45]`** *(default)*
    - **Basis (physical reasoning + G2 static anchor):** Sliding of painted or finished hardwood blocks (`cover_blocks` cubes, `play_Xylophone` mallet) against a hard thermoplastic X5 finger face (`pad_hard`). Anchored at `0.80×` the pair's static band `0.35 [0.20, 0.58]` (`arXiv:1405.3049` dry wood-on-PMMA static `0.31–0.58` proxy in `mu_static`, widened down to `0.20` for smooth paint); `[0.15, 0.45]` spans glossy painted hardwood (`0.15`) to bare finished oak/beech (`0.45`).

24. **`pairs.hardwood__table_laminate.mu_kinetic` — `0.28 [0.15, 0.45]`** *(default)*
    - **Basis (physical reasoning + G2 static anchor):** Steady sliding of painted or finished hardwood blocks (`cover_blocks` cubes, `play_Xylophone` mallet stand) across dry melamine laminate (`table_laminate`). Anchored at `0.80×` the pair's static band `0.35 [0.20, 0.50]` (`Engineering ToolBox` dry wood `0.25–0.50` proxy in `mu_static`); `[0.15, 0.45]` spans slick painted cube faces on melamine (`0.15`) to bare hardwood (`0.45`).

25. **`pairs.pad_hard__polyolefin_plastic.mu_static` — `0.20 [0.10, 0.35]`** *(default)*
    - **Basis (`put_bottles_into_dustbin.json` + G1 polyolefin anchor):** Estimated band if untextured bottles (00005/50/55/59/60) or moulded PP cups/dustpans/brushes (`cover_blocks`, `sweep_blocks*`) are PP/HDPE against hard thermoplastic X5 jaw (aligned with `#73` v2 `[0.10, 0.35]`, anchored on `Engineering ToolBox` PE-on-PE `0.20` and `Professional Plastics` PP `0.10–0.30` in `sourcing_G1.md`; smooth PP may reach ~`0.40`).

