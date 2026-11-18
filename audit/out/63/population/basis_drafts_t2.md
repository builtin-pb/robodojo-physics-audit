# Draft plausible-range basis lines for Lane 3 Step T2: 6 estimated pair friction leaves, 4 unsourced class leaves, and 20 unsourced pair friction leaves (`#59`, `#63`, `#173`, `#175`)

Companion to `audit/out/63/population/basis_drafts.md` (`872e503f…`, pinned by `audit/tools/build_docs_params.py` for T0+T1; left untouched so all T0+T1 class and pair rows stay byte-identical).

Across the `4` `class_values.json` material classes entering in T2 (`fabric_textile`, `food_organic`, `glass`, `rubber_elastomer` -> `21` classes total) and their `78` new class pairs (`145 -> 223` class pairs total), Sections 1 and 2 supply per-value `#173` physical-reasoning `basis` lines for all **`30` new T2 leaves** (`6` `estimated` pair friction leaves + `4` `unsourced` class leaves + `20` `unsourced` pair friction leaves) whose `class_values.json` entries lacked an explicit `basis` string (zero external citations added or invented; `status` is never raised). The centralized `PROPOSED_DECISION_TAGS["Q1"]` tag from `audit/tools/build_t2_population_rows.py` is appended by `audit/tools/build_docs_params.py` in one place so Bo's answers replace `proposed (#59 Q1)` with a single edit.

---

## 1. Six new T2 `estimated` pair friction leaves (`fabric_textile` in G2, `rubber_elastomer` in G1)

1. **`pairs.fabric_textile__pad_hard.mu_static` — `0.35 [0.20, 0.60]`** *(default)*
   - **Basis (physical reasoning + G2 leather/textile anchor):** Hard thermoplastic X5 finger face (`pad_hard`, ABS/PLA analogue) pinching woven textile, plush pile, synthetic/natural leather watch straps, or sneaker uppers (`Rigid/shoe/*`, `Rigid/teddy_bear/00000`, `Rigid/toy/0000{0,1}`, `Rigid/thread/00000`, `Rigid/watch/0000{5,6}`, `Clutter/{bag,cap,jeans}/*`). Flexible yarn loops and leather grain conform moderately against smooth rigid polymer without elastomer bulk hysteresis (`sourcing_G2.md` leather-on-wood `0.30–0.50` proxy): `0.20` covers slick nylon/polyester weave or dusty PU faux leather on hard polymer, `0.35` nominal dry cotton/canvas/leather pinch breakaway, and `0.60` textured plush pile or rubberized sneaker-outsole rim contact.

2. **`pairs.fabric_textile__pad_hard.mu_kinetic` — `0.28 [0.15, 0.50]`** *(default)*
   - **Basis (physical reasoning + G2 static anchor):** Steady in-grasp sliding of woven textile, plush, leather watch bands, or shoe uppers against a hard thermoplastic X5 finger face (`pad_hard`), anchored at `0.80×` static breakaway `0.35 [0.20, 0.60]` (`sourcing_G2.md` §Caveats kinetic-to-static ratio); `[0.15, 0.50]` spans slick synthetic lining/nylon (`0.15`) to textured canvas/leather or outsole edge rub (`0.50`).

3. **`pairs.fabric_textile__table_laminate.mu_static` — `0.35 [0.20, 0.60]`** *(default)*
   - **Basis (physical reasoning + G2 textile anchor):** Woven fabric, plush dolls, wound thread cones, leather-strap watches, and sneakers (`fabric_textile`) resting on a dry smooth melamine-laminate tabletop (`table_laminate`). Compliant fibres on cured melamine resin touch at discrete yarn crowns (`0.20` slick polyester plush or dusty nylon on laminate, `0.35` nominal cotton/canvas/leather breakaway aligned with `sourcing_G2.md`'s leather-on-wood `0.30–0.40` anchor, up to `0.60` when a sneaker's rubberized outsole sits flat on clean laminate).

4. **`pairs.fabric_textile__table_laminate.mu_kinetic` — `0.28 [0.15, 0.50]`** *(default)*
   - **Basis (physical reasoning + G2 static anchor):** Steady sliding of textile, plush, leather, or shoe models pushed across dry melamine laminate (`table_laminate`), set at `0.80×` static breakaway `0.35 [0.20, 0.60]`; `[0.15, 0.50]` spans smooth synthetic plush/woven fabric gliding on melamine (`0.15`) to leather/outsole drag (`0.50`).

5. **`pairs.pad_elastomer__rubber_elastomer.mu_static` — `1.10 [0.60, 1.50]`** *(sensitivity)*
   - **Basis (physical reasoning + G1 rubber self-pair anchor):** Shore A 60–90 silicone/TPU/rubber finger pad (`pad_elastomer`) pinching moulded rubber/TPR balls, erasers, dog chew toys, rubber mallet heads (`Rigid/hammer/00008`), or respirator masks (`rubber_elastomer`). Anchored on the clean dry rubber-on-rubber self-pair (`1.16` in `Engineering ToolBox` and `1.15` in `Engineers Edge`, `sourcing_G1.md`), set to `1.10` nominal with `[0.60, 1.50]` spanning dusty/filled firmer TPU-on-TPR (`0.60`) to clean soft high-adhesion elastomer-on-elastomer contact (`1.50`).

6. **`pairs.pad_hard__rubber_elastomer.mu_static` — `0.80 [0.50, 1.20]`** *(default)*
   - **Basis (physical reasoning + G1 elastomer-on-polymer analogue):** Hard thermoplastic X5 finger face (`pad_hard`, ABS/PLA analogue) pinching compliant moulded rubber/TPR (`rubber_elastomer`: balls, eraser, dog toy, rubber mallet, mask). Symmetric counterpart to compliant-pad-on-rigid-polymer (`sourcing_G1.md` §8): soft rubber deforms over the flat polymer jaw face to generate real contact area well above rigid plastic-on-plastic (`0.40–0.50`), with `0.50` covering dusty/harder filled TPR, `0.80` nominal dry Shore A 60–75 rubber on ABS, and `1.20` soft tacky natural/silicone rubber.

---

## 2. Twenty-four new T2 `unsourced` class and pair leaves (`status: unsourced` preserved)

### 2.1 Four `unsourced` T2 class leaves (`fabric_textile`, `food_organic`, `glass`)

7. **`classes.fabric_textile.restitution` — `0.10 [0.00, 0.30]`** *(unsourced)*
   - **Basis (physical reasoning, no matched source fetched):** Woven textiles, fiberfill plush dolls (`Rigid/teddy_bear/00000`, `Rigid/toy/0000{0,1}`), leather watch bands (`Rigid/watch/0000{5,6}`), and sneakers (`Rigid/shoe/*`) dissipate nearly all low-speed normal impact energy through internal yarn friction and air expulsion: `0.00` covers dead inelastic settling of loose fabric/plush, `0.10` nominal heavily damped bounce, and `0.30` firmer leather-cased or rubber-outsole shoe rebounds.

8. **`classes.food_organic.density_kg_m3` — `800 [300, 1100] kg/m^3`** *(unsourced)*
   - **Basis (physical reasoning, no matched source fetched):** Effective bulk density across solid produce, bakery items, and rigid-modelled food props in T2 (`Rigid/{apple,banana,lemon,orange,peach,pear,strawberry,potato}/*`, `Clutter/{bread,croissant,donut,hamburger}/*`). High-water cellular produce sits near water density (`800–1,100 kg/m^3` depending on internal air spaces and peel/core porosity), while aerated baked goods sit much lower (`300 kg/m^3` lower bound), centred at `800 kg/m^3` nominal.

9. **`classes.food_organic.restitution` — `0.30 [0.10, 0.50]`** *(unsourced)*
   - **Basis (physical reasoning, no matched source fetched):** Whole fruit, vegetables, and baked food items experience viscoelastic parenchymal damping and local bruising at tabletop drop speeds: `0.10` covers soft baked goods or bruised/ripe produce, `0.30` nominal firm produce low-drop rebound, and `0.50` crisp turgid fruit (`apple`, `lemon`) at very low drop heights before tissue yield.

10. **`classes.glass.restitution` — `0.66 [0.40, 0.85]`** *(unsourced)*
    - **Basis (physical reasoning, no matched low-speed source fetched):** Rigid soda-lime and borosilicate glass objects (`Rigid/test_tube/00002`, `Clutter/{glass_cup,magic_potion,wine_glass}/*`) have high elastic modulus and low internal material damping below fracture threshold (`class_values.json` v3.1 downgraded its concrete-drop paper to `unsourced` for `4.2 m/s` conditions mismatch): `[0.40, 0.85]` spans thin-wall hollow glassware flex losses (`0.40`) through `0.66` nominal thick-base glassware to stiff solid-wall glass elastic rebound (`0.85`).

### 2.2 Twenty `unsourced` T2 pair friction leaves (`fabric_textile`, `food_organic`, `glass`, `rubber_elastomer`)

11. **`pairs.fabric_textile__fabric_textile.mu_static` — `0.45 [0.25, 0.70]`** *(unsourced)*
    - **Basis (physical reasoning, no matched source fetched):** Dry textile-on-textile and leather-on-textile breakaway between plush toys, canvas bags, caps, jeans, shoes, and leather watch straps (`classify_objects*`, `general_pickup`, `pack_objects_into_box*`, `*_random`). Intermeshing surface yarns and pile fibres resist breakaway more strongly than fabric on smooth melamine (`0.35 [0.20, 0.60]`): `0.25` covers slick synthetic nylon/satin linings, `0.45` nominal dry cotton/canvas/plush contact, and `0.70` coarse denim/pile or outsole-on-canvas contact.

12. **`pairs.fabric_textile__fabric_textile.mu_kinetic` — `0.35 [0.20, 0.60]`** *(unsourced)*
    - **Basis (physical reasoning, no matched source fetched):** Steady sliding of woven fabric, plush, or leather against another textile surface after yarn crowns disengage (~`0.78×` static `0.45 [0.25, 0.70]`); `[0.20, 0.60]` spans smooth synthetic weave (`0.20`) to coarse canvas/denim/plush drag (`0.60`).

13. **`pairs.fabric_textile__ground.mu_static` — `0.375 [0.20, 0.70]`** *(unsourced)*
    - **Basis (physical reasoning, floor material unidentified):** Hull of `fabric_textile` on smooth hard horizontal surfaces (`table_laminate` `[0.20, 0.60]` and `table_wood` `[0.25, 0.70]`): `0.20` covers slick synthetic fabric on smooth floor tile/vinyl, `0.375` midpoint nominal breakaway, and `0.70` textured canvas/leather/shoe sole on finished flooring.

14. **`pairs.fabric_textile__ground.mu_kinetic` — `0.30 [0.15, 0.50]`** *(unsourced)*
    - **Basis (physical reasoning, floor material unidentified):** Sliding of dropped textile, plush, leather, or shoe items across the unidentified hard room floor, spanning `fabric_textile__table_laminate` (`[0.15, 0.50]`) and `table_wood` kinetic ranges with `0.30` nominal (~`0.80×` static `0.375`).

15. **`pairs.food_organic__food_organic.mu_static` — `0.40 [0.20, 0.80]`** *(unsourced)*
    - **Basis (physical reasoning, no matched source fetched):** Dry-to-waxy fruit cuticles, citrus rind, and baked crusts contacting each other (`classify_objects*`, `general_pickup`, `pack_objects_into_box*`, `*_random`). Natural epicuticular fruit wax (`apple`, `pear`) can be fairly slick (`0.20` lower bound), whereas textured citrus peel, fuzzy peach skin, strawberry achenes, and dry bread crust conform and interlock (`0.40` nominal, up to `0.80` for rough/tacky rind or crust).

16. **`pairs.food_organic__food_organic.mu_kinetic` — `0.30 [0.15, 0.70]`** *(unsourced)*
    - **Basis (physical reasoning, no matched source fetched):** Steady sliding between whole produce or bakery items after breakaway (~`0.75×` static `0.40 [0.20, 0.80]`), spanning smooth waxy apple/banana skin (`0.15`) to textured peel or bread-crust rubbing (`0.70`).

17. **`pairs.food_organic__ground.mu_static` — `0.375 [0.20, 0.70]`** *(unsourced)*
    - **Basis (physical reasoning, floor material unidentified):** Hull of `food_organic` static bands across smooth hard horizontal surfaces (`table_laminate` `[0.20, 0.60]` and `table_wood` `[0.20, 0.70]`), centred at `0.375` nominal for dropped produce or bakery items on hard flooring.

18. **`pairs.food_organic__ground.mu_kinetic` — `0.29 [0.15, 0.60]`** *(unsourced)*
    - **Basis (physical reasoning, floor material unidentified):** Hull of `food_organic` kinetic bands across hard horizontal surfaces (`table_laminate` `[0.15, 0.50]` and `table_wood` `[0.15, 0.60]`), centred at `0.29` nominal (~`0.77×` static `0.375`).

19. **`pairs.food_organic__pad_elastomer.mu_static` — `0.70 [0.40, 1.20]`** *(unsourced, sensitivity)*
    - **Basis (physical reasoning, no matched source fetched):** Compliant Shore A 60–90 elastomer finger pad (`pad_elastomer`) pinching waxy fruit skin, textured citrus peel, or dry bakery crust. Soft elastomer conforms over curved produce skin to generate high real contact area (`0.40` dusty/waxy cuticle on firmer TPU, `0.70` nominal clean dry pinch, `1.20` tacky silicone on smooth fruit skin).

20. **`pairs.food_organic__pad_elastomer.mu_kinetic` — `0.60 [0.30, 1.00]`** *(unsourced, sensitivity)*
    - **Basis (physical reasoning, no matched source fetched):** In-grasp slip of whole produce or bakery items against an elastomer finger pad (~`0.85×` static `0.70 [0.40, 1.20]`), spanning waxy/dusty peel (`0.30`) to high-conformity silicone drag (`1.00`).

21. **`pairs.food_organic__pad_hard.mu_static` — `0.35 [0.15, 0.60]`** *(unsourced, default)*
    - **Basis (physical reasoning, no matched source fetched):** Hard thermoplastic X5 finger face (`pad_hard`) pinching whole fruit, vegetables, or baked props (`general_pickup`, `classify_objects*`, `pack_objects_into_box*`). Smooth waxy fruit cuticle on rigid polymer can slip easily (`0.15` lower bound), with `0.35` nominal dry cuticle/rind breakaway and `0.60` compliant citrus peel or soft berry/bread conformity against the flat jaw.

22. **`pairs.food_organic__pad_hard.mu_kinetic` — `0.28 [0.10, 0.50]`** *(unsourced, default)*
    - **Basis (physical reasoning, no matched source fetched):** Steady in-grasp sliding of produce or baked items against a hard thermoplastic jaw (`0.80×` static `0.35 [0.15, 0.60]`), from slick waxy fruit skin (`0.10`) to textured peel/crust (`0.50`).

23. **`pairs.food_organic__table_laminate.mu_static` — `0.35 [0.20, 0.60]`** *(unsourced, default)*
    - **Basis (physical reasoning, no matched source fetched):** Whole produce and dry bakery items resting on smooth melamine laminate (`table_laminate`): `0.20` covers slick waxy apple/pear skin or dry bread crust on melamine, `0.35` nominal dry produce breakaway, and `0.60` compliant citrus/peach skin under self-weight.

24. **`pairs.food_organic__table_laminate.mu_kinetic` — `0.28 [0.15, 0.50]`** *(unsourced, default)*
    - **Basis (physical reasoning, no matched source fetched):** Steady sliding of produce or bakery items across dry melamine laminate (`0.80×` static `0.35 [0.20, 0.60]`), spanning slick waxy cuticle (`0.15`) to textured rind drag (`0.50`).

25. **`pairs.food_organic__table_wood.mu_static` — `0.40 [0.20, 0.70]`** *(unsourced, sensitivity)*
    - **Basis (physical reasoning, no matched source fetched):** Produce and bakery items resting on finished/lacquered wood (`table_wood`), slightly above smooth melamine (`0.35 [0.20, 0.60]`) due to wood finish micro-texture: `[0.20, 0.70]` spans dry crust/waxy peel (`0.20`) to compliant fruit rind on satin wood (`0.70`).

26. **`pairs.food_organic__table_wood.mu_kinetic` — `0.30 [0.15, 0.60]`** *(unsourced, sensitivity)*
    - **Basis (physical reasoning, no matched source fetched):** Steady sliding of produce or bakery items across finished wood (`0.75×` static `0.40 [0.20, 0.70]`), spanning `0.15` to `0.60`.

27. **`pairs.glass__ground.mu_static` — `0.30 [0.20, 0.50]`** *(unsourced)*
    - **Basis (physical reasoning, floor material unidentified):** Hull of smooth soda-lime/borosilicate glassware (`glass`) on hard horizontal surfaces (`table_laminate` and `table_wood`), spanning clean smooth glass on hard floor finish (`0.20` to `0.50`, nominal `0.30`).

28. **`pairs.glass__ground.mu_kinetic` — `0.25 [0.15, 0.40]`** *(unsourced)*
    - **Basis (physical reasoning, floor material unidentified):** Sliding of smooth rigid glassware across the hard room floor (~`0.83×` static `0.30 [0.20, 0.50]`), spanning `0.15` to `0.40`.

29. **`pairs.ground__rubber_elastomer.mu_static` — `0.65 [0.35, 1.10]`** *(unsourced)*
    - **Basis (physical reasoning, floor material unidentified):** Hull of compliant moulded rubber/TPR (`rubber_elastomer`) static breakaway across hard smooth horizontal surfaces (`table_laminate` `[0.35, 1.00]` and `table_wood` `[0.40, 1.10]`), centred at `0.65` nominal.

30. **`pairs.ground__rubber_elastomer.mu_kinetic` — `0.55 [0.30, 0.90]`** *(unsourced)*
    - **Basis (physical reasoning, floor material unidentified):** Hull of `rubber_elastomer` kinetic sliding across hard smooth horizontal surfaces (`table_laminate` `[0.30, 0.85]` and `table_wood` `[0.35, 0.90]`), centred at `0.55` nominal (~`0.85×` static `0.65`).

---

## 3. Contingency band (foam__softwood — non-formable across all 49 in-scope configs under proposed classes)

Under the proposed T2 `mixed_unclear` assignments (`Clutter/chinese_pavillion/00000 -> hardwood`, `Clutter/house/0000{0,1,2} -> engineered_wood / glazed_ceramic / abs_toy_plastic`, `Clutter/piano/00000 -> abs_device_housing`, `Clutter/plane/00000 -> abs_toy_plastic`), no task in T0+T1+T2 spawns both `foam` and `softwood`, so `foam__softwood` remains unformed (`0` episodes). If Bo's side assigns any `Clutter/*` miniature in `arrange_largest_number_random` (which spawns `Geometry/cube_cushion/00001` in `foam`) to `softwood`, use the following analogue band matching `foam__hardwood` (`class_values.json`):

- Contingency `foam__softwood.mu_static` (`unsourced`, `0.65 [0.35, 1.00]`): Open- or closed-cell PU/EVA foam pad (`Geometry/cube_cushion/00001`) against painted, lacquered, or sanded softwood miniatures; matches `foam__hardwood` (`0.65 [0.35, 1.00]`, `basis_drafts.md` §1.4) because cured paint/lacquer or fine wood grain governs micro-conformity identically on softwood and hardwood.
- Contingency `foam__softwood.mu_kinetic` (`unsourced`, `0.55 [0.30, 0.90]`): Steady sliding of painted/finished softwood across compliant PU/EVA foam at ~`0.85×` static `0.65 [0.35, 1.00]`, matching `foam__hardwood` (`0.55 [0.30, 0.90]`, `basis_drafts.md` §1.4).
