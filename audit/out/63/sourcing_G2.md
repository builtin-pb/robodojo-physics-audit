# G2 sources and caveats (softwood, hardwood, engineered_wood, paper_cardboard, fabric_textile, food_organic)

Counts over 72 values: 13 sourced, 41 estimated, 18 unsourced. Each class has density and restitution; each of the 30 pairs has static and kinetic.

## Sources actually fetched
1. EngineeringToolbox friction table, https://www.engineeringtoolbox.com/friction-coefficients-d_778.html. It gives wood/wood, oak/oak, leather/wood, rubber/cardboard and rubber/rubber. It is a secondary compilation with no speed, load or surface-finish data. Single values are not labelled static or kinetic; I read two-value rows as static then kinetic, which is the page layout and not stated.
2. EngineeringToolbox wood density, https://www.engineeringtoolbox.com/wood-density-d_40.html (seasoned dry wood).
3. EngineeringToolbox solids density, https://www.engineeringtoolbox.com/density-solids-d_1265.html (paper, cardboard, plywood, leather, cotton).
4. SwissKrono MDF datasheet, https://www.swisskrono.com/pl-en/products/interiors/mdf-boards/technical-data/ (EN 323 density 710 to 780 kg/m3, MDF MR FRONT 800).
5. Wood-particle DEM calibration paper, https://arxiv.org/pdf/1405.3049. It was read from the saved PDF with pdftotext because WebFetch could not parse it. It gives wood-wood friction 0.33 to 0.43 and wood-on-methacrylate friction 0.31 to 0.58, both static by inclined plane. It also gives particle-on-methacrylate restitution from drops of about 19 to 45 mm (about 0.6 to 0.9 m/s), with overall mean 0.53 and cell means 0.39 to 0.85.
6. J-STAGE abstract, https://www.jstage.jst.go.jp/article/jsms/71/7/71_603/_article/-char/en. It says restitution depends on wood density and gives no numbers. It is qualitative support only.
7. https://hypertextbook.com/facts/2006/restitution.shtml. A wooden ball on concrete from 92 cm gave 0.603. That is about 4 m/s, so it is not low speed. It is a high-school lab, so I cited it only as a loose bound.

## Not fetched
These returned 403, a redirect or a 404, so I did not use them as sources:
- the apple friction papers (agro.icm.edu.pl)
- core.ac.uk
- the Springer cotton-on-polycarbonate paper
- the textile friction PDF
- the FPL Wood Handbook chapter (502)
- the EngineeringToolbox food-density page (404)

Search snippets mentioned quince and apple friction and restitution, cotton-cotton friction, and TAPPI T815/T816. I did not fetch those pages, so none of their numbers appear as sources.

## Caveats
- **Quote fidelity.** WebFetch returns a small-model summary, not raw text. Quotes from sources 1 to 4 and 7 are the strings it reported. Several (the pine, spruce and ash density lines) are composites of table cells, not single verbatim lines. Re-check against the pages before relying on exact wording.
- **Friction of listed wood-wood pairs.** The data are for bare, clean, dry wood. The class and the table are lacquered or painted, which changes friction. The wood-on-wood entries (softwood and hardwood vs self and table_wood) are therefore proxies. I marked them sourced with wide ranges (static 0.25 to 0.62, kinetic 0.2 to 0.48).
- **Estimated pairs.** Every pad_hard, pad_elastomer and table_laminate pair, and the engineered_wood, fabric and paper-vs-wood pairs, are estimates. I found no fetched measurement for ABS, PLA, TPU or melamine/HPL against these materials. The nearest anchors are wood-on-PMMA (pad_hard proxy), rubber-on-cardboard (paper vs pad_elastomer, the only pad pair with a direct source) and leather-on-wood or rubber-on-rubber (fabric bounds).
- **Kinetic vs static.** Kinetic values are mostly my estimates at about 0.75 to 0.85 times static, with kinetic no greater than static (PhysX requires dynamic friction at or below static). Only oak-on-oak gave both columns directly.
- **Pad_elastomer.** Friction of rubber and TPU is speed-, load- and area-dependent and is not Coulomb. Ranges up to 1.1 (wood) and 1.5 (fabric) reflect this.
- **Restitution.** There are no sourced values (all estimated or unsourced). Restitution is not constant; it varies with speed, with the wood's density, and with the surface it hits. The only quantitative low-speed data are wood particles on PMMA (about 0.4 to 0.85), used to anchor the two wood classes. Engineered wood is an analogy. Paper, fabric and food are guesses.
- **Density.** Sourced for the four rigid classes. Fabric is estimated from leather (860) and loose cotton (80); the 400 point is a guess, so use declared mass where it exists. Food density is unsourced. Bamboo is not covered. Particleboard density is not fetched.
- **Unsourced entries** carry a "why" field beyond the requested schema.
- **Build script.** The script that generated G2.json is at /tmp/g2scripts/build.py; the JSON was written by it, not by hand.
