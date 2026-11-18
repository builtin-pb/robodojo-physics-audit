# G3: steel, aluminium, glazed_ceramic, glass

Counts (48 values): 9 sourced, 23 estimated, 16 unsourced.

## Sources fetched
- Engineering Toolbox, friction coefficients: https://engineeringtoolbox.com/friction-coefficients-d_778.html (steel/steel, Al/Al, glass/glass, glass/metal, wood/metal; clean and dry)
- Roymech, coefficients of friction: https://roymech.org/Useful_Tables/Tribology/co_of_frict.html (same pairs, plus Plexiglas/steel and polystyrene/steel as hard-plastic analogues)
- Engineering Toolbox, density of solids: https://www.engineeringtoolbox.com/density-solids-d_1265.html
- Hypertextbook, restitution classroom experiment: https://hypertextbook.com/facts/2006/restitution.shtml
- Vendor blog on rubber COF: https://coefficientoffrictiontester-co.pages.dev/blog/rubber-cof-testing-guide (weak, marketing source)
- Peer-reviewed ceramic tile friction, PMC8624764: https://pmc.ncbi.nlm.nih.gov/articles/PMC8624764 (rubber slider on tile, sliding)

## What is sourced
- All four densities.
- Kinetic mu for steel/steel, Al/Al, glass/glass.
- Static mu for steel/steel.
- Kinetic mu for ceramic against rubber (from tile data).

## Caveats
- **Quotes are not strictly verbatim.** WebFetch returns a model-written summary of the page, not raw text. The quotes are the lines it returned, and I did not re-check them against the raw HTML. Table rows from Engineering Toolbox and Roymech should be spot-checked before they are cited externally.
- **Restitution is not a low-speed value.** The only numbers found (steel 0.597, glass 0.658) come from a high-school drop test from 92 cm onto concrete. That is about 4.2 m/s, not under 1 m/s. I therefore marked them "estimated", not "sourced". Aluminium and ceramic restitution are unsourced guesses. An aluminium paper (Aryaei et al.) was found but its page redirected, so I did not fetch it.
- **Handbook friction is for clean, dry, bare surfaces.** It is likely high for anodised aluminium, glazed ceramic and handled glass. Al/Al static 1.05–1.35 is a galling-prone bare-metal figure. I used 0.9 with a wide range for that reason. Roymech's Al/Al sliding value of 1.4 looks like a typo, so I ignored it.
- **Elastomer pad pairs** rely on a vendor blog for natural rubber on steel (static 0.8–2.0, kinetic 0.5–1.5). TPU or Shore A 60–90 pads are probably at the lower end, but I found no measurement. The ceramic and glass elastomer values use the tile paper (rubber on glazed tile, dry, 0.58 sliding), and the glass values borrow from it.
- **Hard-pad pairs** use Plexiglas/steel and polystyrene/steel as analogues. No ABS or PLA data was found, and none for glass or ceramic.
- **Table pairs.** Wood is from bare wood on clean metal (0.2–0.6), not lacquered wood. No laminate (melamine or HPL) source was found, so all laminate pairs are unsourced. Ceramic/wood and glass/wood are also unsourced.
- **Static vs kinetic.** Where sources gave only static values, kinetic is an estimate set about 15–25% lower.
- **Ceramic/ceramic** is borrowed from glass/glass.
- **Hollow items** (cans, vessels): density is the bulk material value, so use effective mass for thin-wall parts.
