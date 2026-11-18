# G1 sourcing notes (pet, abs_toy, abs_device, polyolefin, rubber, foam)

Counts (72 values: 6 densities, 6 restitutions, 30 pairs x static and kinetic): **8 sourced, 20 estimated, 44 unsourced.**

## Status meanings
- sourced: a fetched source gives a number for this material or pair (static friction for three self-pairs; five densities).
- estimated: no number for the exact pair, but a fetched number for a close pair anchors it (for example kinetic taken as ~0.8-0.9 of static, or a plastic-on-hard-plastic pad taken from the plastic's self-pair).
- unsourced: no fetched number bears on it; value is my engineering estimate.

## Sources actually fetched and used
- https://www.engineeringtoolbox.com/density-solids-d_1265.html: bulk densities (ABS 1.06, PET 1.35, PP 0.91-0.94, HDPE 0.96, soft rubber 1.1, PU foam 0.03, rubber foam 0.06-0.25).
- https://www.engineeringtoolbox.com/friction-coefficients-d_778.html: rubber/rubber 1.16, PS/PS 0.5, PE/PE 0.2, all "Clean and Dry".
- https://engineersedge.com/coeffients_of_friction.htm: rubber/rubber 1.15, PS/PS 0.50, polyethene/steel 0.2 (static table).
- https://www.professionalplastics.com/professionalplastics/content/downloads/MechanicalPropertiesofPlastics.pdf (read with pdftotext): "Coefficient of friction" column: ABS 0.5, HDPE 0.29, PP 0.1-0.3, PET 0.2-0.4.
- https://hypertextbook.com/facts/2006/restitution.shtml: hollow hard plastic ball e=0.688, 92 cm drop onto concrete.
- https://en.wikipedia.org/wiki/Friction and https://en.wikipedia.org/wiki/Coefficient_of_restitution were fetched; they gave nothing I used beyond confirming wood/wood ranges and that no polymer COR values are listed.

## Caveats
1. **Quote fidelity.** The Engineering Toolbox, Engineers Edge and hypertextbook lines in G1.json are what WebFetch's page extraction returned (a summarising model), not raw HTML. Column order (static first, kinetic second) is the page's stated convention, but the tables were flattened. Verify before publishing any single number.
2. **Professional Plastics table** gives no counterface, load, speed or static/kinetic label. I used it only as a range anchor.
3. **Restitution: nothing sourced.** No fetched source gives a under-1 m/s restitution for any of these classes. All six values are estimates flagged "unsourced". The only fetched datum (0.688, hollow plastic ball) is at about 4.2 m/s, unidentified material, concrete floor. The Güttler et al. arXiv survey (arxiv.org/pdf/1204.0001, read via pdftotext) shows restitution depends on velocity and material, but I extracted no polymer value from it.
4. **No source for any polymer, rubber or foam against wood or laminate, or against an elastomer pad.** Those 38-ish pair values are estimates. Wood/wood (0.25-0.62 static, 0.2-0.48 kinetic) was fetched but is not a polymer pair.
5. **Search-only claims not used as citations** (I never fetched the underlying document): PET bottle COF 0.34 vs HDPE 0.12 (patent snippet; the patent PDF was image-only), TPU 40D vs steel 0.30/0.26 (Susheng datasheet snippet), HR foam rebound 40% and a 31.9% rebonded-foam sheet (ASTM D3574 Test H; patent PDF image-only), a forum value of ~0.6 kinetic for rubber on wood. These could be fetched later (HTML versions or OCR) to upgrade estimates.
6. **Blocked or failed fetches:** Misumi (403), MDPI Polymers 15/24/4696 and MDPI Lubricants 10/9/204 (403), the Aviation PET-on-PET paper (connection reset), foamforyou datasheet (404). The tribology.rs 3D-printed polymer paper was read but tests PLA/PETG against steel only, so I did not use it.
7. **abs_device_housing density (450)** is an effective object density I chose; bulk ABS is sourced but real value depends on how the benchmark derives density from mass and volume.
8. **Pad scenarios.** pad_hard pairs reuse each plastic's self-pair value as an analogue. pad_elastomer pairs for hard plastics are 0.5-0.7 estimates; elastomer on plastic can span 0.4-1.5 depending on finish, so widen if the audit is sensitive.
9. **Coulomb limits.** Foam, soft rubber and TPU show load- and speed-dependent friction (adhesion, deformation), so a single mu is a rough fit; ranges are wide on purpose.
10. Kinetic values for sourced static pairs are estimates (about 0.8-0.9 of static), because no fetched table gave a kinetic number for those pairs.
