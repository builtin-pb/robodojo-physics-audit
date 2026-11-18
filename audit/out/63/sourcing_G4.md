# G4: partner surfaces and cross-class pairs

Output: `G4.json`. It holds 32 values: 4 restitution values and 14 pairs, each with a static and a kinetic coefficient.

**Counts: 6 sourced, 21 estimated, 5 unsourced.**

## What "sourced" and "estimated" mean here

- **sourced**: a page I fetched gives a number for that same pair, and I quote the line.
- **estimated**: no fetched source covers the pair. The value is judged from a fetched analogue pair (for example polystyrene or polyethylene on steel), and the analogue is quoted with a note that it is only an analogue.
- **unsourced**: no usable source at all; the number is my guess.

## Sources actually fetched and used

- Engineering Toolbox friction table: https://www.engineeringtoolbox.com/friction-coefficients-d_778.html
- Wikipedia, Friction: https://en.wikipedia.org/wiki/Friction
- Physics Factbook, wood: https://hypertextbook.com/facts/2005/wood.shtml
- Stanford Advanced Materials restitution table: https://samaterials.com/content/coefficient-of-restitution.html

The fetch tool summarises pages with a small model, so quotes are the cell text it returned, not raw HTML. Wikipedia's column labels were inferred from flattened text. I re-fetched the Engineering Toolbox page once and the rows matched.

## Results (static / kinetic, point value with range)

| Pair | μs | μk | Status |
|---|---|---|---|
| softwood–softwood | 0.40 (0.20–0.62) | 0.35 (0.15–0.48) | sourced |
| softwood–hardwood | 0.40 (0.20–0.62) | 0.35 (0.15–0.48) | sourced |
| steel–softwood | 0.40 (0.20–0.60) | 0.35 (0.20–0.50) | sourced |
| glazed_ceramic–glazed_ceramic | 0.50 (0.25–0.90) | 0.35 (0.15–0.60) | estimated |
| glazed_ceramic–glass | 0.50 (0.30–0.90) | 0.35 (0.20–0.50) | estimated |
| steel–glazed_ceramic | 0.50 (0.30–0.80) | 0.40 (0.20–0.60) | estimated |
| softwood–abs_toy | 0.35 (0.20–0.60) | 0.30 (0.15–0.50) | estimated |
| abs_toy–polyolefin | 0.25 (0.10–0.50) | 0.20 (0.08–0.40) | estimated |
| pet–polyolefin | 0.25 (0.10–0.50) | 0.20 (0.08–0.40) | estimated |
| pet–steel | 0.30 (0.15–0.50) | 0.25 (0.12–0.45) | estimated |
| paper–softwood | 0.40 (0.25–0.60) | 0.30 (0.20–0.50) | estimated |
| paper–abs_toy | 0.35 (0.20–0.60) | 0.30 (0.15–0.50) | estimated |
| aluminium–abs_toy | 0.35 (0.20–0.60) | 0.30 (0.15–0.50) | estimated |
| abs_toy–abs_toy | 0.40 (0.20–0.70) | 0.30 (0.15–0.50) | unsourced |

Surface restitution against a hard object, below 1 m/s:

| Surface | Value (range) | Status |
|---|---|---|
| table_wood | 0.45 (0.30–0.60) | estimated |
| table_laminate | 0.40 (0.20–0.60) | unsourced |
| pad_hard | 0.50 (0.30–0.70) | unsourced |
| pad_elastomer | 0.40 (0.15–0.80) | unsourced |

## Caveats that matter for the audit

1. **The "sourced" wood pairs are for bare, clean, dry wood.** RoboDojo's softwood is lacquered or painted. The ranges are widened downward for that, but no fetched source measures lacquered wood. Treat the "sourced" label as applying to bare wood only.
2. **The steel–softwood kinetic value is thin.** It rests on one Wikipedia cell whose label was inferred. The static range is better supported.
3. **Glass on glass at 0.9–1.0 is a lab-clean value.** I used it only as an upper bound for the ceramic and glass pairs. Real glazed tableware is contaminated and rough, so I put the point value near 0.5.
4. **Restitution evidence is weak.**
   - The only fetched material-pair table (Stanford) is secondary and states no impact speeds.
   - Its one stated speed is a steel ball at 1 m/s, COR about 0.65.
   - The Wikipedia COR page confirms COR belongs to a pair of surfaces, not one material. It gives no values for these materials.
   - I never read a primary measurement of ABS, laminate or elastomer restitution, so those entries are guesses.

## Leads I could not verify

None of these were fetched, so none are cited in the JSON.

- Pouzada et al., Polymer Testing 2006 (ABS and PP on mild steel). A search snippet gave ABS about 0.35 static and 0.30 kinetic, and PP about 0.11 and 0.08. The full text returned HTTP 403. If confirmed, these would lift abs_toy–steel and polyolefin pairs to sourced.
- Demarch et al. on ceramic tile friction. The PDF has no extractable text. A snippet gave dry tile static of 0.60–0.85.
- TAPPI T 815 and T 816 for paper and board. These are standards, and no numbers were found.
- Curbell white paper on friction. Its PDF text was unreadable.

## Out of scope

Friction between the pad or table surfaces and the object classes was not requested in this task, so no such pairs are in `G4.json`.
