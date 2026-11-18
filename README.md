# RoboDojo Physics Audit

An audit of the simulated physics and scoring of [RoboDojo](https://github.com/RoboDojo-Benchmark/RoboDojo), forked at upstream commit `f57e163`. It asks whether what the benchmark simulates and scores matches what it declares: whether declared physical values reach the engine and are plausible, whether PhysX agrees with its own converged answer, and whether each task's score counts what its instruction asks. By Bo Peng, Wenhao Chai and Zhifei Li.

- **Start here:** [`docs/SUMMARY.md`](docs/SUMMARY.md), the closing summary: five verified tasks, what we found, what the fork changes, what is unfinished.
- **Blog post:** [What does RoboDojo really simulate?](https://p-b-p-b.github.io/blog/2026/what-does-robodojo-really-simulate/)
- **Findings:** [`docs/findings.md`](docs/findings.md), about 80 entries, each with its evidence.
- **Method:** [`docs/METHOD.md`](docs/METHOD.md). **Running the harness:** [`docs/REPRODUCE.md`](docs/REPRODUCE.md). **Lessons and overturned conclusions:** [`docs/research/lessons.md`](docs/research/lessons.md).
- **Ledger:** [`audit/gates.json`](audit/gates.json), each task config's gate with its evidence under `audit/out/`.
- **RoboDojo's own code map and README:** [`docs/CODE_MAP.md`](docs/CODE_MAP.md), [`docs/UPSTREAM_README.md`](docs/UPSTREAM_README.md).

## About this snapshot

- This is a snapshot of our private working repository at commit `3f2a600` (November 18, 2026). It leaves out the history, the issue threads and the files that coordinated our work (`AGENTS.md`, `STATUS.md`, handoff notes), which a few documents still mention. Numbers such as #58 refer to issues and pull requests in that repository; `docs/findings.md` and `docs/SUMMARY.md` give their substance. Notes on requests and approvals between the authors were removed from the prose; compute budgets and who ran each step are kept, and a few value choices are credited to the author who made them.
- Personal and cluster details are redacted (`<user>`, `<netid>`, `<account>`, `/scratch/<user>`, `<tmp>`), and listings of other cluster users' GPU processes and queued jobs are removed.
- Checksum manifests (`SHA256SUMS`) were regenerated after redaction and verify against this tree. Some entries list raw outputs that stayed on the cluster, and hashes quoted inside READMEs and summaries refer to the unredacted originals.
- The work has stopped; no maintenance is planned.

RoboDojo's code, assets and data belong to their authors under RoboDojo's MIT licence ([`LICENSE`](LICENSE)); our additions are released under the same licence.
