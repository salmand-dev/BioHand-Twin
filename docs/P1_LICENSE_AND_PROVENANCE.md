# P1 — License and Provenance Report: MyoHand / MyoSim and its OpenSim sources

- Last updated: 2026-10-31
- Machine-readable record: `data/provenance/model_manifest.yaml`
- Hashes: `data/provenance/myohand_file_hashes.json`
- Licence texts: `data/provenance/licenses/`

*This is a technical reading of the licence texts, not legal advice.*

## 1. Pinned sources

| Source | Identity | Location (gitignored) | Modified by BioHand |
|---|---|---|---|
| MyoSim | https://github.com/MyoHub/myo_sim @ `93b0ca8f4ec90c9899ee7f05fee561e9911da91b` (2026-09-15, v0.2.3, no tag) | `third_party/myo_sim` | No (73 SHA-256 hashes checked by test) |
| MoBL-ARMS bimanual (OpenSim 3) | `MobL_ARMS_OpenSim3_bimanual_model.zip`, sha256 `1735972e…f6d55c`; `MoBL_ARMS_bimanual_6_2_21.osim`, sha256 `a776ad04…c85c673`; credits Saul, Murray, Goehler, Daly, Vidt, Crouch | `third_party/simtk_mobl_arms_bimanual` | No |
| 2nd-Hand (Lee 2015) | Archive "hand model with geometry files" (zip, entries dated 2014-09-04), sha256 `0e7254c0…512dc31`; contains `2nd_Hand_Model_Mass.osim` and `2nd_Hand_Model_An_attachments.osim`, 33 VTP + 33 ASC bone files | `third_party/simtk_2nd_hand` | No |

All three were obtained on 2026-10-31. The two SimTK archives were downloaded by the project owner from https://simtk.org/projects/upexdyn and https://simtk.org/projects/hand_muscle. Neither archive contains a licence file. The licence texts were copied by the owner from the SimTK download agreements and saved verbatim.

## 2. Licence terms

| Component | Licence | Key conditions |
|---|---|---|
| MyoSim repository | Apache-2.0 (LICENSE file, GitHub API, XML headers) | Notice, licence copy, statement of changes |
| **MoBL-ARMS** (`MoBL-ARMS_use_agreement.txt`) | **BSD-3-Clause, open-sourced "solely for non-commercial purposes (including research, academic, evaluation and personal uses)"** | (1) non-commercial only — **commercial use requires a commercial licence**; (2) retain the copyright notice ("© 2014-present, NC State Univ., Northwestern Univ., RIC, Valparaiso Univ., Wake Forest Univ., Stanford Univ. and VA HCS"); (3) **must cite** Saul et al. 2015 *and* McFarland et al. 2019 in all publications, presentations or documents; (4) no endorsement use of names |
| **2nd-Hand** (`2nd-Hand_MIT_license.txt`) | **MIT**, © 2015 Stanford University | Keep the copyright and permission notice |
| MuJoCo 3.17.0 | Apache-2.0 | — |

## 3. Is MyoHand derived from MoBL-ARMS? — yes, demonstrably

Evidence: `scripts/p1_osim_inventory.py` → `reports/P1/osim_vs_myohand.json`.

1. **25 of the 26 extrinsic muscles** in MyoHand have a peak force equal to MoBL-ARMS `max_isometric_force`:
   - 24 match exactly;
   - ECRB is 252.2 vs 252.5 N.
   - The 26th, EDC5, differs (§ gap analysis G-S2).
2. **20 of 23 joint ranges** are identical to 2nd-Hand coordinate ranges.
3. All 33 bone mesh names in the 2nd-Hand geometry set appear in MyoSim's mesh folder.

**Consequence.** The MoBL-ARMS terms (non-commercial, attribution, citation) apply to the MoBL-derived content of MyoHand. That includes muscle forces, wrist/forearm kinematics and probably the bone geometry. The Apache-2.0 label on the MyoSim repository does not remove that restriction. MyoSim does not state the restriction itself.

## 4. What BioHand Twin may do

| Action | Allowed? | Conditions |
|---|---|---|
| Use MyoHand for research, education or personal projects | **Yes** | Cite Saul 2015, McFarland 2019, MyoSuite, Lee 2015; keep notices |
| Commercial product, paid service or commercial company use | **No** | Needs a MoBL-ARMS commercial licence from the copyright holders |
| Redistribute MyoSim XML/STL or derived GLB (non-commercial) | Permitted under BSD-3 + Apache-2.0 + MIT conditions | **Project policy: still do not bundle.** Users fetch upstream at the pinned commit; visual assets are built locally |
| Release BioHand's own code under MIT | Yes | The README must state that the full system depends on non-commercial model data |

## 5. Provenance completeness checklist (project rule)

| Field | Status |
|---|---|
| Source URL | ✔ all three sources |
| Exact version / commit | ✔ MyoSim commit. SimTK archives are identified by SHA-256; SimTK gives no version numbers |
| Author / copyright | ✔ |
| Licence | ✔ verbatim texts archived |
| Date retrieved | ✔ 2026-10-31 |
| Modifications | ✔ none |
| Redistribution permission | ✔ determined (non-commercial, with notices) |
| Intended use declared | ✔ **Non-commercial: research and education** (owner, 2026-10-31) |

## 6. Remaining actions

1. ~~Owner: confirm non-commercial use~~ — done 2026-10-31 (research and education).
2. Add the MoBL-ARMS copyright notice and required citations to `README.md` and the eventual UI "About/Credits" screen.
3. Optional: report the EDC5 force discrepancy and the missing non-commercial notice to MyoHub.
