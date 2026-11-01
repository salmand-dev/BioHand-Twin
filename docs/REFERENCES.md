# Scientific and Technical References

Access date for all web sources: 2026-10-31 unless stated. "Retrieved" says what was actually read; claims elsewhere in the project may rely only on what was retrieved.

## Reference model
1. MyoSim repository, commit `93b0ca8f4ec90c9899ee7f05fee561e9911da91b` (v0.2.3). https://github.com/MyoHub/myo_sim. Apache-2.0. Retrieved: full repository.
2. Caggiano V, Wang H, Durandau G, Sartori M, Kumar V. *MyoSuite — A contact-rich simulation suite for musculoskeletal motor control.* arXiv:2205.13600 (2022). Retrieved: full text (HTML), §3.1. Requested citation by upstream.
3. *Towards Embodied AI with MuscleMimic.* arXiv:2603.25544 (2026). Requested citation by upstream. Not retrieved.

## Models MyoHand derives from
4. Holzbaur KRS, Murray WM, Delp SL. A model of the upper extremity for simulating musculoskeletal surgery and analyzing neuromuscular control. *Ann Biomed Eng* 2005;33(6):829–840. doi:10.1007/s10439-005-3320-7. Not retrieved yet; needed for P3.
5. Saul KR, Hu X, Goehler CM, Vidt ME, Daly M, Velisar A, Murray WM. Benchmarking of dynamic simulation predictions in two software platforms using an upper limb musculoskeletal model. *Comput Methods Biomech Biomed Eng* 2015;18(13):1445–1458. doi:10.1080/10255842.2014.916698. Retrieved: abstract.
6. Lee JH, Asakawa DS, Dennerlein JT, Jindrich DL. Finger muscle attachments for an OpenSim upper-extremity model. *PLoS ONE* 2015;10(4):e0121712. doi:10.1371/journal.pone.0121712. Retrieved: full text (journals.plos.org; Tables 2–6 used in P8). Correction: PMC9020702, **not retrieved** (PMC bot check).
7. SimTK projects: MoBL-ARMS https://simtk.org/projects/upexdyn (BSD-3, non-commercial only; text in `data/provenance/licenses/`); 2nd-Hand https://simtk.org/projects/hand_muscle (MIT). Model files obtained 2026-10-31 and parsed (`data/reference_osim_inventory.json`).

## Independent comparison models (MyoHand is NOT derived from these)
8. Mirakhorlo M, Van Beek N, Wesseling M, Maas H, Veeger HEJ, Jonkers I. A musculoskeletal model of the hand and wrist: model definition and evaluation. *Comput Methods Biomech Biomed Eng* 2018;21(9):548–557. doi:10.1080/10255842.2018.1490952. PMID 30257101. Retrieved: abstract. 23 DoF, 43 muscles, 19 segments, carpals as one segment, single-cadaver data.
9. McFarland DC, Binder-Markey BI, Nichols JA, Wohlman SJ, de Bruin M, Murray WM. A musculoskeletal model of the hand and wrist capable of simulating functional tasks. *IEEE Trans Biomed Eng* 2023;70(5):1424–1435. doi:10.1109/TBME.2022.3217722. PMID 36301780. Retrieved: abstract.

## Range-of-motion references (P3)
13. Ryu J, Cooney WP, Askew LJ, An KN, Chao EY. Functional ranges of motion of the wrist joint. *J Hand Surg Am* 1991;16(3):409–419. Retrieved: abstract-level results via search summary (40 subjects; ADL maxima 60° extension, 54° flexion, 40° ulnar, 17° radial deviation). Full text not retrieved.
14. Bain GI, Polites N, Higgs BG, Heptinstall RJ, McGrath AM. The functional range of motion of the finger joints. *J Hand Surg Eur* 2015;40(4):406–411. doi:10.1177/1753193414533754. Retrieved: abstract-level results via repository record (functional MCP 19–71°, PIP 23–87°, DIP 10–64°).
15. Hume MC, Gellman H, McKellop H, Brumfield RH. Functional range of motion of the joints of the hand. *J Hand Surg Am* 1990;15(2):240–243. Identified; numeric results **not** retrieved and not used.

## Moment-arm validation references (P8)
16. Garland AK, Shah DS, Kedgley AE. Wrist tendon moment arms: quantification by imaging and experimental techniques. *J Biomech* 2018;68:136–140. doi:10.1016/j.jbiomech.2017.12.024. CC BY 4.0. Retrieved: full text. Table 1 used.
17. An KN, Ueba Y, Chao EY, Cooney WP, Linscheid RL. Tendon excursion and moment arm of index finger muscles. *J Biomech* 1983;16(6):419–425. Not retrieved; used only as quoted by Lee 2015.

## Tools
10. MyoConverter documentation. https://myoconverter.readthedocs.io. Retrieved: pipeline and limitations pages (via search summary).
11. MuJoCo 3.17.0 documentation. https://mujoco.readthedocs.io. Muscle model, tendons, actuators.
12. Godot 4.7 documentation. https://docs.godotengine.org.

## To retrieve before P2/P3 claims
- Wu G et al. ISB recommendation on definitions of joint coordinate systems … Part II: shoulder, elbow, wrist and hand. *J Biomech* 2005;38(5):981–992.
- An KN et al. 1979 (finger tendon attachments) and An 1983 numeric tables.
- A standard anatomy text (canonical names, attachments).
- Published wrist/finger ROM sources.

For every parameter used in the model: add full bibliographic details, DOI, access date, and the relevant page/figure/table.

## Added muscles, joint stiffness and force comparison (fixes, retrieved 2026-11-01)
- Mirakhorlo M, Visser JMA, Goislard de Monsabert BAAX, van der Helm FCT, Maas H, Veeger HEJ. Anatomical parameters for musculoskeletal modeling of the hand and wrist. *International Biomechanics* 2016;3(1):40–49. doi:10.1080/23335432.2016.1191373. Data: doi:10.6084/m9.figshare.4434791.v1. Retrieved: full text and supplement.
- O'Brien TD, Reeves ND, Baltzopoulos V, Jones DA, Maganaris CN. In vivo measurements of muscle specific tension in adults and children. *Exp Physiol* 2010;95(1):202–210. doi:10.1113/expphysiol.2009.048967. Retrieved: abstract via search result only.
- Formica D, Charles SK, Zollo L, Guglielmelli E, Hogan N, Krebs HI. The passive stiffness of the wrist and forearm. *J Neurophysiol* 2012;108(4):1158–1166. doi:10.1152/jn.01014.2011. Retrieved: full text.
- Shi XQ, Heung HL, Tang ZQ, Tong KY, Li Z. Verification of finger joint stiffness estimation method with soft robotic actuator. *Front Bioeng Biotechnol* 2020;8:592637. doi:10.3389/fbioe.2020.592637. Retrieved: full text, by automated summary.
- Wu JZ, Li ZM, Cutlip RG, An KN. A simulating analysis of the effects of increased joint stiffness on muscle loading in a thumb. *BioMed Eng OnLine* 2009;8:41. doi:10.1186/1475-925X-8-41. Retrieved: abstract in PMC (PMC2804669), which states the values are assumed.
- Park CY, Yim HW, Koo JW, Ahn BY, Baag YM, Lee KS. Normative data of grip and pinch strength for screening of cumulative trauma disorders. *Korean J Occup Environ Med* 1998;10(3):362–378. doi:10.35371/kjoem.1998.10.3.362. Retrieved: abstract, means only.
- Smutz WP, Kongsayreepong A, Hughes RE, Niebur G, Cooney WP, An KN. Mechanical advantage of the thumb muscles. *J Biomech* 1998;31(6):565–570. Retrieved: not retrieved; only its reported muscle actions as summarised elsewhere. Used for expected signs, not for numbers.
