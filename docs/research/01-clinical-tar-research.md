# Total Ankle Replacement (TAR) — Surgical Simulation Reference Report

> Comprehensive reference data for building a biomechanically accurate surgical simulation of Total Ankle Arthroplasty (TAA). All numerical values are drawn from peer-reviewed literature and manufacturer documentation.

---

## Table of Contents

1. [Surgical Procedure](#1-surgical-procedure)
2. [Biomechanical Data](#2-biomechanical-data)
3. [Anatomical Structures](#3-anatomical-structures)
4. [Simulation Parameters](#4-simulation-parameters)
5. [Clinical Outcome Metrics](#5-clinical-outcome-metrics)
6. [Sources](#sources)

---

## 1. Surgical Procedure

### 1.1 Pre-Operative ROM Assessment

| Parameter | Typical Value |
|---|---|
| Normal total sagittal ROM | ~70 degrees (20 degrees DF + 50 degrees PF) |
| End-stage arthritic total ROM | **22-31 degrees** (combined DF + PF) |
| Mean pre-op DF | ~5-10 degrees |
| Mean pre-op PF | ~15-20 degrees |
| Functional gait requirement | 15 degrees DF + 15 degrees PF (minimum) |

Pre-operative assessment includes weight-bearing AP and lateral radiographs, CT scan (for patient-specific instrumentation planning), assessment of coronal plane deformity (classified as: neutral < 5 degrees, moderate 5-15 degrees, severe > 15 degrees varus/valgus), and evaluation of ligamentous integrity.

### 1.2 Surgical Approach (Anterior)

**Standard Anterior Approach:**
- **Incision:** 12 cm longitudinal incision centered over the anterior ankle joint
- **Interval:** Between the tibialis anterior (TA) tendon and the extensor hallucis longus (EHL) tendon
- **Superficial structures at risk:** Superficial peroneal nerve (traverses lateral-proximal to medial-distal, immediately deep to dermis)
- **Deep structures at risk:** Anterior tibial artery/vein, deep peroneal nerve (DPN) -- retracted laterally with EHL
- **Retinaculum:** Extensor retinaculum incised longitudinally; TA tendon sheath must NOT be opened (risk of bowstringing and wound complications)
- **Capsulotomy:** Joint capsule and periosteum incised; flaps tagged for closure

**Patient Positioning:** Supine, slight limb internal rotation, bump under ipsilateral hip.

**Alternative Approaches:**
- Extensile anteromedial: incision along medial border of anterior angiosome
- Modified anterolateral (Bohler's): skin incision not placed directly over TA/EHL; reduced eschar formation (13% vs. 46% conventional)
- Lateral transfibular: improved visualization for severe deformity correction

### 1.3 Bone Resection Steps

#### 1.3.1 Tibial Cut

| Parameter | Value |
|---|---|
| Orientation | Perpendicular to the mechanical axis of the tibia |
| Target beta angle (coronal) | 90 degrees (range: 85-95 degrees acceptable) |
| Anterior distal tibial angle (sagittal) | 89 +/- 3 degrees (acceptable range: 86-92 degrees) |
| Resection depth (from plafond apex) | **~5 mm** (STAR system reference); range 2-10 mm depending on implant system |
| Incremental adjustment | 2 mm increments for global tightness |
| FE-modeled resection depths | 6-16 mm across different implant designs (STAR, Mobility, Salto) |

**Procedure:**
1. External alignment jig placed over anterior tibial crest, fixed proximally and distally
2. Jig aligned perpendicular to the long axis of the tibia
3. Position confirmed with fluoroscopy (AP and lateral)
4. Cutting block adjusted for rotation and resection depth per implant-specific protocol
5. Oscillating saw used for flat transverse cut
6. Care taken to avoid malleolar fracture and excessive bone removal

**Deformity correction through tibial cut:**
- Valgus deformity: greater medial resection
- Varus deformity: greater lateral resection
- Deformities < 10 degrees: correctable with tibial cut alone
- Deformities > 10 degrees: may require dome or wedge osteotomy prior to TAR

#### 1.3.2 Talar Cut

| Parameter | Value |
|---|---|
| Geometry | Dome-shaped or flat depending on implant system |
| Resection depth | Minimal; typically ~2 mm from medial side |
| Goal | Maintain concentric talar reduction; adequate dome coverage |
| Chamfer cuts | System-dependent (e.g., INFINITY offers chamfered or flat-cut talar options) |

**Procedure:**
1. Talar cutting block positioned flush with talar surface
2. Orientation set to minimize bone resection while matching implant geometry
3. For STAR: talar dome resurfaced to accept anatomical talar component
4. For fixed-bearing systems: flat talar cut with possible anterior/posterior chamfers
5. Component must adequately cover talar dome without overhang (risk of impingement, stress fracture)

### 1.4 Implant Components

All modern TAR systems consist of three functional elements:

| Component | Material | Function |
|---|---|---|
| **Tibial component** | CoCr alloy with Ti plasma-spray coating | Articulates with polyethylene; anchored in distal tibia via barrels/pegs/stem |
| **Talar component** | CoCr alloy with Ti plasma-spray coating | Resurfaces talar dome; anchored via pegs/keel/sulcus design |
| **Polyethylene bearing** | UHMWPE (ultra-high molecular weight polyethylene) | Articulates between tibial and talar components; mobile or fixed |

### 1.5 Common Implant Systems

#### STAR (Scandinavian Total Ankle Replacement)

| Feature | Specification |
|---|---|
| Manufacturer | Originally Stryker; now Enovis/DJO Global |
| Generation | 2nd generation (revised to 4th-gen coating) |
| Bearing type | **Mobile** (only mobile-bearing system FDA-approved in US) |
| Components | 3 (tibial plate + talar component + mobile PE bearing) |
| Fixation | Cementless (porous Ti plasma spray on CoCr) |
| Tibial plate thickness | 2.5 mm |
| Tibial fixation | Two parallel cylindrical barrels (anterior-posterior oriented) |
| Talar design | Anatomical dome with central sagittal ridge |
| PE bearing heights | **6, 7, 8, 9, 10 mm** |
| Sizes available | 5 (right and left) |
| Example tibial plate | 32 x 30 mm |
| Example talar component | 30 x 31 mm |
| PE marker | 0.5 mm stainless steel wire for radiographic monitoring |
| 5-yr survivorship | 89-96% |
| 10-yr survivorship | 88-94% |
| 15-yr survivorship | ~73% |

#### INFINITY Total Ankle System

| Feature | Specification |
|---|---|
| Manufacturer | Wright Medical Group (now Stryker) |
| Generation | 4th generation |
| Bearing type | **Fixed** |
| Components | 2 (tibial tray with PE insert + talar dome) |
| Tibial design | Adaptis 3D-printed porous metal tray |
| Talar options | Chamfered or flat-cut Adaptis talar dome |
| PE insert | Everlast highly cross-linked polyethylene |
| PSI available | Yes (Prophecy -- uses pre-op CT for patient-specific cut guides) |
| Key advantage | Smaller implant; reduced soft-tissue dissection and bone resection |

#### INBONE II Total Ankle System

| Feature | Specification |
|---|---|
| Manufacturer | Wright Medical Group (now Stryker) |
| Generation | 3rd generation |
| Bearing type | **Fixed** |
| Components | 2 (tibial + talar) |
| Materials | Titanium-coated CoCr tibial and talar base |
| Tibial fixation | Long intramedullary modular stem (superior fixation in poor bone stock) |
| Talar design | Sulcus-shaped profile with two anterior pegs |
| 5-yr survivorship (INBONE II) | ~98% |

#### Salto Talaris

| Feature | Specification |
|---|---|
| Manufacturer | Integra LifeSciences |
| Generation | 3rd generation |
| Bearing type | **Fixed** |
| Components | 2 (tibial with fixed PE + talar) |
| Talar design | Conical shape with dual radii of curvature (medial < lateral); lateral facet resurfaced, medial spared |
| Sagittal groove | Forces external rotation in DF, internal rotation in PF |
| Fixation surface | Plasma titanium spray + keel |
| Revision rate | 2.4% at 34.9 months (systematic review of 212 ankles) |

#### Vantage Total Ankle System

| Feature | Specification |
|---|---|
| Manufacturer | Exactech |
| Generation | 4th generation |
| Bearing type | **Fixed** |
| Talar component | Curved design aligned with trabecular bone structure |
| Key advantage | Anatomic alignment for increased stability and minimal bone resection |

### 1.6 Post-Operative ROM Expectations

| Parameter | Value |
|---|---|
| Mean post-op total ROM | **33-53 degrees** (wide variability) |
| Target post-op DF | ~10 degrees |
| Target post-op PF | ~35 degrees |
| Ideal post-op total | ~45 degrees |
| ROM improvement from pre-op | Modest: **4-14 degrees** mean improvement (systematic review) |
| Key caveat | TAR has limited potential to increase pre-operative ROM; primary benefit is **pain relief** |

---

## 2. Biomechanical Data

### 2.1 Normal Ankle Range of Motion

| Movement | Normal Range | Functional Gait Requirement |
|---|---|---|
| Dorsiflexion | **10-20 degrees** (mean ~20 degrees) | 15 degrees |
| Plantarflexion | **40-50 degrees** (mean ~50 degrees) | 15 degrees |
| Inversion | 0-35 degrees | -- |
| Eversion | 0-15 degrees | -- |
| Total sagittal arc | ~70 degrees | 30 degrees minimum |

**Age effects:** Younger females (20-39 yrs) have higher ROM than males; older females (60+) demonstrate ~8 degrees less dorsiflexion than age-matched males.

**Stability:** The talus is wider anteriorly and narrower posteriorly, making **dorsiflexion the most stable position** (close-packed position).

### 2.2 ROM in Disease and After Surgery

| Condition | Total Sagittal ROM |
|---|---|
| Normal healthy ankle | ~70 degrees |
| End-stage ankle OA (pre-op) | **22-31 degrees** |
| Post-TAR (1 year) | **33-53 degrees** |
| Post-TAR (long-term) | **30-45 degrees** (regression toward pre-op possible) |
| Post-ankle arthrodesis | 0 degrees (fused) |

### 2.3 Key Ligaments

#### Lateral Ligament Complex

| Ligament | Function | Injury Prevalence in LAS | Relative Strength |
|---|---|---|---|
| **ATFL** (Anterior Talofibular) | Resists anterior translation and internal rotation | 85-100% | Weakest of the three |
| **CFL** (Calcaneofibular) | Resists inversion; acts in neutral and DF | ~35% | Intermediate |
| **PTFL** (Posterior Talofibular) | Resists posterior translation | ~10% | Strongest of the three |

#### Deltoid (Medial) Ligament Complex

| Layer | Components | Function |
|---|---|---|
| **Superficial** | Tibionavicular, tibiospring, tibiocalcaneal, superficial posterior tibiotalar | Resist eversion |
| **Deep** | Anterior tibiotalar (ATTL), posterior tibiotalar (PTTL) | Greatest restraint against lateral talar translation; must be completely ruptured for valgus tilt |

**Origin anatomy:** Superficial from anterior colliculus of medial malleolus; deep from intercollicular groove and posterior colliculus.

#### Syndesmosis (Tibiofibular Ligaments)

| Ligament | Contribution to Ankle Stability |
|---|---|
| **AITFL** (Anterior Inferior Tibiofibular) | ~35% |
| **Deep PITFL** (Posterior Inferior Tibiofibular) | ~33% |
| **Interosseous ligament/membrane** | ~22% |
| **Superficial PITFL** | ~9% |

### 2.4 Load-Bearing Characteristics

| Parameter | Value |
|---|---|
| Peak tibiotalar contact force (walking) | **Up to 5x body weight** |
| Peak force (running) | ~4-5x body weight |
| Peak force (sprinting) | ~6-7x body weight |
| Tibiotalar load share | **~83%** of total ankle load |
| Fibular load share | **~17%** of total ankle load |
| Talar dome load share (of tibiotalar load) | 77-90% |
| Static contact pressure (neutral, 1.5 kN load) | **9.9 MPa** |
| Static contact area (neutral) | **483 mm-squared** |
| Heel-strike calcaneus load | ~80% of body weight directly over calcaneus |
| Maximum joint power generation | ~50% of gait cycle (push-off) |

**Contact migration:** The contact centroid moves anteriorly-to-posteriorly on the talus as the joint moves from dorsiflexion to plantarflexion.

**Higher pressures in plantarflexion** than in dorsiflexion under static loading.

---

## 3. Anatomical Structures

### 3.1 Bones

#### Distal Tibia
- Forms the tibial plafond (inferior quadrilateral articular surface)
- Medial malleolus provides medial buttress of the ankle mortise
- The tibial plafond is the primary weight-bearing articular surface

#### Talus
- Second largest hindfoot bone
- **No direct muscular attachments**; **tenuous, limited blood supply**
- Wider anteriorly, narrower posteriorly (wedge shape)
- Articulates with: tibia (talocrural), calcaneus (subtalar), navicular (talonavicular)
- Blood supply: posterior tibial artery, peroneal artery, anterior tibial artery (dorsalis pedis)
- Trochlear surface (dome) is the primary articular surface for TAR

#### Fibula (Lateral Malleolus)
- Externally rotated **25-30 degrees** relative to the distal tibia in the incisura fibularis
- Forms lateral wall of ankle mortise
- Provides attachment for lateral ligament complex
- Bears ~17% of ankle joint load

#### Calcaneus
- Largest tarsal bone
- Receives ~80% of body weight at heel strike
- Subtalar joint with talus allows inversion/eversion

### 3.2 Articular Surfaces

| Surface | Description |
|---|---|
| **Tibial plafond** | Concave inferior surface of distal tibia; primary weight-bearing surface |
| **Talar dome (trochlea)** | Convex superior surface of talus; wider anteriorly; articulates with tibial plafond |
| **Medial malleolar facet** | Medial articular surface of talus for medial malleolus |
| **Lateral malleolar facet** | Lateral articular surface of talus for lateral malleolus (fibula) |
| **Ankle mortise** | Formed by tibial plafond + medial malleolus + lateral malleolus + inferior transverse tibiofibular ligament |

### 3.3 Key Ligaments and Tendons

**Tendons crossing the anterior ankle (relevant to surgical approach):**

| Tendon | Action | Surgical Relevance |
|---|---|---|
| Tibialis anterior | Dorsiflexion + inversion | Defines medial border of surgical interval |
| Extensor hallucis longus | Great toe extension + DF | Defines lateral border of surgical interval |
| Extensor digitorum longus | Toe extension + DF | Lateral to EHL |
| Peroneus tertius | DF + eversion | Lateral |

**Plantarflexor complex (posterior):**
- Gastrocnemius, soleus, plantaris (Achilles tendon) -- primary plantarflexors
- Peroneus longus and brevis -- plantarflexion + eversion
- Tibialis posterior, FHL, FDL -- posterior compartment

### 3.4 Neurovascular Structures

| Structure | Location | Surgical Risk |
|---|---|---|
| **Superficial peroneal nerve** | Superficial, immediately deep to dermis; traverses lateral-proximal to medial-distal | Highest risk during anterior approach skin incision |
| **Deep peroneal nerve** | Travels with anterior tibial vessels between EHL and EDL | Retracted laterally with neurovascular bundle |
| **Anterior tibial artery/vein** | Deep to extensor retinaculum, between EHL and EDL; becomes dorsalis pedis | Protected and retracted laterally during dissection |
| **Saphenous nerve/vein** | Medial, along distal medial tibia | At risk with medial portal or anteromedial approach |
| **Posterior tibial artery/nerve** | Posterior-medial, behind medial malleolus | Protected by FHL in posterolateral approach |
| **Sural nerve** | Posterolateral | At risk in lateral approaches |

**Nerve supply to ankle joint:** L4-S2 via deep fibular (peroneal) nerve, tibial nerve, and sural nerve.

**Arterial supply:** Anterior tibial, posterior tibial, and peroneal arteries form an anastomosis around the malleoli, giving off anterior medial and lateral malleolar branches.

---

## 4. Simulation Parameters

### 4.1 Material Properties of Bone

| Property | Cortical Bone | Cancellous Bone (Apparent/Bulk) | Cancellous Bone (Tissue-Level) |
|---|---|---|---|
| **Young's modulus (longitudinal)** | **17-20 GPa** | **0.2-5 GPa** (mean ~1 GPa) | 10-20 GPa |
| **Young's modulus (transverse)** | **6-13 GPa** | Density-dependent | -- |
| **Poisson's ratio** | ~0.3 | ~0.3 | -- |
| **Compressive strength (longitudinal)** | **190 MPa** | Variable (density-dependent) | -- |
| **Compressive strength (transverse)** | **130 MPa** | Variable | -- |
| **Tensile strength (longitudinal)** | **130 MPa** | Variable | -- |
| **Tensile strength (transverse)** | **50 MPa** | Variable | -- |
| **Max elongation before fracture** | 0.5-3% | Variable | -- |

**FE modeling recommendation for tibia:**
- Cortical shell: E = 17,000 MPa (17 GPa), nu = 0.3
- Cancellous core: E = 1,000 MPa (1 GPa), nu = 0.3
- Simplified isotropic model: E = 7,300 MPa, nu = 0.3 (combined skeleton value used in ankle FE literature)

**Anisotropy note:** Cortical bone is transversely isotropic. The longitudinal axis (along the tibial shaft) is the stiffest direction.

### 4.2 Articular Cartilage Properties

| Property | Value |
|---|---|
| **Young's modulus (equilibrium/indentation)** | **0.5-10 MPa** |
| Tibial cartilage (ankle-specific) | **1.19 MPa** |
| Talar cartilage (ankle-specific) | **1.06 MPa** |
| Aggregate modulus | 0.5-0.9 MPa |
| **Tensile modulus** | **5-25 MPa** |
| Shear modulus (equilibrium) | 0.05-0.25 MPa |
| **Poisson's ratio** | **0.14-0.18** (compressible behavior) |
| FE model value (common) | E = 1.0 MPa, nu = 0.4 (or E = 0.7 MPa, nu = 0.49) |
| Alternative FE value | E = 5-12 MPa, nu = 0.46 |

**Behavior:** Cartilage is a biphasic (solid + fluid), viscoelastic material. Simulation should consider time-dependent creep/relaxation under sustained loading.

### 4.3 Ligament Properties

| Ligament / Complex | Linear Elastic Modulus | Stiffness | Notes |
|---|---|---|---|
| **ATFL** | **~255 MPa** | ~150 N/mm (with ITCL complex) | Most commonly injured; weakest lateral ligament |
| **CFL** | -- | **55.8 +/- 23.0 N/mm** | Cordlike; prevents adduction |
| **PTFL** | -- | **63.9 +/- 38.0 N/mm** | Strongest lateral ligament |
| **Deltoid (ATTL)** | Higher than PTTL | -- | Deep layer: greatest restraint against lateral translation |
| **Deltoid (PTTL)** | ~50% of ATTL | -- | Deep layer component |
| **General ligament** | **1.0-2.0 GPa** (tissue level) | -- | Ultimate tensile strength: 50-150 MPa |

**Nonlinear modeling (recommended for FE):**
- Ligaments are best modeled with nonlinear elastic response: `F = A * (e^(B*epsilon) - 1)`
  - Where F = force, epsilon = strain, A and B are material-specific constants
- Zero-force region (toe region): 0.33-3.84 mm (relative length: 6.7 +/- 3.9%)
- Maximum strain before failure: 12-15%
- Strain without damage: 5-7%

**Tendon properties (for reconstruction reference):**

| Tendon | Elastic Modulus |
|---|---|
| Posterior tibial | 2,076 MPa |
| Peroneus longus | 2,769 MPa |
| Plantaris | 1,172 MPa |
| Extensor pollicis longus | 450 MPa |

### 4.4 Implant Material Properties

| Material | Young's Modulus | Tensile Strength | Poisson's Ratio | Use in TAR |
|---|---|---|---|---|
| **Cobalt-Chromium (CoCrMo)** | **200-250 GPa** | 400-1,000 MPa | ~0.30 | Tibial and talar component base |
| **Titanium (Ti-6Al-4V)** | **110-116 GPa** | ~835 MPa (3D-printed) | ~0.34 | Plasma spray coating; some component bodies |
| **Beta-Titanium alloys** | **12-80 GPa** | Variable | -- | Newer low-modulus designs |
| **UHMWPE** | **0.8-1.0 GPa** | Low | ~0.46 | Polyethylene bearing/insert |
| **Stainless Steel 316L** | **200-210 GPa** | -- | ~0.30 | X-ray markers; older designs |

**Stress shielding context:** CoCr (210 GPa) is ~10-15x stiffer than cortical bone (17-20 GPa) and ~200x stiffer than cancellous bone (~1 GPa). This mismatch drives research into porous/3D-printed metal structures and lower-modulus alloys.

### 4.5 Typical Resection Depths and Angles

| Parameter | Value | Notes |
|---|---|---|
| **Tibial resection depth** | 2-10 mm (modern systems); STAR reference: ~5 mm from plafond apex | FE studies model 6-16 mm |
| **Tibial cut angle (coronal)** | 90 degrees to mechanical axis | Beta angle = 90 degrees |
| **Tibial cut angle (sagittal)** | 89 +/- 3 degrees (ADTA) | Acceptable: 86-92 degrees |
| **Talar resection depth** | Minimal; ~2 mm medial side; system-dependent | Goal: preserve bone stock |
| **Incremental adjustment** | 2 mm increments | For fine-tuning joint tension |
| **PE bearing thickness range** | 6-10 mm (STAR); 7-12 mm typical across systems | Determines joint-line height |
| **PSI accuracy standard** | +/- 2 mm or +/- 2 degrees of pre-op plan | Patient-specific instrumentation target |

### 4.6 Alignment Parameters

| Parameter | Target | Acceptable Range |
|---|---|---|
| **Tibiotalar angle (coronal)** | 0 degrees (neutral) | **< 10 degrees** varus or valgus |
| **Beta angle (tibial component, coronal)** | 90 degrees | -- |
| **Anterior distal tibial angle (sagittal)** | 89 degrees | **86-92 degrees** |
| **Tibiotalar congruence** | < 2 degrees | -- |
| **Component migration threshold** | -- | **> 5 degrees angular change** = migration/subsidence |
| **Posterior slope (sagittal)** | < 5 degrees | > 5 degrees considered malalignment |

---

## 5. Clinical Outcome Metrics

### 5.1 AOFAS Ankle-Hindfoot Score (0-100)

| Component | Maximum Points |
|---|---|
| Pain | 40 points |
| Function (including 16 pts for hindfoot motion) | 50 points |
| Hindfoot alignment | 10 points |
| **Total** | **100 points** (best clinical state) |

| Timepoint | Typical Score |
|---|---|
| Pre-operative | **33-45** |
| Post-operative (1-2 years) | **82-98** |
| Mean improvement | ~40-55 points |

**Implant-specific AOFAS results:**

| Implant | Pre-op AOFAS | Post-op AOFAS | Follow-up |
|---|---|---|---|
| STAR | 43.7 +/- 9.4 | 84.9 +/- 8.1 | Long-term (mean 159 months) |
| Vantage | 42.1 +/- 2.4 (SE) | 96.0 +/- 0.8 (SE) | 12 months |
| Zimmer TM | 33.8 +/- 14.3 | 88.5 +/- 6.6 | 24 months |
| Zenith | 39.4 +/- 10.2 | 82.8 +/- 14.0 | Variable |

**Note:** The AOFAS Board of Directors no longer endorses the AOFAS score due to insufficient reliability and validity. However, it remains the most frequently reported outcome measure in TAR literature.

### 5.2 VAS Pain Score (0-10)

| Timepoint | Typical Score |
|---|---|
| Pre-operative | **6.7-7.6** |
| Post-operative (1-2 years) | **0.2-2.1** |
| Mean reduction | ~5-7 points (all studies p < 0.001) |

### 5.3 ROM Improvement

| Measure | Pre-op | Post-op | Change |
|---|---|---|---|
| Total sagittal ROM (mean) | 22-31 degrees | 33-53 degrees | **+4 to +14 degrees** (modest) |
| One large series (n=357) | 31.3 degrees | 33.9 degrees (1 yr) | +2.6 degrees |
| Favorable series (n=103) | 24.9 degrees | 52.9 degrees | +28.0 degrees |

**Clinical caveat:** Systematic reviews recommend that patients be advised that **significant ROM increase is not a reliable expected benefit** of TAR. The primary benefit is pain relief. Patients with lower pre-op ROM tend to experience higher residual pain at follow-up.

### 5.4 Radiographic Alignment Criteria

| Parameter | Acceptable Post-Op Value |
|---|---|
| Tibiotalar angle (coronal) | **< 10 degrees** varus/valgus |
| Anterior distal tibial angle (sagittal) | **89 +/- 3 degrees** |
| Tibiotalar congruence | **< 2 degrees** incongruence |
| Component migration | **< 5 degrees** angular change from immediate post-op |
| Posterior slope | **< 5 degrees** |
| Radiolucent lines | < 2 mm non-progressive = acceptable; progressive = concern for loosening |
| Periprosthetic osteolysis | Absent = ideal; present in up to 50% of STAR at 10+ years |

### 5.5 Survivorship and Complications

#### Implant Survival Rates

| Timeframe | Survival Rate |
|---|---|
| 5-year (large studies) | **90-95%** |
| 10-year (large studies) | **86-94%** |
| 15-year | **61-73%** |
| UK NJR registry (5-year) | 90.2% |
| UK NJR registry (10-year) | 86.2% |

#### Complication Frequency (meta-analysis)

| Complication | Frequency |
|---|---|
| Technical error | 28.15% |
| Subsidence | 16.89% |
| Implant failure | 13.28% |
| Aseptic loosening | 6.3% |
| Intraoperative fracture | 5.67% |
| Wound problems | 4.3-9% |
| Residual pain | 27-60% |
| Deep infection | 1-4.6% |

**Risk factors for failure:** Younger age, chronic pulmonary disease, diabetes, heavy alcohol consumption, prior operative treatment.

---

## Sources

### Surgical Technique and Procedure
- [Ankle Arthroplasty - StatPearls (NCBI Bookshelf)](https://www.ncbi.nlm.nih.gov/books/NBK606105/)
- [Total Ankle Replacement: Why, When and How? (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC2958283/)
- [Avoiding Wound Complications in Total Ankle Arthroplasty (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC7821969/)
- [Surgical Technique for TAR in Ankles with Varus Deformity >= 10 degrees (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC6407960/)
- [Current Trends in Total Ankle Replacement (RadioGraphics)](https://pubs.rsna.org/doi/full/10.1148/rg.230111)
- [STAR Operative Technique (Stryker)](https://www.stryker.com/content/dam/stryker/foot-and-ankle/products/star/resources/eu/STAR%20operative%20technique.pdf)
- [STAR FDA Summary of Safety and Effectiveness](https://www.accessdata.fda.gov/cdrh_docs/pdf5/P050050b.pdf)
- [Accuracy of Patient-Specific Instrument Resections in TAA (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC12123138/)

### Implant Systems
- [Design Rationale for Total Ankle Arthroplasty Systems (SOGACOT)](https://sogacot.org/articulos/design-rationale-for-total-ankle-arthroplasty-systems/)
- [Review Study on Total Ankle Replacement (MDPI Applied Sciences)](https://www.mdpi.com/2076-3417/13/1/535)
- [Evaluation of TAR in the Modern Era (Annals of Translational Medicine)](https://atm.amegroups.org/article/view/115549/html)
- [Infinity Total Ankle System (Stryker)](https://www.stryker.com/us/en/foot-and-ankle/products/infinity.html)
- [Total Ankle Arthroplasty with Salto Talaris Prosthesis (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC6407948/)
- [Novel Approach to Determine Components Size in TAR (Nature Scientific Reports)](https://www.nature.com/articles/s41598-025-13004-4)

### Biomechanics and Anatomy
- [Biomechanics of the Ankle (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC4994968/)
- [Anatomy of the Ankle Ligaments: A Pictorial Essay (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC2855022/)
- [Open and Arthroscopic Surgical Anatomy of the Ankle (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC3830799/)
- [Ankle Joint Anatomy (Medscape)](https://emedicine.medscape.com/article/1946201-overview)
- [Deltoid Complex Anatomy (LaPrade)](https://drrobertlaprademd.com/wp-content/uploads/2015/07/ligament-anatomy-deltoid-complex-ankle-qualitative-quantitative-anatomical-study-2014.pdf)
- [Biomechanics of the Foot and Ankle (OrthoPaedia)](https://www.orthopaedia.com/biomechanics-of-the-foot-and-ankle/)
- [Joint Angle, ROM, Force, and Moment Assessment (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC8476262/)

### Material Properties
- [Bone Mechanical Properties in Healthy and Diseased States (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC6053074/)
- [Young's Modulus of Trabecular and Cortical Bone (PubMed)](https://pubmed.ncbi.nlm.nih.gov/8429054/)
- [Bone Material Property (ScienceDirect)](https://www.sciencedirect.com/topics/engineering/bone-material-property)
- [Critical Evaluation of Known Bone Material Properties (MIT/Wirtz 2000)](http://web.mit.edu/awinter/Public/MGH%20Literature/Wirtz-2000-Evaluation%20of%20known%20bone%20material%20properties.pdf)
- [Biomechanical Topography of Human Ankle Cartilage (Springer)](https://link.springer.com/article/10.1007/BF02584467)
- [Biomechanics of Cartilage (MIT)](https://web.mit.edu/cortiz/www/3.052/3.052CourseReader/27_BiomechanicsofCartilage.pdf)
- [ATFL Thickness and Elastic Modulus Effect on Ankle Joint Stiffness (PMC/Frontiers)](https://pmc.ncbi.nlm.nih.gov/articles/PMC10166853/)
- [Deltoid Ligament Injury and Rotational Ankle Instability FE Model (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC11094218/)
- [Titanium-Based Alloys for Orthopedic Implants (ScienceDirect)](https://www.sciencedirect.com/science/article/pii/S0264127524002235)
- [Elastic Modulus in Interbody Implant Selection (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC5506312/)
- [Mechanical Properties of Ligament and Tendon (Musculoskeletal Key)](https://musculoskeletalkey.com/mechanical-properties-of-ligament-and-tendon/)

### Clinical Outcomes
- [Outcomes of Total Ankle Replacement -- Current Evidence (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC10806254/)
- [Most Frequently Used Outcome Instruments in TAA Studies (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC2816756/)
- [Long-Term Survival Analysis of 5619 TAA (MDPI JCM)](https://www.mdpi.com/2077-0383/13/1/179)
- [TAA Survivorship Meta-analysis (PubMed)](https://pubmed.ncbi.nlm.nih.gov/32600863/)
- [Long-term Outcomes After TAA Systematic Review (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC11544666/)
- [How Successful are Current Ankle Replacements? Systematic Review (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC2795846/)
- [Survival of Primary Ankle Replacements: Global Joint Registries (JFAR)](https://jfootankleres.biomedcentral.com/articles/10.1186/s13047-022-00539-2)
- [Total Ankle Arthroplasty Imaging Overview (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC4842860/)
- [Postoperative Evaluation of TAA (AJR)](https://ajronline.org/doi/10.2214/AJR.07.2729)

### Radiographic Alignment
- [TAA Radiographic Alignment: PSI vs Standard (PubMed)](https://pubmed.ncbi.nlm.nih.gov/33749342/)
- [Ankle Alignment on Lateral Radiographs (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC2274959/)
