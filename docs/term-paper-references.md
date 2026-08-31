# Term paper — reference list (IEEE style)

**Status key**

- **✓ verified** — URL and details checked against the live source on 21 Aug 2026.
- **⚠ verify** — a standard work cited from a known bibliography. The content is right;
  confirm the **edition and year** against the copy you actually consult before submitting.
- Do not cite anything here you have not at least looked at. A reference list is a claim
  that you read them.

Numbering below is grouped by section for convenience. Renumber in citation order —
IEEE requires references numbered in the order they first appear in the text.

---

## A. Introduction and context (§I, §II)

[1] Grand View Research, "Autonomous mobile robots market size, share & trends analysis
report," Grand View Research, San Francisco, CA, USA, 2025. [Online]. Available:
https://www.grandviewresearch.com/industry-analysis/autonomous-mobile-robots-market
[Accessed: Aug. 21, 2026]. **✓ verified**

[2] ABB Robotics, "Flexley Mover AMR P603," ABB Ltd. [Online]. Available:
https://www.abb.com/global/en/areas/robotics/products/mobile-robots/flexley-mover/amr-p603
[Accessed: Aug. 21, 2026]. **✓ verified** — *note: the P603 is an **ABB** product; the
current draft attributes it to no manufacturer. Payload 1500 kg, top speed 2 m/s,
±5 mm positioning, ISO 3691-4 compliant — all citable from this page.*

[3] Seegrid Corporation, "Palion Tow Tractor S7 autonomous mobile robot," Seegrid Corp.,
Pittsburgh, PA, USA. [Online]. Available:
https://seegrid.com/autonomous-mobile-robots/tow-tractor-s7-amr/
[Accessed: Aug. 21, 2026]. **✓ verified** — *the draft spells this "Seagrid". It is
**Seegrid**, and the product line is **Palion**. Fix in §I.*

[4] B. E. Ilon, "Wheels for a course stable selfpropelling vehicle movable in any desired
direction on the ground or some other base," U.S. Patent 3 876 255, Apr. 8, 1975.
**✓ verified**

[5] Industrial trucks — Safety requirements and verification — Part 4: Driverless
industrial trucks and their systems, ISO 3691-4:2023, International Organization for
Standardization, Geneva, Switzerland, 2023. **✓ verified** — *worth citing in §V: it is
the standard your safety discussion is measured against, and naming it shows you know
the prototype is not yet compliant.*

[6] R. Siegwart, I. R. Nourbakhsh, and D. Scaramuzza, *Introduction to Autonomous Mobile
Robots*, 2nd ed. Cambridge, MA, USA: MIT Press, 2011. **⚠ verify edition** — *general
support for AMR vs. AGV navigation, wheel kinematics, dead reckoning.*

[7] A. Gfrerrer, "Geometry and kinematics of the Mecanum wheel," *Computer Aided Geometric
Design*, vol. 25, no. 9, pp. 784–791, Dec. 2008. **⚠ verify** — *the standard analytical
treatment of mecanum roller geometry; supports the omnidirectional-motion claim in §II.C.1.*

---

## B. Drivetrain and mechanical modelling (§III.A, §III.B)

[8] T. D. Gillespie, *Fundamentals of Vehicle Dynamics*. Warrendale, PA, USA: Society of
Automotive Engineers, 1992. **⚠ verify** — *source for the µ = 0.015 rolling-resistance
coefficient. At present that figure appears in §III.A with no citation at all, which is
the most exposed number in the mechanical section.*

[9] R. G. Budynas and J. K. Nisbett, *Shigley's Mechanical Engineering Design*, 11th ed.
New York, NY, USA: McGraw-Hill Education, 2020. **⚠ verify edition** — *bolted-joint shear
capacity and safety-factor practice, for Tables 3 and 4.*

[10] R. C. Hibbeler, *Mechanics of Materials*, 10th ed. Hoboken, NJ, USA: Pearson, 2017.
**⚠ verify edition** — *classical beam theory for the chassis deflection model in the
abstract and §III.B.*

[11] ASM International, "Aluminum 6061-T6; 6061-T651," *ASM Aerospace Specification Metals
Handbook*. [Online]. Available: https://asm.matweb.com [Accessed: Aug. 21, 2026].
**⚠ verify the exact page** — *source for the 207 MPa shear strength used in the Phase 3
clamp calculation.*

[12] Gates Corporation, *Design Manual: Synchronous Belt Drives*, Gates Corp., Denver, CO,
USA. [Online]. Available: https://www.gates.com [Accessed: Aug. 21, 2026].
**⚠ verify** — *HTD 3M belt working-tension figure in §III.B. Note the paper's Table 4
labels this row "GT2 belt" while the text says HTD3M — pick one and cite accordingly.*

---

## C. Electronics and power (§II.C.3, §III.A) — your section

[13] Espressif Systems, *ESP32-S3 Series Datasheet*, v1.6, Espressif Systems (Shanghai)
Co., Ltd., 2024. [Online]. Available:
https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf
[Accessed: Aug. 21, 2026]. **⚠ verify version number**

[14] Espressif Systems, "ESP-NOW," *ESP-IDF Programming Guide*. [Online]. Available:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/network/esp_now.html
[Accessed: Aug. 21, 2026]. **⚠ verify** — *supports the inter-controller link description
and the 250-byte payload / channel constraints.*

[15] *BLD-120A Brushless DC Motor Controller Specification*, manufacturer datasheet.
**✓ in your possession** — `docs/BLDC-BLD120A-bldc-motor-controller-specs.pdf`. *Cite this
one, not `BLD-120-English-version.pdf`: the two disagree on PWM input range and the
BLD-120A-specific sheet is the one your 2 kHz figure follows.*

[16] *80 mm Flange Brushless DC Gearmotor Specification*, supplier datasheet.
**✓ in your possession** — `docs/hardware/img/motor-80-flange-spec-table.jpeg`. *Source for
120 W, 5 A rated, 0.4 N·m base torque, 15:1 ratio.*

[17] STMicroelectronics, *VL53L0X: World's Smallest Time-of-Flight Ranging and Gesture
Detection Sensor*, DS11555, STMicroelectronics, 2018. [Online]. Available:
https://www.st.com/resource/en/datasheet/vl53l0x.pdf [Accessed: Aug. 21, 2026].
**⚠ verify document number**

[18] CEVA, Inc., *BNO08X Datasheet*, CEVA/Hillcrest Laboratories. [Online]. Available:
https://www.ceva-ip.com [Accessed: Aug. 21, 2026]. **⚠ verify** — *the BNO085 sensor-fusion
IMU; the datasheet is published under the BNO08X family name.*

[19] Allegro MicroSystems, *ACS758: Thermally Enhanced, Fully Integrated, Hall-Effect-Based
Linear Current Sensor IC*, Allegro MicroSystems, LLC. [Online]. Available:
https://www.allegromicro.com [Accessed: Aug. 21, 2026]. **⚠ verify** — *supports the
26.4 mV/A ratiometric sensitivity and ±50 A range in your sensor-selection argument.*

[20] Texas Instruments, *ADS111x Ultra-Small, Low-Power, I2C-Compatible, 860-SPS,
16-Bit ADCs with Internal Reference, Oscillator, and Programmable Comparator*, SBAS444,
Texas Instruments Inc. [Online]. Available: https://www.ti.com/lit/ds/symlink/ads1115.pdf
[Accessed: Aug. 21, 2026]. **⚠ verify document number**

[21] NXP Semiconductors, *PCA9685: 16-channel, 12-bit PWM Fm+ I2C-bus LED controller*,
Rev. 4, NXP Semiconductors N.V., 2015. [Online]. Available:
https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf [Accessed: Aug. 21, 2026].
**⚠ verify revision**

[22] Infineon Technologies, *BTN7960B/BTS7960B High Current PN Half Bridge NovalithIC™*,
Infineon Technologies AG. [Online]. Available: https://www.infineon.com
[Accessed: Aug. 21, 2026]. **⚠ verify** — *the 43 A H-bridge modules on the arm board.*

[23] onsemi, *2N3904 General Purpose Transistor NPN Silicon*, Rev. 12, onsemi.
[Online]. Available: https://www.onsemi.com/pdf/datasheet/2n3904-d.pdf
[Accessed: Aug. 21, 2026]. **⚠ verify revision** — *supports the saturation calculation
for the Driver PCB open-collector stage (Appendix C).*

[24] NXP Semiconductors, *I2C-bus specification and user manual*, UM10204, Rev. 7, NXP
Semiconductors N.V., 2021. [Online]. Available:
https://www.nxp.com/docs/en/user-guide/UM10204.pdf [Accessed: Aug. 21, 2026].
**⚠ verify revision** — *cite where you describe XSHUT address reassignment on a shared bus.*

[25] Littelfuse, Inc., *Fuseology Selection Guide*, Littelfuse, Inc., Chicago, IL, USA.
[Online]. Available: https://www.littelfuse.com [Accessed: Aug. 21, 2026].
**⚠ verify** — *this is the citation that turns your fuse table from assertion into method:
the 1.5× continuous-current rule and, importantly, the time-delay vs. fast-acting
distinction that the four 10 A driver fuses depend on.*

[26] Low-voltage electrical installations — Part 5-52: Selection and erection of electrical
equipment — Wiring systems, IEC 60364-5-52:2009, International Electrotechnical
Commission, Geneva, Switzerland, 2009. **⚠ verify** — *conductor ampacity and derating,
for the pack-conductor gauge row of Table 6. Singapore's SS 638 derives from BS 7671,
which follows this; cite whichever your library gives you access to.*

[27] T. L. Bergman and A. S. Lavine, *Fundamentals of Heat and Mass Transfer*, 8th ed.
Hoboken, NJ, USA: Wiley, 2017. **⚠ verify edition** — *natural-convection correlations
behind the "15–25 W in still air" claim in the thermal paragraph, which is currently
uncited.*

[28] Secondary cells and batteries containing alkaline or other non-acid electrolytes —
Safety requirements for portable sealed secondary lithium cells, and for batteries made
from them, for use in portable applications — Part 2: Lithium systems, IEC 62133-2:2017,
International Electrotechnical Commission, Geneva, Switzerland, 2017. **⚠ verify** —
*optional, but it gives you something to point at in §V when you note the custom pack has
no documented ratings.*

---

## D. Where each reference belongs in the text

| Claim currently uncited | Cite |
|---|---|
| AMR market growth (§I) | [1] |
| "Flexley Mover P603 … Seagrid S7" (§I) | [2], [3] — and fix both names |
| Mecanum omnidirectional capability (§I, §II.C.1) | [4], [7] |
| AMR vs. AGV distinction (§I) | [6] |
| µ = 0.015 rolling resistance (§III.A) | [8] |
| Gearbox efficiency η = 0.88 "2-stage planetary estimate" (§III.A) | [16] if the datasheet states it; otherwise label it an assumption, not a spec |
| 0.38 / 0.4 N·m rated torque, 120 W (§III.A) | [16] |
| Motor natural convection 15–25 W (§III.A) | [27] |
| 6061-T6 shear strength 207 MPa (§III.B) | [11] |
| M4 grade 8.8 bolt shear capacity (§III.B) | [9] |
| HTD3M belt working tension 60 N (§III.B) | [12] |
| PWM 1–3 kHz, 2–90 % duty, P-sv range (§II.C.3) | [15] |
| ToF sensing and XSHUT re-addressing (§II.C.3) | [17], [24] |
| Current-sensor selection argument (§II.C.3) | [19], [20] |
| ESP-NOW link (§II.C.3) | [13], [14] |
| Fuse sizing rule and time-delay requirement (§III.A, Table 6) | [25] |
| Conductor gauge (Table 6) | [26] |
| Safety limitations, E-stop discussion (§V) | [5] |

---

## E. Two corrections the reference hunt turned up

1. **The Flexley Mover P603 is an ABB product**, and it meets ISO 3691-4 with a 1500 kg
   payload and ±5 mm positioning. Your §I uses it as an example of a platform that
   "cannot fit under a trolley" — that is a fair claim about form factor, but state the
   manufacturer and give the payload, or a reader familiar with the product will read the
   sentence as vague.
2. **"Seagrid" is Seegrid**, and the product is the Palion Tow Tractor S7. A misspelled
   competitor name in the first page of a report is a cheap thing to lose credibility on.

## F. What is still missing

The strongest citation you do not yet have is a **peer-reviewed AMR docking or
trolley-coupling paper** — something that places your clamp-and-tow approach in the
literature rather than presenting it as invented from nothing. Search IEEE Xplore for
"autonomous cart docking", "AGV automatic hitching", and "under-cart AMR coupling"
through the SUTD library; one or two results there would strengthen §I and §V more than
any additional datasheet.
