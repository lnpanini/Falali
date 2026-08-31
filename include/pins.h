// GPIO map for ESP32-S3-WROOM-1 N16R8 on the Wheel Drive PCB.
//
// *** AUTHORITATIVE SOURCE: the manufactured board's KiCad netlist ***
// Extracted from "Wheel Drive PCB.net" (Eeschema 9.0.6, 2026-08-05). Every
// assignment below was read out of the netlist, not inferred from a schematic
// image. If the board is respun, re-extract rather than hand-editing.
//
// The map that used to live here was the PRE-PCB design intent and does not
// match the fabricated board at all — it had SV on 4/5/6/7 (actually the ToF
// XSHUT lines) and I2C on 38/39 (actually FR BRK and FL BRK). Flashing that
// version would have driven the I2C bus as motor enables.
//
// RESERVED on this module — the board correctly leaves all of these unconnected:
//   GPIO26–37  flash + OCTAL PSRAM ("R8")
//   GPIO19/20  native USB
//   GPIO0/45/46  strapping
#pragma once

#include <cstdint>

namespace pins {

// Sentinel for "not wired on our hardware". Adapters must check for it rather
// than configuring a GPIO that has nothing on the other end.
constexpr uint8_t kNoPin = 0xFF;

// --- Wheel motors: 4× BLD-120A ---
// Index order matches fal::Corner — FL=0, FR=1, RL=2, RR=3.
//
// NOTE: EN and BRK are PER-WHEEL on this board, not ganged as the original
// design assumed. That is strictly better (independent shutdown per wheel) but
// means main.cpp must pass each motor its own pins, not one shared pair.
//
// *** EN / BRK / F-R IDLE ABOVE 3.3 V — GO THROUGH THE TRANSISTOR ADAPTERS. ***
// The manual draws each control input as an optocoupler LED fed from the driver's
// own rail. Bench attempts to characterise that rail gave contradictory readings
// and have been discarded, so the exact idle voltage and pull-up are UNKNOWN —
// but it is above 3.3 V and the ESP32-S3 is not 5 V tolerant, which is all the
// justification the adapters need.
//
// Never wire these straight to a GPIO. The transistor INVERTS the logic; see
// kControlViaMosfet in Bld120aMotor.h.
constexpr uint8_t kWheelSV[4]  = {42, 21, 18, 10};  // FL, FR, RL, RR — native PWM 1–10 kHz
constexpr uint8_t kWheelFR[4]  = {41, 47, 17, 11};  // direction
constexpr uint8_t kWheelEN[4]  = {40, 48, 16, 12};  // enable  (per wheel)
constexpr uint8_t kWheelBRK[4] = {39, 38, 15, 13};  // brake   (per wheel)

// ALARM — NOT WIRED, and there is no terminal for it. The BLD-120A's control
// block exposes only SV/COM/F-R/EN/BRK; "RUN/ALM" is an LED, not an output
// (manual + bench-confirmed 2026-07-27). The board has no ALARM net either.
//
// RE-CONFIRMED 2026-08-16 against the BLD-120A spec sheet's full port table:
// the only terminals are SV/COM/F-R/EN/BRK, REF+/HU/HV/HW/REF-, W/V/U, DC+/DC-.
// No FG, no ALM.
//
// *** IGNORE the feature bullets in docs/BLD-120-English-version.pdf. ***
// Its intro page advertises "Speed signal output" and "Abnormal alarm signal
// output". Both are boilerplate from a family datasheet and contradict the
// terminal table in the SAME document. Do not go looking for those pins.
//
// *** SO THERE IS NO MOTOR FAULT DETECTION IN HARDWARE. ***
// Protection comes from the driver's own P-sv overload trim (set it to the
// motor's rated watts) and from the ACS758 -> ADS1115 current sense below
// (cfg::kWheelStallAmps). With the encoders retired there is no speed feedback
// at all, so current is the ONLY drivetrain fault signal this board has. Do not
// run the drivetrain unattended.
constexpr uint8_t kWheelALARM = kNoPin;

// --- CLAMP / LIMIT SWITCHES / E-STOP — NOT ON THIS BOARD ---
//
// The Wheel Drive PCB is wheels + sensors only; the netlist has no connector for
// any of these. Per the Pi-5 architecture the clamp belongs to ESP-ARM, which is
// not built yet.
//
// They are kNoPin rather than absent so main.cpp still compiles as the one-ESP
// firmware. The HAL adapters no-op on kNoPin, so nothing can be actuated.
//
// *** DO NOT restore the pre-PCB values (11/12/13, 1/2, 14/21, 47). ***
// On the fabricated board every one of those is now a wheel or encoder signal:
//   GPIO11=RR F/R  12=RR EN  13=RR BRK  14=Encoder RR (now free)
//   GPIO21=FR SV   47=FR F/R
//   GPIO1/2 = FL/FR encoder headers (free again since the arm link went radio)
// The clamp would fight the rear-right wheel.
constexpr uint8_t kClampRPWM     = kNoPin;
constexpr uint8_t kClampLPWM     = kNoPin;
constexpr uint8_t kClampEN       = kNoPin;
constexpr uint8_t kClampIS_Close = kNoPin;
constexpr uint8_t kClampIS_Open  = kNoPin;
constexpr uint8_t kLimitOpen     = kNoPin;
constexpr uint8_t kLimitClosed   = kNoPin;
constexpr uint8_t kEstop         = kNoPin;
constexpr uint8_t kClampRPWMCh   = 4;   // LEDC channels, unused while kNoPin
constexpr uint8_t kClampLPWMCh   = 5;

// --- ToF: 4× VL53L0X, individual XSHUT for address re-assignment ---
// All four share the I2C bus at 0x29 and are brought up one at a time via XSHUT,
// which is exactly what lib/hal_esp32/Vl53l0xArray.h implements. No mux needed.
//
// *** THE REFERENCE DESIGNATORS ARE NOT THE GPIO NUMBERS. ***
// CORRECTED 2026-08-16 from the netlist and the PCB's silkscreen layer. The
// connectors are TOF5..TOF8 (there is no TOF4), and no arithmetic relates a
// designator to its GPIO:
//
//   designator   TOF5  TOF6  TOF7  TOF8
//   corner       FL    FR    RL    RR
//   XSHUT GPIO   4     5     6     7
//
// The functional silkscreen IS reliable, and is what you should read on the
// board: pin 6 of each connector is labelled "XSHUT FL" / "XSHUT FR" /
// "XSHUT RL" / "XSHUT RR". Corner identity is unambiguous.
//
// GPIO8 is kI2C_SDA below and can never be an XSHUT: pulling it low during
// Vl53l0xArray::begin()'s sequencing would take the whole I2C bus down, ToF and
// IMU and ADS together.
constexpr uint8_t kTofXSHUT[4] = {4, 5, 6, 7};      // FL, FR, RL, RR

// --- Shared I2C bus: ToF + IMU + current-sense ADC ---
constexpr uint8_t kI2C_SDA = 8;
constexpr uint8_t kI2C_SCL = 9;

// --- 4x ACS758 -> ADS1115 at 0x48: which connector feeds which ADC channel ---
//
// CORRECTED 2026-08-16 from KiCad/Wheel Drive PCB/Wheel Drive PCB.net. The
// table that used to sit here listed the connectors as J14/J17/J12/J9 and
// claimed they were "NOT in numeric order". The netlist says otherwise -- they
// are in order, and the previous note was a misreading of the silkscreen:
//
//   connector    J9    J12   J14   J17     <- schematic names them FL FR RL RR
//   corner       FL    FR    RL    RR
//   ADS channel  A0    A1    A2    A3
//
// The kCurrentAdsChannel values below were always correct; only the comment
// was wrong. Netlist evidence, one line per channel:
//
//   /A0 : J8.7  J9.4      /A1 : J12.4 J8.8
//   /A2 : J14.4 J8.9      /A3 : J17.4 J8.10
//
// On the board itself, read the A0..A3 silkscreen next to the connector -- every
// pin on this PCB is functionally labelled. Do not infer a channel from a
// J-number. Note pcb_identify.cpp can no longer confirm a corner on its own: it
// used to watch an encoder AND a current channel move together, the encoder half
// was deleted with the encoders, and the ACS758 outputs are not wired either --
// so both halves of that check are currently unavailable.
//
// kAdsAddr is fixed in copper, not by a jumper: J8 pin 5 is the module's ADDR
// pin and the board ties it to ground. [VERIFIED -- netlist]
constexpr uint8_t kAdsAddr = 0x48;
constexpr uint8_t kCurrentAdsChannel[4] = {0, 1, 2, 3};  // FL, FR, RL, RR

// A TCA9548A mux at 0x70 was fitted for the encoders (all four AS5600 are
// hard-wired to 0x36 with no address pin, and this board commons their SDA/SCL,
// so they could not be addressed individually as built). Both the encoders and
// the mux were retired; kMuxAddr was removed 2026-08-31 once nothing read it.
// The ToF sensors never needed it — they use XSHUT re-addressing, see above.

// --- ENCODERS: REMOVED 2026-08-13. Do not reinstate without reading this. ---
//
// The four AS5600 never worked on the fabricated board and the project has moved
// on without wheel feedback. Their pin tables are deleted rather than left
// commented out, because a pin table that describes hardware nobody drives is
// indistinguishable from one that describes hardware somebody does.
//
// All four encoder pins -- GPIO1, 2, 3 and 14 -- are free again as of
// 2026-08-14, when the base<->arm link moved from UART on GPIO1/2 to ESP-NOW.
// Caveats if anything claims them:
//
//   GPIO1/2   routed to the FL/FR encoder HEADERS. Safe to drive as long as
//             nothing is plugged into those headers -- an AS5600's OUT pin is an
//             actively driven analog output and would fight whatever else is on
//             the net.
//   GPIO3     strapping pin (JTAG source select)
//   GPIO14    ADC2 -- stops working the moment WiFi is enabled, and bench_ble
//             now enables WiFi for ESP-NOW. Treat it as digital-only.

// --- Free on the board, available if anything needs relocating ---
//   GPIO0, 3, 14, 35, 36, 37, 43, 44, 45, 46
//     0/45/46  strapping — avoid
//     3        strapping (JTAG source select)
//     14       ADC2 — dead once WiFi is on
//     35–37    PSRAM — avoid
//   Genuinely clean spares: GPIO43, GPIO44 — but ONLY in a build whose console
//   is native USB-CDC. bench_ble's is not (CONFIG_ESP_CONSOLE_UART_NUM=0 puts
//   the IDF console on UART0 = GPIO43/44), which is why the arm link uses 1/2.

// --- LEDC PWM configuration ---
//
// 12-BIT, NOT 8. With 150 mm wheels the drivetrain reaches ~1400 mm/s but the
// docking sequence only asks for ~300 mm/s, so the whole useful command range
// sits in the bottom ~20% of full scale. Resolution here is free: LEDC's ceiling
// is log2(80 MHz / f_pwm) bits — ~13.9 at 5 kHz, ~16 at 1 kHz.
constexpr uint8_t kPwmResBits = 12;     // duty 0..4095

// Clamp H-bridge (BTS7960) — a genuine PWM input, switching the motor directly.
constexpr uint32_t kPwmFreqHz = 1000;

// Wheel SV — the driver's NATIVE PWM speed input.
//
// *** FREQUENCY RANGE RESOLVED 2026-08-16: 1-3 kHz. 2000 IS CORRECT. ***
// Three documents, two of which now agree against the third:
//   docs/BLDC-BLD120A-...-specs.pdf  : "The pulse frequency range: 1-3KHz"  <- BLD-120A specific
//   docs/SYS-BLD-120A-manual.pdf     : "PWM 幅值 5V 频率 1~3KHz"            (amplitude 5 V, 1-3 kHz)
//   docs/BLD-120-English-version.pdf : "PWM control between 1KHz~10KHz"     <- the outlier
// A previous revision used 5 kHz, which is OUTSIDE the real range, and before
// that 20 kHz, outside all three.
//
// The spec sheet also advises a DUTY RANGE of 2%-90%: 2% duty = 5% of top speed,
// 90% duty = maximum. Commanding above 90% buys nothing.
//
// *** OPEN: our 3.3 V SV drive may cost ~30% of top speed. [HYPOTHESIS] ***
// Both remaining manuals specify 5 V PWM amplitude, and the driver's analog
// curve runs 0.25 V = 5% speed to 4.7 V = maximum. SV is driven straight from a
// GPIO through a 1.5k series resistor with NO level shifting, so it swings ~3.3 V.
//   - if the driver reads DUTY   -> 90% duty gives 3000 RPM -> ~1571 mm/s
//   - if it reads AVERAGE VOLTS  -> 3.3 V gives ~2100 RPM   -> ~1100 mm/s
// The measured max_lin_mm_s estimates (1155, 1231, 1384, 1657) cluster near the
// LOWER prediction, which is suggestive but not decisive.
// Experiment: level-shift SV to 5 V on one wheel and re-measure top speed. If it
// rises, every wheel has been running at ~70% of capability.
// See docs/hardware/components.md section 2.
//
// Required alongside it (manual, Attentions E — stated three separate times):
//   *** TURN THE ONBOARD RV POT FULLY LEFT, or external speed control FAILS ***
// plus a 1–10 kΩ series resistor between GPIO and SV (manual FAQ A).
//
// SV input impedance: UNVERIFIED. A bench diode test on 2026-08-05 read open in
// both directions, suggesting a high-impedance voltage input — but a later
// powered test on the same terminals gave conducting readings, and the two
// cannot both be right. Treat neither as established.
//
// It does not matter in practice: 3.3 V PWM at 2 kHz drives all four motors
// through the transistor adapters, which is verified by behaviour. Re-measure
// only if you need the driver's actual input model for a redesign.
constexpr uint32_t kSvPwmFreqHz = 2000;

// LEDC channels — used by Arduino-ESP32 core 2.x, ignored by the 3.x pin API.
constexpr uint8_t kWheelPwmCh[4] = {0, 1, 2, 3};

} // namespace pins
