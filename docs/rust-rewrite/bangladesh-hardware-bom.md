# Bangladesh hardware sourcing and BOM

Checked: 2026-10-05. Scope: one ESP32-S3/MPU6050 bench system, followed by a
candidate brushed micro quadcopter. This is the complete project shopping list,
not an exhaustive catalog of electronics sold in Bangladesh.

Prices are advertised BDT, excluding delivery and checkout fees. “In stock” means
the page explicitly said so; “listed” means a listing/cart was visible without
confirmed stock. Marketplace listings may ship from overseas. None of these
listings establishes authenticity, measured current, flight support or a pin map.

## 1. First purchase: controller and sensor bench

| Part | Qty | Bangladesh source / advertised price | Decision |
| --- | --- | --- | --- |
| ESP32-S3 DevKitC1 | 1 | [HTE](https://www.htebd.com/esp32-s3-devkitc1-dual-usb-type-c-development-board/): ৳1,050, explicitly in stock | First bench candidate; inspect actual module and schematic |
| ESP32-S3 DevKitC1 N16R8 | Alternative, not additional | [TechShopBD](https://techshopbd.com/product/esp32-s3-devkitc1-n16r8-development-board-dual-usb-type-c/): ৳980.04, listed | Seller advertises 16 MB flash/8 MB PSRAM; verify delivered marking |
| MPU6050 / GY-521 breakout | 1; spare optional | [HTE](https://www.htebd.com/mpu-6050-accelerometer-gyroscope-module/): ৳250, explicitly in stock; [TechShopBD](https://techshopbd.com/product/6dof-accelerometer-gyroscope-gy-521-mpu-6050): ৳320.04, **out of stock** on product page | Existing Rust IMU profile; verify chip/readback, axes and 3.3 V bus levels |
| USB data cable matching the chosen board | 1 | Local USB accessory retailer; exact cable/price not verified | Must carry data; a charge-only cable cannot flash/debug |
| Breadboard, 830 points | 1 | [TechShopBD](https://techshopbd.com/product/breadboard-830-point): ৳150.04, listed | Controller/sensor signal bench only |
| Jumper wires, female/female and male/female | 1 pack each | [HTE breadboard kit](https://www.htebd.com/basic/): ৳380 kit listing, includes breadboard/power module/jumpers | Inspect included ends; kit is an alternative to buying breadboard separately |
| Headers, short signal wire, test hooks | 1 assortment | [HTE component catalog](https://www.htebd.com/brand/hte/) | Exact SKU/price to choose after board arrives |
| Computer with USB and existing Rust/Gazebo tools | 1 existing | No purchase needed if using the current workstation | Host replay and simulation continue without electronics |

HTE board + IMU = **৳1,300**. Adding the TechShopBD breadboard gives **৳1,450.04**.
These are partial bench subtotals; they exclude cable, wire, instruments and motors.
Buy one board, not both. The large development board is useful on a bench;
fitting it to the original 65 g simulation airframe remains unverified.

## 2. Propulsion candidates: select a matched set before buying

| Part | Qty | Bangladesh sourcing evidence | Decision / missing evidence |
| --- | --- | --- | --- |
| Brushed coreless motors, matched 2 CW + 2 CCW | 4; spare pair optional | [Daraz 8520 four-piece set](https://www.daraz.com.bd/products/8520-magnetic-coreless-motor-with-abs-blade-2-cw-and-2-ccw-4-pieces-i138854538.html): ৳640, listed; [CityTechBD 8520 set](https://www.citytechbd.com/coreless-motor-with-75mm-propeller-set-of-4): ৳340, **sold out** | Daraz is a sourcing lead, not confirmed ready stock. Verify direction, shaft, loaded/stall current and thrust |
| Alternative 820 motor pairs | 2 packs of 2 | [Electronics.Com.BD](https://electronics.com.bd/820-coreless-motor): ৳350 per pair, explicitly in stock; four cost ৳700 | 820 is not automatically an 8520 mounting replacement; verify motor diameter and CW/CCW pairing |
| Matching propellers | 2 CW + 2 CCW, plus 1 spare set | [Electrapac 76 mm pair](https://www.electrapac.com/product/76mm-Propeller-For-1020-8520-Coreless-motor): ৳110/pair, page says 15 in stock; [Electronics.Com.BD 75 mm set](https://electronics.com.bd/75mm-2-blade-propeller): ৳60 listing | Confirm shaft bore, blade clearance and motor pairing. Listed GRacer frame supports only 65 mm props: these 75/76 mm candidates are not a matched frame set |
| Frame with matching motor mounts and prop clearance | 1 | [GRacer ONE v5 PRO](https://electronics.com.bd/gracer-one-mini-rc-drone-3d-printed-frame): local listing for 8520 mounts/65 mm props; [BDTronics Q100](https://www.bdtronics.com/q100-100mm-micro-quadcopter-frame-for-8520-coreless-brushed-motor.html): listing has Request Stock | Price/ready stock not confirmed here. Original Flix frame or local print is another candidate; check actual mass and S3 board fit |
| Motor mounts, fasteners, IMU mounting, battery strap | 1 set | Choose with the frame or local fabrication | Exact sizes/price unconfirmed; rigid axis alignment and secure battery required |

Retailer no-load current or RPM is insufficient to select a driver or battery.
“8520” describes a motor size family, not a single winding/current specification;
the [manufacturer example](https://www.ricmotor.com/details/8520-coreless-motor)
has materially different variants. Measure the delivered motor/prop combination.

## 3. Motor drivers and power circuit

| Part | Qty | Local lead / price | Decision |
| --- | --- | --- | --- |
| N-channel logic-level MOSFETs, AO3400 candidates | 4 plus 2 spares | [LeetechBD](https://leetechbd.com/product/ao3400-n-channel-mosfet/): ৳15 each in indexed listing; live page fetch failed | SOT-23 PCB/adapter required; stock/authenticity unconfirmed. Verify electrical and thermal suitability for measured motor current |
| Gate series resistors and gate-to-ground resistors | 4 each plus spares | Local resistor assortment, e.g. [HTE](https://www.htebd.com/brand/hte/) | Values selected with PWM/stop design; do not leave gates floating at boot |
| Flyback diodes for discrete low-side motor channels | 4 plus spares | Local diode assortment, e.g. [HTE](https://www.htebd.com/brand/hte/) | Exact diode current/voltage rating to select after motor measurements |
| Decoupling, bulk capacitors and motor noise suppression | 1 set | Local capacitor assortment, e.g. [HTE](https://www.htebd.com/brand/hte/) | Values/layout require power and noise measurements |
| Driver alternative: DRV8833 dual bridge | 2 modules instead of four discrete channels | [ElectroXBD](https://electroxbd.com/product/drv8833-motor-driver-module-electroxbd/): ৳150 each, page says 4 in stock; [ZerOneTech](https://www.zeronetechbd.com/public/product/details/drv8833-2-channel-dc-motor-driver-modulemdm010): ৳300 each, Stock Available | Bench candidate only: verify package/current, exposed nSLEEP and coast/stop wiring. Requires its own adapter/electrical profile |
| Battery: 1S LiPo with documented high discharge rating | 1; spare optional | [HTE 250 mAh](https://www.htebd.com/3-7v-250mah-1cell-lipo-bettery/): ৳396, explicitly in stock, discharge rating unspecified; [HTE 800 mAh](https://www.htebd.com/3-7v-800mah-1cell-lipo-bettery/): ৳350, explicitly in stock, **maximum 1C advertised** | **Hold flight selection.** Reject the 800 mAh/1C listing for this propulsion candidate: advertised maximum is only 0.8 A. The 250 mAh cell still needs discharge, mass and connector evidence |
| Alternative high-discharge battery leads | Alternative, not additional requirement | [RC Product BD 1S 1000 mAh](https://rcproductbd.com/singel-product/1757/37v-1000mah-25c-lipo-battery): ৳300, page says 10 in stock; [Cartup HH502030 300 mAh](https://cartup.com/product/hh502030_300mah_37v_li-ion_25c_2348148579688873_vf38um): ৳167, In Stock | RC title says 25C, description says 20C/28 g; resolve discrepancy and test actual current/mass. Cartup says 25C but dimensions/mass and charge claims need manufacturer confirmation. Neither is qualified for flight |
| Charger specifically suitable for selected 1S cell | 1 | [RobotechShop TP4056 protected Type-C](https://robotechshop.com/product-category/breakout-boards/): ৳40, category says in stock | Verify exact SKU and set charge current to cell specification. An advertised 1 A default is not approval for a 250 mAh cell; charging circuit does not establish motor discharge capacity |
| Regulated logic power supply | 1 | [Daraz TPS63020 3.3 V module](https://www.daraz.com.bd/products/tps63020-automatic-buck-boost-step-up-down-power-supply-module-lithium-battery-low-ripple-voltage-converter-33v-i460430506.html): marketplace listing | Price/local stock unconfirmed. Candidate architecture only; verify actual IC, output, peaks, ripple and delivered schematic before connection |
| Battery measurement divider/filter | 1 circuit: 2 resistors + filter capacitor | Local passive assortment | Select ratio for actual battery max and chosen ADC pin/range; calibrate physically |
| Rated motor power disconnect and independent output inhibit | 1 each | Switch/connectors plus circuit components selected from local catalogs | Select from measured current and stop design. A tiny signal switch is not assumed capable of interrupting motor supply |
| Matched battery connectors, silicone power wire, heat shrink | 1 set, plus spare mating connectors | [HTE battery/component catalogs](https://www.htebd.com/brand/hte/) | Verify connector polarity, current and wire gauge; exact SKU/price remains open |
| Driver/power PCB or suitable soldered prototype | 1 | Local PCB fabrication or perfboard for initial measurements | Motor current path must not use the solderless breadboard; flight PCB/mass/layout remain open |

The [AO3400 datasheet](https://www.aosmd.com/sites/default/files/res/datasheets/AO3400.pdf)
specifies on-resistance at low gate voltage, making it a candidate for 3.3 V drive.
Its headline current is not guaranteed safe current on our small PCB. The
[DRV8833 datasheet](https://www.ti.com/lit/ds/symlink/drv8833.pdf) gives package-dependent
current limits and protection behavior; do not assume every module delivers the
advertised peak continuously. Discrete channels and dual bridges are alternatives,
not a combined purchase requirement.

Bench logic power can initially come from USB. Battery operation needs a reviewed
power path; do not feed a 1S cell directly into an unverified dev-board power pin.
The [TPS63020 manufacturer specification](https://www.ti.com/lit/ds/symlink/tps63020.pdf)
is for the chip, not proof that a marketplace module is implemented correctly.
Pin assignments, regulator choice, charge current and battery thresholds remain
in the exact hardware gate; this document does not silently change firmware defaults.

## 4. Bench instruments and assembly tools

| Item | Qty | Sourcing / advertised price | Need |
| --- | --- | --- | --- |
| Digital multimeter | 1 | [TechShopBD UT33B+ listing](https://techshopbd.com/product/ut33b-palm-size-multimeter): ৳1,260.03 | Rails, polarity, continuity and divider calibration; seller Q&A conflicts with authenticity claim, inspect unit |
| USB logic analyzer | 1 | [Industrial.com.bd 24 MHz/8-channel](https://www.industrial.com.bd/product/logic-analyzer): ৳2,400, listed; [BDTronics equivalent](https://www.bdtronics.com/usb-logic-analyzer-24mhz-8-channel-24m-seconds-debugger-for-arm-fpg-saleae-compatible.html): ৳1,200 but Request Stock | PWM/stop/I²C/UART timing; verify 3.3 V thresholds and supported software on the actual Mac |
| Higher-end logic analyzer alternative | 1 instead of budget analyzer | [TechShopBD LA1010](https://techshopbd.com/product/la1010-16-channel-100mhz-logic-analyzer): ৳8,560.04, listed | Optional; verify driver/software and capture settings, no need to buy both |
| Adjustable current-limited bench supply | 1, buy or borrow | [TechShopBD KORAD KA3005D](https://techshopbd.com/product/lab-power-supply-single-channel-dc-30v-5a-encoder-knob-korad-ka3005d): ৳10,450.06, listed | First power and one-channel tests. Its 5 A rating does not guarantee enough current for all four loaded motors |
| Oscilloscope with probes | 1, borrow acceptable | [TechShopBD instruments](https://techshopbd.com/category/instruments) | Required access for rail transients/noise and analog stop validation; exact model/price not selected |
| Soldering iron with temperature control | 1 | [TechShopBD Sunshine SL936D](https://techshopbd.com/product/sunshine-sl936d-80w-soldering-iron): ৳1,250.04, listed | Assembly and rework |
| Solder, flux, braid, tweezers, cutter, helping hands | 1 set | Local electronics tools catalogs | SOT-23 assembly may need extra equipment; exact SKUs/prices unconfirmed |
| Small scale and calipers | 1 each, borrow acceptable | [HTE scale/catalog](https://www.htebd.com/brand/hte/), [TechShopBD instruments](https://techshopbd.com/category/instruments) | Measure total mass, dimensions and shaft fit; exact SKUs/prices unconfirmed |
| Secured motor test fixture, protective eyewear and suitable LiPo storage | 1 set | Local fabrication/tools suppliers | Initial powered tests without propellers; thrust measurement needs a guarded fixture |

Example quoted instruments: multimeter + budget analyzer + supply + iron =
**৳15,360.13**, before accessories or oscilloscope. Borrowing lab tools may be more
practical than buying them. No complete flight budget is stated while battery,
power board and matched propulsion remain unresolved.

## 5. Optional control hardware

* Wi-Fi/USB development uses the board's existing interfaces; no receiver purchase
  is required to start. Signed hardware control still needs the provisioning work
  identified in the [radio audit](radio-provisioning-audit.md).
* SBUS profile: one transmitter and compatible SBUS receiver, plus any required
  voltage/inversion circuit. No exact local pair was verified in this research;
  defer purchase until its output and failsafe contract are checked. A receiver
  labeled only iBUS is not evidence of SBUS compatibility.
* ESP-NOW profile: one additional compatible ESP32 as a USB proxy, plus its data
  cable. It is optional and needs separate end-to-end enrollment and timing tests.
* GPS, compass, camera, barometer, brushless ESCs and large brushless motors are
  outside the first implemented hardware profile. Introducing them requires a
  separate adapter/airframe scope.

## 6. Purchase order and acceptance

1. Obtain S3 board + MPU6050 + data cable + signal wiring. Capture delivered module
   marking and board schematic; validate sensor identity/axes while outputs disabled.
2. Obtain or borrow multimeter/analyzer/supply/scope. Measure first boot, reset,
   brownout and output-stop behavior before fitting propellers.
3. Freeze one matched motor/prop/frame set. Measure loaded current and thrust;
   choose drivers, rated connectors and battery from those results.
4. Review the battery/logic power path and independent inhibit. Record exact pins
   and power configuration in R3; build that exact target configuration.
5. Qualify timing and electrical stop, then proceed to the guarded propulsion and
   operator flight gates in the [remaining plan](remaining-plan.md).

The board and IMU are immediately actionable local candidates. A fully matched,
locally stocked flight BOM has not yet been established: propulsion dimensions,
high-discharge battery, power circuit and board mass/fit are the remaining gaps.
