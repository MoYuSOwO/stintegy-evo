<p align="center">
  <img src="logo.svg" alt="StintegyEVO" width="500"/>
</p>

# StintegyEVO – Racing Strategy Engineer Simulator

[中文正式版](README_zh.md)

_The maintainer will make every reasonable effort to keep the English and Chinese versions consistent. If a difference cannot be resolved, the Chinese version prevails._

**StintegyEVO** is a **racing strategy simulation game driven by real-time vehicle physics**. You play not as a driver, but as a **race engineer**. During races, qualifying sessions, and endurance events, you adjust tire usage, battery power, and other strategies, direct AI drivers to execute your tactics, and live with the consequences inside the same vehicle simulation. Four-wheel torque vectoring, adjustable energy recovery, motor thermal management, and a more detailed nonlinear tire model are long-term directions.

> **Note: You do not need a controller or steering wheel.** All driving operations are performed by AI drivers; you only need to issue strategic commands.

> **The fundamental premise will not change: the AI driver is driving a car that genuinely participates in the simulation.** Planning, control, tires, energy, collisions, and track boundaries all share the same vehicle state instead of playing unrelated animations or predetermined outcomes.

---

## 🤝 Focused Collaboration Welcome

StintegyEVO is still at an early engineering stage. It is now a **running, observable engineering prototype**, but not yet a content-complete game. Vehicle physics, foundational AI driving, vehicle-motion path prediction, rolling local speed planning, and longitudinal avoidance along a fixed path now form a closed loop. The current focus is stable local attacking and defending lines in multi-car situations, followed by the race flow and strategy interface.

The repository is public and open to focused collaboration, but the project is still maintainer-led and exploratory. Gameplay scope, physics fidelity, AI architecture, and roadmap priorities may change while the first stable prototype is being found.

Collaboration is especially useful around:

* **Vehicle control and AI**: Local path candidates, collision constraints, overtaking and defending, racing AI, and other real-time planning and control approaches.
* **Vehicle dynamics**: Validation of the current reduced-order model and performance envelope, plus more detailed tire, powertrain, and energy models in the future.
* **Godot 4 / C# engineering**: Simulation architecture, profiling, testing, telemetry, and debugging tools.
* **Art and interface design**: Low-poly vehicles, race-engineer UI, telemetry visualization, and technical presentation.

You do not need to solve the entire system. Design reviews, reproducible experiments, benchmarks, partial implementations, and unsuccessful approaches with useful conclusions are all meaningful contributions.

[Issues](https://github.com/MoYuSOwO/stintegy-evo/issues) · [Discussions](https://github.com/MoYuSOwO/stintegy-evo/discussions) · [Contributing guide](CONTRIBUTING.md)

---

## 🚀 Core Features

### 1. Real-Time Electric Racing Physics Core

The current model deliberately uses a reduced-order representation designed for 20-car fields and online planning, while every important state continues to evolve:

* **Grip and body dynamics**: Independent front/rear friction circles, longitudinal and lateral load transfer, yaw and body sideslip, rear-axle sliding, traction control, and sideslip energy loss.
* **Per-tire accounting**: Each tire tracks its own load, surface temperature, core temperature, and wear; forces are currently aggregated at the front/rear axle level.
* **Battery and powertrain**: Power-versus-speed limits, battery SOC, low-charge power derating, consumption, and regenerative braking, with five power presets and a foundation for continuous adjustment.
* **Track and collision interaction**: Track boundaries, wall contact, car-to-car contact, and post-impact state recovery all run inside the same simulation loop.

### 2. AI Strategy Execution (Running Prototype)

The AI driver can currently:

* **Follow the racing line**: Use a Stanley control law to produce target curvature from the car's live position and attitude.
* **Predict its actual path**: Roll the same control law forward to predict the path the car will really take instead of assuming it always sits on the static racing line.
* **Plan speed on a rolling horizon**: Run forward/backward propagation along the local predicted path under curvature, tire-state, battery-power, and combined longitudinal/lateral acceleration constraints.
* **Avoid traffic longitudinally**: Each car reads the same frozen opponent state, predicts conflicts independently, and chooses whether to follow or stop without depending on any other car's plan.
* **Express driver differences**: Pace, consistency, car control, tire management, and adaptability already affect planning, control, or physics outcomes with reproducible random variation.

Lateral overtaking, defending, and side-by-side driving remain on the roadmap. The current avoidance layer changes speed only along the existing path.

### 3. Strategic Depth
The current simulation supports and is actively calibrating:

* Tire usage: Protect, Light, Normal, Push, and Attack presets with continuous interpolation.
* Battery output: Save, Eco, Normal, Push, and Attack presets with a continuous power ceiling.
* Tire temperature, wear, battery charge, and driver ability all affect lap time instead of changing UI numbers alone.

The long-term plan is to add front/rear and left/right torque distribution, regenerative-braking strength, motor thermal management, aerodynamics, and slipstream/dirty-air interaction within the same strategy framework.

### 4. Open Source & Extensible
* The simulation core is fully self-developed, independent of the Godot node lifecycle, and can be built, tested, and run on its own.
* Track data is defined based on curves, making it easy for the community to create new tracks.
* The Community Edition is open source under **AGPL-3.0**. Contributors keep ownership of their work, and accepted contributions remain open.

---

## 🧠 Design Philosophy

Racing games usually make you the driver. But in real racing, victory or defeat is often decided on the Pit Wall. StintegyEVO wants you to experience being **the person holding the tactical board**. You don't need zero-second reaction times, but you do need to understand the interplay of tires, motors, battery power, and aerodynamics.

### 🆚 Positioning Differences from Other Racing Games

| Dimension | Motorsport Manager | F1 Manager 2024 | iRacing | **StintegyEVO** |
|:---|:---|:---|:---|:---|
| **Positioning** | Racing Manager/Business Sim | Official F1 Team Management | Hardcore Driving Sim | **Racing Strategy Engineer Simulator** |
| **Physics Engine** | Simplified Numerical Model | Game-Level Physics | High-Fidelity Driving Physics | **Self-Developed Real-Time Vehicle Dynamics** |
| **Powertrain** | Traditional ICE | F1 Hybrid (Abstracted) | ICE / Hybrid | **Modern Electric Racing Car** |
| **Driving Method** | AI Auto | AI Auto | Player Driven | **AI Driver Executes, Player Issues Strategic Commands** |
| **Strategic Depth** | Pit Stops / Tires / Engine Modes | Pit Stops / Tires / ERS | On-Track Racing | **Currently Tire Usage and Battery Output; Progressively Expanding Vehicle and Energy Control** |
| **Tire Model** | Simplified Wear | Game-Level | High-Fidelity Driving Tires | **Front/Rear Friction Circles + Per-Tire Dual-Layer Temperature and Wear** |
| **Extensibility** | Mod Support | Limited | Closed Ecosystem | **Open Source (AGPLv3)** |

### Why Does StintegyEVO Take This Route?

- **Traditional driving simulators (iRacing/ACC/Forza) make you the driver.** StintegyEVO makes you the race engineer.
- **Racing management games emphasize team and race decisions.** StintegyEVO goes one step further by trying to make tire, battery, and vehicle states produce strategy consequences inside a real-time simulation.
- **StintegyEVO is an experiment at the intersection of those two directions:** It pursues a *Motorsport Manager*-style tactical viewpoint while retaining a vehicle-physics core that can grow deeper over time. It is still an early prototype and does not claim the fidelity of a professional driving simulator or engineering tool.

---

## 🛠️ Tech Stack

* **Engine**: [Godot Engine 4.6.x](https://godotengine.org/) (.NET edition)
* **Language & Runtime**: C# / .NET 8
* **Core**: 2D vehicle, race, and planning simulation separated from the Godot presentation layer, with headless automated testing
* **Presentation**: Godot 2D debugging and race views today; Low Poly 3D remains a future direction
* **AI**: Stanley tracking, actual-motion path prediction, rolling local speed planning, and traffic constraints built from shared frame snapshots
* **Numerics**: HiGHS and BLAS support minimum-curvature reference-line optimization

---

## 🗺️ Roadmap

StintegyEVO is an independent solo-developed project. It has passed the first major blocker—whether the AI can reliably drive the car—and now has a running closed loop from vehicle state through path prediction and speed planning to physics execution. The next stage focuses on local line selection and racecraft in multi-car situations, followed by the game-facing race flow and interface.

### Versioning Philosophy

Each completed module receives a version number, marking its transition from "unavailable" to "functional." Pending goals use logical numbering to indicate **suggested priority and dependencies**; they do not strictly lock the release order. The roadmap is a working map for exploration, not a promise of stable API, feature scope, or release order.

If you are interested in any of these goals, focused experiments, small implementations, benchmarks, and design reviews are welcome. Contributors keep ownership of their work, and accepted contributions remain available to the community under **AGPL-3.0**.

### ✅ v0.1 Running Vehicle Physics (Completed)

* Independent front/rear friction circles with natural competition between longitudinal and lateral grip.
* Four-wheel load transfer, with surface temperature, core temperature, and wear tracked separately for all four tires.
* Yaw, body sideslip, rear-axle sliding, traction control, and sideslip energy loss.
* Battery power limits, SOC, energy consumption, regenerative braking, and five battery-output modes.
* Vehicle-to-track-boundary and vehicle-to-vehicle contact handling; track boundaries use swept tests to reduce high-speed tunneling.

### ✅ v0.2 Foundational AI Driving and Dynamic Speed Planning (Completed)

* Stable lapping using a minimum-curvature reference line and Stanley control law.
* Forward prediction of the path the controller will actually produce from the car's live position, heading, sideslip, and yaw state.
* Rolling local speed planning over that predicted path under curvature, power, braking, tire-state, and friction-circle constraints.
* Per-frame updates over a local window without continuously recomputing the entire circuit.
* Pace, Consistency, Car Control, Tire Management, and Adaptability already affect planning, control, or physics outcomes.

### ✅ v0.3 Longitudinal Multi-Car Avoidance (Stage Complete)

* Every car reads the same frozen frame state and plans independently without reading or waiting for another car's plan.
* Short-term opponent motion prediction with body bounding boxes and swept tests for potential conflicts.
* Conflict constraints feed directly into the rolling speed planner, supporting slow-car following and stopping for stationary or crossing vehicles.
* This stage changes speed only; it does not yet generate lateral overtaking or defending paths.

### 🚧 Pending Implementation (Ordered by suggested priority)

#### 1. Local Attack and Defense Paths (Current Focus)
*Dependencies: v0.2, v0.3*

* Generate a small, interpretable set of keep-line, move-left, and move-right candidates around the current reference path.
* Check track boundaries, vehicle conflicts, and feasible speed plans together for each candidate instead of choosing a line first and relying on emergency braking afterward.
* Preserve the valid untraveled portion of the previous path and use switching hysteresis to suppress the negative-feedback loop caused by per-frame replanning.
* Keep every car dependent only on the shared state snapshot so multi-car planning can run in parallel.

**Stage goal**: First achieve stable, fast side-by-side and passing behavior without left-right oscillation; then layer overtaking value, defensive intent, and racing rules on top.

#### 2. Full Strategy Console
*Dependency: v0.2*

* Connect the existing five tire-usage and battery-power presets to continuous sliders and an in-race control interface.
* Display tire surface/core temperatures, wear, battery SOC, power, vehicle stability, and planning state.
* Add torque distribution, recovery strength, and thermal-management controls as the advanced vehicle model arrives.

#### 3. Driver Racecraft Abilities and Decisions
*Dependency: Goal 1*

* Continue connecting Reactions, Awareness, Overtaking, and Defending to conflict detection, path selection, and decision timing.
* Preserve the model in which a lower-rated driver can still perform brilliantly on occasion, but with greater variance.
* Strategy and resource management remain the player's responsibility; driver ability primarily describes how well the driver handles the car and executes tactics.

#### 4. Advanced Vehicle Model (Long-Term Direction, Not the Current Focus)
*Dependency: v0.1*

More detailed physics will not require the AI driver to output raw steering-wheel position or four individual motor torques. The plan is to preserve a stable control boundary:

```text
AI driver: target curvature, target acceleration
                         ↓
Vehicle low-level controller: steering angle, four-wheel torque,
                              mechanical/regenerative brake allocation
                         ↓
Detailed vehicle physics: tire forces, yaw, wheel speed,
                          motors, battery, and thermal state
```

* Add a vehicle actuator-allocation layer that converts semantic driving targets into steering, drive, and braking commands.
* Introduce four-wheel independent torque vectoring, adjustable regenerative-braking blending, and motor thermal derating.
* Evaluate a nonlinear tire model with combined slip, force relaxation, and wheel-speed state. Pacejka is a candidate, not an implementation locked in for marketing purposes.
* Runtime physics may grow more detailed, while planning and motion prediction continue to use a reduced-order performance envelope calibrated from the same vehicle parameters, avoiding a full tire-model evaluation at every path sample.

#### 5. Track Gradient & Banking
*Dependency: v0.1*

* Add track gradient and banking to load, longitudinal-acceleration, and cornering-capability calculations.
* Expose their effects to planners through the vehicle performance envelope so the planning layer does not reimplement physics.

#### 6. Race Weekend Framework
*Dependencies: Goals 1, 2*

* Complete flow for Practice, Qualifying, and Race sessions.
* Tire selection, pit stops, and setup presets.
* Move from "running laps" to "managing a race."

#### 7. Low Poly 3D Visuals
*Dependency: None (pure presentation layer)*

* Explore an *Art of Rally*-style low-poly visual language with a Godot 3D rendering layer over the independent 2D simulation core.
* Build the race-engineer workstation, strategy controls, and telemetry panels.

#### 8. Track Editor Prototype
*Dependency: Goal 5*

* Let players create and import tracks while reference-line optimization, AI driving, and local planning all reuse the same track data.
* Provide the foundation for a community-driven content ecosystem.

### 🌟 Long-Term Outlook

* **Multiplayer**: Multiple players commanding their respective teams in real-time online competition.
* **Advanced Simulation**: Safety Cars (SC/VSC), dynamic weather, track surface temperature, and standing water.
* **Career Mode**: Full team management, R&D trees, and a driver market.
* **Hardware-in-the-Loop**: Interfaces for real vehicle data acquisition or exporting the engine output to professional simulation software.

> **This roadmap describes goals, but implementation remains entirely open. Alternative proposals—even those that overhaul existing plans—are welcome.**

---

## 🏁 Quick Start

Clone the repository and validate the independent simulation core:

```bash
git clone https://github.com/MoYuSOwO/stintegy-evo.git
cd stintegy-evo
dotnet build StintegyEVO.sln
./test.sh
```

To run the default demo, open `project.godot` in the Godot 4.6.x .NET edition. The current default scene shows a car detecting and following a slower car ahead along a fixed path. The development environment requires .NET SDK 8.0+.

---

## 🤝 Contributing

StintegyEVO welcomes focused contributions. You don’t need to be a physics engine expert or a C# maestro, but contributions are most useful when they are tied to a concrete problem, experiment, or small implementation:

* **AI Development**: Local path candidates, collision constraints, racecraft decisions, and multi-car performance optimization.
* **Physics Tuning & Validation**: Calibrate tire, vehicle, and energy parameters while validating consistency between runtime physics and the planning performance envelope.
* **Track & Vehicle Data**: Create new tracks or design vehicle configurations.
* **Art & UI**: Low Poly models, UI interfaces, and particle effects.
* **Documentation & Outreach**: Improve the README, write tutorials, or record demo videos.

### How to Get Involved

> **Before you begin, please read the [English](CONTRIBUTING.md) or [Chinese](CONTRIBUTING_zh.md) contributing guide for development setup, coding conventions, the CLA, and other details.**

1.  **Pick a Goal or Focused Problem**: Use the [Roadmap](#-roadmap) as context. It records current progress, suggested priorities, and dependencies without freezing the project direction.

2.  **Communicate First, Code Later**:
    * If your **idea is already formed**, head over to [Issues](https://github.com/MoYuSOwO/stintegy-evo/issues) to create a task post. Outline your proposed solution and technical approach, and be sure to attach the relevant Issue Label.
    * If your **idea is still evolving**, start a thread in [Discussions](https://github.com/MoYuSOwO/stintegy-evo/discussions) to get feedback from the community and maintainers.

    **Why communicate first?** The main challenge of this project isn't "writing the code," but rather "navigating the uncertainty of the approach." Communicating beforehand prevents you from spending weeks on a solution that might conflict with someone else's work or be incompatible with the existing architecture. Broad feature requests or large rewrites may be deferred until the current racecraft planning and product loop are clearer.

3.  **Start Contributing**: Communication is for coordination, not an approval gate. Once the scope is clear—or immediately for a small, self-contained improvement—you can begin. Please read the [English CLA](CLA.md) and the [governing Chinese CLA](CLA_zh.md) before submitting a contribution.

We look forward to advancing this project together while fully respecting the copyright and intellectual property of every contributor.

---

## 📜 License

This project is open-sourced under the **GNU Affero General Public License v3.0 (AGPL-3.0)**.

The standalone Community Edition will remain open source. The Project may eventually charge for official distributions through Steam or other platforms. Charging for an official distribution does not turn the Community Edition into a proprietary product or reduce recipients' rights under AGPL-3.0 to obtain, modify, and redistribute the corresponding source. Every publicly distributed official client build must correspond to a published source revision (normally a Git tag) and make the complete AGPL-covered source used by that build available. Project-owned client changes, including platform integration code, are part of that corresponding source and may not be withheld merely as an “official-only feature.”

The code license does not require every piece of music, artwork, track data, font, or other game content to use the same license. Project assets published in this repository without a separate notice are provided with the Community Edition under AGPL-3.0; third-party material remains under its respective license. Future official distributions may include separately licensed content, provided that its terms are identified clearly and AGPL-covered program code or modifications are not relabeled as “content” to withhold their source. See the [Content and Asset Licensing Notice](CONTENT_LICENSE.md) for the detailed boundary. The StintegyEVO name, logo, and other project identifiers may not be used to make an unofficial product appear official; reasonable use to describe compatibility, origin, or derivation is unaffected.

The [CLA](CLA.md) gives the maintainer a limited additional license for a possible future official online version, so shared systems such as physics, AI, and UI would not need to be rewritten. It does not permit converting the standalone Community Edition into a proprietary product or selling a contribution separately from StintegyEVO; this does not prevent charging for an AGPL-3.0 Community Edition that includes the contribution. The maintainer will make every reasonable effort to keep the English and [Chinese](CLA_zh.md) texts consistent; if a difference cannot be resolved, the Chinese text prevails.

---

## 💬 Contact

* Maintainer: MoYuSOwO
* Email: stintegy-evo@proton.me
* Discussions: [GitHub Discussions](https://github.com/MoYuSOwO/stintegy-evo/discussions)

---

> **"We don't compete on who drives faster. We compete on who calculates better."**
