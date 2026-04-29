# Contributing Guide

Thank you for your interest in **StintegyEVO**! This is a solo-initiated race strategy engineer simulator project currently in early development. We welcome all forms of contributions, including but not limited to code, documentation, art, UI, track data, vehicle tuning, and feedback.

To help us collaborate efficiently, please take a few minutes to read the following guidelines.

---

## Code of Conduct

This project adheres to the [Contributor Covenant Code of Conduct](CODE_OF_CONDUCT.md). We expect all participants to help foster a friendly, inclusive, and respectful community environment.

---

## How to Contribute

### 1. Choose a Task & Communicate First
*   **Look at the roadmap**: Our [roadmap](README.md#-roadmap) lists numbered goals with dependencies and I/O. Pick one that fits your interest.
*   **Discuss before coding**:
    *   If you have a **concrete plan** (e.g., "I'll implement AI using MPPI"), open an **Issue** with the corresponding label (e.g., `ai-driving`) and describe your approach.
    *   If your idea is **still shaping up**, start a **Discussion** in the appropriate category (e.g., `🧠 AI & Physics`).
    *   **Why?** Because the main challenge is not "can you write code", but "does your plan fit the dynamic physics and path architecture?" Early communication prevents you from spending weeks on a solution that may conflict with others or the project's direction.

### 2. Reporting Bugs & Suggestions

If you find a bug or have a suggestion, please submit it via GitHub Issues with as much of the following information as possible:

- **Title**: Concisely describe the issue.
- **Environment**: Godot version, operating system, etc.
- **Steps to Reproduce**: List the actions that trigger the problem in order.
- **Expected vs. Actual Behavior**: Clearly state what you expected to happen and what actually happened.
- **Screenshots / Logs**: Attach console output or screenshots if available.

### 3. Submitting Code (Pull Requests)

We follow a standard GitHub Flow workflow. Please adhere to the following steps:

1.  **Fork** this repository to your own GitHub account.
2.  Create your feature branch: `git checkout -b feature/your-feature-name`
3.  Make your changes. Please keep your code style consistent with the existing codebase (see "Coding Conventions" below).
4.  Commit your changes: `git commit -m 'Add some feature'`
    - Please use clear and explicit commit messages (e.g., `feat:`, `fix:`, `docs:`).
5.  Push your branch: `git push origin feature/your-feature-name`
6.  Return to the GitHub web interface and open a Pull Request against the main `stintegy-evo` repository.

**Before opening a Pull Request, please confirm the following:**

- [ ] Your code has been tested locally and does not break existing functionality.
- [ ] You have read and agree to the [Contributor License Agreement (CLA)](CLA.md). **Submitting a PR signifies your acceptance of and agreement to the terms of this agreement**, granting the project maintainer the rights to your contribution for potential future commercial licensing needs.

### 4. Improving Documentation & Art

Contributions to documentation, README improvements, Low Poly models, UI design, and the like are equally valuable. These can also be submitted via Pull Request, following the same process as above.

---

## Development Environment Setup

1.  **Engine**: [Godot Engine 4.x](https://godotengine.org/) (the standard version does not support C#; please ensure you use the **.NET edition**)
2.  **.NET SDK**: Version 8.0 or higher.
3.  **IDE**: Rider is recommended; you can also use Visual Studio Code with the corresponding C# extension.
4.  **Clone the Project**:
    ```bash
    git clone https://github.com/MoYuSOwO/stintegy-evo.git
    ```
5.  Open the `project.godot` file in the project root directory with the Godot .NET edition.
6.  Click the "Build" button in the top-right corner of the Godot editor (or run `dotnet build`) to ensure the project compiles successfully.

---

## Coding Conventions

To maintain consistency across the project, please keep the following in mind when contributing code:

- **Language**: C#. English is recommended for code comments and commit messages.
- **Namespaces**: Non-Godot-node classes should be placed under the `StintegyEvo.Core` namespace and its sub-namespaces. Godot-node classes should be placed under `StintegyEvo.Nodes` and its sub-namespaces.
- **Formatting**: Try to maintain the formatting conventions of the existing code (brace style, indentation, etc.).
- **Physics / Vehicle Terminology**: Use precise professional terminology when naming variables whenever possible, e.g., `WheelAngularVel`, `SlipRatio`, `DownforceFront`.

If you are unsure, take a look at the existing code first, or ask directly in Issues / Discussions.

---

## License & Legal Notice

- **Project Code**: Licensed under the [GNU Affero General Public License v3.0 (AGPL-3.0)](LICENSE).
- **Contributor Licensing**: All code contributions are subject to the [Contributor License Agreement (CLA)](CLA.md). **By submitting a Pull Request, you confirm that you have read and agree to this agreement, granting the project maintainer the right to use your contribution for commercial licensing and other purposes in the future.** If you do not agree to this agreement, please do not submit code.

---

> **Let's build the first race strategy simulator driven by a real physics engine — together.**

**StintegyEVO Maintainer**