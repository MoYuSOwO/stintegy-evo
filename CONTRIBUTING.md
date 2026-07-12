# Contributing Guide

[中文正式版](CONTRIBUTING_zh.md)

_The maintainer will make every reasonable effort to keep the English and Chinese versions consistent. If a difference cannot be resolved, the Chinese version prevails._

Thank you for your interest in **StintegyEVO**! This is a solo-initiated race strategy engineer simulator project currently in early development. Focused contributions are welcome, especially when they are tied to concrete experiments, bug reports, tooling, documentation, or small implementations.

The project is public and collaborative, but still exploratory. Broad feature requests, large rewrites, and long-term roadmap proposals may be deferred until the current racecraft planning and product loop are clearer.

To help us collaborate efficiently, please take a few minutes to read the following guidelines.

---

## Code of Conduct

This project adheres to the [Contributor Covenant Code of Conduct](CODE_OF_CONDUCT.md). We expect all participants to help foster a friendly, inclusive, and respectful community environment.

---

## How to Contribute

### 1. Choose a Task & Communicate First
*   **Look at the roadmap**: Our [roadmap](README.md#-roadmap) describes current progress, suggested priorities, and dependencies between goals. Pick one that fits your interest.
*   **Discuss before coding**:
    *   If you have a **concrete plan**, open an **Issue** with the appropriate label and describe the goal, your approach, and how you intend to validate it.
    *   If your idea is **still shaping up**, start a **Discussion** in the appropriate category (e.g., `🧠 AI & Physics`).
    *   **Why?** Because the main challenge is not "can you write code", but "does your plan fit the real-time vehicle physics, shared state snapshots, and rolling planning architecture?" Early communication prevents you from spending weeks on a solution that may conflict with others or the project's direction.
*   **Keep the scope focused**: Small fixes, reproducible experiments, and reviewable prototypes are easier to accept than broad rewrites.

### 2. Reporting Bugs & Suggestions

If you find a bug or have a focused suggestion, please submit it via GitHub Issues with as much of the following information as possible:

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
4.  Commit your changes, for example: `git commit -m 'feat(driver): add local path candidates'`
    - Follow the existing history and use clear Conventional Commit-style messages such as `feat(driver):`, `fix(physics):`, or `docs(readme):`.
5.  Push your branch: `git push origin feature/your-feature-name`
6.  Return to the GitHub web interface and open a Pull Request against the main `stintegy-evo` repository.

**Before opening a Pull Request, please confirm the following:**

- [ ] Your code has been tested locally and does not break existing functionality.
- [ ] You have read and agree to the [English CLA](CLA.md) and the [governing Chinese CLA](CLA_zh.md).

### 4. Improving Documentation & Art

Contributions to documentation, README improvements, Low Poly models, UI design, and the like are equally valuable. Focused changes can be submitted via Pull Request, following the same process as above.

---

## Development Environment Setup

1.  **Engine**: [Godot Engine 4.6.x](https://godotengine.org/) (the standard version does not support C#; please ensure you use the **.NET edition**)
2.  **.NET SDK**: Version 8.0 or higher.
3.  **IDE**: Rider is recommended; you can also use Visual Studio Code with the corresponding C# extension.
4.  **Clone the Project**:
    ```bash
    git clone https://github.com/MoYuSOwO/stintegy-evo.git
    cd stintegy-evo
    ```
5.  **Build the solution**: `dotnet build StintegyEVO.sln`
6.  **Run the core tests**: `./test.sh`
7.  Open `project.godot` in the Godot .NET edition, then build and run the default demo scene.

---

## Coding Conventions

To maintain consistency across the project, please keep the following in mind when contributing code:

- **Language**: C#. English is recommended for code comments and commit messages.
- **Namespaces**: The independent simulation core belongs under `StintegyEVO.Core` and its sub-namespaces. The Godot presentation layer belongs under `StintegyEVO.Presentation` and its sub-namespaces.
- **Formatting**: Try to maintain the formatting conventions of the existing code (brace style, indentation, etc.).
- **Physics / Vehicle Terminology**: Use precise terms that match the current model layer, such as `DesiredCurvature`, `YawRateRadiansPerSecond`, and `BatterySoc`.

If you are unsure, take a look at the existing code first, or ask directly in Issues / Discussions.

---

## License & Legal Notice

- **Project Code**: Licensed under the [GNU Affero General Public License v3.0 (AGPL-3.0)](LICENSE).
- **Your Copyright**: You keep ownership of your contribution. Once accepted into the public repository, it remains available under AGPL-3.0.
- **Why the CLA exists**: StintegyEVO may eventually need official online services with ongoing server, moderation, and maintenance costs. The [CLA](CLA.md) gives the maintainer a limited additional license to reuse shared project code in that future official online version. It does not allow the standalone Community Edition to be turned into a proprietary product or individual contributions to be sold separately. The maintainer will make every reasonable effort to keep the [English](CLA.md) and [Chinese](CLA_zh.md) texts consistent; if a difference cannot be resolved, the Chinese text prevails.

The purpose of the CLA is not to take ownership of community work. It lets us build shared systems such as physics, AI, and UI together without requiring them to be rewritten solely for a possible official online service later.

---

> **Focused experiments and small, clear improvements are the best way to move StintegyEVO forward while it is still finding its shape.**

**StintegyEVO Maintainer**
