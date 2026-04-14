# RocketSim3D: AI Coding Discipline & Style Guide

To maintain a high-quality, high-performance, and human-readable codebase, all code generation and refactoring MUST strictly adhere to the following rules.

---

## 🏗️ 1. ECS Architecture (EnTT)
*   **Separation of Concerns**: Keep data and logic completely decoupled.
*   **Components as POD**: Components must be `struct`s containing only data (no methods except simple constructors). Use `entt::registry` for storage.
*   **Systems as Logic**: Systems are stateless functions or classes that operate on `entt::view` or `entt::group`.
*   **No Cross-Component Links**: Do not store pointers or references to other components inside a component. Use `entt::entity` IDs if linkage is required.

## 🏔️ 2. Anti-"Mountain" Code (Flatness)
"Mountain code" (deep nesting) is forbidden. It hides logic and creates bugs.
*   **Max Nesting Level**: 2. If you need a 3rd level, extract the inner logic into a helper function or use early returns.
*   **Early Returns / Guards**: Use `if (!condition) return;` at the start of functions to handle edge cases and "flatten" the success path.
*   **Loop Control**: Use `continue` and `break` to avoid wrapping large blocks of loop logic in `if` statements.
*   **Small Functions**: Aim for functions < 40 lines. If a function is doing "too much," split it.

## 👓 3. Human-Readable "Clean Code"
Code is read by humans more often than by compilers.
*   **Self-Documenting Names**:
    *   ❌ `rel_px`, `mu`, `e_mag`
    *   ✅ `relativePositionX`, `gravitationalParameter`, `eccentricityMagnitude`
*   **No Magic Numbers**: Every physical constant or UI offset must be a `constexpr` or `#define` with a descriptive name.
*   **Vertical Spacing**: Group related lines of code together. Use a single blank line to separate logical blocks within a function.
*   **Comments as "Why", not "What"**:
    *   ❌ `// Calculate distance`
    *   ✅ `// Use Keplerian approximation because high precision isn't required for the HUD`

## 🛠️ 4. C++ Implementation Details
*   **Math Safety**: Always check for `NaN`, `Inf`, and division by zero in physics calculations.
*   **Headers**: Use `#pragma once`. Keep `#include`s sorted and minimal to reduce compile times.
*   **Memory**: Avoid `new` and `delete`. Use EnTT's managed memory or smart pointers (`std::unique_ptr`) if heap allocation is strictly necessary.

---

---

### How to use this guide:
Whenever you (the AI) are asked to write or refactor code, you must:
1.  Read this `DISCIPLINE.md`.
2.  Self-audit the proposed code against these 4 categories.
3.  Explicitly mention if you had to "flatten" a function to follow the rules.

---

## 🔝 5. Pro AI Discipline Tips

### 🎭 Persona & Roleplay
*   **Architect Mode**: Act as a **Senior Lead Game Engine Architect**. Prioritize cache-friendly data structures and zero-cost abstractions.
*   **Plan First**: Before applying any multi-file change, provide a "Proposed Plan" and wait for confirmation.

### 📐 Performance & Hardware Awareness (DOD)
*   **Data-Oriented Design**: Prefer `std::vector` for contiguous memory. Avoid linked lists or maps in hot simulation loops to minimize cache misses.
*   **Parallelism Ready**: Keep systems stateless so they can eventually be parallelized across the registry.

### 🔍 Self-Validation
*   **Edge Case Audit**: After implementing a feature, list 3 potential edge cases (e.g., zero gravity, infinite velocity, empty registry) and explain how the code handles them.
*   **The "One-Shot" Contrast**: If you encounter complex logic, show a "Bad" vs "Good" implementation in your thought process to verify you are following the "Flatness" rule.

---
*Last Updated: 2026-04-14*
