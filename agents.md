# si473x-rs AGENTS

This document is the shared entry point for ongoing collaboration on this library.
It is intentionally concise and practical. We can extend it over time.

## Purpose

- Keep a shared understanding of what the library does and where it is heading.
- Capture stable decisions so future changes stay consistent.
- Make handoffs between sessions easier.

## Current Scope

- `no_std` async Rust driver for Si47xx radios.
- Primary modes supported in API: FM and AM/SW/LW.
- Transport: async I2C.
- Reset control: generic `OutputPin`.

## Current Architecture (high-level)

- `Si47xxDevice<T, R, A>`: low-level command/property operations.
- `Si47xxRadio<T, R, A>`: mode/state wrapper (`Am`, `Fm`, `Off`).
- `Si47xx` trait: async high-level interface abstraction.
- `Si47xxProperty`: property IDs (shared + FM-only + AM-only), with helper classifiers:
  - `is_shared()`
  - `is_fm_only()`
  - `is_am_only()`

## Reference Source of Truth

- AN332 (Si47XX Programming Guide)
  - FM properties: section 5.1 / 5.1.2
  - AM/SW/LW properties: section 5.2 / 5.2.2

## Working Agreements

- Prefer smallest correct changes.
- Keep public API names consistent and explicit.
- Document units/ranges for hardware-facing values where possible.
- Avoid mode-mixing mistakes by using property group helpers.
- Do not commit generated or local reference artifacts unless explicitly requested.

## Open Topics / Backlog Ideas

- Fill out `AmLwSwBands` with clear band presets (US AM, EU/ASIA AM, SW, LW).
- Add mode-aware property setter guardrails (optional strict validation layer).
- Expand tests around frequency bounds and property encoding.
- Consider splitting very large enums/docs into module-level property docs if file grows further.

## Change Log (collaboration notes)

- Added complete FM + AM/SW/LW property enum mapping from AN332.
- Unified shared property IDs across FM and AM/SW/LW.
- Added property-level docs (description, units, range where available).
- Added enum-level group docs and AN332 quick references.

## How to Extend This File

You can append:

- product goals and non-goals,
- supported chip/firmware matrix constraints,
- coding/style preferences specific to this repo,
- desired roadmap priorities.

I will treat updates in this file as collaboration guidance for future tasks.
