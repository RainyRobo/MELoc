# Contributing to MELoc

Thank you for considering a contribution.

## Development Setup

1. Install uv.
2. Create an environment:

   uv venv .venv

3. Install dependencies:

   uv pip install --python .venv/bin/python -e .

4. Run the demo:

   .venv/bin/python main.py --demo --demo_out demo_data --seed 7

## Coding Standards

- Target Python 3.10+.
- Keep the implementation minimal and deterministic.
- Prefer readable numerical code over premature abstraction.
- Add concise comments only where geometry or optimization logic is non-obvious.

## Commit and Pull Request Guidelines

- Use clear commit titles in imperative mood.
- Keep PRs focused and small.
- Include a short validation note with command and output summary.
- Update README.md if CLI arguments or behavior change.
- Update CHANGELOG.md under Unreleased.

## Testing Expectations

Before opening a PR, run:

- .venv/bin/python main.py --demo --demo_out demo_data --seed 7
- Optional static checks if configured: ruff check . and black --check .

## Scope Policy

This repository intentionally avoids full SLAM/ROS/system-level expansion.
Contributions should stay within minimal stereo-planar localization scope.
