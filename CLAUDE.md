# Maly woz

## Project description

We are building an amateur planetary rover for student competitions (like European Rover Challenge). This repository contains all the software used in the project. Main goal is to learn new stuff rather than try hard to win competition.

### Hardware and software stack

We use ROS 2 Jazzy and Gazebo Harmonic.
Code is developed primarily on PCs with Linux, sometimes on Windows.
Computer used on the real robot is Raspberry 4.

### Team structure

We are from Almukantarat Astronomy Club and our team is distributed across all Poland. Team members have varying level of experience in software and hardware. Some are more familiar with Windows rather than Linux, and vice versa. Code should be well documented (comments and READMEs) but concise.

## Coding guidelines

### Think before coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**
Before implementing:

- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

### Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

### Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:

- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:

- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

### Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:

- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:

```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

### Coding style

General rules:

- Update READMEs in packages you change.
- Follow Google Style Guides unless existing code uses different conventions.
- Use proper design patterns.
- Avoid excessive comments.
- Use docstrings, keep them short.
- Create useful tests for application logic, avoid redundant tests.
- Split your work into meaningful commits.

Python:

- Use `ruff` for formatting and linting.
- Don't use `typing` library, use `dict`, `list`, etc. directly.
- Avoid lazy imports. Put all imports at the top of the file.
- Always use type annotations.
- Use `__all__` for exporting.

### ROS 2

- Use ROS 2 nodes only for communication, application logic should be independent.
- Use `yaml` config files for node parameters.
- Don't use ":" in xml/urdf/... comments, it breaks parsing.
- Use ROS 2 logger instead of raw printing.
- Message interfaces should be defined in a dedicated `_msgs` package.
- Prefer using single thread executors.
- Always define ROS 2 package dependencies on package-level.
- If we need to use external dependencies, use `vcstool`.
- Document each ROS package with `README.md`: nodes with descriptions and interfaces exposed by them (subs, pubs, params, etc.)

## Avoid AI slop

- Prefer standard ASCII character in source files, avoid —, ×, etc.
- Don't over-comment.
- Don't create bloat with unnecessary `try` blocks.
- Don't duplicate functionalities, check if they are already implemented before writing a new one.
- Avoid type check workarounds.
