# Artifact Evaluation Preparation Plan

## Anytime ROS 2 — RTAS Paper

**Paper:** "Anytime ROS 2: Timely Task Completion in Non-Preemptive Robotic Systems"
**Deadline:** Friday, February 27, 2026
**Critical path:** GPU sync fix -> Tests -> Docker + Automation -> Documentation

---

## Context

The paper was accepted at RTAS. The artifact evaluation requires demonstrating that the codebase is correct, reproducible, and easy for reviewers to use. Currently:

- The async GPU mode has race conditions (documented in `plan.md`)
- There are no unit or integration tests
- Reviewers need a single-command setup to reproduce experiments
- Documentation is incomplete (no architecture overview, no component details)

This plan addresses 6 core requirements + additional value-add items, organized across 4 working days.

---

## Schedule

### Day 1 (Tue Feb 24) — GPU Sync Fix + Code Refactoring

- [01-gpu-sync-fix.md](01-gpu-sync-fix.md) — Fix CUDA async race conditions (CRITICAL)
- [02-code-refactoring.md](02-code-refactoring.md) — Package cleanup, strict aliasing fix, reproducible seeds

### Day 2 (Wed Feb 25) — Tests

- [03-unit-tests.md](03-unit-tests.md) — Unit tests for anytime_core and anytime_monte_carlo
- [04-integration-tests.md](04-integration-tests.md) — GPU sync and cancellation integration tests
- [05-smoke-test.md](05-smoke-test.md) — Quick validation script (<2 min)

### Day 3 (Thu Feb 26) — Docker + Automated Setup

- [06-docker-restructuring.md](06-docker-restructuring.md) — Shared base Dockerfile + variants
- [07-python-cli.md](07-python-cli.md) — `anytime-eval` Python CLI tool
- [08-cicd-pipeline.md](08-cicd-pipeline.md) — GitHub Actions CI/CD

### Day 4 (Fri Feb 27) — Documentation + Polish

- [09-documentation.md](09-documentation.md) — Architecture, components, experiments docs
- [10-artifact-evaluation-guide.md](10-artifact-evaluation-guide.md) — Reviewer-facing guide
- [11-final-polish.md](11-final-polish.md) — Formatting, LICENSE, release tag
- [12-additional-items.md](12-additional-items.md) — Pre-built Docker images, requirements.txt, seeds

---

## Dependency Graph

```text
Day 1: GPU Sync Fix (01) ──────────────────────┐
       Code Refactoring (02) ───────────────────┤
                                                │
Day 2: Unit Tests (03) <── depends on 01 ──────┤
       Integration Tests (04) <── depends on 01 ┤
       Smoke Test (05) <── depends on 03, 04 ──┤
                                                │
Day 3: Docker Restructure (06) ── independent ──┤
       Python CLI (07) <── depends on 06 ───────┤
       CI/CD (08) <── depends on 05, 06 ────────┤
                                                │
Day 4: Documentation (09) <── depends on 01 ────┤
       AE Guide (10) <── depends on 07 ─────────┤
       Final Polish (11) <── depends on all ─────┘
       Additional Items (12) ── parallel with 11
```

---

## Critical Files

| File | Role |
| ---- | ---- |
| `packages/src/anytime_yolo/include/anytime_yolo/anytime_management.hpp` | GPU sync fix target |
| `packages/src/anytime_yolo/include/anytime_yolo/yolo.hpp` | `cudaLaunchHostFunc` bug fix |
| `packages/src/anytime_core/include/anytime_core/anytime_base.hpp` | Base class for virtual GPU completion hook |
| `packages/src/anytime_core/include/anytime_core/anytime_waitable.hpp` | Guard condition / executor integration |
| `.devcontainer/linux/Dockerfile` | Primary Docker config |
| `experiments/yolo/6_run_experiments.sh` | Main YOLO experiment orchestrator |
| `experiments/monte_carlo/run_monte_carlo_experiments.sh` | Monte Carlo orchestrator |

---

## Verification Checklist

- [ ] Unit tests pass: `colcon test --packages-select anytime_core anytime_monte_carlo`
- [ ] GPU tests pass (if hardware available): `colcon test --packages-select anytime_yolo`
- [ ] Smoke test: `anytime-eval test --smoke` completes in <2 min
- [ ] Full automated setup: `anytime-eval run-all --mode cpu` runs end-to-end
- [ ] CLI help: `anytime-eval --help` displays all subcommands
- [ ] Documentation renders correctly in markdown, links work
- [ ] Sync/async equivalence: same YOLO input produces same detections in both modes
- [ ] Pre-built Docker images pull successfully from GHCR
