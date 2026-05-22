# Contributing Guidelines

This document covers the conventions all interns and mentors should follow when working in this repository.

---

## Branch Naming

Always branch off from `main`. Use lowercase letters and hyphens only.

| Type | Pattern | Example |
|------|---------|---------|
| New feature | `feat/<short-description>` | `feat/obstacle-avoidance` |
| Bug fix | `fix/<short-description>` | `fix/imu-calibration-offset` |
| Documentation | `docs/<short-description>` | `docs/add-wiring-diagram` |
| Refactor | `refactor/<short-description>` | `refactor/motor-driver` |
| Experiment / WIP | `exp/<short-description>` | `exp/pid-auto-tuning` |

---

## Commit Messages

Use [Conventional Commits](https://www.conventionalcommits.org/) — a short type prefix, a colon, and a brief description in present tense.

```
feat: add ultrasonic distance sensor driver
fix: correct PWM duty cycle range for servo
docs: add wiring diagram for motor controller
test: add unit tests for PID controller
chore: update rosdep dependencies
refactor: simplify I2C read/write helpers
```

**Rules:**
- Present tense: "add sensor" not "added sensor"
- Keep the first line under 72 characters
- For more detail, add a blank line after the first line, then a body paragraph

---

## Pull Requests

1. Keep your branch up to date with `main` before opening a PR (`git pull origin main`)
2. Fill in the PR template completely
3. Request a review from your **mentor**
4. Do **not** merge until the mentor has approved

For large features, consider opening a draft PR early so the mentor can give feedback while work is in progress.

---

## Code Review

- Address every review comment before merging — either fix it or leave a reply explaining your reasoning
- Small, focused PRs are much easier to review than large ones; prefer one feature per PR
- Avoid mixing unrelated changes in a single PR

---

## Wiki Updates

After completing a significant milestone, update the GitHub Wiki:

- What was built or decided
- Any design trade-offs that were made
- Challenges encountered and how they were resolved

See [`docs/wiki-guide.md`](docs/wiki-guide.md) for the Wiki structure and page templates.
