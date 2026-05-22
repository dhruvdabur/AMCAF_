# GitHub Wiki — Progress Log Guide

This file explains what GitHub Wiki is and how to use it to document your
project progress. Read this before you start — keeping a good Wiki is a
required part of working on this project.

---

## What is GitHub Wiki?

Every GitHub repository has a built-in Wiki: a documentation space attached
to your repo. Think of it as your **project notebook on GitHub**.

The README is for *setting up and running* the project.
The Wiki is for *documenting your journey*:

- What you built each week and what decisions you made
- Why you chose one approach over another
- Challenges you hit and how you solved them
- Diagrams, screenshots, test results, and progress notes

Your mentor will check the Wiki regularly — keep it updated.

---

## Step 1 — Enable the Wiki

1. Go to your repository on GitHub
2. Click the **Settings** tab at the top of the repo page
3. Scroll down to the **Features** section
4. Tick the **Wikis** checkbox
5. A **Wiki** tab will now appear in the repo navigation bar — click it

---

## Step 2 — Create Your Home Page

The first page you create is the **Home** page. It acts as a table of
contents linking to all other pages.

Paste this as your Home page content and fill in the blanks:

```markdown
# <Project Name> — Wiki

| | |
|---|---|
| **Mentor** | <Name> |
| **Intern(s)** | <Your Name(s)> |
| **Repo** | [Link](<your-repo-url>) |

## Pages
- [Project Overview](Project-Overview)
- [Setup and Environment](Setup-and-Environment)
- [Progress Log](Progress-Log)
- [Design Decisions](Design-Decisions)
- [Final Report](Final-Report)
```

After saving, create each of those pages by clicking their links.

---

## Step 3 — Log Your Progress

Create a page called **Progress Log**. Add a new entry every week, or after
completing each milestone. This is the most important part of the Wiki.

---

### Weekly Log Template

```markdown
## Week N — Short Title
**Date:** YYYY-MM-DD to YYYY-MM-DD

### What I did
-

### What I learned
-

### Challenges
-

### Plan for next week
-
```

---

### Milestone Log Template

```markdown
## Milestone N — Title
**Completed:** YYYY-MM-DD

### What was built
-

### How it works
Brief description of the approach.

### Challenges and solutions
-

### Screenshots / Demo
(Drag and drop images directly into the Wiki editor — GitHub uploads them.)

### What is next
-
```

---

## Tips

- **Add images freely** — drag and drop screenshots, waveform captures,
  circuit photos, or terminal output straight into the GitHub Wiki editor.
  GitHub hosts them automatically.
- **Write about blockers too** — if something did not work, write that down.
  Your mentor wants to see your reasoning process, not just successes.
- **Update the same day** — a Wiki updated days later is written from memory
  and misses important details. Write it while the work is fresh.
- **Link to commits** — paste a GitHub commit URL into the Wiki to reference
  the exact code change you are describing.
- **Short and honest beats long and vague** — bullet points are fine.
