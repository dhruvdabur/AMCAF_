#!/usr/bin/env python3
"""
e-Yantra Project Setup Script
Run once after creating your project from this template.
Requires Python 3.8+
"""

import re
import sys
from pathlib import Path
from datetime import datetime

ROOT = Path(__file__).parent.resolve()

PROJECT_TYPES = [
    "Hardware & Embedded Systems",
    "ROS 2 / Robotics",
    "Software / Web",
    "FPGA / Digital Design",
]

TYPE_KEY = {
    "Hardware & Embedded Systems": "hardware",
    "ROS 2 / Robotics": "ros",
    "Software / Web": "software",
    "FPGA / Digital Design": "fpga",
}

FOLDERS = {
    "hardware": [
        ("src",               "Source code (C/C++/MicroPython/Arduino)"),
        ("firmware",          "Compiled binaries, hex files, bootloaders"),
        ("hardware/schematics","Circuit schematics (KiCad, Eagle, PDF exports)"),
        ("hardware/pcb",      "PCB layout files"),
        ("hardware/bom",      "Bill of Materials (CSV/Excel)"),
        ("hardware/3d_models","3D-printed enclosures and mechanical parts"),
        ("tests",             "Test scripts and hardware validation"),
        ("scripts",           "Build, flash, and utility scripts"),
        ("docs",              "Datasheets, design notes, references"),
        ("assets",            "Images and diagrams for documentation"),
    ],
    "ros": [
        ("src",    "ROS 2 packages (one subdirectory per package)"),
        ("launch", "Top-level launch files"),
        ("config", "Parameter YAML files"),
        ("urdf",   "Robot description files (URDF/xacro)"),
        ("worlds", "Gazebo simulation worlds"),
        ("scripts","Utility and environment setup scripts"),
        ("docs",   "Architecture docs, design notes"),
        ("assets", "Images, diagrams, demo GIFs"),
    ],
    "software": [
        ("frontend", "Frontend application (React/Vue/etc.)"),
        ("backend",  "Backend application and API server"),
        ("api",      "API specifications (OpenAPI/Swagger)"),
        ("scripts",  "Dev, build, and deployment scripts"),
        ("tests",    "Integration and end-to-end tests"),
        ("docs",     "Architecture docs, design notes"),
        ("assets",   "Images, icons, and static resources"),
    ],
    "fpga": [
        ("rtl",         "RTL source files (Verilog/VHDL/SystemVerilog)"),
        ("tb",          "Testbench files"),
        ("constraints", "Timing and pin constraint files (XDC/SDC/QSF)"),
        ("sim",         "Simulation scripts and waveform outputs"),
        ("synth",       "Synthesis scripts and implementation reports"),
        ("ip",          "IP cores and vendor primitives"),
        ("scripts",     "Build, program, and automation scripts (Tcl/Python)"),
        ("docs",        "Design docs, block diagrams, timing diagrams"),
        ("assets",      "Waveform screenshots and block diagram images"),
    ],
}

GITIGNORE = {
    "hardware": """\
# OS
.DS_Store
Thumbs.db
desktop.ini

# Build artifacts
build/
*.hex
*.elf
*.bin
*.map
*.lst

# Editor
.vscode/
.idea/
*.swp
*~

# Python
__pycache__/
*.py[cod]
.venv/
venv/
*.egg-info/

# PlatformIO
.pio/
""",
    "ros": """\
# OS
.DS_Store
Thumbs.db
desktop.ini

# ROS 2 / colcon
build/
install/
log/

# Gazebo
.gazebo/

# Editor
.vscode/
.idea/
*.swp
*~

# Python
__pycache__/
*.py[cod]
.venv/
venv/

# ROS bag files
*.db3
*.mcap
""",
    "software": """\
# OS
.DS_Store
Thumbs.db
desktop.ini

# Node.js
node_modules/
dist/
.next/
.nuxt/
.npm
.yarn-integrity

# Python
__pycache__/
*.py[cod]
.venv/
venv/
*.egg-info/
.mypy_cache/

# Environment — never commit secrets
.env
.env.local
.env.*.local
!.env.example

# Editor
.vscode/
.idea/
*.swp
*~

# Build output
build/
*.log
""",
    "fpga": """\
# OS
.DS_Store
Thumbs.db
desktop.ini

# Xilinx Vivado generated files
.Xil/
*.jou
*.log
*.str
*.runs/
*.cache/
*.hw/
*.ip_user_files/
*.sim/
*.gen/
NA/

# Intel Quartus generated files
db/
incremental_db/
output_files/
*.qws
greybox_tmp/

# ModelSim / Questa
work/
transcript
*.wlf

# Icarus Verilog / Verilator
*.vvp
*.vcd
obj_dir/

# Editor
.vscode/
.idea/
*.swp
*~

# Python scripts
__pycache__/
*.py[cod]
.venv/
venv/
""",
}

WIKI_GUIDE = """\
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
"""

MIT_LICENSE = """\
MIT License

Copyright (c) {year} {mentor} — e-Yantra, IIT Bombay

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""


# ---------------------------------------------------------------------------
# Prompting helpers
# ---------------------------------------------------------------------------

def _divider():
    print("\n" + "─" * 52)


def ask(prompt, options=None, default=None):
    if options:
        print(f"\n{prompt}")
        for i, opt in enumerate(options, 1):
            tag = "  (default)" if default == opt else ""
            print(f"  [{i}] {opt}{tag}")
        while True:
            raw = input("  Choice: ").strip()
            if not raw and default is not None:
                return default
            try:
                idx = int(raw) - 1
                if 0 <= idx < len(options):
                    return options[idx]
            except ValueError:
                pass
            print(f"  Please enter a number from 1 to {len(options)}.")
    else:
        suffix = f" [{default}]" if default is not None else ""
        while True:
            val = input(f"{prompt}{suffix}: ").strip()
            if val:
                return val
            if default is not None:
                return default
            print("  This field is required.")


def ask_names(prompt):
    print(f"\n{prompt} (separate multiple names with commas)")
    raw = input("  > ").strip()
    names = [n.strip() for n in raw.split(",") if n.strip()]
    return names if names else ["TBD"]


# ---------------------------------------------------------------------------
# File / folder generation
# ---------------------------------------------------------------------------

def create_folders(type_key):
    folders = FOLDERS[type_key]
    tree = [f"{ROOT.name}/"]
    for i, (path, comment) in enumerate(folders):
        marker = "└──" if i == len(folders) - 1 else "├──"
        d = ROOT / path
        d.mkdir(parents=True, exist_ok=True)
        if not any(d.iterdir()):
            (d / ".gitkeep").touch()
        tree.append(f"{marker} {(path + '/'):.<28} {comment}")
        print(f"  ✓  {path}/")
    return "\n".join(tree)


def patch_readme(info, tree):
    path = ROOT / "README.md"
    if not path.exists():
        print("  !  README.md not found — skipping.")
        return

    text = path.read_text(encoding="utf-8")

    # Remove the template notice block
    text = re.sub(
        r"<!-- TEMPLATE_NOTICE_START -->.*?<!-- TEMPLATE_NOTICE_END -->\n?",
        "",
        text,
        flags=re.DOTALL,
    )

    # Keep relevant category sections, remove the others
    all_tags = {
        "hardware": "HARDWARE_ONLY",
        "ros":      "ROS_ONLY",
        "software": "SOFTWARE_ONLY",
        "fpga":     "FPGA_ONLY",
    }
    type_key = info["type_key"]
    for key, tag in all_tags.items():
        start = f"<!-- [{tag}_START] -->"
        end   = f"<!-- [{tag}_END] -->"
        if key == type_key:
            text = text.replace(start + "\n", "").replace(end + "\n", "")
        else:
            text = re.sub(
                re.escape(start) + r".*?" + re.escape(end) + r"\n?",
                "",
                text,
                flags=re.DOTALL,
            )

    # Build intern table rows
    intern_rows = "\n".join(f"| {n} | Intern |" for n in info["interns"])

    # URL-encoded badge label for shields.io
    badge_label = {
        "hardware": "Hardware%20%26%20Embedded",
        "ros":      "ROS2%20%2F%20Robotics",
        "software": "Software%20%2F%20Web",
        "fpga":     "FPGA%20%2F%20Digital%20Design",
    }[type_key]

    replacements = {
        "{{PROJECT_NAME}}":      info["name"],
        "{{SHORT_DESCRIPTION}}": info["description"],
        "{{PROJECT_TYPE}}":      info["project_type"],
        "{{PROJECT_TYPE_BADGE}}":badge_label,
        "{{MENTOR_NAME}}":       info["mentor"],
        "{{INTERN_TABLE}}":      intern_rows,
        "{{YEAR}}":              str(datetime.now().year),
        "{{REPO_NAME}}":         ROOT.name,
        "{{PROJECT_STRUCTURE}}": tree,
    }
    for placeholder, value in replacements.items():
        text = text.replace(placeholder, value)

    path.write_text(text, encoding="utf-8")
    print("  ✓  README.md")


def write_gitignore(type_key):
    (ROOT / ".gitignore").write_text(GITIGNORE[type_key], encoding="utf-8")
    print("  ✓  .gitignore")


def write_license(info):
    text = MIT_LICENSE.format(year=datetime.now().year, mentor=info["mentor"])
    (ROOT / "LICENSE").write_text(text, encoding="utf-8")
    print("  ✓  LICENSE")


def write_ros_stub(name):
    pkg = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")
    pkg_dir = ROOT / "src" / pkg
    pkg_dir.mkdir(parents=True, exist_ok=True)

    (pkg_dir / "package.xml").write_text(
        f'<?xml version="1.0"?>\n'
        f'<?xml-model href="http://download.ros.org/schema/package_format3.xsd"'
        f' schematypens="http://www.w3.org/2001/XMLSchema"?>\n'
        f"<package format=\"3\">\n"
        f"  <name>{pkg}</name>\n"
        f"  <version>0.0.1</version>\n"
        f"  <description>{name}</description>\n"
        f"  <maintainer email=\"intern@e-yantra.org\">Intern</maintainer>\n"
        f"  <license>MIT</license>\n\n"
        f"  <buildtool_depend>ament_cmake</buildtool_depend>\n"
        f"  <depend>rclcpp</depend>\n"
        f"  <depend>std_msgs</depend>\n\n"
        f"  <test_depend>ament_lint_auto</test_depend>\n"
        f"  <test_depend>ament_lint_common</test_depend>\n\n"
        f"  <export>\n"
        f"    <build_type>ament_cmake</build_type>\n"
        f"  </export>\n"
        f"</package>\n",
        encoding="utf-8",
    )

    (pkg_dir / "CMakeLists.txt").write_text(
        f"cmake_minimum_required(VERSION 3.8)\n"
        f"project({pkg})\n\n"
        f"if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES \"Clang\")\n"
        f"  add_compile_options(-Wall -Wextra -Wpedantic)\n"
        f"endif()\n\n"
        f"find_package(ament_cmake REQUIRED)\n"
        f"find_package(rclcpp REQUIRED)\n"
        f"find_package(std_msgs REQUIRED)\n\n"
        f"install(DIRECTORY launch config urdf worlds\n"
        f"  DESTINATION share/${{PROJECT_NAME}}/\n"
        f"  OPTIONAL\n"
        f")\n\n"
        f"ament_package()\n",
        encoding="utf-8",
    )

    launch_dir = pkg_dir / "launch"
    launch_dir.mkdir(exist_ok=True)
    (launch_dir / f"{pkg}.launch.py").write_text(
        f"from launch import LaunchDescription\n"
        f"from launch_ros.actions import Node\n\n\n"
        f"def generate_launch_description():\n"
        f"    return LaunchDescription([\n"
        f"        Node(\n"
        f"            package=\"{pkg}\",\n"
        f"            executable=\"main\",\n"
        f"            name=\"{pkg}_node\",\n"
        f"            output=\"screen\",\n"
        f"        ),\n"
        f"    ])\n",
        encoding="utf-8",
    )

    print(f"  ✓  src/{pkg}/  (ROS 2 package stub with CMakeLists, package.xml, launch/)")


def write_wiki_guide():
    docs_dir = ROOT / "docs"
    docs_dir.mkdir(exist_ok=True)
    (docs_dir / "wiki-guide.md").write_text(WIKI_GUIDE, encoding="utf-8")
    print("  ✓  docs/wiki-guide.md")


def write_env_example():
    (ROOT / ".env.example").write_text(
        "# Application\n"
        "APP_ENV=development\n"
        "APP_PORT=8000\n"
        "APP_SECRET_KEY=change-this-in-production\n\n"
        "# Database\n"
        "DB_HOST=localhost\n"
        "DB_PORT=5432\n"
        "DB_NAME=myapp\n"
        "DB_USER=myuser\n"
        "DB_PASSWORD=change-this\n\n"
        "# Add other environment variables your project needs below\n",
        encoding="utf-8",
    )
    print("  ✓  .env.example")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("=" * 52)
    print("  e-Yantra Project Setup")
    print("  IIT Bombay")
    print("=" * 52)
    print()
    print("Answer a few questions to configure this template.")

    name         = ask("\nProject name")
    description  = ask("Short description (1–2 sentences)")
    project_type = ask("Project category", options=PROJECT_TYPES)
    interns      = ask_names("Intern name(s)")
    mentor       = ask("\nMentor name")

    type_key = TYPE_KEY[project_type]
    info = {
        "name":         name,
        "description":  description,
        "project_type": project_type,
        "type_key":     type_key,
        "interns":      interns,
        "mentor":       mentor,
    }

    _divider()
    print("Creating folder structure...")
    tree = create_folders(type_key)

    _divider()
    print("Generating project files...")
    patch_readme(info, tree)
    write_gitignore(type_key)
    write_license(info)
    write_wiki_guide()

    if type_key == "ros":
        write_ros_stub(name)
    elif type_key == "software":
        write_env_example()

    _divider()
    print(f"\n  Setup complete!\n")
    print(f"  Project  : {name}")
    print(f"  Category : {project_type}")
    print(f"  Intern(s): {', '.join(interns)}")
    print(f"  Mentor   : {mentor}")
    print()
    print("  Next steps:")
    print("  1. Review README.md and fill in the Usage section")
    print("  2. Enable GitHub Wiki — follow docs/wiki-guide.md")
    print("  3. Commit everything and share the repo URL with your intern(s)")
    print()

    remove = ask("Remove setup.py now that setup is complete?",
                 options=["Yes", "No"], default="Yes")
    if remove == "Yes":
        Path(__file__).unlink(missing_ok=True)
        print("\n  setup.py removed. You're all set!")
    print()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nSetup cancelled.")
        sys.exit(0)
