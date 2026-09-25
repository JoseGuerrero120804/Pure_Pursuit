# pure_pursuit 🚗✨

Welcome to the pure_pursuit package! 🐣

This directory contains the source, configuration, resources, and tests for the pure_pursuit module used in the mobility project. The package provides pure pursuit related functionality (path following, guidance helpers, etc.).

Summary
- Name: pure_pursuit
- Purpose: Path-following / guidance helper code used by the mobility project 🧭
- Main contents: Python package code, configuration files, resources, and tests 📁

Quick file overview
- config/ — configuration files (e.g., tuning parameters) ⚙️
- pure_pursuit/ — Python package source code (the library) 🐍
- resource/ — extra assets and resources used by the package 🗂️
- test/ — unit and integration tests (pytest) ✅
- package.xml, setup.py — package metadata and installation helpers 📦
- setup.cfg — test and linter configuration

Getting started
1. Install locally (editable) for development:

   pip install -e .

2. If this project is part of a ROS workspace, build it with your workspace tooling:

   - ROS 1 (catkin): catkin_make / catkin build (from workspace root)
   - ROS 2 (colcon): colcon build (from workspace root)

   Use whichever tool matches your workspace/setup. 🛠️

Running tests
- Run tests with pytest from the package root:

  pytest -q

Notes
- The project provides Python packaging via setup.py. Use pip install -e . during development to pick up live edits.
- Configuration files in `config/` control tuning parameters — adjust them carefully and add versioned copies for experiments. 🔬

Contributing
- Contributions welcome! Please open a PR with a clear description of changes and include tests for new behavior. Be kind and include a short note about why the change helps. 🤝

Style & linting
- setup.cfg contains test and style settings used by the project. Follow the existing style rules when editing code. ✨

Contact / Help
- If you have questions about how the package is used in the larger project, check the top-level repository README or reach out to the maintainers in your team chat. ❤️

License
- Add or update the LICENSE file at the repo root if not already present. This package doesn't include an explicit license file here — make sure the repository has one. 📜

Thanks for checking out pure_pursuit! Happy coding and safe driving 🚘💖
