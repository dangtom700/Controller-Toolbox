# Contributing to Controller Toolbox

First off, thank you for considering contributing to Controller Toolbox!

## Code of Conduct
By participating in this project, you agree to abide by our Code of Conduct.

## How Can I Contribute?

### Reporting Bugs
- Ensure the bug was not already reported.
- Open a new issue with a clear title and description, including steps to reproduce.

### Suggesting Enhancements
- Open an issue describing the enhancement.
- Explain why this enhancement would be useful.

### Pull Requests
- Fork the repo and create your branch from `main`.
- If you've added code that should be tested, add tests.
- Ensure the test suite passes (run `ctest`).
- Make sure your code lints (we use `clang-tidy`, see `.clang-tidy`).
- Issue that pull request!

## Development Setup
- Use CMake 3.16+ and a C++20 compiler.
- Tests use Catch2 (downloaded automatically).
- Eigen is required for building.

## Algorithm Documentation
If you are adding a new control algorithm or modifying an existing one, please ensure your changes are accompanied by clear mathematical documentation in the header files. We value derivations over simple summaries.
