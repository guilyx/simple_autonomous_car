# Contributing to Car Loc Viz SDK

Thank you for your interest in contributing! This document provides guidelines for contributing to the project.

## Development Setup

1. Fork the repository
2. Clone your fork:
   ```bash
   git clone https://github.com/yourusername/car-loc-viz.git
   cd car-loc-viz
   ```
3. Install development dependencies:
   ```bash
   uv sync --dev
   ```
4. Install pre-commit hooks:
   ```bash
   uv run pre-commit install
   ```

## Code Style

- Follow PEP 8 style guidelines
- Use `black` for formatting (line length: 100)
- Use `ruff` for linting
- Use `mypy` for type checking
- Maximum line length: 100 characters

## Testing

- Write tests for all new features
- Ensure all tests pass: `uv run pytest`
- Aim for high test coverage
- Add tests to appropriate files in `tests/`

## Documentation

- Update docstrings for all new functions/classes
- Follow NumPy docstring format
- Update relevant documentation files
- Add examples if applicable

## Pull Request Process

1. Create a feature branch from `main`
2. Make your changes
3. Ensure tests pass and code is linted
4. Update documentation
5. Submit a pull request with a clear description

## Commit Messages

- Use clear, descriptive commit messages
- Reference issues when applicable
- Follow conventional commit format when possible

## Questions?

Open an issue for questions or discussions about contributions.
