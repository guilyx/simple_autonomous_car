# Installation

This guide covers installing the Simple Autonomous Car SDK.

## Requirements

- Python >= 3.10
- NumPy >= 1.24.0
- Matplotlib >= 3.7.0
- SciPy >= 1.10.0

## Installation Methods

### From PyPI (Recommended)

The easiest way to install the SDK:

```bash
pip install simple-autonomous-car
```

### From Source

If you want to install from source or contribute:

```bash
# Clone the repository
git clone https://github.com/yourusername/simple-autonomous-car.git
cd simple-autonomous-car

# Install in editable mode
pip install -e .
```

### Using uv (Recommended for Development)

[uv](https://github.com/astral-sh/uv) is a fast Python package installer:

```bash
# Install uv if not already installed
curl -LsSf https://astral.sh/uv/install.sh | sh

# Install the package
uv pip install simple-autonomous-car
```

For development:

```bash
# Clone the repository
git clone https://github.com/yourusername/simple-autonomous-car.git
cd simple-autonomous-car

# Install with development dependencies
uv sync --dev
```

### Using pip with requirements.txt

```bash
# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Linux/Mac
# or
venv\Scripts\activate  # On Windows

# Install dependencies
pip install -r requirements.txt
```

## Verify Installation

Test that the installation works:

```python
from simple_autonomous_car import Track, Car, CarState

# Create a simple track
track = Track.create_simple_track()
print(f"Track created with {len(track.centerline)} points")

# Create a car
car = Car(initial_state=CarState(x=0.0, y=0.0, heading=0.0))
print("Car created successfully!")

print("✓ Installation verified!")
```

## Development Dependencies

For development, install additional dependencies:

```bash
# Using uv
uv sync --dev

# Using pip
pip install -e ".[dev]"
```

Development dependencies include:
- `pytest` - Testing framework
- `pytest-cov` - Coverage reporting
- `black` - Code formatting
- `ruff` - Linting
- `mypy` - Type checking
- `pre-commit` - Git hooks
- `jupyter` - For notebooks
- `ipykernel` - Jupyter kernel

## Troubleshooting

### Import Errors

If you get import errors, ensure:
1. The package is installed: `pip list | grep simple-autonomous-car`
2. You're using the correct Python environment
3. Your `PYTHONPATH` is set correctly (if installing from source)

### Matplotlib Backend Issues

If you encounter display issues with matplotlib:

```python
import matplotlib
matplotlib.use('TkAgg')  # or 'Qt5Agg', 'Agg', etc.
```

### Version Conflicts

If you encounter version conflicts:
1. Use a virtual environment
2. Check `requirements.txt` for version constraints
3. Update conflicting packages

## Next Steps

Once installed, proceed to the [Quick Start Guide](quickstart.md) to get started!
