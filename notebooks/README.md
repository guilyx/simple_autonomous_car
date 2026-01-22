# Notebooks

This directory contains Jupyter notebooks for learning and experimenting with the Simple Autonomous Car SDK.

## Structure

- **[tutorials/](tutorials/)** - Step-by-step tutorials for learning SDK components
- **[building/](building/)** - Guides for building custom components and simulations
- **[examples/](examples/)** - Example notebooks demonstrating specific use cases

## Getting Started

1. **New to the SDK?** Start with the tutorials in `tutorials/`
2. **Want hands-on learning?** Try the fill-in-the-blank notebooks in `learning/`
3. **Want to build custom components?** Check out the complete guides in `building/`
4. **Looking for examples?** Browse `examples/` for specific use cases

## Running Notebooks

All notebooks assume you're running from the repository root. They use `sys.path.insert(0, '../../src')` to import the SDK (since they're in subfolders).

```bash
# From repository root
jupyter notebook notebooks/
```

## Prerequisites

- Python 3.10+
- Jupyter Notebook or JupyterLab
- All SDK dependencies (install with `uv sync`)
