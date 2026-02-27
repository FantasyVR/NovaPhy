import os
import sys

# Add project roots so autodoc can import the Python package/bindings.
sys.path.insert(0, os.path.abspath(".."))
sys.path.insert(0, os.path.abspath("../python"))

project = "PhysicsEngine"
author = "PhysicsEngine Contributors"
release = "0.1.0"

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "breathe",
]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

breathe_projects = {"PhysicsEngine": "./doxygen_output/xml"}
breathe_default_project = "PhysicsEngine"

autodoc_default_options = {
    "members": True,
    "undoc-members": False,
    "show-inheritance": True,
}

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
