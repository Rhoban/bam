import os
import sys

project = "BAM"
author = "BAM Contributors"
copyright = "2026, BAM Contributors"

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.mathjax",
    "sphinx.ext.viewcode",
    "sphinx.ext.todo",
    "sphinx_copybutton",
]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

sys.path.insert(0, os.path.abspath(".."))

# External dependencies that are optional or hardware-specific.
autodoc_mock_imports = [
    "mujoco",
    "matplotlib",
    "pygame",
    "serial",
    "dynamixel_sdk",
    "pypot",
    "rustypot",
    "placo",
    "optuna",
    "wandb",
    "zmq",
    "google",
    "google.protobuf",
    "scipy",
]

autoclass_content = "both"
autodoc_member_order = "bysource"
napoleon_google_docstring = True
napoleon_numpy_docstring = False
todo_include_todos = True

html_theme = "pydata_sphinx_theme"
html_static_path = ["_static"]
html_css_files = ["bam.css"]

html_theme_options = {
    "show_nav_level": 2,
    "show_toc_level": 2,
    "navigation_with_keys": True,
    "icon_links": [
        {
            "name": "GitHub",
            "url": "https://github.com/Rhoban/bam",
            "icon": "fa-brands fa-square-github",
            "type": "fontawesome",
        }
    ],
}
