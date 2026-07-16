import os
import sys

project = "BAM"
author = "Marc Duclusaud & Grégoire Passault"
copyright = "2026, Marc Duclusaud & Grégoire Passault"

extensions = [
    "autoapi.extension",
    "sphinx.ext.napoleon",
    "sphinx.ext.mathjax",
    "sphinx.ext.todo",
    "sphinx_copybutton",
    "sphinxcontrib.video",
    "sphinx_design",
]

templates_path = ["_templates"]
exclude_patterns = ["_build", ".venv", "Thumbs.db", ".DS_Store"]

sys.path.insert(0, os.path.abspath(".."))

# sphinx-autoapi — scans bam/ via AST, no runtime imports needed
autoapi_type = "python"
autoapi_dirs = ["../bam"]
autoapi_root = "autoapi"
autoapi_ignore = ["*/drive_backdrive.py", "*/plot.py", "*/jitter.py", "*/mae.py", "*/message.py", "*/process.py"]
autoapi_add_toctree_entry = False
autoapi_options = [
    "members",
    "undoc-members",
    "show-inheritance",
    "show-module-summary",
    "imported-members",
]
autoapi_python_class_content = "both"
autoapi_member_order = "bysource"

napoleon_google_docstring = True
napoleon_numpy_docstring = False
todo_include_todos = True

html_theme = "pydata_sphinx_theme"
pygments_style = "friendly"
pygments_dark_style = "monokai"
html_static_path = ["_static"]
html_css_files = ["bam.css"]
html_logo = "_static/BAM_logo.png"

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
