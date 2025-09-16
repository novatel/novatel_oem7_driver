# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('..'))


# -- Project information -----------------------------------------------------

project = 'novatel_oem7_driver'
copyright = '2025, NovAtel'
author = 'NovAtel'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'alabaster'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']


# -- Docuementation Formatting - Guidelines for development only. Follow Before Publishing!---------------

# Spelling Check. To run a spell check, follow the below:
#   1) Uncomment the below lines
# extensions = ['sphinxcontrib.spelling']
# spelling_lang = 'en_CA'
# spelling_word_list_filename = ['spelling_wordlist.txt']
#   2) Install requirements for sphinxcontrib-spelling (pip install sphinxcontrib-spelling)
#   3) Build the documentation with sphinx-build -b spelling src/novatel_oem7_driver/doc spelling_output

# Link Check. To check links, you can check it directly as:
#   1) sphinx-build -b linkcheck src/novatel_oem7_driver/doc link_output

