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
import os
import sys

sys.path.insert(0, os.path.abspath('../../../'))
sys.path.insert(0, os.path.abspath('../../../PipeLab'))
sys.path.insert(0, os.path.abspath('../../../PipeLab/FunctionTestGUI'))
sys.path.insert(0, os.path.abspath('../../../PipeLab/OPC_UA'))
sys.path.insert(0, os.path.abspath('../../../PipeLab/PathFinding'))
sys.path.insert(0, os.path.abspath('../../../PipeLab/ProcessPlanning'))
sys.path.insert(0, os.path.abspath('../../../PipeLab/PathFinding/pf_data_class'))
sys.path.insert(0, os.path.abspath('../../../PipeLab/PathFinding/path_finding_util'))
sys.path.insert(0, os.path.abspath('../../../PipeLab/ProcessPlanning/pp_data_class'))
sys.path.insert(0, os.path.abspath('../../../PipeLab/ProcessPlanning/process_planning_util'))
sys.path.insert(0, os.path.abspath('../../../PipeLab/type_dictionary'))



print(sys.path[0])

# -- Project information -----------------------------------------------------

project = 'PipeLab'
copyright = '2022, Richard Reider'
author = 'Richard Reider'

# The full version, including alpha/beta/rc tags
release = '2022'

html_theme = "sphinx_rtd_theme"

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = ['sphinx.ext.napoleon',
              "sphinx.ext.autodoc",
              "sphinx_paramlinks",
              # "sphinx.ext.coverage",
              # "sphinx.ext.viewcode",
              "sphinx.ext.autosectionlabel",
              ]  # "]#, "sphinx_autodoc_typehints"]

napoleon_use_param = True

autodoc_typehints = "none"

# extensions.append("autoapi.extension")
#
# autoapi_type = 'python'
# autoapi_dirs = ['.', '../../PipeLab']

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []

autodoc_type_aliases = {
    "Pos": "type_aliases.Pos",
    "Node": "type_aliases.Node",
    "Path": "type_aliases.Path",
    "NodePath": "type_aliases.NodePath",
    "Trail": "type_aliases.Trail",
    "NodeTrail": "type_aliases.NodeTrail",
    "DirectedConnection": "type_aliases.DirectedConnection",
    "UndirectedConnection": "type_aliases.UndirectedConnection",
    "StateGrid": "type_aliases.StateGrid",
    "MotionEvent": "type_aliases.MotionEvent",
    "BuildingInstructions": "class_types.BuildingInstructions",
    "Action": "type_aliases.Action",
    "NodePair": "type_aliases.NodePair",
    "NodePairSet": "type_aliases.NodePairSet",
    "OrderedTrails": "type_aliases.OrderedTrails",
    "TrailList": "type_aliases.TrailList",
    "DirectionDict": "type_aliases.DirectionDict",
    "PosSet": "type_aliases.PosSet"
}

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']
