Process Planning
=================

This module contains the :ref:`process planner <ProcessPlanner class>` for monitoring and managing the build process.
:ref:`Dataclasses <Process Planning Dataclasses>` are used for easier handling of
complex data structures. The most important dataclass is :ref:`ProcessState class`.

.. figure:: ../loop.png
    :alt: Main loop of the process planner with an OPC-UA server as a input source.
    :width: 800

    Main loop

.. toctree::
   :maxdepth: 4

   process_planner
   process_state
   pp_data_class
   process_planning_util
