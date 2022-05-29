ProcessState class
===================

The process state class describes the last known state of the build process.
On each motion event it evaluates the input and modify its instance variables accordingly.
(See :func:`process_state.ProcessState.evaluate_assembly` for :ref:`assembly events <Assembly Events>` and
:func:`process_state.ProcessState.pick_part` for :ref:`pick events <Pick Events>`.

.. automodule:: process_state
   :members:
   :undoc-members:
   :show-inheritance:
