Encoding Information
=============================

-> See also :ref:`Constants module`

Motion Event Codes
-------------------------------
- 1: Fitting placed
- 2: Pipe placed
- 3: Attachment placed
- 4: Accepted part from picking robot
- 5: Picked part manually

Currently Code 4 and 5 are treated as the same

State Grid State Codes
-------------------------------
- 0: Free position
- 1: occupied by obstacle
- 2: occupied by part
- 3: transition

Currently Code 4 and 5 are treated as the same

Part IDs
****************
- -99: Placeholder
- -2: Unknown straight pipe
- -1: attachment
- 0: fitting
- 1-n: straight pipe (corresponding to the amount of nodes it passes)

Picking Robot Command Codes
--------------------------------------
- -2: STOP (unused)
- -1: Cancel all picking tasks (unused)
- 0: Go to neutral state
- 1: Move to pick-up point of part id x and pick part
- 2: Move to return position of part id x and return part (unused)
- 3: Move into offering position
- 4: Wait for worker to accept part


Fastening Robot Command Codes
---------------------------------
- -2: STOP (unused)
- -1: Remove fastening of positions x from command pipeline (unused)
- 0: Go to neutral state
- 1: Move to fastening position and fasten attachment at pos x
- 2: Move to fastening position and fasten pipe at pos x


