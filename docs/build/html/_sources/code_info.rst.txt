Encoding Information
=============================

-> See also :ref:`Constants module`

Motion Event Codes
-------------------------------

Assembly Events
~~~~~~~~~~~~~~~~
- 1: Fitting placed
- 2: Pipe placed
- 3: Attachment placed

Pick Events
~~~~~~~~~~~~~~~~
- 4: Accepted part from picking robot
- 5: Picked part manually

Currently Code 4 and 5 are treated as the same

State Grid State Codes
-------------------------------
- 0: Free position
- 1: occupied by obstacle
- 2: occupied by part
- 3: transition

Part IDs
----------------
- -99: Placeholder
- -2: Open ID (currently not determinable)
- -1: attachment
- 0: fitting
- 1-n: straight pipe (corresponding to the amount of nodes it passes)

Picking Robot Command Codes
--------------------------------------
- -2: STOP (unused)
- -1: Cancel all picking tasks (unused)
- 0: Go to neutral state
- (1,x): Move to pick-up point of part id x and pick part
- (2,x): Move to return position of part id x and return part (unused)
- 3: Move into offering position
- 4: Wait for worker to accept part


Fastening Robot Command Codes
---------------------------------
- -2: STOP (unused)
- -1: Remove fastening of positions x from command pipeline (unused)
- 0: Go to neutral state
- (1,x,y): Move to fastening position and fasten attachment at coordinates (x,y)
- (2,x,y): Move to fastening position and fasten pipe at coordinates (x,y)


