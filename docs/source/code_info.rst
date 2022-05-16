Code Information
=============================

Motion Event Codes
***********************
1: Fitting placed

2: Pipe placed

3: Attachment placed

4: Accepted part from picking robot

5: Picked part manually

-> See :ref:`Constants module`

Currently Code 4 and 5 are treated the same

Picking robot command codes
--------------------------------------
-2: STOP (unused)

-1: Cancel all picking tasks (unused)

0: Go to neutral state

1: Move to pick-up point of part id x and pick part

2: Move to return position of part id x and return part (unused)

3: Move into offering position

4: Wait for worker to accept part


Fastening robot commands
-----------------------------
-2: STOP (unused)

-1: Remove fastening of positions x from command pipeline (unused)

0: Go to neutral state

1: Move to fastening position and fasten attachment at pos x

2: Move to fastening position and fasten pipe at pos x


