Encoding information
====================
Motion event codes
****************************
1 -> fitting placed

2 -> pipe placed

3 -> attachment placed

4 -> accepted part from picking robot

5 -> picked part manually

-> removals are determined implicitly

Part ids
******************
-99: Placeholder

-2: Unknown straight pipe

-1: attachment

0: fitting

1-n: straight pipe (corresponding to the amount of nodes it passes)

Picking robot event codes/command codes
*****************************************
-2: STOP

-1: Cancel all picking tasks

0: (go to) neutral state

1: move to pick-up point of part id x and pick part

2: move to return position of part id x and return part # unused

3: move into offering position

4: wait for worker to accept part

Fastening robot event codes
****************************
-2: STOP

-1: remove fastening of positions x from command pipeline (if still in pipeline)

0: (go to) neutral state

1: move to a fastening position and fasten attachment at pos x

2: move to a fastening position and fasten pipe at pos x
