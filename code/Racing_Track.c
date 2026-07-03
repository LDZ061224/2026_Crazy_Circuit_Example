/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Racing_Track.c
Description: Default build track — 31 actions for reference.
              Racing_track_Typedef removed from Ctrl.h;
              track now defined as flat enum array
              Default_Build_Actions[] in Ctrl.c.
Others: NONE
**************************************************/

/*
 * Node/segment map (17 nodes, 18 segments, 14 elements):
 *
 * seg 0: [ELEM_SHORT][ELEM_SHORT]  → NODE_STRAIGHT
 * seg 1: (none)                    → NODE_LEFT
 * seg 2: [ELEM_SHORT][ELEM_SHORT]  → NODE_LEFT
 * seg 3: (none)                    → NODE_STRAIGHT
 * seg 4: [ELEM_SHORT]              → NODE_LEFT
 * seg 5: (none)                    → NODE_STRAIGHT
 * seg 6: [ELEM_SHORT]              → NODE_LEFT
 * seg 7: (none)                    → NODE_STRAIGHT
 * seg 8: (none)                    → NODE_STRAIGHT
 * seg 9: [ELEM_LEFT]               → NODE_STRAIGHT
 * seg10: [ELEM_LEFT]               → NODE_LEFT
 * seg11: [ELEM_SHORT]              → NODE_RIGHT
 * seg12: (none)                    → NODE_RIGHT
 * seg13: [ELEM_SHORT]              → NODE_LEFT
 * seg14: (none)                    → NODE_LEFT
 * seg15: [ELEM_SHORT]              → NODE_STRAIGHT
 * seg16: (none)                    → NODE_LEFT
 * seg17: [ELEM_SHORT][ELEM_SHORT]  → (end)
 *
 * 31 actions (flat): 4,4,1, 2, 4,4,2, 1, 4,2, 1, 4,2, 1, 1, 6,1, 6,2, 4,3, 3, 4,2, 2, 4,1, 2, 4,4
 */
