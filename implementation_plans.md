# Implementation Plan - Puzzle-Solving Pathfinder

Provide dynamic puzzle-solving logic to the HoloGPS Pathfinder. Currently, the pathfinder points topologically through closed doors and barriers, ignoring whether they are physically impassable or require a switch to open (as seen in Doom 2 Map 02). 

## Proposed Changes

### Pathfinder Mod

#### [MODIFY] [HoloGPSHandler.zs](file:///home/sofia/Documents/GitHub/uzdoom/mods/pathfinder/ZScript/HoloGPSHandler.zs)
1. **Dynamic Passability Check:**
   Add a new method `IsPortalPassable(Sector curSec, Sector nextSec, Line ln)` to dynamically evaluate if a portal can be walked through by the player. This checks for blocking flags, step heights (<= 24 units), gap clearances (>= 56 units), and if the line is a direct-use door.
2. **Two-Pass BFS Algorithm in `FindPathBFS`:**
   - **Pass 1 (Reachable Zone):** Run a forward BFS from the player using *only* passable portals. If the target is reached, reconstruct the path normally.
   - **Pass 2 (Reverse Puzzle-Solve):** If the target is unreachable, run a reverse topological BFS from the target to find the exact impassable sector (barrier) blocking progress.
3. **Switch Targeting:**
   Once the barrier is identified, retrieve its Sector Tags. Search the entire level for any linedefs (switches, walkovers) that trigger those tags. If a triggering line is located within the player's reachable zone, dynamically route the pathfinder to that switch instead!

## Verification Plan

### Automated Tests
- ZScript compilation via `uzdoom` run from terminal.

### Manual Verification
- Acknowledging the user's issue with Doom 2 Map 02, the pathfinder should now lead the player to the switch in the starting area instead of pointing at the raised barrier. 
