# Holographic GPS Pathfinder - Project Wins

This document outlines potential improvements, features, and refactors for the HoloGPS mod, categorized by difficulty (Easy to Hard).

## 🟢 Easy Wins (Low Effort, High Impact)

### 1. Teleporter Line Support
**The Problem**: Currently, the BFS traverses physically adjacent sectors. If a path requires taking a teleporter to reach a key or an exit, the topological graph might hit a dead end.
**The Fix**: Add logic in the BFS graph construction to check for `Teleport` line specials, linking the source sector to the destination sector (`args[0]` TID location) as adjacent nodes.

### 2. User Customization (CVARs)
**The Problem**: Players have different visual preferences. Some might find the bright blue arrows distracting, or the spacing too tight/loose.
**The Fix**: Add `CVar` options in `MENUDEF` for:
- Marker Color (Cyan, Red, Green, etc.)
- Marker Spacing Density
- Arrow Opacity/Translucency
- Toggle for "Prioritize Secrets"

### 3. Asynchronous/Throttled Pathfinding
**The Problem**: On massive megawads (e.g., *Sunder*, *Eviternity*), running a full topological BFS across 10,000+ sectors in a single tick could cause micro-stutters.
**The Fix**: Limit the BFS `while` loop to process a maximum number of sectors per frame, carrying the `queue` over to the next tick if it exceeds the limit.

## 🟡 Medium Wins (Moderate Effort, High Impact)

### 1. 3D Floor & Bridge Awareness
**The Problem**: The current `IsPortalPassable` relies on standard `floorplane` and `ceilingplane` checks. It doesn't account for GZDoom 3D floors, meaning it might try to route the player *under* a bridge when they need to go *over* it, or vice versa.
**The Fix**: Implement checks using `Sector.Get3DFloorCount()` and iterate through the 3D floors at the portal boundary to find a valid clearance gap.

### 2. Intelligent Trap/Walkover Filtering
**The Problem**: The Reverse BFS switch-finder looks for *any* line that targets the barrier's tag. Mappers sometimes use walkover lines (W1) with the same tag to *close* a door or spring a trap.
**The Fix**: Refine the line scanner to heavily prioritize `SPAC_Use` lines over `SPAC_Cross` lines, or filter out specific `Door_Close` and `Floor_Raise` specials so the pathfinder doesn't accidentally route the player into a trap.

### 3. A* Algorithm Implementation
**The Problem**: BFS finds the path with the fewest *sector transitions*, which can sometimes result in jagged, zig-zagging paths if a level has heavily fragmented sector geometry.
**The Fix**: Upgrade the BFS to an A* algorithm using Euclidean distance to the target as the heuristic. This will yield much smoother, more "human-like" routes.

## 🔴 Hard Wins (High Effort, Massive Impact)

### 1. Polyobject Avoidance
**The Problem**: Hexen-style Polyobjects (moving walls) don't alter the underlying sector topology. The pathfinder currently treats sectors with Polyobjects inside them as fully empty, meaning arrows can draw straight through a moving Polyobject wall.
**The Fix**: Add dynamic intersection checks against Polyobject line arrays during the `PathIsBlocked` raycast and `IsPortalPassable` evaluations.

### 2. Threat & Hazard Avoidance
**The Problem**: The pathfinder always takes the shortest route, even if it goes straight through a pool of 20% damage nukage or a room with 4 Cyberdemons.
**The Fix**: Apply a "weight" to sector nodes in the pathfinding graph based on sector damage properties and enemy proximity. The pathfinder would then dynamically route around dangerous areas if a slightly longer, safer detour is available.

### 3. Scripted (ACS) Switch Deduction
**The Problem**: Modern advanced WADs often open doors using `ACS_Execute` scripts rather than direct line tags. Since the pathfinder can't parse compiled ACS byte-code, it won't know which switch opens the door.
**The Fix**: Build a brute-force sandbox simulation where the pathfinder invisibly "presses" unknown switches in a simulated game-state copy to see if the barrier sector geometry changes, then maps the successful switch. (Highly experimental, but incredibly powerful).
