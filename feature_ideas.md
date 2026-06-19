# Holographic GPS - Feature Proposals & Brainstorming

This document outlines new advanced features and integrations for the **Holographic GPS PK3 Mod**, detailing both the user experience concept and the technical ZScript/GZDoom implementation strategy.

---

## 1. 2D Automap Path Overlay
* **Concept**: Render the navigation path directly on the GZDoom Automap screen (2D overlay), letting players see the complete route from above without having to switch back and forth.
* **Technical Implementation**:
  - Hook into the `DrawHUD` or `RenderOverlay` event inside `StaticEventHandler`.
  - Check if the automap is active (`automapactive` global flag).
  - Project the 3D path coordinates (`pathX`, `pathY`) using GZDoom's `Screen.FrameToScreen` or draw directly on the map coordinate grid using `AM_DrawLine` (or by drawing custom indicators at each node's sector coordinates).
  - This provides a clean, native-feeling minimap navigation line.

## 2. Client-Side Particle Trails (Zero-Actor Performance)
* **Concept**: Replace or supplement the physical `HoloPathMarker` actors with GZDoom's native particle engine.
* **Technical Implementation**:
  - Instead of spawning and pool-managing `Actor` entities (which still require VM memory allocation, collision, and tick overhead), use GZDoom's native `level.SpawnParticle` API.
  - Spawning particles is executed on the client-side renderer, creating zero server-side tick overhead and bypassing the ZScript GC completely.
  - We can create elegant flowing stream effects, neon dotted trails, or glowing dust paths that trace the route dynamically.

## 3. Hazard & Damage-Floor Warning System
* **Concept**: Visually highlight portions of the path in RED (or orange) when they traverse hazard sectors (e.g., lava, acid, crushing ceilings) so players can prepare or avoid them.
* **Technical Implementation**:
  - During the path tracer/A* phase, query the destination sector properties.
  - Check if the sector does damage (`sec.damageamount > 0` or has a damaging type index like `sec.GetDamageFormula()`).
  - Check if the sector has active sector actions like crushing ceilings (`sec.ceilingplane` movement states).
  - Set a `hazard` flag on those path nodes and render those markers using a red/hazard sprite or color code.

## 4. Cooperative Ping System (Multiplayer Breadcrumbs)
* **Concept**: Allow players in cooperative multiplayer to ping locations, share active paths, or set destinations for their teammates (similar to modern ping wheels in Apex Legends).
* **Technical Implementation**:
  - Add a keybind in `KEYCONF` that triggers a custom network event (`SendNetworkEvent`).
  - Hook `NetworkProcess` in the event handler to capture the player's current reticle aiming coordinates in 3D space (`plyr.mo.AimLine`).
  - Calculate a path from the teammate's position to the pinged coordinates, rendering a specialized waypoint marker on their HUD and world.

## 5. Quest & Target Selector (100% Completion Helper)
* **Concept**: Allow the player to choose their navigation target type via a quick menu or keybind:
  - **Standard**: Keys -> Level Exit (default).
  - **Secrets Hunt**: Route to the nearest undiscovered secret sector.
  - **Purge Mode**: Route to the nearest monster (for 100% kills completion).
  - **Scavenger**: Route to nearby weapons or ammo if low on supplies.
* **Technical Implementation**:
  - Extend the `ThinkerIterator` inside `FindNextObjective` to query based on a new menu CVar (`holo_gps_target_mode`).
  - For Secrets, iterate through sectors looking for `Sector.SecFlags.SEC_SECRET` that haven't been visited.
  - For Monsters, iterate over `Actor` with `bISMONSTER` flags that are alive and active.

## 6. Native C++ Pathfinding Hooks (UZDoom Port Integration)
* **Concept**: Since you are developing a custom engine port (`uzdoom`), we can offload the pathfinding algorithms (A* / Dijkstra) entirely from the ZScript VM to the native C++ engine side.
* **Technical Implementation**:
  - Write a native A* pathfinding implementation inside `uzdoom`'s C++ source code.
  - Bind the C++ pathfinder to ZScript using `native` class bindings (e.g. `native class HoloGPSPathfinder`).
  - Expose a method such as `native static void GetNativePath(...)` which takes the map's geometry arrays and returns the calculated path list directly. This makes path calculations near-instantaneous even on gigantic slaughter maps (like *Sunder* or *Okuplok*), resolving all ZScript CPU overhead.

## 7. Mod-Agnostic / Total Conversion Compatibility Framework
* **Concept**: Enable Holographic GPS to function out-of-the-box with total conversions (TCs) like *WolfenDoom*, *Blade of Agony*, or *Ashes 2063*, which bypass standard Doom base classes (`Health`, `BasicArmorPickup`, `Ammo`, `Key`) and implement custom items, weapons, ammo systems, and HUD variables.
* **Technical Implementation**:
  - **Custom Definition Lump (`HOLOGPS` config)**:
    - Create a text-based definition file format (e.g., parsing a custom lump named `HOLOGPS` loaded from pk3 mods).
    - Allow mods to explicitly define mappings of actor class names to scavenger groups (e.g., `DefineAmmo "BoAPistolAmmo", "PistolAmmo", 100`, `DefineHealth "RationTin", 25, 100`, `DefineKey "SecretPlansLump"`).
  - **Flag & Metadata Reflection Heuristics**:
    - Instead of strict `is` class checks, inspect actor properties and class metadata dynamically.
    - Check actor flags using GZDoom's reflection: `inv.bSPECIAL`, `inv.GetTag()`, or `inv.FindInventory()` chains.
    - Check standard engine traits: `inv.bISHEALTH`, `inv.bISARMOR`.
    - Check the inheritance tree dynamically (e.g. searching for subclasses of `Inventory` that have "Ammo", "Armor", "Health", "Key", "Card", or "Skull" in their class names).
  - **Dynamic Engine Lock Introspection**:
    - Read the engine's active lock definitions (`LockDefs`) to see which actor class names are registered as locking items.
    - When searching for key goals, look for any ground items that match the lock key classes currently required by locked doors in the level.
  - **Fallback Status-Bar / HUD Capacity Queries**:
    - For unknown health/armor classes, use standard player queries like `plyr.mo.health < plyr.mo.GetMaxHealth()` or search for custom armor types like `HexenArmor` or `BasicArmor` dynamically rather than casting to fixed Doom classes.
    - Provide user CVars (`holo_gps_custom_health_cap`, `holo_gps_custom_armor_cap`) so users can manually customize thresholds for non-standard mods.
