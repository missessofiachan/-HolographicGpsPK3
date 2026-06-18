# Holographic GPS Navigation Mod (`.pk3`)

A lightweight, dynamic navigation assistant for GZDoom and compatible engines. This mod automatically tracks map objectives, sequences puzzle progression (such as routing to keys and remote switches before exits), and projects a customizable holographic pathway directly onto the map floor to guide you along an optimal line-of-sight route.

---

## Features

* **Objective Sequence Manager:** Dynamically scans map geometry upon level load. It prioritizes required progression keys and switches before redirecting your path to the level exit.
* **3D Floor & Bridge Awareness:** Fully supports multi-tiered geometry, routing pathways both over and under bridges and tunnels by propagating and validating 3D heights ($Z$) across sector transitions.
* **Dynamic Updating:** Path markers recalculate instantly when you pick up a progression key, toggle a switch, or step out of bounds.
* **Performance In-Engine Optimization:** Features time-sliced tick throttling and pre-built CSR adjacency graphs to ensure smooth frame rates, even on massive community slaughter maps.
* **Fully Configurable Interface:** Includes a native GZDoom options menu to adjust spacing, update intervals, opacity, and toggle states on the fly.

---

## File Structure

For development or compiling your own build, ensure your directory is laid out exactly as follows before archiving:

```text
HoloGPSMod/
├── ZScript/
│   ├── HoloMarker.zs       # Defines the visual rendering of the hologram
│   ├── PathfinderTracer.zs # Performs 3D raycast line-of-sight checks
│   └── HoloGPSHandler.zs   # Houses A* pathfinding, detour, and objective logic
├── sprites/
│   └── AMRKA0.png          # Custom transparent sprite for the path markers
├── cvarinfo.txt            # Defines global user configuration variables
├── menudef.txt             # Implements the visual Options Menu engine hook
└── zscript.txt             # Master engine assembly bootstrapper
```

---

## Installation & Usage

### For Players

1. Download the compiled `HoloGPS.pk3` file.
2. Drag and drop the `.pk3` directly onto your GZDoom/UZDoom executable, or load it via your preferred launcher (e.g., ZDL, Doom Runner).
3. Open the game, press `ESC` -> Go to **Options** -> Click **Holographic GPS Options** to customize your pathing behavior.

### For Developers (Compiling from Source)

1. Ensure your assets and scripts match the file structure above.
2. Compress the root contents of your development folder into a standard `.zip` archive.
3. Rename the file extension from `.zip` to `.pk3`.

---

## Configuration Settings

Accessible via the native **Options Menu**:

| Setting | Type | Range / Options | Description |
| --- | --- | --- | --- |
| **Enable Navigation Path** | Toggle | On / Off | Instantly toggles the tracking visualizer system. |
| **Marker Spacing** | Slider | 32 to 256 units | Controls how close individual arrows spawn to one another. |
| **Update Interval** | Slider | 1 to 35 ticks | Controls how often the logic processes. Higher = better performance. |
| **Hologram Opacity** | Slider | 0.1 to 1.0 alpha | Controls the intensity and transparency of the glow effect. |

---

## Technical & Compatibility Notes

* **Engine Requirements:** Target environment is GZDoom/UZDoom v4.10 or higher.
* **Render Pipeline:** This mod utilizes `Style_Add` blending for a pristine translucent neon glow. Hardware acceleration (OpenGL/Vulkan) is highly recommended for best results.
* **Compatibility:** Designed natively using global `StaticEventHandler` monitoring. It does not alter actor inventory structures or player states, ensuring 100% compatibility with massive gameplay overhauls like *Brutal Doom* or *Project Brutality*.
