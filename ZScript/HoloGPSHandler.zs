
// Represents the learned navigation data for a specific map to persist it across level changes.
class HoloMapKnowledge : Object {
  string mapHash;
  Array<bool> adjProven;
  Array<double> sectorPheromone;
}

// HoloGPSHandler is a StaticEventHandler that coordinates the pathfinding and
// marker projection logic. Design & Implementation Details:
// - Adjacency Graph: Pre-built once per map load (`BuildAdjacencyGraph`) using
// a CSR (Compressed Sparse Row)
//   representation (`adjStart`, `adjNeighbor`, `adjLineIdx`) to avoid nested
//   allocations and maximize cache efficiency.
// - Memory Management: Helper arrays (`parent`, `reachable`, `gScore`,
// `fScore`, `openSet`, `heapPos`, `sectorZ`, etc.)
//   are class members resized once, avoiding frame-by-frame
//   allocation/deallocation to bypass GC thrashing.
// - Performance Optimization: Config CVars are cached (`cache_enabled`,
// `cache_freq`, etc.) because GZDoom CVar lookups
//   incur string hash stutters if evaluated per-frame. Logic executes on a
//   throttled tick interval (`cache_freq`).
// - Multi-Stage Pathfinding (Synchronous):
//   1. A* Direct: Attempts to find the shortest path from player to target
//   (keys first, then exits).
//   2. Pedestal Retry: Fallback search if a target is raised on a
//   pedestal/pillar (finds adjacent walkable sectors).
//   3. Reverse BFS: If direct A* fails, runs a reverse topological BFS from the
//   target to find the first impassable
//      barrier/door. Then scans map lines for switches that open this barrier.
//   4. Switch Routing: Routes the player to the closest reachable switch that
//   unlocks the barrier first, solving the map puzzle.
class HoloGPSHandler : StaticEventHandler {
  // Navigation geometry thresholds
        const TRACE_Z_OFFSET = 16.0;  // Height above floor for line-of-sight traces
  const EXIT_NORMAL_PUSH =
      32.0; // Distance to push exit targets away from their line
  const DETOUR_WALL_MARGIN =
      32.0; // Longitudinal margin when detouring around walls
  const DETOUR_PERP_MARGIN =
      24.0; // Perpendicular margin when detouring around walls
  const SCORE_INFINITY = 1e37; // Sentinel value for unreached A* nodes

  // Stencil color lookup: 0=unused, 1=Red, 2=Green, 3=Blue, 4=Yellow, 5=Orange,
  // 6=Purple, 7=Pink, 8=White
  static const color COLOR_TABLE[] = {0x000000, 0xFF0000, 0x00FF00,
                                      0x0000FF, 0xFFFF00, 0xFF8000,
                                      0x8000FF, 0xFF00FF, 0xFFFFFF};

  // Trans pride flag cycle: blue, pink, white, pink
  static const color TRANS_COLORS[] = {0x5BCEFA, 0xF5A9B8, 0xFFFFFF, 0xF5A9B8};

  Actor currentTarget;
  int tickCounter;
  bool pathFresh; // True when FindNextObjective just found a target (path
                  // already computed)
  Array<Actor> activeMarkers;

  // Consolidated reusable tracer instance to avoid garbage collector load
  PathfinderTracer mTracer;

  // Cached CVar references to completely bypass tick-based string hash lookups
  CVar cv_enabled;
  CVar cv_freq;
  CVar cv_use_secrets;
  CVar cv_alpha;
  CVar cv_spacing;
  CVar cv_max_markers;
  CVar cv_fade;
  CVar cv_color;
  CVar cv_scale;
  CVar cv_height;
  CVar cv_priority;
  CVar cv_style;
  CVar cv_extended_search;
  CVar cv_wolfendoom_compat;
  CVar cv_hazard_avoidance;
  CVar cv_target_snapping;
  CVar cv_raycast_detouring;
  CVar cv_learning_proven;
  CVar cv_learning_pheromone;
  CVar cv_learning_persistent;
  CVar cv_learning_heuristics;
  CVar cv_learning_limit;
  CVar cv_3dfloors;
  CVar cv_portals;
  CVar cv_solve_switches;
  CVar cv_smoothing;

  // Speedrun Heuristics
  CVar cv_sr_momentum;
  CVar cv_sr_door_penalty;
  CVar cv_sr_monster_avoidance;
  CVar cv_sr_clearance;
  CVar cv_sr_reckless;

  // Engine Tuning Constants
  CVar cv_clearance_min;
  CVar cv_step_max;
  CVar cv_weight_doors;
  CVar cv_weight_monsters;
  CVar cv_weight_momentum;

  // Dynamically cached CVars to bypass continuous string lookups
  bool cache_enabled;
  int cache_freq;
  bool cache_use_secrets;
  float cache_alpha;
  int cache_spacing;
  int cache_max_markers;
  bool cache_fade;
  int cache_color;
  float cache_scale;
  float cache_height;
  int cache_priority;
  int cache_style;
  bool cache_extended_search;
  bool cache_wolfendoom_compat;
  bool cache_hazard_avoidance;
  bool cache_target_snapping;
  bool cache_raycast_detouring;
  bool cache_learning_proven;
  bool cache_learning_pheromone;
  bool cache_learning_persistent;
  bool cache_learning_heuristics;
  int cache_learning_limit;
  bool cache_3dfloors;
  bool cache_portals;
  bool cache_solve_switches;
  bool cache_smoothing;

  bool cache_sr_momentum;
  bool cache_sr_door_penalty;
  bool cache_sr_monster_avoidance;
  bool cache_sr_clearance;
  bool cache_sr_reckless;

  float cache_clearance_min;
  float cache_step_max;
  float cache_weight_doors;
  float cache_weight_monsters;
  float cache_weight_momentum;

  bool learningDirty;
  int lastPlayerSecIdx;
  Array<bool> adjProven;
  Array<double> sectorPheromone;
  Array<HoloMapKnowledge> sessionKnowledge;
  
  string GetMapHash() {
    return level.mapname.MakeLower() .. "_" .. level.sectors.Size() .. "_" .. level.lines.Size();
  }

  string lastMapName;

  // Global Heuristic Learning Data
  Array<string> knownTextures;
  Array<double> textureScores;
  double avgBrightness;
  int brightnessSamples;

  Array<int> monsterDensity;
  int monsterScanTick;

  int FindTextureIndex(string texName) {
    for (int i = 0; i < knownTextures.Size(); i++) {
      if (knownTextures[i] == texName) {
        return i;
      }
    }
    return knownTextures.Size();
  }

  // Persistent helper arrays to avoid garbage collector thrashing
  Array<int> parent;
  Array<int> parentLine;
  Array<int> reachable;
  Array<double> gScore;
  Array<double> fScore;
  Array<int> openSet;
  Array<bool> inOpenSet;
  Array<int>
      heapPos; // Maps sector index → position in openSet heap (-1 if absent)
  Array<double>
      sectorZ; // Cached Z height for each sector reached in A* search; used to
               // propagate vertical level across 3D floors/bridges

  // Persistent helper arrays for reverse topological fallback search
  Array<int> visitedReverse;
  Array<int> queueReverse;

  // Pathfinding result: ordered waypoints from player to target
  Array<double> pathX;
  Array<double> pathY;
  Array<double> refinedX;
  Array<double> refinedY;

  // Pre-built adjacency graph (built once per map load)
  Array<int> adjStart;
  Array<int> adjNeighbor;
  Array<int> adjLineIdx;
  int graphBuilt;

  // Sprite cache for WolfenDoom key compatibility checks
  SpriteID ykeySprite, bkeySprite, rkeySprite, gkeySprite, skeySprite;
  SpriteID goldKeySprite, silvKeySprite;
  bool spritesCached;

  
  string SerializeLearningData() {
    string outBlob = "G:"..avgBrightness..":"..brightnessSamples.."|";
    string texStr = "";
    for(int i = 0; i < knownTextures.Size(); i++) {
       texStr = texStr .. knownTextures[i] .. ":" .. textureScores[i];
       if (i < knownTextures.Size() - 1) texStr = texStr .. ",";
    }
    outBlob = outBlob .. texStr .. "|";
    
    PlayerInfo plyr = players[consoleplayer];
    int limitVal = 10;
    if (plyr) {
      CVar limitCV = CVar.GetCVar("holo_gps_learning_limit", plyr);
      if (limitCV) limitVal = limitCV.GetInt();
    }
    
    int mapCount = 0;
    for(int i = sessionKnowledge.Size() - 1; i >= 0 && mapCount < limitVal; i--) {
       HoloMapKnowledge map = sessionKnowledge[i];
       if (!map) continue;
       outBlob = outBlob .. "M:" .. map.mapHash .. ":";
       bool first = true;
       for(int k = 0; k < map.adjProven.Size(); k++) {
         if (map.adjProven[k]) {
           if (!first) outBlob = outBlob .. ",";
           outBlob = outBlob .. k;
           first = false;
         }
       }
       outBlob = outBlob .. "|";
       mapCount++;
    }
    return outBlob;
  }

  void DeserializeLearningData(string blob) {
    if (blob == "") return;
    Array<string> parts;
    blob.Split(parts, "|");
    if (parts.Size() >= 2) {
      Array<string> gStats;
      parts[0].Split(gStats, ":");
      if (gStats.Size() >= 3 && gStats[0] == "G") {
        avgBrightness = gStats[1].ToDouble();
        brightnessSamples = gStats[2].ToInt();
      }
      
      knownTextures.Clear();
      textureScores.Clear();
      if (parts[1] != "") {
        Array<string> texPairs;
        parts[1].Split(texPairs, ",");
        for(int i = 0; i < texPairs.Size(); i++) {
          Array<string> kv;
          texPairs[i].Split(kv, ":");
          if (kv.Size() == 2) {
            knownTextures.Push(kv[0]);
            textureScores.Push(kv[1].ToDouble());
          }
        }
      }
      
      for(int p = 2; p < parts.Size(); p++) {
         if (parts[p] == "") continue;
         Array<string> mHead;
         parts[p].Split(mHead, ":");
         if (mHead.Size() >= 2 && mHead[0] == "M") {
            string mHash = mHead[1];
            HoloMapKnowledge existing = null;
            for(int i = 0; i < sessionKnowledge.Size(); i++) {
               if (sessionKnowledge[i].mapHash == mHash) {
                  existing = sessionKnowledge[i];
                  break;
               }
            }
            if (!existing) {
               existing = new("HoloMapKnowledge");
               existing.mapHash = mHash;
               sessionKnowledge.Push(existing);
            }
            
            if (mHead.Size() >= 3 && mHead[2] != "") {
               Array<string> indices;
               mHead[2].Split(indices, ",");
               int maxIdx = -1;
               for(int i = 0; i < indices.Size(); i++) {
                 int idx = indices[i].ToInt();
                 if (idx > maxIdx) maxIdx = idx;
               }
               if (maxIdx >= existing.adjProven.Size()) {
                 existing.adjProven.Resize(maxIdx + 1);
               }
               for(int i = 0; i < indices.Size(); i++) {
                 int idx = indices[i].ToInt();
                 if (idx >= 0 && idx < existing.adjProven.Size()) {
                   existing.adjProven[idx] = true;
                 }
               }
            }
         }
      }
    }
  }

  override void WorldLoaded(WorldEvent e) {
    PlayerInfo plyr = players[consoleplayer];
    bool usePersistent = false;
    if (plyr) {
      CVar cv = CVar.GetCVar("holo_gps_learning_persistent", plyr);
      if (cv) usePersistent = cv.GetBool();
    }

    if (usePersistent && lastMapName != "") {
      HoloMapKnowledge saved = null;
      int existingIdx = -1;
      for (int i = 0; i < sessionKnowledge.Size(); i++) {
        if (sessionKnowledge[i].mapHash == lastMapName) {
          saved = sessionKnowledge[i];
          existingIdx = i;
          break;
        }
      }
      if (!saved) {
        saved = new("HoloMapKnowledge");
        saved.mapHash = lastMapName;
        sessionKnowledge.Push(saved);
      } else if (existingIdx >= 0) {
        sessionKnowledge.Delete(existingIdx);
        sessionKnowledge.Push(saved);
      }
      saved.adjProven.Copy(adjProven);
      saved.sectorPheromone.Copy(sectorPheromone);

      int limitVal = 10;
      if (plyr) {
        CVar limitCV = CVar.GetCVar("holo_gps_learning_limit", plyr);
        if (limitCV) limitVal = limitCV.GetInt();
      }
      if (limitVal < 1) limitVal = 1;
      while (sessionKnowledge.Size() > limitVal) {
        sessionKnowledge.Delete(0);
      }
    }

    currentTarget = null;
    tickCounter = 0;
    activeMarkers.Clear();
    pathX.Clear();
    pathY.Clear();
    refinedX.Clear();
    refinedY.Clear();
    graphBuilt = 0;

    BuildAdjacencyGraph();

    string currentMap = GetMapHash();
    lastMapName = currentMap;

    if (plyr) {
      CVar cvBlob = CVar.GetCVar("holo_gps_memory_blob", plyr);
      if (cvBlob) {
        Console.Printf("HoloGPS Debug: Loaded CVAR string length is %d", cvBlob.GetString().Length());
        DeserializeLearningData(cvBlob.GetString());
      }
    }

    int numSectors = level.sectors.Size();
    sectorPheromone.Clear();
    sectorPheromone.Resize(numSectors);
    for (int i = 0; i < numSectors; i++)
      sectorPheromone[i] = 0.0;
    lastPlayerSecIdx = -1;

    monsterScanTick = 0;
    monsterDensity.Clear();

    if (usePersistent) {
      HoloMapKnowledge loaded = null;
      int loadedIdx = -1;
      for (int i = 0; i < sessionKnowledge.Size(); i++) {
        if (sessionKnowledge[i].mapHash == currentMap) {
          loaded = sessionKnowledge[i];
          loadedIdx = i;
          break;
        }
      }
      if (loaded) {
        sessionKnowledge.Delete(loadedIdx);
        sessionKnowledge.Push(loaded);

        if (loaded.sectorPheromone.Size() == sectorPheromone.Size()) {
          sectorPheromone.Copy(loaded.sectorPheromone);
        }
        if (loaded.adjProven.Size() == adjProven.Size()) {
          adjProven.Copy(loaded.adjProven);
        }
      }
    } 

    mTracer = new("PathfinderTracer");

    ykeySprite = Actor.GetSpriteIndex("YKEY");
    bkeySprite = Actor.GetSpriteIndex("BKEY");
    rkeySprite = Actor.GetSpriteIndex("RKEY");
    gkeySprite = Actor.GetSpriteIndex("GKEY");
    skeySprite = Actor.GetSpriteIndex("SKEY");
    goldKeySprite = Actor.GetSpriteIndex("GOLD");
    silvKeySprite = Actor.GetSpriteIndex("SILV");
    spritesCached = true;

    FindNextObjective();
  }

  // Builds a static topological graph of the level's sectors once on map load.
  // Why: Parsing mappers' level geometry in real-time is too slow for 35 FPS
  // ticks. How (CSR - Compressed Sparse Row):
  // - Pass 1: Counts the number of passable neighbors (linedef connections) for
  // each sector.
  // - Creates prefix-sum index pointers (`adjStart`).
  // - Pass 2: Fills the contiguous flat arrays (`adjNeighbor`, `adjLineIdx`)
  // with neighbor sector indices. This avoids multi-dimensional dynamic arrays,
  // preventing memory fragmentation and VM garbage collector overhead.
  void BuildAdjacencyGraph() {
    int numSectors = level.sectors.Size();
    int numLines = level.lines.Size();

    Array<int> neighborCount;
    neighborCount.Resize(numSectors);
    for (int i = 0; i < numSectors; i++)
      neighborCount[i] = 0;

    for (int li = 0; li < numLines; li++) {
      Line ln = level.lines[li];
      if (!ln)
        continue;

      if (ln.isLinePortal()) {
        Line dest = ln.getPortalDestination();
        if (dest && ln.frontsector && dest.frontsector) {
          int fi = ln.frontsector.sectornum;
          int bi = dest.frontsector.sectornum;
          if (fi != bi) {
            neighborCount[fi]++;
            neighborCount[bi]++;
          }
        }
      } else {
        if (!ln.frontsector || !ln.backsector)
          continue;
        if (ln.flags & Line.ML_BLOCKING)
          continue;

        int fi = ln.frontsector.sectornum;
        int bi = ln.backsector.sectornum;
        if (fi == bi)
          continue;

        neighborCount[fi]++;
        neighborCount[bi]++;
      }
    }

    adjStart.Clear();
    adjStart.Resize(numSectors + 1);
    adjStart[0] = 0;
    for (int i = 0; i < numSectors; i++) {
      adjStart[i + 1] = adjStart[i] + neighborCount[i];
    }

    int totalAdj = adjStart[numSectors];
    adjNeighbor.Clear();
    adjNeighbor.Resize(totalAdj);
    adjLineIdx.Clear();
    adjLineIdx.Resize(totalAdj);
    adjProven.Clear();
    adjProven.Resize(totalAdj);
    for (int i = 0; i < totalAdj; i++)
      adjProven[i] = false;

    Array<int> fillPos;
    fillPos.Resize(numSectors);
    for (int i = 0; i < numSectors; i++)
      fillPos[i] = adjStart[i];

    for (int li = 0; li < numLines; li++) {
      Line ln = level.lines[li];
      if (!ln)
        continue;

      if (ln.isLinePortal()) {
        Line dest = ln.getPortalDestination();
        if (dest && ln.frontsector && dest.frontsector) {
          int fi = ln.frontsector.sectornum;
          int bi = dest.frontsector.sectornum;
          if (fi != bi) {
            adjNeighbor[fillPos[fi]] = bi;
            adjLineIdx[fillPos[fi]] = li;
            fillPos[fi]++;

            adjNeighbor[fillPos[bi]] = fi;
            adjLineIdx[fillPos[bi]] = li;
            fillPos[bi]++;
          }
        }
      } else {
        if (!ln.frontsector || !ln.backsector)
          continue;
        if (ln.flags & Line.ML_BLOCKING)
          continue;

        int fi = ln.frontsector.sectornum;
        int bi = ln.backsector.sectornum;
        if (fi == bi)
          continue;

        adjNeighbor[fillPos[fi]] = bi;
        adjLineIdx[fillPos[fi]] = li;
        fillPos[fi]++;

        adjNeighbor[fillPos[bi]] = fi;
        adjLineIdx[fillPos[bi]] = li;
        fillPos[bi]++;
      }
    }

    graphBuilt = 1;
  }

  void InitCVars(PlayerInfo plyr) {
    if (cv_enabled)
      return;
    cv_enabled = CVar.GetCVar("holo_gps_enabled", plyr);
    cv_freq = CVar.GetCVar("holo_gps_freq", plyr);
    cv_use_secrets = CVar.GetCVar("holo_gps_use_secrets", plyr);
    cv_alpha = CVar.GetCVar("holo_gps_alpha", plyr);
    cv_spacing = CVar.GetCVar("holo_gps_spacing", plyr);
    cv_max_markers = CVar.GetCVar("holo_gps_max_markers", plyr);
    cv_fade = CVar.GetCVar("holo_gps_fade", plyr);
    cv_color = CVar.GetCVar("holo_gps_color", plyr);
    cv_scale = CVar.GetCVar("holo_gps_scale", plyr);
    cv_height = CVar.GetCVar("holo_gps_height", plyr);
    cv_priority = CVar.GetCVar("holo_gps_priority", plyr);
    cv_style = CVar.GetCVar("holo_gps_style", plyr);
    cv_extended_search = CVar.GetCVar("holo_gps_extended_search", plyr);
    cv_wolfendoom_compat = CVar.GetCVar("holo_gps_wolfendoom_compat", plyr);
    cv_hazard_avoidance = CVar.GetCVar("holo_gps_hazard_avoidance", plyr);
    cv_target_snapping = CVar.GetCVar("holo_gps_target_snapping", plyr);
    cv_raycast_detouring = CVar.GetCVar("holo_gps_raycast_detouring", plyr);
    cv_learning_proven = CVar.GetCVar("holo_gps_learning_proven", plyr);
    cv_learning_pheromone = CVar.GetCVar("holo_gps_learning_pheromone", plyr);
    cv_learning_persistent = CVar.GetCVar("holo_gps_learning_persistent", plyr);
    cv_learning_heuristics = CVar.GetCVar("holo_gps_learning_heuristics", plyr);
    cv_3dfloors = CVar.GetCVar("holo_gps_3dfloors", plyr);
    cv_portals = CVar.GetCVar("holo_gps_portals", plyr);
    cv_solve_switches = CVar.GetCVar("holo_gps_solve_switches", plyr);
    cv_smoothing = CVar.GetCVar("holo_gps_smoothing", plyr);

    cv_sr_momentum = CVar.GetCVar("holo_gps_sr_momentum", plyr);
    cv_sr_door_penalty = CVar.GetCVar("holo_gps_sr_door_penalty", plyr);
    cv_sr_monster_avoidance = CVar.GetCVar("holo_gps_sr_monster_avoidance", plyr);
    cv_sr_clearance = CVar.GetCVar("holo_gps_sr_clearance", plyr);
    cv_sr_reckless = CVar.GetCVar("holo_gps_sr_reckless", plyr);

    cv_clearance_min = CVar.GetCVar("holo_gps_clearance_min", plyr);
    cv_step_max = CVar.GetCVar("holo_gps_step_max", plyr);
    cv_weight_doors = CVar.GetCVar("holo_gps_weight_doors", plyr);
    cv_weight_monsters = CVar.GetCVar("holo_gps_weight_monsters", plyr);
    cv_weight_momentum = CVar.GetCVar("holo_gps_weight_momentum", plyr);
  }

  override void WorldTick() {
    PlayerInfo plyr = players[consoleplayer];
    if (!plyr || !plyr.mo || plyr.mo.health <= 0)
      return;

    InitCVars(plyr);
    cache_learning_proven = cv_learning_proven.GetBool();
    cache_learning_pheromone = cv_learning_pheromone.GetBool();
    cache_learning_persistent = cv_learning_persistent.GetBool();
    cache_learning_heuristics = cv_learning_heuristics.GetBool();

    cache_sr_momentum = cv_sr_momentum.GetBool();
    cache_sr_door_penalty = cv_sr_door_penalty.GetBool();
    cache_sr_monster_avoidance = cv_sr_monster_avoidance.GetBool();
    cache_sr_clearance = cv_sr_clearance.GetBool();
    cache_sr_reckless = cv_sr_reckless.GetBool();

    cache_clearance_min = cv_clearance_min.GetFloat();
    cache_step_max = cv_step_max.GetFloat();
    cache_weight_doors = cv_weight_doors.GetFloat();
    cache_weight_monsters = cv_weight_monsters.GetFloat();
    cache_weight_momentum = cv_weight_momentum.GetFloat();

    Sector playerSec = level.PointInSector(plyr.mo.pos.xy);
    if (playerSec && graphBuilt == 1) {
      int playerSecIdx = playerSec.sectornum;
      if (lastPlayerSecIdx != -1 && lastPlayerSecIdx != playerSecIdx) {
        if (cache_learning_proven) {
          int nStart = adjStart[lastPlayerSecIdx];
          int nEnd = adjStart[lastPlayerSecIdx + 1];
          for (int ni = nStart; ni < nEnd; ni++) {
            if (adjNeighbor[ni] == playerSecIdx) {
              adjProven[ni] = true;
          learningDirty = true;

              int rnStart = adjStart[playerSecIdx];
              int rnEnd = adjStart[playerSecIdx + 1];
              for (int rni = rnStart; rni < rnEnd; rni++) {
                if (adjNeighbor[rni] == lastPlayerSecIdx) {
                  adjProven[rni] = true;
                  break;
                }
              }
              break;
            }
          }
        }

        if (cache_learning_heuristics) {
          TextureID floorTexID = playerSec.GetTexture(Sector.floor);
          string texName = TexMan.GetName(floorTexID);
          texName = texName.MakeLower();
          
          int tIdx = FindTextureIndex(texName);
          if (tIdx == knownTextures.Size()) {
            knownTextures.Push(texName);
            textureScores.Push(0.01);
            learningDirty = true;
          } else {
            textureScores[tIdx] = textureScores[tIdx] + 0.01;
            if (textureScores[tIdx] > 1.0) textureScores[tIdx] = 1.0;
            learningDirty = true;
          }

          int light = playerSec.lightlevel;
          if (brightnessSamples == 0) {
            avgBrightness = light;
            brightnessSamples = 1;
            learningDirty = true;
          } else {
            avgBrightness = (avgBrightness * brightnessSamples + light) / (brightnessSamples + 1);
            brightnessSamples++;
            learningDirty = true;
          }
        }
      }
      lastPlayerSecIdx = playerSecIdx;

      if (cache_learning_pheromone && sectorPheromone.Size() > playerSecIdx) {
        double newVal = sectorPheromone[playerSecIdx] + 0.01;
        if (newVal > 1.0)
          newVal = 1.0;
        sectorPheromone[playerSecIdx] = newVal;
      }
    }

    cache_enabled = cv_enabled.GetBool();
    if (!cache_enabled)
      return;

    if (cache_sr_monster_avoidance && graphBuilt == 1) {
      monsterScanTick++;
      if (monsterScanTick >= 35) {
        monsterScanTick = 0;
        int numSectors = level.sectors.Size();
        monsterDensity.Resize(numSectors);
        for (int i = 0; i < numSectors; i++) monsterDensity[i] = 0;
        
        ThinkerIterator it = ThinkerIterator.Create("Actor");
        Actor mo;
        while (mo = Actor(it.Next())) {
          if (mo.bISMONSTER && mo.health > 0) {
            Sector sec = level.PointInSector(mo.pos.xy);
            if (sec) monsterDensity[sec.sectornum]++;
          }
        }
      }
    }

    cache_freq = cv_freq.GetInt();
    if (cache_freq < 1)
      cache_freq = 1;
    tickCounter++;
    if (tickCounter % cache_freq != 0)
      return;

    // Read remaining CVars only on ticks that do actual work
    cache_use_secrets = cv_use_secrets.GetBool();
    cache_alpha = cv_alpha.GetFloat();
    cache_spacing = cv_spacing.GetInt();
    cache_max_markers = cv_max_markers.GetInt();
    cache_fade = cv_fade.GetBool();
    cache_color = cv_color.GetInt();
    cache_scale = cv_scale.GetFloat();
    cache_height = cv_height.GetFloat();
    cache_priority = cv_priority.GetInt();
    cache_style = cv_style.GetInt();
    cache_extended_search = cv_extended_search.GetBool();
    cache_wolfendoom_compat = cv_wolfendoom_compat.GetBool();
    cache_hazard_avoidance = cv_hazard_avoidance.GetBool();
    cache_target_snapping = cv_target_snapping.GetBool();
    cache_raycast_detouring = cv_raycast_detouring.GetBool();
    cache_3dfloors = cv_3dfloors.GetBool();
    cache_portals = cv_portals.GetBool();
    cache_solve_switches = cv_solve_switches.GetBool();
    cache_smoothing = cv_smoothing.GetBool();

    if (currentTarget) {
      if (currentTarget is "Inventory") {
        Inventory inv = Inventory(currentTarget);
        if (inv.owner || plyr.mo.FindInventory(inv.GetClass())) {
          currentTarget = null;
        }
      }
    }

    if (!currentTarget || currentTarget.bDestroyed) {
      currentTarget = null;
      pathFresh = false;
      FindNextObjective();
    }

    if (currentTarget) {
      ClearOldMarkers();
      if (!pathFresh) {
        FindPathAStar(plyr.mo, currentTarget);
      }
      pathFresh = false;
      if (cache_smoothing) {
        RefinePath(plyr.mo.pos.xy, plyr.mo.pos.z);
      } else {
        refinedX.Copy(pathX);
        refinedY.Copy(pathY);
      }
      SpawnPathMarkers(plyr.mo);
    }

    if (learningDirty && plyr) {
      learningDirty = false;
      string curHash = GetMapHash();
      HoloMapKnowledge curMap = null;
      for(int i = 0; i < sessionKnowledge.Size(); i++) {
         if (sessionKnowledge[i].mapHash == curHash) {
            curMap = sessionKnowledge[i];
            break;
         }
      }
      if (!curMap) {
         curMap = new("HoloMapKnowledge");
         curMap.mapHash = curHash;
         sessionKnowledge.Push(curMap);
      }
      curMap.adjProven.Copy(adjProven);
      CVar cvBlob = CVar.GetCVar("holo_gps_memory_blob", plyr);
      if (cvBlob) cvBlob.SetString(SerializeLearningData());
    }
  }

  // Actor Pooling System:
  // Spawning and destroying actors in GZDoom triggers expensive memory
  // allocation and sector relinking. To prevent garbage collection stutters (GC
  // thrashing) and micro-lags, we keep visual marker actors alive in a pool.
  // When clearing markers, we simply hide them (STYLE_None) rather than calling
  // Destroy(), ready to be reused.
  void ClearOldMarkers() {
    for (int i = 0; i < activeMarkers.Size(); i++) {
      if (activeMarkers[i] && !activeMarkers[i].bDestroyed) {
        activeMarkers[i].A_SetRenderStyle(0.0, STYLE_None);
      }
    }
  }

  int GetEstimatedDatabaseSize() {
    int totalBytes = 0;
    for (int i = 0; i < sessionKnowledge.Size(); i++) {
      HoloMapKnowledge knowledge = sessionKnowledge[i];
      if (!knowledge) continue;

      totalBytes += knowledge.mapHash.Length();
      totalBytes += knowledge.adjProven.Size() * 1; // 1 byte per bool
      totalBytes += knowledge.sectorPheromone.Size() * 8; // 8 bytes per double
      totalBytes += 64; // Object overhead estimation
    }
    return totalBytes;
  }

  override void NetworkProcess(ConsoleEvent e) {
    if (e.Name == "purge_gps_memory") {
      int estSize = GetEstimatedDatabaseSize();
      double estKB = estSize / 1024.0;

      sessionKnowledge.Clear();
      lastPlayerSecIdx = -1;
      knownTextures.Clear();
      textureScores.Clear();
      avgBrightness = 0;
      brightnessSamples = 0;

      int numSectors = level.sectors.Size();
      for (int i = 0; i < numSectors; i++) {
        sectorPheromone[i] = 0.0;
      }
      int totalAdj = adjProven.Size();
      for (int i = 0; i < totalAdj; i++) {
        adjProven[i] = false;
      }
      PlayerInfo plyrInfo = players[consoleplayer];
      if (plyrInfo) {
        CVar cvBlob = CVar.GetCVar("holo_gps_memory_blob", plyrInfo);
        if (cvBlob) cvBlob.SetString("");
      }

      Console.Printf("Holographic GPS: Memory database purged. Freed approx. %.2f KB.", estKB);
    } else if (e.Name == "check_gps_memory") {
      int estSize = GetEstimatedDatabaseSize();
      double estKB = estSize / 1024.0;
      Console.Printf("Holographic GPS: %d maps stored in memory database. Approx. %.2f KB saved.", sessionKnowledge.Size(), estKB);
      PlayerInfo plyrInfo = players[consoleplayer];
      if (plyrInfo) {
        CVar cvBlob = CVar.GetCVar("holo_gps_memory_blob", plyrInfo);
        if (cvBlob) {
           Console.Printf("HoloGPS Debug: Current live CVAR length is %d", cvBlob.GetString().Length());
        }
      }
    }
  }

  // Scans the map to identify the next progression objective (keys first, then
  // exits). Why: To automate the progression sequence of picking up keys before
  // routing to the exit. How:
  // - Pass 1: Scans the thinker directory for key items. Uses name/sprite
  // matching to support custom maps/mods.
  // - Filters out keys already present in the player's inventory.
  // - Marks all uncollected key sectors as goals and runs a Multi-Goal Dijkstra
  // search. The search terminates
  //   upon reaching the closest key, resolving the nearest reachable key in
  //   $O(N \log N)$ time.
  // - Pass 2: If no keys remain or keys are disabled, scans linedefs for exit
  // specials (e.g. Exit_Normal, Exit_Secret).
  //   Spawns temporary MapSpots, offsets their coordinates to front sector
  //   space (preventing crevice stuck checks), and routes the player to the
  //   nearest exit.
  void FindNextObjective() {
    PlayerInfo plyr = players[consoleplayer];
    if (!plyr || !plyr.mo)
      return;

    // Pass 1: Find valid, reachable Keys using single multi-target search
    if (cache_priority == 0 || cache_priority == 1) {
      // Collect all uncollected key candidates and their sectors
      Array<Actor> keyCandidates;
      Array<int> keySectors;

      ThinkerIterator it = ThinkerIterator.Create(
          (cache_extended_search || cache_wolfendoom_compat) ? "Inventory"
                                                             : "Key");
      Inventory mapKey;
      while (mapKey = Inventory(it.Next())) {
        if (mapKey.owner)
          continue;

        bool isKey = false;
        if (mapKey is "Key") {
          isKey = true;
        } else if (cache_extended_search && mapKey.bSPECIAL) {
          string cname = mapKey.GetClassName();
          cname = cname.MakeLower();

          bool hasKeyMatch = false;
          if (cname.IndexOf("key") == cname.Length() - 3)
            hasKeyMatch = true;
          else if (cname.IndexOf("key") == 0)
            hasKeyMatch = true;
          else if (cname.IndexOf("card") == cname.Length() - 4)
            hasKeyMatch = true;
          else if (cname.IndexOf("skull") == cname.Length() - 5)
            hasKeyMatch = true;

          if (hasKeyMatch) {
            isKey = true;
          }
        }

        if (!isKey && cache_wolfendoom_compat && mapKey.bSPECIAL) {
          if (mapKey.sprite == ykeySprite || mapKey.sprite == bkeySprite ||
              mapKey.sprite == rkeySprite || mapKey.sprite == gkeySprite ||
              mapKey.sprite == skeySprite || mapKey.sprite == goldKeySprite ||
              mapKey.sprite == silvKeySprite) {
            isKey = true;
          }
        }

        if (isKey && !plyr.mo.FindInventory(mapKey.GetClass())) {
          Sector keySec = level.PointInSector(mapKey.pos.xy);
          if (keySec) {
            int snappedIdx =
                GetValidSnappedSector(keySec.sectornum, mapKey, plyr.mo);
            keyCandidates.Push(mapKey);
            keySectors.Push(snappedIdx);
          }
        }
      }

      // Single multi-target search: find nearest reachable key
      if (keyCandidates.Size() > 0) {
        Sector startSec = level.PointInSector(plyr.mo.pos.xy);
        if (startSec) {
          int startIdx = startSec.sectornum;
          int numSectors = level.sectors.Size();
          parent.Resize(numSectors);
          parentLine.Resize(numSectors);
          reachable.Resize(numSectors);
          gScore.Resize(numSectors);
          fScore.Resize(numSectors);
          inOpenSet.Resize(numSectors);

          // Mark goal sectors
          Array<bool> isGoal;
          isGoal.Resize(numSectors);
          for (int i = 0; i < numSectors; i++)
            isGoal[i] = false;
          for (int i = 0; i < keySectors.Size(); i++)
            isGoal[keySectors[i]] = true;

          int hitGoal =
              RunPathfinder(startIdx, -1, isGoal, plyr.mo.pos.xy, plyr.mo);
          if (hitGoal >= 0) {
            // Find which key actor lives in the hit sector
            for (int i = 0; i < keyCandidates.Size(); i++) {
              if (keySectors[i] == hitGoal) {
                currentTarget = keyCandidates[i];
                // Build the path from RunAStar's parent data
                BuildPathFromParent(startIdx, hitGoal, keyCandidates[i]);
                pathFresh = true;
                return;
              }
            }
          }
        }
      }
    }

    // Pass 2 Fallback: Target level exits
    if (cache_priority == 0 || cache_priority == 2) {
      Array<Actor> exitSpots;
      Array<int> exitSectors;

      for (int i = 0; i < level.lines.Size(); i++) {
        Line ln = level.lines[i];
        if (ln && (ln.special == 243 || ln.special == 244 || ln.special == 74 ||
                   ln.special == 124)) {
          Vector2 midpoint = (ln.v1.p + ln.v2.p) / 2.0;

          // FIX: Push target 32 units away from the line into its front sector.
          // This prevents pathfinding into tight switch crevices and walls.
          if (ln.frontsector) {
            Vector2 lineVec = ln.v2.p - ln.v1.p;
            Vector2 normal =
                (lineVec.y, -lineVec.x)
                    .Unit(); // Right hand normal points to front sector
            midpoint = midpoint + normal * EXIT_NORMAL_PUSH;
          }

          Sector exitSec = level.PointInSector(midpoint);
          if (!exitSec)
            continue;

          double z = exitSec.floorplane.ZatPoint(midpoint);
          Actor spot = Actor.Spawn("MapSpot", (midpoint.x, midpoint.y, z));
          if (spot) {
            exitSpots.Push(spot);
            exitSectors.Push(exitSec.sectornum);
          }
        }
      }

      int foundIdx = -1;
      if (exitSpots.Size() > 0) {
        Sector startSec = level.PointInSector(plyr.mo.pos.xy);
        if (startSec) {
          int startIdx = startSec.sectornum;
          int numSectors = level.sectors.Size();
          parent.Resize(numSectors);
          parentLine.Resize(numSectors);
          reachable.Resize(numSectors);
          gScore.Resize(numSectors);
          fScore.Resize(numSectors);
          inOpenSet.Resize(numSectors);

          Array<bool> isGoal;
          isGoal.Resize(numSectors);
          for (int i = 0; i < numSectors; i++)
            isGoal[i] = false;
          for (int i = 0; i < exitSectors.Size(); i++)
            isGoal[exitSectors[i]] = true;

          int hitGoal =
              RunPathfinder(startIdx, -1, isGoal, plyr.mo.pos.xy, plyr.mo);
          if (hitGoal >= 0) {
            for (int i = 0; i < exitSpots.Size(); i++) {
              if (exitSectors[i] == hitGoal) {
                currentTarget = exitSpots[i];
                BuildPathFromParent(startIdx, hitGoal, exitSpots[i]);
                pathFresh = true;
                foundIdx = i;
                break;
              }
            }
          }
        }
      }

      // Clean up all scratch actors except the one we're keeping
      for (int i = 0; i < exitSpots.Size(); i++) {
        if (i != foundIdx && exitSpots[i]) {
          exitSpots[i].Destroy();
        }
      }
      if (foundIdx >= 0)
        return;
    }

    currentTarget = null;
  }

  // Evaluates if a portal (linedef connection) can be walked through by the
  // player, taking into account lock constraints, blocking flags, clearance,
  // and step heights. Propagates vertical level (refZ) to calculate and output
  // nextFloor for the neighboring sector.
  bool IsPortalPassable(Sector curSec, Sector nextSec, Line ln, Actor player,
                        double refZ, out double nextFloor, out double nextCeil, bool isProven = false) {
    nextFloor = nextSec.floorplane.ZatPoint(ln.v1.p); 
    nextCeil = nextFloor + 128.0;

    if (!ln || !curSec || !nextSec)
      return false;

    if (ln.flags & Line.ML_BLOCKING)
      return false;
    if (!cache_use_secrets && (ln.flags & Line.ML_SECRET))
      return false;

    if (ln.locknumber != 0) {
      PlayerPawn plyrPawn = PlayerPawn(player);
      if (plyrPawn && !plyrPawn.CheckKeys(ln.locknumber, false, true)) {
        return false;
      }
    }

    Vector2 midpoint = (ln.v1.p + ln.v2.p) / 2.0;
    Vector2 nextMidpoint = midpoint;
    if (ln.isLinePortal()) {
      nextMidpoint = midpoint + ln.getPortalDisplacement();
    }
    double curFloor, curCeil;
    // Resolve the effective floor/ceiling heights at the portal crossing point,
    // using the propagated reference Z of the current sector.
    GetEffectiveFloorCeil(curSec, midpoint, refZ, curFloor, curCeil);
    // Calculate the neighbor sector's effective floor/ceiling, referencing the
    // resolved current floor to ensure we align with the correct vertical layer
    // (e.g. crossing a bridge vs under it).
    GetEffectiveFloorCeil(nextSec, nextMidpoint, curFloor, nextFloor, nextCeil);

    if (abs(nextFloor - curFloor) > cache_step_max)
      return false;

    double portalFloor = (curFloor > nextFloor) ? curFloor : nextFloor;
    double portalCeil = (curCeil < nextCeil) ? curCeil : nextCeil;
    double clearance = portalCeil - portalFloor;

    // FIX: Lowered clearance check if WolfenDoom compat is enabled
    if (clearance < cache_clearance_min) {
      if (cache_wolfendoom_compat && clearance >= (cache_clearance_min - 16.0)) {
        // Pass for tight WolfenDoom elevator logic
      } else {
        bool isDirectUse = (ln.activation & (SPAC_Use | SPAC_UseThrough)) != 0;
        bool isStandardDoor = (ln.special >= 10 && ln.special <= 13) ||
                              ln.special == 105 || ln.special == 106 ||
                              ln.special == 202;
        bool isScriptDoor = (ln.special == 80 || ln.special == 226);

        if (!isDirectUse && !isStandardDoor && !isScriptDoor) {
          // If it's a remote door and it's currently closed, we cannot pass it
          // even if we proved it earlier when it was open!
          return false;
        }
      }
    }

    if (isProven) return true;

    if (abs(nextFloor - curFloor) > cache_step_max)
      return false;

    return true;
  }

  bool IsPlayerTriggerable(Line ln) {
    if (!ln || ln.special == 0)
      return false;
    int act = ln.activation;
    if (act & (SPAC_Use | SPAC_UseThrough | SPAC_Cross | SPAC_Impact |
               SPAC_Push | SPAC_AnyCross)) {
      return true;
    }
    return false;
  }

  // --- Binary Min-Heap Priority Queue implementation ---
  // Why: A* and Dijkstra searches require retrieving the node with the lowest
  // `fScore` at each iteration.
  //      A linear search on a large array takes $O(M)$ time, which becomes a
  //      bottleneck on complex maps. A binary heap allows extracting the
  //      minimum element in $O(\log M)$ time.
  // How:
  // - `openSet` stores the sector numbers matching the heap tree nodes.
  // - `heapPos` acts as an inverse lookup map (sector index -> heap index).
  // This enables checking
  //   membership in $O(1)$ and updating a sector's position via `HeapSiftUp` in
  //   $O(\log M)$ when its score decreases, effectively implementing a fast
  //   Dijkstra decrease-key operation.

  void HeapSwap(int a, int b) {
    int tmp = openSet[a];
    openSet[a] = openSet[b];
    openSet[b] = tmp;
    heapPos[openSet[a]] = a;
    heapPos[openSet[b]] = b;
  }

  void HeapSiftUp(int idx) {
    while (idx > 0) {
      int par = (idx - 1) / 2;
      if (fScore[openSet[idx]] < fScore[openSet[par]]) {
        HeapSwap(idx, par);
        idx = par;
      } else
        break;
    }
  }

  void HeapSiftDown(int idx, int size) {
    while (true) {
      int best = idx;
      int left = 2 * idx + 1;
      int right = 2 * idx + 2;
      if (left < size && fScore[openSet[left]] < fScore[openSet[best]])
        best = left;
      if (right < size && fScore[openSet[right]] < fScore[openSet[best]])
        best = right;
      if (best != idx) {
        HeapSwap(idx, best);
        idx = best;
      } else
        break;
    }
  }

  void HeapPush(int sector) {
    int idx = openSet.Size();
    openSet.Push(sector);
    heapPos[sector] = idx;
    HeapSiftUp(idx);
  }

  int HeapPop() {
    int top = openSet[0];
    int last = openSet.Size() - 1;
    if (last > 0) {
      HeapSwap(0, last);
    }
    heapPos[top] = -1;
    openSet.Delete(last);
    if (openSet.Size() > 0)
      HeapSiftDown(0, openSet.Size());
    return top;
  }

  // Calculates the cost penalty for traversing a damaging sector.
  double GetHazardPenalty(Sector sec, Actor player) {
    if (!cache_hazard_avoidance || sec.damageamount <= 0)
      return 0.0;

    if (cache_sr_reckless && player && player.health > 100)
      return 0.0;

    // If the player has a Radiation Suit or Invulnerability, ignore the hazard
    // penalty.
    if (player && (player.FindInventory("PowerIronFeet") ||
                   player.FindInventory("PowerInvulnerable"))) {
      return 0.0;
    }

    // Return a penalty proportional to the damage amount.
    return 1000.0 + sec.damageamount * 50.0;
  }

  // Snaps an unreachable/isolated/pedestal target sector to the nearest
  // passable neighboring sector.
  int GetValidSnappedSector(int endIdx, Actor target, Actor player) {
    if (endIdx < 0 || endIdx >= level.sectors.Size())
      return endIdx;

    Sector endSec = level.sectors[endIdx];
    if (!endSec)
      return endIdx;

    // 1. Check if the target sector is passable on its own (has sufficient
    // clearance for a player)
    double refZ =
        target ? target.pos.z : endSec.floorplane.ZatPoint(endSec.centerspot);
    double endFloor, endCeil;
    GetEffectiveFloorCeil(endSec, endSec.centerspot, refZ, endFloor, endCeil);
    double clearance = endCeil - endFloor;

    bool isTargetPassable = (clearance >= cache_clearance_min);
    if (!isTargetPassable && cache_wolfendoom_compat &&
        clearance >= (cache_clearance_min - 16.0)) {
      isTargetPassable = true;
    }

    // 2. Check if the target sector has any passable connections to its
    // neighbors.
    int nStart = adjStart[endIdx];
    int nEnd = adjStart[endIdx + 1];
    bool hasPassableConnection = false;

    for (int ni = nStart; ni < nEnd; ni++) {
      int neighbor = adjNeighbor[ni];
      Sector nextSec = level.sectors[neighbor];
      Line ln = level.lines[adjLineIdx[ni]];
      double nextFloor, nextCeil;
      if (IsPortalPassable(endSec, nextSec, ln, player, refZ, nextFloor, nextCeil)) {
        hasPassableConnection = true;
        break;
      }
    }

    // If it's passable and not isolated, keep it as is.
    if (isTargetPassable && hasPassableConnection) {
      return endIdx;
    }

    // 3. Otherwise, it's a pedestal/isolated sector. Find the best neighbor to
    // snap to.
    int bestNeighbor = -1;
    double closestDist = SCORE_INFINITY;
    Vector2 targetPos = target ? target.pos.xy : endSec.centerspot;

    for (int ni = nStart; ni < nEnd; ni++) {
      int neighbor = adjNeighbor[ni];
      Sector nextSec = level.sectors[neighbor];
      Line ln = level.lines[adjLineIdx[ni]];

      // Calculate the midpoint of the border line to measure distance
      Vector2 mid = (ln.v1.p + ln.v2.p) / 2.0;
      double dist = (mid - targetPos).Length();

      if (dist < closestDist) {
        // Verify the neighbor itself is passable
        double neighborFloor, neighborCeil;
        double neighborRefZ = nextSec.floorplane.ZatPoint(mid);
        GetEffectiveFloorCeil(nextSec, mid, neighborRefZ, neighborFloor,
                              neighborCeil);
        double neighborClearance = neighborCeil - neighborFloor;

        if (neighborClearance >= cache_clearance_min ||
            (cache_wolfendoom_compat &&
             neighborClearance >= (cache_clearance_min - 16.0))) {
          // Ensure the neighbor has at least one passable connection to its own
          // neighbors (other than endIdx)
          int nnStart = adjStart[neighbor];
          int nnEnd = adjStart[neighbor + 1];
          bool neighborIsConnected = false;
          for (int nni = nnStart; nni < nnEnd; nni++) {
            int nnNeighbor = adjNeighbor[nni];
            if (nnNeighbor == endIdx)
              continue;

            Sector nnSec = level.sectors[nnNeighbor];
            Line nnLn = level.lines[adjLineIdx[nni]];
            double nnFloor, nnCeil;
            if (IsPortalPassable(nextSec, nnSec, nnLn, player, neighborFloor,
                                 nnFloor, nnCeil)) {
              neighborIsConnected = true;
              break;
            }
          }

          if (neighborIsConnected) {
            closestDist = dist;
            bestNeighbor = neighbor;
          }
        }
      }
    }

    if (bestNeighbor != -1) {
      return bestNeighbor;
    }

    return endIdx; // Fallback to original if no valid neighbor found
  }

  // Unified A* / Dijkstra pathfinder. 
  // If goalIdx >= 0, it uses A* heuristic to target targetPos and returns 1 on success.
  // If goalIdx == -1, it uses pure Dijkstra to hit any true value in isGoal and returns the hit sector index.
  int RunPathfinder(int startIdx, int goalIdx, in out Array<bool> isGoal, Vector2 targetPos, Actor player) {
    int numSectors = level.sectors.Size();
    openSet.Clear();
    heapPos.Resize(numSectors);
    sectorZ.Resize(numSectors);

    for (int i = 0; i < numSectors; i++) {
      parent[i] = -1;
      parentLine[i] = -1;
      reachable[i] = 0;
      gScore[i] = SCORE_INFINITY;
      fScore[i] = SCORE_INFINITY;
      inOpenSet[i] = false;
      heapPos[i] = -1;
      sectorZ[i] = 0.0;
    }

    gScore[startIdx] = 0.0;
    if (goalIdx >= 0) fScore[startIdx] = (level.sectors[startIdx].centerspot - targetPos).Length();
    else fScore[startIdx] = 0.0;
    
    reachable[startIdx] = 1;
    inOpenSet[startIdx] = true;
    sectorZ[startIdx] = player.pos.z;
    HeapPush(startIdx);

    while (openSet.Size() > 0) {
      int current = HeapPop();
      inOpenSet[current] = false;
      
      if (goalIdx >= 0 && current == goalIdx) return 1;
      if (goalIdx == -1 && isGoal[current]) return current;

      int nStart = adjStart[current];
      int nEnd = adjStart[current + 1];
      for (int ni = nStart; ni < nEnd; ni++) {
        int neighbor = adjNeighbor[ni];
        Line ln = level.lines[adjLineIdx[ni]];
        if (!cache_portals && ln.isLinePortal())
          continue;
        Sector curSec = level.sectors[current];
        Sector nextSec = level.sectors[neighbor];

        double nextFloor;
        double nextCeil;
        bool isProven = (cache_learning_proven && adjProven[ni]);
        bool isPassable = IsPortalPassable(curSec, nextSec, ln, player, sectorZ[current], nextFloor, nextCeil, isProven);

        if (isPassable) {
          double dist = ln.isLinePortal()
                            ? (nextSec.centerspot + ln.getPortalDisplacement() -
                               curSec.centerspot)
                                  .Length()
                            : (nextSec.centerspot - curSec.centerspot).Length();
          double pDiscount = 1.0;
          if (cache_learning_pheromone) {
            pDiscount = 1.0 - (sectorPheromone[neighbor] * 0.4);
          }
          if (cache_learning_heuristics) {
            TextureID nTexID = nextSec.GetTexture(Sector.floor);
            string nTexName = TexMan.GetName(nTexID);
            nTexName = nTexName.MakeLower();
            int tIdx = FindTextureIndex(nTexName);
            if (tIdx < knownTextures.Size()) {
              pDiscount -= textureScores[tIdx] * 0.2;
            }
            if (brightnessSamples > 50) {
              double lightDiff = abs(nextSec.lightlevel - avgBrightness);
              if (lightDiff > 255.0) lightDiff = 255.0;
              pDiscount += (lightDiff / 255.0) * 0.2;
            }
          }
          if (pDiscount < 0.1) pDiscount = 0.1;

          double speedCostAdd = 0.0;
          double speedCostMul = 1.0;

          if (cache_sr_momentum && parent[current] != -1) {
            Vector2 v1 = curSec.centerspot - level.sectors[parent[current]].centerspot;
            Vector2 v2 = nextSec.centerspot - curSec.centerspot;
            double len1 = v1.Length();
            double len2 = v2.Length();
            if (len1 > 0 && len2 > 0) {
              double dotp = (v1.x * v2.x + v1.y * v2.y) / (len1 * len2);
              if (dotp < 0.5) speedCostAdd += cache_weight_momentum;
            }
          }

          if (cache_sr_door_penalty && ln.special > 0) {
            bool isSlowDoor = (ln.special >= 10 && ln.special <= 14) || ln.special == 202;
            bool isLift = (ln.special >= 20 && ln.special <= 39);
            if (isSlowDoor || isLift) speedCostAdd += cache_weight_doors;
          }

          if (cache_sr_monster_avoidance && monsterDensity.Size() > neighbor) {
            speedCostAdd += monsterDensity[neighbor] * cache_weight_monsters;
          }

          if (cache_sr_clearance) {
            if (nextCeil - nextFloor > 128.0) speedCostMul = 0.8;
          }

          double tentativeG = gScore[current] + (dist * pDiscount * speedCostMul) + speedCostAdd +
                              GetHazardPenalty(nextSec, player);

          if (tentativeG < gScore[neighbor]) {
            reachable[neighbor] = 1;
            parent[neighbor] = current;
            parentLine[neighbor] = adjLineIdx[ni];
            gScore[neighbor] = tentativeG;
            if (goalIdx >= 0) fScore[neighbor] = tentativeG + (nextSec.centerspot - targetPos).Length();
            else fScore[neighbor] = tentativeG;
            sectorZ[neighbor] = nextFloor;

            if (!inOpenSet[neighbor]) {
              inOpenSet[neighbor] = true;
              HeapPush(neighbor);
            } else {
              HeapSiftUp(heapPos[neighbor]);
            }
          }
        }
      }
    }

    return goalIdx >= 0 ? 0 : -1;
  }

  void BuildPathFromParent(int startIdx, int endIdx, Actor target) {
    pathX.Clear();
    pathY.Clear();

    Array<int> reversedLines;
    int cur = endIdx;
    while (cur != startIdx && parentLine[cur] >= 0) {
      reversedLines.Push(parentLine[cur]);
      cur = parent[cur];
    }

    for (int i = reversedLines.Size() - 1; i >= 0; i--) {
      Line ln = level.lines[reversedLines[i]];
      Vector2 mid = (ln.v1.p + ln.v2.p) / 2.0;
      pathX.Push(mid.x);
      pathY.Push(mid.y);
    }

    if (target) {
      pathX.Push(target.pos.x);
      pathY.Push(target.pos.y);
    }
  }

  // Synchronous multi-stage pathfinding: A* direct → pedestal retry → reverse
  // BFS → switch routing. This runs to completion on the same tick, ensuring
  // path data is always valid before markers render.
  void FindPathAStar(Actor player, Actor target) {
    pathX.Clear();
    pathY.Clear();

    if (graphBuilt == 0 || !player || !target)
      return;

    int numSectors = level.sectors.Size();

    Sector startSec = level.PointInSector(player.pos.xy);
    Sector endSec = level.PointInSector(target.pos.xy);

    if (!startSec || !endSec)
      return;

    int startIdx = startSec.sectornum;
    int endIdx = endSec.sectornum;
    int realEndIdx = cache_target_snapping ? GetValidSnappedSector(endIdx, target, player) : endIdx;

    if (startIdx == realEndIdx) {
      pathX.Push(target.pos.x);
      pathY.Push(target.pos.y);
      return;
    }

    parent.Resize(numSectors);
    parentLine.Resize(numSectors);
    reachable.Resize(numSectors);
    gScore.Resize(numSectors);
    fScore.Resize(numSectors);
    inOpenSet.Resize(numSectors);
    sectorZ.Resize(numSectors);

    Vector2 targetPos = target.pos.xy;
    Array<bool> dummyGoal;
    int found = RunPathfinder(startIdx, realEndIdx, dummyGoal, targetPos, player);

    if (found == 1) {
      BuildPathFromParent(startIdx, realEndIdx, target);
      return;
    }

    if (!cache_solve_switches)
      return;

    // Pass 3: Reverse topological fallback — find the barrier sector and route
    // to its switch. Runs a reverse BFS from the target back toward the
    // player's reachable zone. The first sector it hits that borders the
    // reachable zone is the "barrier sector" (locked door/gate).
    visitedReverse.Resize(numSectors);
    for (int i = 0; i < numSectors; i++)
      visitedReverse[i] = 0;

    queueReverse.Clear();
    visitedReverse[endIdx] = 1;
    queueReverse.Push(endIdx);

    int qHeadReverse = 0;
    Sector barrierSec = null;
    bool barrierFound = false;

    while (qHeadReverse < queueReverse.Size()) {
      int current = queueReverse[qHeadReverse];
      qHeadReverse++;

      int nStart = adjStart[current];
      int nEnd = adjStart[current + 1];
      for (int ni = nStart; ni < nEnd; ni++) {
        int neighbor = adjNeighbor[ni];

        if (reachable[neighbor] == 1) {
          barrierSec = level.sectors[current];
          barrierFound = true;
          break;
        }

        if (visitedReverse[neighbor] == 0) {
          visitedReverse[neighbor] = 1;
          queueReverse.Push(neighbor);
        }
      }
      if (barrierFound)
        break;
    }

    if (!barrierFound || !barrierSec)
      return;

    // Scan all linedefs for player-triggerable switches that target the barrier
    // sector's tag.
    int bestSwitchSector = -1;
    Line bestSwitchLine = null;
    int minPathSteps = int.MAX;

    for (int li = 0; li < level.lines.Size(); li++) {
      Line ln = level.lines[li];
      if (!ln || !IsPlayerTriggerable(ln))
        continue;

      int targetTag = ln.args[0];
      if (targetTag == 0)
        continue;

      bool hasTag = false;
      SectorTagIterator it = level.CreateSectorTagIterator(targetTag);
      int secNum;
      while ((secNum = it.Next()) >= 0) {
        if (secNum == barrierSec.sectornum) {
          hasTag = true;
          break;
        }
      }
      if (!hasTag)
        continue;

      int switchSecIdx = -1;
      if (ln.frontsector && reachable[ln.frontsector.sectornum] == 1) {
        switchSecIdx = ln.frontsector.sectornum;
      } else if (ln.backsector && reachable[ln.backsector.sectornum] == 1) {
        switchSecIdx = ln.backsector.sectornum;
      }

      if (switchSecIdx != -1) {
        int steps = 0;
        int cur = switchSecIdx;
        while (cur != startIdx && parentLine[cur] >= 0) {
          steps++;
          cur = parent[cur];
        }
        if (steps < minPathSteps) {
          minPathSteps = steps;
          bestSwitchSector = switchSecIdx;
          bestSwitchLine = ln;
        }
      }
    }

    if (bestSwitchLine && bestSwitchSector != -1) {
      // Route to the switch: re-run A* to the switch sector for a clean path
      found = RunPathfinder(startIdx, bestSwitchSector, dummyGoal, targetPos, player);
      if (found == 1) {
        BuildPathFromParent(startIdx, bestSwitchSector, null);
        Vector2 switchMid = (bestSwitchLine.v1.p + bestSwitchLine.v2.p) / 2.0;
        pathX.Push(switchMid.x);
        pathY.Push(switchMid.y);
      }
    }
  }

  // Compute effective walkable floor and ceiling at a point, accounting for
  // solid 3D floors. Partitioning the sector's vertical space based on refZ:
  // - Finds the highest solid floor at or below refZ + STEP_HEIGHT_MAX
  // (allowing stepping up/down).
  // - Finds the lowest ceiling (or bottom of a 3D floor) directly above refZ.
  // This allows the pathfinder and tracer to distinguish between stacked 3D
  // levels (e.g. on a bridge vs under it).
  clearscope static void GetEffectiveFloorCeil(Sector sec, Vector2 pos,
                                               double refZ, out double floorZ,
                                               out double ceilZ) {
    floorZ = sec.floorplane.ZatPoint(pos);
    ceilZ = sec.ceilingplane.ZatPoint(pos);

    PlayerInfo plyr = players[consoleplayer];
    double stepMax = 24.0;
    if (plyr) {
      CVar cv3d = CVar.GetCVar("holo_gps_3dfloors", plyr);
      if (cv3d && !cv3d.GetBool()) {
        return;
      }
      CVar cvStep = CVar.GetCVar("holo_gps_step_max", plyr);
      if (cvStep) stepMax = cvStep.GetFloat();
    }

    int count = sec.Get3DFloorCount();
    for (int i = 0; i < count; i++) {
      F3DFloor f3d = sec.Get3DFloor(i);
      if (!f3d || !(f3d.flags & F3DFloor.FF_EXISTS) ||
          !(f3d.flags & F3DFloor.FF_SOLID))
        continue;

      double fTop = f3d.top.ZatPoint(pos);
      double fBot = f3d.bottom.ZatPoint(pos);

      // A solid 3D floor acts as a barrier.
      // If the top of the 3D floor is below refZ, it could be a walkable floor.
      // We want the highest top that is <= refZ + stepMax.
      if (fTop <= refZ + stepMax && fTop > floorZ) {
        floorZ = fTop;
      }
      // If the bottom of the 3D floor is above refZ, it acts as a ceiling.
      // We want the lowest bottom that is > refZ.
      if (fBot > refZ && fBot < ceilZ) {
        ceilZ = fBot;
      }
    }
  }

  double GetFloorZ(Vector2 pos, double refZ) {
    Sector sec = level.PointInSector(pos);
    if (!sec)
      return refZ;
    double f, c;
    GetEffectiveFloorCeil(sec, pos, refZ, f, c);
    return f;
  }

  // Performs a 3D raycast line trace to determine if the line of sight between
  // start and end is blocked by geometry (walls, ceilings, floors, portals).
  // Uses refZ to resolve the correct 3D floor layer.
  bool PathIsBlocked(Vector2 start, Vector2 end, double refZ) {
    if (!mTracer)
      return false;

    // Resolve Z coordinate on start/end using the reference Z context
    double floorzA = GetFloorZ(start, refZ);
    double floorzB = GetFloorZ(end, floorzA);

    // Raycast is performed at TRACE_Z_OFFSET height above the resolved floor Z
    Vector3 pA = (start.x, start.y, floorzA + TRACE_Z_OFFSET);
    Vector3 pB = (end.x, end.y, floorzB + TRACE_Z_OFFSET);

    Vector3 diff = pB - pA;
    double len = diff.Length();
    if (len < 1.0)
      return false;
    Vector3 dir = diff / len;

    Sector sec = level.PointInSector(start);
    if (!sec)
      return false;

    // Trace ray using PathfinderTracer
    mTracer.Trace(pA, sec, dir, len, TRACE_ReportPortals, 0xFFFFFFFF, true);
    if (mTracer.Results.HitType != TRACE_HitNone)
      return true;

    // Validate midpoint: catch traces that clip through thin walls or vertical
    // level transitions
    if (len > 64.0) {
      Vector2 mid = (start + end) * 0.5;
      Sector midSec = level.PointInSector(mid);
      if (midSec) {
        double midFloor, midCeil;
        // Query effective heights at the midpoint using floorzA as reference
        // height
        GetEffectiveFloorCeil(midSec, mid, floorzA, midFloor, midCeil);
        // If midpoint has no clearance or a massive floor jump, it's blocked
        if (midCeil - midFloor < cache_clearance_min)
          return true;
        if (abs(midFloor - floorzA) > cache_step_max &&
            abs(midFloor - floorzB) > cache_step_max)
          return true;
      }
    }

    return false;
  }

  double GetPathDist(Vector2 start, in out Array<double> px,
                     in out Array<double> py, Vector2 end) {
    double d = 0;
    Vector2 prev = start;
    for (int i = 0; i < px.Size(); i++) {
      Vector2 cur = (px[i], py[i]);
      d += (cur - prev).Length();
      prev = cur;
    }
    d += (end - prev).Length();
    return d;
  }

  void AppendArray(in out Array<double> dest, in out Array<double> src) {
    for (int i = 0; i < src.Size(); i++) {
      dest.Push(src[i]);
    }
  }

  // Attempts to find a safe, walkable position near a candidate detour point by
  // nudging outward from the wall to avoid landing inside geometry or
  // low-clearance areas.
  Vector2 GetSafeDetourPoint(Vector2 candidate, Vector2 perp, double refZ) {
    double stepSize = 12.0;
    int maxAttempts = 3;

    for (int i = 0; i <= maxAttempts; i++) {
      Vector2 testPt = candidate + perp * (i * stepSize);
      Sector sec = level.PointInSector(testPt);
      if (!sec)
        continue;

      double floorZ, ceilZ;
      GetEffectiveFloorCeil(sec, testPt, refZ, floorZ, ceilZ);

      // Check clearance and step height relative to our reference height
      if (ceilZ - floorZ >= cache_clearance_min &&
          abs(floorZ - refZ) <= cache_step_max) {
        return testPt;
      }
    }

    return candidate; // Fallback
  }

  // Calculates a physical detour around a blocking wall or corner between two
  // points A and B. Why: A* outputs topological waypoints (midpoints of
  // portals/lines), which can result in lines
  //      clipping corners of walls or pillars.
  // How (Recursive Detour Raycast):
  // - Traces a ray from A to B. If it hits a linedef (`hitLn`):
  // - Finds the two endpoints of the hit line, and offsets them outward
  // (`DETOUR_WALL_MARGIN`, `DETOUR_PERP_MARGIN`)
  //   to generate two candidate detour points (d1 and d2).
  // - Recursively solves `GetDetourPath(A, d1, ...)` and `GetDetourPath(d1, B,
  // ...)` (and similarly for d2).
  // - Selects the shorter of the two detour paths that compiles successfully,
  // refining the path to wrap smoothly around corners.
  bool GetDetourPath(Vector2 A, Vector2 B, in out Array<double> outX,
                     in out Array<double> outY, double refZ, int depth = 0) {
    if (depth > 3 || !mTracer)
      return false;

    double floorzA = GetFloorZ(A, refZ);
    double floorzB = GetFloorZ(B, floorzA);

    Vector3 start = (A.x, A.y, floorzA + TRACE_Z_OFFSET);
    Vector3 end = (B.x, B.y, floorzB + TRACE_Z_OFFSET);

    Vector3 diff = end - start;
    double len = diff.Length();
    if (len < 1.0)
      return true;
    Vector3 dir = diff / len;

    Sector sec = level.PointInSector(A);
    if (!sec)
      return false;

    mTracer.Trace(start, sec, dir, len, TRACE_ReportPortals, 0xFFFFFFFF, true);
    if (mTracer.Results.HitType == TRACE_HitNone)
      return true;

    Line hitLn = mTracer.Results.HitLine;
    if (!hitLn)
      return false;

    Vector2 lnDir = (hitLn.v2.p - hitLn.v1.p).Unit();
    Vector2 perp = (-lnDir.y, lnDir.x);

    if (((A - hitLn.v1.p) dot perp) < 0) {
      perp = -perp;
    }

    Vector2 d1 =
        hitLn.v1.p - lnDir * DETOUR_WALL_MARGIN + perp * DETOUR_PERP_MARGIN;
    if (cache_raycast_detouring) {
      d1 = GetSafeDetourPoint(d1, perp, floorzA);
    }

    Vector2 d2 =
        hitLn.v2.p + lnDir * DETOUR_WALL_MARGIN + perp * DETOUR_PERP_MARGIN;
    if (cache_raycast_detouring) {
      d2 = GetSafeDetourPoint(d2, perp, floorzA);
    }

    Array<double> path1X;
    Array<double> path1Y;
    bool ok1 = GetDetourPath(A, d1, path1X, path1Y, floorzA, depth + 1);
    if (ok1) {
      path1X.Push(d1.x);
      path1Y.Push(d1.y);
      double d1Floor = GetFloorZ(d1, floorzA);
      ok1 = GetDetourPath(d1, B, path1X, path1Y, d1Floor, depth + 1);
    }

    Array<double> path2X;
    Array<double> path2Y;
    bool ok2 = GetDetourPath(A, d2, path2X, path2Y, floorzA, depth + 1);
    if (ok2) {
      path2X.Push(d2.x);
      path2Y.Push(d2.y);
      double d2Floor = GetFloorZ(d2, floorzA);
      ok2 = GetDetourPath(d2, B, path2X, path2Y, d2Floor, depth + 1);
    }

    if (ok1 && ok2) {
      double len1 = GetPathDist(A, path1X, path1Y, B);
      double len2 = GetPathDist(A, path2X, path2Y, B);
      if (len1 <= len2) {
        AppendArray(outX, path1X);
        AppendArray(outY, path1Y);
      } else {
        AppendArray(outX, path2X);
        AppendArray(outY, path2Y);
      }
      return true;
    } else if (ok1) {
      AppendArray(outX, path1X);
      AppendArray(outY, path1Y);
      return true;
    } else if (ok2) {
      AppendArray(outX, path2X);
      AppendArray(outY, path2Y);
      return true;
    }

    return false;
  }

  void RefinePath(Vector2 start, double startZ) {
    refinedX.Clear();
    refinedY.Clear();

    if (pathX.Size() == 0)
      return;

    Vector2 prev = start;
    double prevZ = startZ;
    for (int i = 0; i < pathX.Size(); i++) {
      Vector2 wp = (pathX[i], pathY[i]);

      Array<double> detX;
      Array<double> detY;
      bool ok = GetDetourPath(prev, wp, detX, detY, prevZ, 0);
      if (ok) {
        for (int j = 0; j < detX.Size(); j++) {
          refinedX.Push(detX[j]);
          refinedY.Push(detY[j]);
        }
      }
      refinedX.Push(wp.x);
      refinedY.Push(wp.y);
      prevZ = GetFloorZ(wp, prevZ);
      prev = wp;
    }
  }

  void SpawnPathMarkers(Actor player) {
    if (refinedX.Size() == 0 || !player)
      return;

    if (cache_alpha <= 0.0)
      cache_alpha = 0.6;
    if (cache_spacing < 32)
      cache_spacing = 32;
    if (cache_max_markers < 1)
      cache_max_markers = 1;

    Vector2 prevPoint = player.pos.xy;
    Vector2 lastSpawnPos = player.pos.xy;
    double currentZ = player.pos.z;
    int spawnedCount = 0;

    for (int wp = 0; wp < refinedX.Size(); wp++) {
      if (spawnedCount >= cache_max_markers)
        break;

      Vector2 waypoint = (refinedX[wp], refinedY[wp]);
      Vector2 seg = waypoint - prevPoint;
      double segLen = seg.Length();

      if (segLen < 16) {
        prevPoint = waypoint;
        currentZ = GetFloorZ(waypoint, currentZ);
        continue;
      }

      int segSteps = int(segLen / cache_spacing);
      if (segSteps < 1)
        segSteps = 1;

      Vector2 stepDir = seg / segLen;
      double arrowAngle = VectorAngle(seg.x, seg.y);

      for (int i = 1; i <= segSteps; i++) {
        if (spawnedCount >= cache_max_markers)
          break;

        Vector2 spawnXY = prevPoint + stepDir * (cache_spacing * i);

        if ((spawnXY - prevPoint).Length() > segLen)
          spawnXY = waypoint;

        if (PathIsBlocked(lastSpawnPos, spawnXY, currentZ)) {
          lastSpawnPos = spawnXY;
          currentZ = GetFloorZ(spawnXY, currentZ);
          continue;
        }

        Sector spawnSec = level.PointInSector(spawnXY);
        if (!spawnSec)
          continue;

        double floorz = GetFloorZ(spawnXY, currentZ);
        currentZ = floorz;
        Vector3 spawnPos = (spawnXY.x, spawnXY.y, floorz + cache_height);

        Actor marker;
        // Reuse existing pooled marker to bypass the expensive instantiation of
        // a new Actor. We just warp its coordinates using SetOrigin, avoiding
        // CPU/memory overhead.
        if (spawnedCount < activeMarkers.Size() &&
            activeMarkers[spawnedCount] &&
            !activeMarkers[spawnedCount].bDestroyed) {
          marker = activeMarkers[spawnedCount];
          marker.SetOrigin(spawnPos, false);
        } else {
          // If the pool is completely exhausted, spawn a new actor and append
          // it to our pool.
          marker = Actor.Spawn("HoloPathMarker", spawnPos);
          if (marker) {
            if (spawnedCount < activeMarkers.Size())
              activeMarkers[spawnedCount] = marker;
            else
              activeMarkers.Push(marker);
          }
        }

        if (marker) {
          if (cache_scale <= 0.0)
            cache_scale = 0.25;
          marker.Scale = (cache_scale, cache_scale);

          double markerAlpha = cache_alpha;
          if (cache_fade) {
            markerAlpha =
                cache_alpha *
                (1.0 - (double(spawnedCount) / double(cache_max_markers)));
          }

          marker.angle = arrowAngle;

          int renderStyle =
              (cache_style == 1) ? STYLE_TranslucentStencil : STYLE_AddStencil;
          int defaultStyle = (cache_style == 1) ? STYLE_Translucent : STYLE_Add;

          if (cache_color == 9) {
            marker.A_SetRenderStyle(markerAlpha, renderStyle);
            marker.SetShade(TRANS_COLORS[spawnedCount % 4]);
          } else if (cache_color >= 1 && cache_color <= 8) {
            marker.A_SetRenderStyle(markerAlpha, renderStyle);
            marker.SetShade(COLOR_TABLE[cache_color]);
          } else {
            marker.A_SetRenderStyle(markerAlpha, defaultStyle);
          }

          spawnedCount++;
          lastSpawnPos = spawnXY;
        }
      }
      if (spawnedCount >= cache_max_markers)
        break;
      prevPoint = waypoint;
    }

    // Destroy excess pooled markers beyond what we used this frame
    for (int i = activeMarkers.Size() - 1; i >= spawnedCount; i--) {
      if (activeMarkers[i] && !activeMarkers[i].bDestroyed) {
        activeMarkers[i].Destroy();
      }
      activeMarkers.Delete(i);
    }
  }
}