
class HoloGPSHandler : StaticEventHandler
{
    // Navigation geometry thresholds
    const CLEARANCE_MIN = 56.0;       // Minimum ceiling-floor gap to consider passable
    const CLEARANCE_MIN_WOLF = 40.0;  // Relaxed clearance for WolfenDoom elevator logic
    const STEP_HEIGHT_MAX = 24.0;     // Maximum floor height difference to step over
    const TRACE_Z_OFFSET = 16.0;      // Height above floor for line-of-sight traces
    const EXIT_NORMAL_PUSH = 32.0;    // Distance to push exit targets away from their line
    const DETOUR_WALL_MARGIN = 32.0;  // Longitudinal margin when detouring around walls
    const DETOUR_PERP_MARGIN = 24.0;  // Perpendicular margin when detouring around walls
    const SCORE_INFINITY = 1e37;      // Sentinel value for unreached A* nodes

    // Stencil color lookup: 0=unused, 1=Red, 2=Green, 3=Blue, 4=Yellow, 5=Orange, 6=Purple, 7=Pink, 8=White
    static const color COLOR_TABLE[] = {
        0x000000, 0xFF0000, 0x00FF00, 0x0000FF,
        0xFFFF00, 0xFF8000, 0x8000FF, 0xFF00FF, 0xFFFFFF
    };

    // Trans pride flag cycle: blue, pink, white, pink
    static const color TRANS_COLORS[] = { 0x5BCEFA, 0xF5A9B8, 0xFFFFFF, 0xF5A9B8 };

    Actor currentTarget;
    int tickCounter;
    bool pathFresh; // True when FindNextObjective just found a target (path already computed)
    Actor lastTarget;
    int lastPlayerSectorIdx;
    int lastTargetSectorIdx;
    Array<Actor> activeMarkers;

    const ASYNC_STATE_IDLE = 0;
    const ASYNC_STATE_ASTAR_DIRECT = 1;
    const ASYNC_STATE_ASTAR_PEDESTAL = 2;
    const ASYNC_STATE_REVERSE_BFS = 3;
    const ASYNC_STATE_ASTAR_SWITCH = 4;

    int searchState;
    int searchStartIdx;
    int searchEndIdx;
    int searchRealEndIdx;
    int searchAlternativeEndIdx;
    Vector2 searchTargetPos;
    Actor searchPlayer;
    Actor searchTarget;

    // For reverse BFS (Phase 3)
    int searchQHeadReverse;
    Sector searchBarrierSec;
    bool searchBarrierFound;
    int searchBestSwitchSector;
    Line searchBestSwitchLine;
    int searchMinPathSteps;
    int searchLi;

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

    // Persistent helper arrays to avoid garbage collector thrashing
    Array<int> parent;
    Array<int> parentLine;
    Array<int> reachable;
    Array<double> gScore;
    Array<double> fScore;
    Array<int> openSet;
    Array<bool> inOpenSet;
    Array<int> heapPos;  // Maps sector index → position in openSet heap (-1 if absent)

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

    override void WorldLoaded(WorldEvent e)
    {
        currentTarget = null;
        tickCounter = 0;
        activeMarkers.Clear();
        pathX.Clear();
        pathY.Clear();
        refinedX.Clear();
        refinedY.Clear();
        graphBuilt = 0;
        searchState = ASYNC_STATE_IDLE;
        
        mTracer = new("PathfinderTracer");

        ykeySprite = Actor.GetSpriteIndex("YKEY");
        bkeySprite = Actor.GetSpriteIndex("BKEY");
        rkeySprite = Actor.GetSpriteIndex("RKEY");
        gkeySprite = Actor.GetSpriteIndex("GKEY");
        skeySprite = Actor.GetSpriteIndex("SKEY");
        goldKeySprite = Actor.GetSpriteIndex("GOLD"); 
        silvKeySprite = Actor.GetSpriteIndex("SILV"); 
        spritesCached = true;

        BuildAdjacencyGraph();
        FindNextObjective();
    }

    void BuildAdjacencyGraph()
    {
        int numSectors = level.sectors.Size();
        int numLines = level.lines.Size();

        Array<int> neighborCount;
        neighborCount.Resize(numSectors);
        for (int i = 0; i < numSectors; i++) neighborCount[i] = 0;

        for (int li = 0; li < numLines; li++)
        {
            Line ln = level.lines[li];
            if (!ln) continue;

            if (ln.isLinePortal())
            {
                Line dest = ln.getPortalDestination();
                if (dest && ln.frontsector && dest.frontsector)
                {
                    int fi = ln.frontsector.sectornum;
                    int bi = dest.frontsector.sectornum;
                    if (fi != bi)
                    {
                        neighborCount[fi]++;
                        neighborCount[bi]++;
                    }
                }
            }
            else
            {
                if (!ln.frontsector || !ln.backsector) continue;
                if (ln.flags & Line.ML_BLOCKING) continue;

                int fi = ln.frontsector.sectornum;
                int bi = ln.backsector.sectornum;
                if (fi == bi) continue;

                neighborCount[fi]++;
                neighborCount[bi]++;
            }
        }

        adjStart.Clear();
        adjStart.Resize(numSectors + 1);
        adjStart[0] = 0;
        for (int i = 0; i < numSectors; i++)
        {
            adjStart[i + 1] = adjStart[i] + neighborCount[i];
        }

        int totalAdj = adjStart[numSectors];
        adjNeighbor.Clear();
        adjNeighbor.Resize(totalAdj);
        adjLineIdx.Clear();
        adjLineIdx.Resize(totalAdj);

        Array<int> fillPos;
        fillPos.Resize(numSectors);
        for (int i = 0; i < numSectors; i++) fillPos[i] = adjStart[i];

        for (int li = 0; li < numLines; li++)
        {
            Line ln = level.lines[li];
            if (!ln) continue;

            if (ln.isLinePortal())
            {
                Line dest = ln.getPortalDestination();
                if (dest && ln.frontsector && dest.frontsector)
                {
                    int fi = ln.frontsector.sectornum;
                    int bi = dest.frontsector.sectornum;
                    if (fi != bi)
                    {
                        adjNeighbor[fillPos[fi]] = bi;
                        adjLineIdx[fillPos[fi]] = li;
                        fillPos[fi]++;

                        adjNeighbor[fillPos[bi]] = fi;
                        adjLineIdx[fillPos[bi]] = li;
                        fillPos[bi]++;
                    }
                }
            }
            else
            {
                if (!ln.frontsector || !ln.backsector) continue;
                if (ln.flags & Line.ML_BLOCKING) continue;

                int fi = ln.frontsector.sectornum;
                int bi = ln.backsector.sectornum;
                if (fi == bi) continue;

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

    void InitCVars(PlayerInfo plyr)
    {
        if (cv_enabled) return;
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
    }

    override void WorldTick()
    {
        PlayerInfo plyr = players[consoleplayer];
        if (!plyr || !plyr.mo || plyr.mo.health <= 0) return;

        InitCVars(plyr);

        cache_enabled = cv_enabled.GetBool();
        if (!cache_enabled) return;

        cache_freq = cv_freq.GetInt();
        if (cache_freq < 1) cache_freq = 1;
        tickCounter++;
        if (tickCounter % cache_freq != 0) return;

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

        if (currentTarget)
        {
            if (currentTarget is "Inventory")
            {
                Inventory inv = Inventory(currentTarget);
                if (inv.owner || plyr.mo.FindInventory(inv.GetClass()))
                {
                    currentTarget = null;
                }
            }
        }

        if (!currentTarget || currentTarget.bDestroyed)
        {
            currentTarget = null;
            pathFresh = false;
            FindNextObjective();
        }

        if (currentTarget)
        {
            ClearOldMarkers();

            Sector pSec = level.PointInSector(plyr.mo.pos.xy);
            Sector tSec = level.PointInSector(currentTarget.pos.xy);
            int pIdx = pSec ? pSec.sectornum : -1;
            int tIdx = tSec ? tSec.sectornum : -1;

            bool needsRebuild = true;
            if (!pathFresh && currentTarget == lastTarget && pIdx == lastPlayerSectorIdx && tIdx == lastTargetSectorIdx)
            {
                needsRebuild = false;
            }

            if (needsRebuild)
            {
                if (!pathFresh)
                {
                    StartAsyncSearch(plyr.mo, currentTarget);
                }
                lastTarget = currentTarget;
                lastPlayerSectorIdx = pIdx;
                lastTargetSectorIdx = tIdx;
            }

            if (searchState != ASYNC_STATE_IDLE)
            {
                TickAsyncSearch();
            }

            pathFresh = false;
            RefinePath(plyr.mo.pos.xy);
            SpawnPathMarkers(plyr.mo);
        }
    }

    void ClearOldMarkers()
    {
        // Hide pooled markers instead of destroying them
        for (int i = 0; i < activeMarkers.Size(); i++)
        {
            if (activeMarkers[i] && !activeMarkers[i].bDestroyed)
            {
                activeMarkers[i].A_SetRenderStyle(0.0, STYLE_None);
            }
        }
    }

    void FindNextObjective()
    {
        searchState = ASYNC_STATE_IDLE;
        PlayerInfo plyr = players[consoleplayer];
        if (!plyr || !plyr.mo) return;

        // Pass 1: Find valid, reachable Keys using single multi-target search
        if (cache_priority == 0 || cache_priority == 1)
        {
            // Collect all uncollected key candidates and their sectors
            Array<Actor> keyCandidates;
            Array<int> keySectors;

            ThinkerIterator it = ThinkerIterator.Create((cache_extended_search || cache_wolfendoom_compat) ? "Inventory" : "Key");
            Inventory mapKey;
            while (mapKey = Inventory(it.Next()))
            {
                if (mapKey.owner) continue;

                bool isKey = false;
                if (mapKey is "Key")
                {
                    isKey = true;
                }
                else if (cache_extended_search && mapKey.bSPECIAL)
                {
                    string cname = mapKey.GetClassName();
                    cname = cname.MakeLower();
                    
                    bool hasKeyMatch = false;
                    if (cname.IndexOf("key") == cname.Length() - 3) hasKeyMatch = true;
                    else if (cname.IndexOf("key") == 0) hasKeyMatch = true;
                    else if (cname.IndexOf("card") == cname.Length() - 4) hasKeyMatch = true;
                    else if (cname.IndexOf("skull") == cname.Length() - 5) hasKeyMatch = true;

                    if (hasKeyMatch)
                    {
                        isKey = true;
                    }
                }

                if (!isKey && cache_wolfendoom_compat && mapKey.bSPECIAL)
                {
                    if (mapKey.sprite == ykeySprite || mapKey.sprite == bkeySprite || 
                        mapKey.sprite == rkeySprite || mapKey.sprite == gkeySprite || 
                        mapKey.sprite == skeySprite || mapKey.sprite == goldKeySprite || 
                        mapKey.sprite == silvKeySprite)
                    {
                        isKey = true;
                    }
                }

                if (isKey && !plyr.mo.FindInventory(mapKey.GetClass()))
                {
                    Sector keySec = level.PointInSector(mapKey.pos.xy);
                    if (keySec)
                    {
                        keyCandidates.Push(mapKey);
                        keySectors.Push(keySec.sectornum);
                    }
                }
            }

            // Single multi-target search: find nearest reachable key
            if (keyCandidates.Size() > 0)
            {
                Sector startSec = level.PointInSector(plyr.mo.pos.xy);
                if (startSec)
                {
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
                    for (int i = 0; i < numSectors; i++) isGoal[i] = false;
                    for (int i = 0; i < keySectors.Size(); i++) isGoal[keySectors[i]] = true;

                    int hitGoal = RunAStarMultiGoal(startIdx, isGoal, plyr.mo.pos.xy, plyr.mo);
                    if (hitGoal >= 0)
                    {
                        // Find which key actor lives in the hit sector
                        for (int i = 0; i < keyCandidates.Size(); i++)
                        {
                            if (keySectors[i] == hitGoal)
                            {
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
        if (cache_priority == 0 || cache_priority == 2)
        {
            Array<Actor> exitSpots;
            Array<int> exitSectors;

            for (int i = 0; i < level.lines.Size(); i++)
            {
                Line ln = level.lines[i];
                if (ln && (ln.special == 243 || ln.special == 244 || ln.special == 74 || ln.special == 124))
                {
                    Vector2 midpoint = (ln.v1.p + ln.v2.p) / 2.0;

                    // FIX: Push target 32 units away from the line into its front sector.
                    // This prevents pathfinding into tight switch crevices and walls.
                    if (ln.frontsector)
                    {
                        Vector2 lineVec = ln.v2.p - ln.v1.p;
                        Vector2 normal = (lineVec.y, -lineVec.x).Unit(); // Right hand normal points to front sector
                        midpoint = midpoint + normal * EXIT_NORMAL_PUSH;
                    }

                    Sector exitSec = level.PointInSector(midpoint);
                    if (!exitSec) continue; 

                    double z = exitSec.floorplane.ZatPoint(midpoint);
                    Actor spot = Actor.Spawn("MapSpot", (midpoint.x, midpoint.y, z));
                    if (spot)
                    {
                        exitSpots.Push(spot);
                        exitSectors.Push(exitSec.sectornum);
                    }
                }
            }

            int foundIdx = -1;
            if (exitSpots.Size() > 0)
            {
                Sector startSec = level.PointInSector(plyr.mo.pos.xy);
                if (startSec)
                {
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
                    for (int i = 0; i < numSectors; i++) isGoal[i] = false;
                    for (int i = 0; i < exitSectors.Size(); i++) isGoal[exitSectors[i]] = true;

                    int hitGoal = RunAStarMultiGoal(startIdx, isGoal, plyr.mo.pos.xy, plyr.mo);
                    if (hitGoal >= 0)
                    {
                        for (int i = 0; i < exitSpots.Size(); i++)
                        {
                            if (exitSectors[i] == hitGoal)
                            {
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
            for (int i = 0; i < exitSpots.Size(); i++)
            {
                if (i != foundIdx && exitSpots[i])
                {
                    exitSpots[i].Destroy();
                }
            }
            if (foundIdx >= 0) return;
        }

        currentTarget = null;
    }

    bool IsPortalPassable(Sector curSec, Sector nextSec, Line ln, Actor player)
    {
        if (!ln || !curSec || !nextSec) return false;

        if (ln.flags & Line.ML_BLOCKING) return false;
        if (!cache_use_secrets && (ln.flags & Line.ML_SECRET)) return false;

        if (ln.locknumber != 0)
        {
            PlayerPawn plyrPawn = PlayerPawn(player);
            if (plyrPawn && !plyrPawn.CheckKeys(ln.locknumber, false))
            {
                return false; 
            }
        }

        Vector2 midpoint = (ln.v1.p + ln.v2.p) / 2.0;
        Vector2 nextMidpoint = midpoint;
        if (ln.isLinePortal())
        {
            nextMidpoint = midpoint + ln.getPortalDisplacement();
        }
        double curFloor, curCeil, nextFloor, nextCeil;
        GetEffectiveFloorCeil(curSec, midpoint, curFloor, curCeil);
        GetEffectiveFloorCeil(nextSec, nextMidpoint, nextFloor, nextCeil);

        if (abs(nextFloor - curFloor) > STEP_HEIGHT_MAX) return false;

        double portalFloor = (curFloor > nextFloor) ? curFloor : nextFloor;
        double portalCeil = (curCeil < nextCeil) ? curCeil : nextCeil;
        double clearance = portalCeil - portalFloor;

        // FIX: Lowered clearance check if WolfenDoom compat is enabled
        if (clearance < CLEARANCE_MIN)
        {
            if (cache_wolfendoom_compat && clearance >= CLEARANCE_MIN_WOLF)
            {
                // Pass for tight WolfenDoom elevator logic
            }
            else
            {
                bool isDirectUse = (ln.activation & (SPAC_Use | SPAC_UseThrough)) != 0;
                bool isStandardDoor = (ln.special >= 10 && ln.special <= 13) || ln.special == 105 || ln.special == 106 || ln.special == 202;
                bool isScriptDoor = (ln.special == 80 || ln.special == 226); 

                if (!isDirectUse && !isStandardDoor && !isScriptDoor)
                {
                    return false;
                }
            }
        }

        return true;
    }

    bool IsPlayerTriggerable(Line ln)
    {
        if (!ln || ln.special == 0) return false;
        int act = ln.activation;
        if (act & (SPAC_Use | SPAC_UseThrough | SPAC_Cross | SPAC_Impact | SPAC_Push | SPAC_AnyCross))
        {
            return true;
        }
        return false;
    }

    // --- Binary min-heap helpers operating on openSet[], keyed by fScore[] ---

    void HeapSwap(int a, int b)
    {
        int tmp = openSet[a];
        openSet[a] = openSet[b];
        openSet[b] = tmp;
        heapPos[openSet[a]] = a;
        heapPos[openSet[b]] = b;
    }

    void HeapSiftUp(int idx)
    {
        while (idx > 0)
        {
            int par = (idx - 1) / 2;
            if (fScore[openSet[idx]] < fScore[openSet[par]])
            {
                HeapSwap(idx, par);
                idx = par;
            }
            else break;
        }
    }

    void HeapSiftDown(int idx, int size)
    {
        while (true)
        {
            int best = idx;
            int left = 2 * idx + 1;
            int right = 2 * idx + 2;
            if (left < size && fScore[openSet[left]] < fScore[openSet[best]]) best = left;
            if (right < size && fScore[openSet[right]] < fScore[openSet[best]]) best = right;
            if (best != idx)
            {
                HeapSwap(idx, best);
                idx = best;
            }
            else break;
        }
    }

    void HeapPush(int sector)
    {
        int idx = openSet.Size();
        openSet.Push(sector);
        heapPos[sector] = idx;
        HeapSiftUp(idx);
    }

    int HeapPop()
    {
        int top = openSet[0];
        int last = openSet.Size() - 1;
        if (last > 0)
        {
            HeapSwap(0, last);
        }
        heapPos[top] = -1;
        openSet.Delete(last);
        if (openSet.Size() > 0) HeapSiftDown(0, openSet.Size());
        return top;
    }

    // Multi-goal Dijkstra core. Returns the index of the first goal sector reached, or -1.
    int RunAStarMultiGoal(int startIdx, in out Array<bool> isGoal, Vector2 targetPos, Actor player)
    {
        int numSectors = level.sectors.Size();
        openSet.Clear();
        heapPos.Resize(numSectors);

        for (int i = 0; i < numSectors; i++)
        {
            parent[i] = -1;
            parentLine[i] = -1;
            reachable[i] = 0;
            gScore[i] = SCORE_INFINITY;
            fScore[i] = SCORE_INFINITY;
            inOpenSet[i] = false;
            heapPos[i] = -1;
        }

        gScore[startIdx] = 0.0;
        fScore[startIdx] = 0.0;
        reachable[startIdx] = 1;
        inOpenSet[startIdx] = true;
        HeapPush(startIdx);

        while (openSet.Size() > 0)
        {
            int current = HeapPop();
            inOpenSet[current] = false;
            if (isGoal[current]) return current;

            int nStart = adjStart[current];
            int nEnd = adjStart[current + 1];
            for (int ni = nStart; ni < nEnd; ni++)
            {
                int neighbor = adjNeighbor[ni];
                Line ln = level.lines[adjLineIdx[ni]];
                Sector curSec = level.sectors[current];
                Sector nextSec = level.sectors[neighbor];

                if (IsPortalPassable(curSec, nextSec, ln, player))
                {
                    double dist = ln.isLinePortal() ? (nextSec.centerspot + ln.getPortalDisplacement() - curSec.centerspot).Length() : (nextSec.centerspot - curSec.centerspot).Length();
                    double tentativeG = gScore[current] + dist;

                    if (tentativeG < gScore[neighbor])
                    {
                        reachable[neighbor] = 1;
                        parent[neighbor] = current;
                        parentLine[neighbor] = adjLineIdx[ni];
                        gScore[neighbor] = tentativeG;
                        fScore[neighbor] = tentativeG; // Pure Dijkstra

                        if (!inOpenSet[neighbor])
                        {
                            inOpenSet[neighbor] = true;
                            HeapPush(neighbor);
                        }
                        else
                        {
                            HeapSiftUp(heapPos[neighbor]);
                        }
                    }
                }
            }
        }

        return -1;
    }

    void BuildPathFromParent(int startIdx, int endIdx, Actor target)
    {
        pathX.Clear();
        pathY.Clear();

        Array<int> reversedLines;
        int cur = endIdx;
        while (cur != startIdx && parentLine[cur] >= 0)
        {
            reversedLines.Push(parentLine[cur]);
            cur = parent[cur];
        }

        for (int i = reversedLines.Size() - 1; i >= 0; i--)
        {
            Line ln = level.lines[reversedLines[i]];
            Vector2 mid = (ln.v1.p + ln.v2.p) / 2.0;
            pathX.Push(mid.x);
            pathY.Push(mid.y);
        }

        if (target)
        {
            pathX.Push(target.pos.x);
            pathY.Push(target.pos.y);
        }
    }

    void InitAStarAsync(int startIdx, int goalIdx, Vector2 targetPos, Actor player)
    {
        int numSectors = level.sectors.Size();
        openSet.Clear();
        heapPos.Resize(numSectors);

        for (int i = 0; i < numSectors; i++)
        {
            parent[i] = -1;
            parentLine[i] = -1;
            reachable[i] = 0;
            gScore[i] = SCORE_INFINITY;
            fScore[i] = SCORE_INFINITY;
            inOpenSet[i] = false;
            heapPos[i] = -1;
        }

        gScore[startIdx] = 0.0;
        fScore[startIdx] = (level.sectors[startIdx].centerspot - targetPos).Length();
        reachable[startIdx] = 1;
        inOpenSet[startIdx] = true;
        HeapPush(startIdx);

        searchStartIdx = startIdx;
        searchEndIdx = goalIdx;
        searchTargetPos = targetPos;
        searchPlayer = player;
    }

    int ResumeAStarAsync(int limit)
    {
        int iterations = 0;
        while (openSet.Size() > 0 && iterations < limit)
        {
            iterations++;
            int current = HeapPop();
            inOpenSet[current] = false;
            if (current == searchEndIdx) return 1;

            int nStart = adjStart[current];
            int nEnd = adjStart[current + 1];
            for (int ni = nStart; ni < nEnd; ni++)
            {
                int neighbor = adjNeighbor[ni];
                Line ln = level.lines[adjLineIdx[ni]];
                Sector curSec = level.sectors[current];
                Sector nextSec = level.sectors[neighbor];

                if (IsPortalPassable(curSec, nextSec, ln, searchPlayer))
                {
                    double dist = ln.isLinePortal() ? (nextSec.centerspot + ln.getPortalDisplacement() - curSec.centerspot).Length() : (nextSec.centerspot - curSec.centerspot).Length();
                    double tentativeG = gScore[current] + dist;

                    if (tentativeG < gScore[neighbor])
                    {
                        reachable[neighbor] = 1;
                        parent[neighbor] = current;
                        parentLine[neighbor] = adjLineIdx[ni];
                        gScore[neighbor] = tentativeG;

                        double heuristic = 0;
                        Sector targetSec = level.PointInSector(searchTargetPos);
                        if (targetSec && nextSec.PortalGroup == targetSec.PortalGroup)
                        {
                            heuristic = (nextSec.centerspot - searchTargetPos).Length();
                        }
                        fScore[neighbor] = tentativeG + heuristic;

                        if (!inOpenSet[neighbor])
                        {
                            inOpenSet[neighbor] = true;
                            HeapPush(neighbor);
                        }
                        else
                        {
                            HeapSiftUp(heapPos[neighbor]);
                        }
                    }
                }
            }
        }

        if (openSet.Size() == 0) return 0;
        return -1;
    }

    void InitReverseBFSAsync()
    {
        int numSectors = level.sectors.Size();
        visitedReverse.Resize(numSectors);
        for (int i = 0; i < numSectors; i++) visitedReverse[i] = 0;

        queueReverse.Clear();
        visitedReverse[searchEndIdx] = 1;
        queueReverse.Push(searchEndIdx);

        searchQHeadReverse = 0;
        searchBarrierSec = null;
        searchBarrierFound = false;
        searchBestSwitchSector = -1;
        searchBestSwitchLine = null;
        searchMinPathSteps = int.MAX;
        searchLi = 0;
    }

    int ResumeReverseBFSAsync(int limit)
    {
        int iterations = 0;
        
        while (searchQHeadReverse < queueReverse.Size() && !searchBarrierFound && iterations < limit)
        {
            iterations++;
            int current = queueReverse[searchQHeadReverse];
            searchQHeadReverse++;

            int nStart = adjStart[current];
            int nEnd = adjStart[current + 1];
            for (int ni = nStart; ni < nEnd; ni++)
            {
                int neighbor = adjNeighbor[ni];

                if (reachable[neighbor] == 1)
                {
                    searchBarrierSec = level.sectors[current];
                    searchBarrierFound = true;
                    break;
                }

                if (visitedReverse[neighbor] == 0)
                {
                    visitedReverse[neighbor] = 1;
                    queueReverse.Push(neighbor);
                }
            }
        }

        if (searchQHeadReverse >= queueReverse.Size() && !searchBarrierFound)
        {
            return 0;
        }

        if (!searchBarrierFound)
        {
            return -1;
        }

        int numLines = level.lines.Size();
        while (searchLi < numLines && iterations < limit)
        {
            iterations++;
            Line ln = level.lines[searchLi];
            searchLi++;

            if (!ln || !IsPlayerTriggerable(ln)) continue;

            int targetTag = ln.args[0];
            if (targetTag == 0) continue;

            bool hasTag = false;
            SectorTagIterator it = level.CreateSectorTagIterator(targetTag);
            int secNum;
            while ((secNum = it.Next()) >= 0)
            {
                if (secNum == searchBarrierSec.sectornum)
                {
                    hasTag = true;
                    break;
                }
            }
            if (!hasTag) continue;

            int switchSecIdx = -1;
            if (ln.frontsector && reachable[ln.frontsector.sectornum] == 1)
            {
                switchSecIdx = ln.frontsector.sectornum;
            }
            else if (ln.backsector && reachable[ln.backsector.sectornum] == 1)
            {
                switchSecIdx = ln.backsector.sectornum;
            }

            if (switchSecIdx != -1)
            {
                int steps = 0;
                int cur = switchSecIdx;
                while (cur != searchStartIdx && parentLine[cur] >= 0)
                {
                    steps++;
                    cur = parent[cur];
                }
                if (steps < searchMinPathSteps)
                {
                    searchMinPathSteps = steps;
                    searchBestSwitchSector = switchSecIdx;
                    searchBestSwitchLine = ln;
                }
            }
        }

        if (searchLi < numLines)
        {
            return -1;
        }

        if (searchBestSwitchLine && searchBestSwitchSector != -1)
        {
            return 1;
        }

        return 0;
    }

    void StartAsyncSearch(Actor player, Actor target)
    {
        if (graphBuilt == 0 || !player || !target)
        {
            searchState = ASYNC_STATE_IDLE;
            return;
        }

        int numSectors = level.sectors.Size();
        Sector startSec = level.PointInSector(player.pos.xy);
        Sector endSec = level.PointInSector(target.pos.xy);

        if (!startSec || !endSec)
        {
            searchState = ASYNC_STATE_IDLE;
            return;
        }

        int startIdx = startSec.sectornum;
        int endIdx = endSec.sectornum;

        if (startIdx == endIdx)
        {
            pathX.Clear();
            pathY.Clear();
            pathX.Push(target.pos.x);
            pathY.Push(target.pos.y);
            searchState = ASYNC_STATE_IDLE;
            return;
        }

        parent.Resize(numSectors);
        parentLine.Resize(numSectors);
        reachable.Resize(numSectors);
        gScore.Resize(numSectors);
        fScore.Resize(numSectors);
        inOpenSet.Resize(numSectors);

        searchPlayer = player;
        searchTarget = target;
        searchTargetPos = target.pos.xy;
        searchStartIdx = startIdx;
        searchEndIdx = endIdx;
        searchRealEndIdx = endIdx;
        searchAlternativeEndIdx = -1;

        InitAStarAsync(startIdx, endIdx, target.pos.xy, player);
        searchState = ASYNC_STATE_ASTAR_DIRECT;
    }

    void TickAsyncSearch()
    {
        int limit = 150;
        
        if (searchState == ASYNC_STATE_ASTAR_DIRECT)
        {
            int res = ResumeAStarAsync(limit);
            if (res == 1)
            {
                BuildPathFromParent(searchStartIdx, searchRealEndIdx, searchTarget);
                searchState = ASYNC_STATE_IDLE;
            }
            else if (res == 0)
            {
                int altIdx = -1;
                double closestDist = SCORE_INFINITY;

                for (int i = 0; i < level.lines.Size(); i++)
                {
                    Line ln = level.lines[i];
                    if (!ln || !ln.frontsector || !ln.backsector) continue;

                    int fNum = ln.frontsector.sectornum;
                    int bNum = ln.backsector.sectornum;

                    if (fNum == searchEndIdx && bNum != searchEndIdx)
                    {
                        Vector2 mid = (ln.v1.p + ln.v2.p) / 2.0;
                        double d = (mid - searchTarget.pos.xy).Length();
                        if (d < closestDist) { closestDist = d; altIdx = bNum; }
                    }
                    else if (bNum == searchEndIdx && fNum != searchEndIdx)
                    {
                        Vector2 mid = (ln.v1.p + ln.v2.p) / 2.0;
                        double d = (mid - searchTarget.pos.xy).Length();
                        if (d < closestDist) { closestDist = d; altIdx = fNum; }
                    }
                }

                if (altIdx != -1 && altIdx != searchStartIdx)
                {
                    searchAlternativeEndIdx = altIdx;
                    InitAStarAsync(searchStartIdx, altIdx, searchTargetPos, searchPlayer);
                    searchState = ASYNC_STATE_ASTAR_PEDESTAL;
                }
                else
                {
                    InitReverseBFSAsync();
                    searchState = ASYNC_STATE_REVERSE_BFS;
                }
            }
        }
        else if (searchState == ASYNC_STATE_ASTAR_PEDESTAL)
        {
            int res = ResumeAStarAsync(limit);
            if (res == 1)
            {
                searchRealEndIdx = searchAlternativeEndIdx;
                BuildPathFromParent(searchStartIdx, searchRealEndIdx, searchTarget);
                searchState = ASYNC_STATE_IDLE;
            }
            else if (res == 0)
            {
                InitReverseBFSAsync();
                searchState = ASYNC_STATE_REVERSE_BFS;
            }
        }
        else if (searchState == ASYNC_STATE_REVERSE_BFS)
        {
            int res = ResumeReverseBFSAsync(limit);
            if (res == 1)
            {
                InitAStarAsync(searchStartIdx, searchBestSwitchSector, searchTargetPos, searchPlayer);
                searchState = ASYNC_STATE_ASTAR_SWITCH;
            }
            else if (res == 0)
            {
                searchState = ASYNC_STATE_IDLE;
            }
        }
        else if (searchState == ASYNC_STATE_ASTAR_SWITCH)
        {
            int res = ResumeAStarAsync(limit);
            if (res == 1)
            {
                BuildPathFromParent(searchStartIdx, searchBestSwitchSector, null);
                Vector2 switchMid = (searchBestSwitchLine.v1.p + searchBestSwitchLine.v2.p) / 2.0;
                pathX.Push(switchMid.x);
                pathY.Push(switchMid.y);
                searchState = ASYNC_STATE_IDLE;
            }
            else if (res == 0)
            {
                searchState = ASYNC_STATE_IDLE;
            }
        }
    }

    // Compute effective walkable floor and ceiling at a point, accounting for solid 3D floors.
    // Finds the highest solid floor at or below the sector ceiling, and the lowest ceiling above it.
    void GetEffectiveFloorCeil(Sector sec, Vector2 pos, out double floorZ, out double ceilZ)
    {
        floorZ = sec.floorplane.ZatPoint(pos);
        ceilZ = sec.ceilingplane.ZatPoint(pos);

        int count = sec.Get3DFloorCount();
        for (int i = 0; i < count; i++)
        {
            F3DFloor f3d = sec.Get3DFloor(i);
            if (!f3d || !(f3d.flags & F3DFloor.FF_EXISTS) || !(f3d.flags & F3DFloor.FF_SOLID))
                continue;

            double fTop = f3d.top.ZatPoint(pos);
            double fBot = f3d.bottom.ZatPoint(pos);

            // If this 3D floor's top is above the current effective floor
            // and below the ceiling, it raises the walkable floor
            if (fTop > floorZ && fTop < ceilZ)
            {
                floorZ = fTop;
            }
            // If this 3D floor's bottom is below the current effective ceiling
            // and above the floor, it lowers the head clearance
            if (fBot < ceilZ && fBot > floorZ)
            {
                ceilZ = fBot;
            }
        }
    }

    double GetFloorZ(Vector2 pos)
    {
        Sector sec = level.PointInSector(pos);
        if (!sec) return 0;
        double f, c;
        GetEffectiveFloorCeil(sec, pos, f, c);
        return f;
    }

    bool PathIsBlocked(Vector2 start, Vector2 end)
    {
        if (!mTracer) return false;

        double floorzA = GetFloorZ(start);
        double floorzB = GetFloorZ(end);

        Vector3 pA = (start.x, start.y, floorzA + TRACE_Z_OFFSET);
        Vector3 pB = (end.x, end.y, floorzB + TRACE_Z_OFFSET);

        Vector3 diff = pB - pA;
        double len = diff.Length();
        if (len < 1.0) return false;
        Vector3 dir = diff / len;

        Sector sec = level.PointInSector(start);
        if (!sec) return false;

        mTracer.Trace(pA, sec, dir, len, TRACE_ReportPortals, 0xFFFFFFFF, true);
        if (mTracer.Results.HitType != TRACE_HitNone) return true;

        // Validate midpoint: catch traces that clip through thin walls
        if (len > 64.0)
        {
            Vector2 mid = (start + end) * 0.5;
            Sector midSec = level.PointInSector(mid);
            if (midSec)
            {
                double midFloor = midSec.floorplane.ZatPoint(mid);
                double midCeil = midSec.ceilingplane.ZatPoint(mid);
                // If midpoint has no clearance or a massive floor jump, it's blocked
                if (midCeil - midFloor < CLEARANCE_MIN) return true;
                if (abs(midFloor - floorzA) > STEP_HEIGHT_MAX && abs(midFloor - floorzB) > STEP_HEIGHT_MAX) return true;
            }
        }

        return false;
    }

    double GetPathDist(Vector2 start, in out Array<double> px, in out Array<double> py, Vector2 end)
    {
        double d = 0;
        Vector2 prev = start;
        for (int i = 0; i < px.Size(); i++)
        {
            Vector2 cur = (px[i], py[i]);
            d += (cur - prev).Length();
            prev = cur;
        }
        d += (end - prev).Length();
        return d;
    }

    void AppendArray(in out Array<double> dest, in out Array<double> src)
    {
        for (int i = 0; i < src.Size(); i++)
        {
            dest.Push(src[i]);
        }
    }

    bool GetDetourPath(Vector2 A, Vector2 B, in out Array<double> outX, in out Array<double> outY, int depth = 0)
    {
        if (depth > 3 || !mTracer) return false;

        double floorzA = GetFloorZ(A);
        double floorzB = GetFloorZ(B);

        Vector3 start = (A.x, A.y, floorzA + TRACE_Z_OFFSET);
        Vector3 end = (B.x, B.y, floorzB + TRACE_Z_OFFSET);

        Vector3 diff = end - start;
        double len = diff.Length();
        if (len < 1.0) return true;
        Vector3 dir = diff / len;

        Sector sec = level.PointInSector(A);
        if (!sec) return false;

        mTracer.Trace(start, sec, dir, len, TRACE_ReportPortals, 0xFFFFFFFF, true);
        if (mTracer.Results.HitType == TRACE_HitNone) return true;

        Line hitLn = mTracer.Results.HitLine;
        if (!hitLn) return false;

        Vector2 lnDir = (hitLn.v2.p - hitLn.v1.p).Unit();
        Vector2 perp = (-lnDir.y, lnDir.x);
        
        if (((A - hitLn.v1.p) dot perp) < 0)
        {
            perp = -perp;
        }

        Vector2 d1 = hitLn.v1.p - lnDir * DETOUR_WALL_MARGIN + perp * DETOUR_PERP_MARGIN;
        Vector2 d2 = hitLn.v2.p + lnDir * DETOUR_WALL_MARGIN + perp * DETOUR_PERP_MARGIN;

        Array<double> path1X;
        Array<double> path1Y;
        bool ok1 = GetDetourPath(A, d1, path1X, path1Y, depth + 1);
        if (ok1)
        {
            path1X.Push(d1.x);
            path1Y.Push(d1.y);
            ok1 = GetDetourPath(d1, B, path1X, path1Y, depth + 1);
        }

        Array<double> path2X;
        Array<double> path2Y;
        bool ok2 = GetDetourPath(A, d2, path2X, path2Y, depth + 1);
        if (ok2)
        {
            path2X.Push(d2.x);
            path2Y.Push(d2.y);
            ok2 = GetDetourPath(d2, B, path2X, path2Y, depth + 1);
        }

        if (ok1 && ok2)
        {
            double len1 = GetPathDist(A, path1X, path1Y, B);
            double len2 = GetPathDist(A, path2X, path2Y, B);
            if (len1 <= len2)
            {
                AppendArray(outX, path1X);
                AppendArray(outY, path1Y);
            }
            else
            {
                AppendArray(outX, path2X);
                AppendArray(outY, path2Y);
            }
            return true;
        }
        else if (ok1)
        {
            AppendArray(outX, path1X);
            AppendArray(outY, path1Y);
            return true;
        }
        else if (ok2)
        {
            AppendArray(outX, path2X);
            AppendArray(outY, path2Y);
            return true;
        }

        return false;
    }

    void RefinePath(Vector2 start)
    {
        refinedX.Clear();
        refinedY.Clear();

        if (pathX.Size() == 0) return;

        Vector2 prev = start;
        for (int i = 0; i < pathX.Size(); i++)
        {
            Vector2 wp = (pathX[i], pathY[i]);
            
            Array<double> detX;
            Array<double> detY;
            bool ok = GetDetourPath(prev, wp, detX, detY, 0);
            if (ok)
            {
                for (int j = 0; j < detX.Size(); j++)
                {
                    refinedX.Push(detX[j]);
                    refinedY.Push(detY[j]);
                }
            }
            refinedX.Push(wp.x);
            refinedY.Push(wp.y);
            prev = wp;
        }
    }

    void SpawnPathMarkers(Actor player)
    {
        if (refinedX.Size() == 0 || !player) return;

        if (cache_alpha <= 0.0) cache_alpha = 0.6;
        if (cache_spacing < 32) cache_spacing = 32;
        if (cache_max_markers < 1) cache_max_markers = 1;

        Vector2 prevPoint = player.pos.xy;
        Vector2 lastSpawnPos = player.pos.xy;
        int spawnedCount = 0;

        for (int wp = 0; wp < refinedX.Size(); wp++)
        {
            if (spawnedCount >= cache_max_markers) break;
            
            Vector2 waypoint = (refinedX[wp], refinedY[wp]);
            Vector2 seg = waypoint - prevPoint;
            double segLen = seg.Length();

            if (segLen < 16)
            {
                prevPoint = waypoint;
                continue;
            }

            int segSteps = int(segLen / cache_spacing);
            if (segSteps < 1) segSteps = 1;

            Vector2 stepDir = seg / segLen;
            double arrowAngle = VectorAngle(seg.x, seg.y);

            for (int i = 1; i <= segSteps; i++)
            {
                if (spawnedCount >= cache_max_markers) break;
                
                Vector2 spawnXY = prevPoint + stepDir * (cache_spacing * i);

                if ((spawnXY - prevPoint).Length() > segLen)
                    spawnXY = waypoint;

                if (PathIsBlocked(lastSpawnPos, spawnXY))
                {
                    lastSpawnPos = spawnXY; 
                    continue; 
                }

                Sector spawnSec = level.PointInSector(spawnXY);
                if (!spawnSec) continue; 

                double floorz = spawnSec.floorplane.ZatPoint(spawnXY);
                Vector3 spawnPos = (spawnXY.x, spawnXY.y, floorz + cache_height);

                Actor marker;
                if (spawnedCount < activeMarkers.Size() && activeMarkers[spawnedCount] && !activeMarkers[spawnedCount].bDestroyed)
                {
                    // Reuse existing pooled marker
                    marker = activeMarkers[spawnedCount];
                    marker.SetOrigin(spawnPos, false);
                }
                else
                {
                    // Pool is exhausted — spawn a new one
                    marker = Actor.Spawn("HoloPathMarker", spawnPos);
                    if (marker)
                    {
                        if (spawnedCount < activeMarkers.Size())
                            activeMarkers[spawnedCount] = marker;
                        else
                            activeMarkers.Push(marker);
                    }
                }

                if (marker)
                {
                    if (cache_scale <= 0.0) cache_scale = 0.25;
                    marker.Scale = (cache_scale, cache_scale);

                    double markerAlpha = cache_alpha;
                    if (cache_fade)
                    {
                        markerAlpha = cache_alpha * (1.0 - (double(spawnedCount) / double(cache_max_markers)));
                    }

                    marker.angle = arrowAngle;

                    int renderStyle = (cache_style == 1) ? STYLE_TranslucentStencil : STYLE_AddStencil;
                    int defaultStyle = (cache_style == 1) ? STYLE_Translucent : STYLE_Add;

                    if (cache_color == 9) 
                    {
                        marker.A_SetRenderStyle(markerAlpha, renderStyle);
                        marker.SetShade(TRANS_COLORS[spawnedCount % 4]);
                    }
                    else if (cache_color >= 1 && cache_color <= 8)
                    {
                        marker.A_SetRenderStyle(markerAlpha, renderStyle);
                        marker.SetShade(COLOR_TABLE[cache_color]);
                    }
                    else
                    {
                        marker.A_SetRenderStyle(markerAlpha, defaultStyle);
                    }

                    spawnedCount++;
                    lastSpawnPos = spawnXY;
                }
            }
            if (spawnedCount >= cache_max_markers) break;
            prevPoint = waypoint;
        }

        // Destroy excess pooled markers beyond what we used this frame
        for (int i = activeMarkers.Size() - 1; i >= spawnedCount; i--)
        {
            if (activeMarkers[i] && !activeMarkers[i].bDestroyed)
            {
                activeMarkers[i].Destroy();
            }
            activeMarkers.Delete(i);
        }
    }
}