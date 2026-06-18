class PathfinderTracer : LineTracer
{
    override ETraceStatus TraceCallback()
    {
        if (Results.HitType == TRACE_HitWall) return TRACE_Stop;
        if (Results.HitType == TRACE_HitFloor || Results.HitType == TRACE_HitCeiling) return TRACE_Stop;
        if (Results.HitType == TRACE_HitActor) return TRACE_Skip; // Ignore actors

        if (Results.HitType == TRACE_CrossingPortal)
        {
            if (Results.HitLine)
            {
                if (Results.HitLine.flags & Line.ML_BLOCKING) 
                    return TRACE_Stop;

                Sector front = Results.HitLine.frontsector;
                Sector back = Results.HitLine.backsector;
                if (front && back)
                {
                    Vector2 hitPt = Results.HitPos.xy;
                    double fFloor = front.floorplane.ZatPoint(hitPt);
                    double fCeil = front.ceilingplane.ZatPoint(hitPt);
                    double bFloor = back.floorplane.ZatPoint(hitPt);
                    double bCeil = back.ceilingplane.ZatPoint(hitPt);

                    if ((fCeil - fFloor < HoloGPSHandler.CLEARANCE_MIN) || (bCeil - bFloor < HoloGPSHandler.CLEARANCE_MIN))
                    {
                        return TRACE_Stop;
                    }

                    if ((fFloor - bFloor > HoloGPSHandler.STEP_HEIGHT_MAX) || (bFloor - fFloor > HoloGPSHandler.STEP_HEIGHT_MAX))
                    {
                        return TRACE_Stop;
                    }
                }
            }
            return TRACE_Skip;
        }

        return TRACE_Stop;
    }
}

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
            if (!ln || !ln.frontsector || !ln.backsector) continue;
            
            if (ln.flags & Line.ML_BLOCKING) continue;

            int fi = ln.frontsector.sectornum;
            int bi = ln.backsector.sectornum;
            if (fi == bi) continue;

            neighborCount[fi]++;
            neighborCount[bi]++;
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
            if (!ln || !ln.frontsector || !ln.backsector) continue;
            
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
            FindNextObjective();
        }

        if (currentTarget)
        {
            ClearOldMarkers();
            FindPathAStar(plyr.mo, currentTarget);
            RefinePath(plyr.mo.pos.xy);
            SpawnPathMarkers(plyr.mo);
        }
    }

    void ClearOldMarkers()
    {
        for (int i = 0; i < activeMarkers.Size(); i++)
        {
            if (activeMarkers[i] && !activeMarkers[i].bDestroyed)
            {
                activeMarkers[i].Destroy();
            }
        }
        activeMarkers.Clear();
    }

    void FindNextObjective()
    {
        PlayerInfo plyr = players[consoleplayer];
        if (!plyr || !plyr.mo) return;

        // Pass 1: Find valid, reachable Keys
        if (cache_priority == 0 || cache_priority == 1)
        {
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
                    FindPathAStar(plyr.mo, mapKey);
                    if (pathX.Size() > 0)
                    {
                        currentTarget = mapKey;
                        return;
                    }
                }
            }
        }

        // Pass 2 Fallback: Target level exits
        if (cache_priority == 0 || cache_priority == 2)
        {
            Array<Actor> exitSpots;
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
                    if (spot) exitSpots.Push(spot);
                }
            }

            int foundIdx = -1;
            for (int i = 0; i < exitSpots.Size(); i++)
            {
                FindPathAStar(plyr.mo, exitSpots[i]);
                if (pathX.Size() > 0)
                {
                    currentTarget = exitSpots[i];
                    foundIdx = i;
                    break;
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
        double curFloor = curSec.floorplane.ZatPoint(midpoint);
        double curCeil = curSec.ceilingplane.ZatPoint(midpoint);
        double nextFloor = nextSec.floorplane.ZatPoint(midpoint);
        double nextCeil = nextSec.ceilingplane.ZatPoint(midpoint);

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

    // Reusable A* core: resets arrays, seeds startIdx, and expands until goalIdx is found.
    // Returns 1 if goal reached, 0 otherwise. Populates parent/parentLine/reachable/gScore/fScore.
    int RunAStar(int startIdx, int goalIdx, Vector2 targetPos, Actor player)
    {
        int numSectors = level.sectors.Size();
        openSet.Clear();

        for (int i = 0; i < numSectors; i++)
        {
            parent[i] = -1;
            parentLine[i] = -1;
            reachable[i] = 0;
            gScore[i] = SCORE_INFINITY;
            fScore[i] = SCORE_INFINITY;
            inOpenSet[i] = false;
        }

        gScore[startIdx] = 0.0;
        fScore[startIdx] = (level.sectors[startIdx].centerspot - targetPos).Length();
        reachable[startIdx] = 1;
        openSet.Push(startIdx);
        inOpenSet[startIdx] = true;

        while (openSet.Size() > 0)
        {
            int bestIdx = 0;
            double minF = fScore[openSet[0]];
            for (int i = 1; i < openSet.Size(); i++)
            {
                int node = openSet[i];
                if (fScore[node] < minF) { minF = fScore[node]; bestIdx = i; }
            }

            int current = openSet[bestIdx];
            if (current == goalIdx) return 1;

            openSet.Delete(bestIdx);
            inOpenSet[current] = false;

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
                    double dist = (nextSec.centerspot - curSec.centerspot).Length();
                    double tentativeG = gScore[current] + dist;

                    if (tentativeG < gScore[neighbor])
                    {
                        reachable[neighbor] = 1;
                        parent[neighbor] = current;
                        parentLine[neighbor] = adjLineIdx[ni];
                        gScore[neighbor] = tentativeG;
                        fScore[neighbor] = tentativeG + (nextSec.centerspot - targetPos).Length();

                        if (!inOpenSet[neighbor])
                        {
                            openSet.Push(neighbor);
                            inOpenSet[neighbor] = true;
                        }
                    }
                }
            }
        }

        return 0;
    }

    void FindPathAStar(Actor player, Actor target)
    {
        pathX.Clear();
        pathY.Clear();

        if (graphBuilt == 0 || !player || !target) return;

        int numSectors = level.sectors.Size();

        Sector startSec = level.PointInSector(player.pos.xy);
        Sector endSec = level.PointInSector(target.pos.xy);

        if (!startSec || !endSec) return;

        int startIdx = startSec.sectornum;
        int endIdx = endSec.sectornum;
        int realEndIdx = endIdx;

        if (startIdx == realEndIdx)
        {
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

        Vector2 targetPos = target.pos.xy;
        int found = RunAStar(startIdx, realEndIdx, targetPos, player);

        // Adaptive Pedestal Retry Pass
        if (found == 0)
        {
            int alternativeEndIdx = -1;
            double closestDist = SCORE_INFINITY;

            for (int i = 0; i < level.lines.Size(); i++)
            {
                Line ln = level.lines[i];
                if (!ln || !ln.frontsector || !ln.backsector) continue;

                int fNum = ln.frontsector.sectornum;
                int bNum = ln.backsector.sectornum;

                if (fNum == endIdx && bNum != endIdx)
                {
                    Vector2 mid = (ln.v1.p + ln.v2.p) / 2.0;
                    double d = (mid - target.pos.xy).Length();
                    if (d < closestDist) { closestDist = d; alternativeEndIdx = bNum; }
                }
                else if (bNum == endIdx && fNum != endIdx)
                {
                    Vector2 mid = (ln.v1.p + ln.v2.p) / 2.0;
                    double d = (mid - target.pos.xy).Length();
                    if (d < closestDist) { closestDist = d; alternativeEndIdx = fNum; }
                }
            }

            if (alternativeEndIdx != -1 && alternativeEndIdx != startIdx)
            {
                found = RunAStar(startIdx, alternativeEndIdx, targetPos, player);
                if (found == 1) realEndIdx = alternativeEndIdx;
            }
        }

        if (found == 1)
        {
            Array<int> reversedLines;
            int cur = realEndIdx;
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

            pathX.Push(target.pos.x);
            pathY.Push(target.pos.y);
            return;
        }

        // Pass 3: Reverse topological fallback finding the barrier sector
        visitedReverse.Resize(numSectors);
        for (int i = 0; i < numSectors; i++) visitedReverse[i] = 0;

        queueReverse.Clear();
        visitedReverse[endIdx] = 1;
        queueReverse.Push(endIdx);

        int qHeadReverse = 0;
        Sector barrierSec = null;
        bool barrierFound = false;

        while (qHeadReverse < queueReverse.Size())
        {
            int current = queueReverse[qHeadReverse];
            qHeadReverse++;

            int nStart = adjStart[current];
            int nEnd = adjStart[current + 1];
            for (int ni = nStart; ni < nEnd; ni++)
            {
                int neighbor = adjNeighbor[ni];

                if (reachable[neighbor] == 1)
                {
                    barrierSec = level.sectors[current];
                    barrierFound = true;
                    break;
                }

                if (visitedReverse[neighbor] == 0)
                {
                    visitedReverse[neighbor] = 1;
                    queueReverse.Push(neighbor);
                }
            }
            if (barrierFound) break;
        }

        if (!barrierFound || !barrierSec) return;

        int bestSwitchSector = -1;
        Line bestSwitchLine = null;
        int minPathSteps = int.MAX;

        for (int li = 0; li < level.lines.Size(); li++)
        {
            Line ln = level.lines[li];
            if (!ln || !IsPlayerTriggerable(ln)) continue;

            int targetTag = ln.args[0];
            if (targetTag == 0) continue;

            bool hasTag = false;
            SectorTagIterator it = level.CreateSectorTagIterator(targetTag);
            int secNum;
            while ((secNum = it.Next()) >= 0)
            {
                if (secNum == barrierSec.sectornum)
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
                while (cur != startIdx && parentLine[cur] >= 0)
                {
                    steps++;
                    cur = parent[cur];
                }
                if (steps < minPathSteps)
                {
                    minPathSteps = steps;
                    bestSwitchSector = switchSecIdx;
                    bestSwitchLine = ln;
                }
            }
        }

        if (bestSwitchLine && bestSwitchSector != -1)
        {
            Array<int> reversedLines;
            int cur = bestSwitchSector;
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

            Vector2 switchMid = (bestSwitchLine.v1.p + bestSwitchLine.v2.p) / 2.0;
            pathX.Push(switchMid.x);
            pathY.Push(switchMid.y);
        }
    }

    double GetFloorZ(Vector2 pos)
    {
        Sector sec = level.PointInSector(pos);
        if (sec) return sec.floorplane.ZatPoint(pos);
        return 0;
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
        return mTracer.Results.HitType != TRACE_HitNone;
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

                Actor marker = Actor.Spawn("HoloPathMarker", spawnPos);
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

                    activeMarkers.Push(marker);
                    spawnedCount++;
                    lastSpawnPos = spawnXY;
                }
            }
            if (spawnedCount >= cache_max_markers) break;
            prevPoint = waypoint;
        }
    }
}