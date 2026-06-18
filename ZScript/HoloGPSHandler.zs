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

                    if ((fCeil - fFloor < 56.0) || (bCeil - bFloor < 56.0))
                    {
                        return TRACE_Stop;
                    }

                    if ((fFloor - bFloor > 24.0) || (bFloor - fFloor > 24.0))
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
    Actor currentTarget;
    int tickCounter;
    Array<Actor> activeMarkers;

    // Consolidated reusable tracer instance
    PathfinderTracer mTracer;

    // Dynamically cached CVars to prevent continuous string lookups
    bool cache_enabled;
    int cache_freq;
    bool cache_use_secrets;
    float cache_alpha;
    int cache_spacing;
    int cache_max_markers;
    bool cache_fade;
    int cache_color;

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
        
        // Allocate single tracer instance for reuse
        mTracer = new("PathfinderTracer");

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

    override void WorldTick()
    {
        PlayerInfo plyr = players[consoleplayer];
        if (!plyr || !plyr.mo || plyr.mo.health <= 0) return;

        // Populate dynamic CVar cache once per frame tick
        cache_enabled = CVar.GetCVar("holo_gps_enabled", plyr).GetBool();
        cache_freq = CVar.GetCVar("holo_gps_freq", plyr).GetInt();
        cache_use_secrets = CVar.GetCVar("holo_gps_use_secrets", plyr).GetBool();
        cache_alpha = CVar.GetCVar("holo_gps_alpha", plyr).GetFloat();
        cache_spacing = CVar.GetCVar("holo_gps_spacing", plyr).GetInt();
        cache_max_markers = CVar.GetCVar("holo_gps_max_markers", plyr).GetInt();
        cache_fade = CVar.GetCVar("holo_gps_fade", plyr).GetBool();
        cache_color = CVar.GetCVar("holo_gps_color", plyr).GetInt();

        if (!cache_enabled) return;

        tickCounter++;
        if (cache_freq < 1) cache_freq = 1;
        if (tickCounter % cache_freq != 0) return;

        if (currentTarget)
        {
            if (currentTarget is "Key")
            {
                if (Key(currentTarget).owner || plyr.mo.FindInventory(currentTarget.GetClassName()))
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
        ThinkerIterator it = ThinkerIterator.Create("Key");
        Key mapKey;
        while (mapKey = Key(it.Next()))
        {
            if (!mapKey.owner && !plyr.mo.FindInventory(mapKey.GetClass()))
            {
                FindPathAStar(plyr.mo, mapKey);
                if (pathX.Size() > 0)
                {
                    currentTarget = mapKey;
                    return;
                }
            }
        }

        // Pass 2 Fallback: If keys are unreachable, target level exits
        Array<Actor> exitSpots;
        for (int i = 0; i < level.lines.Size(); i++)
        {
            Line ln = level.lines[i];
            if (ln && (ln.special == 243 || ln.special == 244 || ln.special == 74 || ln.special == 124))
            {
                Vector2 midpoint = (ln.v1.p + ln.v2.p) / 2.0;
                Sector exitSec = level.PointInSector(midpoint);
                if (!exitSec) continue; // Boundary check prevention

                double z = exitSec.floorplane.ZatPoint(midpoint);
                Actor spot = Actor.Spawn("MapSpot", (midpoint.x, midpoint.y, z));
                if (spot) exitSpots.Push(spot);
            }
        }

        for (int i = 0; i < exitSpots.Size(); i++)
        {
            FindPathAStar(plyr.mo, exitSpots[i]);
            if (pathX.Size() > 0)
            {
                currentTarget = exitSpots[i];
                
                for (int j = 0; j < exitSpots.Size(); j++)
                {
                    if (i != j && exitSpots[j])
                    {
                        exitSpots[j].Destroy();
                    }
                }
                return;
            }
            else
            {
                exitSpots[i].Destroy(); 
            }
        }

        currentTarget = null;
    }

    bool IsPortalPassable(Sector curSec, Sector nextSec, Line ln, Actor player)
    {
        if (!ln || !curSec || !nextSec) return false;

        if (ln.flags & Line.ML_BLOCKING) return false;
        if (!cache_use_secrets && (ln.flags & Line.ML_SECRET)) return false;

        Vector2 midpoint = (ln.v1.p + ln.v2.p) / 2.0;
        double curFloor = curSec.floorplane.ZatPoint(midpoint);
        double curCeil = curSec.ceilingplane.ZatPoint(midpoint);
        double nextFloor = nextSec.floorplane.ZatPoint(midpoint);
        double nextCeil = nextSec.ceilingplane.ZatPoint(midpoint);

        if (nextFloor - curFloor > 24.0) return false;

        double portalFloor = (curFloor > nextFloor) ? curFloor : nextFloor;
        double portalCeil = (curCeil < nextCeil) ? curCeil : nextCeil;
        double clearance = portalCeil - portalFloor;

        if (clearance < 56.0)
        {
            bool isDirectUse = (ln.activation & (SPAC_Use | SPAC_UseThrough)) != 0;
            bool isStandardDoor = (ln.special >= 10 && ln.special <= 13) || ln.special == 105 || ln.special == 106 || ln.special == 202;
            bool isScriptDoor = (ln.special == 80 || ln.special == 226); 

            if (isDirectUse || isStandardDoor || isScriptDoor)
            {
                if (ln.locknumber != 0)
                {
                    PlayerPawn plyrPawn = PlayerPawn(player);
                    if (plyrPawn && !plyrPawn.CheckKeys(ln.locknumber, false))
                    {
                        return false; 
                    }
                }
            }
            else
            {
                return false;
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

    void FindPathAStar(Actor player, Actor target)
    {
        pathX.Clear();
        pathY.Clear();

        if (graphBuilt == 0 || !player || !target) return;

        int numSectors = level.sectors.Size();

        Sector startSec = level.PointInSector(player.pos.xy);
        Sector endSec = level.PointInSector(target.pos.xy);

        // Boundary Check Prevention
        if (!startSec || !endSec) return;

        int startIdx = startSec.sectornum;
        int endIdx = endSec.sectornum;

        // Localized Structural Proximity Targeting For Shared/Isolated Sectors
        int realEndIdx = endIdx;
        if (adjStart[realEndIdx] == adjStart[realEndIdx + 1])
        {
            double closestDist = 1e37;
            for (int i = 0; i < level.lines.Size(); i++)
            {
                Line ln = level.lines[i];
                if (!ln || (!ln.frontsector && !ln.backsector)) continue;
                
                Vector2 lineMid = (ln.v1.p + ln.v2.p) / 2.0;
                double d = (lineMid - target.pos.xy).Length(); 

                if (ln.frontsector && ln.frontsector.sectornum == endIdx && ln.backsector)
                {
                    int bIdx = ln.backsector.sectornum;
                    if (adjStart[bIdx] != adjStart[bIdx + 1]) 
                    {
                        if (d < closestDist) { closestDist = d; realEndIdx = bIdx; }
                    }
                }
                else if (ln.backsector && ln.backsector.sectornum == endIdx && ln.frontsector)
                {
                    int fIdx = ln.frontsector.sectornum;
                    if (adjStart[fIdx] != adjStart[fIdx + 1])
                    {
                        if (d < closestDist) { closestDist = d; realEndIdx = fIdx; }
                    }
                }
            }
        }

        if (startIdx == realEndIdx)
        {
            pathX.Push(target.pos.x);
            pathY.Push(target.pos.y);
            return;
        }

        Array<int> parent;
        Array<int> parentLine;
        Array<int> reachable;
        Array<double> gScore;
        Array<double> fScore;
        Array<int> openSet;
        Array<bool> inOpenSet;

        parent.Resize(numSectors);
        parentLine.Resize(numSectors);
        reachable.Resize(numSectors);
        gScore.Resize(numSectors);
        fScore.Resize(numSectors);
        inOpenSet.Resize(numSectors);

        for (int i = 0; i < numSectors; i++)
        {
            parent[i] = -1;
            parentLine[i] = -1;
            reachable[i] = 0;
            gScore[i] = 1e37; 
            fScore[i] = 1e37;
            inOpenSet[i] = false;
        }

        Vector2 targetPos = target.pos.xy;
        gScore[startIdx] = 0.0;
        fScore[startIdx] = (level.sectors[startIdx].centerspot - targetPos).Length();
        
        reachable[startIdx] = 1;
        openSet.Push(startIdx);
        inOpenSet[startIdx] = true;

        int found = 0;

        while (openSet.Size() > 0)
        {
            int bestIdx = 0;
            double minF = fScore[openSet[0]];
            for (int i = 1; i < openSet.Size(); i++)
            {
                int node = openSet[i];
                if (fScore[node] < minF)
                {
                    minF = fScore[node];
                    bestIdx = i;
                }
            }

            int current = openSet[bestIdx];
            
            if (current == realEndIdx)
            {
                found = 1;
                break;
            }

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

        // Pass 2 Fallback: Reverse tracking setup for structural blockers
        Array<int> visitedReverse;
        Array<int> queueReverse;

        visitedReverse.Resize(numSectors);
        for (int i = 0; i < numSectors; i++) visitedReverse[i] = 0;

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
        int minPathSteps = 999999;

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
        // Reused instance verification
        if (!mTracer) return false;

        double floorzA = GetFloorZ(start);
        double floorzB = GetFloorZ(end);

        Vector3 pA = (start.x, start.y, floorzA + 16.0);
        Vector3 pB = (end.x, end.y, floorzB + 16.0);

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

        Vector3 start = (A.x, A.y, floorzA + 16.0);
        Vector3 end = (B.x, B.y, floorzB + 16.0);

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

        Vector2 d1 = hitLn.v1.p - lnDir * 32.0 + perp * 24.0;
        Vector2 d2 = hitLn.v2.p + lnDir * 32.0 + perp * 24.0;

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
                if (!spawnSec) continue; // Boundary check prevention

                double floorz = spawnSec.floorplane.ZatPoint(spawnXY);
                Vector3 spawnPos = (spawnXY.x, spawnXY.y, floorz + 1.0);

                Actor marker = Actor.Spawn("HoloPathMarker", spawnPos);
                if (marker)
                {
                    double markerAlpha = cache_alpha;
                    if (cache_fade)
                    {
                        markerAlpha = cache_alpha * (1.0 - (double(spawnedCount) / double(cache_max_markers)));
                    }

                    marker.angle = arrowAngle;

                    if (cache_color == 9) // Trans Pride
                    {
                        int cycleStep = spawnedCount % 4;
                        color transCol = 0xFFFFFF; 
                        if (cycleStep == 0) transCol = 0x5BCEFA; 
                        else if (cycleStep == 1 || cycleStep == 3) transCol = 0xF5A9B8; 
                        
                        marker.A_SetRenderStyle(markerAlpha, STYLE_AddStencil);
                        marker.SetShade(transCol);
                    }
                    else if (cache_color > 0)
                    {
                        color col = 0xFFFFFF;
                        if (cache_color == 1) col = 0xFF0000;      
                        else if (cache_color == 2) col = 0x00FF00; 
                        else if (cache_color == 3) col = 0x0000FF; 
                        else if (cache_color == 4) col = 0xFFFF00; 
                        else if (cache_color == 5) col = 0xFF8000; 
                        else if (cache_color == 6) col = 0x8000FF; 
                        else if (cache_color == 7) col = 0xFF00FF; 
                        else if (cache_color == 8) col = 0xFFFFFF; 
                        
                        marker.A_SetRenderStyle(markerAlpha, STYLE_AddStencil);
                        marker.SetShade(col);
                    }
                    else
                    {
                        marker.A_SetRenderStyle(markerAlpha, STYLE_Add);
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