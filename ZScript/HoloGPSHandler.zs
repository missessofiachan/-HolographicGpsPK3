class PathfinderTracer : LineTracer
{
    override ETraceStatus TraceCallback()
    {
        if (Results.HitType == TRACE_HitWall) return TRACE_Stop;
        if (Results.HitType == TRACE_HitFloor || Results.HitType == TRACE_HitCeiling) return TRACE_Stop;
        if (Results.HitType == TRACE_HitActor) return TRACE_Skip; // Ignore actors

        if (Results.HitType == TRACE_CrossingPortal)
        {
            if (Results.HitLine && (Results.HitLine.flags & Line.ML_BLOCKING))
            {
                return TRACE_Stop;
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

    // Pathfinding result: ordered waypoints from player to target
    Array<double> pathX;
    Array<double> pathY;
    Array<double> refinedX;
    Array<double> refinedY;

    // Pre-built adjacency graph (built once per map load)
    // adjStart[i] = first index in adjNeighbor/adjLineIdx for sector i
    // adjStart[numSectors] = total entries
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
        BuildAdjacencyGraph();
        FindNextObjective();
    }

    void BuildAdjacencyGraph()
    {
        int numSectors = level.sectors.Size();
        int numLines = level.lines.Size();

        // Count neighbors per sector
        Array<int> neighborCount;
        neighborCount.Resize(numSectors);
        for (int i = 0; i < numSectors; i++) neighborCount[i] = 0;

        for (int li = 0; li < numLines; li++)
        {
            Line ln = level.lines[li];
            if (!ln.frontsector || !ln.backsector) continue;
            
            // Ignore impassable lines (walls/windows that can't be walked through)
            if (ln.flags & Line.ML_BLOCKING) continue;

            int fi = ln.frontsector.sectornum;
            int bi = ln.backsector.sectornum;
            if (fi == bi) continue;

            neighborCount[fi]++;
            neighborCount[bi]++;
        }

        // Build prefix sum for adjStart
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

        // Fill adjacency data
        Array<int> fillPos;
        fillPos.Resize(numSectors);
        for (int i = 0; i < numSectors; i++) fillPos[i] = adjStart[i];

        for (int li = 0; li < numLines; li++)
        {
            Line ln = level.lines[li];
            if (!ln.frontsector || !ln.backsector) continue;
            
            // Ignore impassable lines
            if (ln.flags & Line.ML_BLOCKING) continue;

            // Ignore secret doors unless the user explicitly wants them for speedrunning
            bool useSecrets = CVar.GetCVar("holo_gps_use_secrets", players[consoleplayer]).GetBool();
            if (!useSecrets && (ln.flags & Line.ML_SECRET)) continue;

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
        if (!CVar.GetCVar("holo_gps_enabled", players[consoleplayer]).GetBool())
            return;

        tickCounter++;
        int currentFreq = CVar.GetCVar("holo_gps_freq", players[consoleplayer]).GetInt();
        if (currentFreq < 1) currentFreq = 1;
        if (tickCounter % currentFreq != 0) return;

        PlayerInfo plyr = players[consoleplayer];
        if (!plyr || !plyr.mo || plyr.mo.health <= 0) return;

        if (currentTarget)
        {
            // If the target is a key, check if it was picked up
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
            FindPathBFS(plyr.mo, currentTarget);
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

        Array<Actor> potentialTargets;

        // Priority 1: Find all keys the player doesn't have
        ThinkerIterator it = ThinkerIterator.Create("Key");
        Key mapKey;
        while (mapKey = Key(it.Next()))
        {
            if (!mapKey.owner && !plyr.mo.FindInventory(mapKey.GetClass()))
            {
                potentialTargets.Push(mapKey);
            }
        }

        // Priority 2: Find exit line specials (243 = Exit_Normal, 244 = Exit_Secret)
        if (potentialTargets.Size() == 0)
        {
            for (int i = 0; i < level.lines.Size(); i++)
            {
                Line ln = level.lines[i];
                if (ln && (ln.special == 243 || ln.special == 244))
                {
                    Vector2 midpoint = (ln.v1.p + ln.v2.p) / 2.0;
                    double z = level.PointInSector(midpoint).floorplane.ZatPoint(midpoint);
                    Actor spot = Actor.Spawn("MapSpot", (midpoint.x, midpoint.y, z));
                    if (spot) potentialTargets.Push(spot);
                }
            }
        }

        // Find the first target that has a valid path
        for (int i = 0; i < potentialTargets.Size(); i++)
        {
            FindPathBFS(plyr.mo, potentialTargets[i]);
            if (pathX.Size() > 0)
            {
                currentTarget = potentialTargets[i];
                
                // Destroy other MapSpots
                for (int j = 0; j < potentialTargets.Size(); j++)
                {
                    if (i != j && potentialTargets[j] && potentialTargets[j].GetClassName() == "MapSpot")
                    {
                        potentialTargets[j].Destroy();
                    }
                }
                return;
            }
        }

        // If no path found, clean up
        currentTarget = null;
        for (int i = 0; i < potentialTargets.Size(); i++)
        {
            if (potentialTargets[i] && potentialTargets[i].GetClassName() == "MapSpot")
            {
                potentialTargets[i].Destroy();
            }
        }
    }

    bool IsPortalPassable(Sector curSec, Sector nextSec, Line ln, Actor player)
    {
        if (!ln || !curSec || !nextSec) return false;

        // 1. Check blocking flags
        if (ln.flags & Line.ML_BLOCKING) return false;

        // Secrets check
        bool useSecrets = CVar.GetCVar("holo_gps_use_secrets", players[consoleplayer]).GetBool();
        if (!useSecrets && (ln.flags & Line.ML_SECRET)) return false;

        Vector2 midpoint = (ln.v1.p + ln.v2.p) / 2.0;
        double curFloor = curSec.floorplane.ZatPoint(midpoint);
        double curCeil = curSec.ceilingplane.ZatPoint(midpoint);
        double nextFloor = nextSec.floorplane.ZatPoint(midpoint);
        double nextCeil = nextSec.ceilingplane.ZatPoint(midpoint);

        // 2. Check step height (player cannot step up more than 24 units)
        if (nextFloor - curFloor > 24.0) return false;

        // 3. Check gap clearance (must be >= 56 units)
        double portalFloor = (curFloor > nextFloor) ? curFloor : nextFloor;
        double portalCeil = (curCeil < nextCeil) ? curCeil : nextCeil;
        double clearance = portalCeil - portalFloor;

        if (clearance < 56.0)
        {
            // Check if it's a direct-use door
            bool isDirectUse = (ln.activation & (SPAC_Use | SPAC_UseThrough)) != 0;
            bool isDoorSpecial = false;
            int spec = ln.special;
            if (spec == 10 || spec == 11 || spec == 12 || spec == 13 || spec == 105 || spec == 106 || spec == 202)
            {
                isDoorSpecial = true;
            }

            if (isDirectUse && isDoorSpecial)
            {
                // Check if locked and player lacks the key
                if (ln.locknumber != 0)
                {
                    PlayerPawn plyrPawn = PlayerPawn(player);
                    if (plyrPawn && !plyrPawn.CheckKeys(ln.locknumber, false))
                    {
                        return false; // Locked and no key
                    }
                }
                // It's a direct-use door the player can open!
            }
            else
            {
                return false; // Not a direct-use door, so it's too narrow/closed
            }
        }

        return true;
    }

    bool IsPlayerTriggerable(Line ln)
    {
        if (ln.special == 0) return false;
        int act = ln.activation;
        if (act & (SPAC_Use | SPAC_UseThrough | SPAC_Cross | SPAC_Impact | SPAC_Push | SPAC_AnyCross))
        {
            return true;
        }
        return false;
    }

    void FindPathBFS(Actor player, Actor target)
    {
        pathX.Clear();
        pathY.Clear();

        if (graphBuilt == 0) return;

        int numSectors = level.sectors.Size();

        Sector startSec = level.PointInSector(player.pos.xy);
        Sector endSec = level.PointInSector(target.pos.xy);

        int startIdx = startSec.sectornum;
        int endIdx = endSec.sectornum;

        // Same sector — just point directly to target
        if (startIdx == endIdx)
        {
            pathX.Push(target.pos.x);
            pathY.Push(target.pos.y);
            return;
        }

        // Pass 1: Forward BFS from player using only passable portals
        Array<int> parent;
        Array<int> parentLine;
        Array<int> reachable;
        Array<int> queue;

        parent.Resize(numSectors);
        parentLine.Resize(numSectors);
        reachable.Resize(numSectors);

        for (int i = 0; i < numSectors; i++)
        {
            parent[i] = -1;
            parentLine[i] = -1;
            reachable[i] = 0;
        }

        reachable[startIdx] = 1;
        queue.Push(startIdx);

        int queueHead = 0;
        int found = 0;

        while (queueHead < queue.Size())
        {
            int current = queue[queueHead];
            queueHead++;

            if (current == endIdx)
            {
                found = 1;
                break;
            }

            // Iterate neighbors of current sector
            int nStart = adjStart[current];
            int nEnd = adjStart[current + 1];
            for (int ni = nStart; ni < nEnd; ni++)
            {
                int neighbor = adjNeighbor[ni];
                if (reachable[neighbor] == 0)
                {
                    Line ln = level.lines[adjLineIdx[ni]];
                    Sector curSec = level.sectors[current];
                    Sector nextSec = level.sectors[neighbor];

                    if (IsPortalPassable(curSec, nextSec, ln, player))
                    {
                        reachable[neighbor] = 1;
                        parent[neighbor] = current;
                        parentLine[neighbor] = adjLineIdx[ni];
                        queue.Push(neighbor);
                    }
                }
            }
        }

        // If path is found in Pass 1, reconstruct normally
        if (found == 1)
        {
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

            pathX.Push(target.pos.x);
            pathY.Push(target.pos.y);
            return;
        }

        // Pass 2: Reverse topological BFS from target to find the barrier
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

        // Search the level for any switches/walkovers that trigger the barrier's tag
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

            // Check if reachable
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
        if (sec)
        {
            return sec.floorplane.ZatPoint(pos);
        }
        return 0;
    }

    bool PathIsBlocked(Vector2 start, Vector2 end)
    {
        double floorzA = GetFloorZ(start);
        double floorzB = GetFloorZ(end);

        Vector3 pA = (start.x, start.y, floorzA + 16.0);
        Vector3 pB = (end.x, end.y, floorzB + 16.0);

        PathfinderTracer tracer = new("PathfinderTracer");
        if (!tracer) return false;

        Vector3 diff = pB - pA;
        double len = diff.Length();
        if (len < 1.0) return false;
        Vector3 dir = diff / len;

        Sector sec = level.PointInSector(start);
        if (!sec) return false;

        tracer.Trace(pA, sec, dir, len, TRACE_ReportPortals, 0xFFFFFFFF, true);
        return tracer.Results.HitType != TRACE_HitNone;
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
        if (depth > 3) return false;

        double floorzA = GetFloorZ(A);
        double floorzB = GetFloorZ(B);

        Vector3 start = (A.x, A.y, floorzA + 16.0);
        Vector3 end = (B.x, B.y, floorzB + 16.0);

        PathfinderTracer tracer = new("PathfinderTracer");
        if (!tracer) return false;

        Vector3 diff = end - start;
        double len = diff.Length();
        if (len < 1.0) return true;
        Vector3 dir = diff / len;

        Sector sec = level.PointInSector(A);
        if (!sec) return false;

        tracer.Trace(start, sec, dir, len, TRACE_ReportPortals, 0xFFFFFFFF, true);
        if (tracer.Results.HitType == TRACE_HitNone)
        {
            return true;
        }

        Line hitLn = tracer.Results.HitLine;
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
        if (refinedX.Size() == 0) return;

        float userAlpha = CVar.GetCVar("holo_gps_alpha", players[consoleplayer]).GetFloat();
        if (userAlpha <= 0.0) userAlpha = 0.6;

        int spacing = CVar.GetCVar("holo_gps_spacing", players[consoleplayer]).GetInt();
        if (spacing < 32) spacing = 32;

        int maxMarkers = CVar.GetCVar("holo_gps_max_markers", players[consoleplayer]).GetInt();
        if (maxMarkers < 1) maxMarkers = 1;

        bool fadeEnabled = CVar.GetCVar("holo_gps_fade", players[consoleplayer]).GetBool();
        int colorMode = CVar.GetCVar("holo_gps_color", players[consoleplayer]).GetInt();

        Vector2 prevPoint = player.pos.xy;
        Vector2 lastSpawnPos = player.pos.xy;
        int spawnedCount = 0;

        for (int wp = 0; wp < refinedX.Size(); wp++)
        {
            if (spawnedCount >= maxMarkers) break;
            
            Vector2 waypoint = (refinedX[wp], refinedY[wp]);

            // Interpolate arrows between prevPoint and this waypoint
            Vector2 seg = waypoint - prevPoint;
            double segLen = seg.Length();

            if (segLen < 16)
            {
                prevPoint = waypoint;
                continue;
            }

            int segSteps = int(segLen / spacing);
            if (segSteps < 1) segSteps = 1;

            Vector2 stepDir = seg / segLen;
            double arrowAngle = VectorAngle(seg.x, seg.y);

            for (int i = 1; i <= segSteps; i++)
            {
                if (spawnedCount >= maxMarkers) break;
                
                Vector2 spawnXY = prevPoint + stepDir * (spacing * i);

                // Don't overshoot the waypoint
                if ((spawnXY - prevPoint).Length() > segLen)
                    spawnXY = waypoint;

                if (PathIsBlocked(lastSpawnPos, spawnXY))
                {
                    spawnedCount = maxMarkers; // Force break
                    break;
                }

                double floorz = level.PointInSector(spawnXY).floorplane.ZatPoint(spawnXY);
                Vector3 spawnPos = (spawnXY.x, spawnXY.y, floorz + 1.0);

                Actor marker = Actor.Spawn("HoloPathMarker", spawnPos);
                if (marker)
                {
                    double markerAlpha = userAlpha;
                    if (fadeEnabled)
                    {
                        markerAlpha = userAlpha * (1.0 - (double(spawnedCount) / double(maxMarkers)));
                    }

                    marker.angle = arrowAngle;

                    if (colorMode == 9) // Trans Pride
                    {
                        // Cycle: 0 = Light Blue, 1 = Pink, 2 = White, 3 = Pink
                        int cycleStep = spawnedCount % 4;
                        color transCol = 0xFFFFFF; // White
                        if (cycleStep == 0) transCol = 0x5BCEFA; // Light Blue
                        else if (cycleStep == 1 || cycleStep == 3) transCol = 0xF5A9B8; // Pink
                        
                        marker.A_SetRenderStyle(markerAlpha, STYLE_AddStencil);
                        marker.SetShade(transCol);
                    }
                    else if (colorMode > 0)
                    {
                        color col = 0xFFFFFF;
                        if (colorMode == 1) col = 0xFF0000;      // Red
                        else if (colorMode == 2) col = 0x00FF00; // Green
                        else if (colorMode == 3) col = 0x0000FF; // Blue
                        else if (colorMode == 4) col = 0xFFFF00; // Yellow
                        else if (colorMode == 5) col = 0xFF8000; // Orange
                        else if (colorMode == 6) col = 0x8000FF; // Purple
                        else if (colorMode == 7) col = 0xFF00FF; // Pink
                        else if (colorMode == 8) col = 0xFFFFFF; // White
                        
                        marker.A_SetRenderStyle(markerAlpha, STYLE_AddStencil);
                        marker.SetShade(col);
                    }
                    else
                    {
                        // Default cyan sprite
                        marker.A_SetRenderStyle(markerAlpha, STYLE_Add);
                    }

                    activeMarkers.Push(marker);
                    spawnedCount++;
                    lastSpawnPos = spawnXY;
                }
            }
            if (spawnedCount >= maxMarkers) break;
            prevPoint = waypoint;
        }
    }

}
