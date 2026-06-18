// PathfinderTracer is a custom 3D line-of-sight raycaster that inherits from GZDoom's native LineTracer.
// It is used to determine if a straight 3D line between waypoints is physically passable by the player.
// How it works:
// - It traces a ray through sectors and portals.
// - It stops when hitting solid walls, floors, or ceilings.
// - It skips actors (monsters, items) since they are dynamic and shouldn't block static pathing.
// - When crossing a portal/linedef boundary, it verifies the clearance and step heights relative to the crossing point's vertical Z position.
class PathfinderTracer : LineTracer
{
    override ETraceStatus TraceCallback()
    {
        if (Results.HitType == TRACE_HitWall) return TRACE_Stop;
        if (Results.HitType == TRACE_HitFloor || Results.HitType == TRACE_HitCeiling) return TRACE_Stop;
        if (Results.HitType == TRACE_HitActor) return TRACE_Skip; // Ignore actors

        // TRACE_CrossingPortal means the tracer crossed a portal or a sector boundary line.
        if (Results.HitType == TRACE_CrossingPortal)
        {
            if (Results.HitLine)
            {
                if (Results.HitLine.flags & Line.ML_BLOCKING) 
                    return TRACE_Stop;

                Sector front = Results.HitLine.frontsector;
                Sector back = Results.HitLine.backsector;
                Vector2 hitPt = Results.HitPos.xy;
                Vector2 destHitPt = hitPt;

                // Portal Warp Support:
                // Linedef portals warp the player to a physically distant part of the map.
                // To keep the ray tracer from flying off-course or getting lost in the void,
                // we must calculate the displacement vector and apply it to get the warped target coordinates.
                if (Results.HitLine.isLinePortal())
                {
                    Line dest = Results.HitLine.getPortalDestination();
                    if (dest)
                    {
                        back = dest.frontsector;
                        destHitPt = hitPt + Results.HitLine.getPortalDisplacement();
                    }
                }

                if (front && back)
                {
                    double fFloor, fCeil, bFloor, bCeil;
                    // Retrieve effective floor and ceiling Z heights using the trace's exact 3D crossing height (Results.HitPos.z)
                    // as the reference Z. This ensures the tracer correctly evaluates the matching vertical slice
                    // of stacked 3D geometry (like bridges or portals).
                    HoloGPSHandler.GetEffectiveFloorCeil(front, hitPt, Results.HitPos.z, fFloor, fCeil);
                    HoloGPSHandler.GetEffectiveFloorCeil(back, destHitPt, Results.HitPos.z, bFloor, bCeil);

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
