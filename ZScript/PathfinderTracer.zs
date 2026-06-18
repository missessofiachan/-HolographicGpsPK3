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
                Vector2 hitPt = Results.HitPos.xy;
                Vector2 destHitPt = hitPt;

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
                    double fFloor = front.floorplane.ZatPoint(hitPt);
                    double fCeil = front.ceilingplane.ZatPoint(hitPt);
                    double bFloor = back.floorplane.ZatPoint(destHitPt);
                    double bCeil = back.ceilingplane.ZatPoint(destHitPt);

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
