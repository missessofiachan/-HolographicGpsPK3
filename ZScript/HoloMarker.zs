class HoloPathMarker : Actor
{
    Default
    {
        Radius 2;
        Height 2;
        +NOBLOCKMAP
        +NOGRAVITY
        +DONTSPLASH
        +NOINTERACTION
        +FLATSPRITE
        +NOBLOODDECALS
        RenderStyle "Add";
        Alpha 0.8;
        Scale 0.25;
    }

    States
    {
    Spawn:
        AMRK A -1 Bright;  // Persist forever — handler will Destroy() us
        Stop;
    }
}
