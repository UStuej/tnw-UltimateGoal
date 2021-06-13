package org.firstinspires.ftc.teamcode.tnwutil;

import java.io.Serializable;

public class AutoSettings implements Serializable {

    // Unique long which specifies which version this class is
    private static final long serialVersionUID = -824028284687819026L;

    // Number of option fields
    public static final short LENGTH = 6;

    // Settings to be stored as fields, and their default values
    public boolean innerStartingLine = true;
    public boolean starterStack = false;
    public boolean partnerRings = false;
    public boolean deliverWobble = false;
    public boolean park = false;
    public byte parkingLocation = 2;

}
