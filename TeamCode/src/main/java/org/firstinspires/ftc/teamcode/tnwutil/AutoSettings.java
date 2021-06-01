package org.firstinspires.ftc.teamcode.tnwutil;

import java.io.Serializable;

public class AutoSettings implements Serializable {

    // Unique long which specifies which version this class is
    private static final long serialVersionUID = 8628420841535456451L;

    // Number of option fields
    public static final short LENGTH = 6;

    // Settings to be stored as fields, and their default values
    public boolean innerStartingLine = true;
    public boolean starterStack = false;
    public boolean powerShots = false;
    public boolean deliverWobble = false;
    public boolean park = false;
    public byte parkingLocation = 2;

}
