package frc.robot;

import edu.wpi.first.wpilibj.util.Color;

public final class Constants {
    
    // subsystems
    public static final class Indexer {
        public static final int kIndexerPort = 12;
        public static final int kProximityLimit = 1250;
        public static final double kIndexerSpeed = 0.2;
        public static final double kConfidenceThreshold = 0.85;
        public static final Color kRedTarget = new Color(1, 0, 0);
        public static final Color kBlueTarget = new Color(0, 0, 1);
    }

    // math/misc.
    public static final class CIEXYZ2sRGBConversion {
        /*  CIE XYZ to sRGB linear transformation:
         *  [3.2406 -1.5372 -0.4986][X_D65] = [R_linear]
         *  [-0.9689 1.8758 0.0415][Y_D65] = [G_linear]
         *  [0.0557 -0.2040 1.0570][Z_D65] = [B_linear]
         *  This is equivalent to:
         *  R =  3.2406*X -1.5372*Y -0.4986*Z
         *  G = -0.9689*X + 1.8758*Y + 0.0415*Z
         *  B =  0.0557*X -0.2040*Y + 1.0570*Z
         */ 

         // R coeffs.
        public static final double rx = 3.2406;
        public static final double ry = -1.5372;
        public static final double rz = -0.4986;

        // G coeffs.
        public static final double gx = -0.9689;
        public static final double gy = 1.8758;
        public static final double gz = 0.0415;

        // B coeffs.
        public static final double bx = 0.0557;
        public static final double by = -0.2040;
        public static final double bz = 1.0570;
    }
}
