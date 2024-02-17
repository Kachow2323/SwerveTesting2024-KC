package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagDetection;
import frc.utils.CoordinateSpace;

public class Eyes {


    // 1. Compute direction offset from tag
    public static double widthOffset(
        AprilTagDetection aprilTag, 
        CoordinateSpace coordinateSpace
    ) {
        double coordinateCenter = coordinateSpace.width/2;
        double aprilTagcenter = aprilTag.getCenterX();
        double offset = aprilTagcenter - coordinateCenter;
        return offset; // 
    }

    // 2. turn offset into movement direction
    public static double rotationRate(double offset)
    {
        double zeroAccuracy = 0.0001; 
        if (offset < zeroAccuracy && offset > -zeroAccuracy) {
            return 0.0;
        } else if (offset < 0) {
            return -1;
        } else {
            return 1;
        }
    }
}