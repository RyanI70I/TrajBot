/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.helpers;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/**
 * Add your docs here.
 */
public class limelightHelper {
    public void getRawY(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double y = ty.getDouble(0.0);
        System.out.println(y);
      }
      public static double getDistence() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double y = ty.getDouble(0.0);
        double distence = 23.2576688011 + Math.abs(y);
        distence = Math.toRadians(distence);
        distence = Math.tan(distence);
        distence = 41 / distence;
        return distence;
    }

    public static double getTargetX(double angle) {
        return getDistence() * Math.cos(angle);
       }
       public static double getTargetY(double angle){
        return getDistence() * Math.sin(angle);
    }

}
