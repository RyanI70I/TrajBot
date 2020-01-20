/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.util.Units;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import com.ctre.phoenix.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.sensors.PigeonIMU;

import org.opencv.core.RotatedRect;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */
  public static WPI_TalonSRX LFSRX;
	public static WPI_TalonSRX LBSRX;
  public static WPI_TalonSRX RFSRX;
  public static float leftDistance = 0;
  public static float rightDistance = 0;
  public static double StartingPose;
  public static WPI_TalonSRX RBSRX;

  public static float WheelCircumfrance = (float) 18.8495559215;//9.4247796077;


  public static DifferentialDriveOdometry DDO;
  public static PigeonIMU pigeon = new PigeonIMU(15);
  public static Rotation2d heading = new Rotation2d();
  public Drivetrain() {

  }

  @Override
  
  public void periodic() {
    //    

    //DDO.update(gyroAngle, leftDistanceMeters, rightDistanceMeters)

    //DDO.update(new Rotation2d(getPigeonCompassHeading()), getDistenceLeft(), getDistenceRight());
    // This method will be called once per scheduler run
  }

  public void initDrive(){
    
    LFSRX  = new WPI_TalonSRX(2);
		LBSRX   = new WPI_TalonSRX(4);
		RFSRX = new WPI_TalonSRX(1);
    RBSRX  = new WPI_TalonSRX(3);
    
    DDO = new DifferentialDriveOdometry(new Rotation2d(0));
    LFSRX.setExpiration(.1);
		LBSRX.setExpiration(.1);
		RBSRX.setExpiration(.1);
		RFSRX.setExpiration(.1);
		LBSRX.follow(LFSRX);
    RBSRX.follow(RFSRX);
		RBSRX.setInverted(true);
		LBSRX.setInverted(true);
    RFSRX.setInverted(true);
    LFSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		RFSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    RFSRX.setSelectedSensorPosition(0);
    LFSRX.setSelectedSensorPosition(0);
    RFSRX.setSensorPhase(true);
    RFSRX.config_kF(0, .001, 10);
    RFSRX.config_kP(0, .002, 10);
    RFSRX.config_kI(0, .007, 10);
    RFSRX.config_kD(0, .0007, 10);
    LFSRX.config_kF(0, .001, 10);
    LFSRX.config_kP(0, .002, 10);
    LFSRX.config_kI(0, .007, 10);
    LFSRX.config_kD(0, .0007, 10);
  }

  public void setPowerLeft(double left){
    LFSRX.set(ControlMode.PercentOutput, left);
    updatePose();
    //System.out.println(left);
  }
  public void updatePose(){
    DDO.update(new Rotation2d(Units.degreesToRadians(getModPigeonYaw())), Units.inchesToMeters(getDistenceLeft()), Units.inchesToMeters(getDistenceRight()));
  }

  public void setPowerRight(double right){
    RFSRX.set(ControlMode.PercentOutput, right);
    System.out.println(getModPigeonYaw());

  }
  public void setVelocity(double left, double right){
//TODO:Closed loop Velocity PID control
    //System.out.println(RFSRX.getSelectedSensorVelocity());
    //System.out.println(LFSRX.getSelectedSensorVelocity());
    //System.out.println(right);
    
    RFSRX.set(ControlMode.Velocity, MetersToUnits(right));
    LFSRX.set(ControlMode.Velocity, MetersToUnits(left));
    System.out.println(getPose());
    System.out.println(RFSRX.getSelectedSensorVelocity());
    System.out.println(right);
    System.out.println(MetersToUnits(right));
    //LBSRX.set(ControlMode.Velocity, 0);
    //LFSRX.set(ControlMode.Velocity, -12);
    

    
  }
  public double MetersToUnits(double meters){
    meters = Units.metersToInches(meters);
    meters = meters / (6 * Math.PI);
    meters = meters * 100;
    return meters;

  }
  public double getDistenceLeft(){
      
     leftDistance = (float) (LFSRX.getSelectedSensorPosition());
     leftDistance = leftDistance/1024;
     leftDistance = leftDistance * WheelCircumfrance;
    return leftDistance;
  }
  public double getDistenceRight(){
     rightDistance = (float) (RFSRX.getSelectedSensorPosition());
     rightDistance = rightDistance/1024;
     rightDistance = rightDistance * WheelCircumfrance;
    return leftDistance;
  }
  public void resetSensors(){
    DDO.resetPosition(new Pose2d(), new Rotation2d());
    RFSRX.setSelectedSensorPosition(0);
    LFSRX.setSelectedSensorPosition(0);
  }
  public static double getPigeonCompassHeading(){
    if(getPigeonYaw() < 0) {return 360 - (getPigeonYaw() % 360.0 + 360.0);}
    return 360 - (getPigeonYaw() % 360.0);
  }
  public static double getPigeonYaw(){
    double[] output = new double[3];
    pigeon.getYawPitchRoll(output);
    return output[0];
  }
  public static double getModPigeonYaw(){
    if (getPigeonCompassHeading() - StartingPose < 0){
      return getPigeonCompassHeading() - StartingPose + 360;
    }
    return getPigeonCompassHeading() - StartingPose;

  }
  public Pose2d getPose(){
    updatePose();
    return DDO.getPoseMeters();
  }
  public void setStartpose(double start){
    StartingPose = start;
  }


  public void initmanual(){
	
  }

}
