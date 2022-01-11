// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.utility.Gains;

public class SwerveModule extends SubsystemBase {
    private WPI_TalonFX driveMotor;
    private WPI_TalonFX angleMotor;
    private CANCoder canCoder;
    public static Gains m_swerveGains = SwerveDriveConstants.SWERVE_GAINS;

    private static double kEncoderTicksPerRotation = 4096;


  /** Creates a new SwerveModule. */
  public SwerveModule(WPI_TalonFX driveMotor, WPI_TalonFX angleMotor, CANCoder canCoder, double offset) {
    this.driveMotor = driveMotor;
    this.angleMotor = angleMotor;
    this.canCoder = canCoder;

    TalonFXConfiguration angleTalonFXConfiguration = new TalonFXConfiguration();

    angleTalonFXConfiguration.slot0.kP = m_swerveGains.m_kP;
    angleTalonFXConfiguration.slot0.kI = m_swerveGains.m_kI;
    angleTalonFXConfiguration.slot0.kD = m_swerveGains.m_kD;

    // Use the CANCoder as the remote sensor for the primary TalonFX PID
    angleTalonFXConfiguration.remoteFilter0.remoteSensorDeviceID = canCoder.getDeviceID();
    angleTalonFXConfiguration.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    angleTalonFXConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
    angleMotor.configAllSettings(angleTalonFXConfiguration);

    /*TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();

    driveTalonFXConfiguration.slot0.kP = kDriveP;
    driveTalonFXConfiguration.slot0.kI = kDriveI;
    driveTalonFXConfiguration.slot0.kD = kDriveD;
    driveTalonFXConfiguration.slot0.kF = kDriveF;

    driveMotor.configAllSettings(driveTalonFXConfiguration);*/

    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.magnetOffsetDegrees = offset;
    canCoder.configAllSettings(canCoderConfiguration);
  }


  public Rotation2d getAngle() {
    // Note: This assumes the CANCoders are setup with the default feedback coefficient
    // and the sesnor value reports degrees.
    return Rotation2d.fromDegrees(canCoder.getAbsolutePosition());
  }

  /**
   * Set the speed + rotation of the swerve module from a SwerveModuleState object
   * @param desiredState - A SwerveModuleState representing the desired new state of the module
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    Rotation2d currentRotation = getAngle();
    SmartDashboard.putNumber("Motor " + angleMotor.getDeviceID(), currentRotation.getDegrees());
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);

    // Find the difference between our current rotational position + our new rotational position
    Rotation2d rotationDelta = state.angle.minus(currentRotation);

    // Find the new absolute position of the module based on the difference in rotation
    double deltaTicks = (rotationDelta.getDegrees() / 360.) * kEncoderTicksPerRotation;
    // Convert the CANCoder from it's position reading back to ticks
    double currentTicks = canCoder.getPosition() / canCoder.configGetFeedbackCoefficient();
    double desiredTicks = currentTicks + deltaTicks;
    angleMotor.set(TalonFXControlMode.Position, desiredTicks);


    double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond);
    driveMotor.set(angleMotor.get() + feetPerSecond / SwerveDriveConstants.SPEED_FEET_PER_SECOND_AT_FULL_POWER);
  }
}
