/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.SwerveDriveConstants;

public class SwerveDrive extends SubsystemBase
{
    SwerveDriveKinematics m_kinematics;
    private WPI_TalonFX m_leftFrontSteerMotor;
    private WPI_TalonFX m_leftFrontWheelMotor;
    private WPI_TalonFX m_rightFrontSteerMotor;
    private WPI_TalonFX m_rightFrontWheelMotor;
    private WPI_TalonFX m_leftBackSteerMotor;
    private WPI_TalonFX m_leftBackWheelMotor;
    private WPI_TalonFX m_rightBackSteerMotor;
    private WPI_TalonFX m_rightBackWheelMotor;
    private CANCoder m_leftFrontEncoder; 
    private CANCoder m_rightFrontEncoder;
    private CANCoder m_leftBackEncoder;
    private CANCoder m_rightBackEncoder;
        
    public SwerveDrive(WPI_TalonFX leftFrontSteerMotor,WPI_TalonFX leftFrontWheelMotor,WPI_TalonFX rightFrontSteerMotor,WPI_TalonFX rightFrontWheelMotor,
    WPI_TalonFX leftBackSteerMotor,WPI_TalonFX leftBackWheelMotor,WPI_TalonFX rightBackSteerMotor,WPI_TalonFX rightBackWheelMotor, CANCoder leftFrontEncoder,
    CANCoder rightFrontEncoder,
    CANCoder leftBackEncoder,
    CANCoder rightBackEncoder)
    {
        m_leftFrontSteerMotor = leftFrontSteerMotor;
        m_leftFrontWheelMotor = leftFrontWheelMotor;
        m_rightFrontSteerMotor = rightFrontSteerMotor;
        m_rightFrontWheelMotor = rightFrontWheelMotor;
        m_leftBackSteerMotor = leftBackSteerMotor;
        m_leftBackWheelMotor = leftBackWheelMotor;
        m_rightBackSteerMotor = rightBackSteerMotor;
        m_rightBackWheelMotor = rightBackWheelMotor;
        m_leftFrontEncoder = leftFrontEncoder; 
        m_rightFrontEncoder = rightFrontEncoder;
        m_leftBackEncoder = leftBackEncoder;
        m_rightBackEncoder = rightBackEncoder; 
        double halfWidth = SwerveDriveConstants.WIDTH / 2.d;
        double halfHeight = SwerveDriveConstants.HEIGHT / 2.d;

        Translation2d m_frontLeftLocation = new Translation2d(halfHeight, halfWidth);
        Translation2d m_frontRightLocation = new Translation2d(halfHeight, -halfWidth);
        Translation2d m_backLeftLocation = new Translation2d(-halfHeight, halfWidth);
        Translation2d m_backRightLocation = new Translation2d(-halfHeight, -halfWidth);
        
        m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    }


    public void driveWithInput(double strafeX, double strafeY, double rotate)
    {
        var speeds = new ChassisSpeeds(strafeX, strafeY, rotate * SwerveDriveConstants.ROTATION_SPEED /*in rad/s */);
        driveFromSpeeds(speeds);
    }

    public void driveFromSpeeds(ChassisSpeeds speeds)
    {
        //https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html 
        // Convert to module states
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        // Front left module state
        SwerveModuleState leftFront = SwerveModuleState.optimize(moduleStates[0], new Rotation2d(m_leftFrontEncoder.getPosition()));
        // Front right module state
        SwerveModuleState rightFront = SwerveModuleState.optimize(moduleStates[1], new Rotation2d(m_rightFrontEncoder.getPosition()));
        // Back left module state
        SwerveModuleState leftBack = SwerveModuleState.optimize(moduleStates[2], new Rotation2d(m_leftBackEncoder.getPosition()));
        // Back right module state
        SwerveModuleState rightBack = SwerveModuleState.optimize(moduleStates[3], new Rotation2d(m_rightBackEncoder.getPosition()));
        
        //Set the Wheel motor speeds
        m_leftFrontWheelMotor.set(m_leftFrontSteerMotor.get() + leftFront.speedMetersPerSecond * SwerveDriveConstants.WHEEL_SPEED);
        m_rightFrontWheelMotor.set(m_rightFrontSteerMotor.get() + rightFront.speedMetersPerSecond * SwerveDriveConstants.WHEEL_SPEED);
        m_leftBackWheelMotor.set(m_leftBackSteerMotor.get() + leftBack.speedMetersPerSecond * SwerveDriveConstants.WHEEL_SPEED);
        m_rightBackWheelMotor.set(m_rightBackSteerMotor.get() + rightBack.speedMetersPerSecond * SwerveDriveConstants.WHEEL_SPEED);

    }

    // public void driveFieldRelative(double awayFromStation, double towardLeftBoundary, double rotate)
    // {
    //     var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(awayFromStation, towardLeftBoundary, 
    //         rotate * SwerveDriveConstants.RotationSpeed, /*get odometry angle*/)
    // }
}