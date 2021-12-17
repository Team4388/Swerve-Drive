/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.utility.Gains;
import frc4388.utility.RobotGyro;

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
    double halfWidth = SwerveDriveConstants.WIDTH / 2.d;
    double halfHeight = SwerveDriveConstants.HEIGHT / 2.d;
    public static Gains m_swerveGains = SwerveDriveConstants.SWERVE_GAINS;


    Translation2d m_frontLeftLocation = 
        new Translation2d(
            Units.inchesToMeters(halfHeight), 
            Units.inchesToMeters(halfWidth));
    Translation2d m_frontRightLocation =
         new Translation2d(
            Units.inchesToMeters(halfHeight), 
            Units.inchesToMeters(-halfWidth));
    Translation2d m_backLeftLocation = 
        new Translation2d(
            Units.inchesToMeters(-halfHeight), 
            Units.inchesToMeters(halfWidth));
    Translation2d m_backRightLocation = 
        new Translation2d(
            Units.inchesToMeters(-halfHeight), 
            Units.inchesToMeters(-halfWidth));
        //setSwerveGains();
        
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    public SwerveModule[] modules;
    public RobotGyro gyro; //TODO Add Gyro Lol


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

        modules = new SwerveModule[] {
            new SwerveModule(m_leftFrontWheelMotor, m_leftFrontSteerMotor, m_leftFrontEncoder), // Front Left
            new SwerveModule(m_rightFrontWheelMotor, m_rightFrontSteerMotor, m_rightFrontEncoder), // Front Right
            new SwerveModule(m_leftBackWheelMotor, m_leftBackSteerMotor, m_leftBackEncoder), // Back Left
            new SwerveModule(m_rightFrontWheelMotor, m_rightFrontSteerMotor, m_rightFrontEncoder)  // Back Right
        };
        //gyro.reset(); 
    }


    public void driveWithInput(double xSpeed, double ySpeed, double rot, boolean fieldRelative)
    {
        /*var speeds = new ChassisSpeeds(strafeX, strafeY, rotate * SwerveDriveConstants.ROTATION_SPEED //in rad/s );
        driveFromSpeeds(speeds);*/
        SwerveModuleState[] states =
            kinematics.toSwerveModuleStates(
                fieldRelative
                  ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                  : new ChassisSpeeds(xSpeed, ySpeed, rot));
         SwerveDriveKinematics.normalizeWheelSpeeds(states, SwerveDriveConstants.MAX_SPEED_FEET_PER_SEC);
         for (int i = 0; i < states.length; i++) {
            SwerveModule module = modules[i];
            SwerveModuleState state = states[i];
            module.setDesiredState(state);
    }
    }
    //Converts a ChassisSpeed to SwerveModuleStates (targets)
    public void driveFromSpeeds(ChassisSpeeds speeds)
    {
        //https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html 
        // Convert to module states
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        // Front left module state
        SwerveModuleState leftFront = SwerveModuleState.optimize(moduleStates[0], Rotation2d.fromDegrees(m_leftFrontEncoder.getPosition()));
        // Front right module state
        SwerveModuleState rightFront = SwerveModuleState.optimize(moduleStates[1], Rotation2d.fromDegrees(m_rightFrontEncoder.getPosition()));
        // Back left module state
        SwerveModuleState leftBack = SwerveModuleState.optimize(moduleStates[2], Rotation2d.fromDegrees(m_leftBackEncoder.getPosition()));
        // Back right module state
        SwerveModuleState rightBack = SwerveModuleState.optimize(moduleStates[3], Rotation2d.fromDegrees(m_rightBackEncoder.getPosition()));
        
        //Set the motors
        setSwerveMotors(leftFront, leftBack, rightFront, rightBack);
    }

    //Sets steering motors to PID values
    public void setSwerveMotors(SwerveModuleState leftFront, SwerveModuleState leftBack, SwerveModuleState rightFront, SwerveModuleState rightBack)
    {
        /*//Set the Wheel motor speeds
        m_leftFrontWheelMotor.set(m_leftFrontSteerMotor.get() + leftFront.speedMetersPerSecond * SwerveDriveConstants.WHEEL_SPEED);
        m_rightFrontWheelMotor.set(m_rightFrontSteerMotor.get() + rightFront.speedMetersPerSecond * SwerveDriveConstants.WHEEL_SPEED);
        m_leftBackWheelMotor.set(m_leftBackSteerMotor.get() + leftBack.speedMetersPerSecond * SwerveDriveConstants.WHEEL_SPEED);
        m_rightBackWheelMotor.set(m_rightBackSteerMotor.get() + rightBack.speedMetersPerSecond * SwerveDriveConstants.WHEEL_SPEED);

        //PID
        m_leftFrontSteerMotor.set(TalonFXControlMode.Position, leftFront.angle.getDegrees() * 12000);
        m_rightFrontSteerMotor.set(TalonFXControlMode.Position, rightFront.angle.getDegrees() * 12000);
        m_leftBackSteerMotor.set(TalonFXControlMode.Position, leftBack.angle.getDegrees() * 12000);
        m_rightBackSteerMotor.set(TalonFXControlMode.Position, rightBack.angle.getDegrees());

        System.out.println("Target: " + leftFront.angle.getDegrees());*/
    }    
    
    /*public void setSwerveGains(){
        m_leftFrontSteerMotor.selectProfileSlot(SwerveDriveConstants.SWERVE_SLOT_IDX, SwerveDriveConstants.SWERVE_PID_LOOP_IDX);
        m_leftFrontSteerMotor.config_kF(SwerveDriveConstants.SWERVE_SLOT_IDX, m_swerveGains.m_kF, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
        m_leftFrontSteerMotor.config_kP(SwerveDriveConstants.SWERVE_SLOT_IDX, m_swerveGains.m_kP, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
        m_leftFrontSteerMotor.config_kI(SwerveDriveConstants.SWERVE_SLOT_IDX, m_swerveGains.m_kI, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
        m_leftFrontSteerMotor.config_kD(SwerveDriveConstants.SWERVE_SLOT_IDX, m_swerveGains.m_kD, SwerveDriveConstants.SWERVE_TIMEOUT_MS);

        m_rightFrontSteerMotor.selectProfileSlot(SwerveDriveConstants.SWERVE_SLOT_IDX, SwerveDriveConstants.SWERVE_PID_LOOP_IDX);
        m_rightFrontSteerMotor.config_kF(SwerveDriveConstants.SWERVE_SLOT_IDX, m_swerveGains.m_kF, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
        m_rightFrontSteerMotor.config_kP(SwerveDriveConstants.SWERVE_SLOT_IDX, m_swerveGains.m_kP, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
        m_rightFrontSteerMotor.config_kI(SwerveDriveConstants.SWERVE_SLOT_IDX, m_swerveGains.m_kI, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
        m_rightFrontSteerMotor.config_kD(SwerveDriveConstants.SWERVE_SLOT_IDX, m_swerveGains.m_kD, SwerveDriveConstants.SWERVE_TIMEOUT_MS);

        m_leftBackSteerMotor.selectProfileSlot(SwerveDriveConstants.SWERVE_SLOT_IDX, SwerveDriveConstants.SWERVE_PID_LOOP_IDX);
        m_leftBackSteerMotor.config_kF(SwerveDriveConstants.SWERVE_SLOT_IDX, m_swerveGains.m_kF, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
        m_leftBackSteerMotor.config_kP(SwerveDriveConstants.SWERVE_SLOT_IDX, m_swerveGains.m_kP, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
        m_leftBackSteerMotor.config_kI(SwerveDriveConstants.SWERVE_SLOT_IDX, m_swerveGains.m_kI, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
        m_leftBackSteerMotor.config_kD(SwerveDriveConstants.SWERVE_SLOT_IDX, m_swerveGains.m_kD, SwerveDriveConstants.SWERVE_TIMEOUT_MS);

        m_rightBackSteerMotor.selectProfileSlot(SwerveDriveConstants.SWERVE_SLOT_IDX, SwerveDriveConstants.SWERVE_PID_LOOP_IDX);
        m_rightBackSteerMotor.config_kF(SwerveDriveConstants.SWERVE_SLOT_IDX, m_swerveGains.m_kF, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
        m_rightBackSteerMotor.config_kP(SwerveDriveConstants.SWERVE_SLOT_IDX, m_swerveGains.m_kP, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
        m_rightBackSteerMotor.config_kI(SwerveDriveConstants.SWERVE_SLOT_IDX, m_swerveGains.m_kI, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
        m_rightBackSteerMotor.config_kD(SwerveDriveConstants.SWERVE_SLOT_IDX, m_swerveGains.m_kD, SwerveDriveConstants.SWERVE_TIMEOUT_MS);
        
    }*/



    // public void driveFieldRelative(double awayFromStation, double towardLeftBoundary, double rotate)
    // {
    //     var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(awayFromStation, towardLeftBoundary, 
    //         rotate * SwerveDriveConstants.RotationSpeed, /*get odometry angle*/)
    // }
}