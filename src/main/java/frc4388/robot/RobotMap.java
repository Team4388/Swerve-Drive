/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc4388.robot.Constants.ArcadeDriveConstants;
import frc4388.robot.Constants.LEDConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.utility.RobotGyro;

/**
 * Defines and holds all I/O objects on the Roborio. This is useful for unit
 * testing and modularization.
 */
public class RobotMap {

    public RobotMap() {
        configureLEDMotorControllers();
        configureArcadeDriveMotorControllers();
    }

    /* LED Subsystem */
    public final Spark LEDController = new Spark(LEDConstants.LED_SPARK_ID);

    void configureLEDMotorControllers() {
        
    }

    /* ArcadeDrive Subsystem */
    public final WPI_TalonFX leftFrontMotor = new WPI_TalonFX(ArcadeDriveConstants.DRIVE_LEFT_FRONT_CAN_ID);
    public final WPI_TalonFX rightFrontMotor = new WPI_TalonFX(ArcadeDriveConstants.DRIVE_RIGHT_FRONT_CAN_ID);
    public final WPI_TalonFX leftBackMotor = new WPI_TalonFX(ArcadeDriveConstants.DRIVE_LEFT_BACK_CAN_ID);
    public final WPI_TalonFX rightBackMotor = new WPI_TalonFX(ArcadeDriveConstants.DRIVE_RIGHT_BACK_CAN_ID);
    public final DifferentialDrive driveTrain = new DifferentialDrive(leftFrontMotor, rightFrontMotor);
    public final RobotGyro gyroDrive = new RobotGyro(new PigeonIMU(ArcadeDriveConstants.DRIVE_PIGEON_ID));

    void configureArcadeDriveMotorControllers() {

        /* factory default values */
        leftFrontMotor.configFactoryDefault();
        rightFrontMotor.configFactoryDefault();
        leftBackMotor.configFactoryDefault();
        rightBackMotor.configFactoryDefault();

        /* set back motors as followers */
        leftBackMotor.follow(leftFrontMotor);
        rightBackMotor.follow(rightFrontMotor);

        /* set neutral mode */
        leftFrontMotor.setNeutralMode(NeutralMode.Brake);
        rightFrontMotor.setNeutralMode(NeutralMode.Brake);
        leftFrontMotor.setNeutralMode(NeutralMode.Brake);
        rightFrontMotor.setNeutralMode(NeutralMode.Brake);

        /* flip input so forward becomes back, etc */
        leftFrontMotor.setInverted(false);
        rightFrontMotor.setInverted(false);
        leftBackMotor.setInverted(InvertType.FollowMaster);
        rightBackMotor.setInverted(InvertType.FollowMaster);
    }
    /* Swerve Subsystem */
    public final WPI_TalonFX leftFrontSteerMotor = new WPI_TalonFX(SwerveDriveConstants.LEFT_FRONT_STEER_CAN_ID);
    public final WPI_TalonFX leftFrontWheelMotor = new WPI_TalonFX(SwerveDriveConstants.LEFT_FRONT_WHEEL_CAN_ID);
    public final WPI_TalonFX rightFrontSteerMotor = new WPI_TalonFX(SwerveDriveConstants.RIGHT_FRONT_STEER_CAN_ID);
    public final WPI_TalonFX rightFrontWheelMotor = new WPI_TalonFX(SwerveDriveConstants.RIGHT_FRONT_WHEEL_CAN_ID);
    public final WPI_TalonFX leftBackSteerMotor = new WPI_TalonFX(SwerveDriveConstants.LEFT_BACK_STEER_CAN_ID);
    public final WPI_TalonFX leftBackWheelMotor = new WPI_TalonFX(SwerveDriveConstants.LEFT_BACK_WHEEL_CAN_ID);
    public final WPI_TalonFX rightBackSteerMotor = new WPI_TalonFX(SwerveDriveConstants.RIGHT_BACK_STEER_CAN_ID);
    public final WPI_TalonFX rightBackWheelMotor = new WPI_TalonFX(SwerveDriveConstants.RIGHT_BACK_WHEEL_CAN_ID);
    public final CANCoder leftFrontEncoder = new CANCoder(SwerveDriveConstants.LEFT_FRONT_STEER_CAN_ENCODER_ID);
    public final CANCoder rightFrontEncoder = new CANCoder(SwerveDriveConstants.RIGHT_FRONT_STEER_CAN_ENCODER_ID);
    public final CANCoder leftBackEncoder = new CANCoder(SwerveDriveConstants.LEFT_BACK_STEER_CAN_ENCODER_ID);
    public final CANCoder rightBackEncoder = new CANCoder(SwerveDriveConstants.RIGHT_BACK_STEER_CAN_ENCODER_ID);

    void configureSwerveMotorController() {
    }
}
