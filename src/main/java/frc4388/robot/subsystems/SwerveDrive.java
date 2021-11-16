public class SwerveDrive
{
    SwerveDriveKinematics m_kinematics;

    public SwerveDrive()
    {
        double halfWidth = SwerveDriveConstants.WIDTH / 2.d;
        double halfHeight = SwerveDriveConstants.HEIGHT / 2.d;

        Translation2d m_frontLeftLocation = new Translation2d(halfHeight, halfWidth);
        Translation2d m_frontRightLocation = new Translation2d(halfHeight, -halfWidth);
        Translation2d m_backLeftLocation = new Translation2d(-halfHeight, halfWidth);
        Translation2d m_backRightLocation = new Translation2d(-halfHeight, -halfWidth);
        
        m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    }

    public void drive(double strafeX, double strafeY, double rotate)
    {
        //https://jacobmisirian.gitbooks.io/frc-swerve-drive-programming/content/chapter1.html
        var speeds = new ChassisSpeeds(strafeX, strafeY, rotate * SwerveDriveConstants.RotationSpeed /*in rad/s */);
        // Convert to module states
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        // Front left module state
        SwerveModuleState frontLeft = SwerveModuleState.optimize(moduleStates[0], new Rotation2d(/*get encoder positions*/));

        // Front right module state
        SwerveModuleState frontRight = SwerveModuleState.optimize(moduleStates[1], new Rotation2d(/*get encoder positions*/));

        // Back left module state
        SwerveModuleState backLeft = SwerveModuleState.optimize(moduleStates[2], new Rotation2d(/*get encoder positions*/));

        // Back right module state
        SwerveModuleState backRight = SwerveModuleState.optimize(moduleStates[3], new Rotation2d(/*get encoder positions*/));
    }

    public void driveFieldRelative(double awayFromStation, double towardLeftBoundary, double rotate)
    {
        var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(awayFromStation, towardLeftBoundary, 
            rotate * SwerveDriveConstants.RotationSpeed, /*get odometry angle*/)
    }
}