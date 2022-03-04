// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.cscore.CvSink;
//import edu.wpi.first.cscore.CvSource;
//import edu.wpi.first.cscore.UsbCamera;
import com.team871.hid.HIDAxis;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import org.opencv.core.Mat;
//import org.opencv.imgproc.Imgproc;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private static final String DEFAULT_AUTO = "Default";
    private static final String CUSTOM_AUTO = "My Auto";
    private String autoSelected;
    private final SendableChooser<String> chooser = new SendableChooser<>();

    private XBController xbox;
    private DriveTrain drive;
    private IRobot config;
    private Collector collector;
    private Shooter shooter;
    private Hanger hanger;
    
    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        chooser.setDefaultOption("Default Auto", DEFAULT_AUTO);
        chooser.addOption("My Auto", CUSTOM_AUTO);
        SmartDashboard.putData("Auto choices", chooser);

        xbox = new XBController(0, 1);
        config = new RobotConfig();
        drive = new DriveTrain(config.getFrontLeftMotor(), config.getFrontRightMotor(), config.getRearLeftMotor(), config.getRearRightMotor());
        collector = new Collector(config);
        shooter = new Shooter(config, 6);
        hanger = new Hanger(config);

//        new Thread(() -> {
//        UsbCamera cam = CameraServer.startAutomaticCapture();
//        cam.setResolution(640, 480);
//        CvSink cvSink = CameraServer.getVideo();
//        CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);
//        Mat source = new Mat();
//        Mat output = new Mat();
//        while (!Thread.interrupted()) {
//            if (cvSink.grabFrame(source) == 0) {
//                continue;
//            }
//            Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
//            outputStream.putFrame(output);
//        }
//        }).start();
    }
    
    
    /**
     * This method is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {}
    
    
    /**
     * This autonomous (along with the chooser code above) shows how to select between different
     * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
     * uncomment the getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to the switch structure
     * below with additional strings. If using the SendableChooser make sure to add them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
    }
    
    
    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }
    
    
    /** This method is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
    }
    
    
    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        //TODO: Determine whether update is needed
        config.update(xbox);
        drive.driveMecanum(xbox.getDriveX(), xbox.getDriveY(), xbox.getDriveZ(), xbox.getFireButton()/*, xbox.getDriveY(), xbox.getDriveX()*/);
        shooter.fireUpdate(xbox.getFireButton());
        //shooter.setRMP(0);
        collector.activateCollector(xbox.getCollectorAxis());
        collector.invertCollector(xbox.getRegurgitateButton());
        // shooter.limeLight();
        //TODO: UNCOMMENT
//        hanger.update(xbox.getClimbButton(), xbox.emergencyStopClimb());
        //hanger.dirveClimber(xbox.getClimbGrabAxis(), xbox.getClimbSwingAxis());
        hanger.calcatePitchPID(xbox.getClimbSwingAxis());
    }
    
    
    /** This method is called once when the robot is disabled. */
    @Override
    public void disabledInit() {}
    
    
    /** This method is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}
    
    
    /** This method is called once when test mode is enabled. */
    @Override
    public void testInit() {}
    
    
    /** This method is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
}
