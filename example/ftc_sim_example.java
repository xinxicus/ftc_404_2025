
/***********************************************************************
*                                                                      *
* OnbotJava Editor is still : beta! Please inform us of any bugs       |
* on our discord channel! https://discord.gg/e7nVjMM                   *
* Only BLOCKS code is submitted when in Arena                          |
*                                                                      *
***********************************************************************/

/**
 * FTC Robot Control Program - Team 404 2025 Season
 * 
 * This OpMode provides comprehensive robot control including:
 * - Manual drive control via keyboard or gamepad
 * - Autonomous operation mode
 * - AprilTag vision processing for navigation
 * - Artifact shooting mechanism
 * - Mecanum drive system for omnidirectional movement
 * 
 * The robot supports three operational modes:
 * Mode 0: Keyboard control for testing/debugging
 * Mode 1: Gamepad control for driver operation
 * Mode 2: Autonomous operation for competition
 */
public class MyFIRSTJavaOpMode extends LinearOpMode {
    
    // ========== MOTOR DECLARATIONS ==========
    // Legacy drive motors (may be unused in current configuration)
    DcMotor driveLeft;           // Left side drive motor
    DcMotor driveRight;          // Right side drive motor
    
    // Shooting system motor
    DcMotor shootwheel;          // Motor that spins the shooting wheel to launch artifacts
    
    // Mecanum drive system motors (4-wheel omnidirectional drive)
    DcMotor backLeftDrive;       // Back left wheel motor
    DcMotor backRightDrive;      // Back right wheel motor  
    DcMotor frontLeftDrive;      // Front left wheel motor
    DcMotor frontRightDrive;     // Front right wheel motor
    
    // ========== SERVO DECLARATIONS ==========
    Servo artifactstopper;       // Servo to control artifact feeding into shooter
    
    // ========== SENSOR DECLARATIONS ==========
    ColorSensor color1;          // Color sensor for detecting game elements
    DistanceSensor distance1;    // Distance sensor for obstacle detection
    BNO055IMU imu;              // Inertial Measurement Unit for robot orientation
    
    // ========== CONTROL AND VISION VARIABLES ==========
    // Note: Using 'var' type - these should ideally have explicit types
    var duration;                // Sleep duration for timed movements
    var myVisionPortalBuilder;   // Builder for creating vision portal
    var forward;                 // Forward/backward movement input (-1 to 1)
    var nArtifacts;             // Counter for artifacts remaining to shoot
    var turn;                   // Turning movement input (-1 to 1)
    var myAprilTagDetections;   // List of detected AprilTags
    var myVisionPortal;         // Vision portal instance for camera processing
    var isShooting;             // Flag to prevent movement during shooting
    var myAprilTagDetection;    // Currently processed AprilTag detection
    var shootPower;             // Power level for shooting motor (0.0 to 1.0)
    var mode;                   // Operating mode (0=keyboard, 1=gamepad, 2=auto)
    var maxDrivePower;          // Maximum power limit for drive motors
    var strafe;                 // Left/right strafing input (-1 to 1)
    var myApriltagProcessor;    // AprilTag processor for vision detection
    var myAprilTagProcessorBuilder; // Builder for AprilTag processor
    
    /**
     * Initial robot setup and configuration
     * 
     * This method performs essential robot initialization:
     * - Configures motor directions for proper mecanum drive operation
     * - Sets initial state flags
     * - Positions servo to prevent premature artifact release
     * 
     * Called once during robot initialization before operation begins.
     */
    public void inititalSetup(){
      // Configure left side motors to run in reverse direction
      // This ensures all wheels rotate correctly for forward movement
      frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
      backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
      
      // Initialize shooting state flag - robot starts ready to shoot
      isShooting = false;
      
      // Position artifact stopper servo to block artifacts from entering shooter
      // Position 0.2 holds artifacts in hopper until shooting sequence begins
      artifactstopper.setPosition(0.2);
    }
    
    /**
     * Initialize AprilTag vision processing system
     * 
     * Sets up the camera and AprilTag detection pipeline:
     * - Creates and configures vision portal for camera access
     * - Initializes AprilTag processor for tag recognition
     * - Links camera input to AprilTag processing
     * 
     * AprilTags are used for autonomous navigation and positioning.
     * The webcam must be properly configured in the robot configuration.
     */
    public void initializeVisionPortal(){
      // Create a new vision portal builder for camera configuration
      myVisionPortalBuilder = new VisionPortal.Builder();
      
      // Build the vision portal (this creates the initial instance)
      myVisionPortal = (myVisionPortalBuilder.build());
      
      // Configure the camera source - must match robot configuration name
      myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
      
      // Create AprilTag processor for detecting and analyzing tags
      myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
      myApriltagProcessor = (myAprilTagProcessorBuilder.build());
      
      // Add the AprilTag processor to the vision pipeline
      myVisionPortalBuilder.addProcessor(myApriltagProcessor);
    }
    
    /**
     * Select and execute the appropriate control mode
     * 
     * Chooses between three control modes based on the 'mode' variable:
     * - Mode 0: Keyboard control for testing and debugging
     * - Mode 1: Gamepad control for driver operation
     * - Mode 2: Autonomous control for competition
     * 
     * This is the main control dispatch method that determines how the robot operates.
     */
    public void pickMode(){
      if (mode == 0) {
        // Keyboard control mode - useful for testing without gamepad
        keyboardDrive();
      } else if (mode == 1) {
        // Gamepad control mode - standard teleop operation
        gamepadDrive();
      } else if (mode == 2) {
        // Autonomous mode - pre-programmed competition routine
        autoDrive();
      }
      // Note: Invalid mode values will result in no action
    }
    
    /**
     * Keyboard-based robot control loop
     * 
     * Enables robot control via keyboard input for testing and development.
     * Key mappings (using key codes):
     * - 108 (L) / 106 (J): Turn left/right
     * - 105 (I) / 107 (K): Move forward/backward  
     * - 111 (O) / 117 (U): Strafe right/left
     * - 112 (P): Trigger shooting sequence
     * 
     * Continuously runs while OpMode is active, processing inputs and updating telemetry.
     */
    public void keyboardDrive(){
      while (opModeIsActive()) {
        // Read turning input: L key (108) for left, J key (106) for right
        turn = keyboard.isPressed(108) - keyboard.isPressed(106);
        
        // Read forward/backward input: I key (105) for forward, K key (107) for backward
        forward = keyboard.isPressed(105) - keyboard.isPressed(107);
        
        // Read strafing input: O key (111) for right, U key (117) for left
        strafe = keyboard.isPressed(111) - keyboard.isPressed(117);
        
        // Apply movement inputs to drive motors
        processDriveInputs();
        
        // Check for shooting trigger (P key) - only shoot if not already shooting
        if (keyboard.isPressed(112) && !isShooting) {
          shoot();
        }
        
        // Update driver station with vision data
        displayVisionPortalData();
      }
    }
    
    /**
     * Gamepad-based robot control loop
     * 
     * Standard teleop control using gamepad for driver operation.
     * Control mapping:
     * - Left stick Y-axis: Forward/backward movement
     * - Left stick X-axis: Left/right strafing
     * - Right stick X-axis: Turning/rotation
     * - A button: Trigger shooting sequence
     * 
     * This provides intuitive control for drivers during competition.
     */
    public void gamepadDrive(){
      while (opModeIsActive()) {
        // Read gamepad inputs for robot movement
        turn = gamepad1.right_stick_x;      // Right stick X controls turning
        forward = gamepad1.left_stick_y;    // Left stick Y controls forward/backward
        strafe = gamepad1.left_stick_x;     // Left stick X controls strafing
        
        // Apply movement inputs to drive motors
        processDriveInputs();
        
        // Check for shooting trigger (A button) - only shoot if not already shooting
        if (gamepad1.a && !isShooting) {
          shoot();
        }
        
        // Update driver station with vision data
        displayVisionPortalData();
      }
    }
    
    /**
     * Autonomous operation sequence
     * 
     * Executes a pre-programmed autonomous routine for competition:
     * 1. Drive to the goal/scoring position
     * 2. Shoot three artifacts into the goal
     * 3. Drive to player station to collect more artifacts and return
     * 4. Shoot three more artifacts
     * 5. Fall back to keyboard control for remainder of match
     * 
     * This sequence is designed to maximize scoring in autonomous period.
     */
    public void autoDrive(){
      // Phase 1: Navigate to scoring position
      driveToGoal();
      
      // Phase 2: Score initial artifacts
      shootThreeArtifacts();
      
      // Phase 3: Collect additional artifacts from player station
      driveToPlayerStationAndBack();
      
      // Phase 4: Score collected artifacts
      shootThreeArtifacts();
      
      // Transition to manual control after autonomous completion
      // This allows for continued operation if autonomous finishes early
      keyboardDrive();
    }
    
    /**
     * Navigate to goal/scoring position during autonomous
     * 
     * Executes a timed sequence to position robot for scoring:
     * 1. Drive forward for 2.3 seconds to approach goal
     * 2. Turn right for 0.22 seconds to align with goal
     * 3. Brief pause to stabilize before shooting
     * 
     * Uses dead reckoning (time-based) navigation.
     * Timing may need adjustment based on field conditions and robot speed.
     */
    public void driveToGoal(){
      // Drive forward at full power toward the goal area
      forward = 1;
      processInputsAndSleep(2300);  // 2.3 seconds forward movement
      
      // Turn right to align with goal opening
      turn = -1;  // Negative turn value = right turn
      processInputsAndSleep(220);   // 0.22 seconds turning
      
      // Brief stabilization pause before shooting begins
      sleep(500);
    }
    
    /**
     * Travel to player station for artifact collection and return
     * 
     * Autonomous sequence for artifact collection:
     * 1. Drive backward to player station (2.8 seconds)
     * 2. Wait at player station for human player to load artifacts (10 seconds)
     * 3. Drive forward back to goal area (2.8 seconds)
     * 4. Brief pause to stabilize for next shooting sequence
     * 
     * The 10-second wait allows time for human player to load artifacts.
     */
    public void driveToPlayerStationAndBack(){
      // Drive backward to reach player station
      forward = -1;
      processInputsAndSleep(2800);  // 2.8 seconds backward movement
      
      // Wait at player station for human player to load artifacts
      // This timing should match competition rules for loading time
      sleep(10000);  // 10 second wait period
      
      // Return to goal area for scoring
      forward = 1;
      processInputsAndSleep(2800);  // 2.8 seconds forward movement
      
      // Stabilization pause before next shooting sequence
      sleep(500);
    }
    
    /**
     * Autonomous shooting sequence for three artifacts
     * 
     * Executes a controlled shooting sequence:
     * - Shoots exactly 3 artifacts in sequence
     * - Waits for each shot to complete before starting next
     * - Continues updating vision data during shooting
     * - Automatically stops when all artifacts are shot or OpMode ends
     * 
     * Used in autonomous mode for consistent scoring.
     */
    public void shootThreeArtifacts(){
      // Initialize counter for number of artifacts to shoot
      nArtifacts = 3;
      
      // Continue shooting until all artifacts are fired or OpMode stops
      while (opModeIsActive() && nArtifacts > 0) {
        // Only trigger new shot if not currently shooting
        // This prevents overlapping shooting sequences
        if (!isShooting) {
          shoot();                    // Execute shooting sequence
          nArtifacts -= 1;           // Decrement artifact counter
        }
        
        // Keep updating telemetry with vision data during shooting
        displayVisionPortalData();
      }
    }
    
    /**
     * Helper method for timed movement sequences
     * 
     * Simplifies autonomous movement by:
     * 1. Applying current movement inputs to motors
     * 2. Maintaining movement for specified duration
     * 3. Stopping all movement after time expires
     * 
     * This ensures clean start/stop behavior for autonomous sequences.
     * 
     * @param duration Time in milliseconds to maintain current movement
     */
    public void processInputsAndSleep(String duration){
      // Apply current forward, turn, and strafe values to drive motors
      processDriveInputs();
      
      // Maintain movement for the specified time period
      sleep(duration);
      
      // Clear all movement inputs to stop the robot
      forward = 0;
      turn = 0;
      strafe = 0;
      
      // Apply the stop command to motors
      processDriveInputs();
    }
    
    /**
     * Process movement inputs and apply to mecanum drive system
     * 
     * Converts normalized input values (-1 to 1) into motor power commands.
     * Implements mecanum drive kinematics for omnidirectional movement:
     * - Forward/backward: All wheels rotate same direction
     * - Turning: Left/right wheels rotate opposite directions
     * - Strafing: Diagonal wheel pairs rotate opposite directions
     * 
     * Motor power calculations combine all three movement types simultaneously.
     */
    public void processDriveInputs(){
      // Scale inputs by maximum drive power setting
      turn = turn * maxDrivePower;
      forward = forward * maxDrivePower;
      strafe = strafe * maxDrivePower;
      
      // Apply mecanum drive kinematics to calculate individual wheel powers
      // Each wheel power combines forward, turn, and strafe components
      frontLeftDrive.setPower(forward + turn + strafe);   // FL: + + +
      frontRightDrive.setPower(forward - turn - strafe);  // FR: + - -
      backLeftDrive.setPower(forward + turn - strafe);    // BL: + + -
      backRightDrive.setPower(forward - turn + strafe);   // BR: + - +
    }
    
    /**
     * Execute complete shooting sequence for one artifact
     * 
     * Carefully timed sequence to shoot a single artifact:
     * 1. Lock robot movement to prevent shooting interference
     * 2. Open artifact stopper to allow one artifact through
     * 3. Spin up shooting wheel to launch artifact
     * 4. Close artifact stopper to prevent multiple artifacts
     * 5. Stop shooting wheel and allow system to reset
     * 6. Re-enable robot movement
     * 
     * Total sequence time: ~2 seconds per artifact
     */
    public void shoot(){
      // Prevent robot movement during shooting for accuracy
      isShooting = true;
      
      // Open artifact stopper servo to allow one artifact to enter shooter
      artifactstopper.setPosition(0);   // Position 0 = open
      
      // Start shooting wheel at configured power level
      shootwheel.setPower(shootPower);
      sleep(250);  // Allow time for artifact to be shot
      
      // Close artifact stopper to prevent additional artifacts from entering
      artifactstopper.setPosition(0.2); // Position 0.2 = closed
      sleep(200);  // Brief delay to ensure stopper closes
      
      // Stop shooting wheel to conserve power and reduce wear
      shootwheel.setPower(0);
      sleep(1500); // Cooling/reset period before next shot
      
      // Re-enable robot movement - shooting sequence complete
      isShooting = false;
    }
    
    /**
     * Display AprilTag detection data on driver station telemetry
     * 
     * Processes and displays information from detected AprilTags:
     * - ID: Unique identifier of detected tag
     * - Range: Distance from robot to tag (useful for navigation)
     * - Yaw: Angular offset from robot's forward direction
     * 
     * This data helps drivers understand robot position relative to field elements
     * and can be used for autonomous navigation assistance.
     */
    public void displayVisionPortalData(){
      // Get current list of detected AprilTags from processor
      myAprilTagDetections = (myApriltagProcessor.getDetections());
      
      // Process each detected AprilTag and display its data
      for (String myAprilTagDetection2 : myAprilTagDetections) {
        myAprilTagDetection = myAprilTagDetection2;
        
        // Display key positioning data for each detected tag
        telemetry.addData("ID", (myAprilTagDetection.id));           // Tag identifier
        telemetry.addData("Range", (myAprilTagDetection.ftcPose.range)); // Distance to tag
        telemetry.addData("Yaw", (myAprilTagDetection.ftcPose.yaw));     // Angular offset
      }
      
      // Send all telemetry data to driver station display
      telemetry.update();
    }
    
    
    /**
     * Main OpMode execution method - called when OpMode is selected
     * 
     * This method handles complete robot initialization and execution:
     * 1. Maps all hardware components from robot configuration
     * 2. Performs initial robot setup and vision system initialization  
     * 3. Configures operational parameters
     * 4. Waits for competition start signal
     * 5. Begins robot operation in selected mode
     */
    @Override
    public void runOpMode() {
      // ========== HARDWARE MAPPING ==========
      // Map all motors from robot configuration to code variables
      // These names must match exactly with robot configuration file
      driveLeft = hardwareMap.get(DcMotor.class, "driveLeft");
      driveRight = hardwareMap.get(DcMotor.class, "driveRight");
      shootwheel = hardwareMap.get(DcMotor.class, "shootwheel");
      backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
      backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
      frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
      frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
      
      // Map servo for artifact control
      artifactstopper = hardwareMap.get(Servo.class, "artifactstopper");
      
      // Map sensors for environmental awareness
      color1 = hardwareMap.get(ColorSensor.class, "color1");
      distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
      imu = hardwareMap.get(BNO055IMU.class, "imu");
      
      // ========== ROBOT INITIALIZATION ==========
      // Configure robot for operation
      inititalSetup();           // Set motor directions and initial states
      initializeVisionPortal();  // Set up AprilTag detection system
      
      // ========== OPERATIONAL PARAMETERS ==========
      shootPower = 0.8;         // Shooting wheel power (80% of maximum)
      maxDrivePower = 1;        // Maximum drive motor power (100%)
      
      // Set operating mode: 0=keyboard, 1=gamepad, 2=autonomous
      // Mode 2 (autonomous) is set for competition use
      mode = 2;
      
      // ========== COMPETITION START ==========
      // Wait for start button press or competition field start signal
      waitForStart();
      
      // Begin robot operation in selected mode
      pickMode();
    }
    
}
