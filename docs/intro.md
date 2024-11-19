# Goals
* Learn the structure of our codebase (beginning 2024 offseason)
* Learn how to organize and collaborate unstable code
* Learn how to use available tools to run and debug code

# General Code Structure
Note the different directories and files, only one is relevant for now
* `src/` contains all source code
* `src/main/java/frc/robot/` is where the "actual" robot code is located
    * note the source files (`Robot.java`, `RobotContainer.java`) and sub-directories (`subsystems/`, `subsystems/flywheels/`)

# The beginning
`Main.java` is ostensibly the start of the program, but only contains a single line of code `RobotBase.startRobot(Robot::new);`

1. `Main.java` calls `Robot.java`, where the important start-up code is located
    - `Robot::new` calls the Robot class` constructor, but `Robot.java` lacks one(?).
2. `robotInit()` calls a lot of AdvantageKit logging functions. The `RobotContainer` class contains the bulk of the robot set-up.
```
@Override
public void robotInit() {
    ...
    robotContainer = new RobotContainer();
}
```
3. `robotPeriodic` is called every 20ms (50Hz) and runs a single method. This is the [Command Scheduler](https://docs.wpilib.org/en/stable/docs/software/commandbased/command-scheduler.html), the class responsible for running all logic.
```
@Override
public void robotPeriodic() {
    CommandScheduler.getInstance().run();
}
```

# What is `RobotContainer.java`?
- `RobotContainer` organizes all component code for the robot into a single class. 
- Here, all (mechanical and virtual) subsystems are integrated with teleop and autonomous inputs.

- The constructor initializes various subsystems.
- `configureBindings()` sets up driver inputs

# Subsystems
To accomodate AdvantageKit logging, code is organized to keep strong distinctions between control logic and hardware interaction (read [AdvantageKit Docs](https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/docs/docs))

- control logic:
- hardware i/o (using vendor api): 
