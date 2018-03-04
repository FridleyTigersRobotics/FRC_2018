/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include <Drive/DifferentialDrive.h>
#include <Joystick.h>
#include <Spark.h>
#include <Timer.h>
#include <iostream>

//Robot specific includes
#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Timer.h>
#include <ADXRS450_Gyro.h>


// ***************************************************************************
//   Function:    deadband
//
//   Description: Applies a deadband to the input value.
//                (output) will be zero when (-zone) <= (value) <= (+zone)
//                (output) will be one at (value) == 1
//
//                         output        . (1,1)
//                            |         /
//                            |        /
//                            |       /
//                    -zone   |      /
//             ---------+-----+-----+--------- value
//                     /      |   +zone
//                    /       |
//                   /        |
//                  /         |
//                 . (-1,-1)
//
// ***************************************************************************
double deadband(
    double value,
    double zone
)
{
    double output;
    if ( fabs( value ) < zone )
    {
        output = 0.0;
    }
    else
    {
        // After the deadband, start with value = 0
        if( value < 0 )
        {
            output = ( value + zone ) / ( 1.0 - zone );
        }
        else
        {
            output = ( value - zone ) / ( 1.0 - zone );
        }
    }

    return output;
}



// ***************************************************************************
//   Function:    saturate
//
//   Description: Saturate the input value with the specified bounds.
//                When (value) > (max), (output) == (max)
//                When (value) < (min), (output) == (min)
//                Otherwise (output) == (value)
//
// ***************************************************************************
double saturate(
    double value,
    double min,
    double max
)
{
    return std::min( std::max( value, min ), max );
}



// ***************************************************************************
//   Function:    saturate
//
//   Description: Saturate the input value with the specified bounds.
//                When (value) > (max), (output) == (max)
//                When (value) < (min), (output) == (min)
//                Otherwise (output) == (value)
//
// ***************************************************************************
double minAbsLimitValue(
    double value,
    double minAbsValue
)
{
    double limitedValue = 0.0;

    if ( fabs( value ) > fabs( minAbsValue ) )
    {
        limitedValue = value;
    }
    else
    {
        if ( value < 0.0 )
        {
            limitedValue = -fabs( minAbsValue );
        }
        else
        {
            limitedValue = fabs( minAbsValue );
        }
    }

    return limitedValue;
}





// ***************************************************************************
//   Class:       Robot
//
//   Description: The class defining our Robot
//
// ***************************************************************************
class Robot : public frc::IterativeRobot
{
public:

    // ***************************************************************************
    //   Method:      RobotInit
    //
    //   Description: Method that is called once when the robot is first powered on.
    //
    // ***************************************************************************
    void RobotInit()
    {
        positionChooser.AddDefault( "1) Nothing",  NO_POSITION );
        positionChooser.AddObject( "2) Left",  LEFT_POSITION );
        positionChooser.AddObject( "3) Middle", MIDDLE_POSITION );
        positionChooser.AddObject( "4) Right",  RIGHT_POSITION );
        frc::SmartDashboard::PutData( "Position Select", &positionChooser);

        actionChooser.AddDefault( "1) Do nothing", DO_NOTHING );
        actionChooser.AddObject( "2) Cross Line", CROSS_LINE );
        actionChooser.AddObject( "3) Load Switch", LOAD_SWITCH );
        actionChooser.AddObject( "4) Load Scale", LOAD_SCALE );
        frc::SmartDashboard::PutData( "Auto Action", &actionChooser);

        cubeLiftEncoder->SetMaxPeriod(1);
        cubeLiftEncoder->SetMinRate(1);
        cubeLiftEncoder->SetDistancePerPulse(1);
        cubeLiftEncoder->SetReverseDirection(false);
        cubeLiftEncoder->SetSamplesToAverage(1);
        gyro.Calibrate();
    }


    // ***************************************************************************
    //   Method:      AutonomousInit
    //
    //   Description: Method that is called once when the robot enters the
    //                Autonomous mode.
    //
    // ***************************************************************************
    void AutonomousInit() override
    {
        std::string gameData;
        gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
        auto_position_t startingPosition = positionChooser.GetSelected();
        auto_action_t   autoAction       = actionChooser.GetSelected();

        std::string autoPositionString = "default";
        std::string autoActionString   = "default";

        cubeLiftEncoder.Reset();

        Auto_Initalize();


        if ( startingPosition == MIDDLE_POSITION &&
             autoAction       == CROSS_LINE      &&
             gameData[0]      == 'L'
            )
        {
            AutoCommandList = DriveTest;
        }


        // left position, load left switch
        if ( startingPosition == LEFT_POSITION &&
             autoAction       == LOAD_SWITCH   &&
             gameData[0]      == 'L'
            )
        {
            AutoCommandList = AutoCommandList2;
        }

        // right position, load right switch
        if ( startingPosition == RIGHT_POSITION &&
             autoAction       == LOAD_SWITCH    &&
             gameData[0]      == 'R'
            )
        {
            AutoCommandList = AutoCommandList3;
        }

        // left position, load left scale
        if ( startingPosition == LEFT_POSITION &&
             autoAction       == LOAD_SCALE    &&
             gameData[1]      == 'L'
            )
        {
            AutoCommandList = AutoCommandList4;
        }

        //left position, load right switch
        if ( startingPosition == LEFT_POSITION &&
             autoAction       == LOAD_SWITCH    &&
             gameData[0]      == 'R'
            )
        {
            AutoCommandList = AutoCommandList12;
        }

        // middle position, load left switch
        if ( startingPosition == MIDDLE_POSITION &&
             autoAction       == LOAD_SWITCH    &&
             gameData[0]      == 'L'
            )
        {
            AutoCommandList = AutoCommandList6;
        }

        // middle position, load right switch
        if ( startingPosition == MIDDLE_POSITION &&
             autoAction       == LOAD_SWITCH    &&
             gameData[0]      == 'R'
            )
        {
            AutoCommandList = AutoCommandList7;
        }

        //right position, load left scale
        if ( startingPosition == RIGHT_POSITION &&
             autoAction       == LOAD_SCALE    &&
             gameData[1]      == 'L'
            )
        {
            AutoCommandList = AutoCommandList15;
        }
/*
        // right position, load right scale
        if ( startingPosition == RIGHT_POSITION &&
             autoAction       == LOAD_SCALE    &&
             gameData[0]      == 'L'
            )
        {
            AutoCommandList = AutoCommandList5;
        }

        // middle position, load left scale
        if ( startingPosition == MIDDLE_POSITION &&
             autoAction       == LOAD_SCALE    &&
             gameData[0]      == 'L'
            )
        {
            AutoCommandList = AutoCommandList8;
        }
        // middle position, load right scale
        if ( startingPosition == MIDDLE_POSITION &&
             autoAction       == LOAD_SCALE    &&
             gameData[0]      == 'L'
            )
        {
            AutoCommandList = AutoCommandList9;
        }

        //middle position, auto line only (from left)
        if ( startingPosition == MIDDLE_POSITION &&
             autoAction       == CROSS_LINE    &&
             gameData[0]      == 'L'
            )
        {
            AutoCommandList = AutoCommandList10;
        }

        //middle position, auto line only (from right)
        if ( startingPosition == MIDDLE_POSITION &&
             autoAction       == CROSS_LINE    &&
             gameData[0]      == 'L'
            )
        {
            AutoCommandList = AutoCommandList11;
        }

        //right position, load left switch
        if ( startingPosition == RIGHT_POSITION &&
             autoAction       == LOAD_SWITCH    &&
             gameData[0]      == 'L'
            )
        {
            AutoCommandList = AutoCommandList13;
        }

        //left position, load right scale
        if ( startingPosition == LEFT_POSITION &&
             autoAction       == LOAD_SCALE    &&
             gameData[0]      == 'R'
            )
        {
            AutoCommandList = AutoCommandList14;
        }



        //left position, auto line only
        if ( startingPosition == LEFT_POSITION &&
             autoAction       == CROSS_LINE    &&
             gameData[0]      == 'L'
            )
        {
            AutoCommandList = AutoCommandList16;
        }

        //right position, auto line only
        if ( startingPosition == RIGHT_POSITION &&
             autoAction       == CROSS_LINE    &&
             gameData[0]      == 'L'
            )
        {
            AutoCommandList = AutoCommandList17;
        }

*/

        // Debug info below here
        if ( startingPosition == MIDDLE_POSITION )
        {
            autoPositionString = "Middle";
        }
        else if ( startingPosition == LEFT_POSITION )
        {
            autoPositionString = "Left";
        }
        else if ( startingPosition == RIGHT_POSITION )
        {
            autoPositionString = "Right";
        }
        else
        {
            autoPositionString = "Do Nothing";
        }

        if ( autoAction == CROSS_LINE )
        {
            autoActionString = "Cross Line";
        }
        else if ( autoAction == LOAD_SWITCH )
        {
            autoActionString = "Load Switch";
        }
        else if ( autoAction == LOAD_SCALE )
        {
            autoActionString = "Load Scale";
        }
        else
        {
            autoActionString = "Do nothing";
        }

        std::cout << "\nAuto Information\n";
        std::cout << "Auto Starting Position: " << autoPositionString << "\n";
        std::cout << "Auto Action           : " << autoActionString << "\n";
        std::cout << "Game Data             : " << gameData << "\n";

        std::cout << std::flush;


    }


    // ***************************************************************************
    //   Method:      AutonomousPeriodic
    //
    //   Description: Method that is called periodically during the Autonomous mode.
    //
    // ***************************************************************************
    void AutonomousPeriodic()
    {
        Auto_ServiceCommandList();
    }


    // ***************************************************************************
    //   Method:      TeleopInit
    //
    //   Description: Method that is called once when the robot enters Teleop mode.
    //
    // ***************************************************************************
    void TeleopInit()
    {
        maxAbsMotorChange = 0.05;
    }


    // ***************************************************************************
    //   Method:      TeleopPeriodic
    //
    //   Description: Method that is called periodically during the Teleop mode.
    //
    // ***************************************************************************
    void TeleopPeriodic()
    {
        std::cout << gyro.GetAngle();
        std::cout << '\n';
        std::cout << std::flush;
        Tele_UpdateDriveSystem();
        Tele_UpdateCubeLift();
        Tele_UpdateCubeIntake();
        Tele_UpdateWinch();
    }


private:
    typedef enum
    {
         LEFT_POSITION,
         MIDDLE_POSITION,
         RIGHT_POSITION,
         NO_POSITION
    } auto_position_t;

    typedef enum
    {
         CROSS_LINE,
         LOAD_SWITCH,
         LOAD_SCALE,
         DO_NOTHING
    } auto_action_t;


    frc::SendableChooser<auto_position_t> positionChooser;
    frc::SendableChooser<auto_action_t> actionChooser;


    std::string m_autoSelected;

    // Digital I/O
    frc::DigitalInput winchLimiterTop{ 0 };
    frc::DigitalInput winchLimiterBot{ 1 };
    frc::Encoder *cubeLiftEncoder = new Encoder{ 2, 3 };
    frc::DigitalInput liftLimiterBot{ 4 };

    // PWM Motors
    frc::VictorSP mWinchA{ 0 };   // -1 = Up
    frc::VictorSP mCubeLift{ 1 }; // -1 = Up
    frc::VictorSP mWinchB{ 2 };   // 1 = Up
    frc::VictorSP mDriveRA{ 3 };  // Right Side ( -1 = forward )
    frc::VictorSP mDriveLA{ 4 };  // Left side ( -1 = backwards )
    frc::VictorSP mDriveRB{ 5 };  // Right Side ( -1 = Forward )
    frc::VictorSP mDriveLB{ 6 };  // Left Side ( -1 = Backwards )
    frc::Spark brakeMotor{ 7 };

    // CAN Motors
    TalonSRX CubeIntakeR{ 0 }; // Cube Intake R ( -1 = out )
    TalonSRX CubeIntakeL{ 1 }; // Cube Intake L ( -1 = in )

    // Sensors
    frc::ADXRS450_Gyro gyro{ frc::SPI::Port::kOnboardCS0  }; // Right turn -> positive angle

    // Controllers
    frc::XboxController XboxController{ 0 };
    frc::XboxController LogitechController{ 1 };



    typedef enum{
        HoldPosition,
        MoveUp,
        MoveDown
    } lift_state_t;

    lift_state_t currentLiftState = HoldPosition;
    int desiredLiftPostion = 0;






// ***************************************************************************
//
//              Generic System Control Functions
//
// ***************************************************************************


    double currentLeftMotorSpeed  = 0.0;
    double currentRightMotorSpeed = 0.0;



    double maxAbsMotorChange = 0.05;

    // ***************************************************************************
    //   Method:      SetDriveMotorSpeeds
    //
    //   Description: Method for setting the drive motor speeds.
    //
    // ***************************************************************************
    void SetDriveMotorSpeeds(
        double leftMotorSpeed,
        double rightMotorSpeed
    )
    {


        // Calculate the motor speeds for the specified input
        double const leftMotorSpeedSaturated  = saturate( leftMotorSpeed, -1, 1 );
        double const rightMotorSpeedSaturated = saturate( rightMotorSpeed, -1, 1 );

        double const leftSpeedDiff  = leftMotorSpeedSaturated  - currentLeftMotorSpeed;
        double const rightSpeedDiff = rightMotorSpeedSaturated - currentRightMotorSpeed;

        double const leftSpeedChange  = saturate( leftSpeedDiff,  -maxAbsMotorChange, maxAbsMotorChange );
        double const rightSpeedChange = saturate( rightSpeedDiff, -maxAbsMotorChange, maxAbsMotorChange );

        double const newLeftMotorSpeed  = currentLeftMotorSpeed  + leftSpeedChange;
        double const newRightMotorSpeed = currentRightMotorSpeed + rightSpeedChange;


        // Update motor controllers
        mDriveLA.Set( newLeftMotorSpeed );
        mDriveLB.Set( newLeftMotorSpeed );
        mDriveRA.Set( -newRightMotorSpeed );
        mDriveRB.Set( -newRightMotorSpeed );

        currentLeftMotorSpeed  = newLeftMotorSpeed;
        currentRightMotorSpeed = newRightMotorSpeed;


    }



    // ***************************************************************************
    //   Method:      SetCubeIntakeMotorSpeed
    //
    //   Description: Method for setting the cube intake motor speeds.
    //
    // ***************************************************************************
    void SetCubeIntakeMotorSpeed(
        double motorSpeed
    )
    {
        CubeIntakeR.Set( ControlMode::PercentOutput,  motorSpeed );
        CubeIntakeL.Set( ControlMode::PercentOutput, -motorSpeed );
    }



    // ***************************************************************************
    //   Method:      SetWinchMotorSpeed
    //
    //   Description: Method for setting the robot lift winch motor speeds.
    //
    // ***************************************************************************
    void SetWinchMotorSpeed(
        double winchMotorSpeed
    )
    {
        // TODO: Add limit code in here?
        mWinchA.Set( -winchMotorSpeed );
        mWinchB.Set(  winchMotorSpeed );
    }




    // ***************************************************************************
    //   Method:      SetCubeLiftState
    //
    //   Description: Specify the state for the cube lift to perform.
    //
    // ***************************************************************************
    void SetCubeLiftState(
        lift_state_t newLiftState,
        double       liftSpeed
    )
    {
        bool const debugCubeLift = false;

        double cubeLiftMotorSpeed;

        // Value of the limiter is nominally one, and zero when limit is hit.
        bool const atBotLimitLift = (liftLimiterBot.Get() == 0);

        // Update this limit for the new motor/encoder.
        int const liftPositionLimit = 5270;

        int currentLiftPosition = cubeLiftEncoder->Get();

        // Encoder value defines top value.
        bool const atTopLimitLift = ( currentLiftPosition >= liftPositionLimit );

        // Need to override the input state and hold position
        // if we are at one of the limits.
        if ( ( ( newLiftState == MoveDown ) && atBotLimitLift ) || ( ( newLiftState == MoveUp ) && atTopLimitLift ) )
        {
            newLiftState = HoldPosition;
        }

        // Changing from a moving state to holding the position.
        // Need to update the desired lift position.
        if ( ( newLiftState     == HoldPosition ) &&
             ( currentLiftState != HoldPosition ) )
        {
            desiredLiftPostion = currentLiftPosition;
        }

        currentLiftState = newLiftState;

        switch ( currentLiftState )
        {
            case MoveUp:
            {
                if ( debugCubeLift )
                {
                    std::cout << "MoveUp";
                }

                cubeLiftMotorSpeed = -1 * ( fabs( speed ) );
                break;
            }

            case MoveDown:
            {
                if ( debugCubeLift )
                {
                    std::cout << "MoveDown";
                }
                cubeLiftMotorSpeed = 0.8 * ( fabs( speed ) );
                break;
            }

            case HoldPosition:
            default:
            {
                int const liftPositionError = currentLiftPosition - desiredLiftPostion;

                if ( debugCubeLift )
                {
                    std::cout << "HoldPosition  ";
                    std::cout << desiredLiftPostion;
                    std::cout << "   ";
                    std::cout << currentLiftPosition;
                }

                if ( liftPositionError < 0 )
                {
                    cubeLiftMotorSpeed = saturate( (double)liftPositionError * 0.1, -1.0, 1.0 );
                }
                else
                {
                    cubeLiftMotorSpeed = 0.0;
                }

                break;
            }
        }

        if ( debugCubeLift )
        {
            std::cout << "\n";
            std::cout << std::flush;
        }

        // Update motor controllers
        mCubeLift.Set( cubeLiftMotorSpeed );
    }








// ***************************************************************************
//
//              Teleop Functions
//
// ***************************************************************************

    double prevXValue = 0.0;

    // ***************************************************************************
    //   Method:      Tele_UpdateDriveSystem
    //
    //   Description: Method for controlling the drive system during teleop.
    //                Controller input values are read and drive motor speeds
    //                are set based on the controller input values.
    //
    // ***************************************************************************
    void Tele_UpdateDriveSystem( void )
    {
        bool const debugDrive = false;
        frc::XboxController::JoystickHand const driveStickHand = frc::XboxController::JoystickHand::kLeftHand;

        // Retrieve the Left stick X and Y positions
        double const leftStickXPosition = -1.0 * XboxController.GetX( driveStickHand );
        // Multiply by negative one to get forward on the stick to be positive,
        // and backwards on the sticl be negative.
        double const leftStickYPosition = -1.0 * XboxController.GetY( driveStickHand );

        if ( debugDrive )
        {
            std::cout << leftStickXPosition;
            std::cout << '\t';
            std::cout << leftStickYPosition;
            std::cout << '\n';
            std::cout << std::flush;
        }

        // Apply a deadband to the stick position.
        double const deadbandEnd        = 0.22;
        double const xValueWithDeadband = deadband( leftStickXPosition, deadbandEnd );
        double const yValueWithDeadband = deadband( leftStickYPosition, deadbandEnd );

        double const maxXValueChange = 0.10;
        double const xValueDiff = saturate( xValueWithDeadband - prevXValue, -maxXValueChange, maxXValueChange ) ;

        double const xValueAdjusted = prevXValue + xValueDiff;
        double const yValueAdjusted = yValueWithDeadband;

        prevXValue = xValueAdjusted;

        // Calculate the motor speeds for the specified input
        double const driveLeftMotorSpeed  = yValueAdjusted - xValueAdjusted;
        double const driveRightMotorSpeed = yValueAdjusted + xValueAdjusted;

        // Update motor controllers
        SetDriveMotorSpeeds(
            driveLeftMotorSpeed,
            driveRightMotorSpeed
        );
    }


    // ***************************************************************************
    //   Method:      Tele_UpdateCubeLift
    //
    //   Description: Method for controlling the cube lift system during teleop.
    //                Controller input values are read and cube lift motor speeds
    //                are set based on the controller input values.
    //
    // ***************************************************************************
    void Tele_UpdateCubeLift( void )
    {
        bool const debugCubeLift = false;
        frc::XboxController::JoystickHand const cubeLiftStickHand = frc::XboxController::JoystickHand::kRightHand;

        // Retrieve the stick Y position
        double const stickYPosition = XboxController.GetY( cubeLiftStickHand );

        lift_state_t newLiftState;

        if ( debugCubeLift )
        {
            std::cout << stickYPosition;
            std::cout << "   ";
        }

        double const liftSpeed = deadband( stickYPosition, 0.5 );

        // Determine the new cube lift state based on the controller input.
        if ( stickYPosition > 0.5 )
        {
            newLiftState = MoveDown;
        }
        else if ( stickYPosition < -0.5 )
        {
            newLiftState = MoveUp;
        }
        else
        {
            newLiftState = HoldPosition;
        }

        SetCubeLiftState(
            newLiftState,
            liftSpeed
        );
    }



    // ***************************************************************************
    //   Method:      Tele_UpdateCubeIntake
    //
    //   Description: Method for controlling the cube intake system during teleop.
    //                Controller input values are read and cube intake motor speeds
    //                are set based on the controller input values.
    //
    // ***************************************************************************
    void Tele_UpdateCubeIntake( void )
    {
        frc::XboxController::JoystickHand const inHand  = frc::XboxController::JoystickHand::kRightHand;
        frc::XboxController::JoystickHand const outHand = frc::XboxController::JoystickHand::kLeftHand;

        // Retrieve the stick Y position
        double const inTriggerPosition  = XboxController.GetTriggerAxis( inHand  );
        double const outTriggerPosition = XboxController.GetTriggerAxis( outHand );

        // Apply deadband
        double const deadbandEnd       = 0.22;
        double const inTriggerPositionWithDeadband  = deadband( inTriggerPosition, deadbandEnd );
        double const outTriggerPositionWithDeadband = deadband( outTriggerPosition, deadbandEnd );

        // Calculate the motor speeds for the specified input
        double const cubeIntakeMotorSpeed  = inTriggerPositionWithDeadband - outTriggerPositionWithDeadband;

        // Update motor controllers
        SetCubeIntakeMotorSpeed( cubeIntakeMotorSpeed );
    }




    // ***************************************************************************
    //   Method:      Tele_UpdateWinch
    //
    //   Description: Method for controlling the robot lift winch system during teleop.
    //                Controller input values are read and robot lift winch motor speeds
    //                are set based on the controller input values.
    //
    // ***************************************************************************
    void Tele_UpdateWinch( void )
    {
        bool const debugWinch = false;

        bool const yButtonPressed = LogitechController.GetYButton( );
        bool const bButtonPressed = LogitechController.GetBButton( );
        bool const xButtonPressed = LogitechController.GetXButton( );
        bool const aButtonPressed = LogitechController.GetAButton( );

        bool const notAtTopLimit = winchLimiterTop.Get(); // Value of the limiter is nominally one, and zero when limit is hit.
        bool const notAtBotLimit = winchLimiterBot.Get(); // Value of the limiter is nominally one, and zero when limit is hit.

        double brakeMotorSpeed = 0.0;

        double winchMotorSpeed = 0.0;

        if ( debugWinch )
        {
            std::cout << winchLimiterTop.Get();
            std::cout << "   ";
            std::cout << winchLimiterBot.Get();
            std::cout << "\n";
            std::cout << std::flush;
        }

        if ( xButtonPressed )
        {
            brakeMotorSpeed = 1.0;
        }
        else if ( aButtonPressed )
        {
            brakeMotorSpeed = -1.0;
        }
        else
        {
            brakeMotorSpeed = 0.0;
        }

        brakeMotor.Set( brakeMotorSpeed );

        if ( yButtonPressed )
        {
            if ( notAtTopLimit )
            {
                winchMotorSpeed = 1.0;
            }
            else
            {
                winchMotorSpeed = 0.0;
            }
        }
        else if ( bButtonPressed )
        {
            if ( notAtBotLimit )
            {
                winchMotorSpeed = -1.0;
            }
            else
            {
                winchMotorSpeed = 0.0;
            }
        }
        else
        {
            winchMotorSpeed = 0.0;
        }

        // Update motor controllers
        SetWinchMotorSpeed( winchMotorSpeed );
    }






// ***************************************************************************
//
//              Autonomous Support
//
// ***************************************************************************

    typedef enum
    {
        DRIVE,
        TURN,
        LIFT,
        CUBE,
        WAIT,
        FINISHED
    } auto_command_types_t;

    typedef struct
    {
        double speed;
        double time;
        bool   blockCommands;
    } drive_command_args_t;

    typedef struct
    {
        double speed;
        double angle;
    } turn_command_args_t;

    typedef struct
    {
        double speed;
        double time;
        bool   blockCommands;
    } cube_command_args_t;

    typedef struct
    {
        lift_state_t liftState;
        double time;
        bool   blockCommands;
    } lift_command_args_t;

    typedef struct
    {
        double time;
    } wait_command_args_t;


    typedef struct
    {
        auto_command_types_t commandType;

        union
        {
            drive_command_args_t drive;
            turn_command_args_t  turn;
            cube_command_args_t  cube;
            lift_command_args_t  lift;
            wait_command_args_t  wait;
        } args;

    } auto_command_t;


    // Middle position, LLR
    auto_command_t const AutoCommandList0[12] = \
    {
        // Push cube forward
        { DRIVE, { .drive = { .speed =  0.2, .time = 2.0, .blockCommands = true  } } },

        { WAIT, { .wait = { .time = 2.0  } } },
        // Move back away from cube to give room for arms to drop down
        { DRIVE, { .drive = { .speed = -0.2, .time = 2.0, .blockCommands = true  } } },
        // Move lift up to drop arms down
        {  LIFT, { .lift  = { .liftState =  MoveUp, .time = 2.0, .blockCommands = true  } } },
        // Move lift back down to ground level
        {  LIFT, { .lift  = { .liftState = MoveDown, .time = 2.0, .blockCommands = true  } } },
        // Turn on cube intake
        {  CUBE, { .cube  = { .speed =  1.0, .time = 2.0, .blockCommands = false } } },
        // Drive forward to pick up cube
        { DRIVE, { .drive = { .speed =  0.2, .time = 2.0, .blockCommands = true  } } },
        // Start lifting cube while we turn and move toward the thingy
        {  LIFT, { .lift  = { .liftState =  MoveUp, .time = 3.0, .blockCommands = false  } } },


        { WAIT, { .wait = { .time = 3.0  } } },
        // Turn Right 90 degrees
        {  TURN, { .turn  = { .speed =  0.3, .angle = 90.0 } } },
        // Drive to the thingy

        { WAIT, { .wait = { .time = 10.0  } } },
        //{ DRIVE, { .drive = { .speed =  0.8, .time = 3.0, .blockCommands = true  } } },
        // Drop cube
        //{  CUBE, { .drive = { .speed =  -1.0, .time = 1.0, .blockCommands = true  } } },
        { FINISHED },
    };


    auto_command_t const TurnTest[17] = \
    {
        {  TURN, { .turn = { .speed =  0.3, .angle = 90.0 } } },
        {  WAIT, { .wait = { .time  = 2.0  } } },
        {  TURN, { .turn = { .speed =  0.3, .angle = -90.0 } } },
        {  WAIT, { .wait = { .time  = 2.0  } } },
        {  TURN, { .turn = { .speed =  0.3, .angle = 90.0 } } },
        {  WAIT, { .wait = { .time  = 2.0  } } },
        {  TURN, { .turn = { .speed =  0.3, .angle = -90.0 } } },
        {  WAIT, { .wait = { .time  = 2.0  } } },
        {  TURN, { .turn = { .speed =  0.3, .angle = 90.0 } } },
        {  WAIT, { .wait = { .time  = 2.0  } } },
        {  TURN, { .turn = { .speed =  0.3, .angle = -90.0 } } },
        {  WAIT, { .wait = { .time  = 2.0  } } },
        {  TURN, { .turn = { .speed =  0.3, .angle = 90.0 } } },
        {  WAIT, { .wait = { .time  = 2.0  } } },
        {  TURN, { .turn = { .speed =  0.3, .angle = -90.0 } } },
        {  WAIT, { .wait = { .time  = 2.0  } } },
        { FINISHED },
    };


    auto_command_t const LiftTest[5] = \
    {
        {  LIFT, { .lift  = { .liftState =  MoveUp, .time = 3.0, .blockCommands = true  } } },
        {  WAIT, { .wait = { .time  = 2.0  } } },
        {  LIFT, { .lift  = { .liftState =  MoveDown, .time = 1.5, .blockCommands = true  } } },
        {  WAIT, { .wait = { .time  = 2.0  } } },
        { FINISHED },
    };


    auto_command_t const DriveTest[30] = \
    {
        { DRIVE, { .drive = { .speed =  0.3, .time = 5.0, .blockCommands = true  } } },
        {  TURN, { .turn  = { .speed =  0.2, .angle = 180.0 } } },
        { DRIVE, { .drive = { .speed =  0.3, .time = 5.0, .blockCommands = true  } } },
        {  TURN, { .turn  = { .speed =  0.2, .angle = -180.0 } } },
        { DRIVE, { .drive = { .speed =  0.3, .time = 5.0, .blockCommands = true  } } },
        {  TURN, { .turn  = { .speed =  0.2, .angle = 180.0 } } },
        { DRIVE, { .drive = { .speed =  0.3, .time = 5.0, .blockCommands = true  } } },
        {  TURN, { .turn  = { .speed =  0.2, .angle = -180.0 } } },
        { DRIVE, { .drive = { .speed =  0.3, .time = 5.0, .blockCommands = true  } } },
        {  TURN, { .turn  = { .speed =  0.2, .angle = 180.0 } } },
        { DRIVE, { .drive = { .speed =  0.3, .time = 5.0, .blockCommands = true  } } },
        {  TURN, { .turn  = { .speed =  0.2, .angle = -180.0 } } },
        { DRIVE, { .drive = { .speed =  0.3, .time = 5.0, .blockCommands = true  } } },
        {  TURN, { .turn  = { .speed =  0.2, .angle = 180.0 } } },
        { DRIVE, { .drive = { .speed =  0.3, .time = 5.0, .blockCommands = true  } } },
        {  TURN, { .turn  = { .speed =  0.2, .angle = -180.0 } } },
        { DRIVE, { .drive = { .speed =  0.3, .time = 5.0, .blockCommands = true  } } },
        {  TURN, { .turn  = { .speed =  0.2, .angle = 180.0 } } },
        { DRIVE, { .drive = { .speed =  0.3, .time = 5.0, .blockCommands = true  } } },
        {  TURN, { .turn  = { .speed =  0.2, .angle = -180.0 } } },
        { FINISHED },
    };




    // Middle position, LLR,
    auto_command_t const AutoCommandList1[50] = \
    {
        // Push cube forward
        { DRIVE, { .drive = { .speed =  0.2, .time = 2.0, .blockCommands = true  } } },

        { WAIT, { .wait = { .time = 2.0  } } },
        // Move back away from cube to give room for arms to drop down
        { DRIVE, { .drive = { .speed = -0.2, .time = 2.0, .blockCommands = true  } } },
        // Move lift up to drop arms down
        {  LIFT, { .lift  = { .liftState =  MoveUp, .time = 2.0, .blockCommands = true  } } },
        // Move lift back down to ground level
        {  LIFT, { .lift  = { .liftState = MoveDown, .time = 2.0, .blockCommands = true  } } },
        // Turn on cube intake
        {  CUBE, { .cube  = { .speed =  1.0, .time = 2.0, .blockCommands = false } } },
        // Drive forward to pick up cube
        { DRIVE, { .drive = { .speed =  0.2, .time = 2.0, .blockCommands = true  } } },
        // Start lifting cube while we turn and move toward the thingy
        {  LIFT, { .lift  = { .liftState =  MoveUp, .time = 3.0, .blockCommands = false  } } },


        { WAIT, { .wait = { .time = 3.0  } } },
        // Turn Right 90 degrees
        {  TURN, { .turn  = { .speed =  0.3, .angle = 90.0 } } },
        // Drive to the thingy

        { WAIT, { .wait = { .time = 10.0  } } },



        { FINISHED },
    };

// changes

    //Left position, Left switch, load
        auto_command_t const AutoCommandList2[50] = \
         {
            { DRIVE, { .drive = { .speed =  0.4, .time = 3.0, .blockCommands = true  } } }, // 132 inches
            {  TURN, { .turn  = { .speed =  0.3, .angle = 90.0 } } },
            {  LIFT, { .lift  = { .liftState =  MoveUp, .time = 3.5, .blockCommands = true  } } },
            { DRIVE, { .drive = { .speed =  0.2, .time = 0.5, .blockCommands = true  } } },
            {  CUBE, { .cube  = { .speed =  -1.0, .time = 1.5, .blockCommands = false } } },
            { FINISHED },
         };

    //right position, right switch, load
        auto_command_t const AutoCommandList3[50] = \
           {
               { DRIVE, { .drive = { .speed =  0.4, .time = 3.0, .blockCommands = true  } } },
               {  TURN, { .turn  = { .speed =  0.3, .angle = -90.0 } } },
               {  LIFT, { .lift  = { .liftState =  MoveUp, .time = 3.5, .blockCommands = true  } } },
               { DRIVE, { .drive = { .speed =  0.2, .time = 0.5, .blockCommands = true  } } },
               {  CUBE, { .cube  = { .speed =  -1.0, .time = 1.5, .blockCommands = false } } },
               { FINISHED },
          };


    //left position, left scale, load
        auto_command_t const AutoCommandList4[50] = \
        {
            { DRIVE, { .drive = { .speed =  0.5, .time = 5.25, .blockCommands = true  } } }, // to center of scale (287.65 inches)
            {  TURN, { .turn  = { .speed =  0.3, .angle = 90.0 } } },
            { DRIVE, { .drive = { .speed =  -0.3, .time = 0.25, .blockCommands = true  } } }, // away from scale plate
            {  LIFT, { .lift  = { .liftState =  MoveUp, .time = 8, .blockCommands = true  } } }, // lift all the way up to scale: WILL TAKE A LONG FRICKEN TIME
            {  CUBE, { .cube  = { .speed =  -1.0, .time = 1.5, .blockCommands = false } } },
            { FINISHED },
       };

    //right position, right scale, load
        auto_command_t const AutoCommandList5[50] = \
        {
            { DRIVE, { .drive = { .speed =  0.4, .time = 6.54, .blockCommands = true  } } }, // to center of scale (287.65 inches)
            {  TURN, { .turn  = { .speed =  0.3, .angle = -90.0 } } },
            {  LIFT, { .lift  = { .liftState =  MoveUp, .time = 10, .blockCommands = true  } } }, // lift all the way up to scale: WILL TAKE A LONG FRICKEN TIME
            { DRIVE, { .drive = { .speed =  0.2, .time = 0.25, .blockCommands = true  } } }, // towards scale plate
            {  CUBE, { .cube  = { .speed =  -1.0, .time = 1.5, .blockCommands = false } } },
            { FINISHED },
       };




    //middle position, left switch, load
    auto_command_t const AutoCommandList6[50] = \
        {
           { DRIVE, { .drive = { .speed =  0.4, .time = 1.0, .blockCommands = true  } } },
           {  TURN, { .turn  = { .speed =  0.3, .angle = -90.0 } } },
           { DRIVE, { .drive = { .speed =  0.4, .time = 2.9, .blockCommands = true  } } }, //driving past left switch
           {  TURN, { .turn  = { .speed =  0.3, .angle = 90.0 } } },
           { DRIVE, { .drive = { .speed =  0.5, .time = 2.0, .blockCommands = true  } } }, //this may need changes so it is in line with switch
           {  TURN, { .turn  = { .speed =  0.3, .angle = 90.0 } } },
           {  LIFT, { .lift  = { .liftState =  MoveUp, .time = 3.5, .blockCommands = true  } } },
           { DRIVE, { .drive = { .speed =  0.2, .time = 0.5, .blockCommands = true  } } }, //forward, towards switch platform
           {  CUBE, { .cube  = { .speed =  -1.0, .time = 1.5, .blockCommands = false } } },
           { FINISHED },
        };


    //middle position, right switch, load
    auto_command_t const AutoCommandList7[50] = \
        {
           { DRIVE, { .drive = { .speed =  0.4, .time = 1.0, .blockCommands = true  } } },
           {  TURN, { .turn  = { .speed =  0.3, .angle = 90.0 } } },
           { DRIVE, { .drive = { .speed =  0.4, .time = 2.9, .blockCommands = true  } } }, //driving past right switch
           {  TURN, { .turn  = { .speed =  0.3, .angle = -90.0 } } },
           { DRIVE, { .drive = { .speed =  0.5, .time = 2.0, .blockCommands = true  } } }, //this may need changes so it is in line with switch
           {  TURN, { .turn  = { .speed =  0.3, .angle = -90.0 } } },
           {  LIFT, { .lift  = { .liftState =  MoveUp, .time = 3.5, .blockCommands = true  } } },
           { DRIVE, { .drive = { .speed =  0.2, .time = 0.5, .blockCommands = true  } } }, //forward, towards switch platform
           {  CUBE, { .cube  = { .speed =  -1.0, .time = 1.5, .blockCommands = false } } },
           { FINISHED },
        };


    //middle position, left scale, load
    auto_command_t const AutoCommandList8[50] = \
         {
           { DRIVE, { .drive = { .speed =  0.4, .time = 1.0, .blockCommands = true  } } },
           {  TURN, { .turn  = { .speed =  0.3, .angle = -90.0 } } },
           { DRIVE, { .drive = { .speed =  0.4, .time = 3.0, .blockCommands = true  } } }, //driving past left switch
           {  TURN, { .turn  = { .speed =  0.3, .angle = 90.0 } } },
           { DRIVE, { .drive = { .speed =  0.4, .time = 3.0, .blockCommands = true  } } }, //this may need changes so it is in line with switch
           { FINISHED },
         };

    //middle position, right scale, load
    auto_command_t const AutoCommandList9[50] = \
          {
        { DRIVE, { .drive = { .speed =  0.4, .time = 1.0, .blockCommands = true  } } },
        {  TURN, { .turn  = { .speed =  0.3, .angle = 90.0 } } },
        { DRIVE, { .drive = { .speed =  0.4, .time = 3.0, .blockCommands = true  } } }, //driving past left switch
        {  TURN, { .turn  = { .speed =  0.3, .angle = -90.0 } } },
        { DRIVE, { .drive = { .speed =  0.4, .time = 3.0, .blockCommands = true  } } }, //this may need changes so it is in line with switch
        { FINISHED },
         };

    //middle position, auto line only (from left)
    auto_command_t const AutoCommandList10[50] = \
        {
           { DRIVE, { .drive = { .speed =  0.4, .time = 1.0, .blockCommands = true  } } },
           {  TURN, { .turn  = { .speed =  0.3, .angle = -90.0 } } },
           { DRIVE, { .drive = { .speed =  0.4, .time = 3.0, .blockCommands = true  } } }, //driving past left switch
           {  TURN, { .turn  = { .speed =  0.3, .angle = 90.0 } } },
           { DRIVE, { .drive = { .speed =  0.4, .time = 2.5, .blockCommands = true  } } },
           { FINISHED },
        };

    //middle position, auto line only (from right)
    auto_command_t const AutoCommandList11[50] = \
        {
           { DRIVE, { .drive = { .speed =  0.4, .time = 1.0, .blockCommands = true  } } },
           {  TURN, { .turn  = { .speed =  0.3, .angle = 90.0 } } },
           { DRIVE, { .drive = { .speed =  0.4, .time = 3.0, .blockCommands = true  } } }, //driving past right switch
           {  TURN, { .turn  = { .speed =  0.3, .angle = -90.0 } } },
           { DRIVE, { .drive = { .speed =  0.4, .time = 2.5, .blockCommands = true  } } },
           { FINISHED },
        };


    //left position, right switch, load
    auto_command_t const AutoCommandList12[50] = \
       {
           { DRIVE, { .drive = { .speed =  0.6, .time = 3.1, .blockCommands = true  } } }, //drive until it passes switch ( 225.47inches )
           {  TURN, { .turn  = { .speed =  0.4, .angle = 90.0 } } },
           { DRIVE, { .drive = { .speed =  0.5, .time = 3.0, .blockCommands = true  } } }, //across field to right side
           {  TURN, { .turn  = { .speed =  0.3, .angle = 90.0 } } },
           {  LIFT, { .lift  = { .liftState =  MoveUp, .time = 3.5, .blockCommands = true  } } }, //might want to do this while it's driving to save time
           { DRIVE, { .drive = { .speed =  0.2, .time = 1.0, .blockCommands = true  } } }, //towards right switch
           {  CUBE, { .cube  = { .speed =  -1.0, .time = 1.5, .blockCommands = false } } },
           { FINISHED },
      };


    //right position, left switch, load
       auto_command_t const AutoCommandList13[50] = \
        {
           { DRIVE, { .drive = { .speed =  0.65, .time = 3.0, .blockCommands = true  } } }, //drive until it passes switch
           {  TURN, { .turn  = { .speed =  0.3, .angle = -90.0 } } },
           { DRIVE, { .drive = { .speed =  0.53, .time = 3.0, .blockCommands = true  } } }, //across field to left side
           {  TURN, { .turn  = { .speed =  0.3, .angle = -90.0 } } },
           { DRIVE, { .drive = { .speed =  0.4, .time = 1.0, .blockCommands = true  } } },  //until it is in line with switch
           {  TURN, { .turn  = { .speed =  0.3, .angle = -90.0 } } },
           {  LIFT, { .lift  = { .liftState =  MoveUp, .time = 3.5, .blockCommands = true  } } }, //might want to do this while it's driving
           { DRIVE, { .drive = { .speed =  0.2, .time = 0.5, .blockCommands = true  } } }, //towards right switch
           {  CUBE, { .cube  = { .speed =  -1.0, .time = 1.5, .blockCommands = false } } },
           { FINISHED },
         };

    //left position, right scale, load
       auto_command_t const AutoCommandList14[50] = \
        {

        };
    //right position, left scale, load
       auto_command_t const AutoCommandList15[50] = \
            {
          { DRIVE, { .drive = { .speed =  0.6, .time = 3.1, .blockCommands = true  } } }, //drive until it passes switch ( 225.47inches )
          {  TURN, { .turn  = { .speed =  0.4, .angle = -90.0 } } },
          { DRIVE, { .drive = { .speed =  0.5, .time = 4.0, .blockCommands = true  } } }, //across field to left side
          {  TURN, { .turn  = { .speed =  0.3, .angle = 90.0 } } },
          { DRIVE, { .drive = { .speed =  0.5, .time = 1.6, .blockCommands = true  } } }, //towards scale
          {  TURN, { .turn  = { .speed =  0.3, .angle = 90.0 } } },
          { DRIVE, { .drive = { .speed =  -0.3, .time = 0.4, .blockCommands = true  } } }, //away left scale
          {  LIFT, { .lift  = { .liftState =  MoveUp, .time = 9.0, .blockCommands = true  } } }, //might want to do this while it's driving to save time
          { DRIVE, { .drive = { .speed =  0.2, .time = 1, .blockCommands = true  } } }, //toward left scale
          {  CUBE, { .cube  = { .speed =  -1.0, .time = 1.5, .blockCommands = false } } },
          { FINISHED },
            };


       //left position, auto line only (if switch and/or scale are on right)
       auto_command_t const AutoCommandList16[50] = \
           {
               { DRIVE, { .drive = { .speed =  0.4, .time = 3.0, .blockCommands = true  } } }, // 132 inches
               { FINISHED },
          };

       //right position, auto line only (if switch and/or scale are on left)
       auto_command_t const AutoCommandList17[50] = \
           {
               { DRIVE, { .drive = { .speed =  0.4, .time = 3.0, .blockCommands = true  } } }, // 132 inches
               { FINISHED },
          };




    auto_command_t const AutoCommanddDoNothing[50] = \
        {
           { FINISHED },
        };




    auto_command_t const *AutoCommandList = AutoCommanddDoNothing;
    int currentCommandIndex = -1;


    frc::Timer driveTimer;
    frc::Timer liftTimer;
    frc::Timer cubeTimer;
    frc::Timer waitTimer;

    bool AutoCommandsFinished = false;

    double driveTimerEnd = 0.0;
    double liftTimerEnd  = 0.0;
    double cubeTimerEnd  = 0.0;
    double waitTimerEnd  = 0.0;

    bool driveCommandActive = false;
    bool liftCommandActive  = false;
    bool cubeCommandActive  = false;
    bool waitCommandActive  = false;
    bool turnCommandActive  = false;

    bool driveCommandBlocking = false;
    bool liftCommandBlocking  = false;
    bool cubeCommandBlocking  = false;
    bool waitCommandBlocking  = false;
    bool turnCommandBlocking  = false;


    drive_command_args_t currentDriveCommand;
    turn_command_args_t  currentTurnCommand;
    cube_command_args_t  currentCubeCommand;
    lift_command_args_t  currentLiftCommand;


    void Auto_Initalize( void )
    {
        gyro.Reset();
        currentCommandIndex = -1;
        AutoCommandsFinished = false;

        driveTimerEnd = 0.0;
        liftTimerEnd  = 0.0;
        cubeTimerEnd  = 0.0;
        waitTimerEnd  = 0.0;

        driveCommandActive = false;
        liftCommandActive  = false;
        cubeCommandActive  = false;
        waitCommandActive  = false;
        turnCommandActive  = false;

        driveCommandBlocking = false;
        liftCommandBlocking  = false;
        cubeCommandBlocking  = false;
        waitCommandBlocking  = false;
        turnCommandBlocking  = false;
        absoluteAngle = 0.0;
        maxAbsMotorChange = 1.0;
    }

    double absoluteAngle = 0.0;

    // ***************************************************************************
    //   Method:      Auto_ServiceCommandList
    //
    //   Description: Method for servicing the commands in the "AutoCommandList"
    //
    // ***************************************************************************
    void Auto_ServiceCommandList( void )
    {
        bool const debugCommandList = false;
        // Check for any active commands that need to be serviced.
        if ( driveCommandActive )
        {
            if ( driveTimer.Get() > driveTimerEnd )
            {
                Auto_StopDriving();
                driveCommandActive  = false;
                driveCommandBlocking = false;
            }
            else
            {
                Auto_DriveStraight(
                    currentDriveCommand.speed
                );
            }
        }

        if ( turnCommandActive )
        {
            bool turnCompleted = false;

            turnCompleted = \
            Auto_Turn(
                currentTurnCommand.speed,
                currentTurnCommand.angle
            );
            if ( turnCompleted == true )
            {
                turnCommandActive   = false;
                turnCommandBlocking = false;
            }
        }


        if ( !driveCommandActive && !turnCommandActive )
        {
            Auto_StopDriving();
        }

        if ( liftCommandActive )
        {
            if ( liftTimer.Get() > liftTimerEnd )
            {
                Auto_MoveLift(
                    HoldPosition
                );
                liftCommandActive   = false;
                liftCommandBlocking = false;
            }
            else
            {
                Auto_MoveLift(
                    currentLiftCommand.liftState
                );
            }
        }
        else
        {
            Auto_MoveLift(
                HoldPosition
            );
        }

        if ( cubeCommandActive )
        {
            if ( cubeTimer.Get() > cubeTimerEnd )
            {
                Auto_StopCube();
                cubeCommandActive   = false;
                cubeCommandBlocking = false;
            }
            else
            {
                Auto_MoveCube(
                    currentCubeCommand.speed
                );
            }
        }

        if ( waitCommandActive )
        {
            if ( waitTimer.Get() > waitTimerEnd )
            {
                waitCommandActive  = false;
                waitCommandBlocking = false;
            }
            else
            {
                // do nothing
            }
        }

        if ( debugCommandList )
        {
            std::cout << "cmd ";
            std::cout << currentCommandIndex;
            std::cout << "dr_b ";
            std::cout << driveCommandBlocking;
            std::cout << " ";
            std::cout << driveCommandActive;
            std::cout << " lf_b ";
            std::cout << liftCommandBlocking;
            std::cout << " ";
            std::cout << liftCommandActive;
            std::cout << " cu_b ";
            std::cout << cubeCommandBlocking;
            std::cout << " ";
            std::cout << cubeCommandActive;
            std::cout << " wt_b ";
            std::cout << waitCommandBlocking;
            std::cout << " ";
            std::cout << waitCommandActive;
            std::cout << " tr_b ";
            std::cout << turnCommandBlocking;
            std::cout << " ";
            std::cout << turnCommandActive;
            std::cout << "\n";
            std::cout << std::flush;
        }


        // Service new command if we are not waiting on any blocking commands,
        // and we have not gotten to the end of the command list
        if ( !AutoCommandsFinished &&
             !driveCommandBlocking &&
             !liftCommandBlocking  &&
             !cubeCommandBlocking  &&
             !waitCommandBlocking  &&
             !turnCommandBlocking
            )
        {
            currentCommandIndex++;

            auto_command_t const *command = &(AutoCommandList[currentCommandIndex]);

            // Service current command
            switch ( command->commandType )
            {
                case DRIVE:
                {
                    Auto_ResetDriveDirection();
                    driveCommandActive   = true;
                    driveTimerEnd        = command->args.drive.time;
                    driveCommandBlocking = command->args.drive.blockCommands;
                    currentDriveCommand  = command->args.drive;

                    driveTimer.Reset();
                    driveTimer.Start();

                    break;
                }

                case LIFT:
                {
                    liftCommandActive   = true;
                    liftTimerEnd        = command->args.lift.time;
                    liftCommandBlocking = command->args.lift.blockCommands;
                    currentLiftCommand  = command->args.lift;

                    liftTimer.Reset();
                    liftTimer.Start();

                    break;
                }

                case CUBE:
                {
                    cubeCommandActive   = true;
                    cubeTimerEnd        = command->args.cube.time;
                    cubeCommandBlocking = command->args.cube.blockCommands;
                    currentCubeCommand  = command->args.cube;

                    cubeTimer.Reset();
                    cubeTimer.Start();

                    break;
                }

                case TURN:
                {
                    Auto_PrepareForTurn();
                    turnCommandActive   = true;
                    turnCommandBlocking = true;
                    currentTurnCommand  = command->args.turn;
                    absoluteAngle += command->args.turn.angle;
                    break;
                }

                case WAIT:
                {
                    waitCommandActive   = true;
                    waitTimerEnd        = command->args.wait.time;
                    waitCommandBlocking = true;
                    waitTimer.Reset();
                    waitTimer.Start();

                    break;
                }

                default:
                case FINISHED:
                {
                    AutoCommandsFinished = true;

                    break;
                }
            }
        }
    }





    typedef enum
    {
        notMoving,
        drivingForward,
        turning
    } auto_drive_state_t;

    auto_drive_state_t driveState = notMoving;
    double desiredDriveDirection = 0.0;


    // ***************************************************************************
    //   Method:      Auto_ResetDriveDirection
    //
    //   Description: Sets the robots current direction to be the direction that
    //                the Auto_DriveStraight function drives.
    //
    // ***************************************************************************
    void Auto_ResetDriveDirection( void )
    {
        desiredDriveDirection = gyro.GetAngle();//absoluteAngle;//gyro.GetAngle();
    }



    // ***************************************************************************
    //   Method:      Auto_ResetDriveDirection
    //
    //   Description: Function for Autonomous to keep the robot driving
    //                straight at the specified speed.
    //                Call Auto_ResetDriveDirection to initialize the "straight"
    //                direction.
    //
    // ***************************************************************************
    void Auto_DriveStraight(
        double speed
    )
    {
        bool const debugDriveStraight = false;
        double const currentDirection                     = gyro.GetAngle();
        double const directionError                       = desiredDriveDirection - currentDirection;
        double const directionErrorCompensationSaturation = 5;
        double const directionErrorSaturated              = fabs( saturate( directionError, -directionErrorCompensationSaturation, directionErrorCompensationSaturation ) );
        double const maxMotorSpeedCorrection              = 0.3;
        double const motorSpeedCorrection                 = maxMotorSpeedCorrection * ( directionErrorSaturated / directionErrorCompensationSaturation );

        // Set nominal speeds
        double leftMotorSpeed  = speed;
        double rightMotorSpeed = speed;

        if ( debugDriveStraight )
        {
            std::cout << "\ndes " << desiredDriveDirection;
            std::cout << " curr " << currentDirection;
            std::cout << " corr " << motorSpeedCorrection;
            std::cout << " mot " << leftMotorSpeed << " " << rightMotorSpeed;
        }

        // Apply correction if needed.
        if ( directionError > 0 )
        {
            rightMotorSpeed = rightMotorSpeed - motorSpeedCorrection;
            if ( debugDriveStraight )
            {
                std::cout << " R ";
            }
        }
        else if ( directionError < 0 )
        {
            leftMotorSpeed = leftMotorSpeed - motorSpeedCorrection;
            if ( debugDriveStraight )
            {
                std::cout << " L ";
            }
        }
        else
        {
            // No correction needed.
            if ( debugDriveStraight )
            {
                std::cout << " N ";
            }
        }

        SetDriveMotorSpeeds(
            leftMotorSpeed,
            rightMotorSpeed
        );
    }


    // ***************************************************************************
    //   Method:      Auto_StopDriving
    //
    //   Description: Turns off the drive motors.
    //
    // ***************************************************************************
    void Auto_StopDriving( void )
    {
        SetDriveMotorSpeeds(
            0,
            0
        );
    }


    // ***************************************************************************
    //   Method:      Auto_MoveCube
    //
    //   Description: Sets the cube intake to run at the specified speed.
    //
    // ***************************************************************************
    void Auto_MoveCube(
        double speed
    )
    {
        SetCubeIntakeMotorSpeed(
            speed
        );
    }


    // ***************************************************************************
    //   Method:      Auto_StopCube
    //
    //   Description: Stops the cube intake motors.
    //
    // ***************************************************************************
    void Auto_StopCube( void )
    {
        SetCubeIntakeMotorSpeed(
            0.0
        );
    }


    // ***************************************************************************
    //   Method:      Auto_MoveLift
    //
    //   Description: Moves the cube lift at the specified speed.
    //
    // ***************************************************************************
    void Auto_MoveLift(
        lift_state_t liftState
    )
    {
         SetCubeLiftState(
             liftState,
             1.0
         );
    }


    double startingAngle   = 0.0;
    int    goodAngleCounts = 0;
    // ***************************************************************************
    //   Method:      Auto_PrepareForTurn
    //
    //   Description: Prepares for performing a turn with Auto_Turn.
    //                Right now just remembers the current angle so we know how
    //                much to turn.
    //
    // ***************************************************************************
    void Auto_PrepareForTurn( void )
    {
        std::cout << "\nStarting Angle: ";
        goodAngleCounts = 0;
        startingAngle = gyro.GetAngle(); //absoluteAngle;//
        std::cout << startingAngle;
    }



    // ***************************************************************************
    //   Method:      Auto_Turn
    //
    //   Description: Perform a turn at the specified speed to the specified angle.
    //                Auto_PrepareForTurn must be run before using this function
    //                to set the starting angle.
    //
    // ***************************************************************************
    bool Auto_Turn(
        double speed,
        double angle
    )
    {
        bool const debugTurn = false;
        bool turnCompleted = false;
        double const currentAngle = gyro.GetAngle();
        double const desiredAngle = startingAngle + angle;
        double const angleError   = currentAngle - desiredAngle;
        double leftMotorSpeed     = 0;
        double rightMotorSpeed    = 0;

        double const anglePrecision = 2.0;

        if ( debugTurn )
        {
            std::cout << "\nDesired Angle: " << desiredAngle;
            std::cout << "\nAngle Error: "   << angleError;
        }

        if ( fabs( angleError ) < anglePrecision )
        {
            goodAngleCounts++;
        }
        else
        {
            goodAngleCounts = 0;
        }


        if ( goodAngleCounts > 3 )
        {
            if ( debugTurn )
            {
                std::cout << "\nEnding Angle: " << currentAngle;
                std::cout << "\nEnding Error: " << angleError  << "\n";
            }
            turnCompleted = true;
        }
        else
        {
            double const speedMultiplier = saturate( angleError * 0.1, -1, 1 );

            leftMotorSpeed  = -speed * speedMultiplier;
            rightMotorSpeed =  speed * speedMultiplier;
        }

        if ( debugTurn )
        {
            std::cout << "\nmotor1: " << leftMotorSpeed << " " << rightMotorSpeed;
        }

        // Set a minimum motor speed
        leftMotorSpeed = \
        minAbsLimitValue(
            leftMotorSpeed,
            0.15
        );

        rightMotorSpeed = \
        minAbsLimitValue(
            rightMotorSpeed,
            0.15
        );

        if ( debugTurn )
        {
            std::cout << "\nmotor2: " << leftMotorSpeed << " " << rightMotorSpeed;
        }

        SetDriveMotorSpeeds(
            leftMotorSpeed,
            rightMotorSpeed
        );


        return turnCompleted;
    }



};

START_ROBOT_CLASS(Robot)
