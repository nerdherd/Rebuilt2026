package frc.robot.util;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// thank you william
public class Controller {
    private CommandPS4Controller cmdPS4;
    private PS4Controller ps4;
    private CommandPS5Controller cmdPS5;
    private PS5Controller ps5;
    private GuliKit guliKit;
    private XboxController xbox;

    private final Type type;

    public enum Type {
        PS4,
        PS5,
        GuliKit,
        Xbox
    }

    /**
     * Constructor for PS4, PS5, and Xbox controllers.
     * Not for GuliKit controllers.
     * @param port in Driver Station
     * @param type of controller
     * @throws Exception
     */
    public Controller(int port, Type type) {
        this.type = type;

        switch (type) {
            case PS4:
            cmdPS4 = new CommandPS4Controller(port);
            ps4 = cmdPS4.getHID();
            case PS5:
            cmdPS5 = new CommandPS5Controller(port);
            ps5 = cmdPS5.getHID();
            case Xbox:
            xbox = new XboxController(port);
            case GuliKit:
            // Use the constructor Controller(int port, Type type, boolean isDigLeft, boolean isDigRight) for the GuliKit.
            default:
        }
    }

    /**
     * Constructor for a GuliKit controller.
     * @param port in Driver Station
     * @param type of controller
     * @param isDigLeft discrete or continuous for the left joystick
     * @param isDigRight discrete or continuous for the right joystick
     */
    public Controller(int port, Type type, boolean isDigLeft, boolean isDigRight) {
        this.type = type;

        if (type == Type.GuliKit)
            guliKit = new GuliKit(port, isDigLeft, isDigRight);
    }

    /**
     * PS4 controller implemntation.
     * <p>Combines CommandPS4Controller and PS4Controller.
     * @param port The port of the controller on FRC Driver Station
     * @deprecated kept for previous versions
     */
    @Deprecated(forRemoval = false)
    public Controller(int port) {
        type = Type.PS4;

        cmdPS4 = new CommandPS4Controller(port);
        ps4 = cmdPS4.getHID();
    }

    /**
     * PS4/PS5 controller implemntation.
     * <p>Combines CommandPS4Controller, PS4Controller, CommandPS5Controller, and PS5Controller. 
     * @param port The port of the controller on FRC Driver Station
     * @param usePS4 Whether to use PS4 (true) or PS5 (false)
     * @deprecated kept for previous versions
     */
    @Deprecated(forRemoval = false)
    public Controller(int port, boolean usePS4) {
        if (usePS4) {
            type = Type.PS4;

            cmdPS4 = new CommandPS4Controller(port);
            ps4 = cmdPS4.getHID();
        } else {
            type = Type.PS5;

            cmdPS5 = new CommandPS5Controller(port);
            ps5 = cmdPS5.getHID();
        }
    }

    /**
     * GuliKit controller implementation.
     * @param port The port of the controller on FRC Driver Station
     * @param isDigLeft If the left switch on the controller is set to digital (dot)
     * @param isDigRight If the right switch on the controller is set to digital (dot)
     * @deprecated kept for previous versions
     */
    @Deprecated(forRemoval = false)
    public Controller(int port, boolean isDigLeft, boolean isDigRight) {
        this.type = Type.GuliKit;

        guliKit = new GuliKit(port, isDigLeft, isDigRight);
    }

    // ***** VALUE METHODS ***** //

    public double getLeftX()    { switch (type) {case PS4: return ps4.getLeftX();  case PS5: return ps5.getLeftX();  case GuliKit: return guliKit.getLeftX();  case Xbox: return xbox.getLeftX();  default: return 0.0;} }
    public double getLeftY()    { switch (type) {case PS4: return ps4.getLeftY();  case PS5: return ps5.getLeftY();  case GuliKit: return guliKit.getLeftY();  case Xbox: return xbox.getLeftY();  default: return 0.0;} }
    public double getRightX()   { switch (type) {case PS4: return ps4.getRightX(); case PS5: return ps5.getRightX(); case GuliKit: return guliKit.getRightX(); case Xbox: return xbox.getRightX(); default: return 0.0;} }
    public double getRightY()   { switch (type) {case PS4: return ps4.getRightY(); case PS5: return ps5.getRightY(); case GuliKit: return guliKit.getRightY(); case Xbox: return xbox.getRightY(); default: return 0.0;} }

    public boolean getButtonRight() { switch (type) {case PS4: return ps4.getCircleButton();   case PS5: return ps5.getCircleButton();   case GuliKit: return guliKit.getA(); case Xbox: return xbox.getAButton(); default: return false;} }
    public boolean getButtonDown()  { switch (type) {case PS4: return ps4.getCrossButton();    case PS5: return ps5.getCrossButton();    case GuliKit: return guliKit.getB(); case Xbox: return xbox.getBButton(); default: return false;} }
    public boolean getButtonUp()    { switch (type) {case PS4: return ps4.getTriangleButton(); case PS5: return ps5.getTriangleButton(); case GuliKit: return guliKit.getX(); case Xbox: return xbox.getXButton(); default: return false;} }
    public boolean getButtonLeft()  { switch (type) {case PS4: return ps4.getSquareButton();   case PS5: return ps5.getSquareButton();   case GuliKit: return guliKit.getY(); case Xbox: return xbox.getYButton(); default: return false;} }

    public boolean getTriggerLeft() { switch (type) {case PS4: return ps4.getL2Button(); case PS5: return ps5.getL2Button(); case GuliKit: return guliKit.getZLdigital(); case Xbox: return xbox.getLeftTriggerAxis() >= 0.7;  default: return false;} }
    public boolean getTriggerRight(){ switch (type) {case PS4: return ps4.getR2Button(); case PS5: return ps5.getR2Button(); case GuliKit: return guliKit.getZRdigital(); case Xbox: return xbox.getRightTriggerAxis() >= 0.7; default: return false;} }
    
    public boolean getBumperLeft()  { switch (type) {case PS4: return ps4.getL1Button(); case PS5: return ps5.getL1Button(); case GuliKit: return guliKit.getL(); case Xbox: return xbox.getLeftBumperButton();  default: return false;} }
    public boolean getBumperRight() { switch (type) {case PS4: return ps4.getR1Button(); case PS5: return ps5.getR1Button(); case GuliKit: return guliKit.getR(); case Xbox: return xbox.getRightBumperButton(); default: return false;} }

    public boolean getControllerLeft()  { switch (type) {case PS4: return ps4.getShareButton();   case PS5: return ps5.getCreateButton();  case GuliKit: return guliKit.getMinus(); case Xbox: return xbox.getStartButton(); default: return false;} }
    public boolean getControllerRight() { switch (type) {case PS4: return ps4.getOptionsButton(); case PS5: return ps5.getOptionsButton(); case GuliKit: return guliKit.getPlus();  case Xbox: return xbox.getBackButton();  default: return false;} }
    
    public boolean getJoystickLeft()    { switch (type) {case PS4: return ps4.getL3Button(); case PS5: return ps5.getL3Button(); case GuliKit: return guliKit.getLeftJoy();  case Xbox: return xbox.getLeftStickButton();  default: return false;} }
    public boolean getJoystickRight()   { switch (type) {case PS4: return ps4.getR3Button(); case PS5: return ps5.getR3Button(); case GuliKit: return guliKit.getRightJoy(); case Xbox: return xbox.getRightStickButton(); default: return false;} }

    public boolean getDpadUp()      { switch (type) {case PS4: return ps4.getPOV() == 0;   case PS5: return ps5.getPOV() == 0;  case GuliKit: return guliKit.getDpadUp();     case Xbox: return xbox.getPOV() == 0;   default: return false;} }
    public boolean getDpadRight()   { switch (type) {case PS4: return ps4.getPOV() == 90;  case PS5: return ps5.getPOV() == 90;  case GuliKit: return guliKit.getDpadRight(); case Xbox: return xbox.getPOV() == 90;  default: return false;} }
    public boolean getDpadDown()    { switch (type) {case PS4: return ps4.getPOV() == 180; case PS5: return ps5.getPOV() == 180; case GuliKit: return guliKit.getDpadDown();  case Xbox: return xbox.getPOV() == 180; default: return false;} }
    public boolean getDpadLeft()    { switch (type) {case PS4: return ps4.getPOV() == 270; case PS5: return ps5.getPOV() == 270; case GuliKit: return guliKit.getDpadLeft();  case Xbox: return xbox.getPOV() == 270; default: return false;} }

    // ***** OBJECT METHODS ***** //

    public Trigger buttonUp()   { switch (type) {case PS4: return cmdPS4.triangle(); case PS5: return cmdPS5.triangle(); case GuliKit: return guliKit.buttonY(); case Xbox: return xbox.y(new EventLoop()).castTo(Trigger::new); default: return null;} }
    public Trigger buttonRight(){ switch (type) {case PS4: return cmdPS4.circle();   case PS5: return cmdPS5.circle();   case GuliKit: return guliKit.buttonB(); case Xbox: return xbox.b(new EventLoop()).castTo(Trigger::new); default: return null;} }
    public Trigger buttonDown() { switch (type) {case PS4: return cmdPS4.cross();    case PS5: return cmdPS5.cross();    case GuliKit: return guliKit.buttonA(); case Xbox: return xbox.a(new EventLoop()).castTo(Trigger::new); default: return null;} }
    public Trigger buttonLeft() { switch (type) {case PS4: return cmdPS4.square();   case PS5: return cmdPS5.square();   case GuliKit: return guliKit.buttonX(); case Xbox: return xbox.x(new EventLoop()).castTo(Trigger::new); default: return null;} }

    public Trigger triggerLeft()    { switch (type) {case PS4: return cmdPS4.L2(); case PS5: return cmdPS5.L2(); case GuliKit: return guliKit.triggerZL(); case Xbox: return xbox.leftTrigger(new EventLoop()).castTo(Trigger::new);  default: return null;} }
    public Trigger triggerRight()   { switch (type) {case PS4: return cmdPS4.R2(); case PS5: return cmdPS5.R2(); case GuliKit: return guliKit.triggerZR(); case Xbox: return xbox.rightTrigger(new EventLoop()).castTo(Trigger::new); default: return null;} }
    
    public Trigger bumperLeft()     { switch (type) {case PS4: return cmdPS4.L1(); case PS5: return cmdPS5.L1(); case GuliKit: return guliKit.bumperL(); case Xbox: return xbox.leftBumper(new EventLoop()).castTo(Trigger::new);   default: return null;} }
    public Trigger bumperRight()    { switch (type) {case PS4: return cmdPS4.R1(); case PS5: return cmdPS5.R1(); case GuliKit: return guliKit.bumperR(); case Xbox: return xbox.rightTrigger(new EventLoop()).castTo(Trigger::new); default: return null;} }

    public Trigger controllerLeft() { switch (type) {case PS4: return cmdPS4.share();   case PS5: return cmdPS5.create();  case GuliKit: return guliKit.buttonMinus(); case Xbox: return xbox.start(new EventLoop()).castTo(Trigger::new); default: return null;} }
    public Trigger controllerRight(){ switch (type) {case PS4: return cmdPS4.options(); case PS5: return cmdPS5.options(); case GuliKit: return guliKit.buttonPlus();  case Xbox: return xbox.back(new EventLoop()).castTo(Trigger::new);  default: return null;} }
    
    public Trigger joystickLeft()   { switch (type) {case PS4: return cmdPS4.L3(); case PS5: return cmdPS5.L3(); case GuliKit: return guliKit.buttonLeftJoy();  case Xbox: return xbox.leftStick(new EventLoop()).castTo(Trigger::new);  default: return null;} }
    public Trigger joystickRight()  { switch (type) {case PS4: return cmdPS4.R3(); case PS5: return cmdPS5.R3(); case GuliKit: return guliKit.buttonRightJoy(); case Xbox: return xbox.rightStick(new EventLoop()).castTo(Trigger::new); default: return null;} }

    public Trigger dpadUp()     { switch (type) {case PS4: return cmdPS4.povUp();    case PS5: return cmdPS5.povUp();    case GuliKit: return guliKit.dpadUp();    case Xbox: return xbox.povUp(new EventLoop()).castTo(Trigger::new);    default: return null;} }
    public Trigger dpadRight()  { switch (type) {case PS4: return cmdPS4.povRight(); case PS5: return cmdPS5.povRight(); case GuliKit: return guliKit.dpadRight(); case Xbox: return xbox.povRight(new EventLoop()).castTo(Trigger::new); default: return null;} }
    public Trigger dpadDown()   { switch (type) {case PS4: return cmdPS4.povDown();  case PS5: return cmdPS5.povDown();  case GuliKit: return guliKit.dpadDown();  case Xbox: return xbox.povDown(new EventLoop()).castTo(Trigger::new);  default: return null;} }
    public Trigger dpadLeft()   { switch (type) {case PS4: return cmdPS4.povLeft();  case PS5: return cmdPS5.povLeft();  case GuliKit: return guliKit.dpadLeft();  case Xbox: return xbox.povLeft(new EventLoop()).castTo(Trigger::new);  default: return null;} }

    // ***** STATE METHODS ***** //

    /** <STRONG> GULIKIT ONLY </STRONG> */
    public void setDigLeft(boolean isDigital) {
        if (this.type == Type.GuliKit) guliKit.setDigLeft(isDigital);
    }
    /** <STRONG> GULIKIT ONLY </STRONG> */
    public void setDigRight(boolean isDigital) {
        if (this.type == Type.GuliKit) guliKit.setDigRight(isDigital);
    }

    /** <STRONG> GULIKIT ONLY </STRONG> */
    public boolean isDigLeft() {
        if (this.type == Type.GuliKit) return guliKit.isDigLeft();
        return true;
    }
    /** <STRONG> GULIKIT ONLY </STRONG> */
    public boolean isDigRight() {
        if (this.type == Type.GuliKit) return guliKit.isDigRight();
        return true;
    }
}
