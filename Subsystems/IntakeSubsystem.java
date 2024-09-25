package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDSubsystem;

public class IntakeSubsystem extends NEDSubsystem {
    private final Obot obot = Obot.getInstance();
    public enum ExtendoState{
        HOME,
        EXTEND,
        SEMI_EXTEND,
        EXTEND_AUTO,
        EXTEND_TRANSFER
    }
    public enum ActiveState{
        START,
        STOP
    }
    public enum WristState{
        HOME,
        TRANSFER,
        OUT_EXTENDED,
        OUT_RETRACTED,
        FIRST_PICK,
        SECOND_PICK,
    }
    public enum CapacState{
        CLOSED,
        OPENED
    }
    private ExtendoState extendoState;
    private WristState wristState;
    private ActiveState activeState;
    private CapacState capacState;
    private double EXTENDO_MANUAL_FACTOR=25;
    private double HomeExtendoPos=0;
    private double ExtendExtendoPos=-1100;
    private double SemiExtendoPos=-600;
    private double ExtendoPosAuto=-1100;
    private double ExtendTransferPos=-10;


    private double HomeWristPos=0.54;
    private double OutExtendedWristPos=0.54;
    private double OutRetractedWristPos=0.54;
    private double TransferWristPos=0.6;
    private double FirstPickWristPos=0;
    private double SecondPickWristPos=0;

    private double CapacOpenPos=0.75;
    private double CapacClosePos=0.18;//24





    public IntakeSubsystem()
    {

    }
    @Override
    public void periodic() {
        obot.Intake.periodic();
    }

    @Override
    public void read() {
        obot.Intake.read();
    }

    @Override
    public void write() {
        obot.Intake.write();
    }

    @Override
    public void reset() {

    }
    public void update(CapacState state){
        capacState=state;
        switch (capacState){
            case CLOSED:
                obot.capac.setPosition(CapacClosePos);
                break;
            case OPENED:
                obot.capac.setPosition(CapacOpenPos);
                break;
        }
    }

    public void update(ExtendoState state)
    {
        extendoState=state;
        switch(extendoState)
        {
            case HOME:
                obot.Intake.setMotionProfileTargetPosition(HomeExtendoPos);
                break;
            case EXTEND:
                obot.Intake.setMotionProfileTargetPosition(ExtendExtendoPos);
                break;
            case SEMI_EXTEND:
                obot.Intake.setMotionProfileTargetPosition(SemiExtendoPos);
                break;
            case EXTEND_AUTO:
                obot.Intake.setMotionProfileTargetPosition(ExtendoPosAuto);
            case EXTEND_TRANSFER:
                obot.Intake.setMotionProfileTargetPosition(ExtendTransferPos);
        }
    }

    public void update(WristState state)
    {
        wristState=state;
        switch (wristState){
            case HOME:
                obot.Wrist.setPosition(HomeWristPos);
                break;
            case FIRST_PICK:
                obot.Wrist.setPosition(FirstPickWristPos);
                break;
            case SECOND_PICK:
                obot.Wrist.setPosition(SecondPickWristPos);
                break;
            case TRANSFER:
                obot.Wrist.setPosition(TransferWristPos);
                break;
            case OUT_EXTENDED:
                obot.Wrist.setPosition(OutExtendedWristPos);
                break;
            case OUT_RETRACTED:
                obot.Wrist.setPosition(OutRetractedWristPos);
        }
    }
    public void update(ActiveState state){
        activeState=state;
        switch (activeState){
            case START:
                obot.Active.setPower(-1);
                break;
            case STOP:
                obot.Active.setPower(0);
        }
    }
}
