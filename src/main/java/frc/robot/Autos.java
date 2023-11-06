package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmPositions;
import frc.robot.POIManager.POIS;
import frc.robot.driver.CommandOperatorKeypad;
import frc.robot.subsystems.ArmS;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.LightStripS;
import frc.robot.subsystems.ArmS.ArmPosition;
import frc.robot.subsystems.LightStripS.States;
import frc.robot.util.AllianceWrapper;
import frc.robot.util.NomadMathUtil;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Autos {
    private DrivebaseS m_drivebaseS;
    private ArmS m_armS;
    private IntakeS m_intakeS;
    private CommandOperatorKeypad m_keypad;

    /**
     * Trigger that determines whether the drivebase is close enough to its target pose to score a cube.
     */
    public final Trigger m_alignSafeToPlace;
    public final Trigger m_alignSafeToPremove;


    public Autos(DrivebaseS drivebaseS, ArmS armS, IntakeS intakeS, CommandOperatorKeypad keypad) {
        m_drivebaseS = drivebaseS;
        m_armS = armS;
        m_intakeS = intakeS;
        m_keypad = keypad;
        m_alignSafeToPlace = new Trigger(()->{
            Transform2d error = new Transform2d(
                getTargetAlignmentPose(), m_drivebaseS.getPose());
            if (isCubeSelected()) {
                return
                Math.abs(error.getRotation().getRadians()) < Units.degreesToRadians(3) &&
                Math.abs(error.getX()) < 0.1 &&
                Math.abs(error.getY()) < 0.1;
            } else {
                return
                Math.abs(error.getRotation().getRadians()) < Units.degreesToRadians(2) &&
                Math.abs(error.getX()) < 0.02 &&
                Math.abs(error.getY()) < Units.inchesToMeters(0.5);
            }
        });
        m_alignSafeToPremove = new Trigger(()->{
            Transform2d error = new Transform2d(
                getTargetAlignmentPose(), m_drivebaseS.getPose());
            if (isCubeSelected()) {
                return
                Math.abs(error.getRotation().getRadians()) < Units.degreesToRadians(10) &&
                Math.abs(error.getX()) < 0.5 &&
                Math.abs(error.getY()) < 0.5;
            } else {
                return
                Math.abs(error.getRotation().getRadians()) < Units.degreesToRadians(10) &&
                Math.abs(error.getX()) < 0.5 &&
                Math.abs(error.getY()) < 0.5;
            }
        });
    }

    public Pose2d getTargetAlignmentPose() {
        return POIManager.ownCommunity().get(
            (int) m_keypad.get() % 9
        ).transformBy(isCubeSelected() ? new Transform2d() : m_intakeS.getConeCenterOffset());
    }
    public ArmPosition getTargetArmPosition() {
        double node = m_keypad.get();
        if (node <= 8) {
            return ArmConstants.SCORE_HYBRID_POSITION;
        } else if (node <= 17) { // mid
            if (node % 3 == 1) {//cube
                return ArmConstants.SCORE_MID_CUBE_POSITION;
            } else {
                return ArmConstants.SCORE_MID_CONE_POSITION;
            }
        } else { // high
            if (node % 3 == 1) {//cube
                return ArmConstants.SCORE_HIGH_CUBE_POSITION;
            } else {
                return ArmConstants.SCORE_HIGH_CONE_POSITION;
            }
        }
    }
    public ArmPosition getPrescoreArmPosition() {
        double node = m_keypad.get();
        if (node <= 8) {
            return ArmConstants.SCORE_HYBRID_POSITION;
        } else if (node <= 17) { // mid
            if (node % 3 == 1) {//cube
                return ArmConstants.SCORE_MID_CUBE_POSITION;
            } else {
                return ArmConstants.SCORE_MID_CONE_POSITION;
            }
        } else { // high
            if (node % 3 == 1) {//cube
                return ArmConstants.SCORE_MID_CUBE_POSITION;
            } else {
                return ArmConstants.SCORE_HIGH_CONE_POSITION;
            }
        }
    }
    public boolean isCubeSelected() {
        double node = m_keypad.get();
        return (node <= 8) || (node > 8 && node % 3 == 1);
    }
    public boolean isHybridSelected() {
        double node = m_keypad.get();
        return (node <= 8);
    }
    /**
     * Command factory for 
     */
    private Command alignToSelectedScoring() {
        return  m_drivebaseS.chasePoseC(this::getTargetAlignmentPose);
    }

        /**
     * Command factory for intaking a game piece.
     * @param position The arm position
     * @param isCube Whether the intake should be in cube mode or cone mode.
     * @return
     */
    public Command armIntakeCG(ArmPosition position, ArmPosition prestow, boolean isCube) {
        return 
        sequence(
            // Start intaking, and stop when a piece is detected.\
            deadline(
                waitSeconds(0.3).andThen(m_intakeS.intakeUntilBeamBreakC(isCube)).andThen(m_intakeS.intakeC(()->isCube).withTimeout(0.2)).asProxy(),
                // move to arm position while intaking.
                m_armS.goToPositionIndefiniteC(position)

            ),
            parallel(
                // Wait a bit, then pulse the intake to ensure piece collection.
                waitSeconds(isCube ? 0: 0.75).andThen(m_intakeS.intakeC(()->isCube).withTimeout(isCube ? 1.5 : 0.75)).asProxy(),
                // stow the arm
                m_armS.goToPositionC(()->prestow).andThen( m_armS.goToPositionC(()->isCube? ArmPositions.CUBE_STOW : ArmPositions.STOW)),
                run(()->LightStripS.getInstance().requestState(isCube ? States.IntakedCube : States.IntakedCone)).asProxy().withTimeout(0.75)
            )
        );
    }

    public Command armIntakeCG(ArmPosition position, boolean isCube) {
        return armIntakeCG(position, isCube? ArmPositions.CUBE_STOW : ArmPositions.STOW, isCube);
    }

    public Command armIntakeSelectedCG(ArmPosition cubePosition, ArmPosition conePosition, BooleanSupplier isCube) {
        return either(
            armIntakeCG(cubePosition, true), armIntakeCG(conePosition, false), isCube);
    }
    public Command autoScoreSequenceCG() {
        return sequence(
                m_armS.goToPositionC(this::getTargetArmPosition).until(m_keypad.leftGrid().and(m_keypad.centerGrid()).and(m_keypad.rightGrid())),
                        m_intakeS.outtakeC(this::isCubeSelected, this::isHybridSelected).withTimeout(0.4),
                        m_armS.stowC()
            )
            .deadlineWith(run(()->LightStripS.getInstance().requestState(States.Scoring)));

    }

    public Command alignScore(){
        return sequence(
                deadline(
                    sequence(
                        sequence(
                             waitUntil(m_alignSafeToPremove),
                            m_armS.goToPositionC(this::getTargetArmPosition),
                            waitUntil(m_alignSafeToPlace)
                        )//.until(m_alignSafeToPlace)//,
                        //m_armS.goToPositionC(this::getTargetArmPosition)
                    ),
                    alignToSelectedScoring().asProxy(),
                    m_intakeS.run(()->{
                        if (this.isCubeSelected() && !this.isHybridSelected()) {
                        m_intakeS.intakeCube(0.5);
                        }
                    })
                ).until(m_keypad.leftGrid().and(m_keypad.centerGrid()).and(m_keypad.rightGrid())),
                m_intakeS.outtakeC(this::isCubeSelected).withTimeout(0.4),
                m_armS.stowC() 
            );
    }



    //Autonomous Commands:
    
    // region oldAutos
    public Command highConeBalance(int blueColumn){
        
        return sequence(
            runOnce(
                ()->m_drivebaseS.resetPose(
                    NomadMathUtil.mirrorPose(POIManager.BLUE_COMMUNITY.get(blueColumn), AllianceWrapper.getAlliance())
                )),
            
            m_keypad.blueSetpointCommand(blueColumn, 2),
            deadline(
                sequence(
                    m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION).withTimeout(3),
                    m_intakeS.outtakeC(()->false).withTimeout(0.4)
                ),
                alignToSelectedScoring()
            ),

            m_armS.goToPositionC(ArmConstants.CLIMBING_POSITION),
            m_drivebaseS.chargeStationAlignC(),
            m_drivebaseS.xLockC()
            //m_drivebaseS.chargeStationBatteryFirstC()
        ).finallyDo((end)->m_drivebaseS.drive(new ChassisSpeeds()));
    }

    public Command ninePointAuto(){
        
        return sequence(
            runOnce(
                ()->m_drivebaseS.resetPose(
                    NomadMathUtil.mirrorPose(POIManager.BLUE_COMMUNITY.get(0), AllianceWrapper.getAlliance())
                )),
            
            m_keypad.blueSetpointCommand(0, 2),
            deadline(
                sequence(
                    m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION).withTimeout(3).asProxy(),
                    m_intakeS.outtakeC(()->false).withTimeout(0.4).asProxy()
                ),
                alignToSelectedScoring().asProxy()
            ),
            m_armS.goToPositionC(ArmConstants.STOW_POSITION).asProxy(),
            m_drivebaseS.chasePoseC(
                ()->NomadMathUtil.mirrorPose(new Pose2d(6, 0.87, Rotation2d.fromRadians(Math.PI)), AllianceWrapper.getAlliance()))
            .asProxy()
            //m_drivebaseS.chargeStationBatteryFirstC()
        ).finallyDo((end)->m_drivebaseS.drive(new ChassisSpeeds()));
    }


    // region newAutos
    public Command highConeHighCubeHPSide() {
        var pathGroup = PathPlanner.loadPathGroup("High Cone High Cube",
        new PathConstraints(2, 2),
        new PathConstraints(4, 2.5));
        return sequence(
            m_drivebaseS.resetPoseToBeginningC(pathGroup.get(0)),
            // Target the HP-side high cone
            m_keypad.blueSetpointCommand(8, 2),
            // Step 1: Align and score
            deadline(
                sequence(
                    m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION),
                    m_intakeS.outtakeC(()->false).withTimeout(0.4)
                )//,
                // sequence(
                //     alignToSelectedScoring().until(m_alignSafeToPlace)
                //     .andThen(m_drivebaseS.stopC())
                // )
            ),
            // Step 2: Fetch cube 1
            m_keypad.blueSetpointCommand(7, 2),
            deadline(
                m_drivebaseS.pathPlannerCommand(pathGroup.get(0)),
                waitSeconds(0.5).andThen(m_intakeS.intakeC(()->true)),
                m_armS.goToPositionC(ArmConstants.GROUND_CUBE_INTAKE_POSITION)
                // drive from first cone score to cube
                
            ).withTimeout(4),
            // Drive back while stowing cube
            // when we're close enough to score, move on
            deadline(
                sequence(
                    m_drivebaseS.pathPlannerCommand(pathGroup.get(1)),
                    alignToSelectedScoring()
                ).until(m_alignSafeToPlace),
                m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CUBE_POSITION),
                m_intakeS.run(()->m_intakeS.intakeCube(0.5))
            ),
            m_intakeS.outtakeC(()->true).withTimeout(0.4)
         );
    }
    public Command midCubeHPAddon() {
        var pathGroup = PathPlanner.loadPathGroup("High Cone High Cube",
        new PathConstraints(2, 2),
        new PathConstraints(4, 2.5));
        return sequence(
            m_keypad.blueSetpointCommand(7, 1),
            // head back out
            deadline(
                m_drivebaseS.pathPlannerCommand(pathGroup.get(2)).andThen(m_drivebaseS.stopOnceC()),
                waitSeconds(0.5).andThen(m_intakeS.intakeC(()->true)),
                m_armS.goToPositionC(ArmConstants.GROUND_CUBE_INTAKE_POSITION)
                // drive from first cone score to cube
               
            ).withTimeout(4),

            // Drive back while stowing cube
            // when we're close enough to score, move on
            deadline(
                sequence(
                    m_drivebaseS.pathPlannerCommand(pathGroup.get(3)),
                    alignToSelectedScoring()
                ).until(m_alignSafeToPlace),
                m_armS.goToPositionC(ArmConstants.SCORE_MID_CUBE_POSITION),
                m_intakeS.run(()->m_intakeS.intakeCube(0.5))
            ),
            //m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CUBE_POSITION),
            // extend, score
            m_intakeS.run(()->m_intakeS.outtakeCube(6)).withTimeout(0.4)
        );
    }

    public Command highConeHighCubeBumpSide() {
        var pathGroup = PathPlanner.loadPathGroup("High Cone High Cube Bump",
        new PathConstraints(2, 2),
        new PathConstraints(4, 2.5));
        return sequence(
            m_drivebaseS.resetPoseToBeginningC(pathGroup.get(0)),
            // Target the HP-side high cone
            m_keypad.blueSetpointCommand(0, 2),
            // Step 1: Align and score
            deadline(
                sequence(
                    m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION),
                    m_intakeS.outtakeC(()->false).withTimeout(0.4)
                )//,
                // sequence(
                //     alignToSelectedScoring().until(m_alignSafeToPlace)
                //     .andThen(m_drivebaseS.stopC())
                // )
            ),
            // Step 2: Fetch cube 1
            m_keypad.blueSetpointCommand(1, 1),
            deadline(
                m_drivebaseS.pathPlannerCommand(pathGroup.get(0)),
                waitSeconds(0.5).andThen(m_intakeS.intakeC(()->true)),
                m_armS.goToPositionC(ArmConstants.GROUND_CUBE_INTAKE_POSITION)
                // drive from first cone score to cube
                
            ).withTimeout(4),
            // Drive back while stowing cube
            // when we're close enough to score, move on
            deadline(
                sequence(
                    m_drivebaseS.pathPlannerCommand(pathGroup.get(1)),
                    alignToSelectedScoring()
                ).until(m_alignSafeToPlace),
                m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CUBE_POSITION),
                m_intakeS.run(()->m_intakeS.intakeCube(0.5))
            ),
            m_intakeS.outtakeC(()->true).withTimeout(0.4)
         );
    }

    public Command midCubeBumpAddon() {
        var pathGroup = PathPlanner.loadPathGroup("High Cone High Cube Bump",
        new PathConstraints(2, 2),
        new PathConstraints(4, 2.5));
        return sequence(
        m_keypad.blueSetpointCommand(1, 1),
            // head back out
            deadline(
                m_drivebaseS.pathPlannerCommand(pathGroup.get(2)).andThen(m_drivebaseS.stopOnceC()),
                waitSeconds(0.5).andThen(m_intakeS.intakeC(()->true)),
                m_armS.goToPositionC(ArmConstants.GROUND_CUBE_INTAKE_POSITION)
                // drive from first cone score to cube
               
            ).withTimeout(4),

            // Drive back while stowing cube
            // when we're close enough to score, move on
            deadline(
                sequence(
                    m_drivebaseS.pathPlannerCommand(pathGroup.get(3)),
                    alignToSelectedScoring()
                ).until(m_alignSafeToPlace),
                m_armS.goToPositionC(ArmConstants.SCORE_MID_CUBE_POSITION),
                m_intakeS.run(()->m_intakeS.intakeCube(0.5))
            ),
            //m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CUBE_POSITION),
            // extend, score
            m_intakeS.run(()->m_intakeS.outtakeCube(6)).withTimeout(0.4)
         );
    }

    public Command highConeBumpAddon() {
        var pathGroup = PathPlanner.loadPathGroup("High Cone High Cube Bump",
        new PathConstraints(2, 2),
        new PathConstraints(4, 2.5));

        var secondConePathGroup = PathPlanner.loadPathGroup("2nd Tipped Cone Bump",
        new PathConstraints(4, 2.5));
        return sequence(
        m_keypad.blueSetpointCommand(2, 2),
            // head back out
            deadline(
                m_drivebaseS.pathPlannerCommand(pathGroup.get(2)).andThen(m_drivebaseS.stopOnceC()),
                waitSeconds(0.5).andThen(m_intakeS.intakeC(()->false).until(m_intakeS::acquiredCone)),
                m_armS.goToPositionC(ArmConstants.ArmPositions.BACK_TIPPED_FLOOR)
                // drive from first cone score to cube
               
            ).withTimeout(4),

            // Drive back while stowing cube
            // when we're close enough to score, move on
            deadline(
                sequence(
                    m_drivebaseS.pathPlannerCommand(secondConePathGroup.get(0)),
                    alignToSelectedScoring().until(m_alignSafeToPlace)
                ),
                m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION),
                m_intakeS.run(()->m_intakeS.intakeCone(2))
            ),

            //m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CUBE_POSITION),
            // extend, score
            m_intakeS.run(()->m_intakeS.outtakeCone(6)).withTimeout(0.4)
         );
    }

    public Command cubePickupClimbBumpAddon() {
        var pathGroup = PathPlanner.loadPathGroup("High Cone High Cube Bump",
        new PathConstraints(2, 2),
        new PathConstraints(4, 2.5));
        return deadline(
            m_drivebaseS.pathPlannerCommand(pathGroup.get(4)),
            m_intakeS.intakeC(()->true).until(m_intakeS::acquiredCube),
            m_armS.goToPositionC(ArmConstants.GROUND_CUBE_INTAKE_POSITION)
        ).andThen(
                sequence(
                    parallel(
                        m_drivebaseS.chargeStationAlignC(),
                        m_armS.goToPositionC(ArmPositions.CUBE_STOW)
                    )
                )
        );
    }

    public Command highConeHighConeCubePickupClimb() {
        var pathGroup = PathPlanner.loadPathGroup("High Cone High Cone Bump",
        new PathConstraints(2, 2),
        new PathConstraints(4, 2.5));
        return sequence(
            m_drivebaseS.resetPoseToBeginningC(pathGroup.get(0)),
            // Target the HP-side high cone
            m_keypad.blueSetpointCommand(0, 2),
            // Step 1: Align and score
            deadline(
                sequence(
                    m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION),
                    m_intakeS.outtakeC(()->false).withTimeout(0.4)
                )//,
                // sequence(
                //     alignToSelectedScoring().until(m_alignSafeToPlace)
                //     .andThen(m_drivebaseS.stopC())
                // )
            ),
            // Step 2: Fetch cone
            m_keypad.blueSetpointCommand(2, 1),
            deadline(
                m_drivebaseS.pathPlannerCommand(pathGroup.get(0)),
                waitSeconds(0.5).andThen(m_intakeS.intakeC(()->false).until(m_intakeS::acquiredCone)),
                m_armS.goToPositionC(ArmConstants.ArmPositions.BACK_TIPPED_FLOOR)
                // drive from first cone score to cube
                
            ).withTimeout(4),
            // Drive back while stowing cube
            // when we're close enough to score, move on
            deadline(
                sequence(
                    m_drivebaseS.pathPlannerCommand(pathGroup.get(1)),
                    alignToSelectedScoring()
                ).until(m_alignSafeToPlace),
                m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION),
                m_intakeS.run(()->m_intakeS.intakeCone(0.5))
            ),
            m_intakeS.outtakeC(()->false).withTimeout(0.4),
            // go out, pickup cube, climb
            deadline(
                m_drivebaseS.pathPlannerCommand(pathGroup.get(4)),
                m_intakeS.intakeC(()->true).until(m_intakeS::acquiredCube),
                m_armS.goToPositionC(ArmConstants.GROUND_CUBE_INTAKE_POSITION)
            ).andThen(
                    sequence(
                        parallel(
                            m_drivebaseS.chargeStationAlignC(),
                            m_armS.goToPositionC(ArmPositions.CUBE_STOW)
                        )
                    )
            )
         );
    
    }

    
}
