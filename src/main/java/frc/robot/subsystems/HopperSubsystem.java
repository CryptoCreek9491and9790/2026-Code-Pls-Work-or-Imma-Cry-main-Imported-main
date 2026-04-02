package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.HopperSubsystemConstants;
import frc.robot.Constants.HopperSubsystemConstants.IndexerSetpoints;
import frc.robot.Constants.HopperSubsystemConstants.RollerSetpoints;

public class HopperSubsystem extends SubsystemBase {
    
    private final SparkMax indexerMotor =
        new SparkMax(HopperSubsystemConstants.kindexerMotorCandId, MotorType.kBrushless);
    
    
    public HopperSubsystem() {      
      indexerMotor.configure(Configs.HopperSubsystem.INDEXER_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    } 


    private void setindexerPower(double power) {
        indexerMotor.set(power);
    }

    public Command rollCommand() {
        return new InstantCommand(
            () -> {
                this.setindexerPower(IndexerSetpoints.kIndex);
            });
        }

public Command stopRollerCommand() {
        return new InstantCommand(() -> {
            setindexerPower(0);
           }
            ,this);
        }
}
