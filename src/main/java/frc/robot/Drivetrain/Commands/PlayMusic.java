package frc.robot.Drivetrain.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain.Drivetrain;

public class PlayMusic extends Command {
    private Drivetrain drivetrain;
    private String filepath;

    public PlayMusic(String filepath) {
        this.drivetrain = Drivetrain.getInstance();
        this.filepath = filepath;
    }

    @Override
    public void initialize() {
        drivetrain.playSong(filepath);
    }
}
