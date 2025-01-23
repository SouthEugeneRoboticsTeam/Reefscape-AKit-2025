package org.sert2521.reefscape2025.commands.dispenser


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.sert2521.reefscape2025.subsystems.dispenser.Dispenser

class DispenserRecenter : SequentialCommandGroup(
    DispenserReverse().until{ Dispenser.getBeambreakBlocked() }
        .withTimeout(0.7),
    DispenserReverse().until{ !Dispenser.getBeambreakBlocked() },
    DispenserIntake()
)
