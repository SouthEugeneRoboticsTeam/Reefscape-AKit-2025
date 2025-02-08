package org.sert2521.reefscape2025.commands.dispenser


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.sert2521.reefscape2025.subsystems.dispenser.Dispenser

class DispenserRecenter : SequentialCommandGroup(
    DispenserReverse().until{ Dispenser.getDispenserBeambreakBlocked() }
        .withTimeout(2.0),
    DispenserIntake()
)
