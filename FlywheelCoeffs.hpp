#pragma once

#include <frc/controller/StateSpaceControllerCoeffs.h>
#include <frc/controller/StateSpaceLoop.h>
#include <frc/controller/StateSpaceObserverCoeffs.h>
#include <frc/controller/StateSpacePlantCoeffs.h>

frc::StateSpacePlantCoeffs<1, 1, 1> MakeFlywheelPlantCoeffs();
frc::StateSpaceControllerCoeffs<1, 1, 1> MakeFlywheelControllerCoeffs();
frc::StateSpaceObserverCoeffs<1, 1, 1> MakeFlywheelObserverCoeffs();
frc::StateSpaceLoop<1, 1, 1> MakeFlywheelLoop();
