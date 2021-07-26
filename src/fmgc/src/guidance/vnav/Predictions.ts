// Local imports
import { Common, FlapConf, AccelFactorMode } from './common';
import { EngineModel } from './EngineModel';
import { FlightModel } from './FlightModel';

export interface StepResults {
    pathAngle: number,
    verticalSpeed: number,
    distanceTraveled: number,
    fuelBurned: number,
    timeElapsed: number,
}

export class Predictions {
    /**
     * Placeholder
     * @param initialAltitude altitude at beginning of step, in feet
     * @param stepSize the size of the altitude step, in feet
     * @param econCAS airspeed during climb (taking SPD LIM & restrictions into account)
     * @param econMach mach during climb, after passing crossover altitude
     * @param climbN1 N1% at CLB setting
     * @param zeroFuelWeight zero fuel weight of the aircraft (from INIT B)
     * @param initialFuelWeight weight of fuel at the end of last step
     * @param headwindAtMidStepAlt headwind component (in knots) at initialAltitude + (stepSize / 2); tailwind is negative
     * @param isaDev ISA deviation (in celsius)
     */
    static altitudeStep(
        initialAltitude: number,
        stepSize: number,
        econCAS: number,
        econMach: number,
        climbN1: number,
        zeroFuelWeight: number,
        initialFuelWeight: number,
        headwindAtMidStepAlt: number,
        isaDev: number,
    ): StepResults {
        const midStepAltitude = initialAltitude + (stepSize / 2);
        const theta = Common.getTheta(midStepAltitude, isaDev);
        const delta = Common.getDelta(theta);
        let mach = Common.CAStoMach(econCAS, delta);

        let eas;
        let tas;
        let usingMach = false;
        // If above crossover altitude, use econMach
        if (mach > econMach) {
            mach = econMach;
            eas = Common.machToEAS(mach, delta);
            tas = Common.machToTAS(mach, theta);
            usingMach = true;
        } else {
            eas = Common.CAStoEAS(econCAS, delta);
            tas = Common.CAStoTAS(econCAS, theta, delta);
        }

        // Engine model calculations
        const theta2 = Common.getTheta2(theta, mach);
        const delta2 = Common.getDelta2(delta, mach);
        const correctedN1 = EngineModel.getCorrectedN1(climbN1, theta2);
        const correctedThrust = EngineModel.tableInterpolation(EngineModel.table1506, correctedN1, mach) * 2 * EngineModel.maxThrust;
        const correctedFuelFlow = EngineModel.getCorrectedFuelFlow(correctedN1, mach, midStepAltitude) * 2;
        const thrust = EngineModel.getUncorrectedThrust(correctedThrust, delta2); // in lbf
        const fuelFlow = EngineModel.getUncorrectedFuelFlow(correctedFuelFlow, delta2, theta2) / 60; // in lbs/hour

        const weightEstimate = zeroFuelWeight + initialFuelWeight;

        let pathAngle;
        let verticalSpeed;
        let stepTime;
        let distanceTraveled;
        let fuelBurned;
        let lift = weightEstimate;
        let midStepWeight = weightEstimate;
        let previousMidStepWeight = midStepWeight;
        let iterations = 0;
        do {
            // Assume lift force is equal to weight as an initial approximation
            const liftCoefficient = FlightModel.getLiftCoefficientFromEAS(lift, eas);
            const dragCoefficient = FlightModel.getDragCoefficient(liftCoefficient);
            const accelFactorMode = usingMach ? AccelFactorMode.CONSTANT_MACH : AccelFactorMode.CONSTANT_CAS;
            const accelFactor = FlightModel.getAccelerationFactor(); // TODO
            const pathAngle = FlightModel.getConstantThrustPathAngleFromCoefficients(
                thrust,
                midStepWeight,
                liftCoefficient,
                dragCoefficient,
                accelFactor,
            );

            verticalSpeed = 101.268 * tas * Math.sin(pathAngle); // in feet per minute
            stepTime = stepSize / verticalSpeed; // in minutes
            distanceTraveled = (tas - headwindAtMidStepAlt) * stepTime;
            fuelBurned = (fuelFlow / 60) * stepTime;
            // const endStepWeight = zeroFuelWeight + (initialFuelWeight - fuelBurned);

            // Adjust variables for better accuracy next iteration
            previousMidStepWeight = midStepWeight;
            midStepWeight = zeroFuelWeight + (initialFuelWeight - (fuelBurned / 2));
            lift = midStepWeight * Math.cos(pathAngle);
            iterations++;
        } while (iterations < 5 && Math.abs(previousMidStepWeight - midStepWeight) < 100);

        let result: StepResults;
        result.pathAngle = pathAngle;
        result.verticalSpeed = verticalSpeed;
        result.timeElapsed = stepTime;
        result.distanceTraveled = distanceTraveled;
        result.fuelBurned = fuelBurned;
        return result;
    }

    static handleClimbPredictions(
        fieldElevation: number,
        zeroFuelWeight: number,
        takeoffFuelWeight: number,
        climbN1: number,
        climbCAS: number,
        climbMach: number,
        targetAltitude: number,
    ): number {
        const firstStepAltitude = fieldElevation + 1500;
        const firstStepSize = (Math.ceil((firstStepAltitude + 100) / 1000) * 1000) - firstStepAltitude;

        let finalStepAltitude;
        let finalStepSize;
        if (targetAltitude % 1000 === 0) {
            finalStepAltitude = targetAltitude - 1000;
            finalStepSize = 1000;
        } else {
            finalStepAltitude = (Math.floor((firstStepAltitude - 100) / 1000) * 1000);
            finalStepSize = targetAltitude - finalStepAltitude;
        }


    } 
}