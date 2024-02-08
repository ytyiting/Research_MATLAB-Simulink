% Clean up script for the Autonomous Emergency Braking with high fidelity
% vehicle Dynamics model
%
% This script cleans up the base workspace variables. It is
% triggered by the CloseFcn callback.
%
%   This is a helper script for example purposes and may be removed or
%   modified in the future.

%   Copyright 2022 The MathWorks, Inc.

clearBuses({...
    'BusActorPose',...
    'BusActorRuntime',...
    'BusDiagnostics',...
    'BusObjectDetections1Detections',...
    'BusVehicleLocationOnLane',...
    'BusVehicleMapLocation',...
    'BusVehicleRuntime',...
    'BusActors',...
    'BusDetectionConcatenation1',...
    'BusDetectionConcatenation1Detections',...
    'BusDetectionConcatenation1DetectionsMeasurementParameters',...
    'BusEgoRefPath',...
    'BusMultiObjectTracker1',...
    'BusMultiObjectTracker1Tracks',...
    'BusRadar',...
    'BusRadarDetections',...
    'BusRadarDetectionsMeasurementParameters',...
    'BusRadarDetectionsObjectAttributes',...
    'BusVehiclePose',...
    'BusVision',...
    'BusVisionDetections',...
    'BusVisionDetectionsMeasurementParameters',...
    'BusVisionDetectionsObjectAttributes'});


clear AEBControllerStepSize;
clear DiscretizationDistance;
clear egoInitialPose;
clear maxNumActors;
clear NLMPCTs;
clear numTargetActors;
clear actorProfiles;
clear AEB;
clear cameraParams;
clear Cf;
clear controlHorizon;
clear Cr;
clear egoActorID;
clear egoVehDyn;
clear FCW;
clear Iz; 
clear lf;
clear lr;
clear m;
clear max_ac;
clear max_dc;
clear maxSteer;
clear min_ac;
clear minSteer;
clear nlobj;
clear paramsBusObject;
clear predictionHorizon;
clear radarParams;
clear safetyGoal;
clear scenarioFcnName;
clear tau;
clear tau2;
clear trackingParams;
clear Ts;
clear v_set;
clear BusActionComplete
clear scenarioFileName
clear steeringGain
clear vehSim3D

function clearBuses(buses)
matlabshared.tracking.internal.DynamicBusUtilities.removeDefinition(buses);
end