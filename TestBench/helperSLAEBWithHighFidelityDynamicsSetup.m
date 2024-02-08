function helperSLAEBWithHighFidelityDynamicsSetup(rrAppObj, scenarioSimulationObj, nvp)
%helperSLAEBWithHighFidelityDynamicsSetup create required buses,
%variables for simulating the
% Autonomous Emergency Braking (AEB) with High Fidelity Dynamics
% Example.
%
% Initializes the AEBWithHighFidelityDynamicsTestBench.slx model by
% creating data in base workspace.
%
% Optional inputs
%   scenarioFcnName:
%     - Name of function which returns scenario which is
%       compatible with AEBWithHighFidelityDynamicsTestBench.slx
%     - Valid values are:
%           "scenario_01_USHighway_EntryRamp.rrscenario"
%           "scenario_02_USHighway_Pedestrian.rrscenario"
%
% Examples of calling this function:
%
%    helperSLAEBWithHighFidelityDynamicsSetup(rrAppObj, scenarioSimulationObj, scenarioFileName= "scenario_01_USHighway_EntryRamp")
%    helperSLAEBWithHighFidelityDynamicsSetup(rrAppObj, scenarioSimulationObj, scenarioFileName= "scenario_02_USHighway_Pedestrian")
%
% This helper function initializes the AEB with high fidelity dynamics
% example model. It loads necessary control constants and sets up the buses
% required for the referenced models
%
%   This is a helper function for example purposes and may be removed or
%   modified in the future.

% Copyright 2022-2023 The MathWorks, Inc.

arguments
    rrAppObj = [];
    scenarioSimulationObj =[];
    nvp.scenarioFileName {mustBeMember(nvp.scenarioFileName,...
        ["scenario_01_USHighway_EntryRamp",...
        "scenario_02_USHighway_Pedestrian",...
        "scenario_02_CarToCar_RearStationary"])} = "scenario_02_CarToCar_RearStationary";
end

scenarioFileName = nvp.scenarioFileName;
assignin('base', 'scenarioFileName', scenarioFileName);

% Initialize simulation sample time
Ts = 0.05;
assignin('base', 'Ts', Ts); % Simulation sample time  (s)

egoActorID = 1;
egoSetSpeed = 2.778; % default speed

egoInitialPose = struct;
egoInitialPose.ActorID = 1;
egoInitialPose.Position = [0 0 0];
egoInitialPose.Velocity = [0 0 0];
egoInitialPose.Roll = 0;
egoInitialPose.Pitch = 0;

egoInitialPose.Yaw = 0;
egoInitialPose.AngularVelocity = [0 0 0];

numActors = 2;

%Default actor poses.
actorInitialPoses = repmat(struct(...
    'ActorID', 0, ...
    'Position',[0,0,0], ...
    'Velocity', [0,0,0], ...
    'Roll', 0, ...
    'Pitch', 0, ...
    'Yaw', 0, ...
    'AngularVelocity', [0 0 0]), 1, numActors);

initActorProfile = struct(...
    'ActorID',1,...
    'ClassID',1,...
    'Length',4.7,...
    'Width',1.8,...
    'Height',1.4,...
    'OriginOffset',[0 0 0],...
    'FrontOverhang',0,...
    'RearOverhang',0,...
    'Wheelbase',0,...
    'Color',[0 0 0]);

actorProfiles = repmat(initActorProfile, 1, numActors);

% Default Vehicle Positions and Rotations
initVehiclePosition = struct(...
    'InitialPos',[0 0 0],...
    'InitialRot',[0 0 0]);

vehSim3D = repmat(initVehiclePosition, 1, numActors-1);

if ~isempty(scenarioSimulationObj)
    
    % Get current RoadRunner application status
    rrStatus = rrAppObj.status();
    % Get the current scenario status
    scenarioStatus = rrStatus.Scenario;

    % If the scenario status from the RoadRunner has empty then open the
    % scenario specified from the input argument.
    if ~isempty(scenarioStatus)
        % Get the file name from scenarioStatus.
        [~,fileName,~] = fileparts(scenarioStatus.Filename);
         % Open scenario if it is not already opened
        if(fileName ~= scenarioFileName)
            openScenario(rrAppObj, strjoin([scenarioFileName, '.rrscenario'],""));
        end
    else
        % Assign the default file Name variable with the input scenario
        % file name.
        fileName = scenarioFileName;
        openScenario(rrAppObj, strjoin([fileName, '.rrscenario'],""));
    end

    % Read actor profiles from RoadRunner Scenario
    worldActor = scenarioSimulationObj.getScenario();
    world = worldActor.actor_spec.world_spec;

    numActors = length(world.actors);
    
    actorProfiles = helperGetActorProfiles(world.actors,OriginOffset="RearAxleCenter");
    actors = world.actors;

    %Get actor initial poses from the scenario. 
    for i = 1:numActors

        id = str2double(actors(i).actor_spec.id);
        vehicleInitialPose = helperGetActorPose(world.actors,id,egoSetSpeed);
        actorInitialPoses(id).ActorID = vehicleInitialPose.ActorID;
        actorInitialPoses(id).Position  = vehicleInitialPose.Position;
        actorInitialPoses(id).Velocity  = vehicleInitialPose.Velocity;
        actorInitialPoses(id).Roll  =  vehicleInitialPose.Roll; 
        actorInitialPoses(id).Pitch   =  vehicleInitialPose.Pitch;
        actorInitialPoses(id).Yaw = vehicleInitialPose.Yaw;
        actorInitialPoses(id).AngularVelocity = vehicleInitialPose.AngularVelocity;
    end
    % Get ego ActorID
    for i = 1:length(world.behaviors)
        if contains(upper(world.behaviors(i).asset_reference),upper('AEBHighFidelityDynamics.rrbehavior'))
            egoBehavior = world.behaviors(i).id;
            break;
        end
    end
    for i = 1:length(world.actors)
        id = str2double(world.actors(i).actor_spec.id);
        if isequal(world.actors(i).actor_spec.behavior_id,egoBehavior)
            egoActorID = id;
        end
    end

    egoSetSpeed = str2double(rrAppObj.getScenarioVariable('VUT_Speed')); % get speed, m/s
    egoInitialSpeed = str2double(rrAppObj.getScenarioVariable('VUT_Speed')); % get speed, m/s
    egoInitialPose = helperGetActorPose(world.actors, egoActorID, egoInitialSpeed);
end

% Create AEB buses
helperCreateAEBBusObjects("numTargetActors", numActors-1, "refPathSize", 10000);

assignin('base', 'maxNumActors', numActors);
assignin('base', 'numTargetActors',  numActors-1);
assignin('base', 'egoActorID', egoActorID);

% Arc length between interpolated ego path points
DiscretizationDistance = 0.2;
assignin('base','DiscretizationDistance', DiscretizationDistance);

%% General Model parameters
AEBControllerStepSize = Ts;
assignin('base','AEBControllerStepSize', AEBControllerStepSize); % Step size for AEB Controller (s)

NLMPCTs = Ts;
assignin('base','NLMPCTs', NLMPCTs); % Sample time for NLMPC Controller (s)

%% Sensor configuration
% Sedan vehicle dimensions 
centerToFront = 1.513;
centerToRear  = 1.305;
frontOverhang = 0.911;
rearOverhang  = 1.119;
vehicleWidth  = 1.842;
vehicleHeight = 1.517;
vehicleLength = centerToFront + centerToRear + frontOverhang + rearOverhang;

sedanDims = vehicleDimensions(vehicleLength,vehicleWidth,vehicleHeight, ...
    'FrontOverhang',frontOverhang,'RearOverhang',rearOverhang);

% Long Range Radar
radarParams.azRes = 4;
radarParams.rangeRes = 2.5;
radarParams.rangeRateRes = 0.5;
radarParams.radarRange = 174;
radarParams.azFov = 20;
radarParams.position        = ...       % Position with respect to rear axle (m)
    [ vehicleLength - rearOverhang, ...
      0,...
      0.8];
radarParams.positionSim3d   = ...       % Position with respect to vehicle center (m)
    radarParams.position - ...
    [ vehicleLength/2 - rearOverhang, 0, 0];
radarParams.rotation = [ 0, 0, 0];      % [roll, pitch, yaw] (deg)
assignin('base','radarParams', radarParams);

% 1.2MP, FoV = 49 deg
cameraParams.focalLength = [800, 800];
cameraParams.principalPoint = [320, 240];
cameraParams.imageSize = [480, 640];
cameraParams.cameraRange = 150;

cameraParams.position        = ...      % Position with respect to rear axle (m)
    [ 1.8750, ...                 %  - X (by the rear-view mirror)
      0,...                       %  - Y (center of vehicle width)
      1.2];                       %  - Height
cameraParams.PositionSim3d   = ...      % Position with respect to vehicle center (m)
    cameraParams.position - ...         %  - Reduce position X by distance from vehicle center to rear axle
    [ vehicleLength/2 - rearOverhang,...
      0, 0];
cameraParams.rotation = [0, 0, 0];          % Rotation [roll, pitch, yaw] (deg)
assignin('base','cameraParams', cameraParams);

%% Tracking and Sensor Fusion Parameters                        Units
trackingParams.clusterSize = 4;        % Distance for clustering               (m)
trackingParams.assigThresh = 50;      % Tracker assignment threshold          (N/A)
trackingParams.M           = 2;        % Tracker M value for M-out-of-N logic  (N/A)
trackingParams.N           = 3;        % Tracker N value for M-out-of-N logic  (N/A)
trackingParams.numCoasts   = 3;        % Number of track coasting steps        (N/A)
trackingParams.numTracks   = 20;       % Maximum number of tracks              (N/A)
trackingParams.numSensors  = 2;        % Maximum number of sensors             (N/A)

% Position and velocity selectors from track state
% The filter initialization function used in this example is initcvekf that
% defines a state that is: [x;vx;y;vy;z;vz].
trackingParams.posSelector = [1,0,0,0,0,0; 0,0,1,0,0,0]; % Position selector   (N/A)

% Assign TrackingParams struct in base workspace
assignin('base','trackingParams',trackingParams);

%% Controller parameters
maxSteer = 1.13; % Maximum steering angle (rad)
assignin('base', 'maxSteer', maxSteer);

minSteer = -1.13; % Minimum steering angle (rad)
assignin('base', 'minSteer', minSteer);

min_ac = -3;
assignin('base','min_ac', min_ac);      % Minimum acceleration   (m/s^2)

max_ac = 3;
assignin('base', 'max_ac', max_ac);     % Maximum acceleration   (m/s^2)

predictionHorizon = 10; % Number of steps for preview    (N/A)
assignin('base', 'predictionHorizon', predictionHorizon);

controlHorizon = 2;  % The number of MV moves to be optimized at control interval. (N/A)
assignin('base', 'controlHorizon', controlHorizon);

assignin('base','max_dc', -9.8); % Maximum deceleration   (m/s^2)

%% FCW parameters
FCW.timeToReact  = 1.2;         % driver reaction time                   (sec)
FCW.driver_decel = 4.0;         % driver braking deceleration            (m/s^2)

% Assign FCW struct in base workspace
assignin('base','FCW',FCW);

% AEB parameters
AEB.PB1_decel = 3.8;            % 1st stage Partial Braking deceleration (m/s^2)
AEB.PB2_decel = 5.3;            % 2nd stage Partial Braking deceleration (m/s^2)
AEB.FB_decel  = 9.8;            % Full Braking deceleration              (m/s^2)
AEB.headwayOffset = 3.7;        % headway offset                         (m)
AEB.timeMargin = 0.08;             % headway time margin                    (sec)

% Assign AEB struct in base workspace
assignin('base','AEB',AEB);

vehSim3D = vehicleSim3DParams(actorInitialPoses,actorProfiles);
egoVehDyn = egoVehicleDynamicsParams(egoInitialPose, actorProfiles(egoActorID));
% Vehicle Parameters
% Dynamics modeling parameters
m       = 1575;                         % Total mass of vehicle                          (kg)
Iz      = 2875;                         % Yaw moment of inertia of vehicle               (m*N*s^2)
lf      = egoVehDyn.CGToFrontAxle;      % Longitudinal distance from c.g. to front axle (m)
lr      = egoVehDyn.CGToRearAxle;       % Longitudinal distance from c.g. to rear axle  (m)
Cf      = 19000;                        % Cornering stiffness of front tires             (N/rad)
Cr      = 33000;                        % Cornering stiffness of rear tires              (N/rad)
tau     = 0.5;                          % Longitudinal time constant (throttle)          (N/A)
tau2    = 0.07;                         % Longitudinal time constant (brake)             (N/A)


%% NLMPC params
% create nlmpc object with 7 prediction model states, 3 prediction model
% outputs.
nlobj = nlmpc(7,3,'MV',[1 2],'MD',3,'UD',4);
% two MV (Manipulated Variables) signals: acceleration and steering.
% measured disturbance (MD) : the product of the road curvature and the longitudinal velocity
% unmeasured disturbance (UD) : white noise.

% Specify the controller sample time, prediction horizon, and control
% horizon.
nlobj.Ts = Ts;
nlobj.PredictionHorizon = predictionHorizon;
nlobj.ControlHorizon = controlHorizon;

% Specify the state function for the nonlinear plant model and its
% Jacobian.
nlobj.Model.StateFcn = "helperNLMPCStateFcn";
nlobj.Jacobian.StateFcn = "helperNLMPCStateJacFcn";

%
% Specify the output function for the nonlinear plant model and its
% Jacobian. The output variables are:
%
% * Longitudinal velocity
% * Lateral deviation
% * Sum of the yaw angle and yaw angle output disturbance
%
nlobj.Model.OutputFcn = "helperNLMPCOutputFcn";
nlobj.Jacobian.OutputFcn = "helperNLMPCJacOutputFcn";

%
% Set the constraints for manipulated variables.
nlobj.MV(1).Min = min_ac;       % Maximum acceleration (m/s^2)
nlobj.MV(1).Max = max_ac;       % Minimum acceleration  (m/s^2)
nlobj.MV(2).Min = minSteer;     % Minimum steering angle  (rad)
nlobj.MV(2).Max = maxSteer;     % Maximum steering angle  (rad)

% Set the scale factors.
nlobj.OV(1).ScaleFactor = 15;   % Typical value of longitudinal velocity
nlobj.OV(2).ScaleFactor = 0.5;  % Range for lateral deviation
nlobj.OV(3).ScaleFactor = 0.5;  % Range for relative yaw angle
nlobj.MV(1).ScaleFactor = 6;    % Range of steering angle
nlobj.MV(2).ScaleFactor = 2.26; % Range of acceleration
nlobj.MD(1).ScaleFactor = 0.2;  % Range of Curvature

% Specify the weights in the standard MPC cost function. The third output,
% yaw angle, is allowed to float because there are only two manipulated
% variables to make it a square system. In this example, there is no
% steady-state error in the yaw angle as long as the second output, lateral
% deviation, reaches 0 at steady state.
nlobj.Weights.OutputVariables = [1 1 0];

% Penalize acceleration change more for smooth driving experience.
nlobj.Weights.ManipulatedVariablesRate = [0.3 0.1];

params = {m,Iz,Cf,Cr,lf,lr,tau}';
nlobj.Model.NumberOfParameters = numel(params);

% Load controller test bench model
controllerRefModel = 'AEBController';
wasRefModelLoaded = bdIsLoaded(controllerRefModel);
if ~wasRefModelLoaded
    load_system(controllerRefModel)
end
mdl = [controllerRefModel,  '/NLMPC Controller/Nonlinear MPC Controller'];

% clear and create parasBusObject if it exists
evalin( 'base', 'clear(''paramsBusObject'')' );
createParameterBus(nlobj, [mdl '/Nonlinear MPC Controller'], 'paramsBusObject', params);
bdclose(mdl);

assignin('base', 'nlobj', nlobj);
assignin('base','egoVehDyn',egoVehDyn);

%% Assign vehicle dynamics modeling parameters to base work space
assignin('base', 'm',    m);
assignin('base', 'Iz',   Iz);
assignin('base', 'Cf',   Cf);
assignin('base', 'Cr',   Cr);
assignin('base', 'lf',   lf);
assignin('base', 'lr',   lr);
assignin('base', 'tau',  tau);
assignin('base', 'tau2', tau2);
assignin('base','egoVehDyn',egoVehDyn);
assignin('base','vehSim3D', vehSim3D);

% Goal for collision mitigation >= 90%
safetyGoal = 90;
assignin('base','safetyGoal', safetyGoal);

% set ego velocity (m/s)
assignin('base','v_set', egoSetSpeed);
assignin('base','actorProfiles', actorProfiles);

% Load vehicle dynamics model
vehDynRefModel = 'VehDyn14DOF';
wasRefModelLoaded = bdIsLoaded(vehDynRefModel);
if ~wasRefModelLoaded
    load_system(vehDynRefModel)
end
mdl = [vehDynRefModel,  '/Passenger Vehicle/Chassis and Tires/Wheels and Tires/VDBS/Tires'];
set_param(mdl, 'LabelModeActiveChoice', '2');
% Define the steering gain
assignin('base', 'steeringGain', 0.2);

createSim3DBus(); % Create Sim 3D Bus.
end

%% Vehicle dynamics parameters from scenario
function egoVehDyn = egoVehicleDynamicsParams(ego, egoActor)
% Ego pose for vehicle dynamics from RoadRunner Scenario.
egoVehDyn.X0  =  ego.Position(1); % (m)
egoVehDyn.Y0  = -ego.Position(2); % (m)
egoVehDyn.Z0  = -ego.Position(3); % (m)
egoVehDyn.VX0 =  ego.Velocity(1); % (m/s)
egoVehDyn.VY0 = -ego.Velocity(2); % (m/s)

% Adjust sign and unit of yaw
egoVehDyn.Yaw0 = -deg2rad(ego.Yaw); % (rad)
egoVehDyn.Pitch0 = -deg2rad(ego.Pitch); % (rad)
egoVehDyn.Roll0 = deg2rad(ego.Roll); % (rad)

% Longitudinal velocity
egoVehDyn.VLong0 = hypot(egoVehDyn.VX0,egoVehDyn.VY0); % (m/sec)

% Distance from center of gravity to axles
egoVehDyn.CGToFrontAxle = egoActor.Length/2 - egoActor.FrontOverhang;
egoVehDyn.CGToRearAxle  = egoActor.Length/2 - egoActor.RearOverhang;
end

function vehSim3D = vehicleSim3DParams(actorPoses,actorProfiles)
%vehicleSim3DParams vehicle parameters used by Sim 3D

% Number of vehicles in scenario
numVehicles = numel(actorPoses);

% Preallocate struct
vehSim3D = repmat(...
    struct(...
        'Length', 0,...
        'RearOverhang', 0,...
        'InitialPos',[0 0 0],...
        'InitialRot',[0 0 0]),...
    numVehicles,1);
    
for n = 1:numVehicles
    % Vehicle information from driving scenario
    veh = actorPoses(n); 
    
    % Update struct elements
    vehSim3D(n).Length = actorProfiles(n).Length;
    vehSim3D(n).RearOverhang = actorProfiles(n).RearOverhang;
    vehSim3D(n).InitialPos = veh.Position;
    vehSim3D(n).InitialRot = [veh.Roll veh.Pitch veh.Yaw]; 
end
end

function createSim3DBus()
%createSim3DBus creates the sim 3d radar bus.

% Bus object: BusRadar 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'NumDetections';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'IsValidTime';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'boolean';
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'Detections';
elems(3).Dimensions = [50 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'Bus: BusRadarDetections';
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

BusRadar = Simulink.Bus;
BusRadar.HeaderFile = '';
BusRadar.Description = '';
BusRadar.DataScope = 'Auto';
BusRadar.Alignment = -1;
BusRadar.PreserveElementDimensions = 0;
BusRadar.Elements = elems;
clear elems;
assignin('base','BusRadar', BusRadar);

% Bus object: BusRadarDetections 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'Time';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'Measurement';
elems(2).Dimensions = [6 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'MeasurementNoise';
elems(3).Dimensions = [6 6];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'SensorIndex';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'ObjectClassID';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'ObjectAttributes';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'Bus: BusRadarDetectionsObjectAttributes';
elems(6).Complexity = 'real';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

elems(7) = Simulink.BusElement;
elems(7).Name = 'MeasurementParameters';
elems(7).Dimensions = [1 1];
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'Bus: BusRadarDetectionsMeasurementParameters';
elems(7).Complexity = 'real';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = '';
elems(7).Description = '';

BusRadarDetections = Simulink.Bus;
BusRadarDetections.HeaderFile = '';
BusRadarDetections.Description = '';
BusRadarDetections.DataScope = 'Auto';
BusRadarDetections.Alignment = -1;
BusRadarDetections.PreserveElementDimensions = 0;
BusRadarDetections.Elements = elems;
clear elems;
assignin('base','BusRadarDetections', BusRadarDetections);

% Bus object: BusRadarDetectionsMeasurementParameters 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'Frame';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'Enum: drivingCoordinateFrameType';
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'OriginPosition';
elems(2).Dimensions = [3 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'Orientation';
elems(3).Dimensions = [3 3];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'HasElevation';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'boolean';
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'HasVelocity';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'boolean';
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

BusRadarDetectionsMeasurementParameters = Simulink.Bus;
BusRadarDetectionsMeasurementParameters.HeaderFile = '';
BusRadarDetectionsMeasurementParameters.Description = '';
BusRadarDetectionsMeasurementParameters.DataScope = 'Auto';
BusRadarDetectionsMeasurementParameters.Alignment = -1;
BusRadarDetectionsMeasurementParameters.PreserveElementDimensions = 0;
BusRadarDetectionsMeasurementParameters.Elements = elems;
clear elems;
assignin('base','BusRadarDetectionsMeasurementParameters', BusRadarDetectionsMeasurementParameters);

% Bus object: BusRadarDetectionsObjectAttributes 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'TargetIndex';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'SNR';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

BusRadarDetectionsObjectAttributes = Simulink.Bus;
BusRadarDetectionsObjectAttributes.HeaderFile = '';
BusRadarDetectionsObjectAttributes.Description = '';
BusRadarDetectionsObjectAttributes.DataScope = 'Auto';
BusRadarDetectionsObjectAttributes.Alignment = -1;
BusRadarDetectionsObjectAttributes.PreserveElementDimensions = 0;
BusRadarDetectionsObjectAttributes.Elements = elems;
clear elems;
assignin('base','BusRadarDetectionsObjectAttributes', BusRadarDetectionsObjectAttributes);

% Bus object: BusMultiObjectTracker1 
BusMultiObjectTracker1 = Simulink.Bus;
BusMultiObjectTracker1.Description = '';
BusMultiObjectTracker1.DataScope = 'Auto';
BusMultiObjectTracker1.HeaderFile = '';
BusMultiObjectTracker1.Alignment = -1;
BusMultiObjectTracker1.PreserveElementDimensions = false;
saveVarsTmp{1} = Simulink.BusElement;
saveVarsTmp{1}.Name = 'NumTracks';
saveVarsTmp{1}.Complexity = 'real';
saveVarsTmp{1}.Dimensions = [1 1];
saveVarsTmp{1}.DataType = 'double';
saveVarsTmp{1}.Min = [];
saveVarsTmp{1}.Max = [];
saveVarsTmp{1}.DimensionsMode = 'Fixed';
saveVarsTmp{1}.SamplingMode = 'Sample based';
saveVarsTmp{1}.DocUnits = '';
saveVarsTmp{1}.Description = '';
saveVarsTmp{1}(2, 1) = Simulink.BusElement;
saveVarsTmp{1}(2, 1).Name = 'Tracks';
saveVarsTmp{1}(2, 1).Complexity = 'real';
saveVarsTmp{1}(2, 1).Dimensions = [20 1];
saveVarsTmp{1}(2, 1).DataType = 'Bus: BusMultiObjectTracker1Tracks';
saveVarsTmp{1}(2, 1).Min = [];
saveVarsTmp{1}(2, 1).Max = [];
saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(2, 1).DocUnits = '';
saveVarsTmp{1}(2, 1).Description = '';
BusMultiObjectTracker1.Elements = saveVarsTmp{1};
clear saveVarsTmp;
assignin('base','BusMultiObjectTracker1', BusMultiObjectTracker1);

% Bus object: BusMultiObjectTracker1Tracks 
BusMultiObjectTracker1Tracks = Simulink.Bus;
BusMultiObjectTracker1Tracks.Description = '';
BusMultiObjectTracker1Tracks.DataScope = 'Auto';
BusMultiObjectTracker1Tracks.HeaderFile = '';
BusMultiObjectTracker1Tracks.Alignment = -1;
BusMultiObjectTracker1Tracks.PreserveElementDimensions = false;
saveVarsTmp{1} = Simulink.BusElement;
saveVarsTmp{1}.Name = 'TrackID';
saveVarsTmp{1}.Complexity = 'real';
saveVarsTmp{1}.Dimensions = [1 1];
saveVarsTmp{1}.DataType = 'uint32';
saveVarsTmp{1}.Min = [];
saveVarsTmp{1}.Max = [];
saveVarsTmp{1}.DimensionsMode = 'Fixed';
saveVarsTmp{1}.SamplingMode = 'Sample based';
saveVarsTmp{1}.DocUnits = '';
saveVarsTmp{1}.Description = '';
saveVarsTmp{1}(2, 1) = Simulink.BusElement;
saveVarsTmp{1}(2, 1).Name = 'BranchID';
saveVarsTmp{1}(2, 1).Complexity = 'real';
saveVarsTmp{1}(2, 1).Dimensions = [1 1];
saveVarsTmp{1}(2, 1).DataType = 'uint32';
saveVarsTmp{1}(2, 1).Min = [];
saveVarsTmp{1}(2, 1).Max = [];
saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(2, 1).DocUnits = '';
saveVarsTmp{1}(2, 1).Description = '';
saveVarsTmp{1}(3, 1) = Simulink.BusElement;
saveVarsTmp{1}(3, 1).Name = 'SourceIndex';
saveVarsTmp{1}(3, 1).Complexity = 'real';
saveVarsTmp{1}(3, 1).Dimensions = [1 1];
saveVarsTmp{1}(3, 1).DataType = 'uint32';
saveVarsTmp{1}(3, 1).Min = [];
saveVarsTmp{1}(3, 1).Max = [];
saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(3, 1).DocUnits = '';
saveVarsTmp{1}(3, 1).Description = '';
saveVarsTmp{1}(4, 1) = Simulink.BusElement;
saveVarsTmp{1}(4, 1).Name = 'UpdateTime';
saveVarsTmp{1}(4, 1).Complexity = 'real';
saveVarsTmp{1}(4, 1).Dimensions = [1 1];
saveVarsTmp{1}(4, 1).DataType = 'double';
saveVarsTmp{1}(4, 1).Min = [];
saveVarsTmp{1}(4, 1).Max = [];
saveVarsTmp{1}(4, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(4, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(4, 1).DocUnits = '';
saveVarsTmp{1}(4, 1).Description = '';
saveVarsTmp{1}(5, 1) = Simulink.BusElement;
saveVarsTmp{1}(5, 1).Name = 'Age';
saveVarsTmp{1}(5, 1).Complexity = 'real';
saveVarsTmp{1}(5, 1).Dimensions = [1 1];
saveVarsTmp{1}(5, 1).DataType = 'uint32';
saveVarsTmp{1}(5, 1).Min = [];
saveVarsTmp{1}(5, 1).Max = [];
saveVarsTmp{1}(5, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(5, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(5, 1).DocUnits = '';
saveVarsTmp{1}(5, 1).Description = '';
saveVarsTmp{1}(6, 1) = Simulink.BusElement;
saveVarsTmp{1}(6, 1).Name = 'State';
saveVarsTmp{1}(6, 1).Complexity = 'real';
saveVarsTmp{1}(6, 1).Dimensions = [6 1];
saveVarsTmp{1}(6, 1).DataType = 'double';
saveVarsTmp{1}(6, 1).Min = [];
saveVarsTmp{1}(6, 1).Max = [];
saveVarsTmp{1}(6, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(6, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(6, 1).DocUnits = '';
saveVarsTmp{1}(6, 1).Description = '';
saveVarsTmp{1}(7, 1) = Simulink.BusElement;
saveVarsTmp{1}(7, 1).Name = 'StateCovariance';
saveVarsTmp{1}(7, 1).Complexity = 'real';
saveVarsTmp{1}(7, 1).Dimensions = [6 6];
saveVarsTmp{1}(7, 1).DataType = 'double';
saveVarsTmp{1}(7, 1).Min = [];
saveVarsTmp{1}(7, 1).Max = [];
saveVarsTmp{1}(7, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(7, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(7, 1).DocUnits = '';
saveVarsTmp{1}(7, 1).Description = '';
saveVarsTmp{1}(8, 1) = Simulink.BusElement;
saveVarsTmp{1}(8, 1).Name = 'ObjectClassID';
saveVarsTmp{1}(8, 1).Complexity = 'real';
saveVarsTmp{1}(8, 1).Dimensions = [1 1];
saveVarsTmp{1}(8, 1).DataType = 'double';
saveVarsTmp{1}(8, 1).Min = [];
saveVarsTmp{1}(8, 1).Max = [];
saveVarsTmp{1}(8, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(8, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(8, 1).DocUnits = '';
saveVarsTmp{1}(8, 1).Description = '';
saveVarsTmp{1}(9, 1) = Simulink.BusElement;
saveVarsTmp{1}(9, 1).Name = 'TrackLogic';
saveVarsTmp{1}(9, 1).Complexity = 'real';
saveVarsTmp{1}(9, 1).Dimensions = [1 1];
saveVarsTmp{1}(9, 1).DataType = 'Enum: trackLogicType';
saveVarsTmp{1}(9, 1).Min = [];
saveVarsTmp{1}(9, 1).Max = [];
saveVarsTmp{1}(9, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(9, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(9, 1).DocUnits = '';
saveVarsTmp{1}(9, 1).Description = '';
saveVarsTmp{1}(10, 1) = Simulink.BusElement;
saveVarsTmp{1}(10, 1).Name = 'TrackLogicState';
saveVarsTmp{1}(10, 1).Complexity = 'real';
saveVarsTmp{1}(10, 1).Dimensions = [1 3];
saveVarsTmp{1}(10, 1).DataType = 'boolean';
saveVarsTmp{1}(10, 1).Min = [];
saveVarsTmp{1}(10, 1).Max = [];
saveVarsTmp{1}(10, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(10, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(10, 1).DocUnits = '';
saveVarsTmp{1}(10, 1).Description = '';
saveVarsTmp{1}(11, 1) = Simulink.BusElement;
saveVarsTmp{1}(11, 1).Name = 'IsConfirmed';
saveVarsTmp{1}(11, 1).Complexity = 'real';
saveVarsTmp{1}(11, 1).Dimensions = [1 1];
saveVarsTmp{1}(11, 1).DataType = 'boolean';
saveVarsTmp{1}(11, 1).Min = [];
saveVarsTmp{1}(11, 1).Max = [];
saveVarsTmp{1}(11, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(11, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(11, 1).DocUnits = '';
saveVarsTmp{1}(11, 1).Description = '';
saveVarsTmp{1}(12, 1) = Simulink.BusElement;
saveVarsTmp{1}(12, 1).Name = 'IsCoasted';
saveVarsTmp{1}(12, 1).Complexity = 'real';
saveVarsTmp{1}(12, 1).Dimensions = [1 1];
saveVarsTmp{1}(12, 1).DataType = 'boolean';
saveVarsTmp{1}(12, 1).Min = [];
saveVarsTmp{1}(12, 1).Max = [];
saveVarsTmp{1}(12, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(12, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(12, 1).DocUnits = '';
saveVarsTmp{1}(12, 1).Description = '';
saveVarsTmp{1}(13, 1) = Simulink.BusElement;
saveVarsTmp{1}(13, 1).Name = 'IsSelfReported';
saveVarsTmp{1}(13, 1).Complexity = 'real';
saveVarsTmp{1}(13, 1).Dimensions = [1 1];
saveVarsTmp{1}(13, 1).DataType = 'boolean';
saveVarsTmp{1}(13, 1).Min = [];
saveVarsTmp{1}(13, 1).Max = [];
saveVarsTmp{1}(13, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(13, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(13, 1).DocUnits = '';
saveVarsTmp{1}(13, 1).Description = '';
saveVarsTmp{1}(14, 1) = Simulink.BusElement;
saveVarsTmp{1}(14, 1).Name = 'ObjectAttributes';
saveVarsTmp{1}(14, 1).Complexity = 'real';
saveVarsTmp{1}(14, 1).Dimensions = [2 1];
saveVarsTmp{1}(14, 1).DataType = ['Bus: BusRadarDetectionsObjectAttribu' ...
                                  'tes'];
saveVarsTmp{1}(14, 1).Min = [];
saveVarsTmp{1}(14, 1).Max = [];
saveVarsTmp{1}(14, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(14, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(14, 1).DocUnits = '';
saveVarsTmp{1}(14, 1).Description = '';
BusMultiObjectTracker1Tracks.Elements = saveVarsTmp{1};
clear saveVarsTmp;
assignin('base','BusMultiObjectTracker1Tracks', BusMultiObjectTracker1Tracks);

% Bus object: BusVision 
BusVision = Simulink.Bus;
BusVision.Description = '';
BusVision.DataScope = 'Auto';
BusVision.HeaderFile = '';
BusVision.Alignment = -1;
BusVision.PreserveElementDimensions = false;
saveVarsTmp{1} = Simulink.BusElement;
saveVarsTmp{1}.Name = 'NumDetections';
saveVarsTmp{1}.Complexity = 'real';
saveVarsTmp{1}.Dimensions = [1 1];
saveVarsTmp{1}.DataType = 'double';
saveVarsTmp{1}.Min = [];
saveVarsTmp{1}.Max = [];
saveVarsTmp{1}.DimensionsMode = 'Fixed';
saveVarsTmp{1}.SamplingMode = 'Sample based';
saveVarsTmp{1}.DocUnits = '';
saveVarsTmp{1}.Description = '';
saveVarsTmp{1}(2, 1) = Simulink.BusElement;
saveVarsTmp{1}(2, 1).Name = 'IsValidTime';
saveVarsTmp{1}(2, 1).Complexity = 'real';
saveVarsTmp{1}(2, 1).Dimensions = [1 1];
saveVarsTmp{1}(2, 1).DataType = 'boolean';
saveVarsTmp{1}(2, 1).Min = [];
saveVarsTmp{1}(2, 1).Max = [];
saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(2, 1).DocUnits = '';
saveVarsTmp{1}(2, 1).Description = '';
saveVarsTmp{1}(3, 1) = Simulink.BusElement;
saveVarsTmp{1}(3, 1).Name = 'Detections';
saveVarsTmp{1}(3, 1).Complexity = 'real';
saveVarsTmp{1}(3, 1).Dimensions = [20 1];
saveVarsTmp{1}(3, 1).DataType = 'Bus: BusVisionDetections';
saveVarsTmp{1}(3, 1).Min = [];
saveVarsTmp{1}(3, 1).Max = [];
saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(3, 1).DocUnits = '';
saveVarsTmp{1}(3, 1).Description = '';
BusVision.Elements = saveVarsTmp{1};
clear saveVarsTmp;
assignin('base','BusVision', BusVision);

% Bus object: BusVisionDetections 
BusVisionDetections = Simulink.Bus;
BusVisionDetections.Description = '';
BusVisionDetections.DataScope = 'Auto';
BusVisionDetections.HeaderFile = '';
BusVisionDetections.Alignment = -1;
BusVisionDetections.PreserveElementDimensions = false;
saveVarsTmp{1} = Simulink.BusElement;
saveVarsTmp{1}.Name = 'Time';
saveVarsTmp{1}.Complexity = 'real';
saveVarsTmp{1}.Dimensions = [1 1];
saveVarsTmp{1}.DataType = 'double';
saveVarsTmp{1}.Min = [];
saveVarsTmp{1}.Max = [];
saveVarsTmp{1}.DimensionsMode = 'Fixed';
saveVarsTmp{1}.SamplingMode = 'Sample based';
saveVarsTmp{1}.DocUnits = '';
saveVarsTmp{1}.Description = '';
saveVarsTmp{1}(2, 1) = Simulink.BusElement;
saveVarsTmp{1}(2, 1).Name = 'Measurement';
saveVarsTmp{1}(2, 1).Complexity = 'real';
saveVarsTmp{1}(2, 1).Dimensions = [6 1];
saveVarsTmp{1}(2, 1).DataType = 'double';
saveVarsTmp{1}(2, 1).Min = [];
saveVarsTmp{1}(2, 1).Max = [];
saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(2, 1).DocUnits = '';
saveVarsTmp{1}(2, 1).Description = '';
saveVarsTmp{1}(3, 1) = Simulink.BusElement;
saveVarsTmp{1}(3, 1).Name = 'MeasurementNoise';
saveVarsTmp{1}(3, 1).Complexity = 'real';
saveVarsTmp{1}(3, 1).Dimensions = [6 6];
saveVarsTmp{1}(3, 1).DataType = 'double';
saveVarsTmp{1}(3, 1).Min = [];
saveVarsTmp{1}(3, 1).Max = [];
saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(3, 1).DocUnits = '';
saveVarsTmp{1}(3, 1).Description = '';
saveVarsTmp{1}(4, 1) = Simulink.BusElement;
saveVarsTmp{1}(4, 1).Name = 'SensorIndex';
saveVarsTmp{1}(4, 1).Complexity = 'real';
saveVarsTmp{1}(4, 1).Dimensions = [1 1];
saveVarsTmp{1}(4, 1).DataType = 'double';
saveVarsTmp{1}(4, 1).Min = [];
saveVarsTmp{1}(4, 1).Max = [];
saveVarsTmp{1}(4, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(4, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(4, 1).DocUnits = '';
saveVarsTmp{1}(4, 1).Description = '';
saveVarsTmp{1}(5, 1) = Simulink.BusElement;
saveVarsTmp{1}(5, 1).Name = 'ObjectClassID';
saveVarsTmp{1}(5, 1).Complexity = 'real';
saveVarsTmp{1}(5, 1).Dimensions = [1 1];
saveVarsTmp{1}(5, 1).DataType = 'double';
saveVarsTmp{1}(5, 1).Min = [];
saveVarsTmp{1}(5, 1).Max = [];
saveVarsTmp{1}(5, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(5, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(5, 1).DocUnits = '';
saveVarsTmp{1}(5, 1).Description = '';
saveVarsTmp{1}(6, 1) = Simulink.BusElement;
saveVarsTmp{1}(6, 1).Name = 'MeasurementParameters';
saveVarsTmp{1}(6, 1).Complexity = 'real';
saveVarsTmp{1}(6, 1).Dimensions = [1 1];
saveVarsTmp{1}(6, 1).DataType = ['Bus: BusVisionDetectionsMeasurementPa' ...
                                 'rameters'];
saveVarsTmp{1}(6, 1).Min = [];
saveVarsTmp{1}(6, 1).Max = [];
saveVarsTmp{1}(6, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(6, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(6, 1).DocUnits = '';
saveVarsTmp{1}(6, 1).Description = '';
saveVarsTmp{1}(7, 1) = Simulink.BusElement;
saveVarsTmp{1}(7, 1).Name = 'ObjectAttributes';
saveVarsTmp{1}(7, 1).Complexity = 'real';
saveVarsTmp{1}(7, 1).Dimensions = [1 1];
saveVarsTmp{1}(7, 1).DataType = ['Bus: BusVisionDetectionsObjectAttribu' ...
                                 'tes'];
saveVarsTmp{1}(7, 1).Min = [];
saveVarsTmp{1}(7, 1).Max = [];
saveVarsTmp{1}(7, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(7, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(7, 1).DocUnits = '';
saveVarsTmp{1}(7, 1).Description = '';
BusVisionDetections.Elements = saveVarsTmp{1};
clear saveVarsTmp;
assignin('base','BusVisionDetections', BusVisionDetections);

% Bus object: BusVisionDetectionsMeasurementParameters 
BusVisionDetectionsMeasurementParameters = Simulink.Bus;
BusVisionDetectionsMeasurementParameters.Description = '';
BusVisionDetectionsMeasurementParameters.DataScope = 'Auto';
BusVisionDetectionsMeasurementParameters.HeaderFile = '';
BusVisionDetectionsMeasurementParameters.Alignment = -1;
BusVisionDetectionsMeasurementParameters.PreserveElementDimensions = false;
saveVarsTmp{1} = Simulink.BusElement;
saveVarsTmp{1}.Name = 'Frame';
saveVarsTmp{1}.Complexity = 'real';
saveVarsTmp{1}.Dimensions = [1 1];
saveVarsTmp{1}.DataType = 'Enum: drivingCoordinateFrameType';
saveVarsTmp{1}.Min = [];
saveVarsTmp{1}.Max = [];
saveVarsTmp{1}.DimensionsMode = 'Fixed';
saveVarsTmp{1}.SamplingMode = 'Sample based';
saveVarsTmp{1}.DocUnits = '';
saveVarsTmp{1}.Description = '';
saveVarsTmp{1}(2, 1) = Simulink.BusElement;
saveVarsTmp{1}(2, 1).Name = 'OriginPosition';
saveVarsTmp{1}(2, 1).Complexity = 'real';
saveVarsTmp{1}(2, 1).Dimensions = [3 1];
saveVarsTmp{1}(2, 1).DataType = 'double';
saveVarsTmp{1}(2, 1).Min = [];
saveVarsTmp{1}(2, 1).Max = [];
saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(2, 1).DocUnits = '';
saveVarsTmp{1}(2, 1).Description = '';
saveVarsTmp{1}(3, 1) = Simulink.BusElement;
saveVarsTmp{1}(3, 1).Name = 'Orientation';
saveVarsTmp{1}(3, 1).Complexity = 'real';
saveVarsTmp{1}(3, 1).Dimensions = [3 3];
saveVarsTmp{1}(3, 1).DataType = 'double';
saveVarsTmp{1}(3, 1).Min = [];
saveVarsTmp{1}(3, 1).Max = [];
saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(3, 1).DocUnits = '';
saveVarsTmp{1}(3, 1).Description = '';
saveVarsTmp{1}(4, 1) = Simulink.BusElement;
saveVarsTmp{1}(4, 1).Name = 'HasVelocity';
saveVarsTmp{1}(4, 1).Complexity = 'real';
saveVarsTmp{1}(4, 1).Dimensions = [1 1];
saveVarsTmp{1}(4, 1).DataType = 'boolean';
saveVarsTmp{1}(4, 1).Min = [];
saveVarsTmp{1}(4, 1).Max = [];
saveVarsTmp{1}(4, 1).DimensionsMode = 'Fixed';
saveVarsTmp{1}(4, 1).SamplingMode = 'Sample based';
saveVarsTmp{1}(4, 1).DocUnits = '';
saveVarsTmp{1}(4, 1).Description = '';
BusVisionDetectionsMeasurementParameters.Elements = saveVarsTmp{1};
clear saveVarsTmp;
assignin('base','BusVisionDetectionsMeasurementParameters', BusVisionDetectionsMeasurementParameters);

% Bus object: BusVisionDetectionsObjectAttributes 
BusVisionDetectionsObjectAttributes = Simulink.Bus;
BusVisionDetectionsObjectAttributes.Description = '';
BusVisionDetectionsObjectAttributes.DataScope = 'Auto';
BusVisionDetectionsObjectAttributes.HeaderFile = '';
BusVisionDetectionsObjectAttributes.Alignment = -1;
BusVisionDetectionsObjectAttributes.PreserveElementDimensions = false;
saveVarsTmp{1} = Simulink.BusElement;
saveVarsTmp{1}.Name = 'TargetIndex';
saveVarsTmp{1}.Complexity = 'real';
saveVarsTmp{1}.Dimensions = [1 1];
saveVarsTmp{1}.DataType = 'double';
saveVarsTmp{1}.Min = [];
saveVarsTmp{1}.Max = [];
saveVarsTmp{1}.DimensionsMode = 'Fixed';
saveVarsTmp{1}.SamplingMode = 'Sample based';
saveVarsTmp{1}.DocUnits = '';
saveVarsTmp{1}.Description = '';
BusVisionDetectionsObjectAttributes.Elements = saveVarsTmp{1};
clear saveVarsTmp;
assignin('base','BusVisionDetectionsObjectAttributes', BusVisionDetectionsObjectAttributes);
end

