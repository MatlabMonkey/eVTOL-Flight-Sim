function animation = fg_play_array6dof(array6dof, cfg)
%FG_PLAY_ARRAY6DOF Launch FlightGear replay from an Array6DoF matrix.

if nargin < 2 || isempty(cfg)
    cfg = fg_config();
end

fg_validate_config(cfg);

if size(array6dof, 2) < 7
    error("fg_play_array6dof:BadShape", ...
        "Array6DoF input must have at least 7 columns: [time lat lon alt phi theta psi].");
end

animation = fganimation;
animation.TimeSeriesSource = double(array6dof(:, 1:7));
animation.TimeSeriesSourceType = "Array6DoF";
animation.TimeScaling = cfg.timeScaling;
animation.FramesPerSecond = cfg.framesPerSecond;
animation.OutputFileName = cfg.outputFileName;
animation.FlightGearBaseDirectory = cfg.flightGearBaseDirectory;
animation.GeometryModelName = cfg.geometryModelName;
animation.DestinationIpAddress = cfg.destinationIpAddress;
animation.DestinationPort = string(cfg.destinationPort);
animation.AirportId = cfg.airportId;
animation.RunwayId = cfg.runwayId;
animation.InitialAltitude = cfg.initialAltitudeFt;
animation.InitialHeading = cfg.initialHeadingDeg;
animation.OffsetDistance = cfg.offsetDistanceMiles;
animation.OffsetAzimuth = cfg.offsetAzimuthDeg;
animation.InstallScenery = cfg.installScenery;
animation.DisableShaders = cfg.disableShaders;
animation.Architecture = cfg.architecture;

animation.GenerateRunScript();
animation.initialize();
animation.play();
end
