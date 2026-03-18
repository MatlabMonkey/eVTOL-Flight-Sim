function cfg = fg_config()
%FG_CONFIG Default FlightGear integration settings for this project.

installInfo = fg_autodetect_installation();

cfg = struct();
cfg.projectRoot = string(fileparts(fileparts(mfilename("fullpath"))));
cfg.modelName = "Brown_Full_Sim";
cfg.modelFile = fullfile(cfg.projectRoot, "Brown_Full_Sim.slx");
cfg.modelCopyName = "Brown_Full_Sim_FlightGear";
cfg.modelCopyFile = fullfile(cfg.projectRoot, "Brown_Full_Sim_FlightGear.slx");

cfg.positionLLAVariable = "positionLLA";
cfg.eulerVariable = "eul";

cfg.destinationIpAddress = "127.0.0.1";
cfg.destinationPort = 5502;
cfg.framesPerSecond = 30;
cfg.timeScaling = 1;

cfg.flightGearBaseDirectory = "/Users/zbrown/Library/Application Support/FlightGear/fgdata_2024_1/Aircraft";
cfg.fgfsExecutable = installInfo.fgfsExecutable;
cfg.geometryModelName = "c172p";
cfg.airportId = "KBOS";
cfg.runwayId = "04R";
cfg.initialAltitudeFt = 415;
cfg.initialHeadingDeg = 270;
cfg.offsetDistanceMiles = 0;
cfg.offsetAzimuthDeg = 0;
cfg.disableShaders = true;
cfg.installScenery = false;
cfg.outputFileName = "runfg_mac.sh";
cfg.architecture = localArchitecture();
end

function arch = localArchitecture()
[status, unameMachine] = system("uname -m");
if status == 0 && contains(string(strtrim(unameMachine)), "arm")
    arch = "MacARM";
else
    arch = "Mac";
end
end
