function fg_validate_config(cfg)
%FG_VALIDATE_CONFIG Validate required FlightGear configuration fields.

arguments
    cfg (1,1) struct
end

if ~isfield(cfg, "modelFile") || ~isfile(cfg.modelFile)
    error("fg_validate_config:MissingModel", ...
        "Model file not found: %s", string(cfg.modelFile));
end

if strlength(string(cfg.flightGearBaseDirectory)) == 0
    error("fg_validate_config:MissingFlightGear", ...
        ["FlightGear was not found automatically. Set cfg.flightGearBaseDirectory " ...
         "in Brown_HW6/fg_config.m before running replay or generating a launch script."]);
end

if ~isfolder(cfg.flightGearBaseDirectory) && ~isfile(cfg.flightGearBaseDirectory)
    error("fg_validate_config:BadFlightGearPath", ...
        "Configured FlightGear path does not exist: %s", string(cfg.flightGearBaseDirectory));
end
end
