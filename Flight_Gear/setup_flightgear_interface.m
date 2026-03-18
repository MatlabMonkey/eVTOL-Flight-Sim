function modelCopyName = setup_flightgear_interface(cfg)
%SETUP_FLIGHTGEAR_INTERFACE Add a live FlightGear subsystem to a model copy.

if nargin < 1 || isempty(cfg)
    cfg = fg_config();
end

if ~isfile(cfg.modelCopyFile)
    copyfile(cfg.modelFile, cfg.modelCopyFile);
end

[~, modelCopyName] = fileparts(cfg.modelCopyFile);
load_system(modelCopyName);

rootSystem = string(modelCopyName);
subsystemPath = rootSystem + "/FlightGear Interface";

if ~isempty(find_system(rootSystem, "SearchDepth", 1, "Name", "FlightGear Interface"))
    delete_block(subsystemPath);
end

add_block("simulink/Ports & Subsystems/Subsystem", subsystemPath, ...
    "Position", [720 455 980 595]);

localDeleteDefaultSubsystemContents(subsystemPath);

add_block("simulink/Sources/In1", subsystemPath + "/positionLLA", ...
    "Position", [40 70 70 84]);
add_block("simulink/Sources/In1", subsystemPath + "/eul", ...
    "Position", [40 170 70 184], ...
    "Port", "2");
add_block("simulink/Signal Routing/Demux", subsystemPath + "/Demux LLA", ...
    "Position", [120 52 125 102], ...
    "Outputs", "3");
add_block("simulink/Signal Routing/Demux", subsystemPath + "/Demux Euler", ...
    "Position", [120 152 125 202], ...
    "Outputs", "3");
add_block("simulink/Signal Routing/Mux", subsystemPath + "/Mux FlightGear", ...
    "Position", [240 72 245 228], ...
    "Inputs", "6");

flightGearBlockPath = localResolveFlightGearBlock();
add_block(flightGearBlockPath, subsystemPath + "/FlightGear 6DoF", ...
    "Position", [320 115 465 185]);

add_line(subsystemPath, "positionLLA/1", "Demux LLA/1", "autorouting", "on");
add_line(subsystemPath, "eul/1", "Demux Euler/1", "autorouting", "on");

add_line(subsystemPath, "Demux LLA/2", "Mux FlightGear/1", "autorouting", "on");
add_line(subsystemPath, "Demux LLA/1", "Mux FlightGear/2", "autorouting", "on");
add_line(subsystemPath, "Demux LLA/3", "Mux FlightGear/3", "autorouting", "on");
add_line(subsystemPath, "Demux Euler/1", "Mux FlightGear/4", "autorouting", "on");
add_line(subsystemPath, "Demux Euler/2", "Mux FlightGear/5", "autorouting", "on");
add_line(subsystemPath, "Demux Euler/3", "Mux FlightGear/6", "autorouting", "on");
add_line(subsystemPath, "Mux FlightGear/1", "FlightGear 6DoF/1", "autorouting", "on");

localConnectRootSignal(rootSystem, "Mux1/1", "FlightGear Interface/1");
localConnectRootSignal(rootSystem, "Reshape/1", "FlightGear Interface/2");

save_system(modelCopyName);
end

function localDeleteDefaultSubsystemContents(subsystemPath)
lineHandles = find_system(subsystemPath, "FindAll", "on", "Type", "line");
for idx = 1:numel(lineHandles)
    delete_line(lineHandles(idx));
end

defaultBlocks = find_system(subsystemPath, "SearchDepth", 1, "Type", "Block");
for idx = 2:numel(defaultBlocks)
    delete_block(defaultBlocks{idx});
end
end

function blockPath = localResolveFlightGearBlock()
load_system("aerolibfltsims");
matches = find_system("aerolibfltsims", ...
    "LookUnderMasks", "all", ...
    "Name", "FlightGear Preconfigured 6DoF Animation");

if ~isempty(matches)
    blockPath = string(matches{1});
    return;
end

error("setup_flightgear_interface:MissingBlock", ...
    "Could not resolve the FlightGear block library path automatically inside aerolibfltsims.");
end

function localConnectRootSignal(rootSystem, sourcePort, destinationPort)
try
    add_line(rootSystem, sourcePort, destinationPort, "autorouting", "on");
catch err
    if ~contains(err.message, "already exists")
        rethrow(err);
    end
end
end
