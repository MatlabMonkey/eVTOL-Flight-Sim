function info = fg_autodetect_installation()
%FG_AUTODETECT_INSTALLATION Locate a local FlightGear install on macOS.

info = struct( ...
    'flightGearBaseDirectory', "", ...
    'fgfsExecutable', "");

envBase = getenv("FLIGHTGEAR_BASE_DIR");
envExec = getenv("FGFS_EXECUTABLE");
envRoot = getenv("FG_ROOT");

if ~isempty(envBase)
    info.flightGearBaseDirectory = string(envBase);
end

if ~isempty(envExec)
    info.fgfsExecutable = string(envExec);
end

if strlength(info.flightGearBaseDirectory) == 0 && ~isempty(envRoot)
    info.flightGearBaseDirectory = string(envRoot);
end

if strlength(info.flightGearBaseDirectory) == 0
    appCandidates = [ ...
        dir("/Applications/FlightGear*.app"); ...
        dir("/Applications/*FlightGear*.app")];

    if ~isempty(appCandidates)
        appCandidates = appCandidates([appCandidates.isdir]);
        info.flightGearBaseDirectory = string(fullfile(appCandidates(1).folder, appCandidates(1).name));
    end
end

if strlength(info.fgfsExecutable) == 0 && strlength(info.flightGearBaseDirectory) > 0
    execCandidates = [ ...
        fullfile(info.flightGearBaseDirectory, "Contents", "MacOS", "fgfs"); ...
        fullfile(info.flightGearBaseDirectory, "bin", "fgfs")];

    for idx = 1:numel(execCandidates)
        if isfile(execCandidates(idx))
            info.fgfsExecutable = execCandidates(idx);
            break;
        end
    end
end
end
