% Prep_Demo_Transition_Corridor_HighBridgeRoute_Smooth120s_Tune_RpmBiasUltra.m
% 120 s high-bridge corridor demo that skips the unreachable 25/70 -> 30/75
% guide region while keeping ultra prop bias and stricter gated settling.

transitionCorridorPrepOptions = build_transition_corridor_test_preset('highbridge');
setappdata(0, 'TransitionCorridorPrepOptions', transitionCorridorPrepOptions);
Prep_Wrapper_TransitionCorridor_BlueGuide_ScheduledLQR
