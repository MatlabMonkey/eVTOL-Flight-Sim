% Prep_Demo_Transition_Corridor_HighRoute_Smooth120s_Tune_RpmBiasUltra.m
% 120 s high-route corridor demo with ultra prop bias and stricter gated settling.

transitionCorridorPrepOptions = build_transition_corridor_test_preset('high');
setappdata(0, 'TransitionCorridorPrepOptions', transitionCorridorPrepOptions);
Prep_Wrapper_TransitionCorridor_BlueGuide_ScheduledLQR
