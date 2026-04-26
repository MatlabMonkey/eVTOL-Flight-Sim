% Prep_Demo_Transition_Corridor_MidRoute_Smooth120s_Tune_RpmBiasUltra.m
% 120 s mid-route corridor demo with ultra prop bias and stricter gated settling.

transitionCorridorPrepOptions = build_transition_corridor_test_preset('mid');
setappdata(0, 'TransitionCorridorPrepOptions', transitionCorridorPrepOptions);
Prep_Wrapper_TransitionCorridor_BlueGuide_ScheduledLQR
