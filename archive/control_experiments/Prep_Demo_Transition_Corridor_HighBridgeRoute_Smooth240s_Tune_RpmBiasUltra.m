% Prep_Demo_Transition_Corridor_HighBridgeRoute_Smooth240s_Tune_RpmBiasUltra.m
% 240 s high-bridge corridor demo using the same ultra prop-bias and
% stricter gated settling as the 120 s preset, but with a slower overall
% command progression and longer run time.

transitionCorridorPrepOptions = build_transition_corridor_test_preset('highbridge');
transitionCorridorPrepOptions.run_stop_time_s = 240.0;
transitionCorridorPrepOptions.schedule_opts.stop_time_s = 240.0;

setappdata(0, 'TransitionCorridorPrepOptions', transitionCorridorPrepOptions);
Prep_Wrapper_TransitionCorridor_BlueGuide_ScheduledLQR
