# Controller Schedule DB

- Created: `2026-04-25 20:56:37 -0700`
- Controller candidates kept: `725`
- Missing linearizations during raw rebuild: `730`
- Rebuilt controller rows before merge: `0`
- Preserved existing inlined controller rows: `725`
- Include quasi trims: `0`
- Allow trim-only candidates: `0`

The controller-facing linear models are inlined in this MAT file. External raw linearization MAT files are not required for normal controller builds.

## Controller Schedule Preview

| key | tilt_deg | vinf_mps | family | classification | rear_collective_rpm |
| --- | ---: | ---: | --- | --- | ---: |
| tilt_0__vinf_0__family_hover_zero_surface__source_transition_trim_map_low_speed_scored__seed_prop_first_pass_tilt_0_v_0 | 0 | 0 | hover_zero_surface | exact_trim | 1756.4 |
| tilt_0__vinf_0__family_named_case__source_transition_trim_map__seed_none | 0 | 0 | named_case | exact_trim | 1756.4 |
| tilt_0__vinf_2p5__family_front_rear_free_flap_elevator__source_transition_trim_map_low_speed_scored__seed_prop_first_pass_tilt_0_v_2p5 | 0 | 2.5 | front_rear_free_flap_elevator | exact_trim | 1755.28 |
| tilt_0__vinf_2p5__family_hover_zero_surface__source_transition_trim_map__seed_none | 0 | 2.5 | hover_zero_surface | exact_trim | 1755.28 |
| tilt_0__vinf_5__family_front_rear_free_flap_elevator__source_transition_trim_map_low_speed_scored__seed_hover_default | 0 | 5 | front_rear_free_flap_elevator | exact_trim | 1751.88 |
| tilt_2p5__vinf_2p5__family_front_rear_free_flap_elevator__source_transition_trim_rearon_connector_forever__seed_parent_history_2 | 2.5 | 2.5 | front_rear_free_flap_elevator | exact_trim | 1751.28 |
| tilt_5__vinf_2p5__family_front_rear_free_flap_elevator__source_transition_trim_map_low_speed_scored__seed_two_pass_tilt_5_v_2p5 | 5 | 2.5 | front_rear_free_flap_elevator | exact_trim | 1748.39 |
| tilt_5__vinf_5__family_front_rear_free_flap_elevator__source_transition_trim_map_low_speed_scored__seed_history_2 | 5 | 5 | front_rear_free_flap_elevator | exact_trim | 1741.7 |
| tilt_5__vinf_7p5__family_front_rear_free_flap_elevator__source_transition_trim_map_low_speed_scored__seed_history_1 | 5 | 7.5 | front_rear_free_flap_elevator | exact_trim | 1733.65 |
| tilt_7p5__vinf_2p5__family_front_rear_free_flap_elevator__source_transition_trim_rearon_connector_forever__seed_parent_history_1 | 7.5 | 2.5 | front_rear_free_flap_elevator | exact_trim | 1744.79 |
| tilt_10__vinf_2p5__family_front_rear_free_flap_elevator__source_transition_trim_map_low_speed_scored__seed_entry_lowspeedscored_tilt5_v2p5 | 10 | 2.5 | front_rear_free_flap_elevator | exact_trim | 1740.46 |
| tilt_10__vinf_5__family_front_rear_free_flap_elevator__source_transition_trim_map_low_speed_scored__seed_entry_lowspeedscored_tilt5_v2p5 | 10 | 5 | front_rear_free_flap_elevator | exact_trim | 1726.15 |
| tilt_10__vinf_7p5__family_front_rear_free_flap_elevator__source_transition_trim_map_low_speed_scored__seed_two_pass_tilt_10_v_7p5 | 10 | 7.5 | front_rear_free_flap_elevator | exact_trim | 1743.03 |
| tilt_10__vinf_10__family_front_rear_free_flap_elevator__source_transition_trim_map_low_speed_scored__seed_prop_first_pass_tilt_10_v_10 | 10 | 10 | front_rear_free_flap_elevator | exact_trim | 1694.42 |
| tilt_10__vinf_12p5__family_front_rear_free_flap_elevator__source_transition_trim_map_low_speed_scored__seed_history_3 | 10 | 12.5 | front_rear_free_flap_elevator | exact_trim | 1685.05 |
| tilt_15__vinf_2p5__family_front_rear_free_flap_elevator__source_transition_trim_map_low_speed_scored__seed_two_pass_tilt_15_v_2p5 | 15 | 2.5 | front_rear_free_flap_elevator | exact_trim | 1731.3 |
| tilt_15__vinf_5__family_front_rear_free_flap_elevator__source_transition_trim_map_low_speed_scored__seed_history_1 | 15 | 5 | front_rear_free_flap_elevator | exact_trim | 1725.02 |
| tilt_15__vinf_7p5__family_front_rear_free_flap_elevator__source_transition_trim_map_low_speed_scored__seed_hover_default | 15 | 7.5 | front_rear_free_flap_elevator | exact_trim | 1716.34 |
| tilt_15__vinf_45__family_low_speed_force_balance_rear_fixed__source_transition_trim_map_low_speed__seed_none | 15 | 45 | low_speed_force_balance_rear_fixed | exact_trim | 950 |
| tilt_20__vinf_2p5__family_front_rear_free_flap_elevator__source_transition_trim_map_low_speed_scored__seed_entry_lowspeedscored_tilt15_v2p5 | 20 | 2.5 | front_rear_free_flap_elevator | exact_trim | 1717.26 |
| tilt_20__vinf_5__family_front_rear_free_flap_elevator__source_transition_trim_lowmid_guidegrid_scored__seed_guide_v5_t20_fm150_r0 | 20 | 5 | front_rear_free_flap_elevator | exact_trim | 1713.53 |
| tilt_20__vinf_7p5__family_front_rear_free_flap_elevator__source_transition_trim_map_low_speed_scored__seed_prop_first_pass_tilt_20_v_7p5 | 20 | 7.5 | front_rear_free_flap_elevator | exact_trim | 1708.97 |
| tilt_20__vinf_10__family_front_rear_free_flap_elevator__source_transition_trim_map_low_speed_scored__seed_history_1 | 20 | 10 | front_rear_free_flap_elevator | exact_trim | 1693.14 |
| tilt_20__vinf_15__family_front_rear_free_flap_elevator__source_transition_trim_map_low_speed_scored__seed_history_2 | 20 | 15 | front_rear_free_flap_elevator | exact_trim | 1678.22 |
| tilt_20__vinf_30__family_front_rear_free_flap_elevator__source_transition_trim_map_bridge_scored__seed_history_35 | 20 | 30 | front_rear_free_flap_elevator | exact_trim | 1231.59 |

