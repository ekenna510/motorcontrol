import odrive
from odrive.enums import *
odrv0=odrive.find_any()

# 5/15/2023
#------------------------------------------------------------------------------------
odrv0.axis0.controller.config.pos_gain = 3
odrv0.axis0.controller.config.vel_gain = 1.3
odrv0.axis0.controller.config.vel_integrator_gain = 15
odrv0.axis0.controller.config.vel_limit = 10
odrv0.axis0.encoder.config.mode=ENCODER_MODE_INCREMENTAL
odrv0.axis0.encoder.config.cpr=3200
odrv0.axis0.motor.config.pole_pairs = 15
odrv0.axis0.motor.config.resistance_calib_max_voltage = 10
odrv0.axis0.motor.config.current_control_bandwidth    = 100
odrv0.axis0.motor.config.torque_constant=0.8270000219345093
odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis0.motor.config.calibration_current=10
odrv0.axis0.motor.config.current_lim =10
odrv0.axis0.motor.config.current_lim_margin=2
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis0.controller.config.vel_limit = 5
odrv0.axis0.controller.config.vel_ramp_rate=.25
odrv0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
odrv0.axis0.motor.config.direction=1 

odrv0.axis1.controller.config.pos_gain = 3
odrv0.axis1.controller.config.vel_gain = 1.3
odrv0.axis1.controller.config.vel_integrator_gain = 15
odrv0.axis1.controller.config.vel_limit = 10
odrv0.axis1.encoder.config.mode=ENCODER_MODE_INCREMENTAL
odrv0.axis1.encoder.config.cpr=3200
odrv0.axis1.motor.config.pole_pairs = 15
odrv0.axis1.motor.config.resistance_calib_max_voltage = 10
odrv0.axis1.motor.config.current_control_bandwidth    = 100
odrv0.axis1.motor.config.torque_constant=0.8270000219345093
odrv0.axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis1.motor.config.calibration_current=10
odrv0.axis1.motor.config.current_lim =10
odrv0.axis1.motor.config.current_lim_margin=2
odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis1.controller.config.vel_limit = 5
odrv0.axis1.controller.config.vel_ramp_rate=.25
odrv0.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP
odrv0.axis1.motor.config.direction=-1 


odrv0.axis0.requested_state=AxisState.FULL_CALIBRATION_SEQUENCE
odrv0.axis1.requested_state=AxisState.FULL_CALIBRATION_SEQUENCE


odrv0.axis1.controller.input_vel=.01
odrv0.axis0.controller.input_vel=-.01

odrv0.axis0.requested_state=AxisState.CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state=AxisState.CLOSED_LOOP_CONTROL




#------------------------------------------------------------------------------------
odrv0.axis1.controller.config.pos_gain = 3
odrv0.axis1.controller.config.vel_gain = 1.3
odrv0.axis1.controller.config.vel_integrator_gain = 15
odrv0.axis1.controller.config.vel_limit = 10


odrv0.axis1.encoder.config.mode=ENCODER_MODE_INCREMENTAL
odrv0.axis1.encoder.config.cpr=3200



odrv0.axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis1.motor.config.pole_pairs = 15
odrv0.axis1.motor.config.resistance_calib_max_voltage = 10
odrv0.axis1.motor.config.current_control_bandwidth    = 100
odrv0.axis1.motor.config.torque_constant=0.8270000219345093

#4/23/2023 
# axiz0 changes
odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis0.motor.config.calibration_current=10
odrv0.axis0.motor.config.current_lim =10
odrv0.axis0.motor.config.current_lim_margin=2
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis0.controller.config.vel_limit = 5
odrv0.axis0.controller.config.vel_ramp_rate=.25
# axiz1 changes 
odrv0.axis1.motor.config.current_lim_margin=2
odrv0.axis1.motor.config.direction=-1
odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis1.controller.config.vel_limit = 5
odrv0.axis1.controller.config.vel_ramp_rate=.25


odrv0.save_configuration()   
odrv0.reboot
#odrv0.axis0.motor.config.torque_constant=0.8270000219345093
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis1.motor.config.pre_calibrated = True
odrv0.axis0.encoder.config.pre_calibrated=True
odrv0.axis0.encoder.config.pre_calibrated=True
odrv0.save_configuration()   
odrv0.reboot




#odrv0.axis1.motor.config.requested_current_range      = 25


odrv0.axis0.requested_state=AxisState.FULL_CALIBRATION_SEQUENCE
odrv0.axis1.motor.config.current_lim=3.0

odrv0.axis1.motor.config.current_lim =4
odrv0.axis1.motor.config.current_lim_margin=1

odrv0.axis0.requested_state=AxisState.FULL_CALIBRATION_SEQUENCE
odrv0.axis1.requested_state=AxisState.FULL_CALIBRATION_SEQUENCE


odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis0.controller.config.vel_ramp_rate=.25
odrv0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
odrv0.axis0.controller.input_vel=.1


odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis1.controller.config.vel_ramp_rate=.25
odrv0.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP
odrv0.axis1.controller.input_vel=.1

odrv0.axis0.motor.config.direction=1 
odrv0.axis1.motor.config.direction=1 

odrv0.axis0.requested_state=AxisState.CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state=AxisState.CLOSED_LOOP_CONTROL

odrv0.axis0.requested_state=AxisState.IDLE

#   odrv0.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
  odrv0.axis0.encoder.error
  odrv0.axis1.requested_state = AXIS_STATE_IDLE




#--------------------------------------- from video
odrv0.axis0.motor.config.requested_current_range #maybe this should be 25?
odrv0.axis1.motor.config.resistance_calib_max_voltage #maybe this should be 4?
odrv0.axis1.motor.config.torque_constant=0.8270000219345093 #maybe this should v 8.27
odrv0.axis1.encoder.config.calib_scan_distance #maybe 150?
odrv0.axis1.encoder.config.bandwidth #maybe reduce to smooth out is this causing the jerkiness.
odrv0.axis1.controller.config.pos_gain #maybe 1
odrv0.axis1.controller.config.vel_gain = #mayvbe 15
odrv0.axis1.controller.config.vel_integrator_gain  #maybe 74
odrv0.axis1.motor.config.phase_inductance # maybe .0005552865332
odrv0.axis1.motor.config.phase_resistance #maybe .25844332575798
odrv0.axis1.encoder.config.offset_float
#---------------------------------------


calibration_lockin:
  accel: 20.0 (float)
  current: 10.0 (float)
  ramp_distance: 3.1415927410125732 (float)
  ramp_time: 0.4000000059604645 (float)
  vel: 40.0 (float)
can_heartbeat_rate_ms: 100 (uint32)
can_node_id: 0 (uint32)
can_node_id_extended: False (bool)
dir_gpio_pin: 2 (uint16)
enable_step_dir: False (bool)
enable_watchdog: False (bool)
general_lockin:
  accel: 20.0 (float)
  current: 10.0 (float)
  finish_distance: 100.0 (float)
  finish_on_distance: False (bool)
  finish_on_enc_idx: False (bool)
  finish_on_vel: False (bool)
  ramp_distance: 3.1415927410125732 (float)
  ramp_time: 0.4000000059604645 (float)
  vel: 40.0 (float)
sensorless_ramp:
  accel: 200.0 (float)
  current: 10.0 (float)
  finish_distance: 100.0 (float)
  finish_on_distance: False (bool)
  finish_on_enc_idx: False (bool)
  finish_on_vel: True (bool)
  ramp_distance: 3.1415927410125732 (float)
  ramp_time: 0.4000000059604645 (float)
  vel: 400.0 (float)
startup_closed_loop_control: False (bool)
startup_encoder_index_search: False (bool)
startup_encoder_offset_calibration: False (bool)
startup_homing: False (bool)
startup_motor_calibration: False (bool)
startup_sensorless_control: False (bool)
step_dir_always_on: False (bool)
step_gpio_pin: 1 (uint16)
turns_per_step: 0.0009765625 (float)
watchdog_timeout: 0.0 (float)




calibration_lockin:
  accel: 20.0 (float)
  current: 10.0 (float)
  ramp_distance: 3.1415927410125732 (float)
  ramp_time: 0.4000000059604645 (float)
  vel: 40.0 (float)
can_heartbeat_rate_ms: 100 (uint32)
can_node_id: 1 (uint32)
can_node_id_extended: False (bool)
dir_gpio_pin: 8 (uint16)
enable_step_dir: False (bool)
enable_watchdog: False (bool)
general_lockin:
  accel: 20.0 (float)
  current: 10.0 (float)
  finish_distance: 100.0 (float)
  finish_on_distance: False (bool)
  finish_on_enc_idx: False (bool)
  finish_on_vel: False (bool)
  ramp_distance: 3.1415927410125732 (float)
  ramp_time: 0.4000000059604645 (float)
  vel: 40.0 (float)
sensorless_ramp:
  accel: 200.0 (float)
  current: 10.0 (float)
  finish_distance: 100.0 (float)
  finish_on_distance: False (bool)
  finish_on_enc_idx: False (bool)
  finish_on_vel: True (bool)
  ramp_distance: 3.1415927410125732 (float)
  ramp_time: 0.4000000059604645 (float)
  vel: 400.0 (float)
startup_closed_loop_control: False (bool)
startup_encoder_index_search: False (bool)
startup_encoder_offset_calibration: False (bool)
startup_homing: False (bool)
startup_motor_calibration: False (bool)
startup_sensorless_control: False (bool)
step_dir_always_on: False (bool)
step_gpio_pin: 7 (uint16)
turns_per_step: 0.0009765625 (float)
watchdog_timeout: 0.0 (float)



motor 

DC_calib_phB: -1.264419674873352 (float)
DC_calib_phC: -1.3882945775985718 (float)
armed_state: 0 (int32)
config:
  acim_autoflux_attack_gain: 10.0 (float)
  acim_autoflux_decay_gain: 1.0 (float)
  acim_autoflux_enable: False (bool)
  acim_autoflux_min_Id: 10.0 (float)
  acim_gain_min_flux: 10.0 (float)
  acim_slip_velocity: 14.706000328063965 (float)
  calibration_current: 4.0 (float)
  current_control_bandwidth: 100.0 (float)
  current_lim: 3.0 (float)
  current_lim_margin: 2.0 (float)
  direction: 1  
  inverter_temp_limit_lower: 100.0 (float)
  inverter_temp_limit_upper: 120.0 (float)
  motor_type: 0 (int32)
  phase_inductance: 0.0010290039936080575 (float)
  phase_resistance: 0.7484160661697388 (float)
  pole_pairs: 15 (int32)
  pre_calibrated: False (bool)
  requested_current_range: 60.0 (float)
  resistance_calib_max_voltage: 10.0 (float)
  torque_constant: 0.8270000219345093 (float)
  torque_lim: inf (float)
current_control:
  I_measured_report_filter_k: 1.0 (float)
  Ibus: 0.0 (float)
  Id_measured: 0.7569441795349121 (float)
  Id_setpoint: 0.0 (float)
  Iq_measured: 5.010852336883545 (float)
  Iq_setpoint: -0.21274857223033905 (float)
  acim_rotor_flux: 0.0 (float)
  async_phase_offset: 0.0 (float)
  async_phase_vel: 0.0 (float)
  final_v_alpha: -2.030479907989502 (float)
  final_v_beta: -1.82060968875885 (float)
  i_gain: 74.84160614013672 (float)
  max_allowed_current: 60.75 (float)
  overcurrent_trip_level: 67.5 (float)
  p_gain: 0.10290040075778961 (float)
  v_current_control_integral_d: 0.0 (float)
  v_current_control_integral_q: 0.0 (float)
current_meas_phB: 0.13068091869354248 (float)
current_meas_phC: 0.02102077007293701 (float)
effective_current_lim: 3.0 (float)
error: 0 (int32)
gate_driver:
  drv_fault: 0 (int32)
is_calibrated: True (bool)
phase_current_rev_gain: 0.02500000037252903 (float)
timing_log:
  adc_cb_dc: 12842 (uint16)
  adc_cb_i: 2714 (uint16)
  enc_calib: 8050 (uint16)
  foc_current: 9294 (uint16)
  foc_voltage: 7994 (uint16)
  general: 60321 (uint16)
  idx_search: 20033 (uint16)
  meas_l: 7590 (uint16)
  meas_r: 7590 (uint16)
  sample_now: 39092 (uint16)
  spi_end: 45691 (uint16)
  spi_start: 7068 (uint16)
