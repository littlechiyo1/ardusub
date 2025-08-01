
"use strict";

let DebugValue = require('./DebugValue.js');
let Mavlink = require('./Mavlink.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let MagnetometerReporter = require('./MagnetometerReporter.js');
let ManualControl = require('./ManualControl.js');
let CameraImageCaptured = require('./CameraImageCaptured.js');
let ExcavatorInclination = require('./ExcavatorInclination.js');
let ParamValue = require('./ParamValue.js');
let StatusText = require('./StatusText.js');
let RTKBaseline = require('./RTKBaseline.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let LandingTarget = require('./LandingTarget.js');
let Waypoint = require('./Waypoint.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let ActuatorControl = require('./ActuatorControl.js');
let FileEntry = require('./FileEntry.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let MountControl = require('./MountControl.js');
let WaypointReached = require('./WaypointReached.js');
let Thrust = require('./Thrust.js');
let Tunnel = require('./Tunnel.js');
let LogEntry = require('./LogEntry.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let RCOut = require('./RCOut.js');
let Trajectory = require('./Trajectory.js');
let HilGPS = require('./HilGPS.js');
let RCIn = require('./RCIn.js');
let LogData = require('./LogData.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let GPSINPUT = require('./GPSINPUT.js');
let HilControls = require('./HilControls.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let WaypointList = require('./WaypointList.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let VFR_HUD = require('./VFR_HUD.js');
let Param = require('./Param.js');
let VehicleInfo = require('./VehicleInfo.js');
let PositionTarget = require('./PositionTarget.js');
let CellularStatus = require('./CellularStatus.js');
let CommandCode = require('./CommandCode.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let ExtendedState = require('./ExtendedState.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let HomePosition = require('./HomePosition.js');
let Altitude = require('./Altitude.js');
let Vibration = require('./Vibration.js');
let TerrainReport = require('./TerrainReport.js');
let ESCTelemetry = require('./ESCTelemetry.js');
let RadioStatus = require('./RadioStatus.js');
let GPSRTK = require('./GPSRTK.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let GPSRAW = require('./GPSRAW.js');
let ESCStatus = require('./ESCStatus.js');
let RTCM = require('./RTCM.js');
let HilSensor = require('./HilSensor.js');
let ESCTelemetryItem = require('./ESCTelemetryItem.js');
let ESCInfo = require('./ESCInfo.js');
let State = require('./State.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let BatteryStatus = require('./BatteryStatus.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let NavControllerOutput = require('./NavControllerOutput.js');
let EstimatorStatus = require('./EstimatorStatus.js');

module.exports = {
  DebugValue: DebugValue,
  Mavlink: Mavlink,
  OpticalFlowRad: OpticalFlowRad,
  MagnetometerReporter: MagnetometerReporter,
  ManualControl: ManualControl,
  CameraImageCaptured: CameraImageCaptured,
  ExcavatorInclination: ExcavatorInclination,
  ParamValue: ParamValue,
  StatusText: StatusText,
  RTKBaseline: RTKBaseline,
  ADSBVehicle: ADSBVehicle,
  LandingTarget: LandingTarget,
  Waypoint: Waypoint,
  PlayTuneV2: PlayTuneV2,
  ActuatorControl: ActuatorControl,
  FileEntry: FileEntry,
  HilActuatorControls: HilActuatorControls,
  MountControl: MountControl,
  WaypointReached: WaypointReached,
  Thrust: Thrust,
  Tunnel: Tunnel,
  LogEntry: LogEntry,
  CamIMUStamp: CamIMUStamp,
  RCOut: RCOut,
  Trajectory: Trajectory,
  HilGPS: HilGPS,
  RCIn: RCIn,
  LogData: LogData,
  OnboardComputerStatus: OnboardComputerStatus,
  GPSINPUT: GPSINPUT,
  HilControls: HilControls,
  ESCStatusItem: ESCStatusItem,
  WaypointList: WaypointList,
  ESCInfoItem: ESCInfoItem,
  VFR_HUD: VFR_HUD,
  Param: Param,
  VehicleInfo: VehicleInfo,
  PositionTarget: PositionTarget,
  CellularStatus: CellularStatus,
  CommandCode: CommandCode,
  OverrideRCIn: OverrideRCIn,
  GlobalPositionTarget: GlobalPositionTarget,
  ExtendedState: ExtendedState,
  AttitudeTarget: AttitudeTarget,
  HomePosition: HomePosition,
  Altitude: Altitude,
  Vibration: Vibration,
  TerrainReport: TerrainReport,
  ESCTelemetry: ESCTelemetry,
  RadioStatus: RadioStatus,
  GPSRTK: GPSRTK,
  TimesyncStatus: TimesyncStatus,
  GPSRAW: GPSRAW,
  ESCStatus: ESCStatus,
  RTCM: RTCM,
  HilSensor: HilSensor,
  ESCTelemetryItem: ESCTelemetryItem,
  ESCInfo: ESCInfo,
  State: State,
  WheelOdomStamped: WheelOdomStamped,
  HilStateQuaternion: HilStateQuaternion,
  BatteryStatus: BatteryStatus,
  CompanionProcessStatus: CompanionProcessStatus,
  NavControllerOutput: NavControllerOutput,
  EstimatorStatus: EstimatorStatus,
};
