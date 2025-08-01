
"use strict";

let SetMode = require('./SetMode.js')
let FileOpen = require('./FileOpen.js')
let MessageInterval = require('./MessageInterval.js')
let LogRequestData = require('./LogRequestData.js')
let ParamPull = require('./ParamPull.js')
let CommandHome = require('./CommandHome.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let WaypointPull = require('./WaypointPull.js')
let FileTruncate = require('./FileTruncate.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let FileChecksum = require('./FileChecksum.js')
let FileClose = require('./FileClose.js')
let SetMavFrame = require('./SetMavFrame.js')
let CommandBool = require('./CommandBool.js')
let FileList = require('./FileList.js')
let FileRead = require('./FileRead.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let LogRequestList = require('./LogRequestList.js')
let WaypointClear = require('./WaypointClear.js')
let CommandTOL = require('./CommandTOL.js')
let CommandInt = require('./CommandInt.js')
let FileRemove = require('./FileRemove.js')
let MountConfigure = require('./MountConfigure.js')
let FileWrite = require('./FileWrite.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let FileRename = require('./FileRename.js')
let WaypointPush = require('./WaypointPush.js')
let StreamRate = require('./StreamRate.js')
let ParamPush = require('./ParamPush.js')
let FileMakeDir = require('./FileMakeDir.js')
let ParamSet = require('./ParamSet.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let CommandAck = require('./CommandAck.js')
let CommandLong = require('./CommandLong.js')
let ParamGet = require('./ParamGet.js')

module.exports = {
  SetMode: SetMode,
  FileOpen: FileOpen,
  MessageInterval: MessageInterval,
  LogRequestData: LogRequestData,
  ParamPull: ParamPull,
  CommandHome: CommandHome,
  FileRemoveDir: FileRemoveDir,
  WaypointPull: WaypointPull,
  FileTruncate: FileTruncate,
  WaypointSetCurrent: WaypointSetCurrent,
  FileChecksum: FileChecksum,
  FileClose: FileClose,
  SetMavFrame: SetMavFrame,
  CommandBool: CommandBool,
  FileList: FileList,
  FileRead: FileRead,
  VehicleInfoGet: VehicleInfoGet,
  CommandVtolTransition: CommandVtolTransition,
  CommandTriggerControl: CommandTriggerControl,
  LogRequestList: LogRequestList,
  WaypointClear: WaypointClear,
  CommandTOL: CommandTOL,
  CommandInt: CommandInt,
  FileRemove: FileRemove,
  MountConfigure: MountConfigure,
  FileWrite: FileWrite,
  CommandTriggerInterval: CommandTriggerInterval,
  FileRename: FileRename,
  WaypointPush: WaypointPush,
  StreamRate: StreamRate,
  ParamPush: ParamPush,
  FileMakeDir: FileMakeDir,
  ParamSet: ParamSet,
  LogRequestEnd: LogRequestEnd,
  CommandAck: CommandAck,
  CommandLong: CommandLong,
  ParamGet: ParamGet,
};
