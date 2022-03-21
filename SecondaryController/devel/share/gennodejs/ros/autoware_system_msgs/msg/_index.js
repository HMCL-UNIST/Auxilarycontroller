
"use strict";

let NodeStatus = require('./NodeStatus.js');
let SystemStatus = require('./SystemStatus.js');
let DiagnosticStatusArray = require('./DiagnosticStatusArray.js');
let DiagnosticStatus = require('./DiagnosticStatus.js');
let HardwareStatus = require('./HardwareStatus.js');

module.exports = {
  NodeStatus: NodeStatus,
  SystemStatus: SystemStatus,
  DiagnosticStatusArray: DiagnosticStatusArray,
  DiagnosticStatus: DiagnosticStatus,
  HardwareStatus: HardwareStatus,
};
