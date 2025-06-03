// Auto-generated. Do not edit!

// (in-package mpc_local_planner_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class OptimalControlResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.dim_states = null;
      this.dim_controls = null;
      this.time_states = null;
      this.states = null;
      this.time_controls = null;
      this.controls = null;
      this.optimal_solution_found = null;
      this.cpu_time = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('dim_states')) {
        this.dim_states = initObj.dim_states
      }
      else {
        this.dim_states = 0;
      }
      if (initObj.hasOwnProperty('dim_controls')) {
        this.dim_controls = initObj.dim_controls
      }
      else {
        this.dim_controls = 0;
      }
      if (initObj.hasOwnProperty('time_states')) {
        this.time_states = initObj.time_states
      }
      else {
        this.time_states = [];
      }
      if (initObj.hasOwnProperty('states')) {
        this.states = initObj.states
      }
      else {
        this.states = [];
      }
      if (initObj.hasOwnProperty('time_controls')) {
        this.time_controls = initObj.time_controls
      }
      else {
        this.time_controls = [];
      }
      if (initObj.hasOwnProperty('controls')) {
        this.controls = initObj.controls
      }
      else {
        this.controls = [];
      }
      if (initObj.hasOwnProperty('optimal_solution_found')) {
        this.optimal_solution_found = initObj.optimal_solution_found
      }
      else {
        this.optimal_solution_found = false;
      }
      if (initObj.hasOwnProperty('cpu_time')) {
        this.cpu_time = initObj.cpu_time
      }
      else {
        this.cpu_time = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OptimalControlResult
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [dim_states]
    bufferOffset = _serializer.int64(obj.dim_states, buffer, bufferOffset);
    // Serialize message field [dim_controls]
    bufferOffset = _serializer.int64(obj.dim_controls, buffer, bufferOffset);
    // Serialize message field [time_states]
    bufferOffset = _arraySerializer.float64(obj.time_states, buffer, bufferOffset, null);
    // Serialize message field [states]
    bufferOffset = _arraySerializer.float64(obj.states, buffer, bufferOffset, null);
    // Serialize message field [time_controls]
    bufferOffset = _arraySerializer.float64(obj.time_controls, buffer, bufferOffset, null);
    // Serialize message field [controls]
    bufferOffset = _arraySerializer.float64(obj.controls, buffer, bufferOffset, null);
    // Serialize message field [optimal_solution_found]
    bufferOffset = _serializer.bool(obj.optimal_solution_found, buffer, bufferOffset);
    // Serialize message field [cpu_time]
    bufferOffset = _serializer.float64(obj.cpu_time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OptimalControlResult
    let len;
    let data = new OptimalControlResult(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [dim_states]
    data.dim_states = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [dim_controls]
    data.dim_controls = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [time_states]
    data.time_states = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [states]
    data.states = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [time_controls]
    data.time_controls = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [controls]
    data.controls = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [optimal_solution_found]
    data.optimal_solution_found = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cpu_time]
    data.cpu_time = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 8 * object.time_states.length;
    length += 8 * object.states.length;
    length += 8 * object.time_controls.length;
    length += 8 * object.controls.length;
    return length + 41;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mpc_local_planner_msgs/OptimalControlResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '43eac4f3a65752bab175326ddc347c3d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    int64 dim_states
    int64 dim_controls
    
    float64[] time_states
    float64[] states # Column Major
    float64[] time_controls
    float64[] controls # Column Major
    
    bool optimal_solution_found
    float64 cpu_time
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OptimalControlResult(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.dim_states !== undefined) {
      resolved.dim_states = msg.dim_states;
    }
    else {
      resolved.dim_states = 0
    }

    if (msg.dim_controls !== undefined) {
      resolved.dim_controls = msg.dim_controls;
    }
    else {
      resolved.dim_controls = 0
    }

    if (msg.time_states !== undefined) {
      resolved.time_states = msg.time_states;
    }
    else {
      resolved.time_states = []
    }

    if (msg.states !== undefined) {
      resolved.states = msg.states;
    }
    else {
      resolved.states = []
    }

    if (msg.time_controls !== undefined) {
      resolved.time_controls = msg.time_controls;
    }
    else {
      resolved.time_controls = []
    }

    if (msg.controls !== undefined) {
      resolved.controls = msg.controls;
    }
    else {
      resolved.controls = []
    }

    if (msg.optimal_solution_found !== undefined) {
      resolved.optimal_solution_found = msg.optimal_solution_found;
    }
    else {
      resolved.optimal_solution_found = false
    }

    if (msg.cpu_time !== undefined) {
      resolved.cpu_time = msg.cpu_time;
    }
    else {
      resolved.cpu_time = 0.0
    }

    return resolved;
    }
};

module.exports = OptimalControlResult;
