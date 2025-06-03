// Auto-generated. Do not edit!

// (in-package jackal_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DriveFeedback = require('./DriveFeedback.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Feedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.drivers = null;
      this.pcb_temperature = null;
      this.mcu_temperature = null;
      this.commanded_mode = null;
      this.actual_mode = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('drivers')) {
        this.drivers = initObj.drivers
      }
      else {
        this.drivers = new Array(2).fill(new DriveFeedback());
      }
      if (initObj.hasOwnProperty('pcb_temperature')) {
        this.pcb_temperature = initObj.pcb_temperature
      }
      else {
        this.pcb_temperature = 0.0;
      }
      if (initObj.hasOwnProperty('mcu_temperature')) {
        this.mcu_temperature = initObj.mcu_temperature
      }
      else {
        this.mcu_temperature = 0.0;
      }
      if (initObj.hasOwnProperty('commanded_mode')) {
        this.commanded_mode = initObj.commanded_mode
      }
      else {
        this.commanded_mode = 0;
      }
      if (initObj.hasOwnProperty('actual_mode')) {
        this.actual_mode = initObj.actual_mode
      }
      else {
        this.actual_mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Feedback
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [drivers] has the right length
    if (obj.drivers.length !== 2) {
      throw new Error('Unable to serialize array field drivers - length must be 2')
    }
    // Serialize message field [drivers]
    obj.drivers.forEach((val) => {
      bufferOffset = DriveFeedback.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [pcb_temperature]
    bufferOffset = _serializer.float32(obj.pcb_temperature, buffer, bufferOffset);
    // Serialize message field [mcu_temperature]
    bufferOffset = _serializer.float32(obj.mcu_temperature, buffer, bufferOffset);
    // Serialize message field [commanded_mode]
    bufferOffset = _serializer.int8(obj.commanded_mode, buffer, bufferOffset);
    // Serialize message field [actual_mode]
    bufferOffset = _serializer.int8(obj.actual_mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Feedback
    let len;
    let data = new Feedback(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [drivers]
    len = 2;
    data.drivers = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.drivers[i] = DriveFeedback.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [pcb_temperature]
    data.pcb_temperature = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [mcu_temperature]
    data.mcu_temperature = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [commanded_mode]
    data.commanded_mode = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [actual_mode]
    data.actual_mode = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 60;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jackal_msgs/Feedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3bdabb0ef46338ee5672d1b82220ab49';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This message represents high-frequency feedback from the MCU,
    # as necessary to support closed-loop control and thermal monitoring.
    # Default publish frequency is 50Hz.
    
    Header header
    
    DriveFeedback[2] drivers
    
    # Temperatures 
    float32 pcb_temperature
    float32 mcu_temperature
    
    # Commanded control mode, use the TYPE_ constants from jackal_msgs/Drive.
    int8 commanded_mode
    
    # Actual control mode. This may differ from the commanded in cases where
    # the motor enable is off, the motors are in over-current, etc.
    int8 actual_mode
    
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
    
    ================================================================================
    MSG: jackal_msgs/DriveFeedback
    # This message represents feedback data from a single drive unit (driver + motor).
    
    # Current flowing from battery into the MOSFET bridge.
    float32 current
    
    # Instantaneous duty cycle of MOSFET bridge.
    float32 duty_cycle
    
    # Temperatures measured in the MOSFET bridge and on the motor casing, in deg C.
    float32 bridge_temperature
    float32 motor_temperature
    
    # Encoder data
    float32 measured_velocity   # rad/s
    float32 measured_travel     # rad
    
    # True if the underlying driver chip reports a fault condition.
    bool driver_fault
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Feedback(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.drivers !== undefined) {
      resolved.drivers = new Array(2)
      for (let i = 0; i < resolved.drivers.length; ++i) {
        if (msg.drivers.length > i) {
          resolved.drivers[i] = DriveFeedback.Resolve(msg.drivers[i]);
        }
        else {
          resolved.drivers[i] = new DriveFeedback();
        }
      }
    }
    else {
      resolved.drivers = new Array(2).fill(new DriveFeedback())
    }

    if (msg.pcb_temperature !== undefined) {
      resolved.pcb_temperature = msg.pcb_temperature;
    }
    else {
      resolved.pcb_temperature = 0.0
    }

    if (msg.mcu_temperature !== undefined) {
      resolved.mcu_temperature = msg.mcu_temperature;
    }
    else {
      resolved.mcu_temperature = 0.0
    }

    if (msg.commanded_mode !== undefined) {
      resolved.commanded_mode = msg.commanded_mode;
    }
    else {
      resolved.commanded_mode = 0
    }

    if (msg.actual_mode !== undefined) {
      resolved.actual_mode = msg.actual_mode;
    }
    else {
      resolved.actual_mode = 0
    }

    return resolved;
    }
};

module.exports = Feedback;
