// Auto-generated. Do not edit!

// (in-package jackal_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Drive {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
      this.drivers = null;
    }
    else {
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('drivers')) {
        this.drivers = initObj.drivers
      }
      else {
        this.drivers = new Array(2).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Drive
    // Serialize message field [mode]
    bufferOffset = _serializer.int8(obj.mode, buffer, bufferOffset);
    // Check that the constant length array field [drivers] has the right length
    if (obj.drivers.length !== 2) {
      throw new Error('Unable to serialize array field drivers - length must be 2')
    }
    // Serialize message field [drivers]
    bufferOffset = _arraySerializer.float32(obj.drivers, buffer, bufferOffset, 2);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Drive
    let len;
    let data = new Drive(null);
    // Deserialize message field [mode]
    data.mode = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [drivers]
    data.drivers = _arrayDeserializer.float32(buffer, bufferOffset, 2)
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jackal_msgs/Drive';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '601cf097cd47c174590c366c6ddd5fb3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This message represents a low-level motor command to Jackal.
    
    # Command units dependent on the value of this field
    int8 MODE_VELOCITY=0   # velocity command (rad/s of wheels)
    int8 MODE_PWM=1        # proportion of full voltage command [-1.0..1.0]
    int8 MODE_EFFORT=2     # torque command (Nm)
    int8 MODE_NONE=-1      # no control, commanded values ignored.
    int8 mode
    
    # Units as above, +ve direction propels chassis forward.
    int8 LEFT=0
    int8 RIGHT=1
    float32[2] drivers
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Drive(null);
    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.drivers !== undefined) {
      resolved.drivers = msg.drivers;
    }
    else {
      resolved.drivers = new Array(2).fill(0)
    }

    return resolved;
    }
};

// Constants for message
Drive.Constants = {
  MODE_VELOCITY: 0,
  MODE_PWM: 1,
  MODE_EFFORT: 2,
  MODE_NONE: -1,
  LEFT: 0,
  RIGHT: 1,
}

module.exports = Drive;
