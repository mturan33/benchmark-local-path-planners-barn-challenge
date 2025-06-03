// Auto-generated. Do not edit!

// (in-package jackal_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Status {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.hardware_id = null;
      this.mcu_uptime = null;
      this.connection_uptime = null;
      this.drivers_active = null;
      this.driver_external_stop_present = null;
      this.driver_external_stop_stopped = null;
      this.measured_battery = null;
      this.measured_12v = null;
      this.measured_5v = null;
      this.drive_current = null;
      this.user_current = null;
      this.computer_current = null;
      this.total_current = null;
      this.total_current_peak = null;
      this.total_power_consumed = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('hardware_id')) {
        this.hardware_id = initObj.hardware_id
      }
      else {
        this.hardware_id = '';
      }
      if (initObj.hasOwnProperty('mcu_uptime')) {
        this.mcu_uptime = initObj.mcu_uptime
      }
      else {
        this.mcu_uptime = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('connection_uptime')) {
        this.connection_uptime = initObj.connection_uptime
      }
      else {
        this.connection_uptime = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('drivers_active')) {
        this.drivers_active = initObj.drivers_active
      }
      else {
        this.drivers_active = false;
      }
      if (initObj.hasOwnProperty('driver_external_stop_present')) {
        this.driver_external_stop_present = initObj.driver_external_stop_present
      }
      else {
        this.driver_external_stop_present = false;
      }
      if (initObj.hasOwnProperty('driver_external_stop_stopped')) {
        this.driver_external_stop_stopped = initObj.driver_external_stop_stopped
      }
      else {
        this.driver_external_stop_stopped = false;
      }
      if (initObj.hasOwnProperty('measured_battery')) {
        this.measured_battery = initObj.measured_battery
      }
      else {
        this.measured_battery = 0.0;
      }
      if (initObj.hasOwnProperty('measured_12v')) {
        this.measured_12v = initObj.measured_12v
      }
      else {
        this.measured_12v = 0.0;
      }
      if (initObj.hasOwnProperty('measured_5v')) {
        this.measured_5v = initObj.measured_5v
      }
      else {
        this.measured_5v = 0.0;
      }
      if (initObj.hasOwnProperty('drive_current')) {
        this.drive_current = initObj.drive_current
      }
      else {
        this.drive_current = 0.0;
      }
      if (initObj.hasOwnProperty('user_current')) {
        this.user_current = initObj.user_current
      }
      else {
        this.user_current = 0.0;
      }
      if (initObj.hasOwnProperty('computer_current')) {
        this.computer_current = initObj.computer_current
      }
      else {
        this.computer_current = 0.0;
      }
      if (initObj.hasOwnProperty('total_current')) {
        this.total_current = initObj.total_current
      }
      else {
        this.total_current = 0.0;
      }
      if (initObj.hasOwnProperty('total_current_peak')) {
        this.total_current_peak = initObj.total_current_peak
      }
      else {
        this.total_current_peak = 0.0;
      }
      if (initObj.hasOwnProperty('total_power_consumed')) {
        this.total_power_consumed = initObj.total_power_consumed
      }
      else {
        this.total_power_consumed = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Status
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [hardware_id]
    bufferOffset = _serializer.string(obj.hardware_id, buffer, bufferOffset);
    // Serialize message field [mcu_uptime]
    bufferOffset = _serializer.duration(obj.mcu_uptime, buffer, bufferOffset);
    // Serialize message field [connection_uptime]
    bufferOffset = _serializer.duration(obj.connection_uptime, buffer, bufferOffset);
    // Serialize message field [drivers_active]
    bufferOffset = _serializer.bool(obj.drivers_active, buffer, bufferOffset);
    // Serialize message field [driver_external_stop_present]
    bufferOffset = _serializer.bool(obj.driver_external_stop_present, buffer, bufferOffset);
    // Serialize message field [driver_external_stop_stopped]
    bufferOffset = _serializer.bool(obj.driver_external_stop_stopped, buffer, bufferOffset);
    // Serialize message field [measured_battery]
    bufferOffset = _serializer.float32(obj.measured_battery, buffer, bufferOffset);
    // Serialize message field [measured_12v]
    bufferOffset = _serializer.float32(obj.measured_12v, buffer, bufferOffset);
    // Serialize message field [measured_5v]
    bufferOffset = _serializer.float32(obj.measured_5v, buffer, bufferOffset);
    // Serialize message field [drive_current]
    bufferOffset = _serializer.float32(obj.drive_current, buffer, bufferOffset);
    // Serialize message field [user_current]
    bufferOffset = _serializer.float32(obj.user_current, buffer, bufferOffset);
    // Serialize message field [computer_current]
    bufferOffset = _serializer.float32(obj.computer_current, buffer, bufferOffset);
    // Serialize message field [total_current]
    bufferOffset = _serializer.float32(obj.total_current, buffer, bufferOffset);
    // Serialize message field [total_current_peak]
    bufferOffset = _serializer.float32(obj.total_current_peak, buffer, bufferOffset);
    // Serialize message field [total_power_consumed]
    bufferOffset = _serializer.float64(obj.total_power_consumed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Status
    let len;
    let data = new Status(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [hardware_id]
    data.hardware_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [mcu_uptime]
    data.mcu_uptime = _deserializer.duration(buffer, bufferOffset);
    // Deserialize message field [connection_uptime]
    data.connection_uptime = _deserializer.duration(buffer, bufferOffset);
    // Deserialize message field [drivers_active]
    data.drivers_active = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [driver_external_stop_present]
    data.driver_external_stop_present = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [driver_external_stop_stopped]
    data.driver_external_stop_stopped = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [measured_battery]
    data.measured_battery = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [measured_12v]
    data.measured_12v = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [measured_5v]
    data.measured_5v = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [drive_current]
    data.drive_current = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [user_current]
    data.user_current = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [computer_current]
    data.computer_current = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [total_current]
    data.total_current = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [total_current_peak]
    data.total_current_peak = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [total_power_consumed]
    data.total_power_consumed = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.hardware_id.length;
    return length + 63;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jackal_msgs/Status';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c851ebcf9a6e20b196bc7894e285b4f6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This message represents Jackal's lower-frequency status updates
    # Default publish frequency is 1Hz.
    
    Header header
    
    # Commit of firmware source.
    string hardware_id
    
    # Times since MCU power-on and MCU rosserial connection, respectively.
    duration mcu_uptime
    duration connection_uptime
    
    # Monitoring the run/stop loop. Changes in these values trigger an immediate
    # publish, outside the ordinarily-scheduled 1Hz updates.
    bool drivers_active
    bool driver_external_stop_present
    bool driver_external_stop_stopped
    
    # Voltage rails, in volts
    # Averaged over the message period
    float32 measured_battery
    float32 measured_12v
    float32 measured_5v
    
    # Current senses available on platform, in amps.
    # Averaged over the message period
    float32 drive_current
    float32 user_current
    float32 computer_current
    float32 total_current
    
    # Highest total system current peak as measured in a 1ms window.
    float32 total_current_peak
    
    # Integration of all power consumption since MCU power-on, in watt-hours.
    float64 total_power_consumed 
    
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
    const resolved = new Status(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.hardware_id !== undefined) {
      resolved.hardware_id = msg.hardware_id;
    }
    else {
      resolved.hardware_id = ''
    }

    if (msg.mcu_uptime !== undefined) {
      resolved.mcu_uptime = msg.mcu_uptime;
    }
    else {
      resolved.mcu_uptime = {secs: 0, nsecs: 0}
    }

    if (msg.connection_uptime !== undefined) {
      resolved.connection_uptime = msg.connection_uptime;
    }
    else {
      resolved.connection_uptime = {secs: 0, nsecs: 0}
    }

    if (msg.drivers_active !== undefined) {
      resolved.drivers_active = msg.drivers_active;
    }
    else {
      resolved.drivers_active = false
    }

    if (msg.driver_external_stop_present !== undefined) {
      resolved.driver_external_stop_present = msg.driver_external_stop_present;
    }
    else {
      resolved.driver_external_stop_present = false
    }

    if (msg.driver_external_stop_stopped !== undefined) {
      resolved.driver_external_stop_stopped = msg.driver_external_stop_stopped;
    }
    else {
      resolved.driver_external_stop_stopped = false
    }

    if (msg.measured_battery !== undefined) {
      resolved.measured_battery = msg.measured_battery;
    }
    else {
      resolved.measured_battery = 0.0
    }

    if (msg.measured_12v !== undefined) {
      resolved.measured_12v = msg.measured_12v;
    }
    else {
      resolved.measured_12v = 0.0
    }

    if (msg.measured_5v !== undefined) {
      resolved.measured_5v = msg.measured_5v;
    }
    else {
      resolved.measured_5v = 0.0
    }

    if (msg.drive_current !== undefined) {
      resolved.drive_current = msg.drive_current;
    }
    else {
      resolved.drive_current = 0.0
    }

    if (msg.user_current !== undefined) {
      resolved.user_current = msg.user_current;
    }
    else {
      resolved.user_current = 0.0
    }

    if (msg.computer_current !== undefined) {
      resolved.computer_current = msg.computer_current;
    }
    else {
      resolved.computer_current = 0.0
    }

    if (msg.total_current !== undefined) {
      resolved.total_current = msg.total_current;
    }
    else {
      resolved.total_current = 0.0
    }

    if (msg.total_current_peak !== undefined) {
      resolved.total_current_peak = msg.total_current_peak;
    }
    else {
      resolved.total_current_peak = 0.0
    }

    if (msg.total_power_consumed !== undefined) {
      resolved.total_power_consumed = msg.total_power_consumed;
    }
    else {
      resolved.total_power_consumed = 0.0
    }

    return resolved;
    }
};

module.exports = Status;
