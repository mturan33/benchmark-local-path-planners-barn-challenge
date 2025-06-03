;; Auto-generated. Do not edit!


(when (boundp 'jackal_msgs::Status)
  (if (not (find-package "JACKAL_MSGS"))
    (make-package "JACKAL_MSGS"))
  (shadow 'Status (find-package "JACKAL_MSGS")))
(unless (find-package "JACKAL_MSGS::STATUS")
  (make-package "JACKAL_MSGS::STATUS"))

(in-package "ROS")
;;//! \htmlinclude Status.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jackal_msgs::Status
  :super ros::object
  :slots (_header _hardware_id _mcu_uptime _connection_uptime _drivers_active _driver_external_stop_present _driver_external_stop_stopped _measured_battery _measured_12v _measured_5v _drive_current _user_current _computer_current _total_current _total_current_peak _total_power_consumed ))

(defmethod jackal_msgs::Status
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:hardware_id __hardware_id) "")
    ((:mcu_uptime __mcu_uptime) (instance ros::time :init))
    ((:connection_uptime __connection_uptime) (instance ros::time :init))
    ((:drivers_active __drivers_active) nil)
    ((:driver_external_stop_present __driver_external_stop_present) nil)
    ((:driver_external_stop_stopped __driver_external_stop_stopped) nil)
    ((:measured_battery __measured_battery) 0.0)
    ((:measured_12v __measured_12v) 0.0)
    ((:measured_5v __measured_5v) 0.0)
    ((:drive_current __drive_current) 0.0)
    ((:user_current __user_current) 0.0)
    ((:computer_current __computer_current) 0.0)
    ((:total_current __total_current) 0.0)
    ((:total_current_peak __total_current_peak) 0.0)
    ((:total_power_consumed __total_power_consumed) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _hardware_id (string __hardware_id))
   (setq _mcu_uptime __mcu_uptime)
   (setq _connection_uptime __connection_uptime)
   (setq _drivers_active __drivers_active)
   (setq _driver_external_stop_present __driver_external_stop_present)
   (setq _driver_external_stop_stopped __driver_external_stop_stopped)
   (setq _measured_battery (float __measured_battery))
   (setq _measured_12v (float __measured_12v))
   (setq _measured_5v (float __measured_5v))
   (setq _drive_current (float __drive_current))
   (setq _user_current (float __user_current))
   (setq _computer_current (float __computer_current))
   (setq _total_current (float __total_current))
   (setq _total_current_peak (float __total_current_peak))
   (setq _total_power_consumed (float __total_power_consumed))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:hardware_id
   (&optional __hardware_id)
   (if __hardware_id (setq _hardware_id __hardware_id)) _hardware_id)
  (:mcu_uptime
   (&optional __mcu_uptime)
   (if __mcu_uptime (setq _mcu_uptime __mcu_uptime)) _mcu_uptime)
  (:connection_uptime
   (&optional __connection_uptime)
   (if __connection_uptime (setq _connection_uptime __connection_uptime)) _connection_uptime)
  (:drivers_active
   (&optional __drivers_active)
   (if __drivers_active (setq _drivers_active __drivers_active)) _drivers_active)
  (:driver_external_stop_present
   (&optional __driver_external_stop_present)
   (if __driver_external_stop_present (setq _driver_external_stop_present __driver_external_stop_present)) _driver_external_stop_present)
  (:driver_external_stop_stopped
   (&optional __driver_external_stop_stopped)
   (if __driver_external_stop_stopped (setq _driver_external_stop_stopped __driver_external_stop_stopped)) _driver_external_stop_stopped)
  (:measured_battery
   (&optional __measured_battery)
   (if __measured_battery (setq _measured_battery __measured_battery)) _measured_battery)
  (:measured_12v
   (&optional __measured_12v)
   (if __measured_12v (setq _measured_12v __measured_12v)) _measured_12v)
  (:measured_5v
   (&optional __measured_5v)
   (if __measured_5v (setq _measured_5v __measured_5v)) _measured_5v)
  (:drive_current
   (&optional __drive_current)
   (if __drive_current (setq _drive_current __drive_current)) _drive_current)
  (:user_current
   (&optional __user_current)
   (if __user_current (setq _user_current __user_current)) _user_current)
  (:computer_current
   (&optional __computer_current)
   (if __computer_current (setq _computer_current __computer_current)) _computer_current)
  (:total_current
   (&optional __total_current)
   (if __total_current (setq _total_current __total_current)) _total_current)
  (:total_current_peak
   (&optional __total_current_peak)
   (if __total_current_peak (setq _total_current_peak __total_current_peak)) _total_current_peak)
  (:total_power_consumed
   (&optional __total_power_consumed)
   (if __total_power_consumed (setq _total_power_consumed __total_power_consumed)) _total_power_consumed)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _hardware_id
    4 (length _hardware_id)
    ;; duration _mcu_uptime
    8
    ;; duration _connection_uptime
    8
    ;; bool _drivers_active
    1
    ;; bool _driver_external_stop_present
    1
    ;; bool _driver_external_stop_stopped
    1
    ;; float32 _measured_battery
    4
    ;; float32 _measured_12v
    4
    ;; float32 _measured_5v
    4
    ;; float32 _drive_current
    4
    ;; float32 _user_current
    4
    ;; float32 _computer_current
    4
    ;; float32 _total_current
    4
    ;; float32 _total_current_peak
    4
    ;; float64 _total_power_consumed
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _hardware_id
       (write-long (length _hardware_id) s) (princ _hardware_id s)
     ;; duration _mcu_uptime
       (write-long (send _mcu_uptime :sec) s) (write-long (send _mcu_uptime :nsec) s)
     ;; duration _connection_uptime
       (write-long (send _connection_uptime :sec) s) (write-long (send _connection_uptime :nsec) s)
     ;; bool _drivers_active
       (if _drivers_active (write-byte -1 s) (write-byte 0 s))
     ;; bool _driver_external_stop_present
       (if _driver_external_stop_present (write-byte -1 s) (write-byte 0 s))
     ;; bool _driver_external_stop_stopped
       (if _driver_external_stop_stopped (write-byte -1 s) (write-byte 0 s))
     ;; float32 _measured_battery
       (sys::poke _measured_battery (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _measured_12v
       (sys::poke _measured_12v (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _measured_5v
       (sys::poke _measured_5v (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _drive_current
       (sys::poke _drive_current (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _user_current
       (sys::poke _user_current (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _computer_current
       (sys::poke _computer_current (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _total_current
       (sys::poke _total_current (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _total_current_peak
       (sys::poke _total_current_peak (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _total_power_consumed
       (sys::poke _total_power_consumed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _hardware_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _hardware_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; duration _mcu_uptime
     (send _mcu_uptime :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _mcu_uptime :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; duration _connection_uptime
     (send _connection_uptime :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _connection_uptime :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _drivers_active
     (setq _drivers_active (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _driver_external_stop_present
     (setq _driver_external_stop_present (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _driver_external_stop_stopped
     (setq _driver_external_stop_stopped (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _measured_battery
     (setq _measured_battery (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _measured_12v
     (setq _measured_12v (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _measured_5v
     (setq _measured_5v (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _drive_current
     (setq _drive_current (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _user_current
     (setq _user_current (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _computer_current
     (setq _computer_current (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _total_current
     (setq _total_current (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _total_current_peak
     (setq _total_current_peak (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _total_power_consumed
     (setq _total_power_consumed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get jackal_msgs::Status :md5sum-) "c851ebcf9a6e20b196bc7894e285b4f6")
(setf (get jackal_msgs::Status :datatype-) "jackal_msgs/Status")
(setf (get jackal_msgs::Status :definition-)
      "# This message represents Jackal's lower-frequency status updates
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

")



(provide :jackal_msgs/Status "c851ebcf9a6e20b196bc7894e285b4f6")


