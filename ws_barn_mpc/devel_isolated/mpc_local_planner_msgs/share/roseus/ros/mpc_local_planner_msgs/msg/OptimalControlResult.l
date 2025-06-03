;; Auto-generated. Do not edit!


(when (boundp 'mpc_local_planner_msgs::OptimalControlResult)
  (if (not (find-package "MPC_LOCAL_PLANNER_MSGS"))
    (make-package "MPC_LOCAL_PLANNER_MSGS"))
  (shadow 'OptimalControlResult (find-package "MPC_LOCAL_PLANNER_MSGS")))
(unless (find-package "MPC_LOCAL_PLANNER_MSGS::OPTIMALCONTROLRESULT")
  (make-package "MPC_LOCAL_PLANNER_MSGS::OPTIMALCONTROLRESULT"))

(in-package "ROS")
;;//! \htmlinclude OptimalControlResult.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mpc_local_planner_msgs::OptimalControlResult
  :super ros::object
  :slots (_header _dim_states _dim_controls _time_states _states _time_controls _controls _optimal_solution_found _cpu_time ))

(defmethod mpc_local_planner_msgs::OptimalControlResult
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:dim_states __dim_states) 0)
    ((:dim_controls __dim_controls) 0)
    ((:time_states __time_states) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:states __states) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:time_controls __time_controls) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:controls __controls) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:optimal_solution_found __optimal_solution_found) nil)
    ((:cpu_time __cpu_time) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _dim_states (round __dim_states))
   (setq _dim_controls (round __dim_controls))
   (setq _time_states __time_states)
   (setq _states __states)
   (setq _time_controls __time_controls)
   (setq _controls __controls)
   (setq _optimal_solution_found __optimal_solution_found)
   (setq _cpu_time (float __cpu_time))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:dim_states
   (&optional __dim_states)
   (if __dim_states (setq _dim_states __dim_states)) _dim_states)
  (:dim_controls
   (&optional __dim_controls)
   (if __dim_controls (setq _dim_controls __dim_controls)) _dim_controls)
  (:time_states
   (&optional __time_states)
   (if __time_states (setq _time_states __time_states)) _time_states)
  (:states
   (&optional __states)
   (if __states (setq _states __states)) _states)
  (:time_controls
   (&optional __time_controls)
   (if __time_controls (setq _time_controls __time_controls)) _time_controls)
  (:controls
   (&optional __controls)
   (if __controls (setq _controls __controls)) _controls)
  (:optimal_solution_found
   (&optional __optimal_solution_found)
   (if __optimal_solution_found (setq _optimal_solution_found __optimal_solution_found)) _optimal_solution_found)
  (:cpu_time
   (&optional __cpu_time)
   (if __cpu_time (setq _cpu_time __cpu_time)) _cpu_time)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int64 _dim_states
    8
    ;; int64 _dim_controls
    8
    ;; float64[] _time_states
    (* 8    (length _time_states)) 4
    ;; float64[] _states
    (* 8    (length _states)) 4
    ;; float64[] _time_controls
    (* 8    (length _time_controls)) 4
    ;; float64[] _controls
    (* 8    (length _controls)) 4
    ;; bool _optimal_solution_found
    1
    ;; float64 _cpu_time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int64 _dim_states
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _dim_states (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _dim_states) (= (length (_dim_states . bv)) 2)) ;; bignum
              (write-long (ash (elt (_dim_states . bv) 0) 0) s)
              (write-long (ash (elt (_dim_states . bv) 1) -1) s))
             ((and (class _dim_states) (= (length (_dim_states . bv)) 1)) ;; big1
              (write-long (elt (_dim_states . bv) 0) s)
              (write-long (if (>= _dim_states 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _dim_states s)(write-long (if (>= _dim_states 0) 0 #xffffffff) s)))
     ;; int64 _dim_controls
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _dim_controls (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _dim_controls) (= (length (_dim_controls . bv)) 2)) ;; bignum
              (write-long (ash (elt (_dim_controls . bv) 0) 0) s)
              (write-long (ash (elt (_dim_controls . bv) 1) -1) s))
             ((and (class _dim_controls) (= (length (_dim_controls . bv)) 1)) ;; big1
              (write-long (elt (_dim_controls . bv) 0) s)
              (write-long (if (>= _dim_controls 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _dim_controls s)(write-long (if (>= _dim_controls 0) 0 #xffffffff) s)))
     ;; float64[] _time_states
     (write-long (length _time_states) s)
     (dotimes (i (length _time_states))
       (sys::poke (elt _time_states i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _states
     (write-long (length _states) s)
     (dotimes (i (length _states))
       (sys::poke (elt _states i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _time_controls
     (write-long (length _time_controls) s)
     (dotimes (i (length _time_controls))
       (sys::poke (elt _time_controls i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _controls
     (write-long (length _controls) s)
     (dotimes (i (length _controls))
       (sys::poke (elt _controls i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _optimal_solution_found
       (if _optimal_solution_found (write-byte -1 s) (write-byte 0 s))
     ;; float64 _cpu_time
       (sys::poke _cpu_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int64 _dim_states
#+(or :alpha :irix6 :x86_64)
      (setf _dim_states (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _dim_states (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _dim_controls
#+(or :alpha :irix6 :x86_64)
      (setf _dim_controls (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _dim_controls (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64[] _time_states
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _time_states (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _time_states i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _states
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _states (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _states i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _time_controls
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _time_controls (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _time_controls i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _controls
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _controls (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _controls i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; bool _optimal_solution_found
     (setq _optimal_solution_found (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _cpu_time
     (setq _cpu_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get mpc_local_planner_msgs::OptimalControlResult :md5sum-) "43eac4f3a65752bab175326ddc347c3d")
(setf (get mpc_local_planner_msgs::OptimalControlResult :datatype-) "mpc_local_planner_msgs/OptimalControlResult")
(setf (get mpc_local_planner_msgs::OptimalControlResult :definition-)
      "std_msgs/Header header

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

")



(provide :mpc_local_planner_msgs/OptimalControlResult "43eac4f3a65752bab175326ddc347c3d")


