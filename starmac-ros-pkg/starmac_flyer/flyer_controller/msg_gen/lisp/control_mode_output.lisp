; Auto-generated. Do not edit!


(cl:in-package flyer_controller-msg)


;//! \htmlinclude control_mode_output.msg.html

(cl:defclass <control_mode_output> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (control_mode
    :reader control_mode
    :initarg :control_mode
    :type cl:string
    :initform "")
   (motors_on
    :reader motors_on
    :initarg :motors_on
    :type cl:boolean
    :initform cl:nil)
   (roll_cmd
    :reader roll_cmd
    :initarg :roll_cmd
    :type cl:float
    :initform 0.0)
   (pitch_cmd
    :reader pitch_cmd
    :initarg :pitch_cmd
    :type cl:float
    :initform 0.0)
   (direct_yaw_rate_commands
    :reader direct_yaw_rate_commands
    :initarg :direct_yaw_rate_commands
    :type cl:boolean
    :initform cl:nil)
   (yaw_cmd
    :reader yaw_cmd
    :initarg :yaw_cmd
    :type cl:float
    :initform 0.0)
   (yaw_rate_cmd
    :reader yaw_rate_cmd
    :initarg :yaw_rate_cmd
    :type cl:float
    :initform 0.0)
   (direct_thrust_commands
    :reader direct_thrust_commands
    :initarg :direct_thrust_commands
    :type cl:boolean
    :initform cl:nil)
   (alt_cmd
    :reader alt_cmd
    :initarg :alt_cmd
    :type cl:float
    :initform 0.0)
   (thrust_cmd
    :reader thrust_cmd
    :initarg :thrust_cmd
    :type cl:float
    :initform 0.0))
)

(cl:defclass control_mode_output (<control_mode_output>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <control_mode_output>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'control_mode_output)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name flyer_controller-msg:<control_mode_output> is deprecated: use flyer_controller-msg:control_mode_output instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <control_mode_output>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:header-val is deprecated.  Use flyer_controller-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'control_mode-val :lambda-list '(m))
(cl:defmethod control_mode-val ((m <control_mode_output>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:control_mode-val is deprecated.  Use flyer_controller-msg:control_mode instead.")
  (control_mode m))

(cl:ensure-generic-function 'motors_on-val :lambda-list '(m))
(cl:defmethod motors_on-val ((m <control_mode_output>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:motors_on-val is deprecated.  Use flyer_controller-msg:motors_on instead.")
  (motors_on m))

(cl:ensure-generic-function 'roll_cmd-val :lambda-list '(m))
(cl:defmethod roll_cmd-val ((m <control_mode_output>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:roll_cmd-val is deprecated.  Use flyer_controller-msg:roll_cmd instead.")
  (roll_cmd m))

(cl:ensure-generic-function 'pitch_cmd-val :lambda-list '(m))
(cl:defmethod pitch_cmd-val ((m <control_mode_output>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:pitch_cmd-val is deprecated.  Use flyer_controller-msg:pitch_cmd instead.")
  (pitch_cmd m))

(cl:ensure-generic-function 'direct_yaw_rate_commands-val :lambda-list '(m))
(cl:defmethod direct_yaw_rate_commands-val ((m <control_mode_output>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:direct_yaw_rate_commands-val is deprecated.  Use flyer_controller-msg:direct_yaw_rate_commands instead.")
  (direct_yaw_rate_commands m))

(cl:ensure-generic-function 'yaw_cmd-val :lambda-list '(m))
(cl:defmethod yaw_cmd-val ((m <control_mode_output>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:yaw_cmd-val is deprecated.  Use flyer_controller-msg:yaw_cmd instead.")
  (yaw_cmd m))

(cl:ensure-generic-function 'yaw_rate_cmd-val :lambda-list '(m))
(cl:defmethod yaw_rate_cmd-val ((m <control_mode_output>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:yaw_rate_cmd-val is deprecated.  Use flyer_controller-msg:yaw_rate_cmd instead.")
  (yaw_rate_cmd m))

(cl:ensure-generic-function 'direct_thrust_commands-val :lambda-list '(m))
(cl:defmethod direct_thrust_commands-val ((m <control_mode_output>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:direct_thrust_commands-val is deprecated.  Use flyer_controller-msg:direct_thrust_commands instead.")
  (direct_thrust_commands m))

(cl:ensure-generic-function 'alt_cmd-val :lambda-list '(m))
(cl:defmethod alt_cmd-val ((m <control_mode_output>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:alt_cmd-val is deprecated.  Use flyer_controller-msg:alt_cmd instead.")
  (alt_cmd m))

(cl:ensure-generic-function 'thrust_cmd-val :lambda-list '(m))
(cl:defmethod thrust_cmd-val ((m <control_mode_output>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:thrust_cmd-val is deprecated.  Use flyer_controller-msg:thrust_cmd instead.")
  (thrust_cmd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <control_mode_output>) ostream)
  "Serializes a message object of type '<control_mode_output>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'control_mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'control_mode))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'motors_on) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'roll_cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pitch_cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'direct_yaw_rate_commands) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yaw_cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yaw_rate_cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'direct_thrust_commands) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'alt_cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'thrust_cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <control_mode_output>) istream)
  "Deserializes a message object of type '<control_mode_output>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'control_mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'control_mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'motors_on) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll_cmd) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch_cmd) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'direct_yaw_rate_commands) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_cmd) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_rate_cmd) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'direct_thrust_commands) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'alt_cmd) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thrust_cmd) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<control_mode_output>)))
  "Returns string type for a message object of type '<control_mode_output>"
  "flyer_controller/control_mode_output")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'control_mode_output)))
  "Returns string type for a message object of type 'control_mode_output"
  "flyer_controller/control_mode_output")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<control_mode_output>)))
  "Returns md5sum for a message object of type '<control_mode_output>"
  "58192697145997c9f1076d3806892e40")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'control_mode_output)))
  "Returns md5sum for a message object of type 'control_mode_output"
  "58192697145997c9f1076d3806892e40")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<control_mode_output>)))
  "Returns full string definition for message of type '<control_mode_output>"
  (cl:format cl:nil "Header header~%string control_mode~%bool motors_on~%float64 roll_cmd # deg~%float64 pitch_cmd # deg~%bool direct_yaw_rate_commands # set true to send yaw rate commands, false to send yaw angle~%float64 yaw_cmd # deg~%float64 yaw_rate_cmd # deg/s~%bool direct_thrust_commands # set true to send thrust directly~%float64 alt_cmd # m - ignored if direct_thrust_commands = true~%float64 thrust_cmd # N - ignored if direct_thrust_commands = false~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'control_mode_output)))
  "Returns full string definition for message of type 'control_mode_output"
  (cl:format cl:nil "Header header~%string control_mode~%bool motors_on~%float64 roll_cmd # deg~%float64 pitch_cmd # deg~%bool direct_yaw_rate_commands # set true to send yaw rate commands, false to send yaw angle~%float64 yaw_cmd # deg~%float64 yaw_rate_cmd # deg/s~%bool direct_thrust_commands # set true to send thrust directly~%float64 alt_cmd # m - ignored if direct_thrust_commands = true~%float64 thrust_cmd # N - ignored if direct_thrust_commands = false~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <control_mode_output>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'control_mode))
     1
     8
     8
     1
     8
     8
     1
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <control_mode_output>))
  "Converts a ROS message object to a list"
  (cl:list 'control_mode_output
    (cl:cons ':header (header msg))
    (cl:cons ':control_mode (control_mode msg))
    (cl:cons ':motors_on (motors_on msg))
    (cl:cons ':roll_cmd (roll_cmd msg))
    (cl:cons ':pitch_cmd (pitch_cmd msg))
    (cl:cons ':direct_yaw_rate_commands (direct_yaw_rate_commands msg))
    (cl:cons ':yaw_cmd (yaw_cmd msg))
    (cl:cons ':yaw_rate_cmd (yaw_rate_cmd msg))
    (cl:cons ':direct_thrust_commands (direct_thrust_commands msg))
    (cl:cons ':alt_cmd (alt_cmd msg))
    (cl:cons ':thrust_cmd (thrust_cmd msg))
))
