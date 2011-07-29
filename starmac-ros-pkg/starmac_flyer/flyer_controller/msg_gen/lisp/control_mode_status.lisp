; Auto-generated. Do not edit!


(cl:in-package flyer_controller-msg)


;//! \htmlinclude control_mode_status.msg.html

(cl:defclass <control_mode_status> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0)
   (ready
    :reader ready
    :initarg :ready
    :type cl:boolean
    :initform cl:nil)
   (info
    :reader info
    :initarg :info
    :type cl:string
    :initform ""))
)

(cl:defclass control_mode_status (<control_mode_status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <control_mode_status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'control_mode_status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name flyer_controller-msg:<control_mode_status> is deprecated: use flyer_controller-msg:control_mode_status instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <control_mode_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:header-val is deprecated.  Use flyer_controller-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <control_mode_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:state-val is deprecated.  Use flyer_controller-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'ready-val :lambda-list '(m))
(cl:defmethod ready-val ((m <control_mode_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:ready-val is deprecated.  Use flyer_controller-msg:ready instead.")
  (ready m))

(cl:ensure-generic-function 'info-val :lambda-list '(m))
(cl:defmethod info-val ((m <control_mode_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:info-val is deprecated.  Use flyer_controller-msg:info instead.")
  (info m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<control_mode_status>)))
    "Constants for message type '<control_mode_status>"
  '((:STATE_ERROR . 0)
    (:STATE_OFF . 1)
    (:STATE_IDLE . 2)
    (:STATE_STANDBY . 3)
    (:STATE_ACTIVE . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'control_mode_status)))
    "Constants for message type 'control_mode_status"
  '((:STATE_ERROR . 0)
    (:STATE_OFF . 1)
    (:STATE_IDLE . 2)
    (:STATE_STANDBY . 3)
    (:STATE_ACTIVE . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <control_mode_status>) ostream)
  "Serializes a message object of type '<control_mode_status>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ready) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'info))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'info))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <control_mode_status>) istream)
  "Deserializes a message object of type '<control_mode_status>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'ready) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'info) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'info) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<control_mode_status>)))
  "Returns string type for a message object of type '<control_mode_status>"
  "flyer_controller/control_mode_status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'control_mode_status)))
  "Returns string type for a message object of type 'control_mode_status"
  "flyer_controller/control_mode_status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<control_mode_status>)))
  "Returns md5sum for a message object of type '<control_mode_status>"
  "69f822cfeb8c98af2c7a1634de76aa9f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'control_mode_status)))
  "Returns md5sum for a message object of type 'control_mode_status"
  "69f822cfeb8c98af2c7a1634de76aa9f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<control_mode_status>)))
  "Returns full string definition for message of type '<control_mode_status>"
  (cl:format cl:nil "int8 STATE_ERROR=0~%int8 STATE_OFF=1~%int8 STATE_IDLE=2~%int8 STATE_STANDBY=3~%int8 STATE_ACTIVE=4~%~%Header header~%int8 state~%bool ready # ready to transition to ACTIVE?~%string info~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'control_mode_status)))
  "Returns full string definition for message of type 'control_mode_status"
  (cl:format cl:nil "int8 STATE_ERROR=0~%int8 STATE_OFF=1~%int8 STATE_IDLE=2~%int8 STATE_STANDBY=3~%int8 STATE_ACTIVE=4~%~%Header header~%int8 state~%bool ready # ready to transition to ACTIVE?~%string info~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <control_mode_status>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     4 (cl:length (cl:slot-value msg 'info))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <control_mode_status>))
  "Converts a ROS message object to a list"
  (cl:list 'control_mode_status
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
    (cl:cons ':ready (ready msg))
    (cl:cons ':info (info msg))
))
