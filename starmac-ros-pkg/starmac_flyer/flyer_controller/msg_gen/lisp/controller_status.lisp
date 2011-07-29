; Auto-generated. Do not edit!


(cl:in-package flyer_controller-msg)


;//! \htmlinclude controller_status.msg.html

(cl:defclass <controller_status> (roslisp-msg-protocol:ros-message)
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
   (info
    :reader info
    :initarg :info
    :type cl:string
    :initform "")
   (active_mode
    :reader active_mode
    :initarg :active_mode
    :type cl:string
    :initform "")
   (standby_modes
    :reader standby_modes
    :initarg :standby_modes
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass controller_status (<controller_status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <controller_status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'controller_status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name flyer_controller-msg:<controller_status> is deprecated: use flyer_controller-msg:controller_status instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <controller_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:header-val is deprecated.  Use flyer_controller-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <controller_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:state-val is deprecated.  Use flyer_controller-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'info-val :lambda-list '(m))
(cl:defmethod info-val ((m <controller_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:info-val is deprecated.  Use flyer_controller-msg:info instead.")
  (info m))

(cl:ensure-generic-function 'active_mode-val :lambda-list '(m))
(cl:defmethod active_mode-val ((m <controller_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:active_mode-val is deprecated.  Use flyer_controller-msg:active_mode instead.")
  (active_mode m))

(cl:ensure-generic-function 'standby_modes-val :lambda-list '(m))
(cl:defmethod standby_modes-val ((m <controller_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:standby_modes-val is deprecated.  Use flyer_controller-msg:standby_modes instead.")
  (standby_modes m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<controller_status>)))
    "Constants for message type '<controller_status>"
  '((:STATE_ERROR . 0)
    (:STATE_OFF . 1)
    (:STATE_INITIALIZING . 2)
    (:STATE_STANDBY . 3)
    (:STATE_OPERATIONAL . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'controller_status)))
    "Constants for message type 'controller_status"
  '((:STATE_ERROR . 0)
    (:STATE_OFF . 1)
    (:STATE_INITIALIZING . 2)
    (:STATE_STANDBY . 3)
    (:STATE_OPERATIONAL . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <controller_status>) ostream)
  "Serializes a message object of type '<controller_status>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'info))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'info))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'active_mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'active_mode))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'standby_modes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'standby_modes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <controller_status>) istream)
  "Deserializes a message object of type '<controller_status>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'info) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'info) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'active_mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'active_mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'standby_modes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'standby_modes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<controller_status>)))
  "Returns string type for a message object of type '<controller_status>"
  "flyer_controller/controller_status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'controller_status)))
  "Returns string type for a message object of type 'controller_status"
  "flyer_controller/controller_status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<controller_status>)))
  "Returns md5sum for a message object of type '<controller_status>"
  "c2d88358f45aae41a821a82436eef1ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'controller_status)))
  "Returns md5sum for a message object of type 'controller_status"
  "c2d88358f45aae41a821a82436eef1ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<controller_status>)))
  "Returns full string definition for message of type '<controller_status>"
  (cl:format cl:nil "int8 STATE_ERROR=0~%int8 STATE_OFF=1~%int8 STATE_INITIALIZING=2~%int8 STATE_STANDBY=3~%int8 STATE_OPERATIONAL=4~%~%Header header~%int8 state~%string info~%string active_mode~%string[] standby_modes~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'controller_status)))
  "Returns full string definition for message of type 'controller_status"
  (cl:format cl:nil "int8 STATE_ERROR=0~%int8 STATE_OFF=1~%int8 STATE_INITIALIZING=2~%int8 STATE_STANDBY=3~%int8 STATE_OPERATIONAL=4~%~%Header header~%int8 state~%string info~%string active_mode~%string[] standby_modes~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <controller_status>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:length (cl:slot-value msg 'info))
     4 (cl:length (cl:slot-value msg 'active_mode))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'standby_modes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <controller_status>))
  "Converts a ROS message object to a list"
  (cl:list 'controller_status
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
    (cl:cons ':info (info msg))
    (cl:cons ':active_mode (active_mode msg))
    (cl:cons ':standby_modes (standby_modes msg))
))
