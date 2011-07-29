; Auto-generated. Do not edit!


(cl:in-package flyer_controller-msg)


;//! \htmlinclude control_mode_autosequence_info.msg.html

(cl:defclass <control_mode_autosequence_info> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0)
   (current_autosequence
    :reader current_autosequence
    :initarg :current_autosequence
    :type cl:string
    :initform "")
   (current_point
    :reader current_point
    :initarg :current_point
    :type cl:integer
    :initform 0)
   (defined_autosequences
    :reader defined_autosequences
    :initarg :defined_autosequences
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass control_mode_autosequence_info (<control_mode_autosequence_info>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <control_mode_autosequence_info>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'control_mode_autosequence_info)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name flyer_controller-msg:<control_mode_autosequence_info> is deprecated: use flyer_controller-msg:control_mode_autosequence_info instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <control_mode_autosequence_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:header-val is deprecated.  Use flyer_controller-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <control_mode_autosequence_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:status-val is deprecated.  Use flyer_controller-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'current_autosequence-val :lambda-list '(m))
(cl:defmethod current_autosequence-val ((m <control_mode_autosequence_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:current_autosequence-val is deprecated.  Use flyer_controller-msg:current_autosequence instead.")
  (current_autosequence m))

(cl:ensure-generic-function 'current_point-val :lambda-list '(m))
(cl:defmethod current_point-val ((m <control_mode_autosequence_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:current_point-val is deprecated.  Use flyer_controller-msg:current_point instead.")
  (current_point m))

(cl:ensure-generic-function 'defined_autosequences-val :lambda-list '(m))
(cl:defmethod defined_autosequences-val ((m <control_mode_autosequence_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:defined_autosequences-val is deprecated.  Use flyer_controller-msg:defined_autosequences instead.")
  (defined_autosequences m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<control_mode_autosequence_info>)))
    "Constants for message type '<control_mode_autosequence_info>"
  '((:WAITING_PROCEED . 0)
    (:MOVING . 1)
    (:PAUSED . 2)
    (:IDLE . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'control_mode_autosequence_info)))
    "Constants for message type 'control_mode_autosequence_info"
  '((:WAITING_PROCEED . 0)
    (:MOVING . 1)
    (:PAUSED . 2)
    (:IDLE . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <control_mode_autosequence_info>) ostream)
  "Serializes a message object of type '<control_mode_autosequence_info>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'current_autosequence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'current_autosequence))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'current_point)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'current_point)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'current_point)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'current_point)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'defined_autosequences))))
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
   (cl:slot-value msg 'defined_autosequences))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <control_mode_autosequence_info>) istream)
  "Deserializes a message object of type '<control_mode_autosequence_info>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_autosequence) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'current_autosequence) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'current_point)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'current_point)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'current_point)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'current_point)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'defined_autosequences) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'defined_autosequences)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<control_mode_autosequence_info>)))
  "Returns string type for a message object of type '<control_mode_autosequence_info>"
  "flyer_controller/control_mode_autosequence_info")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'control_mode_autosequence_info)))
  "Returns string type for a message object of type 'control_mode_autosequence_info"
  "flyer_controller/control_mode_autosequence_info")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<control_mode_autosequence_info>)))
  "Returns md5sum for a message object of type '<control_mode_autosequence_info>"
  "23f909b9602b395bbe12d106602f3d6d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'control_mode_autosequence_info)))
  "Returns md5sum for a message object of type 'control_mode_autosequence_info"
  "23f909b9602b395bbe12d106602f3d6d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<control_mode_autosequence_info>)))
  "Returns full string definition for message of type '<control_mode_autosequence_info>"
  (cl:format cl:nil "int8 WAITING_PROCEED = 0~%int8 MOVING = 1~%int8 PAUSED = 2~%int8 IDLE = 3~%~%Header header~%int8 status~%string current_autosequence~%uint32 current_point~%string[] defined_autosequences~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'control_mode_autosequence_info)))
  "Returns full string definition for message of type 'control_mode_autosequence_info"
  (cl:format cl:nil "int8 WAITING_PROCEED = 0~%int8 MOVING = 1~%int8 PAUSED = 2~%int8 IDLE = 3~%~%Header header~%int8 status~%string current_autosequence~%uint32 current_point~%string[] defined_autosequences~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <control_mode_autosequence_info>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:length (cl:slot-value msg 'current_autosequence))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'defined_autosequences) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <control_mode_autosequence_info>))
  "Converts a ROS message object to a list"
  (cl:list 'control_mode_autosequence_info
    (cl:cons ':header (header msg))
    (cl:cons ':status (status msg))
    (cl:cons ':current_autosequence (current_autosequence msg))
    (cl:cons ':current_point (current_point msg))
    (cl:cons ':defined_autosequences (defined_autosequences msg))
))
