; Auto-generated. Do not edit!


(cl:in-package flyer_controller-msg)


;//! \htmlinclude Autosequence.msg.html

(cl:defclass <Autosequence> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (num_points
    :reader num_points
    :initarg :num_points
    :type cl:integer
    :initform 0)
   (points
    :reader points
    :initarg :points
    :type (cl:vector flyer_controller-msg:AutosequencePoint)
   :initform (cl:make-array 0 :element-type 'flyer_controller-msg:AutosequencePoint :initial-element (cl:make-instance 'flyer_controller-msg:AutosequencePoint))))
)

(cl:defclass Autosequence (<Autosequence>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Autosequence>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Autosequence)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name flyer_controller-msg:<Autosequence> is deprecated: use flyer_controller-msg:Autosequence instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <Autosequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:name-val is deprecated.  Use flyer_controller-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'num_points-val :lambda-list '(m))
(cl:defmethod num_points-val ((m <Autosequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:num_points-val is deprecated.  Use flyer_controller-msg:num_points instead.")
  (num_points m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <Autosequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:points-val is deprecated.  Use flyer_controller-msg:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Autosequence>) ostream)
  "Serializes a message object of type '<Autosequence>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_points)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_points)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_points)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_points)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Autosequence>) istream)
  "Deserializes a message object of type '<Autosequence>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_points)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_points)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_points)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_points)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'flyer_controller-msg:AutosequencePoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Autosequence>)))
  "Returns string type for a message object of type '<Autosequence>"
  "flyer_controller/Autosequence")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Autosequence)))
  "Returns string type for a message object of type 'Autosequence"
  "flyer_controller/Autosequence")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Autosequence>)))
  "Returns md5sum for a message object of type '<Autosequence>"
  "2fc3d91e94190d44de5f18ae9fa72ffd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Autosequence)))
  "Returns md5sum for a message object of type 'Autosequence"
  "2fc3d91e94190d44de5f18ae9fa72ffd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Autosequence>)))
  "Returns full string definition for message of type '<Autosequence>"
  (cl:format cl:nil "string name~%uint32 num_points~%flyer_controller/AutosequencePoint[] points~%================================================================================~%MSG: flyer_controller/AutosequencePoint~%flyer_controller/HoverPoint hover_point~%bool pause~%================================================================================~%MSG: flyer_controller/HoverPoint~%string name~%float64 x # [m] (North)~%float64 y # [m] (East)~%float64 alt # [m]~%float64 yaw # [deg]~%float64 vx # [m/s]~%float64 vy # [m/s]~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Autosequence)))
  "Returns full string definition for message of type 'Autosequence"
  (cl:format cl:nil "string name~%uint32 num_points~%flyer_controller/AutosequencePoint[] points~%================================================================================~%MSG: flyer_controller/AutosequencePoint~%flyer_controller/HoverPoint hover_point~%bool pause~%================================================================================~%MSG: flyer_controller/HoverPoint~%string name~%float64 x # [m] (North)~%float64 y # [m] (East)~%float64 alt # [m]~%float64 yaw # [deg]~%float64 vx # [m/s]~%float64 vy # [m/s]~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Autosequence>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Autosequence>))
  "Converts a ROS message object to a list"
  (cl:list 'Autosequence
    (cl:cons ':name (name msg))
    (cl:cons ':num_points (num_points msg))
    (cl:cons ':points (points msg))
))
