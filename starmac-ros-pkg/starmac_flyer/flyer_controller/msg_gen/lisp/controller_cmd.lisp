; Auto-generated. Do not edit!


(cl:in-package flyer_controller-msg)


;//! \htmlinclude controller_cmd.msg.html

(cl:defclass <controller_cmd> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:string
    :initform ""))
)

(cl:defclass controller_cmd (<controller_cmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <controller_cmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'controller_cmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name flyer_controller-msg:<controller_cmd> is deprecated: use flyer_controller-msg:controller_cmd instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <controller_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:cmd-val is deprecated.  Use flyer_controller-msg:cmd instead.")
  (cmd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <controller_cmd>) ostream)
  "Serializes a message object of type '<controller_cmd>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cmd))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <controller_cmd>) istream)
  "Deserializes a message object of type '<controller_cmd>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cmd) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<controller_cmd>)))
  "Returns string type for a message object of type '<controller_cmd>"
  "flyer_controller/controller_cmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'controller_cmd)))
  "Returns string type for a message object of type 'controller_cmd"
  "flyer_controller/controller_cmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<controller_cmd>)))
  "Returns md5sum for a message object of type '<controller_cmd>"
  "43a54fa49066cddcf148717d9d4a6353")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'controller_cmd)))
  "Returns md5sum for a message object of type 'controller_cmd"
  "43a54fa49066cddcf148717d9d4a6353")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<controller_cmd>)))
  "Returns full string definition for message of type '<controller_cmd>"
  (cl:format cl:nil "string cmd~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'controller_cmd)))
  "Returns full string definition for message of type 'controller_cmd"
  (cl:format cl:nil "string cmd~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <controller_cmd>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'cmd))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <controller_cmd>))
  "Converts a ROS message object to a list"
  (cl:list 'controller_cmd
    (cl:cons ':cmd (cmd msg))
))
