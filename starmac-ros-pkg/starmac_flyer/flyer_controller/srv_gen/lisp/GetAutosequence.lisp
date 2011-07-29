; Auto-generated. Do not edit!


(cl:in-package flyer_controller-srv)


;//! \htmlinclude GetAutosequence-request.msg.html

(cl:defclass <GetAutosequence-request> (roslisp-msg-protocol:ros-message)
  ((autosequence_name
    :reader autosequence_name
    :initarg :autosequence_name
    :type cl:string
    :initform ""))
)

(cl:defclass GetAutosequence-request (<GetAutosequence-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAutosequence-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAutosequence-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name flyer_controller-srv:<GetAutosequence-request> is deprecated: use flyer_controller-srv:GetAutosequence-request instead.")))

(cl:ensure-generic-function 'autosequence_name-val :lambda-list '(m))
(cl:defmethod autosequence_name-val ((m <GetAutosequence-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-srv:autosequence_name-val is deprecated.  Use flyer_controller-srv:autosequence_name instead.")
  (autosequence_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAutosequence-request>) ostream)
  "Serializes a message object of type '<GetAutosequence-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'autosequence_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'autosequence_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAutosequence-request>) istream)
  "Deserializes a message object of type '<GetAutosequence-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'autosequence_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'autosequence_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAutosequence-request>)))
  "Returns string type for a service object of type '<GetAutosequence-request>"
  "flyer_controller/GetAutosequenceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAutosequence-request)))
  "Returns string type for a service object of type 'GetAutosequence-request"
  "flyer_controller/GetAutosequenceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAutosequence-request>)))
  "Returns md5sum for a message object of type '<GetAutosequence-request>"
  "afeb270411c0d7737154215bd6840afe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAutosequence-request)))
  "Returns md5sum for a message object of type 'GetAutosequence-request"
  "afeb270411c0d7737154215bd6840afe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAutosequence-request>)))
  "Returns full string definition for message of type '<GetAutosequence-request>"
  (cl:format cl:nil "string autosequence_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAutosequence-request)))
  "Returns full string definition for message of type 'GetAutosequence-request"
  (cl:format cl:nil "string autosequence_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAutosequence-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'autosequence_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAutosequence-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAutosequence-request
    (cl:cons ':autosequence_name (autosequence_name msg))
))
;//! \htmlinclude GetAutosequence-response.msg.html

(cl:defclass <GetAutosequence-response> (roslisp-msg-protocol:ros-message)
  ((found
    :reader found
    :initarg :found
    :type cl:boolean
    :initform cl:nil)
   (autosequence
    :reader autosequence
    :initarg :autosequence
    :type flyer_controller-msg:Autosequence
    :initform (cl:make-instance 'flyer_controller-msg:Autosequence)))
)

(cl:defclass GetAutosequence-response (<GetAutosequence-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAutosequence-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAutosequence-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name flyer_controller-srv:<GetAutosequence-response> is deprecated: use flyer_controller-srv:GetAutosequence-response instead.")))

(cl:ensure-generic-function 'found-val :lambda-list '(m))
(cl:defmethod found-val ((m <GetAutosequence-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-srv:found-val is deprecated.  Use flyer_controller-srv:found instead.")
  (found m))

(cl:ensure-generic-function 'autosequence-val :lambda-list '(m))
(cl:defmethod autosequence-val ((m <GetAutosequence-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-srv:autosequence-val is deprecated.  Use flyer_controller-srv:autosequence instead.")
  (autosequence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAutosequence-response>) ostream)
  "Serializes a message object of type '<GetAutosequence-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'found) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'autosequence) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAutosequence-response>) istream)
  "Deserializes a message object of type '<GetAutosequence-response>"
    (cl:setf (cl:slot-value msg 'found) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'autosequence) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAutosequence-response>)))
  "Returns string type for a service object of type '<GetAutosequence-response>"
  "flyer_controller/GetAutosequenceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAutosequence-response)))
  "Returns string type for a service object of type 'GetAutosequence-response"
  "flyer_controller/GetAutosequenceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAutosequence-response>)))
  "Returns md5sum for a message object of type '<GetAutosequence-response>"
  "afeb270411c0d7737154215bd6840afe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAutosequence-response)))
  "Returns md5sum for a message object of type 'GetAutosequence-response"
  "afeb270411c0d7737154215bd6840afe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAutosequence-response>)))
  "Returns full string definition for message of type '<GetAutosequence-response>"
  (cl:format cl:nil "bool found~%flyer_controller/Autosequence autosequence~%~%================================================================================~%MSG: flyer_controller/Autosequence~%string name~%uint32 num_points~%flyer_controller/AutosequencePoint[] points~%================================================================================~%MSG: flyer_controller/AutosequencePoint~%flyer_controller/HoverPoint hover_point~%bool pause~%================================================================================~%MSG: flyer_controller/HoverPoint~%string name~%float64 x # [m] (North)~%float64 y # [m] (East)~%float64 alt # [m]~%float64 yaw # [deg]~%float64 vx # [m/s]~%float64 vy # [m/s]~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAutosequence-response)))
  "Returns full string definition for message of type 'GetAutosequence-response"
  (cl:format cl:nil "bool found~%flyer_controller/Autosequence autosequence~%~%================================================================================~%MSG: flyer_controller/Autosequence~%string name~%uint32 num_points~%flyer_controller/AutosequencePoint[] points~%================================================================================~%MSG: flyer_controller/AutosequencePoint~%flyer_controller/HoverPoint hover_point~%bool pause~%================================================================================~%MSG: flyer_controller/HoverPoint~%string name~%float64 x # [m] (North)~%float64 y # [m] (East)~%float64 alt # [m]~%float64 yaw # [deg]~%float64 vx # [m/s]~%float64 vy # [m/s]~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAutosequence-response>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'autosequence))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAutosequence-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAutosequence-response
    (cl:cons ':found (found msg))
    (cl:cons ':autosequence (autosequence msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAutosequence)))
  'GetAutosequence-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAutosequence)))
  'GetAutosequence-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAutosequence)))
  "Returns string type for a service object of type '<GetAutosequence>"
  "flyer_controller/GetAutosequence")
