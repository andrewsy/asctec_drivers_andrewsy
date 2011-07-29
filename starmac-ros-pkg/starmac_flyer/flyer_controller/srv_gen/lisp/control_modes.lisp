; Auto-generated. Do not edit!


(cl:in-package flyer_controller-srv)


;//! \htmlinclude control_modes-request.msg.html

(cl:defclass <control_modes-request> (roslisp-msg-protocol:ros-message)
  ((request
    :reader request
    :initarg :request
    :type cl:string
    :initform ""))
)

(cl:defclass control_modes-request (<control_modes-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <control_modes-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'control_modes-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name flyer_controller-srv:<control_modes-request> is deprecated: use flyer_controller-srv:control_modes-request instead.")))

(cl:ensure-generic-function 'request-val :lambda-list '(m))
(cl:defmethod request-val ((m <control_modes-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-srv:request-val is deprecated.  Use flyer_controller-srv:request instead.")
  (request m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <control_modes-request>) ostream)
  "Serializes a message object of type '<control_modes-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'request))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'request))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <control_modes-request>) istream)
  "Deserializes a message object of type '<control_modes-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'request) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'request) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<control_modes-request>)))
  "Returns string type for a service object of type '<control_modes-request>"
  "flyer_controller/control_modesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'control_modes-request)))
  "Returns string type for a service object of type 'control_modes-request"
  "flyer_controller/control_modesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<control_modes-request>)))
  "Returns md5sum for a message object of type '<control_modes-request>"
  "f9d56a6b456c1cfa47b023d486afffdb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'control_modes-request)))
  "Returns md5sum for a message object of type 'control_modes-request"
  "f9d56a6b456c1cfa47b023d486afffdb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<control_modes-request>)))
  "Returns full string definition for message of type '<control_modes-request>"
  (cl:format cl:nil "string request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'control_modes-request)))
  "Returns full string definition for message of type 'control_modes-request"
  (cl:format cl:nil "string request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <control_modes-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'request))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <control_modes-request>))
  "Converts a ROS message object to a list"
  (cl:list 'control_modes-request
    (cl:cons ':request (request msg))
))
;//! \htmlinclude control_modes-response.msg.html

(cl:defclass <control_modes-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:string
    :initform "")
   (reason
    :reader reason
    :initarg :reason
    :type cl:string
    :initform ""))
)

(cl:defclass control_modes-response (<control_modes-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <control_modes-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'control_modes-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name flyer_controller-srv:<control_modes-response> is deprecated: use flyer_controller-srv:control_modes-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <control_modes-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-srv:response-val is deprecated.  Use flyer_controller-srv:response instead.")
  (response m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <control_modes-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-srv:reason-val is deprecated.  Use flyer_controller-srv:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <control_modes-response>) ostream)
  "Serializes a message object of type '<control_modes-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <control_modes-response>) istream)
  "Deserializes a message object of type '<control_modes-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reason) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'reason) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<control_modes-response>)))
  "Returns string type for a service object of type '<control_modes-response>"
  "flyer_controller/control_modesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'control_modes-response)))
  "Returns string type for a service object of type 'control_modes-response"
  "flyer_controller/control_modesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<control_modes-response>)))
  "Returns md5sum for a message object of type '<control_modes-response>"
  "f9d56a6b456c1cfa47b023d486afffdb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'control_modes-response)))
  "Returns md5sum for a message object of type 'control_modes-response"
  "f9d56a6b456c1cfa47b023d486afffdb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<control_modes-response>)))
  "Returns full string definition for message of type '<control_modes-response>"
  (cl:format cl:nil "string response~%string reason~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'control_modes-response)))
  "Returns full string definition for message of type 'control_modes-response"
  (cl:format cl:nil "string response~%string reason~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <control_modes-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response))
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <control_modes-response>))
  "Converts a ROS message object to a list"
  (cl:list 'control_modes-response
    (cl:cons ':response (response msg))
    (cl:cons ':reason (reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'control_modes)))
  'control_modes-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'control_modes)))
  'control_modes-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'control_modes)))
  "Returns string type for a service object of type '<control_modes>"
  "flyer_controller/control_modes")
