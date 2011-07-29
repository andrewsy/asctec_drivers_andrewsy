; Auto-generated. Do not edit!


(cl:in-package flyer_controller-msg)


;//! \htmlinclude AutosequencePoint.msg.html

(cl:defclass <AutosequencePoint> (roslisp-msg-protocol:ros-message)
  ((hover_point
    :reader hover_point
    :initarg :hover_point
    :type flyer_controller-msg:HoverPoint
    :initform (cl:make-instance 'flyer_controller-msg:HoverPoint))
   (pause
    :reader pause
    :initarg :pause
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass AutosequencePoint (<AutosequencePoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AutosequencePoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AutosequencePoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name flyer_controller-msg:<AutosequencePoint> is deprecated: use flyer_controller-msg:AutosequencePoint instead.")))

(cl:ensure-generic-function 'hover_point-val :lambda-list '(m))
(cl:defmethod hover_point-val ((m <AutosequencePoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:hover_point-val is deprecated.  Use flyer_controller-msg:hover_point instead.")
  (hover_point m))

(cl:ensure-generic-function 'pause-val :lambda-list '(m))
(cl:defmethod pause-val ((m <AutosequencePoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flyer_controller-msg:pause-val is deprecated.  Use flyer_controller-msg:pause instead.")
  (pause m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AutosequencePoint>) ostream)
  "Serializes a message object of type '<AutosequencePoint>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'hover_point) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'pause) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AutosequencePoint>) istream)
  "Deserializes a message object of type '<AutosequencePoint>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'hover_point) istream)
    (cl:setf (cl:slot-value msg 'pause) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AutosequencePoint>)))
  "Returns string type for a message object of type '<AutosequencePoint>"
  "flyer_controller/AutosequencePoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AutosequencePoint)))
  "Returns string type for a message object of type 'AutosequencePoint"
  "flyer_controller/AutosequencePoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AutosequencePoint>)))
  "Returns md5sum for a message object of type '<AutosequencePoint>"
  "c397fd5685c6ff6ca05d78fa239709a5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AutosequencePoint)))
  "Returns md5sum for a message object of type 'AutosequencePoint"
  "c397fd5685c6ff6ca05d78fa239709a5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AutosequencePoint>)))
  "Returns full string definition for message of type '<AutosequencePoint>"
  (cl:format cl:nil "flyer_controller/HoverPoint hover_point~%bool pause~%================================================================================~%MSG: flyer_controller/HoverPoint~%string name~%float64 x # [m] (North)~%float64 y # [m] (East)~%float64 alt # [m]~%float64 yaw # [deg]~%float64 vx # [m/s]~%float64 vy # [m/s]~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AutosequencePoint)))
  "Returns full string definition for message of type 'AutosequencePoint"
  (cl:format cl:nil "flyer_controller/HoverPoint hover_point~%bool pause~%================================================================================~%MSG: flyer_controller/HoverPoint~%string name~%float64 x # [m] (North)~%float64 y # [m] (East)~%float64 alt # [m]~%float64 yaw # [deg]~%float64 vx # [m/s]~%float64 vy # [m/s]~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AutosequencePoint>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'hover_point))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AutosequencePoint>))
  "Converts a ROS message object to a list"
  (cl:list 'AutosequencePoint
    (cl:cons ':hover_point (hover_point msg))
    (cl:cons ':pause (pause msg))
))
