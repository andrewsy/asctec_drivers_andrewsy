
(cl:in-package :asdf)

(defsystem "vicon_mocap-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "viconGrabPose" :depends-on ("_package_viconGrabPose"))
    (:file "_package_viconGrabPose" :depends-on ("_package"))
  ))