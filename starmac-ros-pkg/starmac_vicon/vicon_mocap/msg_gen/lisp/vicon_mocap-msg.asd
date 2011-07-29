
(cl:in-package :asdf)

(defsystem "vicon_mocap-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Marker" :depends-on ("_package_Marker"))
    (:file "_package_Marker" :depends-on ("_package"))
    (:file "Markers" :depends-on ("_package_Markers"))
    (:file "_package_Markers" :depends-on ("_package"))
  ))