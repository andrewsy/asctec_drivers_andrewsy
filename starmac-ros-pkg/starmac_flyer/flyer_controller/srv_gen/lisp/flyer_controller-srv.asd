
(cl:in-package :asdf)

(defsystem "flyer_controller-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :flyer_controller-msg
)
  :components ((:file "_package")
    (:file "GetAutosequence" :depends-on ("_package_GetAutosequence"))
    (:file "_package_GetAutosequence" :depends-on ("_package"))
    (:file "control_modes" :depends-on ("_package_control_modes"))
    (:file "_package_control_modes" :depends-on ("_package"))
  ))