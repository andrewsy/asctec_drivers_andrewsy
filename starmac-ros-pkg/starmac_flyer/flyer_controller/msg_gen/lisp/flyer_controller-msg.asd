
(cl:in-package :asdf)

(defsystem "flyer_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "control_mode_autosequence_info" :depends-on ("_package_control_mode_autosequence_info"))
    (:file "_package_control_mode_autosequence_info" :depends-on ("_package"))
    (:file "control_mode_output" :depends-on ("_package_control_mode_output"))
    (:file "_package_control_mode_output" :depends-on ("_package"))
    (:file "Autosequence" :depends-on ("_package_Autosequence"))
    (:file "_package_Autosequence" :depends-on ("_package"))
    (:file "control_mode_cmd" :depends-on ("_package_control_mode_cmd"))
    (:file "_package_control_mode_cmd" :depends-on ("_package"))
    (:file "controller_status" :depends-on ("_package_controller_status"))
    (:file "_package_controller_status" :depends-on ("_package"))
    (:file "control_mode_status" :depends-on ("_package_control_mode_status"))
    (:file "_package_control_mode_status" :depends-on ("_package"))
    (:file "controller_cmd" :depends-on ("_package_controller_cmd"))
    (:file "_package_controller_cmd" :depends-on ("_package"))
    (:file "control_mode_hover_info" :depends-on ("_package_control_mode_hover_info"))
    (:file "_package_control_mode_hover_info" :depends-on ("_package"))
    (:file "HoverPoint" :depends-on ("_package_HoverPoint"))
    (:file "_package_HoverPoint" :depends-on ("_package"))
    (:file "AutosequencePoint" :depends-on ("_package_AutosequencePoint"))
    (:file "_package_AutosequencePoint" :depends-on ("_package"))
  ))