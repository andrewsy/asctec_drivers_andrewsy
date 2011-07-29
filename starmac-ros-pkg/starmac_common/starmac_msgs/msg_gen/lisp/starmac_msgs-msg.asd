
(cl:in-package :asdf)

(defsystem "starmac_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "EulerAnglesStamped" :depends-on ("_package_EulerAnglesStamped"))
    (:file "_package_EulerAnglesStamped" :depends-on ("_package"))
    (:file "EulerAngles" :depends-on ("_package_EulerAngles"))
    (:file "_package_EulerAngles" :depends-on ("_package"))
  ))